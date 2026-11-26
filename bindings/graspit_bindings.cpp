#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <QApplication>
#include <QCoreApplication>
#include <QTimer>
#include <QFile>
#include <QString>
#include <QPixmap>
#include <QtGlobal>

#include <dlfcn.h>

#include <filesystem>
#include <chrono>
#include <thread>
#include <cstdio>
#include <array>
#include <cmath>
#include <optional>
#include <sstream>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/sensors/SoAlarmSensor.h>

// GraspIt GUI header lives in the GraspIt source tree.
#include "mainWindow.h"

// GraspIt! includes
#include "graspit/graspitCore.h"
#include "graspit/grasp.h"
#include "graspit/world.h"
#include "graspit/robot.h"
#include "graspit/kinematicChain.h"
#include "graspit/body.h"
#include "graspit/contact/contact.h"
#include "graspit/ivmgr.h"
#include "graspit/quality/qualEpsilon.h"
#include "graspit/quality/qualVolume.h"
#include "graspit/transform.h"
#include "graspit/math/matrix.h"
#include "tinyxml.h"

namespace py = pybind11;
using namespace pybind11::literals;

static bool guiAvailable();

// Qt prints (and may abort) if any exception propagates through an event handler.
// When embedding GraspIt as a library via Python, we defensively catch everything.
class SafeQApplication final : public QApplication {
public:
    SafeQApplication(int& argc, char** argv) : QApplication(argc, argv) {}

    bool notify(QObject* receiver, QEvent* event) override {
        try {
            return QApplication::notify(receiver, event);
        } catch (const std::bad_alloc& e) {
            std::fprintf(stderr, "Qt event handler threw std::bad_alloc: %s\n", e.what());
            return false;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "Qt event handler threw std::exception: %s\n", e.what());
            return false;
        } catch (...) {
            std::fprintf(stderr, "Qt event handler threw unknown exception\n");
            return false;
        }
    }
};

// Helper functions to convert between numpy arrays and GraspIt! vectors
py::array_t<double> vec3_to_numpy(const vec3& vec) {
    py::array_t<double> out(3);
    auto r = out.mutable_unchecked<1>();
    r(0) = vec.x();
    r(1) = vec.y();
    r(2) = vec.z();
    return out;
}

vec3 numpy_to_vec3(py::array_t<double> arr) {
    py::buffer_info buf = arr.request();
    if (buf.size != 3) {
        throw std::runtime_error("Array must have exactly 3 elements");
    }
    double* ptr = static_cast<double*>(buf.ptr);
    return vec3(ptr[0], ptr[1], ptr[2]);
}

static py::dict transf_to_py(const transf& tr) {
    py::dict out;
    const Quaternion& q = tr.rotation();
    const vec3& t = tr.translation();

    out["translation"] = py::make_tuple(t.x(), t.y(), t.z());
    out["quat_wxyz"] = py::make_tuple(q.w(), q.x(), q.y(), q.z());

    // GraspIt's matrix exporter transposes the rotation. Match the active
    // quaternion action used by transf * position, with translation in mm.
    const mat3 R = q.toRotationMatrix();
    double m[4][4] = {
        {R(0, 0), R(0, 1), R(0, 2), t.x()},
        {R(1, 0), R(1, 1), R(1, 2), t.y()},
        {R(2, 0), R(2, 1), R(2, 2), t.z()},
        {0.0, 0.0, 0.0, 1.0}
    };
    py::list rows;
    for (int i = 0; i < 4; ++i) {
        py::list row;
        for (int j = 0; j < 4; ++j) {
            row.append(m[i][j]);
        }
        rows.append(row);
    }
    out["matrix_row_major"] = rows;

    std::stringstream ss;
    ss << tr;
    out["full_transform"] = ss.str();

    return out;
}

// Wrapper classes for Python
class PythonWorld {
public:
    World* world;
    bool owns_world;


    PythonWorld() : world(nullptr), owns_world(false) {}

    void init() {
        if (world) {
            return;
        }
        // Prefer the GraspIt core "mainWorld" so the GUI reflects changes.
        if (graspitCore && graspitCore->getWorld()) {
            world = graspitCore->getWorld();
            owns_world = false;
            return;
        }

        // Fallback: standalone World (no GUI integration).
        world = new World();
        owns_world = true;
    }

    // Parse a GraspIt! <fullTransform> string of the form:
    //   "(qw qx qy qz)[tx ty tz]"
    // into a Quaternion and translation vec3. Returns true on success.
    static bool parseFullTransform(const std::string& s, Quaternion& q, vec3& t) {
        std::size_t lpar = s.find('(');
        std::size_t rpar = s.find(')');
        std::size_t lbrk = s.find('[');
        std::size_t rbrk = s.find(']');
        if (lpar == std::string::npos || rpar == std::string::npos ||
            lbrk == std::string::npos || rbrk == std::string::npos ||
            rpar <= lpar || rbrk <= lbrk) {
            return false;
        }

        std::string quatStr = s.substr(lpar + 1, rpar - lpar - 1);
        std::string transStr = s.substr(lbrk + 1, rbrk - lbrk - 1);

        std::stringstream qss(quatStr);
        std::stringstream tss(transStr);

        double qw, qx, qy, qz;
        double tx, ty, tz;
        if (!(qss >> qw >> qx >> qy >> qz)) {
            return false;
        }
        if (!(tss >> tx >> ty >> tz)) {
            return false;
        }

        q = Quaternion(qw, qx, qy, qz);
        t = vec3(tx, ty, tz);
        return true;
    }

    ~PythonWorld() {
        if (world && owns_world) {
            delete world;
        }
    }

    bool loadRobot(const std::string& filename) {
        if (!world) throw std::runtime_error("World not initialized");
        const QString qfile = QString::fromStdString(filename);
        if (qfile.endsWith(".urdf", Qt::CaseInsensitive)) {
            TiXmlDocument doc(qfile.toLatin1().constData());
            if (!doc.LoadFile()) {
                return false;
            }
            const TiXmlElement* root = doc.RootElement();
            if (!root) {
                return false;
            }
            return world->importRobotFromURDF(qfile, root) != nullptr;
        }
        return world->importRobot(qfile) != nullptr;
    }

    bool loadBody(const std::string& filename) {
        if (!world) throw std::runtime_error("World not initialized");
        return world->importBody("Body", QString::fromStdString(filename)) != nullptr;
    }

    bool loadGraspableBody(const std::string& filename) {
        if (!world) throw std::runtime_error("World not initialized");
        return world->importBody("GraspableBody", QString::fromStdString(filename)) != nullptr;
    }

    // Load a GraspIt! world from an XML file.
    // by reading its <robot> and <graspableBody> elements and reproducing
    // their filenames, base transforms, and DOF values.
    bool loadWorldFromXml(const std::string& filename) {
        if (!world) throw std::runtime_error("World not initialized");

        TiXmlDocument doc(filename.c_str());
        if (!doc.LoadFile()) {
            return false;
        }
        const TiXmlElement* root = doc.RootElement();
        if (!root) {
            return false;
        }

        // Resolve relative paths against GRASPIT root if available.
        const char* graspitRootC = std::getenv("GRASPIT");
        std::string graspitRoot = graspitRootC ? std::string(graspitRootC) : std::string();
        auto resolvePath = [&](const std::string& rel) -> std::string {
            if (rel.empty()) return rel;
            if (!graspitRoot.empty() && rel[0] != '/' && rel.find(':') == std::string::npos) {
                return graspitRoot + "/" + rel;
            }
            return rel;
        };

        bool loadedRobot = false;
        bool loadedObject = false;

        // IMPORTANT: Load robots / hands first.
        // World::addRobot() performs an "in collision by default" pass and
        // permanently disables collisions for any colliding pairs present at the
        // time the robot is added. If we import the graspable body first (often
        // at its default pose), GraspIt may disable robot-object collisions and
        // later grasp quality computation will report zero contacts.
        for (const TiXmlElement* rb = root->FirstChildElement("robot");
             rb != nullptr;
             rb = rb->NextSiblingElement("robot")) {
            const TiXmlElement* fnEl = rb->FirstChildElement("filename");
            if (!fnEl || !fnEl->GetText()) {
                continue;
            }
            std::string robotPath = resolvePath(fnEl->GetText());
            if (!loadRobot(robotPath)) {
                continue;
            }

            Hand* hand = world->getCurrentHand();
            if (hand) {
                // Optional DOF values
                const TiXmlElement* dofEl = rb->FirstChildElement("dofValues");
                if (dofEl && dofEl->GetText()) {
                    std::stringstream dss(dofEl->GetText());
                    std::vector<double> dofs;
                    double v;
                    while (dss >> v) {
                        dofs.push_back(v);
                    }
                    if (!dofs.empty()) {
                        setCurrentHandDOFValues(dofs);
                    }
                }

                // Optional base transform
                const TiXmlElement* tfEl = rb->FirstChildElement("transform");
                if (tfEl) {
                    const TiXmlElement* fullEl = tfEl->FirstChildElement("fullTransform");
                    if (fullEl && fullEl->GetText()) {
                        Quaternion q;
                        vec3 t;
                        if (parseFullTransform(fullEl->GetText(), q, t)) {
                            hand->setTran(transf(q, t));
                        }
                    }
                }
            }
            loadedRobot = true;
        }

        // Load graspable bodies
        for (const TiXmlElement* gb = root->FirstChildElement("graspableBody");
             gb != nullptr;
             gb = gb->NextSiblingElement("graspableBody")) {
            const TiXmlElement* fnEl = gb->FirstChildElement("filename");
            if (!fnEl || !fnEl->GetText()) {
                continue;
            }
            std::string objPath = resolvePath(fnEl->GetText());
            if (!loadGraspableBody(objPath)) {
                continue;
            }
            int gbIndex = world->getNumGB() - 1;

            // Optional transform
            const TiXmlElement* tfEl = gb->FirstChildElement("transform");
            if (tfEl) {
                const TiXmlElement* fullEl = tfEl->FirstChildElement("fullTransform");
                if (fullEl && fullEl->GetText()) {
                    Quaternion q;
                    vec3 t;
                    if (parseFullTransform(fullEl->GetText(), q, t)) {
                        GraspableBody* object = world->getGB(gbIndex);
                        if (object) {
                            object->setTran(transf(q, t));
                        }
                    }
                }
            }
            loadedObject = true;
        }

        return loadedRobot && loadedObject;
    }

    void setGraspableBodyTranslation(int graspableBodyIndex, double x, double y, double z) {
        if (!world) throw std::runtime_error("World not initialized");
        if (world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }

        GraspableBody* object = world->getGB(graspableBodyIndex);
        if (!object) {
            throw std::runtime_error("Failed to get graspable body");
        }
        const transf& cur = object->getTran();
        transf next(cur.rotation(), vec3(x, y, z));
        object->setTran(next);
    }

    void setGraspableBodyDynamic(int graspableBodyIndex, bool dynamic) {
        if (!world) throw std::runtime_error("World not initialized");
        if (world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }
        GraspableBody* object = world->getGB(graspableBodyIndex);
        if (!object) {
            throw std::runtime_error("Failed to get graspable body");
        }
        object->setUseDynamics(dynamic);
    }

    std::vector<double> getGraspableBodyTranslation(int graspableBodyIndex) const {
        if (!world) throw std::runtime_error("World not initialized");
        if (world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }

        GraspableBody* object = world->getGB(graspableBodyIndex);
        if (!object) {
            throw std::runtime_error("Failed to get graspable body");
        }
        const vec3& t = object->getTran().translation();
        return {t.x(), t.y(), t.z()};
    }

    py::dict getGraspableBodyPose(int graspableBodyIndex) const {
        if (!world) throw std::runtime_error("World not initialized");
        if (world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }

        GraspableBody* object = world->getGB(graspableBodyIndex);
        if (!object) {
            throw std::runtime_error("Failed to get graspable body");
        }

        return transf_to_py(object->getTran());
    }

    std::vector<double> getGraspableBodyCoGWorld(int graspableBodyIndex) const {
        if (!world) throw std::runtime_error("World not initialized");
        if (world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }

        GraspableBody* object = world->getGB(graspableBodyIndex);
        if (!object) {
            throw std::runtime_error("Failed to get graspable body");
        }

        const position& cog_local = object->getCoG();
        const vec3 cog_world = object->getTran() * cog_local;
        return {cog_world.x(), cog_world.y(), cog_world.z()};
    }

    std::vector<double> getGraspableBodyCoGLocal(int graspableBodyIndex) const {
        if (!world) throw std::runtime_error("World not initialized");
        if (world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }

        GraspableBody* object = world->getGB(graspableBodyIndex);
        if (!object) {
            throw std::runtime_error("Failed to get graspable body");
        }

        const position& cog = object->getCoG();
        return {cog.x(), cog.y(), cog.z()};
    }

    py::dict getCurrentHandPose() const {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        return transf_to_py(hand->getTran());
    }

    void setCurrentHandPose(const std::array<double, 4>& quatWxyz,
                            const std::array<double, 3>& translationMm) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        Quaternion q(quatWxyz[0], quatWxyz[1], quatWxyz[2], quatWxyz[3]);
        q.normalize();
        hand->setTran(transf(q, vec3(translationMm[0], translationMm[1], translationMm[2])));
    }

    void clear() {
        if (!world) throw std::runtime_error("World not initialized");
        while (world->getNumRobots() > 0) {
            world->destroyElement(world->getRobot(world->getNumRobots() - 1));
        }
        for (int b = world->getNumBodies() - 1; b >= 0; --b) {
            if (!world->getBody(b)->inherits("Link")) {
                world->destroyElement(world->getBody(b));
            }
        }
    }

    bool noCollision() {
        if (!world) throw std::runtime_error("World not initialized");
        return world->noCollision();
    }

    Link* handLink(int chain, int link) const {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        if (chain < 0) return hand->getBase();
        if (chain >= hand->getNumChains()) throw std::runtime_error("Invalid chain index");
        KinematicChain* kc = hand->getChain(chain);
        if (link < 0 || link >= kc->getNumLinks()) throw std::runtime_error("Invalid link index");
        return kc->getLink(link);
    }

    py::list getLinkPoses() const {
        Hand* hand = world ? world->getCurrentHand() : nullptr;
        if (!hand) throw std::runtime_error("World has no current hand (load a robot/hand first)");
        py::list out;
        for (int c = -1; c < hand->getNumChains(); ++c) {
            int n = c < 0 ? 1 : hand->getChain(c)->getNumLinks();
            for (int l = 0; l < n; ++l) {
                Link* body = handLink(c, c < 0 ? 0 : l);
                py::dict rec = transf_to_py(body->getTran());
                rec["chain"] = c;
                rec["link"] = c < 0 ? -1 : l;
                rec["name"] = body->getName().toStdString();
                out.append(rec);
            }
        }
        return out;
    }

    py::array_t<double> getLinkVerticesWorld(int chain, int link) const {
        Link* body = handLink(chain, link);
        std::vector<position> vertices;
        body->getGeometryVertices(&vertices);
        const transf& tr = body->getTran();
        py::array_t<double> out({static_cast<py::ssize_t>(vertices.size()), static_cast<py::ssize_t>(3)});
        auto view = out.mutable_unchecked<2>();
        for (size_t i = 0; i < vertices.size(); ++i) {
            vec3 w = tr * vertices[i];
            view(i, 0) = w.x(); view(i, 1) = w.y(); view(i, 2) = w.z();
        }
        return out;
    }

    py::dict getGraspableBodyPoseRelativeToHand(int graspableBodyIndex) const {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        if (world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }

        GraspableBody* object = world->getGB(graspableBodyIndex);
        if (!object) {
            throw std::runtime_error("Failed to get graspable body");
        }

        transf rel = hand->getTran().inverse() % object->getTran();
        return transf_to_py(rel);
    }

    void placeGraspableBodyRelativeToHand(int graspableBodyIndex, double x, double y, double z) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        if (world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }

        GraspableBody* object = world->getGB(graspableBodyIndex);
        if (!object) {
            throw std::runtime_error("Failed to get graspable body");
        }

        // Compose: hand pose in world * translation in hand frame.
        transf target = hand->getTran() % transf::TRANSLATION(vec3(x, y, z));
        object->setTran(target);
    }

    // Add translation method to support MPC 'force'/'approach' actions by moving the robot
    void translateCurrentRobot(double dx, double dy, double dz) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand");
        }

        // Translate robot in WORLD coordinates
        transf cur = hand->getTran();
        transf shift = transf::TRANSLATION(vec3(dx, dy, dz));
        hand->setTran(cur % shift);
    }

    bool autoGrasp(bool renderIt = false, double speedFactor = 1.0, bool stopAtContact = false) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        return hand->autoGrasp(renderIt, speedFactor, stopAtContact);
    }

    bool quickOpen(double speedFactor = 0.1) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        return hand->quickOpen(speedFactor);
    }

    bool approachToContact(double moveDist, bool oneStep = true) {
        if (!world) throw std::runtime_error("World not initialized");
        // NOTE: approachToContact has been observed to segfault when GraspIt is embedded
        // and initialized headlessly (e.g., QT_QPA_PLATFORM=offscreen). Avoid calling it.
        const QString platform = qEnvironmentVariable("QT_QPA_PLATFORM");
        const bool headless_platform = (platform == "offscreen" || platform == "minimal");
        const bool no_display = !qEnvironmentVariableIsSet("DISPLAY");
        if (headless_platform || no_display) {
            throw std::runtime_error(
                "approach_to_contact is disabled in headless/offscreen mode (can segfault). "
                "Run with --gui (and a valid DISPLAY) to use it."
            );
        }
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        return hand->approachToContact(moveDist, oneStep);
    }

    int getNumRobots() const {
        if (!world) return 0;
        return world->getNumRobots();
    }

    int getNumBodies() const {
        if (!world) return 0;
        return world->getNumBodies();
    }

    int getNumGB() const {
        if (!world) return 0;
        return world->getNumGB();
    }

    // ========== Display Toggle Methods ==========

    void showFrictionCones(bool on) {
        if (!world) return;
        // Apply to all graspable bodies
        for (int i = 0; i < world->getNumGB(); ++i) {
            GraspableBody* gb = world->getGB(i);
            if (gb) gb->showFrictionCones(on);
        }
        // Apply to all bodies in the world (including robot links)
        for (int i = 0; i < world->getNumBodies(); ++i) {
            Body* b = world->getBody(i);
            if (b) b->showFrictionCones(on);
        }
    }

    void showAxes(bool on) {
        if (!world) return;
        // Apply to all graspable bodies (GraspableBody inherits from DynamicBody which has showAxes)
        for (int i = 0; i < world->getNumGB(); ++i) {
            GraspableBody* gb = world->getGB(i);
            if (gb) gb->showAxes(on);
        }
    }

    void setCleanDisplay(bool clean) {
        // Convenience method: if clean==true, hide all overlays
        showFrictionCones(!clean);
        showAxes(!clean);
    }

    int getCurrentHandNumDOF() const {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }
        return hand->getNumDOF();
    }

    std::vector<double> getDOFDefaultVelocities() const {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) throw std::runtime_error("World has no current hand (load a robot/hand first)");
        std::vector<double> v(hand->getNumDOF());
        for (int i = 0; i < hand->getNumDOF(); ++i) v[i] = hand->getDOF(i)->getDefaultVelocity();
        return v;
    }

    void setDOFDefaultVelocities(const std::vector<double>& v) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) throw std::runtime_error("World has no current hand (load a robot/hand first)");
        if (v.size() != static_cast<size_t>(hand->getNumDOF())) {
            throw std::runtime_error("Incorrect number of DOF velocities. Expected " + std::to_string(hand->getNumDOF()));
        }
        for (int i = 0; i < hand->getNumDOF(); ++i) hand->getDOF(i)->setDefaultVelocity(v[i]);
    }

    std::vector<double> getCurrentHandDOFValues() const {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }

        int numDOF = hand->getNumDOF();
        double* dofVals = new double[numDOF];
        hand->getDOFVals(dofVals);

        std::vector<double> result(dofVals, dofVals + numDOF);
        delete[] dofVals;
        return result;
    }

    void setCurrentHandDOFValues(const std::vector<double>& dofValues) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }

        int numDOF = hand->getNumDOF();
        if (dofValues.size() != static_cast<size_t>(numDOF)) {
            throw std::runtime_error("Incorrect number of DOF values. Expected " + std::to_string(numDOF));
        }

        // Copy to raw array for GraspIt API
        double* vals = new double[numDOF];
        for (int i = 0; i < numDOF; ++i) {
            vals[i] = dofValues[i];
        }
        hand->checkDOFVals(vals);
        // Use forceDOFVals to set posture immediately without collision check (teleport)
        static_cast<Robot*>(hand)->forceDOFVals(vals);

        delete[] vals;
    }

    bool moveCurrentHandDOFToContacts(
        const std::vector<double>& desiredDOFValues,
        const std::vector<double>& desiredStepSizes,
        bool stopAtContact = false,
        bool renderIt = false
    ) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }

        const int numDOF = hand->getNumDOF();
        if (desiredDOFValues.size() != static_cast<size_t>(numDOF)) {
            throw std::runtime_error("Incorrect number of DOF values. Expected " + std::to_string(numDOF));
        }

        std::vector<double> desiredVals(desiredDOFValues.begin(), desiredDOFValues.end());
        double* stepPtr = nullptr;
        std::vector<double> steps;
        if (!desiredStepSizes.empty()) {
            if (desiredStepSizes.size() != static_cast<size_t>(numDOF)) {
                throw std::runtime_error("Incorrect number of step sizes. Expected " + std::to_string(numDOF));
            }
            steps.assign(desiredStepSizes.begin(), desiredStepSizes.end());
            stepPtr = steps.data();
        }

        // GraspIt will internally take small steps and do collision detection.
        return static_cast<Robot*>(hand)->moveDOFToContacts(desiredVals.data(), stepPtr, stopAtContact, renderIt);
    }

    py::tuple traceMoveCurrentHandDOFToContacts(
        const std::vector<double>& desiredDOFValues,
        const std::vector<double>& desiredStepSizes,
        bool stopAtContact = false,
        bool renderIt = false
    ) {
        if (!world) throw std::runtime_error("World not initialized");
        Hand* hand = world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }

        Robot* robot = static_cast<Robot*>(hand);
        const int numDOF = robot->getNumDOF();
        if (desiredDOFValues.size() != static_cast<size_t>(numDOF)) {
            throw std::runtime_error("Incorrect number of DOF values. Expected " + std::to_string(numDOF));
        }

        std::vector<double> desiredVals(desiredDOFValues.begin(), desiredDOFValues.end());
        double* stepPtr = nullptr;
        std::vector<double> steps;
        if (!desiredStepSizes.empty()) {
            if (desiredStepSizes.size() != static_cast<size_t>(numDOF)) {
                throw std::runtime_error("Incorrect number of step sizes. Expected " + std::to_string(numDOF));
            }
            steps.assign(desiredStepSizes.begin(), desiredStepSizes.end());
            stepPtr = steps.data();
        }

        struct StepRec {
            int numCols;
            std::vector<double> dofs;
        };
        std::vector<StepRec> trace;
        trace.reserve(128);

        auto conn = QObject::connect(
            robot,
            &Robot::moveDOFStepTaken,
            robot,
            [&](int numCols, bool &stopRequest) {
                (void)stopRequest;
                StepRec rec;
                rec.numCols = numCols;
                rec.dofs.resize(numDOF);
                robot->getDOFVals(rec.dofs.data());
                trace.push_back(std::move(rec));
            },
            Qt::DirectConnection
        );

        const bool moved = robot->moveDOFToContacts(desiredVals.data(), stepPtr, stopAtContact, renderIt);
        QObject::disconnect(conn);

        py::list out;
        for (const auto& rec : trace) {
            py::dict d;
            d["num_cols"] = rec.numCols;
            d["dof_values"] = rec.dofs;
            out.append(d);
        }
        return py::make_tuple(moved, out);
    }

    void findAllContacts() {
        if (!world) throw std::runtime_error("World not initialized");
        world->findAllContacts();
    }
};

class PythonGrasp {
public:
    Grasp* grasp;
    World* world = nullptr;
    bool quasistaticWrenchesComputed = false;
    bool quasistaticWrenchesAttempted = false;
    int lastQuasistaticResult = 0;

    bool refreshContacts(bool invalidateQuasistatic = true) {
        if (!grasp) {
            return false;
        }

        if (world) {
            world->findAllContacts();
        }

        grasp->collectContacts();
        grasp->update();

        if (invalidateQuasistatic) {
            quasistaticWrenchesComputed = false;
            quasistaticWrenchesAttempted = false;
            lastQuasistaticResult = 0;
        }

        return grasp->isValid();
    }

    PythonGrasp(PythonWorld& pyWorld, int graspableBodyIndex = 0) {
        if (!pyWorld.world) {
            throw std::runtime_error("World not initialized");
        }
        world = pyWorld.world;
        if (pyWorld.world->getNumGB() <= 0) {
            throw std::runtime_error("World has no graspable bodies");
        }
        if (graspableBodyIndex < 0 || graspableBodyIndex >= pyWorld.world->getNumGB()) {
            throw std::runtime_error("Invalid graspableBodyIndex");
        }

        Hand* hand = pyWorld.world->getCurrentHand();
        if (!hand) {
            throw std::runtime_error("World has no current hand (load a robot/hand first)");
        }

        GraspableBody* object = pyWorld.world->getGB(graspableBodyIndex);
        grasp = new Grasp(hand);
        grasp->setObject(object);
    }

    ~PythonGrasp() {
        if (grasp) {
            delete grasp;
        }
    }

    bool computeGrasp() {
        return refreshContacts(true);
    }

    void ensureQuasistaticWrenches() {
        if (!grasp) {
            return;
        }
        if (quasistaticWrenchesComputed || quasistaticWrenchesAttempted) {
            return;
        }

        quasistaticWrenchesAttempted = true;

        // GraspIt only populates Contact::dynamicForce wrenches when a
        // quasi-static force computation is run.
        // This solver also writes the resulting contact wrenches onto the
        // *mate* contacts (typically on the object) for visualization.
        Hand* hand = grasp->getHand();
        if (!hand) {
            std::fprintf(stderr, "ensureQuasistaticWrenches: no hand\n");
            return;
        }

        int numJoints = hand->getNumJoints();
        std::fprintf(stderr, "ensureQuasistaticWrenches: hand has %d joints\n", numJoints);

        if (numJoints <= 0) {
            std::fprintf(stderr, "ensureQuasistaticWrenches: invalid numJoints\n");
            lastQuasistaticResult = -1;
            return;
        }

        Matrix robotTau(numJoints, 1);
        robotTau.setAllElements(0.0);

        // Try CONTACT_FORCE_EXISTENCE first (simpler, just checks feasibility)
        // If infeasible, try GRASP_FORCE_EXISTENCE (includes joint torque constraints)
        lastQuasistaticResult = grasp->computeQuasistaticForcesAndTorques(&robotTau, Grasp::CONTACT_FORCE_EXISTENCE);
        if (lastQuasistaticResult != 0) {
            // Try the alternative formulation
            robotTau.setAllElements(0.0);
            int altResult = grasp->computeQuasistaticForcesAndTorques(&robotTau, Grasp::GRASP_FORCE_EXISTENCE);
            if (altResult == 0) {
                lastQuasistaticResult = 0;
                std::fprintf(stderr, "CONTACT_FORCE_EXISTENCE failed, but GRASP_FORCE_EXISTENCE succeeded\n");
            } else {
                std::fprintf(stderr, "computeQuasistaticForcesAndTorques failed with result: %d (both formulations)\n", lastQuasistaticResult);
            }
        }
        quasistaticWrenchesComputed = (lastQuasistaticResult == 0);
    }

    int computeQuasistaticWrenches() {
        ensureQuasistaticWrenches();
        return lastQuasistaticResult;
    }

    bool quasistaticWrenchesAvailable() const {
        return quasistaticWrenchesComputed;
    }

    double getVolumeQuality() {
        if (!grasp) return 0.0;
        // Refresh contacts to avoid evaluating on a stale/empty set after motion.
        if (!refreshContacts(false)) {
            return -1.0;
        }
        QualVolume qm(grasp, "volume", "L1 Norm");
        // QualVolume adds/uses a GWS. Ensure the grasp is updated *after* the GWS
        // exists so its hyperplanes are built.
        grasp->update();
        return qm.evaluate();
    }

    double getEpsilonQuality() {
        if (!grasp) return 0.0;
        // Refresh contacts to avoid evaluating on a stale/empty set after motion.
        if (!refreshContacts(false)) {
            return -1.0;
        }
        QualEpsilon qm(grasp, "epsilon", "L1 Norm");
        // QualEpsilon adds/uses a GWS. Ensure the grasp is updated *after* the GWS
        // exists so its hyperplanes are built.
        grasp->update();
        return qm.evaluate();
    }

    std::vector<std::array<double, 3>> getContactPoints() {
        std::vector<std::array<double, 3>> points;
        if (!grasp) return points;

        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (contact) {
                // translation() returns by value; do NOT bind a reference to it.
                const vec3 p = contact->getContactFrame().translation();
                points.push_back({p.x(), p.y(), p.z()});
            }
        }
        return points;
    }

    std::vector<std::array<double, 3>> getContactNormals() {
        std::vector<std::array<double, 3>> normals;
        if (!grasp) return normals;

        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (contact) {
                // getNormal() returns by value in this GraspIt version.
                const vec3 n = contact->getNormal();
                normals.push_back({n.x(), n.y(), n.z()});
            }
        }
        return normals;
    }

    py::list getObjectContactGeometry() {
        py::list result;
        if (!grasp) return result;
        for (int i = 0; i < grasp->getNumContacts(); ++i) {
            Contact* handContact = grasp->getContact(i);
            Contact* contact = handContact ? handContact->getMate() : nullptr;
            if (!contact) throw std::runtime_error("Contact has no object-side mate");
            const vec3 p = contact->getContactFrame().translation();
            const vec3 n = contact->getNormal();
            py::dict record;
            record["position_mm"] = std::array<double, 3>{p.x(), p.y(), p.z()};
            record["normal"] = std::array<double, 3>{n.x(), n.y(), n.z()};
            result.append(record);
        }
        return result;
    }

    std::vector<double> getContactNormalForces(bool useLimits = false) {
        std::vector<double> forces;
        if (!grasp) return forces;

        if (!useLimits) {
            ensureQuasistaticWrenches();
        }

        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (contact) {
                if (useLimits) {
                    const double limit = contact->getNormalForceLimit();
                    forces.push_back(std::isfinite(limit) ? limit : 0.0);
                } else {
                    Contact* wrenchContact = contact->getMate() ? contact->getMate() : contact;
                    double* wrench = wrenchContact->getDynamicContactWrench();
                    const double fz = wrench ? wrench[2] : 0.0;
                    forces.push_back(std::isfinite(fz) ? fz : 0.0);
                }
            }
        }
        return forces;
    }

    std::vector<std::optional<double>> getContactNormalForcesOptional() {
        std::vector<std::optional<double>> forces;
        if (!grasp) return forces;

        ensureQuasistaticWrenches();

        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (!contact) {
                forces.push_back(std::nullopt);
                continue;
            }

            if (!quasistaticWrenchesComputed) {
                forces.push_back(std::nullopt);
                continue;
            }

            Contact* wrenchContact = contact->getMate() ? contact->getMate() : contact;
            double* wrench = wrenchContact->getDynamicContactWrench();
            if (!wrench || !std::isfinite(wrench[2])) {
                forces.push_back(std::nullopt);
            } else {
                forces.push_back(wrench[2]);
            }
        }

        return forces;
    }

    std::vector<std::array<double, 6>> getContactWrenches() {
        std::vector<std::array<double, 6>> wrenches;
        if (!grasp) return wrenches;

        ensureQuasistaticWrenches();

        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (contact) {
                Contact* wrenchContact = contact->getMate() ? contact->getMate() : contact;
                double* wrench = wrenchContact->getDynamicContactWrench();
                if (wrench) {
                    const double wx = std::isfinite(wrench[0]) ? wrench[0] : 0.0;
                    const double wy = std::isfinite(wrench[1]) ? wrench[1] : 0.0;
                    const double wz = std::isfinite(wrench[2]) ? wrench[2] : 0.0;
                    const double tx = std::isfinite(wrench[3]) ? wrench[3] : 0.0;
                    const double ty = std::isfinite(wrench[4]) ? wrench[4] : 0.0;
                    const double tz = std::isfinite(wrench[5]) ? wrench[5] : 0.0;
                    wrenches.push_back({wx, wy, wz, tx, ty, tz});
                } else {
                    wrenches.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
                }
            }
        }
        return wrenches;
    }

    double getNetNormalForce(bool useLimits = false, bool absolute = true) {
        if (!grasp) return 0.0;
        if (!useLimits) {
            ensureQuasistaticWrenches();
        }
        double total = 0.0;
        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (!contact) {
                continue;
            }
            double value = 0.0;
            if (useLimits) {
                const double limit = contact->getNormalForceLimit();
                value = std::isfinite(limit) ? limit : 0.0;
            } else {
                Contact* wrenchContact = contact->getMate() ? contact->getMate() : contact;
                double* wrench = wrenchContact->getDynamicContactWrench();
                const double fz = wrench ? wrench[2] : 0.0;
                value = std::isfinite(fz) ? fz : 0.0;
            }
            total += absolute ? std::abs(value) : value;
        }
        return total;
    }

    bool isNonPenetrating() {
        if (!grasp) return false;
        // GraspIt does not expose a simple "isLegal" on Grasp in this version.
        // As a lightweight proxy, report whether the grasp is updated/valid.
        return grasp->isValid();
    }

    std::vector<double> getContactFrictionCoefficients() {
        std::vector<double> cofs;
        if (!grasp) return cofs;

        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (contact) {
                cofs.push_back(contact->getCof());
            }
        }
        return cofs;
    }

    std::vector<double> getContactSlipProxy() {
        std::vector<double> slipValues;
        if (!grasp) return slipValues;

        ensureQuasistaticWrenches();

        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (!contact) {
                slipValues.push_back(0.0);
                continue;
            }

            Contact* wrenchContact = contact->getMate() ? contact->getMate() : contact;
            double* wrench = wrenchContact->getDynamicContactWrench();
            double cof = contact->getCof();

            if (!wrench || !std::isfinite(wrench[2]) || wrench[2] <= 1e-9) {
                slipValues.push_back(0.0);
                continue;
            }

            double fn = std::abs(wrench[2]);
            double ft = std::sqrt(wrench[0] * wrench[0] + wrench[1] * wrench[1]);
            double mu_required = ft / fn;
            double slip_proxy = (cof > 1e-9) ? (mu_required / cof) : 0.0;
            slipValues.push_back(std::min(slip_proxy, 1.0));
        }
        return slipValues;
    }

    std::vector<double> getContactRequiredFriction() {
        std::vector<double> muReq;
        if (!grasp) return muReq;

        ensureQuasistaticWrenches();

        for (int i = 0; i < grasp->getNumContacts(); i++) {
            Contact* contact = grasp->getContact(i);
            if (!contact) {
                muReq.push_back(0.0);
                continue;
            }

            Contact* wrenchContact = contact->getMate() ? contact->getMate() : contact;
            double* wrench = wrenchContact->getDynamicContactWrench();

            if (!wrench || !std::isfinite(wrench[2]) || wrench[2] <= 1e-9) {
                muReq.push_back(0.0);
                continue;
            }

            double fn = std::abs(wrench[2]);
            double ft = std::sqrt(wrench[0] * wrench[0] + wrench[1] * wrench[1]);
            muReq.push_back(ft / fn);
        }
        return muReq;
    }
};

// Initialization function
void initializeGraspit() {
    static bool initialized = false;
    static QApplication* app = nullptr;

    if (!initialized) {
        // Help GraspIt find its resources (models/worlds) in common dev layouts.
        if (!qEnvironmentVariableIsSet("GRASPIT")) {
            Dl_info info;
            if (dladdr(reinterpret_cast<void*>(&initializeGraspit), &info) && info.dli_fname) {
                try {
                    std::filesystem::path soPath(info.dli_fname);
                    // .../graspit_python_wrapper/graspit_python_wrapper/core/_bindings*.so
                    // -> go up to workspace root and look for ./graspit
                    auto candidate = soPath.parent_path()      // core
                                         .parent_path()        // graspit_python_wrapper (package)
                                         .parent_path()        // graspit_python_wrapper (repo)
                                         .parent_path()        // graspit-simulator (workspace)
                                         / "graspit";
                    if (std::filesystem::exists(candidate)) {
                        qputenv("GRASPIT", QByteArray(candidate.string().c_str()));
                    }
                } catch (...) {
                    // Best-effort only.
                }
            }

            // Fallback: search upwards from CWD for a sibling graspit/ directory.
            if (!qEnvironmentVariableIsSet("GRASPIT")) {
                try {
                    auto cur = std::filesystem::current_path();
                    for (int i = 0; i < 8; ++i) {
                        auto candidate = cur / "graspit";
                        if (std::filesystem::exists(candidate / "CMakeLists.txt")) {
                            qputenv("GRASPIT", QByteArray(candidate.string().c_str()));
                            break;
                        }
                        if (!cur.has_parent_path()) {
                            break;
                        }
                        cur = cur.parent_path();
                    }
                } catch (...) {
                    // Best-effort only.
                }
            }
        }

        // GraspIt constructs QWidget-based UI elements, so we need a QApplication.
        // In headless environments, default to offscreen unless the user set a platform.
        if (!QCoreApplication::instance()) {
            if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) {
                // If DISPLAY isn't set, assume headless.
                if (!qEnvironmentVariableIsSet("DISPLAY")) {
                    qputenv("QT_QPA_PLATFORM", QByteArray("offscreen"));
                }
            }

            int argc = 1;
            static char arg0[] = "graspit_python";
            static char* argv[] = {arg0, nullptr};
            app = new SafeQApplication(argc, argv);
        }

        // Initialize GraspIt! core (requires argc/argv).
        int argc = 1;
        static char arg0_core[] = "graspit_python";
        static char* argv_core[] = {arg0_core, nullptr};
        graspitCore = new GraspitCore(argc, argv_core);

        // Match graspit/src/main.cpp behavior so closing the last window exits
        // the Qt event loop when used as a library.
        if (qApp) {
            QObject::connect(qApp, SIGNAL(lastWindowClosed()), qApp, SLOT(quit()));
        }
        initialized = true;
    }
}

static bool guiAvailable() {
    return graspitCore && graspitCore->getMainWindow() && graspitCore->getIVmgr();
}

static void showGui() {
    if (!graspitCore) {
        throw std::runtime_error("GraspIt core not initialized; call initialize() first");
    }
    if (!guiAvailable()) {
        throw std::runtime_error(
            "GraspIt was initialized headless (no MainWindow/IVmgr). "
            "Ensure QT_QPA_PLATFORM is not offscreen and DISPLAY is set, then re-run."
        );
    }

    // Mirror GraspitCore::startMainLoop() setup without blocking.
    graspitCore->getMainWindow()->setMainWorld(graspitCore->getWorld());
    SoQt::show(graspitCore->getMainWindow()->mWindow);
    graspitCore->getMainWindow()->mWindow->resize(QSize(1070, 937));
}

static void viewAll() {
    if (!graspitCore) {
        throw std::runtime_error("GraspIt core not initialized; call initialize() first");
    }
    if (!guiAvailable()) {
        throw std::runtime_error(
            "GraspIt was initialized headless (no MainWindow/IVmgr). "
            "Ensure QT_QPA_PLATFORM is not offscreen and DISPLAY is set, then re-run."
        );
    }

    showGui();
    auto* ivmgr = graspitCore->getIVmgr();
    if (!ivmgr || !ivmgr->getViewer()) {
        throw std::runtime_error("GraspIt viewer is not available");
    }

    // Equivalent to the Inventor toolbar "view all" (often the eye icon).
    // Note: viewAll() updates the Inventor camera, but the Qt widget repaint can lag.
    // We do a brief best-effort settle so that subsequent screenshots reflect the new view.
    ivmgr->getViewer()->viewAll();
    ivmgr->getViewer()->render();

    if (QCoreApplication::instance()) {
        // ~0.8s settle: enough for camera change + viewer redraw + Qt repaint.
        for (int i = 0; i < 16; ++i) {
            QCoreApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        ivmgr->getViewer()->render();
        for (int i = 0; i < 4; ++i) {
            QCoreApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

static void saveScreenshot(const std::string& filename) {
    if (!graspitCore) {
        throw std::runtime_error("GraspIt core not initialized; call initialize() first");
    }
    if (!guiAvailable()) {
        throw std::runtime_error(
            "GraspIt was initialized headless (no MainWindow/IVmgr). "
            "Ensure QT_QPA_PLATFORM is not offscreen and DISPLAY is set, then re-run."
        );
    }
    if (!graspitCore->getMainWindow() || !graspitCore->getMainWindow()->mWindow) {
        throw std::runtime_error("GraspIt main window is not available");
    }

    // Ensure the view is zoomed out / framed before capture.
    // Then settle once more right before grabbing to avoid catching mid-redraw.
    viewAll();
    if (QCoreApplication::instance()) {
        for (int i = 0; i < 6; ++i) {
            QCoreApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    QPixmap pix = graspitCore->getMainWindow()->mWindow->grab();
    if (pix.isNull()) {
        throw std::runtime_error("Failed to grab GraspIt main window pixels");
    }
    if (!pix.save(QString::fromStdString(filename))) {
        throw std::runtime_error("Failed to save screenshot to: " + filename);
    }
}

static std::uint64_t mainWindowXid() {
    if (!graspitCore) {
        throw std::runtime_error("GraspIt core not initialized; call initialize() first");
    }
    if (!guiAvailable()) {
        throw std::runtime_error(
            "GraspIt was initialized headless (no MainWindow/IVmgr). "
            "Ensure QT_QPA_PLATFORM is not offscreen and DISPLAY is set, then re-run."
        );
    }

    showGui();
    if (!graspitCore->getMainWindow() || !graspitCore->getMainWindow()->mWindow) {
        throw std::runtime_error("GraspIt main window is not available");
    }

    // On X11, QWidget::winId() is the XID. We expose it so external tools (xwd)
    // can capture the correct window without interactive selection.
    WId wid = graspitCore->getMainWindow()->mWindow->winId();
    return static_cast<std::uint64_t>(wid);
}

static void startMainLoop() {
    if (!graspitCore) {
        throw std::runtime_error("GraspIt core not initialized; call initialize() first");
    }
    graspitCore->startMainLoop();
}

static void mainLoop(double seconds) {
    if (!graspitCore) {
        throw std::runtime_error("GraspIt core not initialized; call initialize() first");
    }
    if (!guiAvailable()) {
        throw std::runtime_error(
            "GraspIt was initialized headless (no MainWindow/IVmgr). "
            "Ensure QT_QPA_PLATFORM is not offscreen and DISPLAY is set, then re-run."
        );
    }

    showGui();

    // Timed exit: use Inventor sensors rather than Qt timers.
    // This avoids exceptions/instability from mixing Qt timers with SoQt's main loop.
    static SoAlarmSensor* exitSensor = nullptr;
    if (exitSensor) {
        exitSensor->unschedule();
        delete exitSensor;
        exitSensor = nullptr;
    }
    if (seconds >= 0.0) {
        exitSensor = new SoAlarmSensor(
            [](void*, SoSensor*) {
                SoQt::exitMainLoop();
            },
            nullptr
        );
        exitSensor->setTimeFromNow(SbTime(seconds));
        exitSensor->schedule();
    }

    py::gil_scoped_release release;
    try {
        // This shows the main window and then runs the SoQt main loop.
        graspitCore->startMainLoop();
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("GraspIt main loop failed: ") + e.what());
    } catch (...) {
        throw std::runtime_error("GraspIt main loop failed with unknown exception");
    }
}

static int eventLoop(double seconds) {
    if (!QCoreApplication::instance()) {
        throw std::runtime_error("No Q(Core)Application exists; call initialize() first");
    }

    if (seconds >= 0.0) {
        const int ms = static_cast<int>(seconds * 1000.0);

        if (guiAvailable()) {
            // Closing the main window tends to be much safer than forcing a quit
            // while SoQt viewers/sensors are active.
            QTimer::singleShot(ms, []() {
                if (graspitCore && graspitCore->getMainWindow() && graspitCore->getMainWindow()->mWindow) {
                    graspitCore->getMainWindow()->mWindow->close();
                } else {
                    QCoreApplication::quit();
                }
            });
        } else {
            QTimer::singleShot(ms, []() { QCoreApplication::quit(); });
        }
    }

    py::gil_scoped_release release;
    return QCoreApplication::exec();
}

static void exitMainLoop() {
    if (!graspitCore) {
        return;
    }
    graspitCore->exitMainLoop();
}

PYBIND11_MODULE(_bindings, m) {
    m.doc() = "GraspIt! Python bindings";

    // Utility functions
    m.def("initialize", &initializeGraspit, "Initialize GraspIt! core");

    // GUI helpers
    m.def("gui_available", &guiAvailable, "Return True if the GraspIt GUI is available (not headless)");
    m.def("show_gui", &showGui, "Show the GraspIt main window (non-blocking)");
    m.def("view_all", &viewAll, "Frame the full scene in the Inventor viewer (zoom out / view all).");
    m.def("main_window_xid", &mainWindowXid, "Return the X11 window id (XID) for the GraspIt main window.");
    m.def(
        "save_screenshot",
        &saveScreenshot,
        py::arg("filename"),
        "Save a screenshot of the GraspIt main window to an image file (e.g. .png)."
    );
    m.def("start_main_loop", &startMainLoop, "Start the SoQt main loop (blocking)");
    m.def(
        "main_loop",
        &mainLoop,
        py::arg("seconds") = -1.0,
        "Show the GUI (if available) and run the SoQt main loop. "
        "Pass seconds>=0 to auto-exit after a duration."
    );
    m.def(
        "event_loop",
        &eventLoop,
        py::arg("seconds") = -1.0,
        "Run the Qt event loop (QCoreApplication::exec). "
        "Pass seconds>=0 to auto-quit after a duration."
    );
    m.def("exit_main_loop", &exitMainLoop, "Exit the SoQt main loop");

    // Qt resources helper
    m.def(
        "resource_exists",
        [](const std::string& path) {
            return QFile::exists(QString::fromStdString(path));
        },
        py::arg("path"),
        "Return True if a Qt resource/path exists (e.g. :/images/play.xpm)."
    );

    // Qt event processing helpers (useful for GUI runs in scripts)
    m.def(
        "process_events",
        []() {
            if (QCoreApplication::instance()) {
                QCoreApplication::processEvents();
            }
        },
        "Process pending Qt events once (no-op if no QCoreApplication exists)."
    );

    m.def(
        "spin",
        [](double seconds, int interval_ms) {
            py::gil_scoped_release release;
            if (interval_ms < 1) {
                interval_ms = 1;
            }
            const bool run_forever = seconds < 0.0;
            const auto start = std::chrono::steady_clock::now();
            while (run_forever || (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds)) {
                if (QCoreApplication::instance()) {
                    QCoreApplication::processEvents();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
        },
        py::arg("seconds") = -1.0,
        py::arg("interval_ms") = 16,
        "Run a lightweight loop that periodically processes Qt events for a duration. "
        "Pass seconds<0 to spin forever."
    );

    // Vector conversions
    m.def("vec3_to_numpy", &vec3_to_numpy, "Convert vec3 to numpy array");
    m.def("numpy_to_vec3", &numpy_to_vec3, "Convert numpy array to vec3");

    // World wrapper
    py::class_<PythonWorld>(m, "World")
        .def(py::init([]() {
            PythonWorld w;
            w.init();
            return w;
        }))
        .def("init", &PythonWorld::init)
        .def(
            "find_all_contacts",
            &PythonWorld::findAllContacts,
            "Compute and populate all proximity contacts in the world (uses Contact::THRESHOLD)."
        )
        .def(
            "load_world",
            &PythonWorld::loadWorldFromXml,
            py::arg("filename"),
            "Load a simple GraspIt! world XML (robots + graspable bodies)."
        )
        .def("load_robot", &PythonWorld::loadRobot)
        .def("load_body", &PythonWorld::loadBody)
        .def("load_graspable_body", &PythonWorld::loadGraspableBody)
         .def("set_graspable_body_translation", &PythonWorld::setGraspableBodyTranslation,
              py::arg("graspable_body_index"), py::arg("x"), py::arg("y"), py::arg("z"))
           .def("set_body_dynamic", &PythonWorld::setGraspableBodyDynamic,
               py::arg("graspable_body_index"), py::arg("dynamic"),
               "Set whether a graspable body is dynamic (simulated) or static.")
           .def("get_graspable_body_translation", &PythonWorld::getGraspableBodyTranslation,

               py::arg("graspable_body_index"))
           .def(
               "get_graspable_body_pose",
               &PythonWorld::getGraspableBodyPose,
               py::arg("graspable_body_index"),
               "Return graspable body pose in world frame as {quat_wxyz, translation, matrix_row_major, full_transform}."
           )
           .def(
               "get_graspable_body_cog_world",
               &PythonWorld::getGraspableBodyCoGWorld,
               py::arg("graspable_body_index"),
               "Return graspable body's center-of-gravity (CoG) in world frame as [x,y,z] (GraspIt units, typically mm)."
           )
           .def(
               "get_graspable_body_cog_local",
               &PythonWorld::getGraspableBodyCoGLocal,
               py::arg("graspable_body_index"),
               "Return graspable body's center-of-gravity (CoG) in the body's local frame as [x,y,z] (GraspIt units, typically mm)."
           )
           .def(
               "get_graspable_body_pose_relative_to_hand",
               &PythonWorld::getGraspableBodyPoseRelativeToHand,
               py::arg("graspable_body_index"),
               "Return graspable body pose in hand frame as {quat_wxyz, translation, matrix_row_major, full_transform}."
           )
           .def(
               "get_current_hand_pose",
               &PythonWorld::getCurrentHandPose,
               "Return current hand base pose in world frame as {quat_wxyz, translation, matrix_row_major, full_transform}."
           )
           .def(
               "set_current_hand_pose",
               &PythonWorld::setCurrentHandPose,
               py::arg("quat_wxyz"), py::arg("translation_mm"),
               "Place the hand base at a world pose (no collision check; call is_non_penetrating afterwards)."
           )
           .def(
               "clear",
               &PythonWorld::clear,
               "Remove every robot and body. World() wraps GraspIt's single main world, so "
               "separate World objects share contents; clear() gives a fresh scene."
           )
           .def(
               "no_collision",
               &PythonWorld::noCollision,
               "True if no pair of bodies in the world interpenetrates (GraspIt World::noCollision)."
           )
           .def(
               "get_link_poses",
               &PythonWorld::getLinkPoses,
               "World poses of the palm (chain=-1) and every chain link, each with chain, link and name fields."
           )
           .def(
               "get_link_vertices_world",
               &PythonWorld::getLinkVerticesWorld,
               py::arg("chain"), py::arg("link"),
               "Geometry vertices of one hand link in the world frame (mm), shape (N, 3); chain=-1 is the palm."
           )
           .def("place_graspable_body_relative_to_hand", &PythonWorld::placeGraspableBodyRelativeToHand,
               py::arg("graspable_body_index"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def(
            "auto_grasp",
            &PythonWorld::autoGrasp,
            py::arg("render_it") = false,
            py::arg("speed_factor") = 1.0,
            py::arg("stop_at_contact") = false,
            "Close fingers using the robot's configured autograsp motion."
        )
        .def(
            "quick_open",
            &PythonWorld::quickOpen,
            py::arg("speed_factor") = 0.1,
            "Open fingers in the opposite direction of autograsp."
        )
        .def(
            "approach_to_contact",
            &PythonWorld::approachToContact,
            py::arg("move_dist"),
            py::arg("one_step") = true,
            "Move hand along its configured approach direction until contact."
        )
        .def("translate_robot", &PythonWorld::translateCurrentRobot,
             py::arg("dx"), py::arg("dy"), py::arg("dz"),
             "Translate the current robot base by (dx,dy,dz) in its own frame.")
        .def("num_robots", &PythonWorld::getNumRobots)
        .def("num_bodies", &PythonWorld::getNumBodies)
        .def("num_graspable_bodies", &PythonWorld::getNumGB)
        .def(
            "show_friction_cones",
            &PythonWorld::showFrictionCones,
            py::arg("on"),
            "Show or hide friction cones on all bodies."
        )
        .def(
            "show_axes",
            &PythonWorld::showAxes,
            py::arg("on"),
            "Show or hide coordinate axes on all bodies and robot links."
        )
        .def(
            "set_clean_display",
            &PythonWorld::setCleanDisplay,
            py::arg("clean"),
            "Convenience: if clean=True, hide friction cones and axes; if False, show them."
        )
        .def("get_current_hand_num_dof", &PythonWorld::getCurrentHandNumDOF,
             "Get number of degrees of freedom in the current hand")
        .def("get_dof_default_velocities", &PythonWorld::getDOFDefaultVelocities,
             "Per-DOF autograsp velocities; auto_grasp holds a DOF with velocity 0 at its current value")
        .def("set_dof_default_velocities", &PythonWorld::setDOFDefaultVelocities, py::arg("velocities"),
             "Set per-DOF autograsp velocities (0 holds the DOF during auto_grasp)")
        .def("get_current_hand_dof_values", &PythonWorld::getCurrentHandDOFValues,
             "Get all DOF values (joint positions) of the current hand as a list")
           .def(
              "move_current_hand_dof_to_contacts",
              &PythonWorld::moveCurrentHandDOFToContacts,
              py::arg("desired_dof_values"),
              py::arg("desired_step_sizes") = std::vector<double>{},
              py::arg("stop_at_contact") = false,
              py::arg("render_it") = false,
              "Move the current hand towards desired DOF values in small steps with collision checking."
           )
              .def(
                  "trace_move_current_hand_dof_to_contacts",
                  &PythonWorld::traceMoveCurrentHandDOFToContacts,
                  py::arg("desired_dof_values"),
                  py::arg("desired_step_sizes") = std::vector<double>{},
                  py::arg("stop_at_contact") = false,
                  py::arg("render_it") = false,
                  "Move the hand to desired DOFs and return a trace of internal DOF states (one per moveDOFStepTaken signal)."
              )
        .def("set_current_hand_dof_values", &PythonWorld::setCurrentHandDOFValues,
             py::arg("dof_values"),
             "Set all DOF values (joint positions) of the current hand from a list");

    // Grasp wrapper
    py::class_<PythonGrasp>(m, "Grasp")
        .def(py::init<PythonWorld&, int>(), py::arg("world"), py::arg("graspable_body_index") = 0)
        .def("compute", &PythonGrasp::computeGrasp)
        .def("volume_quality", &PythonGrasp::getVolumeQuality)
        .def("epsilon_quality", &PythonGrasp::getEpsilonQuality)
        .def("object_contact_geometry", &PythonGrasp::getObjectContactGeometry,
             "Object-side contacts in the object frame: position_mm and inward force normal.")
        .def("contact_points", &PythonGrasp::getContactPoints)
        .def("contact_normals", &PythonGrasp::getContactNormals)
        .def(
            "compute_quasistatic_wrenches",
            &PythonGrasp::computeQuasistaticWrenches,
            "Attempt to compute quasi-static contact wrenches. Returns 0 on success; non-zero means unavailable/failed."
        )
        .def(
            "quasistatic_wrenches_available",
            &PythonGrasp::quasistaticWrenchesAvailable,
            "Return True if quasi-static contact wrenches were computed successfully."
        )
        .def(
            "contact_normal_forces",
            &PythonGrasp::getContactNormalForces,
            py::arg("use_limits") = false,
            "Return per-contact normal force values."
        )
        .def(
            "contact_normal_forces_optional",
            &PythonGrasp::getContactNormalForcesOptional,
            "Return per-contact normal force values if available; otherwise None for each contact."
        )
        .def("contact_wrenches", &PythonGrasp::getContactWrenches)
        .def(
            "net_normal_force",
            &PythonGrasp::getNetNormalForce,
            py::arg("use_limits") = false,
            py::arg("absolute") = true,
            "Return summed normal forces across contacts."
        )
        .def("is_non_penetrating", &PythonGrasp::isNonPenetrating,
             "Legacy name: returns GraspIt Grasp::isValid(), NOT a penetration test. "
             "Use World.no_collision() to test interpenetration.")
        .def(
            "contact_friction_coefficients",
            &PythonGrasp::getContactFrictionCoefficients,
            "Return per-contact friction coefficients."
        )
        .def(
            "contact_slip_proxy",
            &PythonGrasp::getContactSlipProxy,
            "Return per-contact slip proxy in [0,1]. Computed as (mu_required / mu_available)."
        )
        .def(
            "contact_required_friction",
            &PythonGrasp::getContactRequiredFriction,
            "Return per-contact required friction coefficient mu_req = ||f_t|| / f_n."
        );
}