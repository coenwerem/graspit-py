# graspit-py

**graspit-py** is a Python library for planning and evaluating robotic grasps using simulated finger closure and contact-force analysis.

It provides direct access to the
[GraspIt! simulator](https://graspit-simulator.github.io/) for planning
and evaluating grasps with multifingered hands. See
{ref}`Miller and Allen <ref-miller-allen>` for the simulator's design.
A Python program can set the palm pose and finger coordinates, invoke
closure, and retrieve object-frame contact positions and normals. This interface makes each simulated closure
available for inspection and external scoring. A planner built on the
interface enumerates approaches and ranks their closures by general-purpose
or task-specific wrench capacity under approach and friction uncertainty.

The MuJoCo exporter produces GraspIt! robot and object models from a
compiled MuJoCo scene. It records the joint and link correspondence needed
to transfer planned configurations to the source model. The application
uses the resulting targets for arm kinematics, motion planning, and
execution. The [source repository](https://github.com/coenwerem/graspit-py)
contains the code and examples. See [Citation and References](references.md)
for software citation metadata and the
{ref}`companion paper <software-preprint>`. A preprint link will be added here when available.

````{container} hand-gallery hand-gallery-overview
```{figure} figures/l6-insertion.png
:alt: RealHand L6 at the planned grasp on a blue insertion key.

**RealHand L6.** Planned key grasp.
```
```{figure} figures/leap-hold.png
:alt: Four-finger LEAP Hand retaining a blue cube.

**LEAP Hand.** Simulated cube hold.
```
```{figure} figures/ability-hold.png
:alt: PSYONIC Ability Hand holding a blue sphere.

**PSYONIC Ability Hand.** Simulated sphere hold.
```
```{figure} figures/barrett-grasp-preview.png
:target: hands.html#grasps-on-native-and-exported-hands
:alt: Native Barrett hand closed around a cube with six verified contacts.

**Barrett Hand.** Cube grasp.
```
```{figure} figures/shadow-grasp-preview.png
:target: hands.html#grasps-on-native-and-exported-hands
:alt: Exported Shadow hand grasping a sphere with all five fingers.

**Shadow Hand.** Planned sphere grasp.
```
```{figure} figures/inspire-grasp-preview.png
:target: hands.html#grasps-on-native-and-exported-hands
:alt: Exported Inspire hand grasping a sphere with all five fingers.

**Inspire Hand.** Planned sphere grasp.
```
````

Start with [Installation](INSTALL.md) and the [first-grasp
walkthrough](quickstart.md). The [planning guide](planning.md) explains
approach specification, task loading, and the grasp records shown above.
The [export guide](export.md) covers model conversion and transfer back
to MuJoCo. The [theory guide](theory.md) derives the joint mapping and
wrench scores used in both guides.

## From Models to Grasp Records

```{figure} figures/architecture.png
:alt: Data flow from MuJoCo model conversion and Python approach settings through native closure, numerical wrench analysis, and grasp records for application clients.
:target: _images/architecture.png

Model conversion supplies geometry and the joint map. Native closure provides contacts, and numerical analysis supplies wrench-quality samples. A grasp record connects the selected command to its nominal contacts and scores. Select the figure for a full-size view.
```

The [API reference](api.md) groups the corresponding operations by scene setup, hand state, contacts, and geometry. The [theory guide](theory.md) connects their numerical inputs to the contact and wrench models.

```{toctree}
:maxdepth: 2

INSTALL
quickstart
theory
export
planning
hands
api
roadmap
references
```
