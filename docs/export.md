# MuJoCo Model Export

The exporter converts the hand and object geometry in a compiled
MuJoCo model to GraspIt! XML and OFF meshes. It records an affine map
from GraspIt! coordinates to source hinges and a correspondence between
exported links and source bodies. Together these records allow planning
in GraspIt! and reconstruction of the resulting configuration in
MuJoCo. The [kinematics guide](theory.md) derives the reference-angle
conversion. See {ref}`Todorov, Erez, and Tassa <ref-todorov-mujoco>`
for the MuJoCo simulator.

We evaluated each simulator at corresponding joint coordinates to
compare the exported and source mesh geometries in `source` mode. Across 49
configurations of the right RealHand L6 and 12 links, the maximum
symmetric Hausdorff distance between corresponding link-vertex sets
was 7.86 nm.

````{container} hand-gallery export-gallery
```{figure} figures/export-source-workcell.png
:alt: MuJoCo source workcell with two A7 arms and RealHand L6 hands during key insertion.

MuJoCo source workcell during a recorded bimanual insertion.
```
```{figure} figures/export-grasp-frames.png
:alt: Exported right RealHand L6 at the planned grasp on the insertion key.

GraspIt! configuration of the exported right L6 at the planned key grasp.
```
````

## Export a Hand and Object

Load an MJCF or URDF scene in MuJoCo and pass the compiled model to the
exporter. Select the palm as the hand root and the rigid object's body
as the object root. The hand export covers the palm and its finger
subtree. The robot arm and workcell remain in the source scene.

```python
import mujoco

from graspit_py.convert.mjcf_hand import export_body, export_hand

model = mujoco.MjModel.from_xml_path("scene.xml")
hand = export_hand(model, "palm", "output/hand", name="my_hand")
object_xml = export_body(model, "object", "output/object")
```

Pass `hand.robot_xml` and `object_xml` to the
[contact planner](planning.md). Keep the hand XML adjacent to its
`iv/` directory and the object XML adjacent to its OFF mesh. Mesh
vertices in both exports use millimeters. The object export records
mass in grams.

## Choose the Mesh Collision Representation

By default, `mesh_collision="source"` exports the source mesh triangles,
including concavities. To reproduce the collision geometry of standard
MuJoCo mesh geoms, select the compiled hulls.

```python
hand = export_hand(model, "palm", "output/hand", name="my_hand",
                   mesh_collision="convex_hull")
object_xml = export_body(model, "object", "output/object",
                         mesh_collision="convex_hull")
```

For each mesh geom, let $V_g$ denote the vertices of its compiled hull
and $T_g$ its rigid transform into the export frame. The exported mesh
contains the boundary triangles of $T_g\,\operatorname{conv}(V_g)$
for each geom $g$. A link with two mesh geoms retains two independently
transformed hulls in its OFF file. The exporter preserves the compiler's
triangle connectivity and winding. Both modes use the same primitive
tessellation and affine joint map. Hull mode raises an error for SDF
or plugin colliders or when a mesh has no compiled hull.

## Prepare the Source Model

Each finger is a serial chain with one hinge on each moving body.
Fixed bodies are assigned to the nearest moving ancestor or to the
palm. The hand exporter transfers collision meshes and tessellates
boxes, spheres, capsules, cylinders, and ellipsoids. A geometry
participates if it has nonzero spatial extent and either `contype`
or `conaffinity` is nonzero. Supply participating geometry for the
palm and each articulated link.

The supported joint equalities express one driven hinge as an affine
function of one independent hinge. Use `preshape_joints` to assign
zero autograsp velocity to named independent coordinates, such as a
thumb opposition joint held at a prescribed setting.

For rigid objects, `export_body` expresses the geometry of the named
body and its descendants in the named body's frame at the reference
configuration. Pass `include_descendants=False` to select the named
body alone.

## Reconstruct the Hand in MuJoCo

The JSON manifest beside the hand XML lists independent coordinates
in `dof_joints`. Each record in `chains` identifies the source
hinge, the driving DOF index, the multiplier `ratio`, and the angular
offset `offset_rad`. Apply this map to the closed DOF vector of a
`GraspCandidate`. The operation preserves the other coordinates in
the MuJoCo state.

```python
qpos = data.qpos.copy()
for chain in hand.chains:
    for joint in chain:
        address = model.joint(joint.mujoco_joint).qposadr[0]
        qpos[address] = joint.ratio * candidate.dofs[joint.dof] + joint.offset_rad
```

`candidate.T_object_hand` maps palm coordinates into object
coordinates. The serialized key is `T_object_hand_mm`. Convert the
translation from millimeters to meters, then left-multiply the
transform by the source object's world pose. The resulting world-frame
palm pose is a target for the calling application's arm kinematics.

## Verify Link Geometry

The repository provides `examples/check_export_geometry.py` to evaluate
corresponding export and source link vertices in the palm frame. Supply
the source scene, palm body name, and a JSON list of joint vectors in
exported DOF order, in radians. The command below uses the Docker image
from [Installation](INSTALL.md) and writes the comparison to `output/parity/`.

```bash
mkdir -p output
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/input:/input:ro" -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python /src/examples/check_export_geometry.py \
    /input/scene.xml l6_hand_base_link /input/configurations.json /output/parity \
    --mesh-collision convex_hull
```

Use `--mesh-collision source` to compare source mesh vertices. The
`geometry-check.json` report records the mode, the maximum symmetric
Hausdorff vertex distance in millimeters for each link across the
configurations, the greatest requested-to-applied DOF discrepancy,
and the tolerance result.
