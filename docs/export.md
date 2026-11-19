# Export a MuJoCo hand and object

````{container} hand-gallery export-gallery
```{figure} figures/export-source-workcell.png
:alt: MuJoCo source workcell with two A7 arms and RealHand L6 hands during key insertion.

**MuJoCo source** · Recorded bimanual insertion
```
```{figure} figures/export-grasp-frames.png
:alt: Exported right RealHand L6 at the planned grasp with the key.

**GraspIt! export** · Right L6 in its planned key grasp
```
````

The panels illustrate the source workcell and exported hand at two
stages of the experiment. The numerical geometry check uses the same
joint coordinates in both simulators. Across 49 configurations and
12 links of the DexReach RealHand L6, its maximum symmetric Hausdorff
vertex discrepancy was 7.86 nm.

See [geometry and grasp quality](theory.md) for the reference-angle
conversion.

The exporter accepts a compiled MuJoCo model, including a model loaded
from MJCF or URDF. Choose the palm body as the root of the hand. The
portion of the model above the palm, such as a robot arm, stays in
MuJoCo.

```python
import mujoco

from graspit_python_wrapper.convert.mjcf_hand import export_body, export_hand

model = mujoco.MjModel.from_xml_path("scene.xml")
hand = export_hand(model, "palm", "output/hand", name="my_hand")
object_xml = export_body(model, "object", "output/object")
```

Load `hand.robot_xml` and `object_xml` in GraspIt! or pass them to the
[planner](planning.md). Keep the hand XML beside its `iv/` directory
and the object XML beside its OFF mesh.

## Supported models

Each finger is a serial chain with one hinge on each jointed body.
Jointless parts are merged into the nearest jointed ancestor or the palm.
The hand exporter copies participating collision meshes and tessellates
boxes, spheres, capsules, cylinders, and ellipsoids. Geometry with both
`contype` and `conaffinity` equal to zero is excluded. Supply collision
geometry for the palm and each articulated link.

A joint can be coupled to an independent hinge by an affine MuJoCo
joint equality. Nonlinear equalities and couplings driven by another dependent
joint fall outside the supported model. Use `preshape_joints` to give
specified independent joints zero autograsp velocity. The named
joints then hold at their preshape during closure.

The MuJoCo [joint equality reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-joint)
gives the source convention for affine couplings.

`export_body` expresses the geometry of the named body and its descendants
in the body's frame at the reference configuration. Use this operation
for a rigid object. Pass `include_descendants=False` to export only the
named body. Mesh vertices are stored in millimeters and mass in grams.

## Recover the MuJoCo joint values

The exporter writes a JSON file alongside the hand XML. Its `dof_joints`
field gives the independent coordinates in GraspIt! order. Each joint
record in `chains` gives the MuJoCo joint name, the driving DOF index,
the scale `ratio`, and the angular offset `offset_rad`. GraspIt! DOF
values and MuJoCo hinge coordinates are in radians.

For driven hinge $y$ and driver $x$, MuJoCo expresses an affine
equality as $y-y_0=a_0+a_1(x-x_0)$, where $x_0$ and $y_0$ are reference
positions from `qpos0`. The exported ratio is $a_1$ and the offset in
absolute coordinates is $y_0+a_0-a_1x_0$. Apply the map to a selected
candidate as follows:

```python
qpos = data.qpos.copy()
for chain in hand.chains:
    for joint in chain:
        address = model.joint(joint.mujoco_joint).qposadr[0]
        qpos[address] = joint.ratio * candidate.dofs[joint.dof] + joint.offset_rad
```

The candidate's `T_object_hand` maps hand-frame coordinates into the
object frame. Its translation is in millimeters. Divide that translation
by 1000 before composing it with a MuJoCo object pose. Use the resulting
target palm pose and joint values in your arm kinematics or controller.

## Compare link geometry

Inside the reference container, the geometry comparison measures the
maximum symmetric Hausdorff distance between corresponding palm-frame
link vertices. Supply a list of configurations with entries in exported
DOF order. The command below uses the scene and configuration file
mounted under `input/`. Replace `l6_hand_base_link` with your palm body
name when using a different model.

```bash
mkdir -p output
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/input:/input:ro" -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python /src/examples/check_export_geometry.py \
    /input/scene.xml l6_hand_base_link /input/configurations.json /output/parity
```

The command records per-link errors in millimeters, the largest difference
between requested and applied DOF values, and the tolerance result in
`output/parity/geometry-check.json`.
