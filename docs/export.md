# Export a MuJoCo hand and object

```{figure} figures/mujoco-export.png
:alt: Side-by-side views of the MuJoCo source gripper collision geometry and the exported gripper loaded by GraspIt at equal joint coordinates.

(a) MuJoCo collision geometry. (b) Exported OFF geometry posed using
native GraspIt! link transforms at the same joint coordinates. The
[geometry checker](https://github.com/coenwerem/graspit-python-wrapper/blob/main/examples/check_export_geometry.py)
compares corresponding vertex sets across three configurations and seven
links. The maximum symmetric Hausdorff distance for this example is
0.0144 mm.
```

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
DOF order. The bundled example uses three configurations of a three-finger
hand:

```bash
docker run --rm graspit-py xvfb-run -a python /src/examples/check_export_geometry.py \
  /src/tests/data/gripper.xml palm /src/examples/parity-test-configs.json /tmp/parity
```

The command prints the maximum vertex discrepancy in millimeters by link,
the largest change between requested and applied DOF values, and the
tolerance result. Use a host mount for the output path to keep the JSON
report after the container exits.

## Regenerate the figures

After running the [first-grasp](quickstart.md) and [planning](planning.md)
examples, capture the source and exported meshes and the two saved
candidate poses. From the repository root:

```bash
docker buildx build --load -f docker/Dockerfile -t graspit-py .
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python /src/docs/examples/render_evidence.py capture \
  /output/first-grasp /output/geometry.npz
python docs/examples/render_evidence.py render \
  output/geometry.npz docs/figures
```

The render step uses NumPy and Matplotlib from the local Python
environment. The capture step queries the native link poses from
GraspIt! and writes the collision triangles in millimeters.
