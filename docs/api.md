# Python API

The public import namespace is `graspit_py`. Existing
`graspit_python_wrapper` imports remain supported and resolve to the
same modules, classes, and native world state. Numerical utilities
operate on arrays or exported XML; the MuJoCo exporter operates on a
compiled model. Native bindings provide the GraspIt! scene, closure,
contact, and quality operations used by the contact planner.

Importing a Python module and executing a native operation have
different requirements. The source checkout supports the following
uses without building the native extension:

| Use | Requirements and Execution Boundary |
|---|---|
| Import and run numerical utilities | NumPy and SciPy; `risk`, `kinematics`, and `grasp.quality` do not require GraspIt!. |
| Import `utils` | No native extension; URDF conversion execution additionally requires `yourdfpy` and `trimesh`, provided by the `urdf` extra. |
| Import planning classes or display CLI help | No native extension; native closure and planning execution require GraspIt!. |
| Export a compiled MuJoCo model | The `mjcf` extra, which supplies `mujoco>=3.1`; export alone does not require GraspIt!. |
| Check an export against native geometry | MuJoCo and the native GraspIt! installation. |

This source-import boundary does not change the package build:
`pip install` builds the native extension. See [Installation](INSTALL.md)
for the native setup. GraspIt! has one main world per process; use a
process per independent native scene. The documented Docker examples
use a virtual X display for headless native execution.

## Native Bindings

### Scene Setup

Call `initialize()` before constructing `World()`. The public Python
constructor initializes its world handle; an additional `world.init()`
is optional and idempotent.

```python
from graspit_py.core.bindings import Grasp, World, initialize

initialize()
world = World()
```

| Operation | API |
|---|---|
| Load a hand | `world.load_robot(filename)` |
| Load an object | `world.load_graspable_body(filename)` |
| Remove loaded robots and bodies | `world.clear()` |
| Check world interpenetration | `world.no_collision()` |

### Hand State

Pose dictionaries provide a scalar-first quaternion `quat_wxyz`, a
translation in millimeters, a row-major rotation matrix, and a
homogeneous transform. Joint coordinates are in radians.

| Operation | API |
|---|---|
| Read hand pose | `world.get_current_hand_pose()` |
| Set hand pose | `world.set_current_hand_pose(quat_wxyz, translation_mm)` |
| Read independent coordinates | `world.get_current_hand_dof_values()` |
| Set independent coordinates | `world.set_current_hand_dof_values(dof_values)` |
| Read object pose | `world.get_graspable_body_pose(graspable_body_index)` |
| Close the hand | `world.auto_grasp(render_it=False, speed_factor=1.0, stop_at_contact=False)` |

For static native closure, `stop_at_contact=True` ends all movement at
the first detected contact. With `False`, movable DOFs continue after
another DOF stops, until their desired values, limits, or contacts
prevent further motion. Collision checking remains active. The native
dynamic branch sets desired joint values or velocities and does not
use this static first-contact termination rule.

### Contacts and Quality

After placing and closing the hand, construct
`Grasp(world, graspable_body_index=0)` and call `compute()`.

| Operation | API and Convention |
|---|---|
| Compute the grasp | `grasp.compute()` |
| Read contact geometry | `grasp.object_contact_geometry()`: object-frame points in millimeters and inward normals. |
| Read native wrench-space metrics | `grasp.epsilon_quality()`, `grasp.volume_quality()` |
| Check grasp validity | `grasp.is_non_penetrating()`: validity under GraspIt!'s grasp criteria. |

Use `world.no_collision()` for world interpenetration. It does not test
force closure. The native quality values depend on the hand/object
model and its wrench-space settings.

### Geometry

Link poses and vertices are expressed in the world frame. Use chain
`-1` for the palm.

| Operation | API |
|---|---|
| Read link poses | `world.get_link_poses()` |
| Read link vertices | `world.get_link_vertices_world(chain, link)` |

## Contact Planner

Import `ContactPlanner`, `HandFrame`, and `PlannerConfig` from
`graspit_py.planning`. Load and place the hand and object before
constructing the planner: it captures the current object pose and
center of mass on construction. This example uses an initialized,
loaded `world` and explicitly chosen sampling settings.

```python
from graspit_py.planning import ContactPlanner, HandFrame, PlannerConfig

frame = HandFrame(
    approach_axis=(0, 0, 1),
    finger_axis=(1, 0, 0),
    grasp_center_mm=(0, 0, 65),
)
config = PlannerConfig(
    approaches=[(0, 0, 1, 0), (0, 0, 1, 45), (0, 0, 1, 90)],
    standoff_mm=80,
    pose_sigma_mm=0.25,
    pose_sigma_deg=0.5,
    n_pose_samples=2,
    friction_samples=[0.3, 0.6, 0.9],
    tail_mass=0.5,
)
candidates = ContactPlanner(
    world, body_index=0, config=config, hand=frame
).plan(top_k=2)
```

Each `GraspCandidate` contains the nominal hand pose, closed independent
coordinates, contacts, and sampled scores. In memory, the pose and
coordinate attributes are `T_object_hand` and `dofs`; `to_dict()` writes
them as `T_object_hand_mm` and `dofs_rad`. Contacts serialize as
`position_mm` and `normal`. Feasible candidates are sorted by descending
score: epsilon lower-tail quality without a task, and task lower-tail
quality in task mode. The [planning guide](planning.md) defines the
settings and returned arrays.

## MuJoCo Exporter

Install the `mjcf` extra or `mujoco>=3.1`, then import from
`graspit_py.convert.mjcf_hand`.

| Function | Result |
|---|---|
| `export_hand(model, palm_body, out_dir, ...)` | `ExportedHand` with `robot_xml`, `dof_joints`, `chains`, and `links`. |
| `export_body(model, body, out_dir, ...)` | Path to the GraspIt! object XML. |
| `dofs_from_qpos(hand, model, qpos)` | Independent MuJoCo hinge coordinates in GraspIt! order. |

Both export functions accept `mesh_collision="convex_hull"` to export
each mesh geom's compiled MuJoCo collision hull. The default,
`mesh_collision="source"`, exports the source mesh triangles. Exporting
requires no native GraspIt! execution. The
[export guide](export.md) documents the files, manifest, geometric
comparison, and reconstruction into the source scene.

## Numerical Utilities

These modules can be imported from the source checkout without a native
GraspIt! build when NumPy and SciPy are available.

- `graspit_py.risk.lower_tail_mean(values, alpha=..., weights=...)`
  averages the smallest `alpha` probability mass of a quality
  distribution. `risk.cvar(costs, alpha=..., weights=...)` averages the
  largest `alpha` mass of a cost distribution. Optional weights are
  normalized to sum to one.
- `graspit_py.kinematics.load_chains(robot_xml)` reads
  Denavit–Hartenberg chains. `link_poses(chains, dofs)` returns
  palm-frame transforms keyed by `(chain, link)`.
- `graspit_py.grasp.quality` provides `build_grasp_matrix()`,
  `analyze_gws()`, `compute_task_wrench_quality()`, and
  `min_weight_force_closure()`.

With SciPy, `min_weight_force_closure()` returns the LP optimum and a
Boolean that requires a positive margin above tolerance and
wrench-matrix rank six. A positive margin in a lower-dimensional wrench
subspace is retained with a false force-closure predicate. Without
SciPy, the epsilon and min-weight entry points return a
minimum-singular-value proxy and its threshold indicator. Use SciPy
for hull and LP force-closure analysis.

The grasp matrix stores force components first and normalized torque
components second. Supply the same moment origin and torque scale for
contact wrench generators and required task wrenches.

### Compute Quality from Prescribed Contacts

This example places six synthetic contacts at the face centers of a
cube with 25 mm half-width and computes force closure and epsilon with NumPy
and SciPy. Run it from a source checkout using the numerical environment
described in [Installation](INSTALL.md). It needs no native simulator.

```{literalinclude} examples/numerical_quality.py
:language: python
```

```bash
PYTHONPATH=. .venv-numerics/bin/python docs/examples/numerical_quality.py
```

Expected output:

```text
G shape: (6, 48)
min-weight: 0.020833; force closure: True
epsilon: 0.408248
```

The [theory guide](theory.md#force-closure-and-ferraricanny-epsilon)
derives the wrench normalization and the two closure criteria.
