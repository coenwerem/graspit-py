# Python API

The native bindings expose simulator operations. The contact planner
uses those operations to generate and rank approach candidates. The
MuJoCo exporter operates on a compiled MuJoCo model, and the numerical
utilities operate on arrays or exported XML. GraspIt! has one main
world per process. Use a process per independent native scene and a
virtual X display for headless execution.

## Native Bindings

Import `graspit_python_wrapper.core.bindings` and call `initialize()`.
Construct `World()` and call `world.init()` before loading geometry.
Call `world.clear()` to remove the loaded robots and bodies.

| Operation | API |
|---|---|
| Load geometry | `World.load_robot(filename)`, `World.load_graspable_body(filename)` |
| Read or set hand pose | `World.get_current_hand_pose()`, `World.set_current_hand_pose(quat_wxyz, translation_mm)` |
| Read or set finger coordinates | `World.get_current_hand_dof_values()`, `World.set_current_hand_dof_values(dof_values)` |
| Read object pose | `World.get_graspable_body_pose(graspable_body_index)` |
| Close the hand | `World.auto_grasp(render_it=False, speed_factor=1.0, stop_at_contact=False)` |
| Check interpenetration | `World.no_collision()` |
| Query link geometry | `World.get_link_poses()`, `World.get_link_vertices_world(chain, link)` |

Pose dictionaries provide a scalar-first quaternion `quat_wxyz`,
translation in millimeters, row-major rotation matrix, and homogeneous
transform. Joint coordinates are in radians. Link poses and vertices are
expressed in the world frame. Use chain `-1` for the palm.

Construct `Grasp(world, graspable_body_index=0)` and call `compute()`
after placing and closing the hand. Read `object_contact_geometry()` for
points in millimeters and inward normals in the object frame.
`epsilon_quality()` and `volume_quality()` return the native GraspIt!
wrench-space metrics. `Grasp.is_non_penetrating()` reports the validity
of the grasp under GraspIt!'s grasp criteria. To query collision in the
world, call `World.no_collision()`.

## Contact Planner

Import `ContactPlanner`, `HandFrame`, and `PlannerConfig` from
`graspit_python_wrapper.planning`. Set the hand and object in the
world before constructing the planner. The planner captures the current
object pose and center of mass on construction.

```python
from graspit_python_wrapper.planning import (
    ContactPlanner,
    HandFrame,
    PlannerConfig,
)

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
candidates = ContactPlanner(world, body_index=0, config=config, hand=frame).plan(top_k=2)
```

Each `GraspCandidate` contains the nominal hand pose, closed
coordinates, contacts, and sampled scores. `to_dict()` returns the
representation written by the CLI. The [planning guide](planning.md)
defines the settings and returned arrays.

## MuJoCo Exporter

Install the `mjcf` extra or `mujoco>=3.1`, then import from
`graspit_python_wrapper.convert.mjcf_hand`.

| Function | Result |
|---|---|
| `export_hand(model, palm_body, out_dir, ...)` | `ExportedHand` with the robot XML path, independent DOFs, affine joint map, and link correspondence. |
| `export_body(model, body, out_dir, ...)` | Path to the GraspIt! object XML. |
| `dofs_from_qpos(hand, model, qpos)` | Independent MuJoCo hinge coordinates in GraspIt! order. |

Both export functions accept `mesh_collision="convex_hull"` to export
each mesh geom's compiled MuJoCo collision hull. The default,
`mesh_collision="source"`, exports the source mesh triangles. See
[MuJoCo Model Export](export.md) for the collision representation and
geometry comparison workflow.

## Numerical Utilities

These modules can be imported from the source checkout without a native
GraspIt! build when NumPy and SciPy are available.

- `graspit_python_wrapper.risk.lower_tail_mean(values, alpha=..., weights=...)`
  averages the smallest `alpha` probability mass of a quality distribution.
  `risk.cvar(costs, alpha=..., weights=...)` averages the largest `alpha`
  mass of a cost distribution. Optional weights are normalized to sum to one.
- `graspit_python_wrapper.kinematics.load_chains(robot_xml)` reads Denavit–Hartenberg
  chains. `link_poses(chains, dofs)` returns palm-frame transforms keyed
  by `(chain, link)`.
- `graspit_python_wrapper.grasp.quality` provides
  `build_grasp_matrix()`, `analyze_gws()`, and
  `compute_task_wrench_quality()`, and `min_weight_force_closure()`.
  With SciPy, the min-weight routine returns the LP optimum and a Boolean
  requiring a positive margin above tolerance and wrench-matrix rank six.
  A positive margin in a lower-dimensional wrench subspace is retained
  with a false force-closure predicate. Without SciPy, the epsilon and
  min-weight entry points return a minimum-singular-value proxy and its
  threshold indicator. Use SciPy for hull and LP force-closure analysis.
  The grasp matrix stores
  force components first and normalized torque components second.
  Supply the same moment origin and torque scale for the contact wrench
  generators and the required task wrenches.
