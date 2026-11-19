# Grasp Planning under Uncertainty

A candidate approach specifies a travel direction, palm roll, finger
preshape, and set of joints to close. For a given hand and object, the
contact planner enumerates these commands, simulates the approach and
finger closure, and ranks the admitted configurations by modeled
wrench capacity. The output has the contact geometry and scoring
samples needed to reproduce the ranking. The calling application
evaluates arm reachability and plans and executes the arm trajectory.

## Set the Hand Frame

The `hand` configuration defines a palm-frame approach axis, a second
axis along the fingers, and a grasp center. The approach axis points
toward the object. The planner projects the finger axis onto its
orthogonal plane to define roll, so the two input axes must be nonzero
and nonparallel. The grasp center $g$ is a hand-fixed point near the
region where the fingers close. If $R$ denotes the planned
palm-to-object rotation, $c$ the object center of mass, $d$ the unit
travel direction, and $s$ the standoff distance, the starting palm
translation is $c-sd-Rg$. Thus the grasp center begins $s$ millimeters
from $c$ along the approach line. The [RealHand L6
walkthrough](quickstart.md) shows the axes and coordinates for an
example model. Explicit preshapes can also be sampled from a
low-dimensional posture subspace before running the planner. See
{ref}`Ciocarlie and Allen <ref-ciocarlie-allen>`
for dexterous grasp synthesis over such subspaces.

## Specify Candidate Commands and Contact Constraints

The planner advances the open hand in discrete increments along $d$
until an increment would cause collision. It starts closure at the
last collision-free placement, with the requested clearance subtracted
from the distance traveled. GraspIt! advances the selected finger
coordinates under autograsp. The planner records closures with enough
object contacts and applies the configured contact-region predicate.
Support and keep-out predicates inspect the final nominal hand vertices.
This provides a geometric filter for the candidate bank. The calling
application verifies continuous hand and arm clearance during execution.

| Setting | Meaning |
|---|---|
| `approaches` | Explicit rows `[dx, dy, dz, roll_deg]` in the object frame. In their absence, `n_directions` gives the number of Fibonacci-sphere directions and `n_rolls` the number of equally spaced rolls per direction. |
| `preshapes` | Starting joint vectors in exported DOF order, in radians. Every direction and roll is evaluated with every specified preshape and closing set. |
| `closing_sets` | Lists of zero-based DOF indices. Coordinates outside the chosen list remain at preshape during closure. |
| `standoff_mm`, `approach_step_mm`, `clearance_mm` | Initial approach distance, travel increment, and retreat from the last collision-free placement, all in millimeters. |
| `min_contacts` | Minimum number of object contacts required at closure. |
| `support_normal`, `support_offset_mm` | Support half-space in the object frame. The planner normalizes the vector and admits a nominal closure if every hand vertex lies at least the specified offset along it. |
| `contact_region_mm` | Allowed contact box in object coordinates, given by its lower and upper corners as `[[xmin, ymin, zmin], [xmax, ymax, zmax]]`. |
| `keepout_boxes_mm` | Excluded boxes in object coordinates, each given by its center and half extents. The filter rejects a nominal closure if any hand vertex lies strictly inside an excluded box. |

## Select a Wrench Score

The default score uses Ferrari–Canny epsilon to quantify the worst
direction of resistance to a normalized wrench. For task-specific
synthesis, supply `task_forces_mm` as rows
`[px, py, pz, fx, fy, fz]`. The first three entries give the point of
application in the object frame in millimeters. The final entries
give the contact force direction required to balance the external
load. The planner normalizes each nonzero force vector before
constructing its wrench. For instance, `[0, 0, 0, 0, 0, 1]`
represents upward support through the object-frame origin. The score
is the greatest magnitude supported across the supplied directions.

## Model Approach and Friction Uncertainty

The planner closes the hand once at the nominal approach and again for
each of `n_pose_samples` perturbations. The parameters `pose_sigma_mm`
and `pose_sigma_deg` set the componentwise standard deviations of
isotropic Gaussian translation and rotation-vector errors. The
translation, rotation, and travel direction are expressed in the
object frame. The seed and command together determine the sampling
stream. Each friction coefficient in `friction_samples` is applied
to each realized contact set when constructing its wrench hull. The
friction grid has equal weights, and a failed perturbed approach or
closure contributes zero across its friction row.

The planner ranks candidates by the lower-tail average over the
lowest-scoring `tail_mass` fraction of pose–friction combinations.
This score favors capacity in weak scenarios even when competing
approaches have similar means. Set `tail_mass=1` for mean ranking.
Set `n_pose_samples=0` and provide one friction coefficient for
ranking at the specified nominal coefficient. The [theory guide](theory.md)
defines both quality measures and the empirical tail calculation.

## Interpret Candidate Records

The CLI orders the JSON records by decreasing `score` and writes the
nominal palm placement `T_object_hand_mm`, closed coordinates `dofs_rad`,
and per-contact positions and inward normals. The homogeneous transform
maps palm coordinates into the object frame. Position arrays and
transform translations use millimeters, and DOFs use radians. Preshape,
travel direction, roll, and closing indices identify the approach.

`epsilon_samples` and `task_samples` contain pose rows and friction
columns. The first row is nominal. `nominal_epsilon` and `nominal_task`
read the middle column of the nominal row, using the upper middle column for
an even grid. `expected_epsilon` and `expected_task` are means over
the corresponding arrays. `epsilon_cvar` stores the epsilon lower-tail
mean even in task mode, where `score` instead stores the task
lower-tail mean. `native_epsilon` comes from the simulator at the
nominal closure and uses the simulator's wrench normalization. The
top-level `rejections` field counts causes of nominal rejection.
See [MuJoCo model export](export.md) for converting the palm and joint
targets to the source model.

(generated-grasps)=
## Planned Grasps and Simulated Execution

````{container} hand-gallery
```{figure} figures/l6-insertion.png
:alt: Planned RealHand L6 closure on the key used for bimanual insertion.

**RealHand L6.** Planned key grasp, $\varepsilon=0.00460$.
```
```{figure} figures/leap-hold.png
:alt: Recorded MuJoCo configuration of the LEAP Hand holding a cube.

**LEAP Hand.** Simulated cube hold, $\varepsilon=0.0708$.
```
```{figure} figures/ability-hold.png
:alt: Recorded MuJoCo configuration of the PSYONIC Ability Hand holding a tapered cylinder.

**PSYONIC Ability Hand.** Simulated cylinder hold, $\varepsilon=0.167$.
```
````

Each $\varepsilon$ value is the Ferrari–Canny score returned by
GraspIt! for the planned closure. The L6 panel shows the planned
closure on the insertion key. The other panels show final states of
recorded lift-and-hold simulations in MuJoCo. In those panels the
score belongs to the GraspIt! plan used to initialize each execution.
