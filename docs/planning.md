# Plan grasps under uncertainty

The contact planner advances the hand along an approach, closes its
fingers, and computes grasp quality from the resulting object-frame
contacts. It returns ranked configurations for the input hand and
object. Arm reachability, arm collision checking, trajectory generation,
and execution belong to the application consuming these configurations.

## Run the planner

Use [the RealHand L6 setup](quickstart.md) to export the hand and cube and
run the planner. The CLI also accepts GraspIt! robot and object XML for
other hands in the [supported model class](hands.md).

(generated-grasps)=
## Generated grasps

````{container} hand-gallery
```{figure} figures/l6-insertion.png
:alt: Planned RealHand L6 key grasp from the bimanual insertion experiment.

**RealHand L6** · Planned key grasp, $\varepsilon=0.00460$.
```
```{figure} figures/leap-hold.png
:alt: Final MuJoCo cube hold with the LEAP Hand.

**LEAP Hand** · Final cube hold, $\varepsilon=0.0708$.
```
```{figure} figures/ability-hold.png
:alt: Final MuJoCo hold of a tapered cylinder with the PSYONIC Ability Hand.

**PSYONIC Ability** · Final cylinder hold, $\varepsilon=0.167$.
```
````

Ferrari–Canny $\varepsilon$ describes the planned GraspIt! closure in
each case. The L6 panel uses the native planned link poses. The LEAP and
Ability panels reconstruct recorded MuJoCo hold states from their
respective simulations. These three grasps come from the insertion and
lift studies.

See [geometry and grasp quality](theory.md) for wrench construction and
the lower-tail objective.

## Configure the search

The JSON `hand` object sets the approach axis, transverse finger axis,
and grasp center in the palm frame. The two axes must be nonzero and
nonparallel. They establish the direction of travel and the reference for
roll. Match these vectors to the hand's geometry when changing models.

The `planner` object controls the set of candidate commands and the
conditions for admitting a closure.

| Setting | Meaning |
|---|---|
| `approaches` | Rows of `[dx, dy, dz, roll_deg]` in the object frame. Omit to sample `n_directions` directions and `n_rolls` rolls. |
| `preshapes` | Open joint vectors in GraspIt! DOF order, in radians. |
| `closing_sets` | Zero-based DOF lists. A DOF outside the selected list holds its preshape during closure. |
| `standoff_mm`, `approach_step_mm`, `clearance_mm` | Search start, travel increment, and retreat from the contact boundary. |
| `min_contacts` | Minimum number of object contacts required after closure. |
| `support_normal`, `support_offset_mm` | Support half-space in the object frame. The normal points away from the support. A unit normal makes the offset a distance in millimeters. |
| `contact_region_mm` | Allowed contact box as `[[xmin, ymin, zmin], [xmax, ymax, zmax]]` in the object frame. |
| `keepout_boxes_mm` | Excluded boxes, each given by its center and half extents in the object frame. |

## Choose the score

The planner uses Ferrari–Canny epsilon by default. To evaluate specified
loads, provide `task_forces_mm` as rows `[px, py, pz, fx, fy, fz]`.
The point is expressed in the object frame in millimeters. The force
vector is the nonzero contact force direction required to balance the
external load. The planner normalizes each direction and computes the
largest common magnitude supported by the modeled contact wrench space.
For example, `[0, 0, 0, 0, 0, 1]` represents upward support at the
object origin.

Each candidate is closed at its nominal approach and after
`n_pose_samples` Gaussian perturbations of the approach. The
parameters `pose_sigma_mm` and `pose_sigma_deg` give the translation and
rotation-vector standard deviations, respectively. The candidate and
`seed` determine its random stream. The planner reevaluates the
wrench space at every `friction_samples` entry for every closure.
Failed closures contribute zero. Use a nonempty friction grid.

The ranking score is the average over the lowest `tail_mass` fraction
of the scores, with fractional weight on the boundary sample when
needed. `tail_mass=1` gives the mean. With zero pose samples and a
single friction coefficient, the score is the nominal quality at that
coefficient.

## Read the candidate file

The CLI returns candidates in descending order of `score`, with the
object-relative palm pose (`T_object_hand_mm`), closed finger coordinates
(`dofs_rad`), approach direction, roll, preshape, and contact geometry.
The score arrays use pose rows and friction columns. Row zero is the
nominal closure. The nominal fields use the middle grid entry, or the
upper middle entry for an even-sized grid. The `rejections` field counts
the causes for rejecting nominal candidates.

`native_epsilon` uses GraspIt!'s metric. `nominal_epsilon`,
`expected_epsilon`, and `epsilon_cvar` describe the Python wrench
scorer. When a task is set, `task_samples`, `nominal_task`, and
`expected_task` describe the task-wrench scores, and `score` is their
lower-tail mean. See [MuJoCo export](export.md) for mapping the planned
pose and joints to the source model.
