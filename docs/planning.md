# Grasp Planning Under Uncertainty

A candidate approach specifies travel direction, palm roll, finger
preshape, and joints to close. The contact planner enumerates these
commands, simulates approach and closure, and ranks admitted candidates
by modeled wrench capacity under pose and friction uncertainty. Records
include nominal contact geometry and scoring samples. The calling
application evaluates arm reachability and plans and executes the arm
trajectory.

## Run the Planner

The CLI requires the native GraspIt! library, an exported hand XML, and
an object XML with their matching assets. For example, a hand exported
as `my_hand` and an object exported as `object` can use the paths below.
Save this configuration as `plan.json`, replacing the `hand` values with
the calibration for your model. The hand-frame values shown are the
dataclass defaults; the planner settings select a nominal example.
The [RealHand L6 walkthrough](quickstart.md) supplies a concrete hand
calibration and Docker commands.

```json
{
  "hand": {
    "approach_axis": [1.0, 0.0, 0.0],
    "finger_axis": [0.0, 0.0, 1.0],
    "grasp_center_mm": [40.0, 10.0, 93.0]
  },
  "planner": {
    "n_pose_samples": 0,
    "friction_samples": [0.5],
    "tail_mass": 1.0
  }
}
```

```bash
xvfb-run -a .venv/bin/python -I -m graspit_py.planning \
  --robot output/hand/my_hand.xml \
  --object output/object/object.xml \
  --config plan.json \
  --out grasps.json
```

This configuration selects nominal scoring at the supplied friction
coefficient. Omitting those planner overrides uses eight perturbed poses,
nine friction nodes, and tail mass 0.2. `--robot`, `--object`, and `--out`
are required; `--config` and integer `--top-k` are optional. Without
`--top-k`, the CLI writes all admitted candidates. A positive value
retains up to that many of the highest-ranked records; zero retains none.
Use a nonnegative value for `--top-k`. This command assumes the native
environment described in [Installation](INSTALL.md); `-I` selects the
installed package instead of an unbuilt source copy.

The CLI constructs `HandFrame` and `PlannerConfig` directly from the
optional `hand` and `planner` JSON objects. Unknown nested fields raise
`TypeError`. Type annotations do not validate values, and the dataclasses
do not enforce all ranges. The tables give meaningful input requirements;
they identify additional runtime checks where present.

## Set the Hand Frame

The approach axis points toward the object in the palm frame. The
planner projects the finger axis onto the plane perpendicular to it to
define roll; both axes must be nonzero and nonparallel. The grasp center
$g$ is a hand-fixed point near the closing region. Let $R$ map palm
coordinates into the object frame, $c$ be the object center of mass,
$d$ the unit travel direction, and $s$ the standoff in millimeters. The
starting palm translation is

$$
t=c-sd-Rg.
$$

Thus the grasp center starts at $Rg+t=c-sd$, a distance $s$ from $c$.
Roll rotates the hand about $d$; zero roll depends on the planner's
reference-axis construction.

```{figure} figures/approach-geometry.svg
:alt: Starting palm origin and grasp center with travel direction toward the object center and standoff measured from the grasp center.
:target: _images/approach-geometry.svg

Approach geometry in the object frame. Standoff $s$ spans the starting
grasp center to $c$ along $d$; the palm origin is at $c-sd-Rg$.
```

| `hand` Field | Default | Type and Units | Valid Input and Meaning |
|---|---|---|---|
| `approach_axis` | `[1, 0, 0]` | Length-three vector | Nonzero, palm-frame approach direction. |
| `finger_axis` | `[0, 0, 1]` | Length-three vector | Nonzero and nonparallel to approach axis; defines roll. |
| `grasp_center_mm` | `[40, 10, 93]` | Length-three vector, mm | Palm-frame grasp center; calibrate for the hand. |

## Specify Candidate Commands

Without explicit approaches, the planner samples Fibonacci-sphere
travel directions and equally spaced rolls. Every approach is combined
with each preshape and closing set. Preshapes use exported DOF order;
coordinates outside the selected closing set remain at preshape during
closure. Explicit preshapes can come from a low-dimensional posture
subspace; see {ref}`Ciocarlie and Allen <ref-ciocarlie-allen>`.

| `planner` Field | Default | Type | Valid Input and Meaning |
|---|---|---|---|
| `n_directions` | `48` | Integer | Positive direction count; used without `approaches`. |
| `n_rolls` | `4` | Integer | Positive roll count; rolls are equally spaced over 360 degrees. |
| `approaches` | `null` | Array `(A,4)` | Rows `[dx, dy, dz, roll_deg]`, object frame, nonzero direction. Replaces generated directions/rolls; empty array gives no candidates. |
| `preshapes` | `null` | Array `(S,D)`, rad | Starting joint vectors of exported DOF length $D$, within model limits. Null or empty uses `open_dofs`, which defaults to zeros. |
| `closing_sets` | `null` | Lists of integer indices | Zero-based DOF indices within bounds; null closes all. `[]` enumerates no candidates; `[[]]` specifies no moving DOFs. |

## Approach and Closure

The planner advances the open hand along $d$ in discrete increments,
searching up to twice the initial standoff. When an increment would
collide, it starts closure at the last collision-free travel distance
minus clearance, clamped to nonnegative travel. GraspIt! advances the
selected coordinates under autograsp. The planner admits closures with
enough object contacts and applies the constraints below. The calling
application verifies continuous hand and arm clearance during execution.

| `planner` Field | Default | Type and Units | Valid Input and Meaning |
|---|---|---|---|
| `standoff_mm` | `120.0` | Float, mm | Positive starting distance from grasp center to object center. |
| `approach_step_mm` | `2.0` | Float, mm | Positive travel increment; zero or negative can prevent termination. |
| `clearance_mm` | `2.0` | Float, mm | Nonnegative retreat from last-free placement. |
| `min_contacts` | `3` | Integer | Nonnegative minimum contact count for admission. |
| `autograsp_speed` | `0.8` | Float, speed factor | Passed as GraspIt! `speed_factor`; Python imposes no range check or physical velocity units. |

## Contact and Environment Constraints

The contact-box and minimum-contact predicates run in every closure,
including perturbed trials. Support and keep-out predicates inspect
final nominal hand vertices. The support test includes its boundary;
the keep-out test rejects strictly interior vertices. These are
geometric filters on the candidate bank.

| `planner` Field | Default | Type and Units | Valid Input and Meaning |
|---|---|---|---|
| `support_normal` | `null` | Length-three vector | Null disables support filtering; otherwise supply a nonzero object-frame normal. |
| `support_offset_mm` | `0.0` | Float, mm | Every nominal hand vertex $v$ must satisfy $v^\top\hat n\ge\text{offset}$ for normalized support normal $\hat n$. |
| `contact_region_mm` | `null` | Array `(2,3)`, mm | Ordered lower/upper object-frame corners; every contact must lie inside or on the box. |
| `keepout_boxes_mm` | `null` | Array `(B,2,3)`, mm | Each box is `[center, half_extents]` in object coordinates, with positive half extents. |

## Select a Wrench Score

The default quality is Ferrari–Canny epsilon: the radius of an
origin-centered ball in the normalized wrench hull. Its moment radius
is the maximum contact distance from the center of mass, with a 0.01 mm
floor. Task loading instead uses the largest common multiplier supported
along specified load rays, with a 100 mm moment radius for both hull
and rays. The [theory guide](theory.md) defines the force budget,
normalization, and strict-interior requirement for task capacity.

| `planner` Field | Default | Type and Units | Valid Input and Meaning |
|---|---|---|---|
| `task_forces_mm` | `null` | Array `(L,6)` | Rows `[px, py, pz, fx, fy, fz]`: object-frame point in mm and nonzero balancing-force direction. Null or empty uses epsilon; a nonempty array enables task scoring. |

The planner normalizes each supplied force direction. For downward
gravity, `[0, 0, 0, 0, 0, 1]` specifies upward support through the
object-frame origin. Its moment about the center of mass depends on the
line of application. Load multipliers use the modeled normal-force
budget; calibrated physical forces are needed to express them in newtons.

## Model Approach and Friction Uncertainty

The planner closes at the nominal approach and at each perturbed pose.
Translation and rotation-vector errors are componentwise isotropic
Gaussian draws in the object frame. The perturbation rotation
left-multiplies the palm rotation and rotates the travel direction.
The seed and command determine each candidate's stream independently
of enumeration history.

| `planner` Field | Default | Type and Units | Valid Input and Meaning |
|---|---|---|---|
| `n_pose_samples` | `8` | Integer | Nonnegative perturbed-pose count; nominal pose is additional. |
| `pose_sigma_mm` | `3.0` | Float, mm/component | Nonnegative translation standard deviation. |
| `pose_sigma_deg` | `3.0` | Float, degrees/component | Nonnegative rotation-vector standard deviation. |
| `friction_samples` | `linspace(0.2, 1.0, 9)` | Sequence of floats | Nonempty finite nonnegative grid; supplied order retained. Equal weights, no planner clipping. |
| `tail_mass` | `0.2` | Float | $0<\text{tail\_mass}\le1$; checked by the risk function when scoring. |
| `seed` | `0` | Integer | Integer seed accepted by NumPy's random generator. |

Every realized contact set is scored at every friction coefficient.
A failed perturbed approach or closure contributes a zero row. The
planner ranks by the lower-tail average of all pose–friction entries,
including the nominal row with equal weight. This emphasizes adverse
low-capacity outcomes. Set `tail_mass=1` for mean ranking. Nominal
scoring requires both `n_pose_samples=0` and a single friction coefficient;
the configuration above selects that case. The [theory guide](theory.md)
includes an executed fractional-tail example.

## Interpret Candidate Records

The CLI sorts candidates by decreasing `score`. Let $D$ be the exported
DOF count, $N$ the nominal contact count, $P=\text{n\_pose\_samples}+1$,
and $F$ the friction-grid length. Sample arrays have pose rows and
friction columns: the first row is nominal, and columns retain supplied
order. The nominal scalar uses column `F//2`, the upper middle column
for an even-length grid. It need not correspond to the mean friction
coefficient. Admission does not require a positive score.

| Candidate JSON Field | Shape or Type | Meaning and Units |
|---|---|---|
| `T_object_hand_mm` | `(4,4)` | Accepted nominal palm-to-object transform; translation mm. Python attribute: `T_object_hand`. |
| `dofs_rad` | `(D,)` | Nominal closed coordinates, rad, exported/native DOF order. Python attribute: `dofs`. |
| `contacts` | $N$ objects | Each has `position_mm` `(3,)` and inward unit `normal` `(3,)`, object frame; no forces. |
| `native_epsilon` | Float | Native simulator epsilon at nominal closure, using simulator normalization. |
| `nominal_epsilon` | Float | `epsilon_samples[0,F//2]`. |
| `epsilon_samples` | `(P,F)` | Normalized Python epsilon values; failed perturbed rows are zero. |
| `score` | Float | Selected epsilon or task lower-tail mean; ranking scalar. |
| `expected_epsilon` | Float | Arithmetic mean of all epsilon entries. |
| `approach_dir` | `(3,)` | Unit travel direction, object frame. |
| `roll_deg` | Float, degrees | Roll about travel direction. |
| `preshape` | `(D,)`, rad | Starting command coordinates. |
| `closing` | List of integers or null | Zero-based closing indices; null means all DOFs. |
| `epsilon_cvar` | Float | Epsilon lower-tail mean, populated also in task mode. |
| `task_samples` | `(P,F)` or null | Task-load multipliers; null without task loading. |
| `nominal_task` | Float or null | `task_samples[0,F//2]`, or null. |
| `expected_task` | Float or null | Arithmetic mean of task entries, or null. |

The top-level JSON contains these fields:

| Field | Interpretation |
|---|---|
| `robot`, `object` | Input XML paths. |
| `hand`, `planner` | Hand-frame and planner settings. |
| `seconds` | Elapsed runtime observed by the CLI, in seconds. |
| `rejections` | Counts of nominal rejection causes; only incremented keys appear. |
| `units` | `{"length":"mm", "angle":"rad", "quaternion":"wxyz"}`; `_deg` fields explicitly use degrees. |
| `candidates` | Ranked candidate records described above. |

Rejection keys are `approach_blocked_or_missed`,
`closure_start_collides`, `grasp_compute_failed`, `too_few_contacts`,
`contact_outside_region`, `below_support`, and `in_keepout`.
Perturbed failures affect scoring arrays, not these counts.
`native_epsilon` and Python epsilon use distinct normalizations.
See [MuJoCo Model Export](export.md) for converting palm and joint
targets back to the source model.

The following abbreviated record comes from the
[L6 reference workflow](https://github.com/coenwerem/graspit-py/tree/main/examples/l6_workflow).
Values are rounded for display. Its scores belong to that workflow's
model and sampling settings, not to the nominal configuration above.

```json
{
  "dofs_rad": [
    0.5847,
    0.45,
    0.612,
    0.566,
    0.641,
    0.712
  ],
  "native_epsilon": 0.122575,
  "nominal_epsilon": 0.064501,
  "score": 0.00895771
}
```

`native_epsilon` and `nominal_epsilon` report the native and Python
quality metrics under their respective wrench conventions. `score` averages the adverse tail of the sampled
Python quality values. The full record also carries the pose, contacts,
and sample arrays listed above.

(generated-grasps)=
## Planned Grasps and Simulated Execution

````{container} hand-gallery
```{figure} figures/l6-insertion.png
:alt: Planned RealHand L6 closure on the key used for bimanual insertion.

**RealHand L6.** Planned key grasp.
```
```{figure} figures/leap-hold.png
:alt: Recorded MuJoCo configuration of the LEAP Hand holding a cube.

**LEAP Hand.** Simulated cube hold.
```
```{figure} figures/ability-hold.png
:alt: Recorded MuJoCo configuration of the PSYONIC Ability Hand holding a sphere.

**PSYONIC Ability Hand.** Simulated sphere hold.
```
````

The L6 panel shows the planned closure on the insertion key. The other
panels show final states of recorded lift-and-hold simulations in MuJoCo,
initialized from GraspIt! plans.
