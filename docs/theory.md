# Kinematics, Contact Mechanics, and Grasp Quality

The exporter preserves the link geometry used to generate contacts. The
planner maps each resulting contact set to the forces and moments it
can exert on the object, then ranks approach commands by their capacity
under pose and friction uncertainty. This guide defines the kinematics,
wrench scores, and lower-tail calculation. The [planning guide](planning.md)
provides the corresponding configuration and output fields.

## Notation and Implementation

Contact and load quantities use the object frame. Hand geometry uses the
palm frame until transformed by the object-relative palm pose.

| Symbol | Meaning | API or Record Correspondence |
|---|---|---|
| $q_i,q_i^0$ | Joint angle and reference angle, radians | Exported joint coordinates; `link_poses` accepts independent coordinates in radians |
| $u_j$ | Independent driver coordinate, radians | Exported DOF order; `dofs_rad` stores the closed coordinates |
| $p_i,n_i$ | Contact position in millimeters and inward unit normal | `contacts[i].position_mm`, `contacts[i].normal`; contact arrays supplied to `build_grasp_matrix` |
| $c$ | Object center of mass, millimeters | Center argument of `build_grasp_matrix`; reference point for moments |
| $\mu$ | Friction coefficient | `friction_coef` in quality APIs; columns of `planner.friction_samples` |
| $t_{i1},t_{i2},s_{ij}$ | Tangent basis and polygon directions | Constructed inside `build_grasp_matrix` |
| $\rho$ | Positive moment-normalization length, millimeters | `max_radius` in `build_grasp_matrix`; planner conventions below |
| $f_{ij},w_{ij}$ | Edge force and normalized primitive wrench | Edge forces define the wrench columns returned by `build_grasp_matrix` |
| $G,W$ | Primitive-wrench matrix and its convex hull | Input to `compute_gws_epsilon`, `min_weight_force_closure`, and task scoring |
| $\varepsilon$ | Ferrari–Canny radius | `epsilon_samples`, `nominal_epsilon`, `epsilon_cvar` |
| $p_k,f_k,\ell_k$ | Task application point, balancing-force direction, and normalized load ray | Rows of `planner.task_forces_mm`; task scorer input |
| $c_0,c_1$ | Constant and linear coefficient of a joint equality | MuJoCo equality coefficients; distinct from center of mass $c$ |
| $f_i$ | Physical contact force, with normal and tangential components | Underlying contact model; `contacts` stores geometry, not forces |
| $Q$ | Common supported task-load multiplier | `task_samples`, `nominal_task`; task-mode `score` is its lower-tail mean |
| $Z,\beta$ | Quality reward and adverse-tail probability mass | Flattened quality samples; `planner.tail_mass`, called `alpha` by `lower_tail_mean` |

## Joint Coordinates and Link Geometry

A hinge constrains the relative motion of two bodies to rotation about
one axis. A serial finger chain composes these motions from the palm
to the fingertip. Under the standard Denavit–Hartenberg convention,
the transform from link frame $i$ to its predecessor is

$$
{}^{i-1}T_i(q_i)=
R_z\bigl(\theta_i^0+q_i-q_i^0\bigr)T_z(d_i)T_x(a_i)R_x(\gamma_i).
$$

Here $\theta_i^0$ is the DH angle at reference, $d_i$ is the hinge-axis
offset, $a_i$ is the common-normal length, and $\gamma_i$ is the twist.
The exporter constructs DH frames from the compiled hinge axes at
reference and transforms collision meshes into their link frames.
Geometry rigidly attached to a link shares its transform. GraspIt! XML
stores angular constants in degrees and mesh coordinates in millimeters.
The Python `link_poses` API takes independent coordinates in radians and
returns palm-frame transforms indexed by `(chain_index, link_index)`.

An affine MuJoCo equality can couple a driven hinge $i$ to an independent
hinge $j$. For driver coordinate $u_j=q_j$, the reference-coordinate
constraint is

$$
q_i-q_i^0=c_0+c_1(q_j-q_j^0).
$$

Its absolute-coordinate reconstruction is

$$
q_i=c_1u_j+b_i,
$$

$$
b_i=q_i^0+c_0-c_1q_j^0.
$$

For $q_j^0=0.35$ rad, $q_i^0=-0.20$ rad, $c_0=0.16$ rad, and
$c_1=0.70$, the manifest offset is $b_i=-0.285$ rad. The manifest stores
the absolute-coordinate map in radians. The source convention is defined
in the [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-joint).

## Frictional Point Contacts

A contact can push inward and resist tangential slip through friction.
For contact force $f_i\in\mathbb R^3$, its normal component is
$f_{n,i}=n_i^\top f_i$ and its tangential component is
$f_{t,i}=f_i-f_{n,i}n_i$. The unilateral Coulomb conditions are

$$
f_{n,i}\ge0,
$$

$$
\lVert f_{t,i}\rVert_2\le\mu f_{n,i}.
$$

At each contact, the force has three components: one normal to the
surface and two tangential components spanning the contact plane.
The moment $(p_i-c)\times f_i$ about $c$ is determined by this force
and its lever arm. See
{ref}`Murray, Li, and Sastry, Chapter 5 <ref-murray-li-sastry>` and
{ref}`Rimon and Burdick, Chapter 4 <ref-rimon-burdick>` for the grasp map
and contact models.

## Polygonal Wrench Construction

By default, `build_grasp_matrix` approximates each friction cone with
eight edge forces. To construct their tangent directions, choose $r_i=(0,0,1)$
when $|n_{i,z}|<0.9$ and $r_i=(1,0,0)$ otherwise. An orthonormal
basis of the contact tangent plane is

$$
t_{i1}=\frac{n_i\times r_i}{\lVert n_i\times r_i\rVert_2},
\qquad t_{i2}=n_i\times t_{i1}.
$$

For $j=0,\ldots,7$, the directions are

$$
s_{ij}=\cos(2\pi j/8)t_{i1}+\sin(2\pi j/8)t_{i2}.
$$

These equations describe the ideal unit-vector construction; the code
adds a small denominator regularizer during normalization. Each edge
has unit normal-force component under the default
`unit_normal_force=True`:

$$
f_{ij}=n_i+\mu s_{ij}.
$$

Dividing moments by a length $\rho>0$ gives the primitive wrench

$$
w_{ij}=\begin{bmatrix}
f_{ij}\\
(p_i-c)\times f_{ij}/\rho
\end{bmatrix}\in\mathbb R^6.
$$

The matrix $G$ contains these columns. Its convex hull is the normalized
grasp wrench space

$$
W=\operatorname{conv}\{w_{ij}\}.
$$

Convex combinations assign nonnegative weights summing to one. Because
$n_i^\top f_{ij}=1$, this imposes a unit total normal-force budget across
contacts. The unit budget refers to normal-force components. With
`unit_normal_force=False`, the builder additionally scales
them by $1/\sqrt{1+\mu^2}$.

## Force Closure and Ferrari–Canny Epsilon

Force closure means that contacts can resist sufficiently small wrenches
in every direction. Mathematically, the origin must lie in the interior
of the full-dimensional hull $W$. Write its facet representation as
$W=\{w:a_h^\top w\le b_h\ \forall h\}$, where $h$ indexes facets.
The {ref}`Ferrari–Canny metric <ref-ferrari-canny>` is

$$
\varepsilon=\min_h\frac{b_h}{\lVert a_h\rVert_2}.
$$

It is the radius of the largest origin-centered Euclidean ball contained
in $W$. For epsilon, the planner uses

$$
\rho=\max\left(0.01\,\mathrm{mm},\max_i\lVert p_i-c\rVert_2\right).
$$

This is a contact-derived radius. Epsilon comparisons depend on this
moment normalization, the friction coefficient, and the force budget;
see {ref}`Roa and Suárez <ref-roa-suarez>`. `compute_gws_epsilon(G)`
returns `(epsilon, is_force_closure)`. Its numerical hull checks can
return zero epsilon with a true Boolean at a boundary, so that Boolean
is not an exact strict-interior certificate. `native_epsilon` is the
separate simulator metric with its own normalization.

The min-weight API tests for an equilibrium combination with uniformly
positive primitive weights. For $G$ with $M$ columns, it solves

$$
\ell^\star=\max_{a\in\mathbb R^M,\,\ell\in\mathbb R}\ell
\quad\text{subject to}\quad
Ga=0,\quad\mathbf1^\top a=1,\quad a_j\ge\ell\ \forall j.
$$

`min_weight_force_closure(G)` returns `(l_star, is_force_closure)`; its
Boolean requires both a margin above `tol=1e-6` and matrix rank six.
The hull and LP computations require SciPy for these metrics; the
no-SciPy path uses a singular-value proxy.

### Analytic Contact Example

The following synthetic contact data place six contacts at the face
centers of an axis-aligned cube, with inward normals and a contact
radius of 25 mm. This analytic quality calculation uses prescribed
contacts with NumPy and SciPy.

```{literalinclude} examples/numerical_quality.py
:language: python
```

Output:

```text
G shape: (6, 48)
min-weight: 0.020833; force closure: True
epsilon: 0.408248
```

## Task-Wrench Capacity

A task score measures capacity along specified loading directions.
Each `task_forces_mm` row supplies an application point $p_k$ and a
nonzero direction $f_k$ for the contact force that balances the external
load. The planner normalizes $\lVert f_k\rVert_2=1$ and constructs

$$
\ell_k=\begin{bmatrix}
f_k\\
(p_k-c)\times f_k/\rho
\end{bmatrix}.
$$

For example, `[0, 0, 0, 0, 0, 1]` requests upward support through the
object-frame origin against downward gravity. Its moment about $c$ is
zero only when that line of force passes through $c$.

The task-wrench capacity is the largest common multiplier supported
along every specified ray:

$$
Q=\min_k\sup\{\lambda\ge0:\lambda\ell_k\in W\}.
$$

Using the facet convention above, this becomes

$$
Q=\min_k\min_{h:a_h^\top\ell_k>0}
\frac{b_h}{a_h^\top\ell_k}.
$$

`task_wrenches_from_forces` constructs the normalized load rays;
`compute_task_wrench_quality` evaluates their common capacity against
the grasp matrix. Both functions are in `graspit_py.grasp.quality`.

The task scorer requires a full-dimensional hull with the origin
strictly inside and returns zero if that check fails. $Q$ multiplies
the load ray under the unit total normal-force budget. Expressing it
as a physical load in newtons requires a calibrated force budget.

```{figure} figures/wrench-capacity.svg
:alt: Schematic two-dimensional wrench set with an epsilon ball and a task ray intersecting the hull at Q times the load ray.
:target: _images/wrench-capacity.svg

Two-dimensional analogy for wrench capacity. The inscribed ball
illustrates $\varepsilon$; the task ray reaches the hull at $Q\ell$.
The planner computes its scores in six-dimensional wrench space.
```

For task scoring the planner uses $\rho=100$ mm for both the hull and
the load rays. Applying the same invertible moment scaling to both
preserves each ray-intersection multiplier. Epsilon depends on the
scaling because it measures a Euclidean radius. For fixed contacts,
increasing $\mu$ expands the polygonal cone cross-sections and their
hull, so the supported task multiplier is nondecreasing.

## Lower-Tail Ranking

A high mean capacity can coexist with low capacity in adverse poses or
at low friction. The planner therefore averages the lowest quality
values over a specified probability mass. For quality reward $Z$ and
tail mass $\beta\in(0,1]$, lower-tail conditional value-at-risk is

$$
\operatorname{CVaR}^{\mathrm{lower}}_\beta(Z)
=\sup_{\eta\in\mathbb R}
\left[\eta-\frac1\beta\mathbb E[(\eta-Z)_+]\right],
\qquad (x)_+=\max(x,0).
$$

Here $\eta$ is a scalar threshold. `lower_tail_mean(values, alpha=beta)`
implements this reward measure; `alpha` is tail mass. The separate
`cvar` API averages the upper tail of costs, with
`lower_tail_mean(q) = -cvar(-q)` for matching arguments.

The settings `n_pose_samples`, `pose_sigma_mm`, and `pose_sigma_deg`
control the perturbed-pose count and the componentwise translation and
rotation-vector standard deviations. Each pose trial repeats the approach
and closure. Translation is added
in the object frame; a sampled rotation-vector rotation left-multiplies
the palm rotation and rotates the travel direction. The default is eight
perturbed poses plus the nominal pose, evaluated over nine equally
weighted friction coefficients. Failed perturbed approaches or closures
contribute a zero row. All pose–friction entries have equal weight,
including the nominal row.

Let $z_{(1)}\le\cdots\le z_{(M)}$ be the sorted entries,
$m=\lfloor\beta M\rfloor$, and $r=\beta M-m$. The empirical tail mean is

$$
\operatorname{CVaR}^{\mathrm{lower}}_\beta
=\frac{\sum_{j=1}^m z_{(j)}+r z_{(m+1)}}{\beta M},
$$

with the boundary term omitted when $r=0$. The planner's
`tail_mass` defaults to 0.2. Setting it to one gives the sample mean;
setting `n_pose_samples=0` and supplying a single friction coefficient
gives nominal scoring at that coefficient. For nonnegative quality,
a zero-quality mass of at least $\beta$ produces zero tail mean. See
{ref}`Rockafellar and Uryasev <ref-rockafellar-uryasev>` for the discrete
probability treatment.

The fractional boundary matters when the tail mass does not select a
whole number of samples. For scores $0,0.03,0.10,0.11$ and
$\beta=0.375$, the tail contains the zero score and half the mass of
the next score:

$$
\frac{0+0.5\cdot0.03}{1.5}=0.01.
$$

```python
from graspit_py.risk import lower_tail_mean

value = lower_tail_mean([0.0, 0.03, 0.10, 0.11], alpha=0.375)
print(f'fractional empirical tail: {value:.6f}')
```

Output:

```text
fractional empirical tail: 0.010000
```

The API also accepts normalized or unnormalized probability weights.
For values `[0.0, 10.0]`, weights `[0.1, 0.9]`, and `alpha=0.5`, it returns
8.0: the tail uses mass 0.1 from the first value and mass 0.4 from the
second, giving $(0.1\cdot0+0.4\cdot10)/0.5=8$.

{ref}`FIRMGrasp <ref-firmgrasp>` evaluates the force-closure margin at
the mean of the adverse friction tail. Here the tail is taken over
quality values after closure simulation and friction-grid evaluation.
`epsilon_cvar` always stores the epsilon tail mean. With task loading,
`score` stores the tail mean of $Q$; otherwise it equals `epsilon_cvar`.

The [planning guide](planning.md) connects these quantities to complete
configurations and returned grasp records. The [API reference](api.md)
documents the Python operations, and the [export guide](export.md)
describes the joint-coordinate map used to reconstruct a command.
