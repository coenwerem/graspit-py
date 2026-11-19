# Kinematics, Contact Mechanics, and Grasp Quality

The exporter preserves the link geometry used to generate contacts. The
planner then maps each contact set to the wrenches it can exert on the
object. This chapter defines both mappings and the lower-tail score
used to compare planned approaches.

## Joint Coordinates and Link Geometry

A hinge constrains the relative motion of two bodies to rotation about
one axis. A serial finger chain composes these joint motions from the
palm to the fingertip. Under the standard Denavit–Hartenberg convention,
the transform from link frame $i$ to its predecessor is

$$
{}^{i-1}T_i(q_i) =
R_z\bigl(\theta_i^0+q_i-q_i^0\bigr)T_z(d_i)T_x(a_i)R_x(\gamma_i).
$$

Here $q_i^0$ is the MuJoCo reference angle, $\theta_i^0$ is the DH
angle at reference, $d_i$ is the offset along the hinge axis, $a_i$ is
the common-normal length, and $\gamma_i$ is the twist. The exporter
constructs the DH frames from the compiled hinge axes at reference and
transforms the participating collision meshes into their corresponding
link frames. Geometry rigidly attached to a link shares its transform.
GraspIt! XML stores angular constants in degrees and mesh coordinates
in millimeters.

Some fingers contain an independent hinge and a driven hinge linked by
an affine MuJoCo equality. If $u_j=q_j$ is the driver coordinate, with
reference angles $q_i^0$ and $q_j^0$, the equality and its reconstruction
in absolute joint coordinates are

$$
q_i-q_i^0=c_0+c_1(q_j-q_j^0),\qquad
q_i=c_1u_j+b_i,\qquad b_i=q_i^0+c_0-c_1q_j^0.
$$

For $q_j^0=0.35$ rad, $q_i^0=-0.20$ rad, $c_0=0.16$ rad, and
$c_1=0.70$, the manifest offset is $b_i=-0.285$ rad. The manifest
expresses the absolute-coordinate map in radians. The source convention
is defined in the [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-joint).

## Frictional Point Contacts

Let $p_i$ be the position of contact $i$, $n_i$ its inward unit
normal, and $c$ the object center of mass, all in the object frame
$\{O\}$. In the frictional point-contact model, the hand exerts a force
$f_i$ with normal component $f_{n,i}=n_i^\top f_i$ and tangential
component $f_{t,i}=f_i-f_{n,i}n_i$. The unilateral Coulomb conditions are

$$
f_{n,i}\ge 0,\qquad \lVert f_{t,i}\rVert_2\le \mu f_{n,i}.
$$

A contact exerts a moment about $c$ through the force lever arm. The
force basis at a point has three components. The moment about $c$ is
therefore determined by $f_i$ and $p_i$. See
[Murray, Li, and Sastry, Chapter 5](https://www.cds.caltech.edu/~murray/books/MLS/)
and [Rimon and Burdick, Chapter 4](https://doi.org/10.1017/9781108552011)
for the grasp map and its contact models.

## Grasp Wrench Space and Force Closure

The scorer approximates each cone by a polygonal cone with eight
equally spaced tangent directions $s_{ij}$. Each edge supports one
unit of normal force. With moment scale $\rho>0$, the edge forces,
primitive wrenches, and normalized grasp wrench space are

$$
f_{ij}=n_i+\mu s_{ij},\qquad
w_{ij}=\begin{bmatrix}f_{ij}\\ (p_i-c)\times f_{ij}/\rho\end{bmatrix},
\qquad
W=\operatorname{conv}\{w_{ij}\}.
$$

A convex combination assigns nonnegative coefficients to the edge
wrenches and constrains their sum to one. Since $n_i^\top f_{ij}=1$,
this constraint imposes a unit total normal-force budget. It is a
contact-level normalization of wrench capacity. Force closure requires
$0$ in the interior of $W$. If this condition holds, a sufficiently
small wrench in every direction lies within the achievable set. See
[Roa and Suárez](https://doi.org/10.1007/s10514-014-9402-3)
for the effect of force budgets and the moment scale on wrench-space metrics.

For a full-dimensional hull $W=\{w:a_h^\top w\le b_h\ \forall h\}$ with
$0$ in its interior, the [Ferrari–Canny
metric](https://doi.org/10.1109/ROBOT.1992.219918) is

$$
\varepsilon=\min_h\frac{b_h}{\lVert a_h\rVert_2}.
$$

This radius is the magnitude of the largest origin-centered Euclidean
ball contained in $W$. The Python planner uses
$\rho=\max(0.01\,\mathrm{mm},\max_i\lVert p_i-c\rVert_2)$ for
$\varepsilon$. Comparisons across grasp records must account for the
moment normalization and friction coefficient. The recorded native
GraspIt! epsilon is a separate quantity returned by the simulator.

## Task-Wrench Capacity

For each loading condition $k$, the planner receives a point $p_k$
and the direction $f_k$ of the contact force required to balance the
external load. It normalizes $\lVert f_k\rVert_2=1$ and forms the wrench
$\ell_k=(f_k,(p_k-c)\times f_k/\rho)$. The task score is the largest
common load multiplier supported along all specified rays:

$$
Q=\min_k\sup\{\lambda\ge 0:\lambda\ell_k\in W\}
=\min_k\,\min_{h:a_h^\top\ell_k>0}
  \frac{b_h}{a_h^\top\ell_k}.
$$

The task scorer requires a full-dimensional hull with the origin
strictly inside. It returns zero when this force-closure condition
fails. The planner uses $\rho=100\,\mathrm{mm}$ for both the hull and
the task wrenches. A common invertible rescaling of their moment
coordinates preserves each ray intersection multiplier. For fixed
contacts, increasing $\mu$ expands each polygonal cone cross-section
and hence $W$. The supported multiplier is therefore nondecreasing
with $\mu$ at fixed contact geometry.

## Lower-Tail Ranking

An approach command specifies palm travel direction, roll, preshape,
and closing coordinates. In pose-perturbation trials, the planner
adds a translation in the object frame and left-multiplies the palm
rotation by a rotation from a sampled rotation vector. It applies
the same rotation to the travel direction, repeats the approach and
closure, then scores the resulting contacts over the friction grid.
Failed approaches or closures contribute zero quality.

Let $Z$ be a reward with larger values indicating greater wrench
capacity. For tail mass $\beta\in(0,1]$, the lower-tail conditional
value-at-risk is

$$
\operatorname{CVaR}^{\mathrm{lower}}_{\beta}(Z)
=\sup_{\eta\in\mathbb{R}}\left[
\eta-\frac{1}{\beta}\mathbb{E}\bigl[(\eta-Z)_+\bigr]\right],
\qquad (x)_+=\max(x,0).
$$

The planner treats all pose–friction pairs, including the nominal pose
row, as equally weighted samples. Write $z_{(1)}\le\cdots\le z_{(M)}$,
$m=\lfloor\beta M\rfloor$, and $r=\beta M-m$. The empirical score is
the sum of the $m$ smallest scores plus $r z_{(m+1)}$, divided by
$\beta M$. For example, the lowest $37.5\%$ of four equally weighted scores
$0,0.03,0.10,0.11$ has average $(0+0.5\cdot 0.03)/1.5=0.01$.
The boundary term is omitted at $r=0$. A score of zero
results when at least $\beta$ of the mass has zero quality. At
$\beta=1$ the ranking uses the sample mean. With one pose and one
friction coefficient, it uses the nominal score at the specified
coefficient. For the discrete probability treatment, see
[Rockafellar and Uryasev](https://doi.org/10.1016/S0378-4266(02)00271-6).

The [planning guide](planning.md) maps these quantities to configuration
fields and candidate records.
