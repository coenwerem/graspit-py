# Geometry and grasp quality

## From MuJoCo hinges to GraspIt! joints

The exporter constructs Denavit–Hartenberg (DH) frames from the compiled
hinge axes at the MuJoCo reference configuration. It transforms each
collision mesh into its link frame and converts meters to millimeters.
Geometry on fixed bodies is expressed in the frame of the link that
moves with them.

For driven hinge $i$ and independent driver $j$, MuJoCo's
[affine joint equality](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-joint)
uses displacements from the reference angles $q_i^0$ and $q_j^0$:

$$
q_i-q_i^0=c_0+c_1(q_j-q_j^0).
$$

The manifest uses absolute joint coordinates in radians. With driver
coordinate $u_j=q_j$, the map is

$$
q_i=c_1u_j+b_i,\qquad b_i=q_i^0+c_0-c_1q_j^0.
$$

For example, a driver reference of $0.35$ rad, a driven reference of
$-0.20$ rad, $c_0=0.16$ rad, and $c_1=0.70$ give an offset of $-0.285$
rad. The rotation of the DH frame at hinge $i$ is
$\theta_i=\theta_i^0+q_i-q_i^0$, where $\theta_i^0$ is the DH angle at the
reference configuration. GraspIt! robot XML stores angular constants
in degrees. The manifest retains radians for reconstruction.

## From contacts to a wrench hull

Let $p_i$ be the contact point, $n_i$ its inward unit normal, and $c$
the object center of mass, all expressed in the object frame. At
contact $i$, the scorer approximates the friction cone with eight
equally spaced tangent directions $s_{ij}$. At friction coefficient
$\mu$, it uses the edge force $f_{ij}=n_i+\mu s_{ij}$ and constructs

$$
w_{ij}=\left(f_{ij},\;(p_i-c)\times f_{ij}/\rho\right),\qquad
W=\operatorname{conv}\{w_{ij}\}.
$$

The length $\rho>0$ scales moments relative to forces. Every edge has
unit normal force, so convex combinations represent a unit total
normal-force budget. Force closure requires the origin in the interior
of $W$. The [Ferrari–Canny metric](https://doi.org/10.1109/ROBOT.1992.219918)
is the radius of the largest origin-centered ball in this normalized
wrench space. The radius depends on the torque scale.

Let $\ell_k$ be the contact wrench needed to oppose task load $k$,
with the same moment scaling as $W$. The task score is the largest
common load multiplier supported along all specified wrench rays:

$$
Q=\min_k\sup\{\lambda\geq 0:\lambda\ell_k\in W\}.
$$

The scorer assigns zero when the wrench set lacks force closure.
For fixed contacts, a larger $\mu$ expands each tangent polygon and
therefore the wrench hull. In exact arithmetic, $Q$ cannot decrease.
A common invertible moment rescaling of $W$ and all $\ell_k$ preserves
the ray intersections and leaves $Q$ unchanged. For the native
simulator’s grasp measures, see
[Miller and Allen](https://doi.org/10.1109/MRA.2004.1371616).

## Nominal, mean, and lower-tail selection

The planner reruns approach and closure from perturbed standoff poses.
A translation offset moves the starting palm in the object frame.
A rotation-vector offset rotates both the palm and its travel
direction. The planner evaluates each realized contact set over the
friction grid. Failed closures contribute zero to the scenario scores
$Z$.

For probability mass $\beta\in(0,1]$, the lower-tail conditional
value-at-risk is

$$
\operatorname{CVaR}^{\mathrm{lower}}_{\beta}(Z)
=\sup_{\eta\in\mathbb{R}}\left[\eta-\frac{1}{\beta}
\mathbb{E}\left[(\eta-Z)_+\right]\right].
$$

Here $(x)_+=\max(x,0)$. The planner sorts scores from smallest to
largest and averages the lowest $\beta$ of their probability mass,
using part of the boundary sample as needed. Setting $\beta=1$
gives the mean. A single nominal closure at one friction coefficient
gives a nominal score. If at least $\beta$ of the mass has zero
quality, the lower-tail score is zero. See
[Rockafellar and Uryasev](https://doi.org/10.21314/JOR.2000.038) for
the variational formulation.

The [planning guide](planning.md) maps these quantities to the
configuration and the records returned by the CLI.
