"""
Quality metrics for grasp evaluation.
"""

from dataclasses import dataclass
from typing import List, Optional, Dict, Any, Tuple
import numpy as np

from ..core.types import GraspResult, GraspQuality, QualityMetric, ContactPoint


class QualityMetrics:
    """
    Interface for computing various grasp quality metrics.
    """

    def __init__(self):
        """Initialize quality metrics calculator."""
        pass

    def evaluate_all(self, grasp: GraspResult) -> GraspQuality:
        """
        Evaluate all quality metrics for a grasp.

        Args:
            grasp: Grasp result to evaluate

        Returns:
            GraspQuality object with all metrics
        """
        volume = self.evaluate_single(grasp, QualityMetric.VOLUME)
        epsilon = self.evaluate_single(grasp, QualityMetric.EPSILON)
        volume_epsilon = self.evaluate_single(grasp, QualityMetric.VOLUME_EPSILON)
        min_force = self.evaluate_single(grasp, QualityMetric.MINIMUM_FORCE)
        contact_patch = self.evaluate_single(grasp, QualityMetric.CONTACT_PATCH)

        return GraspQuality(
            volume=volume,
            epsilon=epsilon,
            volume_epsilon=volume_epsilon,
            min_force=min_force,
            contact_patch=contact_patch,
        )

    def evaluate_single(self, grasp: GraspResult, metric: QualityMetric) -> float:
        """
        Evaluate a single quality metric.

        Args:
            grasp: Grasp result to evaluate
            metric: Quality metric to compute

        Returns:
            Quality value
        """
        if metric == QualityMetric.VOLUME:
            return float(grasp.quality.volume)
        if metric == QualityMetric.EPSILON:
            return float(grasp.quality.epsilon)
        if metric == QualityMetric.VOLUME_EPSILON:
            return float(grasp.quality.volume_epsilon)
        if metric == QualityMetric.MINIMUM_FORCE:
            # Compute minimum normal force across contacts
            if not grasp.contacts:
                return 0.0
            forces = []
            for c in grasp.contacts:
                if c.force is not None:
                    import numpy as np

                    forces.append(float(np.linalg.norm(c.force)))
            if not forces:
                # No force data available, use epsilon as proxy
                return max(0.0, grasp.quality.epsilon)
            return min(forces)
        if metric == QualityMetric.CONTACT_PATCH:
            # Contact patch approximation: number of contacts weighted by spread
            if len(grasp.contacts) < 2:
                return 0.0
            import numpy as np

            positions = np.array([c.position for c in grasp.contacts])
            # Compute spread as mean pairwise distance
            centroid = positions.mean(axis=0)
            spread = np.mean(np.linalg.norm(positions - centroid, axis=1))
            # Normalize by contact count
            return float(
                len(grasp.contacts) * spread / 100.0
            )  # Scale to reasonable range

        raise ValueError(f"Unknown quality metric: {metric.value}")


class GraspComparator:
    """
    Utility class for comparing and ranking grasps.
    """

    def __init__(self, primary_metric: QualityMetric = QualityMetric.VOLUME_EPSILON):
        """
        Initialize comparator with primary quality metric.

        Args:
            primary_metric: Main quality metric for ranking
        """
        self.primary_metric = primary_metric
        self.quality_metrics = QualityMetrics()

    def rank_grasps(self, grasps: List[GraspResult]) -> List[GraspResult]:
        """
        Rank grasps by quality.

        Args:
            grasps: List of grasps to rank

        Returns:
            Sorted list of grasps (best first)
        """
        # Compute quality for all grasps
        grasp_qualities = []
        for grasp in grasps:
            quality_value = self.quality_metrics.evaluate_single(
                grasp, self.primary_metric
            )
            grasp_qualities.append((grasp, quality_value))

        # Sort by quality (descending)
        grasp_qualities.sort(key=lambda x: x[1], reverse=True)

        return [grasp for grasp, _ in grasp_qualities]

    def filter_grasps(
        self,
        grasps: List[GraspResult],
        quality_threshold: float = 0.1,
        min_contacts: int = 3,
        non_penetrating_only: bool = True,
    ) -> List[GraspResult]:
        """
        Filter grasps based on criteria.

        Args:
            grasps: List of grasps to filter
            quality_threshold: Minimum quality score
            min_contacts: Minimum number of contact points
            non_penetrating_only: Only keep non-penetrating grasps

        Returns:
            Filtered list of grasps
        """
        filtered_grasps = []

        for grasp in grasps:
            # Check quality threshold
            quality = self.quality_metrics.evaluate_single(grasp, self.primary_metric)
            if quality < quality_threshold:
                continue

            # Check contact count
            if len(grasp.contacts) < min_contacts:
                continue

            # Check penetration
            if non_penetrating_only and not grasp.is_non_penetrating:
                continue

            filtered_grasps.append(grasp)

        return filtered_grasps


# ============================================================================
# Grasp Wrench Space (GWS) Computation Functions
# ============================================================================

try:
    from scipy.spatial import ConvexHull, QhullError
    from scipy.optimize import linprog

    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


@dataclass
class GWSResult:
    """Result of GWS analysis."""

    epsilon: float  # Ferrari-Canny quality (largest inscribed ball)
    volume: float  # GWS volume
    min_singular: float  # Minimum singular value of grasp matrix
    is_force_closure: bool  # True if origin is inside GWS
    n_contacts: int  # Number of contacts used

    def quality(self) -> float:
        """Primary quality metric (normalized epsilon)."""
        if not self.is_force_closure:
            return 0.0
        return float(min(self.epsilon / 0.5, 1.0))


def build_grasp_matrix(
    contact_positions: np.ndarray,
    contact_normals: np.ndarray,
    object_center: np.ndarray,
    friction_coef: float = 0.5,
    n_friction_edges: int = 8,
    max_radius: Optional[float] = None,
    unit_normal_force: bool = True,
) -> np.ndarray:
    """Build the grasp matrix G from contact data.

    For each contact with friction cone approximation, we get primitive
    wrenches that span the achievable wrench space at that contact.

    Following GraspIt! convention, torques are normalized by max object radius
    to balance force and torque components in the wrench space metric.

    Args:
        contact_positions: (N, 3) contact point positions
        contact_normals: (N, 3) contact surface normals (pointing into object)
        object_center: (3,) object center of mass position
        friction_coef: Friction coefficient for cone linearization
        n_friction_edges: Number of edges to approximate friction cone
        max_radius: Maximum object radius for torque normalization
        unit_normal_force: Cone edges with unit normal component, n + mu t_k
            (Ferrari-Canny / GraspIt convention; epsilon then grows with mu).
            False scales each edge to unit total force, which shrinks the normal
            component as mu grows.

    Returns:
        G: (6, n_wrenches) grasp matrix where each column is a primitive wrench
    """
    contact_positions = np.atleast_2d(contact_positions)
    contact_normals = np.atleast_2d(contact_normals)

    if contact_positions.shape[0] == 0:
        return np.zeros((6, 1))

    n_contacts = contact_positions.shape[0]

    # Compute max radius if not provided (GraspIt style normalization)
    if max_radius is None:
        radii = [np.linalg.norm(p - object_center) for p in contact_positions]
        max_radius = float(max(radii)) if radii else 1.0
        max_radius = max(max_radius, 0.01)  # Avoid division by zero

    # Generate friction cone edge directions
    angles = np.linspace(0, 2 * np.pi, n_friction_edges, endpoint=False)

    all_wrenches = []

    for i in range(n_contacts):
        pos = contact_positions[i]
        normal = contact_normals[i]
        normal = normal / (np.linalg.norm(normal) + 1e-10)

        # Build local frame at contact (Gram-Schmidt orthogonalization)
        if abs(normal[2]) < 0.9:
            tangent1 = np.cross(normal, np.array([0.0, 0.0, 1.0]))
        else:
            tangent1 = np.cross(normal, np.array([1.0, 0.0, 0.0]))
        tangent1 = tangent1 / (np.linalg.norm(tangent1) + 1e-10)
        tangent2 = np.cross(normal, tangent1)

        # Friction cone edges: f = (n + mu*cos(θ)*t1 + mu*sin(θ)*t2) * scale
        scale = 1.0 if unit_normal_force else 1.0 / np.sqrt(1.0 + friction_coef**2)

        for angle in angles:
            # Direction on friction cone (unit magnitude)
            f_dir = scale * (
                normal
                + friction_coef * np.cos(angle) * tangent1
                + friction_coef * np.sin(angle) * tangent2
            )

            # Moment arm from object center to contact
            r = pos - object_center

            # Wrench = [force; torque/maxRadius] (GraspIt normalization)
            torque = np.cross(r, f_dir) / max_radius
            wrench = np.concatenate([f_dir, torque])
            all_wrenches.append(wrench)

    return np.array(all_wrenches).T  # (6, n_wrenches)


def build_grasp_matrix_from_contacts(
    contacts: List[ContactPoint],
    object_center: np.ndarray,
    friction_coef: float = 0.5,
    n_friction_edges: int = 8,
    max_radius: Optional[float] = None,
) -> np.ndarray:
    """Build grasp matrix from ContactPoint list.

    Convenience wrapper for build_grasp_matrix using ContactPoint objects.

    Args:
        contacts: List of ContactPoint objects
        object_center: Object center of mass
        friction_coef: Friction coefficient
        n_friction_edges: Number of friction cone edges
        max_radius: Max radius for torque normalization

    Returns:
        (6, n_wrenches) grasp matrix
    """
    if not contacts:
        return np.zeros((6, 1))

    positions = np.array([c.position for c in contacts])
    normals = np.array([c.normal for c in contacts])

    return build_grasp_matrix(
        positions, normals, object_center, friction_coef, n_friction_edges, max_radius
    )


def compute_gws_epsilon(
    grasp_matrix: np.ndarray,
    max_iter: int = 1000,
) -> Tuple[float, bool]:
    """Compute Ferrari-Canny epsilon quality metric.

    Epsilon is the radius of the largest ball centered at origin
    that fits inside the GWS convex hull.

    Args:
        grasp_matrix: (6, n_wrenches) matrix of primitive wrenches
        max_iter: Max iterations for convex hull

    Returns:
        (epsilon, is_force_closure)
    """
    if not HAS_SCIPY:
        # Fallback: use minimum singular value as proxy
        _, s, _ = np.linalg.svd(grasp_matrix, full_matrices=False)
        min_sv = s[-1] if len(s) > 0 else 0.0
        return float(min_sv), min_sv > 1e-6

    try:
        # Build convex hull of wrench columns in 6D
        points = grasp_matrix.T  # (n_wrenches, 6)

        if points.shape[0] < 7:
            # Not enough points for 6D hull
            return 0.0, False

        # Check rank: degenerate wrench sets crash QHull
        _, s, _ = np.linalg.svd(points - points.mean(axis=0), full_matrices=False)
        if s.shape[0] < 6 or s[5] < 1e-10:
            return 0.0, False

        hull = ConvexHull(points)

        # Check if origin is inside hull and compute epsilon
        # For each hyperplane, compute signed distance from origin
        # ConvexHull.equations format: [a1, ..., a6, b] where a·x + b = 0

        min_dist = float("inf")
        for eq in hull.equations:
            normal = eq[:-1]  # (6,) normal vector
            offset = eq[-1]  # scalar offset

            normal_norm = np.linalg.norm(normal)
            if normal_norm < 1e-15:
                continue

            # Signed distance from origin: d = offset / ||normal||
            signed_dist = offset / normal_norm

            if signed_dist > 1e-10:
                # Origin is outside this hyperplane -> not force closure
                return 0.0, False

            min_dist = min(min_dist, float(abs(signed_dist)))

        return float(min_dist), True

    except QhullError:
        return 0.0, False


def task_wrenches_from_forces(
    points: np.ndarray,
    forces: np.ndarray,
    object_center: np.ndarray,
    torque_scale: float,
) -> np.ndarray:
    """(n, 6) wrenches [f, (p - c) x f / torque_scale] of forces applied at points."""
    points, forces = np.atleast_2d(points), np.atleast_2d(forces)
    torques = np.cross(points - np.asarray(object_center, float), forces) / torque_scale
    return np.hstack([forces, torques])


def compute_task_wrench_quality(
    grasp_matrix: np.ndarray,
    task_wrenches: np.ndarray,
) -> float:
    """Largest scale t such that t * w lies in the grasp wrench space for every
    task wrench w (rows of ``task_wrenches``), i.e. the worst-case ray distance
    from the origin to the GWS boundary along the task directions.

    With the GraspIt (L1) wrench space this is the largest task load the grasp
    resists per unit total normal force. A linear rescaling of the torque axes
    maps both the hull and the task wrenches, so unlike epsilon the value does
    not depend on the torque normalization. Returns 0 without force closure.
    """
    if not HAS_SCIPY:
        raise ImportError("compute_task_wrench_quality needs scipy")
    points = grasp_matrix.T
    if points.shape[0] < 7:
        return 0.0
    _, s, _ = np.linalg.svd(points - points.mean(axis=0), full_matrices=False)
    if s.shape[0] < 6 or s[5] < 1e-10:
        return 0.0
    try:
        hull = ConvexHull(points)
    except QhullError:
        return 0.0
    A, b = hull.equations[:, :-1], hull.equations[:, -1]   # inside: A x + b <= 0
    if np.any(b > -1e-10):
        return 0.0                                          # origin not strictly inside
    W = np.atleast_2d(np.asarray(task_wrenches, float))
    proj = A @ W.T                                          # (n_facets, n_task)
    with np.errstate(divide='ignore'):
        t = np.where(proj > 1e-15, -b[:, None] / proj, np.inf)
    return float(np.min(t.min(axis=0)))


def min_weight_force_closure(
    grasp_matrix: np.ndarray,
    tol: float = 1e-6,
) -> Tuple[float, bool]:
    """Compute the FRoGGeR min-weight LP and certify force closure in R^6.

    With the LP backend, force closure requires an optimum above ``tol`` and
    full wrench rank, evaluated with NumPy's default SVD tolerance. A positive
    optimum for a rank-deficient set places the origin in its relative
    interior, so its optimum is retained without certifying force closure.

    Args:
        grasp_matrix: (6, n_wrenches) matrix of primitive wrenches
        tol: Positive-optimum threshold for the LP certificate

    Returns:
        (l_star, is_force_closure), retaining the LP optimum independently of
        the rank condition. Without the LP backend, delegates to
        ``compute_gws_epsilon``. The SciPy-free path returns a minimum-
        singular-value proxy and its threshold indicator, not an LP certificate.
    """
    if not HAS_SCIPY:
        return compute_gws_epsilon(grasp_matrix)

    linprog_fn = globals().get("linprog")
    if linprog_fn is None:
        return compute_gws_epsilon(grasp_matrix)

    W = grasp_matrix  # (6, n)
    n = W.shape[1]

    if n < 7:
        return 0.0, False

    c = np.zeros(n + 1)
    c[-1] = -1.0

    # Equality: W @ alpha = 0, sum(alpha) = 1
    A_eq = np.hstack([np.vstack([W, np.ones(n)]), np.zeros((7, 1))])
    b_eq = np.zeros(7)
    b_eq[-1] = 1.0

    A_ub = np.zeros((n, n + 1))
    A_ub[:, :n] = -np.eye(n)
    A_ub[:, -1] = 1.0
    b_ub = np.zeros(n)

    bounds = [(None, None) for _ in range(n + 1)]

    try:
        result = linprog_fn(
            c,
            A_ub=A_ub,
            b_ub=b_ub,
            A_eq=A_eq,
            b_eq=b_eq,
            bounds=bounds,
            method="highs",
        )

        if result.success:
            l_opt = float(result.x[-1])
            return l_opt, bool(l_opt > tol and np.linalg.matrix_rank(W) == 6)
        else:
            return 0.0, False
    except Exception:
        return 0.0, False


def analyze_gws(
    contacts: List[ContactPoint],
    object_center: np.ndarray,
    friction_coef: float = 0.5,
    max_radius: Optional[float] = None,
) -> GWSResult:
    """Analyze grasp wrench space for a set of contacts.

    Args:
        contacts: List of ContactPoint objects
        object_center: Object center of mass
        friction_coef: Friction coefficient
        max_radius: Max object radius for torque normalization

    Returns:
        GWSResult with quality metrics
    """
    if len(contacts) < 2:
        return GWSResult(
            epsilon=0.0,
            volume=0.0,
            min_singular=0.0,
            is_force_closure=False,
            n_contacts=len(contacts),
        )

    G = build_grasp_matrix_from_contacts(
        contacts, object_center, friction_coef, max_radius=max_radius
    )

    # Singular value analysis
    _, s, _ = np.linalg.svd(G, full_matrices=False)
    min_singular = float(s[-1]) if len(s) > 0 else 0.0

    # Epsilon quality
    epsilon, is_fc = compute_gws_epsilon(G)

    # Volume (scaled for readability)
    volume = 0.0
    if HAS_SCIPY and G.shape[1] >= 7:
        try:
            hull = ConvexHull(G.T)
            volume = hull.volume * 1e6
        except QhullError:
            pass

    return GWSResult(
        epsilon=epsilon,
        volume=volume,
        min_singular=min_singular,
        is_force_closure=is_fc,
        n_contacts=len(contacts),
    )


def compute_gws_under_friction_uncertainty(
    contacts: List[ContactPoint],
    object_center: np.ndarray,
    friction_mean: float = 0.7,
    friction_std: float = 0.15,
    n_samples: int = 100,
    rng: Optional[np.random.Generator] = None,
) -> Dict[str, Any]:
    """Compute GWS metrics under friction coefficient uncertainty.

    Samples friction coefficients and computes statistics of GWS metrics.

    Args:
        contacts: Contact points
        object_center: Object COM
        friction_mean: Mean friction coefficient
        friction_std: Standard deviation of friction
        n_samples: Number of MC samples
        rng: Random number generator

    Returns:
        Dictionary with sampled metrics and statistics
    """
    if rng is None:
        rng = np.random.default_rng()

    mu_samples = rng.normal(friction_mean, friction_std, size=n_samples)
    mu_samples = np.clip(mu_samples, 0.1, 2.0)

    epsilon_samples = []
    volume_samples = []
    fc_count = 0

    for mu in mu_samples:
        result = analyze_gws(contacts, object_center, friction_coef=mu)
        epsilon_samples.append(result.epsilon)
        volume_samples.append(result.volume)
        if result.is_force_closure:
            fc_count += 1

    epsilon_samples = np.array(epsilon_samples)
    volume_samples = np.array(volume_samples)

    return {
        "epsilon_mean": float(np.mean(epsilon_samples)),
        "epsilon_std": float(np.std(epsilon_samples)),
        "epsilon_min": float(np.min(epsilon_samples)),
        "epsilon_max": float(np.max(epsilon_samples)),
        "volume_mean": float(np.mean(volume_samples)),
        "volume_std": float(np.std(volume_samples)),
        "fc_probability": fc_count / n_samples,
        "n_samples": n_samples,
        "epsilon_samples": epsilon_samples,
        "volume_samples": volume_samples,
    }
