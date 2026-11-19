"""
Geometry utilities for grasp planning.
"""

import numpy as np
from typing import Tuple, List, Union


def quaternion_to_matrix(quat: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to rotation matrix.

    Args:
        quat: Quaternion [x, y, z, w]

    Returns:
        3x3 rotation matrix
    """
    x, y, z, w = quat

    # Normalize quaternion
    quat = quat / np.linalg.norm(quat)
    x, y, z, w = quat

    # Compute rotation matrix
    R = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )

    return R


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    Convert rotation matrix to quaternion.

    Args:
        R: 3x3 rotation matrix

    Returns:
        Quaternion [x, y, z, w]
    """
    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return np.array([qx, qy, qz, qw])


def transform_points(
    points: np.ndarray, translation: np.ndarray, rotation: np.ndarray
) -> np.ndarray:
    """
    Transform points by rotation and translation.

    Args:
        points: Nx3 array of points
        translation: 3D translation vector
        rotation: 3x3 rotation matrix or quaternion [x, y, z, w]

    Returns:
        Transformed points
    """
    if rotation.shape == (4,):  # Quaternion
        R = quaternion_to_matrix(rotation)
    else:  # Rotation matrix
        R = rotation

    # Apply transformation: p' = R @ p + t
    transformed = (R @ points.T).T + translation
    return transformed


def compute_mesh_properties(vertices: np.ndarray, faces: np.ndarray) -> dict:
    """
    Compute mesh properties.

    Args:
        vertices: Nx3 array of vertex positions
        faces: Nx3 array of face indices

    Returns:
        Dictionary with mesh properties
    """
    # Compute center of mass
    center = np.mean(vertices, axis=0)

    # Compute bounding box
    min_bounds = np.min(vertices, axis=0)
    max_bounds = np.max(vertices, axis=0)
    size = max_bounds - min_bounds

    # Compute surface area
    face_vertices = vertices[faces]
    v1 = face_vertices[:, 1] - face_vertices[:, 0]
    v2 = face_vertices[:, 2] - face_vertices[:, 0]
    face_areas = 0.5 * np.linalg.norm(np.cross(v1, v2), axis=1)
    surface_area = np.sum(face_areas)

    # Estimate volume (using signed tetrahedron method)
    volume = 0.0
    for face in face_vertices:
        v1, v2, v3 = face
        # Volume of tetrahedron from origin to face
        vol = np.dot(v1, np.cross(v2, v3)) / 6.0
        volume += abs(vol)

    return {
        "center": center,
        "size": size,
        "min_bounds": min_bounds,
        "max_bounds": max_bounds,
        "surface_area": surface_area,
        "volume": volume,
        "num_vertices": len(vertices),
        "num_faces": len(faces),
    }


def compute_convex_hull(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute convex hull of points.

    Args:
        points: Nx3 array of points

    Returns:
        Tuple of (hull_vertices, hull_faces)
    """
    try:
        from scipy.spatial import ConvexHull

        hull = ConvexHull(points)

        # Hull vertices are subset of input points
        hull_vertices = points[hull.vertices]
        hull_faces = hull.simplices

        return hull_vertices, hull_faces
    except ImportError:
        # Fallback: return all points and empty faces
        print("Warning: scipy not available, convex hull computation disabled")
        return points, np.array([])


def compute_distance_to_mesh(
    points: np.ndarray, vertices: np.ndarray, faces: np.ndarray
) -> np.ndarray:
    """
    Compute minimum distance from points to mesh surface.

    Args:
        points: Nx3 array of query points
        vertices: Nx3 array of mesh vertices
        faces: Nx3 array of mesh faces

    Returns:
        N array of minimum distances
    """
    distances = np.zeros(len(points))

    for i, point in enumerate(points):
        min_dist = float("inf")

        # Check distance to all faces
        for face_idx in faces:
            face_vertices = vertices[face_idx]

            # Compute distance to triangle
            dist = point_to_triangle_distance(point, face_vertices)
            min_dist = min(min_dist, dist)

        distances[i] = min_dist

    return distances


def point_to_triangle_distance(point: np.ndarray, triangle: np.ndarray) -> float:
    """
    Compute minimum distance from point to triangle.

    Args:
        point: 3D point
        triangle: 3x3 array of triangle vertices

    Returns:
        Minimum distance
    """
    # Use vectorized approach for distance to triangle
    v0, v1, v2 = triangle

    # Compute triangle edges
    edge0 = v1 - v0
    edge1 = v2 - v0

    # Compute normal
    normal = np.cross(edge0, edge1)
    normal_length = np.linalg.norm(normal)

    if normal_length == 0:
        # Degenerate triangle
        return np.linalg.norm(point - v0)

    normal = normal / normal_length

    # Project point onto triangle plane
    point_to_v0 = point - v0
    distance_to_plane = np.dot(point_to_v0, normal)
    projected_point = point - distance_to_plane * normal

    # Check if projected point is inside triangle
    if point_in_triangle(projected_point, v0, v1, v2):
        return abs(distance_to_plane)

    # Otherwise, find minimum distance to edges
    dist_to_edge0 = point_to_line_distance(point, v0, v1)
    dist_to_edge1 = point_to_line_distance(point, v1, v2)
    dist_to_edge2 = point_to_line_distance(point, v2, v0)

    return min(dist_to_edge0, dist_to_edge1, dist_to_edge2)


def point_in_triangle(
    p: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray
) -> bool:
    """
    Check if point is inside triangle using barycentric coordinates.

    Args:
        p: Point to test
        a, b, c: Triangle vertices

    Returns:
        True if point is inside triangle
    """
    # Compute vectors
    v0 = c - a
    v1 = b - a
    v2 = p - a

    # Compute dot products
    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)

    # Compute barycentric coordinates
    inv_denom = 1 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * inv_denom
    v = (dot00 * dot12 - dot01 * dot02) * inv_denom

    # Check if point is in triangle
    return (u >= 0) and (v >= 0) and (u + v <= 1)


def point_to_line_distance(
    point: np.ndarray, line_start: np.ndarray, line_end: np.ndarray
) -> float:
    """
    Compute minimum distance from point to line segment.

    Args:
        point: Query point
        line_start: Line segment start
        line_end: Line segment end

    Returns:
        Minimum distance
    """
    line_vec = line_end - line_start
    point_vec = point - line_start
    line_length = np.linalg.norm(line_vec)

    if line_length == 0:
        return np.linalg.norm(point - line_start)

    # Project point onto line
    t = max(0, min(1, np.dot(point_vec, line_vec) / (line_length**2)))
    projection = line_start + t * line_vec

    return np.linalg.norm(point - projection)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to quaternion.

    Args:
        roll: Rotation around X axis
        pitch: Rotation around Y axis
        yaw: Rotation around Z axis

    Returns:
        Quaternion [x, y, z, w]
    """
    # Compute half angles
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    # Compute quaternion
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return np.array([qx, qy, qz, qw])
