import numpy as np
from graspit_py.grasp.quality import (
    build_grasp_matrix, compute_gws_epsilon, min_weight_force_closure,
)

positions_mm = 25.0 * np.vstack([np.eye(3), -np.eye(3)])
normals = -positions_mm / 25.0
center_mm = np.zeros(3)
G = build_grasp_matrix(positions_mm, normals, center_mm, friction_coef=0.5)
l_star, force_closure = min_weight_force_closure(G)
epsilon, _ = compute_gws_epsilon(G)
print('G shape:', G.shape)
print(f'min-weight: {l_star:.6f}; force closure: {force_closure}')
print(f'epsilon: {epsilon:.6f}')
