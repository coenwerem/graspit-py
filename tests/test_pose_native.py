"""Native pose matrices must apply the same active transform as geometry queries."""
import os
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

gp = pytest.importorskip('graspit_python_wrapper.core.bindings',
                         reason='native GraspIt bindings are not built',
                         exc_type=ImportError)
ROOT = Path(os.environ.get('GRASPIT', ''))
ROBOT = ROOT / 'models/robots/Barrett/Barrett.xml'
CUBE = ROOT / 'models/objects/cube.xml'
pytestmark = pytest.mark.skipif(not ROBOT.is_file() or not CUBE.is_file(),
                                reason='set GRASPIT to the upstream models directory')


@pytest.fixture
def pose_world():
    gp.initialize()
    world = gp.World()
    world.init()
    world.clear()
    assert world.load_robot(str(ROBOT))
    assert world.load_graspable_body(str(CUBE))
    return world


def _active_matrix(pose):
    """Build the independent quaternion/translation representation for comparison."""
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_quat(pose['quat_wxyz'], scalar_first=True).as_matrix()
    expected[:3, 3] = pose['translation']
    return expected


def _assert_pose_matrix(pose):
    matrix = np.asarray(pose['matrix_row_major'])
    np.testing.assert_allclose(matrix, _active_matrix(pose), atol=1e-12, rtol=0)
    return matrix


def test_world_pose_matrices_act_on_native_geometry(pose_world):
    # Break caught: a transposed rotation paired with an active translation.
    world = pose_world
    world.set_current_hand_pose([1., 0., 0., 0.], [0., 0., 0.])
    local_vertices = {}
    initial_world_vertices = {}
    for pose in world.get_link_poses():
        key = pose['chain'], max(pose['link'], 0)
        points = np.asarray(world.get_link_vertices_world(*key))
        initial_world_vertices[key] = points
        transform = _active_matrix(pose)
        local_vertices[key] = (points - transform[:3, 3]) @ transform[:3, :3]

    q = Rotation.from_rotvec([.47, -.61, .28]).as_quat(scalar_first=True)
    world.set_current_hand_pose(q.tolist(), [31., -17., 43.])
    hand = _assert_pose_matrix(world.get_current_hand_pose())
    for pose in world.get_link_poses():
        key = pose['chain'], max(pose['link'], 0)
        matrix = _assert_pose_matrix(pose)
        actual = np.asarray(world.get_link_vertices_world(*key))
        expected = local_vertices[key] @ matrix[:3, :3].T + matrix[:3, 3]
        np.testing.assert_allclose(actual, expected, atol=1e-9, rtol=0)
        moved = initial_world_vertices[key] @ hand[:3, :3].T + hand[:3, 3]
        np.testing.assert_allclose(actual, moved, atol=1e-9, rtol=0)


def test_object_pose_matrices_compose_in_world_and_hand_frames(pose_world):
    # Break caught: returning a passive matrix for an active object quaternion.
    world = pose_world
    first_q = Rotation.from_rotvec([-.38, .29, .72]).as_quat(scalar_first=True)
    world.set_current_hand_pose(first_q.tolist(), [-22., 18., 57.])
    world.place_graspable_body_relative_to_hand(0, 23., -11., 149.)
    first_hand = _assert_pose_matrix(world.get_current_hand_pose())
    expected_object = first_hand.copy()
    expected_object[:3, 3] += first_hand[:3, :3] @ [23., -11., 149.]
    object_world = _assert_pose_matrix(world.get_graspable_body_pose(0))
    np.testing.assert_allclose(object_world, expected_object, atol=1e-10, rtol=0)

    second_q = Rotation.from_rotvec([.51, .34, -.46]).as_quat(scalar_first=True)
    world.set_current_hand_pose(second_q.tolist(), [14., -35., 9.])
    hand_world = _assert_pose_matrix(world.get_current_hand_pose())
    object_hand = _assert_pose_matrix(world.get_graspable_body_pose_relative_to_hand(0))
    np.testing.assert_allclose(hand_world @ object_hand, object_world, atol=1e-10, rtol=0)
    np.testing.assert_allclose(world.get_graspable_body_translation(0),
                               object_world[:3, 3], atol=1e-10, rtol=0)
