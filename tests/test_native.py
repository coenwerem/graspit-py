"""Tests against the native GraspIt build (skipped without one).

Run under a virtual display: ``xvfb-run -a python -m pytest tests``.
The hand is exported from the bundled MJCF fixture. The cube comes from
the upstream models directory pointed to by the GRASPIT variable.
"""
import os
from pathlib import Path

import numpy as np
import pytest

gp = pytest.importorskip('graspit_python_wrapper.core.bindings',
                         reason='native GraspIt bindings are not built')

GRASPIT = Path(os.environ.get('GRASPIT', ''))
CUBE = GRASPIT / 'models/objects/cube.xml'
CUBE_IN_HAND_MM = (0., 0., 50.)

pytestmark = pytest.mark.skipif(not CUBE.exists(),
                                reason='GraspIt models not found (set GRASPIT)')


@pytest.fixture(scope='module')
def world(exported_gripper):
    gp.initialize()
    w = gp.World()
    w.init()
    w.clear()
    assert w.load_robot(str(exported_gripper.robot_xml))
    assert w.load_graspable_body(str(CUBE))
    return w


def _reset(w, pose=((1., 0., 0., 0.), (0., 0., 0.))):
    w.set_current_hand_dof_values([0.] * w.get_current_hand_num_dof())
    w.set_current_hand_pose(list(pose[0]), list(pose[1]))


def test_hand_pose_round_trip(world):
    q = np.array([.9, .1, -.3, .3]); q /= np.linalg.norm(q)
    _reset(world, (q, (12., -4., 30.)))
    pose = world.get_current_hand_pose()
    assert np.allclose(pose['translation'], (12., -4., 30.))
    assert np.allclose(np.abs(np.dot(pose['quat_wxyz'], q)), 1.)


def test_link_poses_and_vertices(world):
    _reset(world)
    poses = world.get_link_poses()
    assert [p['chain'] for p in poses].count(-1) == 1
    assert len(poses) == 1 + sum(1 for p in poses if p['chain'] >= 0)
    for p in poses:
        v = np.asarray(world.get_link_vertices_world(p['chain'], max(p['link'], 0)))
        assert v.ndim == 2 and v.shape[1] == 3 and len(v) > 0


def test_python_kinematics_matches_native(world, exported_gripper):
    from graspit_python_wrapper.kinematics import link_poses, load_chains
    chains = load_chains(exported_gripper.robot_xml)
    rng = np.random.default_rng(0)
    for _ in range(5):
        _reset(world)
        world.set_current_hand_dof_values(list(rng.uniform(0, .5, world.get_current_hand_num_dof())))
        q = world.get_current_hand_dof_values()
        ours = link_poses(chains, q)
        for p in world.get_link_poses():
            if p['chain'] < 0:
                continue
            T = ours[(p['chain'], p['link'])]
            assert np.allclose(T[:3, 3], p['translation'], atol=1e-6)


def test_no_collision_detects_interpenetration(world):
    _reset(world)
    world.place_graspable_body_relative_to_hand(0, 0., 0., 500.)
    assert world.no_collision()
    world.place_graspable_body_relative_to_hand(0, 0., 0., 0.)
    assert not world.no_collision()


def test_closure_contacts_are_on_the_cube_with_inward_normals(world):
    _reset(world)
    world.place_graspable_body_relative_to_hand(0, *CUBE_IN_HAND_MM)
    world.auto_grasp(render_it=False, speed_factor=.8, stop_at_contact=False)
    grasp = gp.Grasp(world, 0)
    assert grasp.compute()
    contacts = list(grasp.object_contact_geometry())
    assert len(contacts) >= 3
    p = np.array([c['position_mm'] for c in contacts])
    n = np.array([c['normal'] for c in contacts])
    assert np.max(np.abs(p)) < 27          # 50 mm cube
    assert np.all(np.sum(p * n, 1) < 0)    # normals point into the object
    assert grasp.epsilon_quality() > 0


def test_exported_mjcf_hand_matches_mujoco_in_graspit(gripper_mjcf, exported_gripper):
    """The converter's output, loaded by native GraspIt, reproduces MuJoCo's geometry."""
    import mujoco
    from scipy.spatial import cKDTree
    model, hand = gripper_mjcf, exported_gripper
    gp.initialize()
    w = gp.World()
    w.init()
    w.clear()
    assert w.load_robot(str(hand.robot_xml))
    w.set_current_hand_pose([1., 0., 0., 0.], [0., 0., 0.])
    data = mujoco.MjData(model)
    dofs = [.7, .4, .9, .3]
    w.set_current_hand_dof_values(dofs)
    for chain in hand.chains:
        for j in chain:
            data.qpos[model.jnt_qposadr[model.joint(j.mujoco_joint).id]] = j.ratio * dofs[j.dof] + j.offset_rad
    mujoco.mj_kinematics(model, data)
    from graspit_python_wrapper.convert.mjcf_hand import geom_mesh
    for (c, l), body in hand.links.items():
        g = np.asarray(w.get_link_vertices_world(c, l))
        bid = model.body(body).id
        pts = [data.geom_xpos[k] + geom_mesh(model, k)[0] @ data.geom_xmat[k].reshape(3, 3).T
               for k in range(model.ngeom) if model.geom_bodyid[k] == bid]
        m = np.vstack(pts) * 1000
        d, _ = cKDTree(g).query(m)
        assert d.max() < 0.1, body
