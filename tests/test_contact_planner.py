"""Contact planner on the exported test gripper and GraspIt's cube."""
import os
from pathlib import Path

import numpy as np
import pytest

gp = pytest.importorskip('graspit_python_wrapper.core.bindings',
                         reason='native GraspIt bindings are not built')
from graspit_python_wrapper.planning import ContactPlanner, HandFrame, PlannerConfig  # noqa: E402

CUBE = Path(os.environ.get('GRASPIT', '')) / 'models/objects/cube.xml'
pytestmark = pytest.mark.skipif(not CUBE.exists(), reason='GraspIt models not found (set GRASPIT)')
GRIPPER = HandFrame(approach_axis=(0, 0, 1), finger_axis=(1, 0, 0), grasp_center_mm=(0, 0, 50))


@pytest.fixture(scope='module')
def world(exported_gripper):
    gp.initialize()
    w = gp.World()
    w.init()
    w.clear()
    assert w.load_robot(str(exported_gripper.robot_xml))
    assert w.load_graspable_body(str(CUBE))
    return w


def _planner(world, **kw):
    base = dict(n_directions=10, n_rolls=2, n_pose_samples=2, standoff_mm=90, seed=3)
    base.update(kw)
    return ContactPlanner(world, 0, PlannerConfig(**base), hand=GRIPPER)


def test_planner_returns_scored_candidates(world):
    cands = _planner(world).plan()
    assert cands, 'no candidate grasp found on the cube'
    scores = [c.score for c in cands]
    assert scores == sorted(scores, reverse=True)
    for c in cands:
        assert c.epsilon_samples.shape == (3, 9)
        assert c.score <= c.expected_epsilon + 1e-12
        R = c.T_object_hand[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert len(c.contacts) >= 3


def test_risk_neutral_tail_equals_mean(world):
    for c in _planner(world, tail_mass=1.0).plan():
        assert c.score == pytest.approx(c.expected_epsilon)


def test_support_plane_rejects_grasps_from_below(world):
    free = _planner(world).plan()
    table = _planner(world, support_normal=(0., 0., 1.), support_offset_mm=-25.).plan()
    assert len(table) < len(free)
    # approach_dir is the hand's direction of travel; +z means coming from below
    assert all(c.approach_dir[2] < .7 for c in table)


def test_contact_region_restricts_contacts(world):
    region = [[-30., -30., 0.], [30., 30., 30.]]          # top half of the cube
    for c in _planner(world, contact_region_mm=region).plan():
        p = np.array([k.position for k in c.contacts])
        assert np.all(p[:, 2] >= 0.)


def test_keepout_box_rejects_hands_inside_it(world):
    free = _planner(world).plan()
    box = [[[0., 0., 60.], [60., 60., 35.]]]                # slab above the cube's top face
    kept = _planner(world, keepout_boxes_mm=box).plan()
    assert len(kept) < len(free)
    planner = _planner(world, keepout_boxes_mm=box)
    for c in kept:
        planner._place(c.T_object_hand)
        world.set_current_hand_dof_values(c.dofs.tolist())
        assert planner.clear_of_keepout()


def test_closing_sets_hold_the_other_dofs(world):
    n = world.get_current_hand_num_dof()
    before = list(world.get_dof_default_velocities())
    planner = _planner(world, closing_sets=[[0]])
    T = planner.hand_pose(np.array([0., 0., -1.]), 0., 90.)
    T_pre = planner.approach(T, np.array([0., 0., -1.]))
    assert T_pre is not None
    planner.closing = [0]
    planner.close(T_pre)
    dofs = np.asarray(world.get_current_hand_dof_values())
    assert np.allclose(dofs[1:], planner.open_dofs[1:])
    assert list(world.get_dof_default_velocities()) == before      # restored after closing
    assert n > 1


def test_explicit_approaches_replace_the_sphere(world):
    from graspit_python_wrapper.planning import palm_pose_for_approach
    approaches = [[0., 0., -1., 0.], [0., 0., -1., 90.]]        # from above only
    planner = _planner(world, approaches=approaches)
    for c in planner.plan():
        assert np.allclose(c.approach_dir, [0., 0., -1.])
        T = palm_pose_for_approach(c.approach_dir, c.roll_deg, 0., GRIPPER, planner.center)
        assert np.allclose(T[:3, :3], c.T_object_hand[:3, :3])     # approach keeps orientation


def test_task_forces_score_the_task_quality(world):
    task = [[0., 0., 60., 1., 0., 0.], [0., 0., 60., 0., 1., 0.]]
    cands = _planner(world, task_forces_mm=task).plan()
    assert cands
    for c in cands:
        assert c.task_samples.shape == c.epsilon_samples.shape
        assert np.all(c.task_samples >= 0)
        assert c.score <= c.expected_task + 1e-12
        assert c.epsilon_cvar <= c.expected_epsilon + 1e-12
        assert c.to_dict()['task_samples'] is not None
    plain = _planner(world).plan()
    assert all(c.task_samples is None and c.score == c.epsilon_cvar for c in plain)


def test_samples_do_not_depend_on_other_candidates(world):
    c = _planner(world).plan()[0]
    one = _planner(world, approaches=[[*c.approach_dir, c.roll_deg]]).plan()
    assert np.allclose(one[0].dofs, c.dofs)
    assert np.allclose(one[0].epsilon_samples, c.epsilon_samples)
