"""Tests that need no GraspIt build."""
import numpy as np
import pytest

from graspit_python_wrapper.core.types import ContactPoint
from graspit_python_wrapper.grasp.quality import (analyze_gws, build_grasp_matrix_from_contacts,
                                                  compute_task_wrench_quality, min_weight_force_closure,
                                                  task_wrenches_from_forces)
from graspit_python_wrapper.risk import cvar, lower_tail_mean


def test_cvar_fractional_boundary():
    assert cvar([0., 10.], alpha=.75) == pytest.approx(20 / 3)


def test_cvar_weighted_tail_and_mean():
    assert cvar([0., 10.], alpha=.5, weights=[.9, .1]) == pytest.approx(2.)
    assert cvar([0., 10.], alpha=1, weights=[.9, .1]) == pytest.approx(1.)


def test_cvar_continuous_in_tail_mass():
    x = np.arange(10.)
    values = [cvar(x, alpha=a) for a in np.linspace(.05, 1, 200)]
    assert np.all(np.diff(values) <= 1e-12)
    assert np.max(np.abs(np.diff(values))) < .5


@pytest.mark.parametrize('bad', [0., 1.5, -1])
def test_cvar_rejects_bad_alpha(bad):
    with pytest.raises(ValueError):
        cvar([1., 2.], alpha=bad)


def test_lower_tail_mean():
    assert lower_tail_mean([1., 2., 3., 4.], alpha=.5) == pytest.approx(1.5)
    assert lower_tail_mean([1., 2., 3., 4.], alpha=1.) == pytest.approx(2.5)


def _cube_contacts(half=25.):
    pts, normals = [], []
    for axis in range(3):
        for sign in (-1, 1):
            p = np.zeros(3); p[axis] = sign * half
            pts.append(p); normals.append(-p / half)
    return [ContactPoint(p, n) for p, n in zip(pts, normals)]


def test_six_face_contacts_are_force_closure():
    res = analyze_gws(_cube_contacts(), np.zeros(3), friction_coef=.5)
    assert res.is_force_closure and res.epsilon > 0


def test_two_antipodal_points_without_torsion_are_not_closure():
    contacts = _cube_contacts()[:2]
    assert not analyze_gws(contacts, np.zeros(3), friction_coef=.5).is_force_closure


def test_epsilon_is_monotone_in_friction():
    contacts = _cube_contacts()
    eps = [analyze_gws(contacts, np.zeros(3), friction_coef=mu).epsilon
           for mu in np.linspace(.1, 1.5, 15)]
    assert np.all(np.diff(eps) > 0)


def test_task_quality_is_the_ray_distance_to_the_hull():
    G = np.hstack([np.eye(6), -np.eye(6)])          # L1 ball: |w|_1 <= 1
    e = np.eye(6)
    assert compute_task_wrench_quality(G, e[0]) == pytest.approx(1.)
    d = (e[0] + e[1]) / np.sqrt(2)
    assert compute_task_wrench_quality(G, d) == pytest.approx(1 / np.sqrt(2))
    assert compute_task_wrench_quality(G, np.vstack([e[0], d])) == pytest.approx(1 / np.sqrt(2))
    assert compute_task_wrench_quality(G[:, :11], e[0]) == 0.        # origin on the boundary


def test_task_quality_does_not_depend_on_torque_scale():
    contacts = _cube_contacts(half=20.)
    tip = task_wrenches_from_forces([[60., 0., 0.]], [[0., 1., 0.]], np.zeros(3), 1.)
    q = []
    for scale in (10., 100.):
        G = build_grasp_matrix_from_contacts(contacts, np.zeros(3), .5, max_radius=scale)
        W = tip.copy(); W[:, 3:] /= scale
        q.append(compute_task_wrench_quality(G, W))
    assert q[0] > 0 and q[0] == pytest.approx(q[1], rel=1e-6)


def test_task_quality_falls_with_the_load_lever_arm():
    contacts = _cube_contacts(half=20.)
    G = build_grasp_matrix_from_contacts(contacts, np.zeros(3), .5, max_radius=100.)
    q = [compute_task_wrench_quality(G, task_wrenches_from_forces([[x, 0., 0.]], [[0., 1., 0.]],
                                                                  np.zeros(3), 100.))
         for x in (0., 50., 150.)]
    assert q[0] > q[1] > q[2] > 0


@pytest.mark.parametrize('rank,repetitions,expected_margin', [(1, 6, 1 / 12), (5, 1, 1 / 10)])
def test_min_weight_positive_margin_requires_full_wrench_rank(rank, repetitions, expected_margin):
    # Uniform weights balance each antipodal pair and attain the LP upper bound
    # 1/n, but unspanned wrench directions prevent six-dimensional closure.
    basis = np.eye(6)[:, :rank]
    W = np.tile(np.hstack([basis, -basis]), (1, repetitions))
    margin, is_force_closure = min_weight_force_closure(W)
    assert margin == pytest.approx(expected_margin)
    assert not is_force_closure


def test_min_weight_full_rank_balanced_wrenches_are_force_closure():
    # The 12 vertices +/-e_i span R^6 and attain the same uniform-weight optimum.
    W = np.hstack([np.eye(6), -np.eye(6)])
    margin, is_force_closure = min_weight_force_closure(W)
    assert margin == pytest.approx(1 / 12)
    assert is_force_closure
