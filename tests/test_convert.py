"""MJCF -> GraspIt hand export, checked with the pure-Python kinematics."""
import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
from graspit_python_wrapper.kinematics import link_poses, load_chains  # noqa: E402


def _read_off(path):
    lines = path.read_text().split('\n')
    nv = int(lines[1].split()[0])
    return np.array([list(map(float, l.split())) for l in lines[2:2 + nv]])


def _set(model, data, hand, dofs):
    for chain in hand.chains:
        for j in chain:
            data.qpos[model.jnt_qposadr[model.joint(j.mujoco_joint).id]] = j.ratio * dofs[j.dof] + j.offset_rad
    mujoco.mj_kinematics(model, data)


def _geom_vertices(model, data, body, ref):
    """Collision-geom vertices of ``body`` in the palm frame (mm), in export order."""
    from graspit_python_wrapper.convert.mjcf_hand import geom_mesh
    out = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == model.body(body).id:
            v, _ = geom_mesh(model, g)
            w = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
            out.append((w - data.xpos[ref]) @ data.xmat[ref].reshape(3, 3) * 1000)
    return np.vstack(out)


def test_dofs_and_couplings(exported_gripper):
    hand = exported_gripper
    assert hand.dof_joints == ['f1_mcp', 'f2_mcp', 'th_mcp', 'th_dip']
    coupled = {j.mujoco_joint: j for c in hand.chains for j in c if j.ratio != 1}
    assert set(coupled) == {'f1_dip', 'f2_dip'}
    assert coupled['f1_dip'].ratio == pytest.approx(1.1)


@pytest.mark.parametrize('seed', range(5))
def test_link_geometry_matches_mujoco(gripper_mjcf, exported_gripper, seed):
    model, hand = gripper_mjcf, exported_gripper
    data = mujoco.MjData(model)
    rng = np.random.default_rng(seed)
    dofs = rng.uniform([0, 0, 0, -.5], [1.2, 1.2, 1.2, .5])
    _set(model, data, hand, dofs)
    chains = load_chains(hand.robot_xml)
    T = link_poses(chains, dofs)
    palm = model.body('palm').id
    for (c, l), body in hand.links.items():
        off = _read_off(hand.robot_xml.parent / 'iv' / f'{body}.off')
        g = off @ T[(c, l)][:3, :3].T + T[(c, l)][:3, 3]
        m = _geom_vertices(model, data, body, palm)
        assert np.max(np.linalg.norm(g[:len(m)] - m, axis=1)) < 1e-2, body


def test_nearly_parallel_axes_stay_near_the_palm(exported_gripper):
    chains = load_chains(exported_gripper.robot_xml)
    assert all(abs(j.d) < 200 for c in chains for j in c.joints)


def test_export_body_keeps_body_frame(gripper_mjcf, tmp_path):
    from graspit_python_wrapper.convert.mjcf_hand import export_body
    xml = export_body(gripper_mjcf, 'f1_dist', tmp_path)
    v = _read_off(tmp_path / 'f1_dist.off')
    # f1_dist's capsule runs 0..30 mm along its body z (radius 7) plus the 7 mm tip sphere
    assert v[:, 2].min() == pytest.approx(-7, abs=.5)
    assert v[:, 2].max() == pytest.approx(37, abs=.5)
    assert '<cog>' in xml.read_text()


def test_affine_coupling_with_nonzero_reference_angles(tmp_path):
    from pathlib import Path

    from graspit_python_wrapper.convert.mjcf_hand import export_hand

    scene = Path(__file__).parent / 'data' / 'gripper-nonzero-ref.xml'
    model = mujoco.MjModel.from_xml_path(str(scene))
    hand = export_hand(model, 'palm', tmp_path)
    assert hand.dof_joints == ['driver']
    driven = hand.chains[0][1]
    assert driven.offset_rad == pytest.approx(-0.285)
    assert driven.ratio == pytest.approx(0.70)

    chains = load_chains(hand.robot_xml)
    data = mujoco.MjData(model)
    palm = model.body('palm').id
    for driver in (-0.2, 0.35, 0.8):
        data.joint('driver').qpos[:] = driver
        data.joint('driven').qpos[:] = -0.20 + 0.16 + 0.70 * (driver - 0.35)
        mujoco.mj_forward(model, data)
        equality = data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        assert data.efc_pos[equality] == pytest.approx([0.0], abs=1e-12)

        poses = link_poses(chains, [driver])
        for (chain, link), body in hand.links.items():
            off = _read_off(tmp_path / 'iv' / f'{body}.off')
            pose = poses[(chain, link)]
            exported = off @ pose[:3, :3].T + pose[:3, 3]
            expected = _geom_vertices(model, data, body, palm)
            np.testing.assert_allclose(exported, expected, atol=1e-7, rtol=0)
