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


def _concave_hand():
    """Two separated copies of a box mesh with a deep indentation in its roof."""
    return mujoco.MjModel.from_xml_string("""
<mujoco>
  <compiler angle="radian"/>
  <asset>
    <mesh name="indent"
      vertex="-.03 -.03 -.02  .03 -.03 -.02  .03 .03 -.02  -.03 .03 -.02
              -.03 -.03 .02   .03 -.03 .02   .03 .03 .02   -.03 .03 .02  0 0 0"
      face="0 2 1  0 3 2  0 1 5  0 5 4  1 2 6  1 6 5
            2 3 7  2 7 6  3 0 4  3 4 7  4 5 8  5 6 8  6 7 8  7 4 8"/>
  </asset>
  <worldbody>
    <body name="palm" pos="0 .06 .15" euler=".1 .2 .3">
      <geom type="box" size=".02 .02 .02"/>
      <body name="finger" pos=".08 .01 .04" euler="-.2 .14 0">
        <joint name="flex" type="hinge" axis="0 1 0" range="-.75 .75"/>
        <geom name="notch" type="mesh" mesh="indent"/>
        <geom name="neighbor" type="mesh" mesh="indent" pos=".13 0 0" euler="0 0 .25"/>
        <body name="tip" pos="0 -.06 .01" euler=".17 0 0">
          <joint name="tip_flex" type="hinge" axis="1 0 0" range="-.8 .8"/>
          <geom type="box" size=".012 .014 .017"/>
        </body>
      </body>
    </body>
    <body name="probe">
      <freejoint name="probe_joint"/>
      <geom name="probe_sphere" type="sphere" size=".001"/>
    </body>
  </worldbody>
  <equality>
    <joint joint1="tip_flex" joint2="flex" polycoef=".03 .7 0 0 0"/>
  </equality>
</mujoco>
""")


def _off_faces(path):
    lines = path.read_text().splitlines()
    nv, nf, _ = map(int, lines[1].split())
    return np.array([list(map(int, row.split()[1:])) for row in lines[2+nv:2+nv+nf]])


def _assert_outward_triangles(vertices, faces):
    triangles = vertices[faces]
    outward = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    assert np.isfinite(triangles).all()
    assert np.all(np.linalg.norm(outward, axis=1) > 0.)
    interior = vertices.mean(axis=0)
    assert np.all(np.einsum('ij,ij->i', outward, triangles.mean(axis=1) - interior) > 0)


def test_export_compiled_mesh_collision_geometry(tmp_path):
    from graspit_python_wrapper.convert.mjcf_hand import export_body, export_hand, geom_mesh

    model = _concave_hand()
    body = model.body('finger').id
    palm = model.body('palm').id
    notch = model.geom('notch').id
    probe = model.geom('probe_sphere').id
    source_vertices, _ = geom_mesh(model, notch)
    hull_vertices, hull_faces = geom_mesh(model, notch, mesh_collision='convex_hull')
    assert len(source_vertices) == 9
    assert len(hull_vertices) == 8
    assert len(hull_faces) == 12
    _assert_outward_triangles(hull_vertices, hull_faces)
    assert np.min(np.linalg.norm(source_vertices[8] - hull_vertices, axis=1)) > .02

    hand = export_hand(model, 'palm', tmp_path / 'hand', mesh_collision='convex_hull')
    assert hand.dof_joints == ['flex']
    tip = hand.chains[0][1]
    assert (tip.mujoco_joint, tip.dof) == ('tip_flex', 0)
    assert tip.range_rad == pytest.approx((-.8, .8))
    assert (tip.ratio, tip.offset_rad) == pytest.approx((.7, .03))
    off = tmp_path / 'hand/iv/finger.off'
    exported = _read_off(off)
    faces = _off_faces(off)
    assert (len(exported), len(faces)) == (16, 24)
    assert np.all((faces < 8).all(axis=1) | (faces >= 8).all(axis=1))
    _assert_outward_triangles(exported[:8], faces[(faces < 8).all(axis=1)])
    _assert_outward_triangles(exported[8:], faces[(faces >= 8).all(axis=1)] - 8)
    export_body(model, 'finger', tmp_path / 'object', include_descendants=False, mesh_collision='convex_hull')
    assert len(_read_off(tmp_path / 'object/finger.off')) == len(exported)
    default_hand = export_hand(model, 'palm', tmp_path / 'source')
    default_off = default_hand.robot_xml.parent / 'iv/finger.off'
    assert len(_read_off(default_off)) == 18

    data = mujoco.MjData(model)
    chain = load_chains(hand.robot_xml)
    for joint_value in (-.55, 0., .61):
        _set(model, data, hand, [joint_value])
        body_R = data.xmat[body].reshape(3, 3)
        datum = data.xpos[body]
        probe_center = datum + body_R @ np.array([0., 0., .03])
        probe_address = model.jnt_qposadr[model.joint('probe_joint').id]
        data.qpos[probe_address:probe_address + 3] = probe_center
        mujoco.mj_kinematics(model, data)
        witness = np.empty(6)
        distance = mujoco.mj_geomDistance(model, data, notch, probe, .1, witness)

        pose = link_poses(chain, [joint_value])[(0, 0)]
        palm_R = data.xmat[palm].reshape(3, 3)
        hull_world = data.xpos[palm] + ((exported[:8] @ pose[:3, :3].T + pose[:3, 3]) / 1000) @ palm_R.T
        roof_z = ((hull_world - datum) @ body_R)[:, 2].max()
        assert roof_z == pytest.approx(.02, abs=2e-8)
        assert distance == pytest.approx(.03 - roof_z - .001, abs=2e-7)
        assert (witness[:3] - datum) @ body_R[:, 2] == pytest.approx(roof_z, abs=2e-7)


def test_convex_collision_export_rejects_missing_graph():
    from graspit_python_wrapper.convert.mjcf_hand import geom_mesh

    model = _concave_hand()
    geom = model.geom('notch').id
    model.mesh_graphadr[model.geom_dataid[geom]] = -1
    with pytest.raises(ValueError, match='compiled convex hull'):
        geom_mesh(model, geom, mesh_collision='convex_hull')


@pytest.mark.parametrize(('kind', 'plugin'), [('mjGEOM_MESH', 0), ('mjGEOM_SDF', -1)])
def test_convex_collision_export_rejects_plugin_geometry(kind, plugin):
    from types import SimpleNamespace

    from graspit_python_wrapper.convert.mjcf_hand import geom_mesh

    model = SimpleNamespace(
        geom_type=[getattr(mujoco.mjtGeom, kind)], geom_size=[[0., 0., 0.]], geom_plugin=[plugin],
        geom=lambda i: SimpleNamespace(name='example_plugin'),
    )
    with pytest.raises(ValueError, match='plugin'):
        geom_mesh(model, 0, mesh_collision='convex_hull')
