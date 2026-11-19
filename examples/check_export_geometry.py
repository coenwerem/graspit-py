"""Compare an exported hand's collision vertices in MuJoCo and native GraspIt.

Run under Xvfb with the installed wrapper and a compiled-model input scene.
The configurations file lists independent joint coordinates in export order.
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial import cKDTree

from graspit_python_wrapper.convert.mjcf_hand import export_hand, geom_mesh
from graspit_python_wrapper.core import bindings as gp


def compare(model, palm_name, hand, configs, tolerance, *, mesh_collision='source'):
    palm = model.body(palm_name).id
    data = mujoco.MjData(model)
    exported_joints = {model.joint(joint.mujoco_joint).id
                       for chain in hand.chains for joint in chain}
    independent = [model.jnt_qposadr[model.joint(name).id] for name in hand.dof_joints]
    equalities = []
    for i in range(model.neq):
        driven, driver = model.eq_obj1id[i], model.eq_obj2id[i]
        if model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT and driven in exported_joints and driver >= 0:
            equalities.append((model.jnt_qposadr[driven], model.jnt_qposadr[driver], i))
    link_names = {(-1, -1): palm_name, **hand.links}
    owners = {model.body(name).id: name for name in link_names.values()}
    geoms = {name: [] for name in link_names.values()}
    for geom in range(model.ngeom):
        if not (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        body = model.geom_bodyid[geom]
        while body and body not in owners and not model.body_jntnum[body]:
            body = model.body_parentid[body]
        if body not in owners:
            continue
        mesh = geom_mesh(model, geom, mesh_collision=mesh_collision)
        if mesh is not None:
            vertices, faces = mesh
            geoms[owners[body]].append((geom, vertices[np.unique(faces)]))

    gp.initialize()
    world = gp.World()
    world.init()
    if not world.load_robot(str(hand.robot_xml.resolve())):
        raise RuntimeError(f"Cannot load {hand.robot_xml}")
    world.set_current_hand_pose([1., 0., 0., 0.], [0., 0., 0.])
    worst = dict.fromkeys(link_names.values(), 0.0)
    clamp = 0.0
    for requested in configs:
        world.set_current_hand_dof_values(requested.tolist())
        actual = np.asarray(world.get_current_hand_dof_values())
        clamp = max(clamp, float(np.max(np.abs(actual - requested))))
        data.qpos[:] = model.qpos0
        data.qpos[independent] = actual
        # Reconstruct the source coordinates from MJCF independently of the export offsets.
        for driven, driver, equality in equalities:
            c0, c1 = model.eq_data[equality, :2]
            data.qpos[driven] = model.qpos0[driven] + c0 + c1 * (data.qpos[driver] - model.qpos0[driver])
        mujoco.mj_forward(model, data)
        palm_position = data.xpos[palm]
        palm_rotation = data.xmat[palm].reshape(3, 3)
        poses = {(pose['chain'], pose['link']): pose for pose in world.get_link_poses()}
        native_origin = np.asarray(poses[(-1, -1)]['translation'])
        for (chain, link), name in link_names.items():
            native = np.asarray(world.get_link_vertices_world(chain, max(0, link))) - native_origin
            source = np.concatenate([
                data.geom_xpos[g] + vertices @ data.geom_xmat[g].reshape(3, 3).T
                for g, vertices in geoms[name]
            ])
            source = (source - palm_position) @ palm_rotation * 1000
            distance = max(cKDTree(native).query(source)[0].max(),
                           cKDTree(source).query(native)[0].max())
            worst[name] = max(worst[name], float(distance))
    maximum = max(worst.values())
    return dict(mesh_collision=mesh_collision, configurations=len(configs), links=len(worst),
                worst_link_mm=worst,
                max_mm=maximum, max_dof_clamp_rad=clamp, passed=maximum <= tolerance)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('palm')
    parser.add_argument('configs', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--tol-mm', type=float, default=0.1)
    parser.add_argument('--mesh-collision', choices=('source', 'convex_hull'), default='source',
                        help='mesh representation to export and compare (default: source)')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    args.output.mkdir(parents=True, exist_ok=True)
    hand = export_hand(model, args.palm, args.output / 'hand', mesh_collision=args.mesh_collision)
    configs = np.asarray(json.loads(args.configs.read_text()), dtype=float)
    if (configs.ndim != 2 or not len(configs) or configs.shape[1] != len(hand.dof_joints)
            or not np.isfinite(configs).all()):
        parser.error('expected finite configuration rows in exported DOF order')
    report = compare(model, args.palm, hand, configs, args.tol_mm, mesh_collision=args.mesh_collision)
    (args.output / 'geometry-check.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
