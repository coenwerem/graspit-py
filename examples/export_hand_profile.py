"""Export a prepared MuJoCo hand scene using a documented kinematic profile.

A profile identifies the palm, held coordinates, and expected affine joint map.
The input is MJCF with explicit joint equalities, including imported URDF mimics.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def coupled_dof_bounds(chains, n_dof):
    """Intersect each driver's interval with every affine follower's limits."""
    bounds = np.tile([-np.inf, np.inf], (n_dof, 1))
    for chain in chains:
        for joint in chain:
            lo, hi = joint.range_rad
            if joint.ratio == 0:
                if not lo <= joint.offset_rad <= hi:
                    raise ValueError('infeasible constant follower')
                continue
            a, b = (lo - joint.offset_rad) / joint.ratio, (hi - joint.offset_rad) / joint.ratio
            bounds[joint.dof, 0] = max(bounds[joint.dof, 0], min(a, b))
            bounds[joint.dof, 1] = min(bounds[joint.dof, 1], max(a, b))
    if not np.isfinite(bounds).all() or np.any(bounds[:, 0] > bounds[:, 1]):
        raise ValueError('infeasible or unbounded coupled coordinate interval')
    return bounds


def validate_profile(hand, profile):
    """Check independent-coordinate identities and the declared affine map."""
    joints = {joint.mujoco_joint: joint for chain in hand.chains for joint in chain}
    if len(hand.dof_joints) != profile['independent_dofs'] or len(joints) != profile['hinges']:
        raise ValueError('scene joint counts differ from the profile; verify MJCF equalities and palm root')
    for name, expected in profile.get('couplings', {}).items():
        joint = joints[name]
        if not np.allclose([joint.ratio, joint.offset_rad], expected, atol=1e-10, rtol=0):
            raise ValueError(f'{name}: scene affine coupling differs from profile')
    for name, expected in profile.get('mimic_joints', {}).items():
        joint = joints[name]
        if hand.dof_joints[joint.dof] != expected['driver']:
            raise ValueError(f'{name}: scene coupling driver differs from profile')
        if not np.allclose([joint.ratio, joint.offset_rad],
                           [expected['ratio'], expected['offset_rad']], atol=1e-10, rtol=0):
            raise ValueError(f'{name}: scene affine coupling differs from profile')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('profile', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--mesh-collision', choices=('source', 'convex_hull'), default='convex_hull')
    parser.add_argument('--check', action='store_true', help='query native GraspIt link geometry under Xvfb')
    args = parser.parse_args()
    import mujoco
    from graspit_py.convert.mjcf_hand import export_hand

    profile = json.loads(args.profile.read_text())
    model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
    hand = export_hand(model, profile['palm'], args.output / 'hand',
                       name=profile['id'], preshape_joints=profile.get('preshape_joints', []),
                       mesh_collision=args.mesh_collision)
    validate_profile(hand, profile)
    bounds = coupled_dof_bounds(hand.chains, len(hand.dof_joints))
    configurations = [bounds[:, 0] + fraction * (bounds[:, 1] - bounds[:, 0])
                      for fraction in (.05, .25, .50, .75, .95)]
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'configurations.json').write_text(json.dumps([q.tolist() for q in configurations], indent=2) + '\n')
    result = dict(profile=profile['id'], palm=profile['palm'], dof_joints=hand.dof_joints,
                  limits_rad=bounds.tolist(), mesh_collision=args.mesh_collision,
                  scene_sha256=hashlib.sha256(args.scene.read_bytes()).hexdigest(),
                  profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
                  mujoco=mujoco.__version__, robot_xml=str(hand.robot_xml))
    if args.check:
        from check_export_geometry import compare
        result['geometry'] = compare(model, profile['palm'], hand, configurations, .1,
                                     mesh_collision=args.mesh_collision)
        if not result['geometry']['passed'] or result['geometry']['max_dof_clamp_rad'] > 1e-6:
            raise RuntimeError('native geometry or coupled-limit verification failed')
    (args.output / 'profile-report.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
