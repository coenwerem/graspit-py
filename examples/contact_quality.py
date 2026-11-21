"""Close a hand on an object and save its object-frame contacts and quality."""
import argparse
import hashlib
import json
from pathlib import Path
from graspit_py.core import bindings as gp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', type=Path, required=True, help='GraspIt robot XML')
    parser.add_argument('--object', type=Path, required=True, help='GraspIt object XML')
    parser.add_argument('--object-position-mm', type=float, nargs=3, required=True,
                        metavar=('X', 'Y', 'Z'), help='Object position in the hand frame')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error('output already exists')
    hand, obj = args.robot.resolve(), args.object.resolve()
    gp.initialize()
    world = gp.World()
    world.init()
    if not world.load_robot(str(hand)):
        parser.error(f'could not load robot {hand}')
    if not world.load_graspable_body(str(obj)):
        parser.error(f'could not load object {obj}')
    world.place_graspable_body_relative_to_hand(0, *args.object_position_mm)
    world.auto_grasp(render_it=False, speed_factor=.8, stop_at_contact=False)
    grasp = gp.Grasp(world, 0)
    if not grasp.compute():
        parser.error('closure did not produce a grasp for quality evaluation')
    contacts = list(grasp.object_contact_geometry())
    record = dict(
        schema_version=1, hand_dof_rad=list(world.get_current_hand_dof_values()),
        object_pose_world=world.get_graspable_body_pose(0),
        hand_pose_world=world.get_current_hand_pose(),
        contacts_object_frame=contacts, object_cog_mm=list(world.get_graspable_body_cog_local(0)),
        units={'translation':'mm','joint_angle':'rad','quaternion':'wxyz',
               'quality':'native GraspIt normalized wrench metric'},
        epsilon=float(grasp.epsilon_quality()), volume=float(grasp.volume_quality()),
        assets={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (hand,obj)},
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(record, indent=2))
    print(json.dumps({k:record[k] for k in ('epsilon','volume')}), 'contacts',len(contacts))

if __name__ == '__main__':
    main()
