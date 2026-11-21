"""Export world-frame link geometry of a GraspIt hand at given joint values.

Places the hand base at the identity pose, sets each requested DOF vector
(radians, GraspIt DOF order) and writes every link's vertices (mm) to JSON.
Used to check a GraspIt hand model against another simulator's model of the
same hand.

    xvfb-run -a python examples/export_hand_geometry.py \
        --robot hand.xml \
        --dofs configs.json --out geometry.json
"""
import argparse
import json
from pathlib import Path

import numpy as np

from graspit_py.core import bindings as gp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', type=Path, required=True)
    parser.add_argument('--dofs', type=Path, required=True,
                        help='JSON list of DOF vectors (radians, GraspIt order)')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()

    configs = json.loads(args.dofs.read_text())
    gp.initialize()
    world = gp.World()
    world.init()
    assert world.load_robot(str(args.robot.resolve()))
    world.set_current_hand_pose([1., 0., 0., 0.], [0., 0., 0.])

    records = []
    for q in configs:
        world.set_current_hand_dof_values(list(map(float, q)))
        links = []
        for pose in world.get_link_poses():
            verts = np.asarray(world.get_link_vertices_world(pose['chain'], max(pose['link'], 0)))
            links.append(dict(chain=pose['chain'], link=pose['link'], name=pose['name'],
                              quat_wxyz=list(pose['quat_wxyz']),
                              translation_mm=list(pose['translation']),
                              vertices_mm=verts.round(4).tolist()))
        records.append(dict(dof_requested=list(map(float, q)),
                            dof_actual=list(world.get_current_hand_dof_values()),
                            links=links))
    args.out.write_text(json.dumps(dict(robot=str(args.robot), units='mm', configs=records)))
    print(f'wrote {len(records)} configurations, {len(records[0]["links"])} links each')


if __name__ == '__main__':
    main()
