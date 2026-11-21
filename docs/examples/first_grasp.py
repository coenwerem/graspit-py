"""Export a hand and cube from MuJoCo and record a closure in GraspIt!."""

import argparse
import json
from pathlib import Path

import mujoco

from graspit_py.convert.mjcf_hand import export_body, export_hand
from graspit_py.core import bindings

GRIPPER = Path(__file__).resolve().parents[2] / "tests" / "data" / "gripper.xml"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    hand_model = mujoco.MjModel.from_xml_path(str(GRIPPER))
    cube_model = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <worldbody>
          <body name="cube">
            <geom type="box" size="0.020 0.020 0.020" mass="0.100"/>
          </body>
        </worldbody>
      </mujoco>
    """)
    hand = export_hand(hand_model, "palm", args.output / "hand")
    cube = export_body(cube_model, "cube", args.output / "object")

    bindings.initialize()
    world = bindings.World()
    world.clear()
    if not world.load_robot(str(hand.robot_xml.resolve())):
        raise RuntimeError(f"could not load {hand.robot_xml}")
    if not world.load_graspable_body(str(cube.resolve())):
        raise RuntimeError(f"could not load {cube}")

    world.set_current_hand_dof_values([0.0] * len(hand.dof_joints))
    world.place_graspable_body_relative_to_hand(0, 0.0, 0.0, 65.0)
    world.auto_grasp(render_it=False, speed_factor=0.8, stop_at_contact=False)

    grasp = bindings.Grasp(world, 0)
    if not grasp.compute():
        raise RuntimeError("GraspIt! could not compute the grasp")
    contacts = list(grasp.object_contact_geometry())
    if not contacts:
        raise RuntimeError("the closure made no object contacts")

    record = {
        "joint_names": hand.dof_joints,
        "dofs_rad": world.get_current_hand_dof_values(),
        "contacts_object_frame": contacts,
        "epsilon": grasp.epsilon_quality(),
        "volume": grasp.volume_quality(),
    }
    (args.output / "grasp.json").write_text(json.dumps(record, indent=2) + "\n")
    print(f"{len(contacts)} contacts -> {args.output / 'grasp.json'}")


if __name__ == "__main__":
    main()
