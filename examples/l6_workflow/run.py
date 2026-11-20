"""Export an L6 hand, plan on a cube, and restore a selected grasp in MuJoCo."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from graspit_python_wrapper.convert.mjcf_hand import export_body, export_hand
from graspit_python_wrapper.core import bindings as gp
from graspit_python_wrapper.planning import ContactPlanner, HandFrame, PlannerConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot", type=Path, help="compiled L6 model with resolved mesh paths")
    parser.add_argument("output", type=Path)
    parser.add_argument("--assets", type=Path, help="root for relative mesh paths in the supplied scene")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    palm_name = "l6_hand_base_link"
    if args.assets is None:
        spec = mj.MjSpec.from_file(str(args.robot.resolve()))
    else:
        root = ET.parse(args.robot).getroot()
        for mesh in root.findall("./asset/mesh[@file]"):
            mesh.set("file", str((args.assets / mesh.attrib["file"]).resolve(strict=True)))
        spec = mj.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    cube = spec.worldbody.add_body(name="workflow_cube")
    cube.add_freejoint(name="workflow_cube")
    cube.add_geom(type=mj.mjtGeom.mjGEOM_BOX, size=(.025, .025, .025), mass=.1)
    model = spec.compile()

    hand = export_hand(model, palm_name, out / "hand", name="l6",
                       preshape_joints=["l6_thumb_cmc_roll"])
    object_xml = export_body(model, "workflow_cube", out / "cube")
    settings = json.loads(Path(__file__).with_name("problem.json").read_text())
    gp.initialize()
    world = gp.World()
    world.init()
    world.clear()
    if not (world.load_robot(str(hand.robot_xml)) and world.load_graspable_body(str(object_xml))):
        raise RuntimeError("Cannot load exported hand and cube in GraspIt!")
    planner = ContactPlanner(world, 0, PlannerConfig(**settings["planner"]),
                             hand=HandFrame(**settings["hand"]))
    candidates = planner.plan(top_k=5)
    if not candidates or candidates[0].score <= 0:
        raise RuntimeError("No candidate has positive lower-tail quality")
    selected = candidates[0].to_dict()
    manifest = json.loads((out / "hand/l6.json").read_text())

    # Set independent and coupled coordinates by MuJoCo joint name.
    data = mj.MjData(model)
    for chain in manifest["chains"]:
        for joint in chain:
            d = selected["dofs_rad"][joint["dof"]]
            data.joint(joint["mujoco_joint"]).qpos[0] = joint["ratio"] * d + joint["offset_rad"]
    mj.mj_forward(model, data)
    palm = data.body(palm_name)
    R_WH = palm.xmat.reshape(3, 3)
    T_OH = np.asarray(selected["T_object_hand_mm"])
    R_WO = R_WH @ T_OH[:3, :3].T
    p_WO = palm.xpos - R_WO @ (T_OH[:3, 3] * .001)
    data.joint("workflow_cube").qpos[:] = np.r_[
        p_WO, Rotation.from_matrix(R_WO).as_quat(scalar_first=True)]
    mj.mj_forward(model, data)

    cube_pose = data.body("workflow_cube")
    R_OW = cube_pose.xmat.reshape(3, 3).T
    palm_mm = R_OW @ (palm.xpos - cube_pose.xpos) * 1000
    translation_error = np.linalg.norm(palm_mm - T_OH[:3, 3])
    rotation_error = Rotation.from_matrix((R_OW @ R_WH).T @ T_OH[:3, :3]).magnitude()
    if translation_error > 1e-6 or rotation_error > 1e-9:
        raise RuntimeError("The reconstructed object-relative palm pose disagrees with the candidate")
    result = {
        "candidates": [c.to_dict() for c in candidates],
        "joint_names": manifest["dof_joints"],
        "selected_rank": 1,
        "palm_translation_error_mm": float(translation_error),
        "palm_rotation_error_rad": float(rotation_error),
        "mujoco_qpos": data.qpos.tolist(),
    }
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"Rank 1: {len(selected['contacts'])} contacts, "
          f"nominal epsilon {selected['nominal_epsilon']:.6f}, "
          f"lower-tail epsilon {selected['score']:.6f}")


if __name__ == "__main__":
    main()
