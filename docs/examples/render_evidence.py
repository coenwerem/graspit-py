"""Capture collision meshes from the documented examples and render them as PNGs."""

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load_json(path):
    return json.loads(path.read_text())


def off_triangles(path):
    with path.open() as stream:
        if stream.readline().strip() != "OFF":
            raise ValueError(f"expected OFF mesh: {path}")
        nvertices, nfaces, _ = map(int, stream.readline().split())
        vertices = np.array([stream.readline().split() for _ in range(nvertices)], float)
        faces = np.array([stream.readline().split()[1:] for _ in range(nfaces)], int)
    return vertices[faces]


def capture(folder, output):
    import mujoco
    from scipy.spatial.transform import Rotation
    from graspit_python_wrapper.convert.mjcf_hand import geom_mesh
    from graspit_python_wrapper.core import bindings

    manifest = load_json(folder / "hand" / "hand.json")
    candidates = load_json(folder / "candidates.json")["candidates"]
    model = mujoco.MjModel.from_xml_path(str(ROOT / "tests/data/gripper.xml"))
    data = mujoco.MjData(model)
    config = load_json(ROOT / "examples/parity-test-configs.json")[1]
    for chain in manifest["chains"]:
        for link in chain:
            data.joint(link["mujoco_joint"]).qpos[:] = (
                config[link["dof"]] * link["ratio"] + link["offset_rad"]
            )
    mujoco.mj_kinematics(model, data)
    source = []
    for i in range(model.ngeom):
        vertices, faces = geom_mesh(model, i)
        source.extend(((vertices @ data.geom_xmat[i].reshape(3, 3).T
                        + data.geom_xpos[i]) * 1000)[faces])

    bindings.initialize()
    world = bindings.World()
    world.init()
    if not world.load_robot(str((folder / "hand/hand.xml").resolve())):
        raise RuntimeError("could not load exported hand")
    meshes = {"-1,-1": off_triangles(folder / "hand/iv/palm.off")}
    for link, name in manifest["links"].items():
        meshes[link] = off_triangles(folder / "hand" / "iv" / f"{name}.off")

    def native_triangles():
        result = []
        for pose in world.get_link_poses():
            angles = Rotation.from_quat(pose["quat_wxyz"], scalar_first=True).as_matrix()
            triangles = meshes[f'{pose["chain"]},{pose["link"]}']
            result.extend(triangles @ angles.T + pose["translation"])
        return np.asarray(result)

    world.set_current_hand_pose([1, 0, 0, 0], [0, 0, 0])
    world.set_current_hand_dof_values(config)
    np.testing.assert_allclose(world.get_current_hand_dof_values(), config, atol=1e-8)
    record = {"source_mm": np.asarray(source), "export_mm": native_triangles(),
              "cube_mm": off_triangles(folder / "object/cube.off")}
    for index, candidate in enumerate(candidates[:2]):
        transform = np.asarray(candidate["T_object_hand_mm"])
        orientation = Rotation.from_matrix(transform[:3, :3]).as_quat(scalar_first=True)
        world.set_current_hand_pose(orientation.tolist(), transform[:3, 3].tolist())
        world.set_current_hand_dof_values(candidate["dofs_rad"])
        np.testing.assert_allclose(world.get_current_hand_dof_values(), candidate["dofs_rad"], atol=1e-8)
        record[f"hand_{index}"] = native_triangles()
        record[f"contacts_{index}"] = [c["position_mm"] for c in candidate["contacts"]]
    np.savez_compressed(output, **record)


def render(capture_file, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    def mesh(ax, triangles, color):
        cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normal = cross / np.maximum(np.linalg.norm(cross, axis=1, keepdims=True), 1e-12)
        light = np.array([-.40, -.65, .65])
        shading = .58 + .42 * np.abs(normal @ light)
        colors = np.asarray(to_rgb(color))[None, :] * shading[:, None]
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, linewidths=0))

    with np.load(capture_file) as arrays:
        for stem, keys in (("mujoco-export", ("source_mm", "export_mm")),
                           ("generated-grasps", ("hand_0", "hand_1"))):
            fig, axes = plt.subplots(1, 2, figsize=(8, 3.9), subplot_kw={"projection": "3d"})
            points = np.concatenate([arrays[key].reshape(-1, 3) for key in keys])
            if stem == "generated-grasps":
                points = np.vstack((points, arrays["cube_mm"].reshape(-1, 3)))
            center = (points.max(axis=0) + points.min(axis=0)) / 2
            radius = np.ptp(points, axis=0).max() * .48
            for i, (ax, key) in enumerate(zip(axes, keys)):
                mesh(ax, arrays[key], "#607996")
                if stem == "generated-grasps":
                    mesh(ax, arrays["cube_mm"], "#53b1b1")
                    ax.plot(*arrays[f"contacts_{i}"].T, linestyle="none", marker="o",
                            color="#d35b22", markeredgecolor="white", markeredgewidth=.8,
                            markersize=5.5, zorder=100)
                ax.set(xlim=(center[0]-radius, center[0]+radius),
                       ylim=(center[1]-radius, center[1]+radius),
                       zlim=(center[2]-radius, center[2]+radius))
                ax.set_box_aspect((1, 1, 1), zoom=1.24)
                ax.set_proj_type("ortho")
                ax.view_init(elev=32 if stem == "generated-grasps" else 21, azim=-70)
                ax.set_axis_off()
                ax.text2D(.5, .025, f"({chr(ord('a') + i)})", transform=ax.transAxes,
                          ha="center", fontsize=13)
            fig.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0)
            fig.savefig(output / f"{stem}.png", dpi=200, facecolor="white")
            plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("capture", "render"))
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.stage == "capture":
        capture(args.input, args.output)
    else:
        render(args.input, args.output)
