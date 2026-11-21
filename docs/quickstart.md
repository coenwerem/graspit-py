# Grasp Planning with a RealHand L6

This walkthrough exports a six-coordinate right RealHand L6 and
plans grasps on a 50 mm cube. It requires the image from
[Installation](INSTALL.md) and a local right-hand MuJoCo scene with
collision meshes. The reference hand uses palm body
`l6_hand_base_link` and the following independent joint order.

```text
l6_thumb_cmc_roll
l6_thumb_cmc_pitch
l6_index_mcp_pitch
l6_middle_mcp_pitch
l6_ring_mcp_pitch
l6_pinky_mcp_pitch
```

## Export the Hand and Cube

Place `scene.xml` and the associated meshes under `input/` and
preserve the relative mesh paths. Check the palm and thumb joint
names against the source scene. Run the export from the repository
root.

```bash
mkdir -p output
docker run --rm -i --user "$(id -u):$(id -g)" \
  -v "$PWD/input:/input:ro" -v "$PWD/output:/output" graspit-py python - <<'PYTHON'
import mujoco

from graspit_py.convert.mjcf_hand import export_body, export_hand

model = mujoco.MjModel.from_xml_path("/input/scene.xml")
hand = export_hand(model, "l6_hand_base_link", "/output/hand", name="l6",
                   preshape_joints=["l6_thumb_cmc_roll"])
cube = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <body name="cube">
      <geom type="box" size="0.025 0.025 0.025" mass="0.100"/>
    </body>
  </worldbody>
</mujoco>
""")
export_body(cube, "cube", "/output/object")
print(hand.dof_joints)
PYTHON
```

To plan on the collision hulls used by MuJoCo, pass
`mesh_collision="convex_hull"` to `export_hand`. See
[MuJoCo Model Export](export.md) for the corresponding object option
and the geometry comparison command.

## Generate and Inspect Grasps

Compare the printed DOF order with the order above before applying
the included preshapes. The configuration places the support plane
25 mm below the cube center and specifies the approach axes and
grasp center in the reference palm frame. For another model, adapt
these three frame parameters and express each preshape in its
exported DOF order.

```bash
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python -m graspit_py.planning \
    --robot /output/hand/l6.xml --object /output/object/cube.xml \
    --config /src/docs/examples/realhand-l6-cube.json \
    --top-k 5 --out /output/candidates.json
```

```{literalinclude} examples/realhand-l6-cube.json
:language: json
:caption: Approach settings, preshapes, and wrench-scoring parameters for the six-coordinate L6.
```

The CLI writes ranked grasps to `output/candidates.json`. The first
record has the greatest ranking score among the admitted grasps.
Inspect its contact count and score using Python's standard library.
For an empty list, the top-level `rejections` field identifies the
geometric criteria the nominal approaches failed to meet.

```python
import json
from pathlib import Path

result = json.loads(Path("output/candidates.json").read_text())
if result["candidates"]:
    candidate = result["candidates"][0]
    print(len(candidate["contacts"]), candidate["epsilon_cvar"])
else:
    print(result["rejections"])
```

See [grasp planning](planning.md) for score and pose conventions and
[MuJoCo export](export.md) for reconstructing a selected grasp in the
source model.
