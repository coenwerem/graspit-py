# Plan with a RealHand L6

This example uses the six-coordinate right RealHand L6 and a 50 mm cube.
Supply a MuJoCo scene with the right hand and its collision meshes. The reference configuration uses palm body
`l6_hand_base_link` and the independent joint order listed below. The
vendor hand and mesh files are obtained separately.

```text
l6_thumb_cmc_roll
l6_thumb_cmc_pitch
l6_index_mcp_pitch
l6_middle_mcp_pitch
l6_ring_mcp_pitch
l6_pinky_mcp_pitch
```

Place the scene under `input/` with any relative mesh paths preserved.
From the repository root, use the image from [Installation](INSTALL.md)
to export the hand and object. Change the body names in the example if
your scene uses different names.

```bash
mkdir -p output
docker run --rm -i --user "$(id -u):$(id -g)" \
  -v "$PWD/input:/input:ro" -v "$PWD/output:/output" graspit-py python - <<'PYTHON'
import mujoco

from graspit_python_wrapper.convert.mjcf_hand import export_body, export_hand

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

Verify the printed joint order against the list above before using the
included preshapes. The configuration places the support plane 25 mm
below the cube center and uses the specified L6 palm frame. With a
different hand or model frame, set the two hand axes, grasp center, and
preshapes for your model before planning.

```bash
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python -m graspit_python_wrapper.planning \
    --robot /output/hand/l6.xml --object /output/object/cube.xml \
    --config /src/docs/examples/realhand-l6-cube.json \
    --top-k 5 --out /output/candidates.json
```

```{literalinclude} examples/realhand-l6-cube.json
:language: json
:caption: Preshapes and approach settings for the six-coordinate L6 model.
```

The result records ranked object-relative palm poses, closed joint
coordinates, contacts, and sampled scores. See [grasp planning](planning.md)
for the field definitions and [MuJoCo export](export.md) for reconstructing
a selected grasp in the source model.
