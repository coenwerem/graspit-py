# First Grasps with Barrett and RealHand L6

The first example generates a Barrett grasp on a cube and computes its contact geometry and quality using models included in the Docker image. The RealHand L6 example extends these operations to MuJoCo model export and grasp planning under uncertainty. Both examples use the image built in [Installation](INSTALL.md).

## First Closure with the Bundled Barrett Hand

The image contains GraspIt!'s Barrett hand and 50 mm cube. From the wrapper checkout, close the hand with the cube center at `(0, 0, 100)` mm in the hand frame:

```bash
mkdir -p output
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python /src/examples/contact_quality.py \
  --robot /opt/graspit/models/robots/Barrett/Barrett.xml \
  --object /opt/graspit/models/objects/cube.xml \
  --object-position-mm 0 0 100 --out /output/barrett-cube.json
```

Xvfb supplies the virtual display used by the container's GraspIt! initialization. The command prints the native epsilon and volume and the number of contacts, then writes `output/barrett-cube.json`. The evaluated example in the [hand-model guide](hands.md) returned six contacts and native Ferrari–Canny quality approximately `0.146`; its saved record is linked there. The program rejects an existing output path, so use a new filename for another run.

Inspect the result using Python's standard library:

```python
import json
from pathlib import Path

record = json.loads(Path("output/barrett-cube.json").read_text())
print("Independent coordinates:", len(record["hand_dof_rad"]))
print("Object contacts:", len(record["contacts_object_frame"]))
print("Native epsilon:", round(record["epsilon"], 3))
```

For the evaluated configuration, the corresponding readout is:

```text
Independent coordinates: 4
Object contacts: 6
Native epsilon: 0.146
```

The record includes object-frame contact positions and inward normals, world-frame hand/object poses, coordinate values, units, and asset hashes. This closure example uses a prescribed placement. The planner below searches approach commands and evaluates pose/friction scenarios.

## Prepare a RealHand L6 Source Scene

The L6 example requires an MJCF scene and its collision meshes in addition to the wrapper image. The evaluated scene is **not bundled** with the wrapper. Its palm is `l6_hand_base_link`, with this independent joint order:

```text
['l6_thumb_cmc_roll', 'l6_thumb_cmc_pitch', 'l6_index_mcp_pitch',
 'l6_middle_mcp_pitch', 'l6_ring_mcp_pitch', 'l6_pinky_mcp_pitch']
```

The [vendor's public L6 description](https://github.com/RealHand-Robotics/Realhand_description/tree/main/l6/right) provides a URDF and meshes for preparing a source scene. It uses different body/joint names, including `hand_base_link` and `thunb_cmc_roll`, and different source couplings. Adapt the [hand profile](hands.md#write-a-profile-for-a-new-hand), palm frame, and preshapes to the actual source model. The bundled Barrett route above remains available without these external assets.

If the evaluated xArm7–L6 asset tree is already available locally, the repository's [complete L6 workflow](https://github.com/coenwerem/graspit-py/tree/main/examples/l6_workflow) supplies an MJCF scene and an export–plan–reconstruct program. Its README specifies the required `arm/meshes/` and `hand/meshes/` layout. For the export commands below, place a matching `scene.xml` and its meshes under `input/`, preserving relative mesh paths.

## Export the L6 Hand and Cube

Run from the wrapper root with the matching six-coordinate source scene:

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

The printed names must match the reference order above before using the supplied preshapes. Export writes `output/hand/l6.xml`, `output/hand/l6.json`, link files under `output/hand/iv/`, and `output/object/cube.xml` with its mesh. To export the compiled collision hulls used for standard MuJoCo mesh collision, pass `mesh_collision="convex_hull"` to both export calls; see [Model Export](export.md) for the geometry convention.

## Generate and Inspect Ranked Grasps

The example configuration sets the support plane 25 mm below the cube center and expresses the approach/finger axes and grasp center in the reference palm frame. Other models require their own frame values and preshapes in the exported coordinate order.

```bash
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python -m graspit_py.planning \
    --robot /output/hand/l6.xml --object /output/object/cube.xml \
    --config /src/docs/examples/realhand-l6-cube.json \
    --top-k 5 --out /output/candidates.json
```

The CLI reports completed approaches and elapsed seconds, then writes `output/candidates.json` with up to five records ordered by decreasing `score`. The JSON also contains the settings, units, elapsed time, and rejection counts. Its terminal summary reports the actual candidate count and runtime; these depend on the hand model, geometry, and search configuration.

```{literalinclude} examples/realhand-l6-cube.json
:language: json
:caption: Reference L6 frame, approach, preshape, constraint, and uncertainty settings.
```

Inspect the output without assuming a nonempty result:

```python
import json
from pathlib import Path

result = json.loads(Path("output/candidates.json").read_text())
print("Candidates:", len(result["candidates"]))
if result["candidates"]:
    candidate = result["candidates"][0]
    print("Contacts:", len(candidate["contacts"]))
    print("Ranking score:", candidate["score"])
    print("Nominal/perturbed pose rows:", len(candidate["epsilon_samples"]))
else:
    print("Rejections:", result["rejections"])
```

A nonempty result supplies the palm transform, closed coordinates, nominal contacts, and score arrays described in [Candidate Records](planning.md#interpret-candidate-records). The first record has the greatest score among the feasible commands retained by the planner. An empty list is accompanied by the rejection counts below.

## Diagnose an Empty Candidate List

| Rejection Key | Inspect First |
|---|---|
| `approach_blocked_or_missed` | Confirm the hand approach axis points toward the object and the standoff/step cover the intended approach. Increase `standoff_mm` when the initial placement is too close; inspect the source frame before changing search distances. |
| `closure_start_collides` | Check the preshape and grasp-center offset at the start of closure using `World.no_collision()`. Verify the preshape coordinate order against `hand.dof_joints`. |
| `grasp_compute_failed`, `too_few_contacts` | Inspect object placement, participating collision geometry, finger velocities, closing indices, and `min_contacts`. |
| `contact_outside_region` | Check the allowed contact box in object-frame millimeters. |
| `below_support`, `in_keepout` | Check the support normal/offset and fixture box centers/half extents in the object frame. These predicates inspect the final nominal hand vertices. |

The [planning guide](planning.md) gives defaults and input requirements for each setting. The [theory guide](theory.md) explains the contact and score calculations, and [Model Export](export.md) explains how to reconstruct a selected grasp in its MuJoCo source scene.
