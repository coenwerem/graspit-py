# Static Grasp Datasets

`graspit_py.dataset` exports contact-planner JSON to a compressed NumPy archive
and a same-stem JSON metadata file. Exporting and loading require NumPy and the
Python standard library. They do not import the native GraspIt! extension or
SciPy. Each archive describes one hand and one object.

The exported arrays support geometric quality prediction, grasp ranking, and
point-cloud-conditioned grasp-generation experiments. Each sample associates an
object-relative hand pose and named joint coordinates with contact geometry and
quality samples. A sample represents a static grasp configuration; sequential
observations and controller actions belong to a trajectory dataset.

The [household-object workflow](https://github.com/coenwerem/graspit-py/tree/main/examples/household_objects)
prepares metric YCB meshes, generates new closures, samples object-surface points,
and writes the archives. Its PyTorch example reads those archives into training
batches on the CPU. The native GraspIt! engine is needed for generation; consumers
can read the NPZ arrays directly with NumPy without installing the simulator.

## Household-Object Examples

The example collection contains 24 saved candidates for each of six YCB objects
and two hands, Barrett and LEAP. The LEAP configurations below were reconstructed
and closed again before rendering. Green markers show contact locations, and
$\varepsilon_{\mathrm{GI}}$ denotes native GraspIt! Ferrari--Canny quality.
Each illustrated configuration was selected by the largest native quality among
its saved candidates. These geometric labels describe the modeled closure.

````{container} hand-gallery hand-gallery-overview
```{figure} figures/household/leap_ycb_tomato_soup_can.png
:alt: LEAP Hand closed around the YCB tomato soup can mesh, with green native contact markers.

**Tomato Soup Can.** 9 contacts; $\varepsilon_{\mathrm{GI}}=0.412$.
```
```{figure} figures/household/leap_ycb_mustard_bottle.png
:alt: LEAP Hand closed around the YCB mustard bottle mesh, with green native contact markers.

**Mustard Bottle.** 27 contacts; $\varepsilon_{\mathrm{GI}}=0.321$.
```
```{figure} figures/household/leap_ycb_tuna_fish_can.png
:alt: LEAP Hand closed around the YCB tuna fish can mesh, with green native contact markers.

**Tuna Fish Can.** 8 contacts; $\varepsilon_{\mathrm{GI}}=0.414$.
```
```{figure} figures/household/leap_ycb_banana.png
:alt: LEAP Hand closed around the YCB banana mesh, with green native contact markers.

**Banana.** 9 contacts; $\varepsilon_{\mathrm{GI}}=0.118$.
```
```{figure} figures/household/leap_ycb_apple.png
:alt: LEAP Hand closed around the YCB apple mesh, with green native contact markers.

**Apple.** 12 contacts; $\varepsilon_{\mathrm{GI}}=0.404$.
```
```{figure} figures/household/leap_ycb_sugar_box.png
:alt: LEAP Hand closed around the YCB sugar box mesh, with green native contact markers.

**Sugar Box.** 15 contacts; $\varepsilon_{\mathrm{GI}}=0.335$.
```
````

Download the {download}`LEAP mustard-bottle example <examples/learning/leap-mustard-example.zip>`
to try dataset loading without a simulator. Extract the NPZ and JSON sidecar
into one directory. The archive contains 24 configurations, their contact and
quality arrays, and 2,048 object-surface points. Its attribution file identifies
the source geometry and dataset license.

```python
import numpy as np

with np.load("ycb_mustard_bottle.npz", allow_pickle=False) as data:
    points_m = data["object_pointcloud_m"]  # (2048,3)
    poses_m = data["T_object_hand_m"]       # (24,4,4)
    q_rad = data["dofs_rad"]               # (24,16)
    dof_names = data["dof_names"]          # (16,), ordered labels
```

## Export

Use a JSON payload written by `python -m graspit_py.planning`, or a dictionary
with the same `candidates` list. Each candidate follows
`GraspCandidate.to_dict()` in `planning/contact_planner.py`.

```bash
python -m graspit_py.dataset \
  --input candidates.json --out learning/test_hand/mug.npz \
  --hand-id test_hand --object-id mug \
  --dof-names dof_0,dof_1,dof_2,dof_3 \
  --mesh-scale 1 \
  --model-file hand=path/to/hand.xml \
  --model-file object=path/to/mug.xml \
  --model-file object_mesh=path/to/mug.off
```

The output is `mug.npz` and `mug.json`. Repeating the command with the same
output path replaces those files. The exporter does not run a planner or load
the models. Use an output stem or directory distinct from the input JSON; the
CLI rejects paths that would replace its source with the metadata sidecar.
`--model-file LABEL=PATH` hashes only the files explicitly listed;
list included robot models, meshes, and other dependencies whose provenance you
need. An XML hash alone does not cover its referenced assets.

`--dof-names` must match the candidate's independent GraspIt! DOF order exactly.
A degree of freedom (DOF) is an independently specified hand coordinate. A
source hand can have additional joints coupled to these coordinates. The names
are caller-supplied labels in the source order; they are not sorted or inferred
from a URDF. Verify the order before assigning anatomical names. Every record
must have the same DOF count.

For exported hands, the household generator retains the adjacent hand
manifest's joint map in `metadata.source.hand_joint_map`. A source joint angle
$q_j$ is reconstructed as $q_j=r_j u_{k(j)}+b_j$, where $u_{k(j)}$ is its named
independent coordinate, $r_j$ is the coupling ratio, and $b_j$ is the angular
offset in radians. The map preserves these values from model export.

`--mesh-scale` accepts one positive factor or three positive factors in XYZ
order. It records the scale already applied to the source model. The exporter
does not apply this scale to candidate coordinates again. A scalar is stored as
three equal factors. Model units and scale must agree with the generated source
records.

Optionally supply `--object-pointcloud-m points.npy`. The file must be a finite
numeric `(P,3)` array of object-frame points in meters and must load with
`allow_pickle=False`. The exporter preserves caller-supplied points without
sampling, centering, scaling, or inventing a camera observation. Without this
argument, the archive has no point-cloud field. Record the point-cloud
generation or sensor procedure with the source payload when it matters.

The equivalent Python interface is:

```python
import json
from pathlib import Path
from graspit_py.dataset import export_dataset

source_path = Path("candidates.json")
payload = json.loads(source_path.read_text())
npz_path, metadata_path = export_dataset(
    payload,
    "learning/test_hand/mug.npz",
    hand_id="test_hand",
    object_id="mug",
    dof_names=["dof_0", "dof_1", "dof_2", "dof_3"],
    mesh_scale=1.0,
    model_files={"hand": "path/to/hand.xml", "object": "path/to/mug.xml"},
    source_path=source_path,
    # object_pointcloud_m=points_in_object_frame_m,
)
```

The caller is responsible for supplying records from one hand/object pair;
the original candidate format contains no per-record hand or object identifier.
Record order is preserved. A candidate's optional `grasp_id` is retained.
Otherwise its ID is `hand_id/object_id/000000`, with the zero-based source row
index. These generated IDs refer to rows in that export, not persistent
identities across reordered source files.

## Coordinates and Precision

The source `T_object_hand_mm` maps hand-base coordinates into the object frame.
The exported `T_object_hand_m` has the same rotation and homogeneous last row;
only its translation is divided by 1000. For column-vector coordinates,
`p_object_m = R_object_hand @ p_hand_m + t_object_hand_m`.
The hand base is the frame used by the source model, which need not be a
particular wrist or palm landmark.

Source contact positions are already in the object frame. Their coordinates
are divided by 1000 without applying the hand transform. Contact normals point
into the object, are expressed in the object frame, and remain unchanged.
Normals and approach directions must be unit vectors within the validation
tolerance; they are not silently normalized. DOF and preshape values remain in
radians. The source roll angle remains in degrees under its explicit
`roll_deg` name.

All real-valued arrays use `float64`. Python/JSON floating-point values are
retained at that precision, without decimal rounding or a conversion to
`float32`. The meter conversion uses floating-point division by exactly 1000;
it does not change rotations, angles, normals, or quality labels. Integer
offsets and shapes use `int64`, presence masks use Boolean arrays, and text
uses NumPy Unicode arrays. No array uses object dtype or pickle.

## Schema 1.0

The schema name is `graspit-py.static-grasps`. Both files carry version `1.0`.
Let `N` be the number of grasps, `D` the number of independent DOFs, `C` the
total number of contacts, `F` the number of source friction samples, `E` the
total number of epsilon sample values, `K` the total number of task sample
values, and `P` the optional point count.

| NPZ field | Shape | Dtype | Meaning |
|---|---|---|---|
| `schema_version` | scalar | Unicode | `1.0` |
| `hand_id`, `object_id` | scalar each | Unicode | Caller-supplied hand and object identity |
| `dof_names` | `(D,)` | Unicode | Independent DOF names in source order |
| `grasp_ids` | `(N,)` | Unicode | Unique record IDs within this archive |
| `mesh_scale` | `(3,)` | float64 | Already-applied XYZ model scale |
| `friction_samples` | `(F,)` | float64 | Source planner friction coefficients in column order |
| `T_object_hand_m` | `(N,4,4)` | float64 | Hand-base to object transform, translation in meters |
| `dofs_rad` | `(N,D)` | float64 | Closed independent hand DOF configurations |
| `preshape_rad` | `(N,D)` | float64 | Source pre-closure independent DOF configurations |
| `closing_mask` | `(N,D)` | bool | DOFs allowed to close; source `closing=None` means all true |
| `approach_dir` | `(N,3)` | float64 | Hand travel direction in the object frame |
| `roll_deg` | `(N,)` | float64 | Source approach roll in degrees |
| `contact_positions_m` | `(C,3)` | float64 | Nominal object-frame contact positions in meters |
| `contact_normals` | `(C,3)` | float64 | Corresponding inward object-frame unit normals |
| `contact_offsets` | `(N+1,)` | int64 | Contact boundaries per record |
| `native_epsilon` | `(N,)` | float64 | Unmodified native GraspIt! epsilon at nominal closure |
| `nominal_epsilon` | `(N,)` | float64 | Recomputed epsilon at nominal closure and the middle friction column |
| `expected_epsilon` | `(N,)` | float64 | Source mean of all epsilon sample values |
| `epsilon_cvar` | `(N,)` | float64 | Source lower-tail mean of epsilon at `planner.tail_mass` |
| `score` | `(N,)` | float64 | Source ranking score interpreted by `score_kind` |
| `score_kind` | `(N,)` | Unicode | `epsilon` or `task_wrench_capacity` |
| `epsilon_samples` | `(E,)` | float64 | Flattened recomputed pose/friction epsilon matrices |
| `epsilon_sample_offsets` | `(N+1,)` | int64 | Epsilon value boundaries per record |
| `epsilon_sample_shapes` | `(N,2)` | int64 | Original pose-by-friction matrix shapes |
| `task_present` | `(N,)` | bool | Whether the record contains task samples |
| `task_samples` | `(K,)` | float64 | Flattened task wrench capacity matrices |
| `task_sample_offsets` | `(N+1,)` | int64 | Task value boundaries per record |
| `task_sample_shapes` | `(N,2)` | int64 | Original task matrix shapes, `(0,0)` when absent |
| `nominal_task`, `expected_task` | `(N,)` each | float64 | Source task summaries; zero placeholders when absent |
| `nominal_task_present`, `expected_task_present` | `(N,)` each | bool | Presence masks for the respective task summaries |
| `object_pointcloud_m` | `(P,3)` | float64 | Optional caller-supplied object-frame points in meters |

Contact slices use `contact_offsets[i]:contact_offsets[i+1]`. Samples are stored
in C order and reconstructed using their own offsets and two-dimensional
shapes. Task matrices, when present, match that record's epsilon matrix shape.
An absent task matrix has an empty slice. Always use presence masks for optional
task summaries; a present value of zero is distinct from an absent value.
An archive with no grasps is valid and retains `(0,D)` configuration arrays and
offset arrays containing only zero.

Sample row 0 represents nominal closure. Subsequent rows represent closures
after perturbing the standoff pose and approach direction according to the
source planner configuration. Failed closures contribute zero quality in the
planner. Columns follow `planner.friction_samples`. If that source configuration
is missing, `friction_samples` is empty and the column coefficients are unknown.
Sample shapes remain explicit. The original records do not contain the actual
sampled perturbation transforms or sampled contact sets, so those quantities
cannot be reconstructed from this archive.

The exporter copies scores without recomputing or relabeling them. Native
epsilon uses the native GraspIt! convention. The other epsilon labels come from
the Python GWS analysis in the source planner. The current planner uses eight
friction-cone edges per contact, a unit total normal-force budget, and torques
normalized by each closure's maximum contact distance to the object center
(with a 0.01 mm minimum). Keep these labels distinct when training or comparing
metrics. The household workflow retains its wrench origin and metric conventions
in the JSON metadata. With no task samples, `score_kind="epsilon"`
means the source planner ranked by `epsilon_cvar`. With task samples,
`score_kind="task_wrench_capacity"` means it ranked by the lower-tail mean of
task wrench capacity. `nominal_task` uses the nominal row's middle friction
column and `expected_task` is the sample mean. These are geometric ranking
quantities under the source contact/friction model; they are not measured
success probabilities, force measurements, or execution outcomes.

## Metadata and Validation

The JSON file records IDs, ordered DOF names, frame and unit meanings, the exact
shape and dtype of every array, and descriptions of all score fields. Its
`source` object retains all original top-level fields except `candidates`,
including the hand configuration, planner configuration, source units, runtime,
and rejections when supplied.

`provenance.source_sha256` hashes the complete source payload serialized as
UTF-8 JSON with sorted keys, compact separators, and no NaN/Infinity values.
`config_sha256` hashes the `hand`, `planner`, and `units` fields with the same
serialization; absent fields are represented as JSON null. When `source_path`
is supplied, `source_file_sha256` also hashes the original file bytes. Each
explicit `model_files` entry records its path and file-byte SHA-256. An empty
model-file mapping means no model contents were verified. Paths can be relative
and are recorded as supplied; hashes identify the bytes at export time.

The exporter validates finite real values, rigid homogeneous transforms,
ordered unique DOF names, DOF vector sizes, unit normals/directions, sample
matrix shapes, friction-column counts when known, and closing indices before
writing. Rotation orthonormality and determinant checks use absolute tolerance
`1e-6`; unit-vector checks use `1e-5`. The loader always uses
`np.load(..., allow_pickle=False)` and rejects unknown versions, field/shape/
dtype mismatches, nonfinite data or metadata, inconsistent offsets, and invalid
presence masks. Validation checks structure and geometry. It does not establish
that a model file matches a physical hand, that the caller's IDs are correct,
or that copied summary labels agree numerically with their samples.

## Load for NumPy or PyTorch

```python
import numpy as np
from graspit_py.dataset import load_dataset

dataset = load_dataset("learning/test_hand/mug.npz")
a = dataset.arrays
i = 0
T_object_hand_m = a["T_object_hand_m"][i]  # (4,4), float64
q_rad = a["dofs_rad"][i]                  # (D,), named by a["dof_names"]
begin, end = a["contact_offsets"][i:i+2]
contacts_m = a["contact_positions_m"][begin:end]
begin, end = a["epsilon_sample_offsets"][i:i+2]
epsilon = a["epsilon_samples"][begin:end].reshape(a["epsilon_sample_shapes"][i])
task = None
if a["task_present"][i]:
    begin, end = a["task_sample_offsets"][i:i+2]
    task = a["task_samples"][begin:end].reshape(a["task_sample_shapes"][i])

# Only if PyTorch is already installed and wanted by the caller:
# import torch
# q_tensor = torch.from_numpy(np.ascontiguousarray(a["dofs_rad"]))
# pose_tensor = torch.from_numpy(np.ascontiguousarray(a["T_object_hand_m"]))
```

PyTorch is not an exporter dependency. The arrays above retain float64; choose
and document any later dtype conversion or normalization in the learning
pipeline. Define the observation source, target representation, split strategy,
and controller contract separately if these static configurations become
training targets.
