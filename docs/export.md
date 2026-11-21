# MuJoCo Model Export

The exporter converts the hand and object geometry in a compiled
MuJoCo model to GraspIt! XML and OFF meshes. It records an affine map
from independent coordinates to source hinges and a correspondence
between exported links and source bodies. These records support
planning in GraspIt! and reconstruction in the source scene. The
[kinematics guide](theory.md) derives the reference-angle conversion.
See {ref}`Todorov, Erez, and Tassa <ref-todorov-mujoco>` for MuJoCo.

We compared corresponding source and exported link vertices at matching
joint coordinates. For the evaluated right RealHand L6 scene, the maximum
symmetric Hausdorff vertex distance across 49 configurations and 12 links
was below 0.01 µm in each mesh mode. Across all five exported
hands and 75 configurations per mode, the largest recorded distance was
approximately 1.3 µm, for Inspire in `convex_hull` mode. These measurements
refer to the evaluated source scenes and sampled configurations.

```{figure} figures/export-correspondence.png
:alt: Source MuJoCo and exported GraspIt! hand geometry at corresponding configurations for three hands.
:target: _images/export-correspondence.png

Source and exported hand geometry at corresponding configurations. Palm
frames and finger labels identify the correspondence used in the
link-vertex comparisons.
```

## Export a Hand and Object

Install the `mjcf` extra from [Installation](INSTALL.md). Exporting
geometry requires MuJoCo; loading the export in GraspIt! requires the
native installation. Load an MJCF or URDF scene and select the palm as
the hand root and the rigid object's body as the object root. The hand
export covers the palm and finger subtree. The arm and scene geometry remain
in the source scene.

This example assumes `input/scene.xml` contains bodies named `palm` and
`object`. Replace those names with the names in your model.

```python
import mujoco

from graspit_py.convert.mjcf_hand import export_body, export_hand

model = mujoco.MjModel.from_xml_path("input/scene.xml")
hand = export_hand(model, "palm", "output/hand", name="my_hand")
object_xml = export_body(model, "object", "output/object")
```

The output has the following structure; `<body>` is a source moving-body
name, with a wrapper and mesh for each exported link.

```text
output/
├── hand/
│   ├── my_hand.xml
│   ├── my_hand.json
│   └── iv/
│       ├── palm.xml
│       ├── palm.off
│       ├── <body>.xml
│       └── <body>.off
└── object/
    ├── object.xml
    └── object.off
```

Pass `hand.robot_xml` and `object_xml` to the
[contact planner](planning.md). Keep the hand XML beside its `iv/`
directory and the object XML beside its OFF mesh. Exported mesh vertices
use millimeters; the object XML records mass in grams. `export_body`
uses the body name for its filenames unless `name` is supplied, so a
body named `cube` produces `cube.xml` and `cube.off` by default.

## Read the Hand Manifest

`ExportedHand` provides `robot_xml`, `dof_joints`, `chains`, and `links`.
The adjacent JSON file serializes these same fields. `dof_joints` lists
independent coordinates in exported order. Each item in a chain records
the source joint name, driving DOF index, affine multiplier and offset,
and source joint range in radians. `links` maps comma-separated chain
and link indices to source body names; it has no palm entry.

The following schema example represents a synthetic serial finger with
one independent coordinate and two hinges. The identity coupling and
ranges illustrate the fields; they are not parameters of an evaluated
hand.

```json
{
  "robot_xml": "output/hand/my_hand.xml",
  "dof_joints": ["finger_base"],
  "chains": [[
    {
      "mujoco_joint": "finger_base",
      "dof": 0,
      "ratio": 1.0,
      "offset_rad": 0.0,
      "range_rad": [0.0, 1.0]
    },
    {
      "mujoco_joint": "finger_tip",
      "dof": 0,
      "ratio": 1.0,
      "offset_rad": 0.0,
      "range_rad": [0.0, 1.0]
    }
  ]],
  "links": {"0,0": "finger_base_link", "0,1": "finger_tip_link"}
}
```

For each source hinge, reconstruction applies
$q_i=\mathrm{ratio}_i u_{\mathrm{dof}_i}+\mathrm{offset\_rad}_i$.
The offset already accounts for source reference angles. Independent
hinges have multiplier one and offset zero. Use attribute access with
an in-memory `ExportedHand`; use dictionary indexing after loading its
JSON with `json.loads`.

## Choose the Mesh Collision Representation

The default `mesh_collision="source"` exports source mesh triangles,
including concavities. To export the collision representation of
standard MuJoCo mesh geoms, select their compiled hulls.

```python
hand = export_hand(model, "palm", "output/hand", name="my_hand",
                   mesh_collision="convex_hull")
object_xml = export_body(model, "object", "output/object",
                         mesh_collision="convex_hull")
```

For each mesh geom, let $V_g$ denote its compiled hull vertices and
$T_g$ its rigid transform into the export frame. The exported mesh
contains the boundary triangles of $T_g\,\operatorname{conv}(V_g)$.
A link with multiple mesh geoms retains their independently transformed
hulls in its OFF file. The exporter preserves the compiler's triangle
connectivity and winding. Both modes use the same primitive tessellation
and affine joint map. Hull mode raises an error for SDF or plugin
colliders or when a mesh has no compiled hull.

## Prepare the Source Model

Each finger is a serial chain with one hinge on each moving body.
Fixed bodies are assigned to the nearest moving ancestor or the palm.
The exporter transfers collision meshes and tessellates boxes, spheres,
capsules, cylinders, and ellipsoids. A geom participates if it has
nonzero spatial extent and either `contype` or `conaffinity` is nonzero.
Supply participating geometry for the palm and every articulated link.

Supported joint equalities express a driven hinge as an affine function
of an independent hinge. For a compiled displacement equality with
coefficient $c_0$, multiplier $a$, and reference angles $q_f^0,q_d^0$,
the absolute reconstruction offset is $q_f^0+c_0-aq_d^0$. The manifest
stores that offset. `preshape_joints` assigns zero autograsp velocity to
named independent coordinates, preserving their caller-set values
during closure. It does not choose their preshape values.

For rigid objects, `export_body` expresses the named body and its
descendants in that body's frame at the reference configuration.
Pass `include_descendants=False` to select the named body alone.

## Reconstruct a Candidate in MuJoCo

Use the same source scene and hand manifest that produced the planning
inputs. The standalone example below expects `output/candidates.json`
from the planning CLI. The scene must contain the palm body `palm` and
an object body `object` with a free joint named `object_free`; replace
all three names together if your scene differs. A fixed object needs
a different placement strategy.

The code restores the closed hinges, keeps the palm at its source
world pose, and places the free object to reproduce the candidate's
object-relative palm pose. For transforms $T_{AB}$ mapping coordinates
from frame $B$ to frame $A$, this construction is
$T_{WO}=T_{WH}T_{OH}^{-1}$. The candidate stores $T_{OH}$ as
`T_object_hand_mm`; convert its translation to meters for MuJoCo.

```python
import json
from pathlib import Path

import mujoco as mj
import numpy as np

model = mj.MjModel.from_xml_path("input/scene.xml")
data = mj.MjData(model)
manifest = json.loads(Path("output/hand/my_hand.json").read_text())
result = json.loads(Path("output/candidates.json").read_text())
if not result["candidates"]:
    raise RuntimeError(result["rejections"])
candidate = result["candidates"][0]

for chain in manifest["chains"]:
    for joint in chain:
        u = candidate["dofs_rad"][joint["dof"]]
        data.joint(joint["mujoco_joint"]).qpos[0] = (
            joint["ratio"] * u + joint["offset_rad"]
        )
mj.mj_forward(model, data)

palm_name = "palm"
object_name = "object"
object_joint_name = "object_free"
object_joint = model.joint(object_joint_name)
if model.jnt_type[object_joint.id] != mj.mjtJoint.mjJNT_FREE:
    raise ValueError(f"{object_joint_name} must be a free joint")
if model.jnt_bodyid[object_joint.id] != model.body(object_name).id:
    raise ValueError("The free joint must belong to the object body")

palm = data.body(palm_name)
T_OH = np.asarray(candidate["T_object_hand_mm"], dtype=float)
R_WH = palm.xmat.reshape(3, 3)
R_WO = R_WH @ T_OH[:3, :3].T
p_WO = palm.xpos - R_WO @ (T_OH[:3, 3] * 0.001)
q_WO = np.empty(4)
mj.mju_mat2Quat(q_WO, np.ascontiguousarray(R_WO).ravel())
data.joint(object_joint_name).qpos[:] = np.r_[p_WO, q_WO]
mj.mj_forward(model, data)
```

The hinge assignments leave unrelated joint coordinates at their
initialized values. To reconstruct into a recorded source state,
restore that state before applying the map. If the object must remain
fixed, $T_{WH}=T_{WO}T_{OH}$ instead defines a palm-pose target for the
application's arm kinematics.

````{container} hand-gallery export-gallery
```{figure} figures/export-source-workcell.png
:alt: MuJoCo source scene with two A7 arms and RealHand L6 hands during key insertion.
:target: _images/export-source-workcell.png

MuJoCo source scene during bimanual insertion.
```
```{figure} figures/export-grasp-frames.png
:alt: Exported right RealHand L6 at the planned key grasp with finger labels and palm and object frames.
:target: _images/export-grasp-frames.png

Exported right L6 at the corresponding key grasp, with annotated frames.
```
````

## Verify Link Geometry

`examples/check_export_geometry.py` compares corresponding source and
exported link vertices in the palm frame. Supply the source scene,
palm name, and a JSON list of joint vectors in exported DOF order,
in radians. This command uses the Docker image from
[Installation](INSTALL.md) and a source palm named `l6_hand_base_link`.

```bash
mkdir -p output
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/input:/input:ro" -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python /src/examples/check_export_geometry.py \
    /input/scene.xml l6_hand_base_link /input/configurations.json /output/parity \
    --mesh-collision convex_hull
```

Use `--mesh-collision source` to compare source mesh vertices.
`geometry-check.json` records the mode, each link's maximum symmetric
Hausdorff vertex distance in millimeters across the configurations,
the greatest requested-to-applied DOF discrepancy, and the tolerance
result. These are comparisons of the selected geometric representation
at the supplied coordinates.
