# Supported Hands

The wrapper accepts native GraspIt! robot XML and exports compatible
MuJoCo palm-and-finger models to the same interface. Model support is
specified by the kinematic representation and evaluated operations.
A native model supplies its own joint transmission and closure behavior.
An exported model supplies serial hinge chains, collision geometry, and
an affine map from independent coordinates to source hinges. The
[theory guide](theory.md) defines the map and coordinate conventions;
the [export guide](export.md) describes source triangles and compiled
collision hulls.

## Model Coverage

| Hand | Model Interface | Geometry Coverage | Evaluated Operation |
|---|---|---|---|
| RealHand L6, right | MuJoCo export; 6 independent coordinates, 11 hinges | 49 configurations per mode, 12 links | Planned key grasp and bimanual A7–L6 insertion |
| LEAP Hand | MuJoCo export; 16 independent coordinates, 16 hinges | 4 configurations per mode, 17 links | Planned cube grasp and ZArm–LEAP lift-and-hold |
| PSYONIC Ability Hand | MuJoCo export; 6 independent coordinates, 10 hinges | 12 configurations per mode, 11 links | Planned sphere grasp and Franka–Ability lift-and-hold |
| Barrett | Upstream native GraspIt! XML; 4 coordinates, 9 links | Native pose and contact queries | Cube closure, object-frame contacts, and wrench metrics |
| Shadow Hand, right palm | MuJoCo export; 18 independent coordinates, 22 hinges | 5 configurations per mode, 23 links | Sphere closure, contacts, quality, and link-geometry queries |
| Inspire Hand, right | MuJoCo export; 6 independent coordinates, 12 hinges | 5 configurations per mode, 13 links | Sphere closure, contacts, quality, and affine coupling |

The export comparisons evaluated both `source` and `convex_hull` modes
at the stated configurations. The [planning gallery](planning.md)
presents the planned and executed L6, LEAP, and Ability configurations.

## Grasps on Native and Exported Hands

The grasps below use the native robot interface for Barrett and the
MuJoCo export profiles for Shadow and Inspire. Barrett grasps a 50 mm
cube; Shadow and Inspire grasp a sphere of 60 mm diameter. Each image
uses the queried closed joint state and object pose. Green markers
identify recorded contacts; the annotations locate the fingers and
palm/object frames. Select an image to view it at full size.

````{container} hand-gallery hand-detail-gallery
```{figure} figures/barrett-grasp.png
:alt: Barrett grasp on a cube with finger labels, palm and object frames, and contact markers.
:target: _images/barrett-grasp.png

Native Barrett cube grasp.
```
```{figure} figures/shadow-grasp.png
:alt: Exported Shadow palm grasp on a sphere with finger labels, coordinate frames, and contact markers.
:target: _images/shadow-grasp.png

Exported Shadow sphere grasp.
```
```{figure} figures/inspire-grasp.png
:alt: Exported Inspire grasp on a sphere with finger labels, coordinate frames, and contact markers.
:target: _images/inspire-grasp.png

Exported Inspire sphere grasp.
```
````

| Hand | Object Contacts | Contacting Finger Chains | Native Ferrari–Canny Quality |
|---|---:|---:|---:|
| Barrett | 6 | 3 | 0.146 |
| Shadow | 11 | 5 | 0.423 |
| Inspire | 7 | 5 | 0.539 |

These scores describe normalized wrench resistance under GraspIt!'s
native wrench-budget convention for each hand/object model. Higher
values indicate greater worst-direction resistance under this convention.
Comparisons require a common object, friction coefficient, moment
scale, and force-budget convention.

The [Barrett](examples/barrett-grasp.json),
[Shadow](examples/shadow-grasp.json), and
[Inspire](examples/inspire-grasp.json) records contain object-frame
contact positions and inward normals, the object-relative palm pose,
independent coordinates, and source asset hashes. They connect each
illustrated configuration to its closure and numerical score.

The Shadow profile covers the palm and fingers; the two wrist joints
remain outside the exported subtree. Its four distal pairs use an
explicit equal-angle planning approximation, $q_{J1}=q_{J2}$. The
[manufacturer's model repository](https://github.com/shadow-robot/sr_common)
provides the broader hardware description. Contact-dependent tendon
motion requires its own closure model; this affine profile specifies
a kinematic planning model. The Inspire profile preserves six URDF
mimic relations, including
$q_{\mathrm{intermediate}}=1.06399q_{\mathrm{proximal}}-0.04545$ rad
for each non-thumb finger.

The [GraspIt! Barrett assets](https://github.com/graspit-simulator/graspit/tree/master/models/robots/Barrett)
and [Inspire model collection](https://github.com/dexsuite/dex-urdf/tree/main/robots/hands/inspire_hand)
provide model-source entry points. The evaluated Inspire source is the
right-hand dex-urdf model at revision `f5e7132`, distributed under its
MIT asset license. The evaluated Shadow source derives from the assets
in [XRoboToolkit's hand example](https://github.com/RealHand-Robotics/XRoboToolkit_Realhand_Dexterous_Hand_Example),
with the equal-angle planning couplings stated above; its retained
asset notices include Apache-2.0 and BSD-3-Clause. The
[validation record](validation/hand-support.json) identifies exact
source files by SHA-256. Users retain the source asset licenses when
obtaining models; this wrapper does not redistribute those meshes.

## Use the Native Barrett Model

The Docker image includes the upstream GraspIt! model collection. From
the wrapper repository, this command closes Barrett on the supplied
50 mm cube and records contacts, coordinates, and quality. The cube
center is expressed in the hand frame in millimeters. The evaluated
configuration returned six contacts and positive Ferrari–Canny quality.

```bash
mkdir -p output
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python /src/examples/contact_quality.py \
  --robot /opt/graspit/models/robots/Barrett/Barrett.xml \
  --object /opt/graspit/models/objects/cube.xml \
  --object-position-mm 0 0 100 --out /output/barrett-cube.json
```

## Prepare Shadow and Inspire Models

Reusable profiles in `examples/hand_profiles/` declare the palm root,
expected independent-coordinate and hinge counts, preshape joints,
and affine couplings. The preparation example resolves collision
mesh paths and converts URDF mimics into MJCF joint equalities. For
$q_f=b+a q_d$, MuJoCo's displacement-coordinate coefficients are
$c_0=b-q_f^0+a q_d^0$ and $c_1=a$, where the superscript denotes a
reference coordinate. This preserves the absolute URDF offset when
reference coordinates are nonzero.

Place a source URDF and its meshes under `input/`. Replace the filename
below with your local model. For Shadow, use `shadow_right.json` and
a right-hand URDF with the named `rh_*` joints. Its profile makes the
four planning couplings explicit and rejects conflicting source values.
The Inspire profile retains the source's six affine mimics.

```bash
.venv/bin/python examples/prepare_urdf_hand.py input/inspire_hand_right.urdf \
  output/inspire --profile examples/hand_profiles/inspire_right.json \
  --geometry-only

xvfb-run -a .venv/bin/python examples/export_hand_profile.py \
  output/inspire/scene.xml examples/hand_profiles/inspire_right.json \
  output/inspire-export --mesh-collision convex_hull --check
```

Install `trimesh` and `yourdfpy` in the native virtual environment for
these URDF helpers. Preparation and export use the `urdf` and `mjcf` extras from
[Installation](INSTALL.md). Native GraspIt! is needed for `--check`.
Preparation writes `prepared.urdf`, `scene.xml`, and `preparation.json`.
The scene references source meshes, which must remain available.
Resolve a `package://name/path` mesh URI with
`--package-root name=/absolute/package/root`. A Docker invocation
mounts the source directory read-only and the output directory writable.

Use `--geometry-only` for geometry-based GraspIt! planning: it removes
source inertial elements so MuJoCo derives inertias from collision
geometry during compilation. Omit it to retain input inertial elements.
Both modes remove visual elements. Actuator definitions, calibrated
physical parameters, and the execution controller belong to the source
execution model.

## Write a Profile for a New Hand

A profile is an input to the example preparation and validation helpers.
It describes the source model; it does not configure the native engine
or declare independent-coordinate order. The exporter discovers that
order and writes it to `dof_joints` in the manifest.

This minimal schema example assumes a palm named `palm`, an independent
hinge `finger_base`, and a serial driven hinge `finger_tip`. Both hinge
ranges are illustratively `[0, 1]` rad, with
$q_{\mathrm{finger\_tip}}=1.0q_{\mathrm{finger\_base}}+0.0$ rad.
It is not a profile for an evaluated hand.

```json
{
  "id": "my_hand",
  "palm": "palm",
  "independent_dofs": 1,
  "hinges": 2,
  "preshape_joints": [],
  "couplings": {
    "finger_tip": [1.0, 0.0]
  },
  "mimic_joints": {
    "finger_tip": {
      "driver": "finger_base",
      "ratio": 1.0,
      "offset_rad": 0.0
    }
  },
  "scope": "Illustrative serial finger; replace with the source model scope."
}
```

Set `id` to the export name and `palm` to the source root body. Set
`independent_dofs` and `hinges` to the actual counts; the helper checks
them. `mimic_joints` can install a missing URDF mimic and rejects a
conflicting relation. Its `driver` must name an independent rotational
joint; affine mimic chaining is unsupported. Ratios are dimensionless
and offsets are in radians. `couplings` validates the exported
multiplier and offset; it does not install a source equality by itself.

`preshape_joints` contains independent joint names held at their current
caller-set values during autograsp. The empty list here allows the
single independent coordinate to close. List `finger_base` only if you
intend to hold it, and set its value before closure. `scope` records
the modeled subtree and approximations. Run the preparation and export
commands with the new profile and verify the resulting report.

## Obtain RealHand L6 Source Assets

The [vendor's right L6 directory](https://github.com/RealHand-Robotics/Realhand_description/tree/main/l6/right)
provides `realhand_l6_right.urdf` and meshes. That source uses the palm
`hand_base_link` and independent joints `thunb_cmc_roll`,
`thumb_cmc_pitch`, `index_mcp_pitch`, `middle_mcp_pitch`,
`ring_mcp_pitch`, and `pinky_mcp_pitch`. Preserve the vendor's spelling
`thunb_cmc_roll` when naming the joint. Its five distal mimic relations
use a thumb multiplier of 2.22 and other multipliers of 1.9, with zero
offsets.

These unprefixed names and source couplings differ from the evaluated
L6 reference scene. The bundled `examples/l6_workflow` source scene
requires external DexReach `xarm7_l6` arm/hand meshes at revision
`c939934ca8e0b9b6dc336afe03c2e044b8b6ba8c`. The public vendor model
provides another source route; prepare its own profile and verify its
export before use.

## Verify Couplings and Geometry

The profile exporter intersects each driven hinge's limits with those
of its independent coordinate. For $q_i=a_i u_j+b_i$, it constrains
$u_j$ using $q_i^{\min}\leq a_i u_j+b_i\leq q_i^{\max}$, reversing
interval endpoints for a negative multiplier. With `--check`, it
samples five interior configurations of the feasible intervals and
queries native geometry at the applied coordinates. Source and hull
comparisons use the same named coordinates and the corresponding
geometric representation.

The output contains `hand/<id>.xml`, `hand/<id>.json`, `hand/iv/`,
`configurations.json`, and `profile-report.json`. The report includes
coordinate order, feasible intervals, model/profile hashes, and
per-link errors.

| Palm Model | Source-Mesh Maximum Error | Compiled-Hull Maximum Error | Requested-to-Applied Joint Error |
|---|---:|---:|---:|
| Shadow | 3.92 nm | 3.92 nm | 0 rad |
| Inspire | 0.836 µm | 1.29 µm | 0 rad |

These maxima cover every link across five configurations in each mode.
The original native `ShadowHand.xml` uses fixed dynamic joints outside
the current loader's chain support; use the verified palm export
profile for Shadow planning. The application supplies arm kinematics,
execution controllers, and physical parameter identification.
