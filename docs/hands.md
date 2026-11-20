# Supported Hands

The wrapper accepts native GraspIt! robot XML and exports compatible
MuJoCo palm-and-finger models to the same interface. Model support is
specified by the kinematic representation and the operations evaluated.
A native model supplies its own joint transmission and closure behavior.
An exported model supplies serial hinge chains, collision geometry, and
an affine map from independent coordinates to source hinges. The
[theory guide](theory.md) defines the map and coordinate conventions;
the [export guide](export.md) describes source triangles and compiled
collision hulls.

## Model Coverage

| Hand | Model Interface | Geometry Coverage | Evaluated Operation |
|---|---|---|---|
| RealHand L6, right | MuJoCo export; 6 independent coordinates, 11 hinges | 49 configurations, 12 links | Planned key grasp and bimanual A7–L6 insertion |
| LEAP Hand | MuJoCo export; 16 independent coordinates, 16 hinges | Three configurations, 17 links | Planned cube grasp and ZArm–LEAP lift-and-hold |
| PSYONIC Ability Hand | MuJoCo export; 6 independent coordinates, 10 hinges | 12 configurations, 11 links | Planned sphere grasp and Franka–Ability lift-and-hold |
| Barrett | Upstream native GraspIt! XML; 4 coordinates, 9 links | Native pose and contact queries | Cube closure, object-frame contacts, and wrench metrics |
| Shadow Hand, right palm | MuJoCo export; 18 independent coordinates, 22 hinges | Five configurations in each mesh mode, 23 links | Native loading and link-geometry queries |
| Inspire Hand, right | MuJoCo export; 6 independent coordinates, 12 hinges | Five configurations in each mesh mode, 13 links | Native loading, affine coupling, and link-geometry queries |

The Shadow profile covers the palm and fingers; the two wrist joints
remain outside the exported subtree. Its four distal pairs use an
explicit equal-angle planning approximation, $q_{J1}=q_{J2}$. The
[manufacturer's model repository](https://github.com/shadow-robot/sr_common)
provides the broader hardware description. A tendon transmission with
contact-dependent joint motion requires its own closure model; the
affine profile specifies a kinematic planning model. The Inspire
profile preserves six URDF mimic relations, including
$q_{\mathrm{intermediate}}=1.06399q_{\mathrm{proximal}}-0.04545$ rad
for each non-thumb finger.

The [GraspIt! Barrett assets](https://github.com/graspit-simulator/graspit/tree/master/models/robots/Barrett)
and [Inspire model collection](https://github.com/dexsuite/dex-urdf/tree/main/robots/hands/inspire_hand)
provide model-source entry points. The evaluated Inspire source is the
right-hand dex-urdf model at revision `f5e7132`, distributed under its
MIT asset license. The evaluated Shadow source derives from the Shadow
assets in [XRoboToolkit's hand example](https://github.com/RealHand-Robotics/XRoboToolkit_Realhand_Dexterous_Hand_Example),
with the equal-angle planning couplings stated above; its retained
asset notices include Apache-2.0 and BSD-3-Clause. The
[validation record](validation/hand-support.json) identifies the exact
source files by SHA-256. Users retain the source asset licenses when
obtaining models; this wrapper does not redistribute those meshes.

## Use the Native Barrett Model

The Docker image includes the upstream GraspIt! model collection. From
the wrapper repository, the command below closes the Barrett hand on
the supplied 50 mm cube and records contacts, joint coordinates, and
quality. The cube center is expressed in the hand frame in millimeters.
The evaluated configuration returned six contacts and positive
Ferrari–Canny quality.

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

The repository provides reusable profiles in `examples/hand_profiles/`.
Each profile declares the palm root, independent and total hinge counts,
held preshape coordinates, and affine coupling coefficients. The
preparation example resolves collision mesh paths and converts URDF
mimics into explicit MJCF joint equalities. For a relation
$q_f=b+a q_d$, MuJoCo's displacement-coordinate coefficient is
$c_0=b-q_f^0+a q_d^0$, with $c_1=a$. This expression preserves the
absolute URDF offset for nonzero reference coordinates.

Place a source hand URDF and its meshes under `input/`. Run the commands
below from the wrapper repository, replacing the input filename with
the local model. For Shadow, use `shadow_right.json` and a right-hand
URDF with the named `rh_*` joints. The profile makes the four declared
planning couplings explicit and rejects conflicting source values.
The Inspire profile retains the source's six affine mimics.

```bash
python examples/prepare_urdf_hand.py input/inspire_hand_right.urdf \
  output/inspire --profile examples/hand_profiles/inspire_right.json \
  --geometry-only

xvfb-run -a python examples/export_hand_profile.py \
  output/inspire/scene.xml examples/hand_profiles/inspire_right.json \
  output/inspire-export --mesh-collision convex_hull --check
```

These commands require the native installation and the `mjcf` extra
from [Installation](INSTALL.md). The Docker equivalent mounts the
source directory read-only and the output directory writable. A
`package://name/path` mesh URI is resolved with
`--package-root name=/absolute/package/root`. The generated scene
retains references to the source meshes, so keep those assets available.
The explicit `--geometry-only` option derives inertias from collision
geometry for static model export; it supplies no actuator or dynamics
calibration. Omit the option to retain a source model's inertial data.

## Verify Couplings and Geometry

The export example intersects every driven hinge's limits with its
independent coordinate's limits. For $q_i=a_i u_j+b_i$, it constrains
$u_j$ to the interval obtained from
$q_i^{\min}\leq a_i u_j+b_i\leq q_i^{\max}$, reversing the interval
endpoints for a negative multiplier. It samples five interior
configurations of the resulting intervals and queries native GraspIt!
geometry at the applied coordinates. The report includes the named
coordinate order, feasible intervals, model and profile hashes, and
per-link geometric errors. Source and hull comparisons evaluate the
same named coordinates and their corresponding geometric representation.

| Palm Model | Source-Mesh Maximum Error | Compiled-Hull Maximum Error | Requested-to-Applied Joint Error |
|---|---:|---:|---:|
| Shadow | 3.92 nm | 3.92 nm | 0 rad |
| Inspire | 0.836 µm | 1.29 µm | 0 rad |

The reported maxima cover all links across the five configurations in
each mode. The original native `ShadowHand.xml` uses fixed dynamic
joints outside the current native loader's chain support; use the
verified palm export profile for Shadow planning. The application
supplies arm kinematics, execution controllers, and physical parameter
identification. The [gallery](planning.md) presents the existing
planned and executed L6, LEAP, and Ability configurations.
