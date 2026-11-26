# graspit-py

**graspit-py** connects simulated hand closure and contact mechanics to Python grasp synthesis, numerical analysis, and learning-data generation.

It provides direct access to the
[GraspIt! simulator](https://graspit-simulator.github.io/) for planning
and evaluating grasps with multifingered hands. See
{ref}`Miller and Allen <ref-miller-allen>` for the simulator's design.
A Python program can set the palm pose and finger coordinates, invoke
closure, and retrieve object-frame contact positions and normals. This interface makes each simulated closure
available for inspection and external scoring. A planner built on the
interface enumerates approaches and ranks their closures by general-purpose
or task-specific wrench capacity under approach and friction uncertainty.

The MuJoCo exporter produces GraspIt! robot and object models from a
compiled MuJoCo scene. It records the joint and link correspondence needed
to transfer planned configurations to the source model. The application
uses the resulting targets for arm kinematics, motion planning, and
execution. The [source repository](https://github.com/coenwerem/graspit-py)
contains the code and examples. The [companion paper on arXiv](https://arxiv.org/abs/2611.29011)
describes the interface and its evaluation. See [Citation and References](references.md)
for the paper citation and software metadata.

## Household Objects and Learning Data

The public [Household Grasps dataset](https://huggingface.co/datasets/cenwerem/graspit-py-household)
provides the full collection as browsable Parquet tables and downloadable
NPZ/JSON archives under CC BY 4.0. Its Dataset Viewer includes grasp-level and
object-level configurations.

The [household-object workflow](https://github.com/coenwerem/graspit-py/tree/main/examples/household_objects)
generates static grasp candidates on metric YCB meshes and exports their
configurations, contacts, and quality samples. The example collection contains
**288 candidates across six objects using Barrett and LEAP hands**: a sugar box,
tomato soup can, mustard bottle, tuna fish can, banana, and apple.
The subcaptions report contact count and native GraspIt! Ferrari–Canny quality
$\varepsilon_{\mathrm{GI}}$.

````{container} hand-gallery hand-gallery-overview
```{figure} figures/household/leap_ycb_mustard_bottle.png
:target: datasets.html#household-object-examples
:alt: LEAP Hand closed around the YCB mustard bottle, with green contact markers.

**Mustard Bottle.** 27 contacts; $\varepsilon_{\mathrm{GI}}=0.321$.
```
```{figure} figures/household/leap_ycb_banana.png
:target: datasets.html#household-object-examples
:alt: LEAP fingers closed around the curved YCB banana mesh.

**Banana.** 9 contacts; $\varepsilon_{\mathrm{GI}}=0.118$.
```
```{figure} figures/household/leap_ycb_tomato_soup_can.png
:target: datasets.html#household-object-examples
:alt: LEAP Hand grasping the YCB tomato soup can mesh.

**Tomato Soup Can.** 9 contacts; $\varepsilon_{\mathrm{GI}}=0.412$.
```
````

Each NPZ/JSON archive associates named hand coordinates and object-relative
poses with contacts, 2,048 object-surface points, and individual pose/friction
quality samples. Positions use meters, and hand coordinates use radians.
Native and recomputed quality values remain distinct. These static records
support quality prediction, candidate ranking, and point-cloud-conditioned
grasp-generation studies with explicit geometric acceptance criteria.

The [dataset guide](datasets.md) includes a downloadable LEAP mustard-bottle
example and NumPy/PyTorch loading instructions. Reading the arrays requires
NumPy and the Python standard library; new closures require native GraspIt!.
The illustrated grasps were closed again from their saved commands before
rendering. Their labels quantify the modeled contact geometry.

## Supported Hands

````{container} hand-gallery hand-gallery-overview
```{figure} figures/l6-insertion.png
:alt: RealHand L6 at the planned grasp on a blue insertion key.

**RealHand L6.** Planned key grasp.
```
```{figure} figures/leap-hold.png
:alt: Four-finger LEAP Hand retaining a blue cube.

**LEAP Hand.** Simulated cube hold.
```
```{figure} figures/ability-hold.png
:alt: PSYONIC Ability Hand holding a blue sphere.

**PSYONIC Ability Hand.** Simulated sphere hold.
```
```{figure} figures/barrett-grasp-preview.png
:target: hands.html#grasps-on-native-and-exported-hands
:alt: Native Barrett hand closed around a cube with six verified contacts.

**Barrett Hand.** Cube grasp.
```
```{figure} figures/shadow-grasp-preview.png
:target: hands.html#grasps-on-native-and-exported-hands
:alt: Shadow Hand visual meshes at the closed sphere-grasp configuration.

**Shadow Hand.** Planned sphere grasp.
```
```{figure} figures/inspire-grasp-preview.png
:target: hands.html#grasps-on-native-and-exported-hands
:alt: Exported Inspire hand grasping a sphere with all five fingers.

**Inspire Hand.** Planned sphere grasp.
```
````

Start with [Installation](INSTALL.md) and the [first-grasp
walkthrough](quickstart.md). The [planning guide](planning.md) explains
approach specification, task loading, and the grasp records shown above.
The [export guide](export.md) covers model conversion and transfer back
to MuJoCo. The [theory guide](theory.md) derives the joint mapping and
wrench scores used in both guides.

## From Models to Grasp Records

```{figure} figures/architecture.png
:alt: Data flow from MuJoCo model conversion and Python approach settings through native closure, numerical wrench analysis, and grasp records for application clients.
:target: _images/architecture.png

Model conversion supplies geometry and the joint map. Native closure provides contacts, and numerical analysis supplies wrench-quality samples. A grasp record connects the selected command to its nominal contacts and scores. Select the figure for a full-size view.
```

The [API reference](api.md) groups the corresponding operations by scene setup, hand state, contacts, and geometry. The [theory guide](theory.md) connects their numerical inputs to the contact and wrench models.

```{toctree}
:maxdepth: 2

INSTALL
quickstart
theory
export
planning
hands
datasets
api
roadmap
references
```
