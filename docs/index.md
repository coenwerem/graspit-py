# graspit-py

graspit-py provides direct access to the
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
contains the code and examples. See [Citation and References](references.md)
for software citation metadata and the
{ref}`arXiv preprint (coming soon) <software-preprint>`.

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
:alt: Exported Shadow hand grasping a sphere with all five fingers.

**Shadow Hand.** Planned sphere grasp.
```
```{figure} figures/inspire-grasp-preview.png
:target: hands.html#grasps-on-native-and-exported-hands
:alt: Exported Inspire hand grasping a sphere with all five fingers.

**Inspire Hand.** Planned sphere grasp.
```
````

Start with [Installation](INSTALL.md) and the [RealHand L6
walkthrough](quickstart.md). The [planning guide](planning.md) explains
approach specification, task loading, and the grasp records shown above.
The [export guide](export.md) covers model conversion and transfer back
to MuJoCo. The [theory guide](theory.md) derives the joint mapping and
wrench scores used in both guides.

```{toctree}
:maxdepth: 2

INSTALL
quickstart
export
planning
hands
roadmap
theory
api
references
```

## Build the Manual

The documentation build requires Python 3.11 or later. Run the
following commands from the wrapper repository root.

```bash
python3 -m venv .venv-docs
.venv-docs/bin/python -m pip install -r docs/requirements.txt
.venv-docs/bin/python -m sphinx -nW -b html docs docs/_build/html
```

Open `docs/_build/html/index.html`.
