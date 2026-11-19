# GraspIt! Python Wrapper

Use GraspIt! from Python to close articulated hands, read contacts, and
evaluate grasp quality. Export a hand and object from MuJoCo when you
need the same geometry in both simulators. The contact planner scores
candidate grasps under errors in the approach pose and uncertain friction.

Start with the [installation guide](INSTALL.md) and [first grasp](quickstart.md).
For your own model, use the [MuJoCo export guide](export.md) to generate the
robot XML, collision meshes, and joint correspondence. The
[planning guide](planning.md) explains how to choose approach parameters,
task loads, and the lower-tail objective. The
[source repository](https://github.com/coenwerem/graspit-python-wrapper)
contains the examples and license. See the [supported hands](hands.md)
for verified models and the [theory guide](theory.md) for the joint
mapping and wrench calculations.

````{container} hand-gallery
```{figure} figures/l6-insertion.png
:alt: RealHand L6 holding the blue key in the planned insertion grasp.

**RealHand L6** · Key grasp
```
```{figure} figures/leap-hold.png
:alt: Four-finger LEAP Hand retaining a blue cube.

**LEAP Hand** · Cube hold
```
```{figure} figures/ability-hold.png
:alt: PSYONIC Ability Hand retaining a blue tapered cylinder.

**PSYONIC Ability** · Cylinder hold
```
````

The L6 image shows the planned GraspIt! closure. The LEAP and Ability
images show the final configurations of recorded MuJoCo holds. See
[grasp generation and execution](planning.md) for the
tasks and grasp scores.

```{toctree}
:maxdepth: 2

INSTALL
quickstart
export
planning
hands
theory
api
```

## Build this manual

Run from the wrapper repository root. The documentation build needs
Python 3.11 or newer and does not need GraspIt! or an X display.

```bash
python3 -m venv .venv-docs
.venv-docs/bin/python -m pip install -r docs/requirements.txt
.venv-docs/bin/python -m sphinx -nW -b html docs docs/_build/html
```

Open `docs/_build/html/index.html`.
