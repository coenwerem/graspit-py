# GraspIt! Python Wrapper

Use GraspIt! from Python to close articulated hands, read contacts, and
evaluate grasp quality. Export a hand and object from MuJoCo when you
need the same geometry in both simulators. The contact planner scores
candidate grasps under errors in the approach pose and uncertain friction.

Start with the [installation guide](INSTALL.md) and [first grasp](quickstart.md).
The example runs on a three-finger hand and a cube supplied with the source.
For your own model, use the [MuJoCo export guide](export.md) to generate the
robot XML, collision meshes, and joint correspondence. The
[planning guide](planning.md) explains how to choose approach parameters,
task loads, and the lower-tail objective. The
[source repository](https://github.com/coenwerem/graspit-python-wrapper)
contains the examples and license.

```{toctree}
:maxdepth: 2

INSTALL
quickstart
export
planning
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
