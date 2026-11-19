# GraspIt! Python Wrapper

`graspit_python_wrapper` exposes the [GraspIt! simulator](https://graspit-simulator.github.io/)
as a Python library for grasp synthesis and evaluation with multifingered
hands. The native bindings provide hand pose and joint control, autograsp,
collision queries, contact geometry, and wrench-space metrics. The contact
planner ranks simulated closures by Ferrari–Canny quality or task-wrench
capacity under approach and friction uncertainty. The MuJoCo exporter
generates GraspIt! hand and object descriptions and records the joint
correspondence for transferring a planned configuration back to MuJoCo.

## Installation on Linux

The reference build uses Ubuntu 24.04 and Python 3.12. Docker builds the
pinned public GraspIt! source with the supplied compatibility patch.

```sh
git clone https://github.com/coenwerem/graspit-python-wrapper.git
cd graspit-python-wrapper
docker buildx build --load -f docker/Dockerfile -t graspit-py .
```

The [manual](https://clintonenwerem.com/graspit-python-wrapper/) covers
[installation](docs/INSTALL.md), [model export](docs/export.md), [grasp
planning](docs/planning.md), and the [Python API](docs/api.md).

## Grasp Planning

The [RealHand L6 walkthrough](docs/quickstart.md) uses a user-supplied
MuJoCo model of the right hand and a 50 mm cube. For an existing GraspIt!
model, pass the robot XML, object XML, and a JSON configuration defining
the hand axes, approach search, and any task loads. From the directory
containing those files, run the planner in the Docker image.

```sh
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/work" graspit-py \
    xvfb-run -a python -m graspit_python_wrapper.planning \
    --robot hand.xml --object object.xml \
    --config problem.json --out candidates.json
```

Each candidate contains the object-relative palm pose, closed joint
coordinates, object-frame contacts, and sampled wrench scores. The
[theory guide](docs/theory.md) defines those quantities and their
coordinate conventions. The [supported-hands guide](docs/hands.md)
documents export and simulation coverage for the RealHand L6, LEAP
Hand, and PSYONIC Ability Hand. The calling application supplies arm
kinematics, trajectory planning, and execution.

## License and Citation

The wrapper is licensed under GPL-3.0-or-later. Cite the wrapper using
[CITATION.cff](CITATION.cff) and the simulator using the
[GraspIt! paper by Miller and Allen](https://doi.org/10.1109/MRA.2004.1371616).
