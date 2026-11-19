# graspit_python_wrapper

Plan and evaluate multifingered grasps in Python using the
[GraspIt! simulator](https://graspit-simulator.github.io/). Use the native
bindings to pose and close a hand, retrieve object-frame contacts and link
geometry, and compute GraspIt! wrench scores. The contact planner ranks
candidate approaches under pose and friction uncertainty. The MuJoCo exporter
writes GraspIt! hand and object models with a named joint map for reconstructing
planned configurations in MuJoCo.

## Install on Linux

The reference build uses Ubuntu 24.04 and Python 3.12. Docker builds the
pinned public GraspIt! source with the bundled compatibility patch.

```sh
git clone https://github.com/coenwerem/graspit-python-wrapper.git
cd graspit-python-wrapper
docker buildx build --load -f docker/Dockerfile -t graspit-py .
```

See the [installation guide](docs/INSTALL.md) for a native build and
[the online documentation](https://coenwerem.github.io/graspit-python-wrapper/) for planning examples, [supported hands](https://coenwerem.github.io/graspit-python-wrapper/hands.html), and API details.

## Work with a supported hand

The manual shows verified grasps for the RealHand L6, LEAP, and PSYONIC
Ability. The [L6 quickstart](docs/quickstart.md) uses a user-supplied
MuJoCo model to export the hand and plan cube grasps. The
[export guide](docs/export.md) shows the L6 in the source workcell and in
GraspIt! and explains how to compare link geometry.

## Plan a grasp

Supply a GraspIt! robot and object XML with a JSON configuration for the
hand axes, approach search, and optional task loads. You can export a compiled
MuJoCo hand and object using `export_hand` and `export_body` in
`graspit_python_wrapper.convert.mjcf_hand`.

```sh
xvfb-run -a python -m graspit_python_wrapper.planning \
    --robot hand.xml --object object.xml \
    --config problem.json --out candidates.json
```

GraspIt! translations and contact points use millimeters. The exporter reads
MuJoCo positions in meters. The wrapper exposes joint angles in radians and
quaternions in scalar-first order. The planner returns contact normals
pointing into the object and reports poses in the object frame. Arm inverse
kinematics and trajectory planning run in the calling application.

## License and citation

This project is licensed under GPL-3.0-or-later. Cite the wrapper using
[CITATION.cff](CITATION.cff) and cite
[Miller and Allen's GraspIt! paper](https://doi.org/10.1109/MRA.2004.1371616)
when using the simulator.
