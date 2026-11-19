# Record your first grasp

This example exports a three-finger MuJoCo hand and a cube, closes the hand
in GraspIt!, and records the joint coordinates, contact geometry, and native
wrench-space scores. The script uses geometry bundled with the wrapper.

With the Docker image from [Installation](INSTALL.md), run from the
repository root:

```bash
mkdir -p output
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/output:/output" graspit-py \
  xvfb-run -a python /src/docs/examples/first_grasp.py /output/first-grasp
```

For a native build with the MuJoCo extra installed:

```bash
xvfb-run -a .venv/bin/python docs/examples/first_grasp.py output/first-grasp
```

The script writes `hand/hand.xml`, the accompanying joint map and meshes,
`object/cube.xml`, and `grasp.json`. The JSON contacts are reported in the
cube frame. Position is in millimeters and each normal points into the cube.
The joint vector contains the independent GraspIt! coordinates in radians.

```{literalinclude} examples/first_grasp.py
:language: python
:caption: Export a hand and cube, close the hand, and record the grasp.
```

Use the exported robot and object XML with the
[contact planner](planning.md) to search over approaches and score the
resulting grasps under pose and friction uncertainty.
