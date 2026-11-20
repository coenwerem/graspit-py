# RealHand L6 Export and Grasp-Planning Workflow

This example exports the right RealHand L6 and a 50 mm cube from a
MuJoCo model, plans grasps through the Python interface to GraspIt!,
and reconstructs the selected object-relative palm pose and coupled
joint coordinates in MuJoCo. The calculation uses kinematics and
native GraspIt! closure; it takes no MuJoCo dynamics steps.

Supply the external xArm7–L6 model meshes under a directory containing
`arm/meshes/` and `hand/meshes/`. The supplied `l6-source.xml` uses the
relative paths from `assets/xarm7_l6` in the DexReach model tree at
revision `c939934ca8e0b9b6dc336afe03c2e044b8b6ba8c`. Mesh assets retain
their source licenses and are provided independently of this example.
Build the wrapper's Docker image as described in the
[installation guide](../../docs/INSTALL.md), then run from the wrapper
repository root:

```bash
bash examples/l6_workflow/run.sh graspit-py \
  /absolute/path/to/DexReach/assets/xarm7_l6 ./l6-workflow-output
```

The container uses a virtual display and limits execution to two CPUs
and 3 GiB of memory. The output directory contains the exported hand
and cube descriptions, the affine joint map, and `result.json`.
Each grasp record includes the object-relative palm transform,
closed independent coordinates, object-frame contacts and inward
normals, and sampled quality scores. The selected record also supplies
the reconstructed MuJoCo `qpos`; the program checks the reconstructed
palm translation and rotation against the planned transform. Search
and uncertainty parameters are specified in `problem.json`.
