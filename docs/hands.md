# Supported Hands

The wrapper loads articulated hand models expressed in GraspIt! robot
XML. The [MuJoCo exporter](export.md) covers palms with serial revolute
finger chains. Each finger has one hinge per moving body, and fixed
parts are attached to their nearest moving link or the palm. A driven
hinge can be related to an independent hinge by an affine equality.
The [kinematics guide](theory.md) gives the coordinate convention.

| Hand | Export Geometry Evaluated | Grasp Planning and Simulation |
|---|---|---|
| RealHand L6, right | 49 joint configurations | Planned key grasp and bimanual A7–L6 insertion |
| LEAP Hand | Three joint configurations | Planned cube grasp and ZArm–LEAP lift-and-hold |
| PSYONIC Ability Hand | 12 joint configurations | Planned tapered-cylinder grasp and Franka–Ability lift-and-hold |

Use a local copy of the hand model and its collision meshes. A model
configuration specifies the palm root, approach axes and grasp center,
joint limits, and the independent joint order for preshapes. The
{ref}`gallery <generated-grasps>` shows the planned and simulated
configurations from these evaluations. The application supplies arm
kinematics and controllers.
