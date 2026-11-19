# Supported hands

The wrapper loads articulated GraspIt! robot XML. The MuJoCo exporter
supports palms with serial revolute finger chains, fixed geometry, and
affine couplings from driven hinges to independent hinges. Use the
[export guide](export.md) to prepare a model with this topology.

| Hand | Verified coverage | Model availability |
|---|---|---|
| RealHand L6 | MuJoCo export and link geometry over 49 configurations. Grasp planning and simulated bimanual A7–L6 key insertion. | Supply your own hand and collision meshes. |
| LEAP Hand | MuJoCo export and link geometry over three configurations. Grasp planning and simulated ZArm–LEAP lift-and-hold. | Supply your own hand and collision meshes. |
| PSYONIC Ability Hand | MuJoCo export and link geometry over 12 configurations. Grasp planning and simulated Franka–Ability lift-and-hold. | Supply your own hand and collision meshes. |

The [hand gallery](index.md) shows grasps on all three models. Simulation validation covers the listed model revisions and
configurations. A new model needs its own palm frame, approach axes,
joint limits, and collision geometry. The calling application supplies
arm kinematics, controllers, and any hardware integration.
