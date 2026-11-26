# Visualization Roadmap

The viewer is planned for a future release. Version 0.2.0 provides link-geometry queries and serialized grasp records for external visualization. The milestones below specify the planned scope; release dates have not been assigned.

## Standalone Model and Grasp Viewer

A planned standalone viewer will present exported hands and saved grasp
records through a modern three-dimensional interface. The design uses
a browser-based WebGL viewport and a Python geometry adapter. Native
GraspIt! remains responsible for closure and contact queries; the
viewer consumes exported geometry, named joint states, and saved
contact records. This separation permits inspection on a machine
without the native Qt interface and preserves the existing Python
planning workflow.

The viewport will expose palm, object, and contact frames with the
coordinate conventions in the [theory guide](theory.md). Users will
inspect finger and link labels, contact positions and inward normals,
and an object's center of mass. A source/export overlay will support
joint-coordinate correspondence and per-link geometric-error
inspection. Mesh representation, units, model identifiers, and the
active joint vector will remain visible in the model inspector.

## Record Loading and Interaction

The proposed data adapter will read the wrapper's hand-export manifest
and saved `GraspCandidate` records, resolve meshes within an explicitly
selected asset directory, and reconstruct each named link transform.
Users will reload a recorded grasp, select a candidate, inspect its
contacts and quality, and adjust independent coordinates within the
coupled feasible limits. A file-backed snapshot will retain the model
and record hashes, camera, displayed frames, and active overlays for
repeatable figure generation. Imported records remain data; loading a
record will not execute embedded code or issue robot commands.

## Delivery Criteria

The first viewer milestone will cover L6, LEAP, Ability, Barrett,
Shadow, and Inspire models through the same geometry adapter. Frame
and normal overlays will be checked against the existing Python
coordinate transforms, and record reload will reproduce the saved
joint state and contact geometry. Browser rendering will be compared
with native queried link geometry at the documented configurations.
The implementation will provide an offscreen image-export path and
keyboard-accessible controls, with no dependency on an interactive
native Qt window. Hardware control and a new grasp planner remain
outside this visualization extension.
