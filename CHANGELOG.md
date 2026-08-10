# Changelog

All notable changes to MayaMMD will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Added

- **vcpkg dependency management with Bullet 3.25** — C/C++ dependencies are
  now resolved via vcpkg (`vcpkg.json`). The `maya*` CMake presets activate
  the vcpkg toolchain from `VCPKG_ROOT` (hidden `with-vcpkg` preset) and
  install dependencies automatically on the first configure. A bootstrapped
  vcpkg with `VCPKG_ROOT` set is now a build prerequisite (see
  `docs/CPPDevelopment.md`).
- **Maya-free C++ physics core (`mmd_core`)** — the Bullet-based simulation
  engine (`simulation.hpp`/`simulation.cpp`, `physics_math.hpp`, `common.hpp`)
  as a static library, covered by Catch2 unit tests (18 cases, no Maya SDK
  required). Internal building block for the upcoming native physics node.
- **Native rigid-body physics node (`pmxPhysicsNode`)** — the C++
  `MPxLocatorNode` that owns the embedded Bullet world is now registered by
  the plugin and created as an **empty node per imported model** (under a
  `{model}_Physics` group). The `bodies`/`joints` arrays and the solved-pose
  write-back are populated by the rigid-body commands (see the
  `pmxRigidBody` command below).
- **Native `pmxRigidBody` command (create mode)** — the C++ command that
  populates a `pmxPhysicsNode`'s `bodies` array with PMX rigid-body data at
  import time. For every body it writes `bodyShapeSize`, `bodyPhysicsMode`,
  `bodyGroupId`/`bodyMask`, mass/damping/friction/restitution, and the
  kinematic anchor data (`bodyWriteBackOffset`, `bodyParentInverseMatrix`,
  `bodyResetAnchorIndex`, `bodyParentBodyIndex`); it also connects the
  physics group's `worldMatrix[0]` to the solver's `groupWorldMatrix`. The
  PMX importer now calls it for each rigid body (matching PMX
  `body`/`bone`/`group`/`mask` semantics), so imported models show their
  bodies on the physics node immediately. Backed by 36 Maya integration
  tests (no PMX file required).
- **Native `pmxRigidBodyConstraint` command (create mode)** — the C++
  command that populates a `pmxPhysicsNode`'s `joints` array with PMX
  rigid-body constraint data at import time (one joint per PMX joint, in
  PMX order). For every joint it writes `jointBodyA`/`jointBodyB` (the
  referenced rigid-body indices, validated against the current body count),
  the PMX `jointType` (0..5, validated), the joint frame (Z-flip + MMD
  radians → Maya degrees handedness conversion) and the linear/angular
  limits and spring constants **verbatim** (angular stays in PMX radians —
  the node hands them to Bullet unchanged). The PMX importer now calls it
  for every rigid-body constraint after the bodies exist, so imported
  models hold the full constraint set. Backed by 11 Maya integration tests
  (no PMX file required) + end-to-end import assertions.

### Changed

- **`pmxPhysicsNode` schema refined to PMX verbatim** — the physics node's
  attribute surface now matches the PMX fields it stores:
  - `bodyRadius`/`bodyExtents`/`bodyLength` are replaced by a single
    `bodyShapeSize` double3 — the PMX `shape_size` **verbatim** (full size).
    The node derives the engine's radius / box half-extents / capsule length
    by collider type via the new Maya-free `mmd::core::applyShapeSize` /
    `shapeSizeFromBodyDefinition` helpers (covered by Catch2 unit tests).
    `DrawBody` carries `shapeSize[3]` as the data contract for the follow-up
    viewport draw override.
  - The per-anchor `anchorParentInverseMatrix[]` array is replaced by a single
    `groupInverseWorldMatrix` matrix (the physics group's world inverse),
    applied once to every kinematic anchor.
  - The `fps` attribute (which only ever served as a rebuild trigger — dt is
    derived from the scene's time unit via `MTime`) is replaced by a hidden
    `configVersion` long as the clean forced-rebuild trigger.
  - Scenes saved with the old schema need a re-import.

### Fixed

- **CCD IK solver reliability** — `ccdIKSolverNode` no longer overshoots the IK
  target or bends hinge joints (e.g. the knee) the wrong way when the target is
  moved incrementally:
  - The per-joint angle-limit state is now initialised from each constrained
    joint's current rotation at the start of every solve, so repeated solves
    (e.g. dragging an IK handle) no longer drift from the joint's real rotation
    and accumulate past the angle limits.
  - A "fold boost" resolves the near-*straight-leg* singularity — when the
    target is nearly in line with the chain, pure angular CCD stalls and the
    effector never quite reaches. The solver now folds the chain toward the
    target's radius from the chain root so it converges even for small raises.
- Added a regression test reproducing the real leg IK setup
  (bones 14/15/16 and the model's knee limit) that verifies the knee bends
  forward, tracks the target, and never overshoots across a full raise sweep.
- **Maya SDK resolution now prefers the version being built** —
  `cmake/FindMaya.cmake` checks the cached SDK matching `MAYA_VERSION` first
  (a Maya-2027 build can no longer resolve the 2026 SDK just because it was
  cached earlier), falling back to the remaining cached SDKs newest-first.
- **`pmxRigidBody` anchor offsets no longer drop group scale** — body anchor
  offsets are built from the body's world-space translation/rotation directly
  instead of round-tripping through the group's matrix (an euler rebuild
  silently lost scale), which would have produced the wrong write-back pose
  for bodies under a scaled physics group.
- **`pmxRigidBody -group` is clamped to the PMX 0..15 range** — out-of-range
  values can no longer write an invalid `bodyGroupId` enum index.
- **Physics bodies reappear after a plugin reload** — `rigid_body_builder`
  is now force-reloaded on plugin init, so a stale cached module no longer
  leaves the physics node empty after a reload in the same Maya session.

### Removed

- **Unused `ci` CMake preset** (configure + build) and the redundant
  `BUILD_NATIVE_MODULE=ON` override in the `base` preset — CI uses the
  `maya*` presets. The build presets now also carry an explicit
  `Debug`/`Release` configuration.

## [0.1.0] - 2026-08-01

### Added

- **PMX model import** — full MMD model import:
  - Mesh geometry — positions, normals, UVs, triangle faces
  - Bone hierarchy — full parent/child structure with tail joints
  - BDEF1/2/4 smooth skinning
  - Per-face materials via `openPBRSurface` with diffuse textures and opacity
  - Vertex morphs as Maya blendShapes
  - Bone morphs via a custom `boneMorphNode` DG node
  - Rigid body visual guides (physics not yet implemented)
  - PMX file validation before import
  - Custom attributes for scene metadata (`pmxBoneData`, `pmxMorphMapping`, rest pose)
- **VMD motion import** — load MMD motion data:
  - Bone keyframe animation (translation + quaternion rotation)
  - Quaternion SLERP interpolation with Bezier curves
  - Morph weight animation for blendShape and bone morph targets
  - IK-compatible playback via the custom CCD solver
- **VPD pose import** — apply MMD pose data with optional keyframing
- **Custom CCD IK solver** — `ccdIKSolverNode` registered as a Maya IK solver, with per-joint angle limits and chain priorities
- **INHERIT_ROTATION / INHERIT_TRANSLATION** constraints with influence scaling
- **Dockable UI** — Maya workspaceControl panel with PMX/VMD/VPD import, pose editor with morph weight sliders, and Reset to Bind Pose
- **Self-describing scene** — all import metadata stored as custom Maya attributes
- **Multi-model support** — multiple PMX models can coexist in the same scene, targeted via selection

### Fixed

- IK bones no longer reported as unsupported during validation
- Joint orientation preserved when controllers are created for fixed-axis or bone-morph bones
- Euler angle flips (gimbal lock) in VMD animation
- IK handle hierarchy and rest-pose capture
- Visible mesh seams caused by unlocked vertex normals
