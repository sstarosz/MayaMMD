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
  write-back K offset (`bodyWriteBackOffset`); it stores the PMX rest pose
  in WORLD space (the Bullet world no longer depends on the physics group's
  location).  It connects the body's related joint as a MESSAGE
  (`bodies[i].bodyJoint`) and ALWAYS wires a dynamic body's
  `outTranslate`/`outRotate` STRAIGHT into that joint (rotation-only for
  PHYSICS_BONE).  The
  PMX importer now calls it for each rigid body (matching PMX
  `body`/`bone`/`group`/`mask` semantics), so imported models show their
  bodies on the physics node immediately. Backed by 52 Maya integration
  tests (no PMX file required).
- **Native `pmxRigidBodyConstraint` command (create mode)** — the C++
  command that populates a `pmxPhysicsNode`'s `joints` array with PMX
  rigid-body constraint data at import time (one joint per PMX joint, in
  PMX order). For every joint it writes `jointNameLocal`/`jointNameUniversal`,
  `jointBodyA`/`jointBodyB` (validated against the current body count and
  against each other — a body cannot constrain itself), the PMX `jointType`
  (0..5, validated, and exposed as an **enum dropdown** — Spring6Dof/SixDof/
  P2P/ConeTwist/Slider/Hinge), and the joint frame stored in world space
  (Z-flip + MMD radians → Maya degrees handedness conversion — matching
  `pmxRigidBody`). The linear/angular **limits are
  converted through the same MMD→Maya reflection** (the Z-flip
  F = diag(1,1,−1)): linear Z negates + min/max swap, angular X/Y negate +
  min/max swap, angular Z and the spring constants pass through (magnitudes).
  Angular limits stay in PMX radians — the node hands them to Bullet
  unchanged. Without this, every joint with asymmetric limits (429/496 in
  the test model) was stored **mirrored** and would rotate the wrong way in
  the sim. Backed by 14 Maya integration tests (no PMX file required) +
  end-to-end import assertions that verify the reflection on a real model.
- **Simulation wiring — the physics now RUNS (Phase 3 direct write-back)** —
  the imported solver is fully wired and time-driven:
  - `time1.outTime` drives the `pmxPhysicsNode`'s `time` input (the
    evaluation manager steps the Bullet world every frame; the node declares
    itself non-cacheable).  The Bullet world runs in **world space** — the
    solver's own location (and the physics group's transform) never matters,
    so the skeleton can be moved freely without breaking the simulation.
  - After every body and joint exists, each dynamic body's write-back is
    resolved by the node itself from the `bodies[i].bodyJoint` MESSAGE and
    the joint DAG — the bone index, the write-back parent, and the
    scrub-back reset anchor (`bodyResetAnchorIndex` semantics) are derived
    internally, with NO per-body wiring inputs.  The node computes every
    solved bone world internally (`bodyLocal * K`) and converts to the
    joint-local pose via the bone hierarchy, never reading driven joints
    from the DG (the old feedback cycle is gone).  A dynamic body whose
    parent bone has no rigid body is still connected — the node falls back
    to the raw solved world pose for it.
  - `outTranslate`/`outRotate` connect STRAIGHT into the related joints
    (rotation-only for PHYSICS_BONE).  The output children are **unit-typed**
    (`MFnUnitAttribute` `kDistance`/`kAngle`, exactly like
    `transform.translate`/`rotate`) so the connections are DIRECT — no
    auto-inserted `unitConversion` (a unitless `k3Double` had forced one per
    body).  Angle values are written in degrees (`MAngle::kDegrees` — the
    default `MAngle` unit is radians, which would have inflated every angle
    by 180/π).
  - Caching is left at the node's default: `getCacheSetup()` already declares
    the stateful solver non-cacheable, so the explicit `caching=0` override
    is gone.
  Behavioural import-suite tests prove the sim is alive: 248/285 dynamic
  joints move when the root bone swings, and the write-back drives a skirt
  joint ~40° over 30 frames.

### Changed

- **`pmxPhysicsNode` schema: per-body anchor input + internally derived K**
  (breaking schema change — re-import required).  The two remaining matrix
  inputs are restructured:
  - `anchorWorldMatrix` (top-level matrix array) is replaced by a
    **`bodyAnchorWorld` matrix child of the `bodies[i]` compound** (the
    parentConstraint `target[i].targetParentMatrix` pattern): every FOLLOW_BONE
    body now declares its own anchor world — `pmxRigidBody` connects
    `joint.worldMatrix[0] → bodies[i].bodyAnchorWorld` (a boneless FOLLOW_BONE
    body pins its rest world instead).  Dynamic bodies leave the child at its
    identity default.
  - `bodyWriteBackOffset` (top-level K matrix array) is removed entirely.  The
    node now **derives** each body's write-back offset
    `K = jointRestWorld * bodyRestWorld^-1` internally **when the world is
    (re)built** from the joints' `pmxRestTranslate/Rotate` + `jointOrient`
    (static, already captured by the bone builder) via a DAG walk, plus the
    stored body rest pose; the result is cached for the per-frame
    anchor/write-back consumers (joint rest-attribute edits are import-baked
    and not detected — body rest-pose and hierarchy edits are).
  Behaviourally identical (validated by the Maya integration suites).
- **`pmxPhysicsNode` inputs simplified — derived, not stored** (breaking
  schema change — re-import required).  Two matrix inputs are gone:
  - `anchorOffset` is removed: it was the kinematic body<->joint rest offset
    `bodyRestWorld * jointRestWorld^-1` — the exact inverse of
    `bodyWriteBackOffset` (K = `jointRestWorld * bodyRestWorld^-1`, already
    baked per body).  The node now derives each kinematic anchor's offset as
    `K[body]^-1` (identity for joint-less static colliders, exactly as
    before).
  - `groupInverseWorldMatrix` is removed: it was the exact inverse of
    `groupWorldMatrix` (which itself is now gone — the Bullet world runs in
    world space, so no group transform is needed) — the node derives
    everything internally.
  Removes the two attributes, the `pmxRigidBody` anchor-offset writes and the
  group-inverse connection.  Behaviourally identical (verified: 248/285
  joints move, write-back dR=40.684 — unchanged before/after).
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
    derived from the scene's time unit via `MTime`) is gone.
  - Scenes saved with the old schema need a re-import.

### Changed

- **`pmxPhysicsNode` bone attachment is now a message inside the `bodies`
  compound** (breaking schema change — re-import required).  The per-body
  wiring inputs `bodyRelatedBoneIndex` and the top-level `boneParentIndices`
  array are GONE; the body's related joint is stored as a MESSAGE child
  `bodies[i].bodyJoint` (mirroring PMX's per-body `related_bone_index`),
  connected `joint.message → bodies[i].bodyJoint` by `pmxRigidBody` at
  create.  The node resolves the bone index, the write-back parent and the
  scrub-back reset anchor from the message + the joint DAG (the DAG IS the
  bone hierarchy — the bone builder parents each joint under its PMX parent),
  all internally in `readBodyData` — there are no per-body wiring inputs at
  all.  `pmxRigidBody` no longer dumps the bone hierarchy.

- **`pmxPhysicsNode` simplified — no config hashing, no `configVersion`**
  (breaking schema change — re-import required).  The node no longer hashes
  its config inputs to detect edits:
  - The hidden `configVersion` long (a manual force-rebuild trigger that
    nothing in production ever set) is removed.
  - The FNV-1a config-signature hashing (`computeConfigSignature`) is replaced
    by re-reading the body/joint/gravity attributes every evaluation and
    comparing them against what the Bullet world was built with (new field-wise
    `operator==` on the core `BodyDefinition`/`JointDefinition`).
  - Behaviour is unchanged: any body/joint/gravity value edit, or a changed
    anchor/write-back array count, rebuilds the world in place at the current
    skeleton pose; the anchor/write-back matrix VALUES stay per-frame reads.
    The `SimulationTransition` state machine is folded into a plain if/else
    in `compute()`, and dead Bullet/Maya includes are dropped.  Scenes that
    set `configVersion` need a re-import.
- **`pmxPhysicsNode` simulation now runs in WORLD space — the
  `groupWorldMatrix` input is gone** (breaking schema change — re-import
  required).  Previously the Bullet world ran in the physics group's local
  space and the node mapped the kinematic anchors (and write-back) through
  `groupWorldMatrix⁻¹`; moving the skeleton without the solver/group
  misaligned the simulation.  Now:
  - The `groupWorldMatrix` matrix attribute (and its `group.worldMatrix[0]`
    connection in the Python builder) is removed.
  - Kinematic anchors are used directly in world space (the `K⁻¹`
    body<->bone offset still applies), and the write-back formula
    `boneLocal = K · bodyLocal · B_parent⁻¹ · M_parent⁻¹` is now purely
    world-space (the old groupWorld cancellation is gone by construction).
  - `pmxRigidBody` stores `bodyRestTranslate`/`bodyRestRotate` and
    `pmxRigidBodyConstraint` stores `jointFrameTranslate`/`jointFrameRotate`
    in world space (no `world · groupWorld⁻¹`), so the solver's own location —
    and the physics group's transform — never affects the simulation.  The
    user is free to move the skeleton only.

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
- **`pmxRigidBody` write-back on a shared bone now replaces correctly** —
  when a later dynamic body references a joint that an earlier body already
  drives, the later body now takes over the joint's `outTranslate`/
  `outRotate` connection as documented ("last wins").  Previously
  `connectOrReplace` matched the existing source by plug equality, and
  `MPlug::elementByLogicalIndex()` on a not-yet-materialized output element
  returns a degenerate handle that compares `==` to the existing element —
  so the "already connected" fast-path fired, the replacement was silently
  skipped, and the FIRST body's wiring stayed (this left some real-model
  bodies unwired, e.g. the 蕾米埃尔-泳装 bodies 668/669).  Sources are now
  matched by plug name.
- **`pmxRigidBody` kinematic anchors now use the per-body
  `bodies[i].bodyAnchorWorld`** — the command no longer writes the removed
  top-level `anchorWorldMatrix` array (a compile error against the current
  schema) and no longer bakes `bodyWriteBackOffset`.  FOLLOW_BONE bodies get
  `joint.worldMatrix[0] → bodies[i].bodyAnchorWorld` (or their pinned rest
  world when boneless); the write-back K offset is derived internally by the
  node at world build, so the stale `bodyWriteBackOffset` writes are gone.
- **`pmxRigidBody` fails loudly on malformed 3-double flags** — when the user
  explicitly provides `-size`/`-position`/`-rotation` but the argument list
  cannot be read (e.g. a malformed invocation), the command now emits a
  `displayError` and returns `MS::kFailure` instead of silently falling back
  to the default values, which made the typo hard to debug.

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
