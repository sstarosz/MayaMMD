# Changelog

All notable changes to MayaMMD will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [0.2.0] - 2026-08-14

### Added

- **Rigid-body physics simulation** — MMD rigid bodies and constraints are
  now imported into a native solver node and simulated with Bullet:
  - `pmxRigidBodyNode` — a native C++ `MPxLocatorNode` that owns an embedded
    Bullet world and runs under Cached Playback (declares itself
    non-cacheable so the sim re-steps every frame).
  - `pmxRigidBody` command — writes each PMX rigid body (shape, physics
    mode, collision group/mask, mass/damping/friction/restitution, rest pose)
    onto the node and wires dynamic bodies' solved pose straight into their
    related joints.
  - `pmxRigidBodyConstraint` command — writes each PMX joint (type, frame,
    linear/angular limits + springs), with the MMD→Maya reflection applied so
    asymmetric limits are stored unmirrored.
  - The simulation is **time-driven and world-space**: the solver steps with
    the timeline, moves with the skeleton, and no longer depends on the
    physics group's location.
- **Maya-free C++ physics core (`mmd_core`)** — the Bullet-based simulation
  engine as a static library, covered by Catch2 unit tests (no Maya SDK
  required). Internal building block for the native node.
- **vcpkg dependency management** — C/C++ dependencies (Bullet 3.25) are
  resolved via vcpkg from `VCPKG_ROOT`; the `maya*` CMake presets configure
  and build out of the box (see `docs/CPPDevelopment.md`).

### Changed

- **Rigid-body physics renamed to `pmxRigidBodyNode`** (breaking — re-import
  required). Node type `pmxPhysicsNode` → `pmxRigidBodyNode`, model-root
  discovery attribute, group `{model}_Physics` → `{model}_RigidBodies`, and
  solver `{model}_PhysicsSolver` → `{model}_RigidBodySolver` are unified with
  the `rigid_body` family.
- **Solver schema refined over the cycle** (breaking — re-import required):
  attributes now mirror the PMX fields verbatim (`bodyShapeSize` replaces
  radius/extents/length, collision mask as one bool per group, the related
  joint stored as a `bodies[i].bodyJoint` message), the write-back offset K is
  derived internally at world build, and the simulation runs in world space —
  the solver's own location never affects the sim.
- **`pmxRigidBodyNode` internals rewritten** (no schema change, no re-import
  needed): the node holds a single immutable `World` value and `compute()`
  runs a pure frame transition over it (build/advance/write), with the
  DAG-derived wiring cached per world build instead of re-resolved every
  evaluation. Behaviour is unchanged.

### Fixed

- **VMD playback range no longer shrinks** — applying a second (shorter)
  animation no longer truncates the timeline and clips a longer animation
  that is already applied; the playback range is now extended only.
- **CCD IK solver reliability** — `ccdIKSolverNode` no longer overshoots the
  IK target or bends hinge joints (e.g. the knee) the wrong way when the
  target is moved incrementally; a "fold boost" resolves the near-straight-leg
  singularity so the chain converges even for small raises. Backed by a
  regression test on a real leg IK setup.
- **`pmxRigidBodyNode` first evaluation could pop a posed skeleton to rest** —
  the first evaluation now goes through the same build path as a config edit /
  scrub-back (anchors applied + dynamic bodies reset to the current skeleton
  pose).
- **Moving the whole character no longer breaks the simulation** — dragging
  the character (all kinematic anchors share one world-space rigid move) now
  rides the dynamic chains along by the same transform at the current pose,
  with no physics step, instead of teleporting the anchors and yanking the
  chains (the old behaviour displaced the skirt/hair by the move and baked the
  offset into the write-back — even at frame 0 with nothing playing).  A local
  bone drag (anchors move differently) is unchanged and still steps one tick.
  The ride-along only fires when every bone-attached anchor shares the move,
  so it can never trigger during normal animation.
- **Rewinding after playing no longer jumps** — going back in time (scrub-back)
  rebuilds the physics world, and the rebuild now pins the chains to the
  CURRENT skeleton pose using the joints' REST worlds as the reference (a
  model constant from the stamped `pmxRest*` attributes).  Previously the
  rebuild compared against the anchors captured at the previous build, so a
  whole-character move or an animation-posed skeleton got baked into the reset
  offset and every rewind + replay re-displaced the chains (the
  "move by -43.781, play, go back to frame 1, jump again" report).  First
  play, rewind rebuild, and replay now all pin identically and are
  move-invariant; dynamic bodies without a kinematic anchor ride along by the
  detected whole-skeleton move instead of staying at rest.
- **Dynamic bones whose parent bone has no rigid body no longer fly meters
  into the air** — the write-back for a body whose PARENT bone carries no
  rigid body expressed the solved pose relative to the body's own world
  instead of its parent joint's world, so the offset doubled through the
  skeleton chain and the bone landed far above the character (Endmin's
  `shengzi` / `jianjia_fk_a/b` / `piaodai_back_L` chains at y≈14-16 launched
  to y≈33).  The fallback now reconstructs the parent joint's CURRENT world
  from the nearest solver-known ancestor's solved world composed with the
  gap bones' live local matrices (no DG pull, so it is cycle-safe even when a
  dynamic ancestor exists) and expresses the solved bone world relative to
  it.
- **... and they stay put during animation** — the parentless-body fallback
  originally composed the gap bones' local matrices in the wrong order
  (pre-multiplying each local onto the ancestor world instead of
  post-multiplying).  At rest, translation-only locals commute so the result
  was exact; once the chain rotates (animation playback), the reconstructed
  parent world was rotated by the gap offset and the bone launched meters
  away mid-playback (Endmin's `shengzi_0_skin_jnt` flew around the scene
  during animation).  The composition now post-multiplies each local
  (`parentWorld = parentWorld * local`, the transpose of the row-vector
  `world(child) = local(child) * world(parent)`), keeping parentless bones at
  their solved pose through the whole animation.
- **Bones below an `_InheritCtrl` controller now rest at their true PMX
  position** — the solver's write-back offset K is derived from each joint's
  rest WORLD, which the node composes by walking the DAG and reading the
  captured `pmxRest*` attributes.  The hidden `_InheritCtrl` transforms
  (plain DAG nodes inserted by the bone builder for INHERIT_ROTATION) carry
  a real local translate but no `pmxRest*`, so the composition treated them
  as identity and the offset was skipped — K was wrong for every bone below
  them and the write-back moved those joints off their PMX rest (Endmin's
  `shengzi_0/1_skin_jnt` landed ~1.17 units from the model's true rest).  The
  bone builder now stamps `pmxRest*` on each `_InheritCtrl` controller at
  creation (its own local translate, identity rotation), so the rest-world
  composition — and therefore K, the rewind re-pin and the whole-skeleton
  move detector — is exact for these chains.  The bone suite's "Joint World
  Positions" / "Rest Pose Attributes" tests now pass 578/578.
- **Rewinding after a whole-character move no longer drops some chains at
  the origin** — the scrub-back reset offset (bodyRest relative to the
  reset-anchor bone) was captured inside the body-creation loop, where
  `mAnchorRest` was only as large as the kinematic bodies seen so far.  A
  dynamic body whose anchor bone's kinematic body appears LATER in body order
  (Endmin's skirt anchors to a bone whose kinematic body follows it) silently
  failed the `resetAnchorIndex < mAnchorRest.size()` check and got NO reset —
  on every rewind it sat at the freshly-built world's rest pose (x≈0) while
  the skeleton was at x≈16, until the next forward step re-dragged it.  The
  offsets are now captured in a second pass AFTER every kinematic anchor is
  registered, so every anchored body re-pins to the CURRENT skeleton pose on
  rewind regardless of body ordering.
- **`pmxRigidBody` write-back on a shared bone now replaces correctly** —
  when a later dynamic body drives a joint an earlier body already drives, the
  later body takes over the connection ("last wins") instead of silently
  leaving the first body's wiring.
- **Physics bodies reappear after a plugin reload** — `rigid_body_builder` is
  force-reloaded on plugin init so a stale cached module no longer leaves the
  physics node empty after a reload in the same Maya session.
- **`pmxRigidBody` robustness** — anchor offsets no longer drop group scale,
  `-group` is clamped to the PMX 0..15 range, and malformed `-size`/
  `-position`/`-rotation` flags fail loudly instead of silently falling back
  to defaults.
- **Maya SDK resolution prefers the version being built** — a Maya-2027 build
  can no longer resolve the 2026 SDK just because it was cached earlier.

### Removed

- Unused `ci` CMake preset and the redundant `BUILD_NATIVE_MODULE=ON`
  override in the `base` preset.

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
