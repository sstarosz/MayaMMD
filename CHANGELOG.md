# Changelog

All notable changes to MayaMMD will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Changed

- **Full guide removal + direct joint write-back (Phase 3).** Guide transforms
  and `parentConstraint`/`orientConstraint` write-back are **gone** — the
  `mmdPhysicsNode` writes the solved JOINT-LOCAL pose straight into the related
  joints (`outTranslate[i]`/`outRotate[i]` → `joint.translate`/`rotate`), and
  the physics group now contains only the solver locator.  The write-back
  reproduces `parentConstraint(maintainOffset)` exactly: `boneLocal =
  K · bodyLocal · B_parent⁻¹ · M_parent⁻¹` (rest poses stay EXACT — 0
  mismatched bones across all models).
- **Fixed: simulation exploded during animation (DG feedback cycle).** The
  write-back used to read `joint.parentInverseMatrix` from the DG; for a body
  whose parent joint is also node-driven (86% of dynamic bodies — whole
  skirt/hair/cape chains), that is a live DG feedback cycle (Maya allows the
  connection) that amplified every frame (bones displaced up to 54 units).
  The parent inverse is now derived from the **parent body's** solved Bullet
  transform (`bodies[i].bodyParentBodyIndex` + the baked
  `bodyParentJointOffset[i]`), so no write-back depends on a node-driven
  joint's DG matrix.
- **Faster import.** The physics build set ~15k attributes through
  `cmds.setAttr`; scalar body/joint children now use the OpenMaya plug API and
  joint rest matrices are read via `worldMatrix` (physics build ≈ 5.9s → 4.4s
  on a 300-body model).

### Added

- **Physics engine replaced: mayaBullet → native `mmdPhysicsNode` (embedded
  Bullet 3.25).** The mayaBullet binding layer is **gone**. `bulletSolverShape`
  is a *stateful* node — Cached Playback's evaluation cache treats node outputs
  as pure functions of their inputs and never re-steps the solver, so dynamic
  bodies froze at rest and the write-back constraints then locked the skeleton
  ("lost mesh binding"). The replacement is a native C++ `MPxNode`
  (`mmd/maya/nodes/mmd_physics_node.h/.cpp`, registered by `MayaMMD.mll`) that
  owns a `btDiscreteDynamicsWorld` and steps inside `compute()` on every
  `time1.outTime` change. It declares itself **non-cacheable**
  (`MPxNode::getCacheSetup` → `MNodeCacheDisablingInfoHelper::setUnsafeNode`),
  so the evaluation manager always re-evaluates it every frame — the one thing
  mayaBullet's built-in solver could not do. Every PMX rigid body and joint is
  written into the node's `bodies` / `joints` compound arrays; Bullet 3.25
  (float precision) is provided by **vcpkg** (`vcpkg.json`, package `bullet3`,
  pinned via `builtin-baseline`).
- **Bullet now comes from vcpkg instead of a vendored source tree.** The
  ~7,500-file nested clone under `third_party/bullet3/` is gone. The native
  build auto-activates the vcpkg toolchain from `VCPKG_ROOT` (bootstrap vcpkg
  once, set `VCPKG_ROOT`) and links Bullet via `find_package(Bullet CONFIG)`
  (`BulletDynamics` target) — no manual vendoring or `vcpkg install` steps.
  CI uses `lukka/run-vcpkg` with GHA-cache-backed binary caching. The
  `rigidbody`
  integration suite is now registered in the CTest presets too.
- **`mmd/maya/physics_builder.py` rewritten for the C++ node.** Each PMX rigid
  body is a **visible polygonal guide mesh** (sphere/box/capsule at the PMX rest
  pose, Z-flip + handedness, carrying
  `pmxRigidBodyIndex` / `pmxGroupId` / `pmxPhysicsMode`). FOLLOW_BONE guides are
  bound to their bone via `parentConstraint` and feed the node's
  `anchorWorldMatrix` / `anchorParentInverseMatrix` (Bullet solves in the
  physics group's local space). Dynamic guides are driven directly from the
  node's `outTranslate[i]` / `outRotate[i]`. Write-back is DG-driven:
  `parentConstraint(guide, bone)` for PHYSICS, `orientConstraint(guide, bone)`
  for PHYSICS_BONE. No solver plugin, no pairBlend, no scriptJob.
- **Guide meshes shaded by collision group with unique surface shaders.** Each
  collision group gets one shared shader colored from the `_RIGID_BODY_GROUP_COLORS`
  palette (16 distinct colors). Uses Maya 2024+'s standard **`openPBRSurface`**
  shader (color via `baseColor`), falling back to a Lambert on older releases.
  This replaces the old draw-override tint (`overrideEnabled` +
  `overrideColorRGB`), a leftover from the mayaBullet collider-shape era that
  did not color mesh guides. Shaders are assigned to the mesh shapes (not the
  constrained transforms, which emitted set-membership warnings) and are cleaned
  up on teardown.

### Added

- **Native `mmdPhysicsNode` C++ node** (`mmd/maya/nodes/mmd_physics_node.h/.cpp`):
  - Owns a `btDiscreteDynamicsWorld`; time-driven (`time1.outTime → time`),
    evaluated by the evaluation manager every frame, never cached.
  - Compound `bodies` array: rest pose, mass, damping, friction, restitution,
    collider type/size (sphere/box/capsule, capsule Y-axis handled), collision
    group/mask, kinematic flag.
  - Compound `joints` array: body A/B, type (all six PMX joint types),
    frame, linear/angular limits, spring constants. Joint mapping:
    every `SPRING_6DOF` joint → **`btGeneric6DofSpring2Constraint`** — this is
    MMD's own mapping; equal lower==upper limits are a locked axis in spring-2
    (zero springs + all axes locked = rigid weld, replacing the earlier
    `btFixedConstraint`); springy/flexible 6DOF → `btGeneric6DofConstraint`
    with exact-zero components clamped to ±1e-3 (exactly-zero limits are a free
    joint in Bullet); P2P/SLIDER/HINGE/CONETWIST mapped directly.
  - Outputs `outTranslate[i]` / `outRotate[i]` (solved local pose per body)
    that Python connects into the guide meshes.
  - Correct Maya rotate-XYZ convention: `eulerDegreesToQuat` = `qz·qy·qx`
    (column matrix `Rz·Ry·Rx`), verified empirically against Maya 2026,
    with correct gimbal-lock handling for `ry = ±90°`.
- **Physics integration tests — now behavioral, not just structural**
  (`tests/integration/maya/test_pmx_rigid_body_integration.py`): the old suite
  only checked node wiring and could not detect a frozen solver. The new suite
  checks structure (node + body/joint counts, guides at rest, colors, anchors,
  write-back constraints) **and** steps `cmds.currentTime` to assert a dynamic
  body actually moves and its related bone follows (translation for PHYSICS,
  rotation for PHYSICS_BONE). 187/187 pass across all 17 bundled models.

### Fixed

- **Simulation "mess" — anchor orientation (critical)** — `mayaMatrixToBtTransform`
  copied Maya's row-vector matrix directly, but Bullet needs the column-vector
  matrix (the transpose). Every kinematic (FOLLOW_BONE) anchor with a non-trivial
  rotation got a transposed — wrong — orientation (error up to ~0.92), which
  yanked the attached dynamic chains into chaos from the very first frame.
  Verified: an isolated real chain flailed 5.35 units before the fix, 0.12 after;
  the full Tololo model's max body displacement at rest dropped from ~6.5 to ~1.7.
- **Long rigid chains swung like springs** — `btGeneric6DofConstraint` with the
  clamped tiny limits behaved like an underdamped spring on long rigid chains
  (16+ link hair/ponytail/skirt strands: tips swung ~1.3 units and never
  settled). Fully-rigid PMX joints (zero springs AND zero limits) now use
  **`btGeneric6DofSpring2Constraint` with all axes locked** — Bullet treats
  equal `lo==hi` limits as a locked axis, and MMD's engine maps *every*
  SPRING_6DOF joint to this constraint, so it is the MMD-faithful rigid weld.
  *(An earlier attempt to use `btGeneric6DofConstraint` ±1e-4 limits +
  `STOP_ERP=1.0` combined with same-chain collision ignoring destabilized the
  whole simulation (chains collapsed / exploded), so those changes were
  reverted; stability first.)*
- **"The bang is longer than normal"** — hair/skirt chains stored with deeply
  overlapping spheres that self-collide (the PMX `non_collision_group` keeps
  the chain's own group in its mask) had the overlap contacts push the chain
  apart; because locked weld axes are compliant (ERP 0.2), the chain slowly
  extended (~1 unit on Tololo's hair tip) and looked like the bones "fell a
  little bit when they shouldn't". Dynamic bodies now clear their own
  collision-group bit from their mask (hair strands pass through each other)
  while still colliding with every other group. This removes collisions only,
  so it cannot destabilize the sim — Tololo hair tip droop dropped from 0.97
  to ~0.06.
- **"The skirt goes through the legs"** — the Tololo PMX (a converted game
  model) ships degenerate collision masks: every body collides only with its
  OWN group (skirt mask `0x0004`, legs mask `0x0002`), so the skirt could never
  collide with the legs in either direction and fell straight through. The
  builder treats kinematic (FOLLOW_BONE) bodies as the model's static *body*
  colliders and applies a **proximity-based correction**: only kinematic
  groups whose rest-pose extents overlap a dynamic body are OR'd into that
  body's mask (and only overlapping dynamic groups into each kinematic body's
  mask). So the body always blocks the cloth it touches at rest (skirt vs
  legs/hips collide) — while dynamic↔dynamic collisions still follow the PMX
  data and the hair self-collision removal is preserved. Verified: the skirt
  hem now rests on the legs (was falling to the floor) and hair stays at 0.05.
- **"The bangs jump back and forth on the jacket instead of lying on it"** —
  two causes. (1) **Inverted damping conversion:** `damping = 1 -
  attenuation` turned MMD bodies with attenuation ≈ 0.96–1.0 (bangs, skirt,
  cape, hair) into *near-zero* damping, so they never settled and vibrated on
  their resting contact. MMD's `move_attenuation` IS the damping coefficient
  (1.0 = fully damped), so it is now mapped directly (`damping =
  attenuation`). (2) **Missing bangs↔skirt collision:** the bangs (mask
  `0x0012` = head group only) never collided with the jacket/skirt (both
  dynamic), so nothing supported them — they hung free, swung out past the
  jacket and ended "under the arm" at frame 20 instead of resting ON the
  jacket colliders. A new **cloth-on-cloth proximity correction** adds the
  skirt group to the bangs (and vice-versa). It is carefully guarded: chain
  connectivity uses dynamic bodies only (jointed bodies in the same chain
  never collide; a cape sharing only the torso anchor is a separate chain),
  the two chains must be jointed to the same body part (bangs and skirt both
  hang off the torso — bangs↔sleeve/hair, which hang off the arm/head, are
  never added), they must genuinely *drape* (real interpenetration at rest,
  not a mere touch like the cape tips brushing the jacket back), and only a
  SHORT chain (≤ 10 bodies) draping a LARGE sheet (≥ 50 bodies) qualifies —
  so models with different cloth layouts are untouched. Verified (Tololo):
  the bang chain now rests ON the jacket (the jacket collision displaces the
  bangs to its surface; worst frame-to-frame move 0.073 after settling — no
  more jumping), the skirt stays on the legs (worst displacement 0.59), hair
  intact, and all other bundled models are unchanged from baseline.
- **"The skirt floats with a gap from the body"** — capsule colliders were
  oriented WRONG. `btCapsuleShape` is already Y-axis (`m_upAxis = 1`, matching
  MMD's vertical capsule and the polyCylinder guide mesh), but the node
  additionally rotated it −90° about X under the mistaken assumption that
  "Bullet's capsule axis is Z". That turned every capsule sideways (e.g. the
  torso capsule pointed its hemispherical cap at the skirt), pushing the skirt
  ~1 unit out so it floated with a visible gap (jacket-to-torso 3.01 vs the
  contact distance 1.93). Removed the erroneous rotation — the jacket now rests
  at 1.96, colliders match their guide meshes, and the sim is even more stable
  (worst displacement 0.80).
- **Gravity matched to MMD** — MMD's physics engine uses exactly **-9.8**, in
  the model's own unit scale. The previous -98 (a 10x guess for the 19-unit
  Tololo model) made every force 10x too strong: rigid chains sagged under the
  huge PMX masses and collisions were 10x too violent.
- **Scrubbing time backwards broke the animation** — on `dt < 0` the node
  rebuilt the world at the PMX *rest* pose while the skeleton was at another
  frame, so hair/skirt hung at rest and the animation looked broken. It now
  **rebuilds the world and initializes every dynamic body at its rest pose
  transformed by the current skeleton pose** (via the kinematic-anchor delta of
  its nearest kinematic-ancestor bone, mapped in Python), zeroing velocities —
  the sim simply continues from the pose the skeleton is actually in.
  *Two sub-bugs found while validating this: (1) `mAnchorCurrent` was never
  populated (only `mAnchorRest`), so the rewind reset was a silent no-op;
  (2) teleporting bodies *in place* left the solver's warm-start impulse state
  from the previous frame, so the first step after a rewind catastrophically
  yanked the chains off their reset pose. Rebuilding the world on rewind gives
  fresh solver state — verified that forward playback after a rewind matches
  normal playback to within ~0.08 units.*
- **Hair does not follow a bone moved at a fixed time** — the node only stepped
  when the *time* input changed, so dragging a bone in the viewport (no
  playback) left the attached chains behind until the next frame (hair didn't
  follow the head's new position). The node now also steps (one 1/60 s solve)
  when a kinematic anchor moves, detected by comparing each anchor against its
  previous captured pose — MMD reacts to bone changes immediately.
- **Crash on scene teardown (use-after-free in the Bullet world)** — the node's
  `destroyWorld()` deleted the rigid bodies *before* destroying the Bullet
  world. `btCollisionWorld`'s destructor iterates its collision objects and
  destroys their broadphase proxies, so deleting the bodies first read freed
  memory — an intermittent access violation (Maya crashes during scene
  teardown / re-import). The world is now destroyed **first** (while bodies are
  alive), then the bodies, constraints and shapes. The dispatcher, broadphase,
  collision configuration, solver and all collision shapes (which Bullet does
  not own) are now held as node members and freed exactly once.
- **Euler round-trip for gimbal-locked bodies** — the node's
  `quatToEulerXYZDegrees` used the `Rx·Ry·Rz` extraction with a flipped
  gimbal `ry` sign; every body whose rest rotation had `ry = ±90°` (shoulder /
  jacket pivots, common in real models) was rotated 180°, displacing the
  write-back bones and breaking mesh binding at rest. Extraction now matches
  Maya's rotate-XYZ convention (`M = Rz·Ry·Rx`, `sin(ry) = -m[2][0]`) with
  correct gimbal handling; verified 0 bone-position mismatches across all 17
  models.
- **Bone position/rotation tests exempt physics-driven bones** — the
  skeleton-construction tests (`test_pmx_bone_integration.py`) asserted
  `rotate == rest` on every bone; physics-driven bones legitimately have their
  `rotate` driven by the simulation write-back (the world matrix stays exactly
  at rest — the raw Euler may be a non-canonical representation of identity).
  The tests now skip bones driven by dynamic rigid bodies.
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
