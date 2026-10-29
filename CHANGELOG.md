# Changelog

All notable changes to MayaMMD will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Added

- **Per-body rigid-body viewport guides** — every PMX rigid body now imports
  as its own selectable `pmxRigidBodyShape` locator node (a TRANSFORM guide
  holding the locator shape):
  - Select an individual body's guide in the viewport / Outliner — it is
    **driven by the solver to the body's CURRENT pose each frame**, so the
    collider follows the animation: kinematic bodies track their bone,
    dynamic bodies track the simulated pose, disabled bodies sit at rest.
  - The body's REST pose lives in the shape's `bodyRestTranslate` /
    `bodyRestRotate` attributes (editable in the Attribute Editor; the
    solver rebuilds from them), not on the animated guide transform.
  - `drawMode` attribute per body (Off / Wire / Solid / Wire+Solid) with a
    16-colour collision-group palette, kinematic bodies dimmed, disabled
    bodies greyed, and the selected guide highlighted.
  - The SOLID surface is rendered **lit** by the viewport lights (the stock
    OpenPBR surface shader via an `MPxGeometryOverride`, matching the
    imported meshes' `openPBRSurface` look — replaces the old flat-colour
    `MUIDrawManager` solid), drawn on top of the character mesh so interior
    colliders stay visible; the wire outline is unchanged.
  - Shape type / size / collision group / mask / physics mode are all editable
    in the Attribute Editor.
- **Attribute-Editor templates for the rigid-body nodes** — shipped as
  `AEpmxRigidBodyShapeTemplate.xml` / `AEpmxRigidBodyNodeTemplate.xml` (the
  documented XML AE-template mechanism), installed into the module's
  `scripts/AETemplates/`.  The plugin registers that folder on
  `MAYA_CUSTOM_TEMPLATE_PATH` at load time (computed from the plugin's own
  location), so the templates work whether the plugin is loaded via
  `MayaMMD.mod` on `MAYA_MODULE_PATH` or directly via `MAYA_PLUG_IN_PATH` /
  the shelf.  The generated `MayaMMD.mod` sets `MAYA_CUSTOM_TEMPLATE_PATH`
  with an ABSOLUTE path (the module loader does not resolve relative paths
  for custom variables) and the dev build auto-deploys the `.mod` into the
  Maya modules folder (`Documents/maya/<version>/modules`), so no manual
  copy is needed.  The body data is shown as the node's primary Attribute-Editor
  content with proper labels (Local Name, Collision Group, Shape Size, …)
  instead of one flat "Extra Attributes" list.  The grouped "Body" view
  (Body / Rest Pose / Physics / Collision Mask / Display) is the default —
  the plugin pre-sets the per-type view optionVar the AE's Views menu uses,
  because the AE only builds a custom XML view when a named view is active
  (otherwise it shows the plain editor with Extra Attributes).  The
  `pmxRigidBodyNode` solver gets a small classic MEL template (Solver:
  `gravity`; Display: Joint Hub Size) that hides its internal arrays and
  outputs; the per-body and per-joint data lives on the connected
  `pmxRigidBodyShape` (grouped XML template) / `pmxRigidBodyJoint` nodes.
  The joint also gets a MEL template (Joint / Limits / Stiffness / Display),
  because its Body A / Body B are message connections, which only a MEL
  navigation control can show.
- **`pmxRigidBodyShape` native node** — a C++ `MPxLocatorNode` that carries
  one PMX body's data (PMX-verbatim fields) and is drawn by a registered
  `MPxDrawOverride`.
- **`pmxRigidBodyNode.jointDisplayScale`** (double, default `1.0`, AE "Joint
  Hub Size") — a viewport-only multiplier for the joint hubs.  All hubs of a
  model otherwise share one size, derived from the median rest distance
  between the bodies its joints link; this is the per-model knob when the
  joints read too small or too large.
- **Joint visualization** (`RigidBodyJointGeometryOverride`; see
  `docs/JointVisualization.md`):
  - every joint draws a pickable **hub** at its live attachment A, coloured by
    what the joint permits — welded (grey) / rotates (cyan) / translates
    (rose); a joint that does both shows both colours in alternating wedges,
    so both read from every direction.  In wireframe mode the hub is a wire
    sphere, so it stays visible and pickable.  A joint is bounded around its
    live hub, so Frame Selected (F) works on it;
  - a joint **picked directly** (its hub or transform) shows its body links
    (A violet, B gold; body centres are small diamonds, attachments larger
    rings), its connected colliders' outlines
    (striped violet/gold for a
    body that is A for one selected joint and B for another), and in the
    Ranges draw mode its **motion ranges**: real-size translation regions and
    per-axis angle arcs in local X/Y/Z colours, each with a **needle** at the
    joint's current angle; a sprung rotation axis draws its arc as a zigzag
    over the allowed span, a sprung translation axis gets a zigzag badge.
    Arcs are drawn where the body really turns (Bullet's joint angles have
    the opposite sign of a right-handed rotation, so a limit `[lo, hi]` is
    drawn at `[-hi, -lo]`).  The overlay has a faint X-ray ghost so joints inside
    cloth and hair stay readable.  Its size follows the hub, not the camera,
    so every viewport shows the same overlay.  A joint selected only through a
    parent group (e.g. all of `{model}_Joints`) keeps the selection wire but
    draws no overlay;
  - the Y rotation dial is cut at ±90°, the furthest the constraint's Euler
    decomposition reaches; the configured part beyond that is dashed, with
    no end stop at the cut.  A free Y with free X and Z is a full ring (the
    Euler angles flip instead of stopping);
  - the rotation dials nest like a **gimbal**, in the order Bullet measures
    the angles: Z in body A's frame, Y turned by the current Z, X turned by Z
    and Y, so every needle points where the body really is;
  - a limit the body **presses on is drawn bold**: the needle and its end
    stop, or the translation ruler's end / the region's faces in that axis's
    colour;
  - a joint that **constrains nothing** (a missing, disabled or shared body,
    or two bone-driven bodies — `RigidBodySimulation::jointStatus`) has a
    hollow dashed hub, and when picked draws only dashed links to its bodies;
  - hidden or isolated-out connected bodies are still outlined for the
    selected joint (isolate-select detection needs Maya 2026+; older versions
    outline hidden bodies only);
  - the display follows Bullet's freedom rules exactly, at Bullet's float
    precision: equal limits lock, any `min < max` range moves (no
    thresholds), inverted limits are free, and stiffness on a locked axis is
    not shown as a spring (it has no effect);
  - joints redraw after every solver step: a real DG connection
    (`pmxRigidBodyNode.outStep` → `pmxRigidBodyJoint.solverStep`) brings
    each step to the joint, which marks its drawing dirty in
    `setDependentsDirty` (DG) and `postEvaluation` (Evaluation Manager), so
    hubs follow the simulation during Parallel / Serial playback too.  They
    do not redraw on camera moves.
- **PMX physics validation** (`mmd/core/pmx_validate.py`, category *Physics*,
  run by the MMD window before an import) — reports the physics-authoring
  faults that otherwise show up only as strange viewport behaviour:
  - a joint connected to a rigid body the model does not have (an error: the
    constraint cannot be built);
  - a joint **placed far away from the two bodies it connects** — a joint is
    normally placed where its bodies meet, so one sitting far from both is
    either a leftover (a joint at the world origin imports as a hub floating
    far from the model) or a pivot that swings the bodies around the wrong
    point;
  - a joint that **cannot hold its bodies** within the model's own size (every
    axis free or opened wider than the model, no stiffness set).

  Each report names the joints and their PMX indices, so they can be fixed in
  PMX Editor.

### Changed

- **A large forward jump in time now rebuilds instead of stepping** — one
  `step()` can carry the simulation at most `kMaxStepTime` = 0.5 s (exposed as
  `RigidBodySimulation::maxStepSpan()`; the sub-step budget is sized so the
  whole span is simulated exactly). Scrubbing forward further used to step
  anyway, truncating the span and leaving every cloth/hair chain lagging behind
  the skeleton: on `Endmin_Female.pmx`, jumping from frame 1 to 2620 showed real
  joint separations up to 32 units (150 joints over 1), which read as broken
  constraints. The world is now rebuilt for such a jump — bodies are re-pinned
  to the current skeleton pose, so the landed frame is self-consistent (max 1.4
  units, 6 joints over 1) and identical on re-evaluation. Frames at 24/30/60 fps
  and multi-frame preview skips stay inside the span, so they are still
  simulated rather than re-pinned.
- **Selected-joint markers are no longer ambiguous** — the body centres were
  drawn as the same shape as the attachments (for body B: two identical gold
  diamonds, so a body's centre could be mistaken for a separated attachment).
  Now every **body centre** is a small diamond and every **attachment** a
  larger ring, for both bodies.
- **Joint display refactor (no behavior change)** — the two geometry overrides
  now share their plumbing instead of duplicating it:
  - `rigid_body_display::fillMeshBuffers` fills a render item's vertex buffers
    (it was copied verbatim in both overrides), and
    `rigid_body_display::appendSphereMesh` is the one UV-sphere builder behind
    both the hub and the sphere collider (it keeps the pole-single-vertex form,
    so the collider no longer emits degenerate pole triangles);
  - `isSelected` / `selectionColor` live in `rigid_body_display` (they were
    duplicated, `selectionColor` with two different signatures), and the
    collider's `kSelectionR/G/B` copy of `palette::kSelectionFallback` is gone;
  - the hub's `Freedom` summary moved from the override into
    `joint_display.hpp` next to `describe()`, and the `JointState` enum plus its
    two-boolean re-encoding were replaced by that summary; the hub colours are
    now derived from it directly;
  - the hub-size policy moved into `joint_display.hpp`
    (`hubRadiusForLinks`, unit-tested) — one formula, one name — and
    `World::jointHubRadius` / `RigidBodyNode::jointHubRadius()` became
    `hubRadius` for consistency with it;
  - `RigidBodyJoint::resolveSolverView` resolves the owning solver, body
    indices, live attachment frames and hub radius once, for both
    `boundingBox()` and the viewport override (which also removes the
    duplicated 0.05 display-scale clamp);
  - `PMXNamingManager._unique_names` replaces the three near-identical
    `_preprocess_{blendshape,joint,rigid_body}_names` uniquifiers.
- **`scripts/maya/joint_glyph_demo.md` is gone** — the gallery instructions now
  live in `docs/JointVisualization.md` ("Trying it"), so there is one place to
  update when the visuals change.
- **Breaking: joint "spring" attributes/flags renamed to "stiffness"** — the
  value is Bullet's axis stiffness (`btGeneric6DofSpringConstraint::setStiffness`),
  not a physical spring constant, so the user-facing names now say so:
  - node attributes `jointLinearSpring` / `jointAngularSpring` →
    `jointLinearStiffness` / `jointAngularStiffness` (shorts `jls` / `jas` →
    `jlsk` / `jask`);
  - command flags `-linearSpring` / `-angularSpring` →
    `-linearStiffness` / `-angularStiffness` (shorts `-ls` / `-as` → `-lsk` / `-ask`);
  - AE labels "Linear/Angular Spring" (group "Springs") → "Linear/Angular
    Stiffness" (group "Stiffness").

  Re-import required (saved scenes referencing the old attribute names must be
  rebuilt).  The PMX source fields (`position_spring_constant`,
  `rotation_spring_constant`) and the joint type `Spring6Dof` are unchanged.
- **Joints are simulated with `btGeneric6DofSpringConstraint`** (was
  `btGeneric6DofSpring2Constraint`), the constraint MMD's Bullet 2.75
  creates, with linear limits in body A's frame.
- **The solver runs Bullet's default 10 iterations, as MMD does** (was 30).
  The 30 were needed for the Spring2 constraints, whose long chains
  detached from their anchors at 10; the Spring6Dof chains hold (no
  physics-driven bone stretched over 0.08 units in two test models).
- **A joint's frame is its transform in the `{model}_RigidBodies` group's
  space**, the space of the bodies' rest poses, so moving the model root no
  longer moves the pivots off their bodies.  The collider guides are
  written against the group's current world matrix, so they follow a
  model-root move once (they showed it twice until a rebuild).
- **The `{model}_RigidBodies` group and its `{model}_Bodies` /
  `{model}_Joints` sub-groups have locked transform channels**: they only
  follow the model root.
- **Joint `drawMode` is now Off / Hub / Ranges** (was Off / Wire / Solid /
  WireSolid).  New joints default to Ranges.
- **Breaking: joints link their bodies by connection.**
  `pmxRigidBodyJoint.jointBodyA` / `jointBodyB` are message attributes
  connected from the bodies' `pmxRigidBodyShape.message` (they were PMX body
  indices).  Deleting or reordering bodies no longer re-targets joints; a
  joint that loses a body is drawn inactive.  `pmxRigidBodyConstraint
  -bodyA/-bodyB` still take PMX indices and make the connections.
  Re-import required.
- **Rigid bodies and joints are named by `PMXNamingManager`**: the local
  name when it is ASCII-safe, else the universal name, else
  `RigidBody_{index}` / `Joint_{index}` (bodies used to keep only the ASCII
  characters of the local name, falling back to `RB_<n>`).  Joints are
  `{model}_{joint}_Constraint`: PMX models often name a joint like the body
  it moves, and `_Jnt` already marks bones.  `pmxRigidBody` and
  `pmxRigidBodyConstraint` gained `-nodeName`, which the importer uses; both
  fall back to the same policy without it.
- **Breaking: `jointAngularMin` / `jointAngularMax` are angle attributes** — the
  Attribute Editor and `cmds.getAttr`/`setAttr` use Maya's UI angle unit
  (degrees by default) instead of raw radians.  The stored value and the
  `pmxRigidBodyConstraint` flags are still radians.  Re-import required.
- **Breaking: only PMX 2.0 joints (Spring6Dof) are supported.**  The importer
  skips PMX 2.1 joint types (6Dof, P2P, ConeTwist, Slider, Hinge) with a
  warning.  The 2.1 types reuse the limit/stiffness
  fields with different meanings, so they are not misread as Spring6Dof.
- **Breaking: the `jointType` attribute and the `-type` flag are gone.**  They
  could only ever hold one value (`Spring6Dof`), so they were a dropdown that
  did nothing and a flag that could only be `0`; PMX 2.1 types are already
  rejected at import.  Remove `type=...` from `pmxRigidBodyConstraint` calls
  and `jointType` reads/writes on saved scenes.
- **Rigid-body schema: `bodies[]` compound → `bodyShapes[]` + per-body nodes**
  (breaking — re-import required).  `pmxRigidBodyNode.bodyShapes[]` is now a
  message array connected to one `pmxRigidBodyShape` per body; the body's
  data attributes (which previously lived as `bodies[i].body*` children on
  the solver) now live on the shape node, and the rest pose lives in the
  shape's `bodyRestTranslate`/`bodyRestRotate` attributes instead of
  `bodies[i].bodyRestTranslate`/`bodyRestRotate`.
- **Guides now follow the simulation** — the solver gained per-body
  `outGuideTranslate[]`/`outGuideRotate[]` outputs (the body's current world
  pose in the RigidBodies group's space), and `pmxRigidBody` connects them to
  the guide transforms, so the collider visualization tracks the animated
  body instead of sitting at rest.

### Fixed

- **Rigid bodies could stay undrawn after isolate select** (e.g. select a
  joint, Ctrl+1, deselect, Ctrl+1) until something refreshed them, such as
  selecting the model root.  Every model-editor change (isolate on or off,
  display mode) now redraws all rigid-body shapes and joints.
- **Box colliders were half their MMD size** — the PMX box `shape_size` IS the
  Bullet half-extent (MMD feeds it verbatim to `btBoxShape`; nanoem /
  MMDAI / blender_mmd_tools agree), but both the engine (`applyShapeSize`)
  and the viewport draw override halved it, so every box rigid body drew and
  collided at half size (jacket / Beg colliders visibly smaller than MMD
  Editor).  Box `shape_size` is now stored and drawn verbatim (full span =
  2 × shape_size), matching MMD.
- **`pmxRigidBody` no longer collapses every body onto index 0** — appending
  to the solver's message array used `MPlug::numElements()` (always 0 for a
  non-cached message array), so each `connectOrReplace` overwrote element 0
  and only the last body survived once the array had never been materialized
  (e.g. when the solver had no time connection yet).  The next-free index is
  now derived from `evaluateNumElements()`, which materializes the array's
  backing store so each body lands on its own element.
- **Bone drags and edits at a paused frame now re-run the simulation** —
  the solver reads bodies and joints through its `bodyShapes[]` /
  `jointShapes[]` message arrays, so moving a bone, editing a body or joint
  attribute, or moving a joint's transform never dirtied it: the rigid
  bodies and the joint hubs on them stayed put until the time changed.
  Every body and joint input now affects its node's `solver` message plug
  (a joint's transform through the new hidden `frameMatrix` input), and the
  solver pulls both arrays on every evaluation so the path is clean for the
  next edit (Maya stops propagating at a plug left dirty).
- **Memory leak on every solver evaluation** — body, joint and anchor values
  were read through `MPlug::asMDataHandle()` without releasing the handle.

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

- **Cross-platform builds (Linux/macOS)** — the release pipeline now installs
  the OpenGL/X11 development headers needed to compile against the Maya SDK on
  Linux, and the `pmxRigidBody`/`pmxRigidBodyConstraint` command headers
  include `<maya/MSyntax.h>`/`<maya/MArgList.h>` instead of forward-declaring
  those types (which clang on macOS rejects against Maya 2027's `USING_LATEST_API`
  namespace aliases). Windows and Maya 2027 builds verified locally.
- **VMD playback range no longer shrinks** — applying a second (shorter)
  animation no longer truncates the timeline and clips a longer animation
  that is already applied; the playback range is now extended only.
- **`pmxRigidBodyNode` first evaluation could pop a posed skeleton to rest** —
  the first evaluation now goes through the same build path as a config edit /
  scrub-back (anchors applied + dynamic bodies reset to the current skeleton
  pose).
- **CCD IK solver reliability** — `ccdIKSolverNode` no longer overshoots the IK
  target or bends hinge joints (e.g. the knee) the wrong way when the target is
  moved incrementally; a "fold boost" resolves the near-straight-leg
  singularity so the chain converges even for small raises. Backed by a
  regression test on a real leg IK setup.
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
  offset and every rewind + replay re-displaced the chains.  First play, rewind
  rebuild, and replay now all pin identically and are move-invariant; dynamic
  bodies without a kinematic anchor ride along by the detected whole-skeleton
  move instead of staying at rest.
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
- **A disabled kinematic body no longer breaks the write-back of every
  anchor after it** — `writeOutputs` pass 1 indexed the raw anchor-world
  array with a kinematic counter that incremented for DISABLED kinematic
  bodies too, but the array only records ENABLED anchors.  A disabled
  kinematic body before an enabled one shifted every subsequent anchor read,
  so the enabled anchor's bone world was read from the wrong slot (or
  skipped) and a dynamic body driven on a child bone wrote its raw WORLD pose
  as the joint-local pose — doubling the skeleton offset (a joint expected at
  y≈6 landed at y≈11).  The pass-1 counter now advances only for ENABLED
  kinematic bodies, and a regression test covers a disabled-anchor-first
  scene.
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
