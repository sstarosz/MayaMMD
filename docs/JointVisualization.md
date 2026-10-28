# Joint visualization

How PMX rigid-body **joints** (constraints) are drawn in the viewport and how to
read them. Each joint is a `pmxRigidBodyJoint` node, drawn by
`RigidBodyJointGeometryOverride`.

A joint connects two rigid bodies, **A** and **B**. The display answers two
questions:

- **Unselected:** what kind of motion does each joint allow? This lets you
  compare a joint with its neighbours at a glance.
- **Selected:** which bodies does this joint connect, and what exactly are its
  limits?

## Supported joints

Only **PMX 2.0 joints (Spring6Dof)** are supported. They are simulated with
Bullet's `btGeneric6DofSpringConstraint`, the constraint MMD's Bullet 2.75
creates, with linear limits in body A's frame. PMX 2.1
joint types (6Dof, P2P, ConeTwist, Slider, Hinge) reuse the limit and stiffness
fields with different meanings. The importer skips them with a warning, and
`pmxRigidBodyConstraint` rejects any `-type` other than `0`.

## How freedom is decided

Every part of the display uses one model (`mmd/core/joint_display_ranges.hpp`).
It follows Bullet exactly, with no display thresholds:

| Limits on an axis         | Meaning                                               |
| ------------------------- | ----------------------------------------------------- |
| `min == max`              | **Locked** (at that value, which may be non-zero)     |
| `min < max`               | **Limited**, even if the range is tiny                |
| `min > max`               | **Free** (no limit)                                   |

- Limits are compared at Bullet's float precision, and angles are normalized
  to ±180° first, as Bullet does. A range written as `[-200°, 200°]`
  therefore wraps and becomes free.
- Rotation is measured as XYZ Euler angles, so the **measured Y angle cannot
  pass ±90°**. Turning further reads as Y coming back while X and Z jump by
  180°. With X and Z free, that jump is allowed, so a free Y turns all the way
  round. With X or Z limited, the jump breaks their limit, so the body is held
  near ±90°.
- A **spring** only counts on an axis that can move. Stiffness on a locked axis
  has no effect in Bullet, so it is ignored. This is common in MMD models, which
  often set position spring constants while the position limits are zero.

Unit tests in `tests/unit_tests/core/test_rigid_body_simulation.cpp` confirm
these rules against the simulation itself.

## Unselected joints: the hub

Each joint draws a small solid **hub** at its live attachment on body A. The
hub is the joint's pick target; in wireframe mode it is a wire sphere, so it
stays visible and pickable. Its colour shows what kind of motion the joint
allows:

| Hub                           | The joint…                                 |
| ----------------------------- | ------------------------------------------ |
| Grey                          | is **welded**: nothing can move            |
| Cyan                          | **rotates** only                           |
| Rose                          | **translates** only                        |
| Cyan and rose wedges          | **rotates and translates**                 |
| Hollow, dashed, dark grey     | is **inactive**: it constrains nothing     |

The two-colour hub alternates its wedges all the way round, like a beach
ball, so both colours show from every direction: no part of the display
changes meaning with the viewing angle.

A joint is **inactive** when a body is missing (unconnected or deleted),
both ends are the same body, a body is disabled, or both bodies follow
their bones, so the constraint has nothing to move. The rule is
`RigidBodySimulation::jointStatus`, the same check that decides which joints
the simulation creates. The hub's form (hollow and dashed, in every display
mode) tells it apart from a welded hub, not its colour.

Springs are not shown on the hub: a single marker could not say which axis is
sprung. Select a joint to see its springs per axis (see below).

The hub colours (cyan, rose) and the body colours (violet, gold, below) sit
about 90° apart on the colour wheel, away from the axis colours and Maya's
selection green, and stay distinct with the common colour-vision
deficiencies. All joint colours are defined in the palette at the top of
`mmd/maya/nodes/rigid_body_joint_geometry_override.cpp`.

## Selected joints

Selecting a joint (its hub, or its transform) adds overlays:

- **Selection outline**: a wire around the hub in Maya's selection colour
  (green for the lead, white for other selected objects). This is the only
  overlay a joint draws when it is selected through a parent group: selecting
  `{model}_Joints` would otherwise draw hundreds of overlays at once.
- **Bodies**: lines from each body's centre to its own live attachment point.
  Body A uses **violet** rings and body B uses **gold** diamonds. When the two
  attachments coincide, which is normal for a satisfied joint, they share one
  ring with a gold centre. The connected colliders are outlined in the same
  colours, and a body that is A for one selected joint and B for another is
  **striped violet and gold**. An inactive joint draws only **dashed** links
  from its hub to the bodies it names, and no motion ranges.
- **Rest frame**: the joint's transform is its frame, and the Move and Rotate
  manipulators sit there, but the hub rides on body A. Once body A has
  carried the hub away, a dashed white **ghost** of the hub marks the frame,
  with a dashed tether to the hub.
- **Motion ranges** (Ranges draw mode), drawn at attachment A: translation in
  its frame, rotation on nested dials (below). Red, green and blue mean local
  **X, Y, Z**:
  - *Translation*: the allowed region at its real size. One movable axis gives
    an offset ruler with witness lines, two give a rectangle, three a box.
    Free axes are open arrows. An axis locked at a non-zero offset shows a
    target cube, with a dashed line from B's current attachment to it.
  - *Rotation*: one arc per axis with end stops and a white zero notch, plus
    a bold **needle** at the joint's current angle, so you can watch a body
    press against its stop during playback. A free axis is a full ring. An
    axis locked at a non-zero angle shows a dashed sweep from zero and a
    target spoke; the sweep is a reference, not an allowed range. The Y arc
    is cut at ±90°: the configured part beyond that is dashed, because the
    joint cannot reach it, and there is no end stop at the cut, because no
    limit stops the body there. A free Y is a full ring when X and Z are free
    too. The arcs are separate per-axis limits, not one combined rotation
    envelope.
  - The dials **nest like a gimbal**. Bullet composes the joint rotation as
    Z (about A's Z axis), then Y, then X (about B's own X axis), so each dial
    lies in the frame its angle is measured in: the outer Z dial stays in A's
    frame, the Y dial turns with the current Z, and the inner X dial turns
    with Z and Y. During playback the Y and X dials follow the body.
  - Arcs, stops and needles are drawn where body B **really turns**. Bullet
    measures joint angles with the opposite sign of a right-handed rotation,
    so a limit `[lo, hi]` is drawn at `[-hi, -lo]`; the needle uses Bullet's
    own measurement, so it always reads against the limits correctly.
  - *Limit hits*: when B presses on a limit, that limit is drawn **bold**.
    On a dial the needle and the stop it touches thicken. On a translation
    ruler the end turns bold in its axis colour; on a rectangle or box, so do
    the edges of each face B touches. "Pressing" means within 2% of the range
    of the limit (at least 0.5° for rotation), or past it. The cut of a Y
    range at ±90° is not a limit, so it never lights up.
  - *Springs*: on a rotation axis with a spring, the arc itself is drawn as a
    zigzag across its whole allowed span, so one element shows both the
    limits and the spring. On a translation axis, a straight zigzag sits along
    the axis. The size does not show stiffness. The white cross or zero notch
    marks the spring's rest point, which this solver always places at zero.

Dashed always means "not reachable": a reference, never an allowed range.

Angular guides, spring badges and markers are sized from the hub, not from
the camera, so every viewport shows the same overlay and it always clears the
hub. Translation regions always keep their physical size.

Joints usually sit inside cloth and hair. The overlay is therefore drawn twice:
a faint **X-ray ghost** that shows through the mesh, and a full-strength pass
that is depth-tested. Parts in front of the mesh look solid; parts behind it
look faint.

If a connected body is hidden, or isolated out of the current view, the
selected joint still draws that body's outline. Isolation, visibility and the
selection are never modified. Isolate-select can only be queried on Maya
2026 and later; on older versions only hidden bodies get this outline.
Turning isolate select on or off (any model-editor change) redraws every
rigid body and joint: Viewport 2.0 could otherwise leave bodies that had been
isolated out undrawn until something else refreshed them.

## Updates during playback

Each joint's hidden `solverStep` input is connected to its solver's
`outStep` output. This real DG dependency brings every solver step to the
joints, and reading it makes the solver evaluate before the joint reads the
simulated poses. The joint node then tells Viewport 2.0 its drawing is out of
date: from `setDependentsDirty` under DG evaluation, and from `postEvaluation`
under the Evaluation Manager, which propagates no dirty during playback. A
joint redraws only when something it draws changes; a camera move costs
nothing.

At a paused frame the solver also re-runs at once when you drag a bone, edit
a joint or body attribute, or move a joint's transform: each body and joint
input feeds its node's `solver` message plug. A joint's transform reaches it
through the hidden `frameMatrix` input, which `pmxRigidBodyConstraint`
connects to the transform's `worldMatrix`.

## Draw modes

`drawMode` on each joint:

| Value | Label  | Draws                                                            |
| ----- | ------ | ---------------------------------------------------------------- |
| 0     | Off    | nothing                                                          |
| 1     | Hub    | the hub; when selected, the body links, outlines and rest frame  |
| 2     | Ranges | Hub, plus the selected joint's motion ranges (the default)       |

## Size

All hubs of a model share one size, derived from the median rest-pose
distance between the bodies its joints link. It reads well at any model scale,
and it never changes during the simulation. The guides are sized from the hub. `jointDisplayScale` on the model's
`pmxRigidBodyNode` ("Joint Hub Size" in the Attribute Editor) multiplies every
hub in that model.

## Attributes and units

- The joint's transform **is** the joint frame. Move or rotate it to move the
  constraint. The frame is read in the `{model}_RigidBodies` group's space,
  the space of the bodies' rest poses, so moving the model root never
  re-rigs a joint. The group and its `{model}_Bodies` / `{model}_Joints`
  sub-groups follow the model root only: their transform channels are locked
  at import.
- A joint is bounded around its live hub, so Frame Selected (F) works on it.
- `jointBodyA` / `jointBodyB` are **message connections** from the bodies'
  `pmxRigidBodyShape.message`, not indices. The Attribute Editor shows each
  as a field naming the body, with a button that jumps to it. Reconnect one
  to re-target the joint. Deleting a body disconnects it, which makes the
  joint inactive, and never shifts other joints onto the wrong bodies. The
  solver turns the connections into its body indices.
- `jointAngularMin` / `jointAngularMax` are angle attributes. The Attribute
  Editor and `cmds.getAttr`/`setAttr` use Maya's UI angle unit (degrees by
  default); internally they are stored in radians. PMX files and the
  `pmxRigidBodyConstraint` flags use radians.
- `jointLinearStiffness` / `jointAngularStiffness` are Bullet axis stiffnesses,
  taken from the PMX spring constants.

## Trying it

`scripts/maya/joint_glyph_demo.py` builds a gallery of 18 joints, each showing
one case (line, plane, box, fixed offsets, free axes, angular limits, springs,
rotated frames, body modes). It never modifies existing scene content:

    exec(open(r"<repo>/scripts/maya/joint_glyph_demo.py").read())

## Implementation map

| File | Role |
| ---- | ---- |
| `mmd/core/joint_display_ranges.hpp` | the freedom model (per-axis kinds, effective springs, summary) |
| `mmd/core/rigid_body_simulation.hpp` | `jointStatus`: whether a joint constrains anything (inactive hubs) |
| `mmd/core/joint_guides.hpp` | motion-range guide geometry as line segments, gimbal dials and limit hits (unit-tested) |
| `mmd/core/joint_display_sizes.hpp` | hub-relative sizes for guides and markers |
| `mmd/maya/nodes/rigid_body_display.*` | everything both overrides share: the palette (every colour, one channel per hue), collider outlines and the A/B role wire, the lit stock shader (OpenPBR on 2026+, Blinn before), selection and isolate-select events handled once per session, shared shaders, and which selected joint outlines a shared hidden body |
| `mmd/maya/nodes/rigid_body_joint_geometry_override.*` | the VP2 override: the hub mesh (triangles and wire) and the joint-specific drawing on top of the shared display module |

Headless tests cannot show viewport output. After changing the display, check
it interactively: select and deselect joints, select several, play and rewind
(also with Parallel Evaluation), switch to wireframe and pick a hub, use
several viewports, isolate-select and Show All, hide a connected body, and
look at joints inside cloth (the X-ray ghost). Also delete a body (its
joints turn hollow), play until a body presses on a limit (bold needle or
face), and pause mid-animation to see the rest-frame ghost and tether.
