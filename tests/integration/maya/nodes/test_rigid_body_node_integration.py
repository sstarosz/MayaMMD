"""
test_rigid_body_node_integration.py

Integration tests for RigidBodyNode (pmxRigidBodyNode) — the native rigid-body
physics node (embedded Bullet via the Maya-free mmd_core engine).

Tests cover:
- Node registration and creation
- Attribute surface and key defaults (gravity, collision mask)
- Empty-node evaluation (no bodies = valid no-op, no errors)
- End-to-end simulation: a single dynamic body falls under gravity
- Kinematic anchor driving a rigidly-welded dynamic body
- A config edit forcing an in-place rebuild at the current pose
- The Bullet world running in world space (the solver's own location is
  irrelevant)
"""

# ── Maya standalone initialised by the test runner ───────────────────────

from maya import cmds  # noqa: E402

from tests.integration.test_helpers import (  # noqa: E402
    setup_test_environment,
    assert_true,
    assert_eq,
)


_NODE_TYPE = "pmxRigidBodyNode"

# Attribute-enum values (RigidBodyNode.hpp): collider type + physics mode.
_COLLIDER_SPHERE = 2  # kColliderSphere
_PHYSICS_MODE_FOLLOW_BONE = 0
_PHYSICS_MODE_PHYSICS = 1

# Maya row-vector convention: translation lives in the LAST row (m30..m32).
_IDENTITY_MATRIX = (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)


def _create_node(name: str = "testRigidBodyNode") -> str:
    """Create a pmxRigidBodyNode and assert it exists."""
    node = cmds.createNode(_NODE_TYPE, name=name)
    assert_true(cmds.objExists(node), f"{_NODE_TYPE} creation failed")
    return node


def _connect_time(node: str) -> None:
    """Drive the node's time input from the scene's time1 node."""
    cmds.connectAttr("time1.outTime", f"{node}.time")


def _set_body_common(node: str, index: int) -> str:
    """Populate the shared body-compound fields (enabled, collider, damping)."""
    p = f"{node}.bodies[{index}]"
    cmds.setAttr(f"{p}.bodyEnabled", True)
    cmds.setAttr(f"{p}.bodyColliderType", _COLLIDER_SPHERE)
    # PMX shape_size verbatim — sphere radius = shape_size[0].
    cmds.setAttr(f"{p}.bodyShapeSize", 0.5, 0.5, 0.0, type="double3")
    cmds.setAttr(f"{p}.bodyRestRotate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p}.bodyLinearDamping", 0.0)
    cmds.setAttr(f"{p}.bodyAngularDamping", 0.0)
    cmds.setAttr(f"{p}.bodyFriction", 0.5)
    cmds.setAttr(f"{p}.bodyRestitution", 0.0)
    return p


def _set_dynamic_body(node: str, index: int, rest_y: float) -> None:
    """Populate one dynamic (full-physics) sphere body at (0, rest_y, 0)."""
    p = _set_body_common(node, index)
    cmds.setAttr(f"{p}.bodyRestTranslate", 0.0, rest_y, 0.0, type="double3")
    cmds.setAttr(f"{p}.bodyMass", 1.0)
    cmds.setAttr(f"{p}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)


def _stamp_joint_rest(joint: str, tx: float, ty: float, tz: float) -> None:
    """Stamp pmxRest* attributes on a mock joint (as the bone builder does).

    The node derives each body's write-back offset K from these attributes +
    jointOrient, so tests that exercise the write-back must carry them.
    """
    for axis, value in (("X", tx), ("Y", ty), ("Z", tz)):
        cmds.addAttr(
            joint,
            longName=f"pmxRestTranslate{axis}",
            attributeType="double",
            defaultValue=0.0,
        )
        cmds.setAttr(f"{joint}.pmxRestTranslate{axis}", value)
        cmds.addAttr(
            joint,
            longName=f"pmxRestRotate{axis}",
            attributeType="double",
            defaultValue=0.0,
        )
        cmds.setAttr(f"{joint}.pmxRestRotate{axis}", 0.0)


def _read_output(node: str, index: int) -> tuple[float, float, float]:
    """Force evaluation and read outTranslate[index] (unit-typed compound)."""
    cmds.dgeval(f"{node}.outTranslate")
    # Maya returns compound values as a list containing one tuple, e.g.
    # [(x, y, z)] — unwrap before indexing.
    return tuple(cmds.getAttr(f"{node}.outTranslate[{index}]")[0])


def _set_welded_chain(node: str) -> None:
    """Set up body 0 (kinematic anchor) + body 1 (dynamic) rigidly welded."""
    # Body 0: kinematic anchor (followBone) at the origin.
    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)

    # Body 1: dynamic, 1 unit above the anchor.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)

    # Rigid weld: SPRING_6DOF (type 0) with zero limits = locked.
    j = f"{node}.joints[0]"
    cmds.setAttr(f"{j}.jointBodyA", 0)
    cmds.setAttr(f"{j}.jointBodyB", 1)
    cmds.setAttr(f"{j}.jointType", 0)
    cmds.setAttr(f"{j}.jointFrameTranslate", 0.0, 0.5, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointFrameRotate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearSpring", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularSpring", 0.0, 0.0, 0.0, type="double3")


# ──────────────────────────────────────────────────────────────────────────
# Test Cases
# ──────────────────────────────────────────────────────────────────────────


def test_node_registration_and_creation():
    """The node type is registered by the plugin and can be created."""
    setup_test_environment()

    node = _create_node()
    assert_eq(cmds.nodeType(node), _NODE_TYPE, "unexpected node type")

    # The node is an MPxLocatorNode — it gets a DAG shape under a transform.
    parent = cmds.listRelatives(node, parent=True)
    assert_true(bool(parent), "locator node should be parented under a transform")

    print(f"✓ {_NODE_TYPE} registered and created")
    return True


def test_attribute_surface_and_defaults():
    """All expected attributes exist, with correct defaults."""
    setup_test_environment()
    node = _create_node()

    top_level = [
        "time",
        "gravity",
        "bodies",
        "joints",
        "outTranslate",
        "outRotate",
    ]
    for attr in top_level:
        assert_true(
            cmds.attributeQuery(attr, node=node, exists=True),
            f"missing top-level attribute {attr}",
        )

    body_children = [
        "bodyEnabled",
        "bodyNameLocal",
        "bodyNameUniversal",
        "bodyGroupId",
        "bodyMaskGroup0",
        "bodyMaskGroup15",
        "bodyColliderType",
        "bodyShapeSize",
        "bodyRestTranslate",
        "bodyRestRotate",
        "bodyMass",
        "bodyLinearDamping",
        "bodyAngularDamping",
        "bodyRestitution",
        "bodyFriction",
        "bodyPhysicsMode",
        "bodyJoint",
        "bodyAnchorWorld",
    ]
    for child in body_children:
        assert_true(
            cmds.attributeQuery(child, node=node, exists=True),
            f"missing body-compound child {child}",
        )

    joint_children = [
        "jointNameLocal",
        "jointNameUniversal",
        "jointBodyA",
        "jointBodyB",
        "jointType",
        "jointFrameTranslate",
        "jointFrameRotate",
        "jointLinearMin",
        "jointLinearMax",
        "jointAngularMin",
        "jointAngularMax",
        "jointLinearSpring",
        "jointAngularSpring",
    ]
    for child in joint_children:
        assert_true(
            cmds.attributeQuery(child, node=node, exists=True),
            f"missing joint-compound child {child}",
        )

    # Defaults
    grav = cmds.getAttr(f"{node}.gravity")[0]
    assert_eq(round(grav[1], 4), -9.8, "gravity default y (MMD uses exactly -9.8)")

    # Collision mask defaults to "collides with every group" (True each).
    assert_eq(
        cmds.getAttr(f"{node}.bodies[0].bodyMaskGroup0"),
        True,
        "bodyMaskGroup0 default (collides with group 0)",
    )

    # Outputs are not writable.
    assert_true(
        not cmds.attributeQuery("outTranslate", node=node, writable=True),
        "outTranslate should not be writable",
    )

    print("✓ attribute surface complete with correct defaults")
    return True


def test_empty_node_evaluates_without_error():
    """An empty node (no bodies) is a valid no-op — evaluation must not fail."""
    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    # Force several evaluations across frames.  Before the empty-state fix this
    # made compute() return kFailure every frame (buildWorld had nothing to do).
    for frame in (1, 2, 3):
        cmds.currentTime(frame)
        cmds.dgeval(f"{node}.outTranslate")
        cmds.dgeval(f"{node}.outRotate")

    assert_eq(
        cmds.getAttr(f"{node}.outTranslate", size=True),
        0,
        "empty node must produce no output elements",
    )
    print("✓ empty pmxRigidBodyNode evaluates cleanly (valid no-op)")
    return True


def test_dynamic_body_falls_under_gravity():
    """A single dynamic body must fall under the node's gravity (-9.8)."""
    setup_test_environment()
    node = _create_node()
    _set_dynamic_body(node, 0, rest_y=5.0)
    _connect_time(node)

    cmds.currentTime(1)
    initial = _read_output(node, 0)
    assert_eq(round(initial[1], 3), 5.0, "rest pose y before stepping")

    prev_y = initial[1]
    fell = False
    for frame in range(2, 31):
        cmds.currentTime(frame)
        pos = _read_output(node, 0)
        if pos[1] < prev_y:
            fell = True
        prev_y = pos[1]

    assert_true(fell, "body never moved downward under gravity")
    assert_true(
        prev_y < 4.0,
        f"body should have fallen well below rest (final y={prev_y:.3f})",
    )
    print(f"✓ dynamic body fell under gravity to y={prev_y:.3f}")
    return True


def test_kinematic_anchor_drives_welded_body():
    """A kinematic (followBone) body drives a rigidly-welded dynamic body."""
    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    # Body 0: kinematic anchor (followBone) at the origin.
    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)

    # Body 1: dynamic, 1 unit above the anchor.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)

    # Rigid weld: SPRING_6DOF (type 0) with zero limits = locked.
    j = f"{node}.joints[0]"
    cmds.setAttr(f"{j}.jointBodyA", 0)
    cmds.setAttr(f"{j}.jointBodyB", 1)
    cmds.setAttr(f"{j}.jointType", 0)
    cmds.setAttr(f"{j}.jointFrameTranslate", 0.0, 0.5, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointFrameRotate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearSpring", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularSpring", 0.0, 0.0, 0.0, type="double3")

    # Anchor world matrix on the body's OWN compound child: translate the
    # kinematic anchor to y=3 (no bodyJoint message -> the derived offset
    # K^-1 = identity).
    cmds.setAttr(
        f"{node}.bodies[0].bodyAnchorWorld",
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        3,
        0,
        1,
        type="matrix",
    )

    # Step several frames so the weld settles and the chain follows the anchor.
    last_y = 1.0
    for frame in range(1, 31):
        cmds.currentTime(frame)
        last_y = _read_output(node, 1)[1]

    # The welded body rides 1 unit above the anchor (y=3) — allow some settle.
    assert_true(
        last_y > 3.5,
        f"welded body should follow the kinematic anchor up (final y={last_y:.3f})",
    )
    print(f"✓ kinematic anchor drove welded body to y={last_y:.3f}")
    return True


def test_config_edit_forces_rebuild():
    """Editing a body config input rebuilds the Bullet world at the current pose."""
    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    # Two mock joints (the bone builder's DAG is the hierarchy): joint_b is
    # parented under joint_a, so the node walks body 1's joint DAG to find its
    # reset anchor (the kinematic anchor on bone 0).
    cmds.select(clear=True)
    joint_a = cmds.joint(name="cfgAnchorJoint", p=(0, 0, 0))
    cmds.select(clear=True)
    joint_b = cmds.joint(name="cfgDynJoint", p=(0, 1, 0))
    cmds.parent(joint_b, joint_a)
    for j, bone_idx in ((joint_a, 0), (joint_b, 1)):
        cmds.addAttr(j, longName="pmxBoneIndex", attributeType="long", defaultValue=-1)
        cmds.setAttr(f"{j}.pmxBoneIndex", bone_idx)
    # Stamp the bone builder's pmxRest* rest capture so the node's derived K
    # (jointRestWorld * bodyRestWorld^-1) is exact.  joint_b's LOCAL rest
    # under joint_a is (0, 1, 0).
    _stamp_joint_rest(joint_a, 0.0, 0.0, 0.0)
    _stamp_joint_rest(joint_b, 0.0, 1.0, 0.0)

    # Body 0: kinematic anchor (followBone) at the origin, on bone 0.
    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)
    cmds.connectAttr(f"{joint_a}.message", f"{p0}.bodyJoint")

    # Body 1: dynamic, 1 unit above the anchor, on bone 1 (DAG parent = bone
    # 0) — its scrub-back reset anchor (body 0's kinematic anchor) is DERIVED
    # by the node from the joint DAG (bodyJoint messages); it does NOT collide
    # with the anchor's group so it falls freely.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p1}.bodyMaskGroup0", False)  # fall through the anchor
    cmds.connectAttr(f"{joint_b}.message", f"{p1}.bodyJoint")

    # Anchor at the origin (identity) so its current pose is captured for reset.
    cmds.setAttr(f"{node}.bodies[0].bodyAnchorWorld", *_IDENTITY_MATRIX, type="matrix")

    cmds.currentTime(1)
    initial = _read_output(node, 1)
    assert_eq(round(initial[1], 3), 1.0, "rest pose y before stepping")

    # Let the body fall under gravity for several frames.
    for frame in range(2, 13):
        cmds.currentTime(frame)
        _read_output(node, 1)
    fallen = _read_output(node, 1)[1]
    assert_true(
        fallen < 0.85,
        f"body should have fallen by frame 12 (y={fallen:.3f})",
    )

    # At the SAME time, edit a body config input (mass) — the node detects the
    # change, rebuilds, and resets the dynamic body to its current skeleton
    # pose (y=1).
    cmds.setAttr(f"{p1}.bodyMass", 2.0)
    reset_y = _read_output(node, 1)[1]
    assert_true(
        reset_y > 0.9,
        f"config edit should reset the body to rest (y={reset_y:.3f})",
    )
    print(f"✓ config edit rebuild reset body to y={reset_y:.3f}")
    return True


def test_solver_location_does_not_affect_simulation():
    """The Bullet world runs in WORLD space — the solver's own location is irrelevant.

    The node reads only its input attributes, so where the solver node sits in
    the DAG (here under a transform at y=10) never affects the solved poses:
    the welded body follows the world-space anchor (y=3) to y=4 regardless.
    """
    setup_test_environment()
    # Park the solver under a transform that is NOT at the origin.
    holder = cmds.createNode("transform", name="SolverHolder")
    cmds.setAttr(f"{holder}.translateY", 10)
    node = cmds.createNode(_NODE_TYPE, name="testRigidBodyNode", parent=holder)
    _connect_time(node)
    _set_welded_chain(node)

    # World-space anchor at y=3 (there is no groupWorldMatrix input anymore).
    cmds.setAttr(
        f"{node}.bodies[0].bodyAnchorWorld",
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        3,
        0,
        1,
        type="matrix",
    )

    # Step several frames so the weld settles.
    last_y = 1.0
    for frame in range(1, 31):
        cmds.currentTime(frame)
        last_y = _read_output(node, 1)[1]

    assert_true(
        last_y > 3.5,
        f"welded body should follow the WORLD anchor regardless of solver "
        f"location (y={last_y:.3f})",
    )
    print(
        f"✓ solver location ignored; welded body followed world anchor to y={last_y:.3f}"
    )
    return True


def test_whole_skeleton_move_rides_dynamic_chain_along():
    """Moving the whole character at a paused frame rides the chains along
    WITHOUT running physics.

    Regression: dragging the character (all kinematic anchors share one
    world-space move) used to teleport the anchors and then step once — the
    dynamic chains did not ride along, so the write-back displaced them by the
    move (skirt/hair offset from the skeleton even at frame 0, nothing
    playing).  A whole-skeleton rigid move must now ride the dynamics along by
    the same transform instead of stepping.
    """
    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    # One bone-attached kinematic anchor (joint_a at the origin) + a welded
    # dynamic body 1 unit above it.  Body 1 has no joint -> its output is the
    # raw solved world pose, so the test reads the ride-along directly.
    cmds.select(clear=True)
    joint_a = cmds.joint(name="dragAnchorJoint", p=(0, 0, 0))
    cmds.addAttr(
        joint_a, longName="pmxBoneIndex", attributeType="long", defaultValue=-1
    )
    cmds.setAttr(f"{joint_a}.pmxBoneIndex", 0)
    _stamp_joint_rest(joint_a, 0.0, 0.0, 0.0)

    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)
    cmds.connectAttr(f"{joint_a}.message", f"{p0}.bodyJoint")
    cmds.connectAttr(f"{joint_a}.worldMatrix[0]", f"{node}.bodies[0].bodyAnchorWorld")

    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p1}.bodyMaskGroup0", False)  # fall through the anchor

    j = f"{node}.joints[0]"
    cmds.setAttr(f"{j}.jointBodyA", 0)
    cmds.setAttr(f"{j}.jointBodyB", 1)
    cmds.setAttr(f"{j}.jointType", 0)
    cmds.setAttr(f"{j}.jointFrameTranslate", 0.0, 0.5, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointFrameRotate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearSpring", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularSpring", 0.0, 0.0, 0.0, type="double3")

    # Settle at frame 1: the welded body hangs 1 unit above the anchor.
    cmds.currentTime(1)
    settled = _read_output(node, 1)
    assert_true(abs(settled[1] - 1.0) < 0.05, f"settled at y≈1 (got {settled[1]:.3f})")

    # At the SAME frame (no time advance), drag the character up by 5.
    cmds.setAttr(f"{joint_a}.translateY", 5)
    after = _read_output(node, 1)

    # The welded body rode along to y≈6 (1 above the moved anchor).  A single
    # physics tick could NOT have carried it there — a step would leave it
    # displaced near y≈1.
    assert_true(
        abs(after[1] - 6.0) < 0.1,
        f"dynamic body should ride along to y≈6 (got {after[1]:.3f})",
    )
    print(f"✓ whole-skeleton drag rode dynamic chain to y={after[1]:.3f}")
    return True


def test_rewind_rebuild_keeps_character_move():
    """Rewinding after moving the whole character keeps the ride-along — the
    rebuild must NOT snap the chains back to the un-moved position.

    Regression: the user moved GirlsFrontline_TololoDefault_Bones by -15 and
    every rewind + replay showed a large persistent jump ("big jump that do
    not go away every step back in time").  Root cause: on a scrub-back the
    world is REBUILT, and the rebuild used the engine's K-conjugated
    resetDynamicBodies (anchorCurrent * (anchorRest^-1 * bodyRest)), which for
    a moved character lands the chains at K^-1·M·K·bodyRest — a rotated
    (wrong) position.  The rebuild must instead detect the whole-skeleton move
    against the ORIGINAL import-time anchors (persisted across rebuilds) and
    ride the chains from their rest pose by that move.
    """
    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    cmds.select(clear=True)
    joint_a = cmds.joint(name="rewindAnchorJoint", p=(0, 0, 0))
    cmds.addAttr(
        joint_a, longName="pmxBoneIndex", attributeType="long", defaultValue=-1
    )
    cmds.setAttr(f"{joint_a}.pmxBoneIndex", 0)
    _stamp_joint_rest(joint_a, 0.0, 0.0, 0.0)

    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)
    cmds.connectAttr(f"{joint_a}.message", f"{p0}.bodyJoint")
    cmds.connectAttr(f"{joint_a}.worldMatrix[0]", f"{node}.bodies[0].bodyAnchorWorld")

    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p1}.bodyMaskGroup0", False)  # fall through the anchor

    j = f"{node}.joints[0]"
    cmds.setAttr(f"{j}.jointBodyA", 0)
    cmds.setAttr(f"{j}.jointBodyB", 1)
    cmds.setAttr(f"{j}.jointType", 0)
    cmds.setAttr(f"{j}.jointFrameTranslate", 0.0, 0.5, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointFrameRotate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearSpring", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularSpring", 0.0, 0.0, 0.0, type="double3")

    # Settle at frame 1: the welded body hangs 1 unit above the anchor.
    cmds.currentTime(1)
    settled = _read_output(node, 1)
    assert_true(abs(settled[1] - 1.0) < 0.05, f"settled at y≈1 (got {settled[1]:.3f})")

    # Move the character up by 5 (first build captured the ORIGINAL anchors,
    # so the detector has a rest reference for the move).
    cmds.setAttr(f"{joint_a}.translateY", 5)
    played = _read_output(node, 1)
    assert_true(
        abs(played[1] - 6.0) < 0.1,
        f"ride-along at frame 1 (got {played[1]:.3f})",
    )

    # Rewind to frame 0 (scrub-back -> rebuild).  The rebuild must preserve
    # the character move: the welded body stays at y≈6, NOT snapping back to
    # y≈1 (the un-moved rest position the K-conjugated reset would produce).
    # (_read_output(node, 1) reads BODY 1 — the dynamic body — at the CURRENT
    # time, which the currentTime(0) above has already set.)
    cmds.currentTime(0)
    rewind = _read_output(node, 1)
    assert_true(
        abs(rewind[1] - 6.0) < 0.1,
        f"rewind rebuild must keep the move (got {rewind[1]:.3f})",
    )

    # Replay to frame 2: still riding at y≈6 — no persistent jump.
    cmds.currentTime(2)
    replay = _read_output(node, 1)
    assert_true(
        abs(replay[1] - 6.0) < 0.15,
        f"replay after rewind stays at y≈6 (got {replay[1]:.3f})",
    )
    print(
        f"✓ rewind rebuild kept the move: y={rewind[1]:.3f} (frame 0), "
        f"y={replay[1]:.3f} (frame 2)"
    )
    return True


# ══════════════════════════════════════════════════════════════════════════
# Test Registry (static — consumed by run_all_integration_tests.py)
# ══════════════════════════════════════════════════════════════════════════

_TESTS = [
    ("Node Registration & Creation", test_node_registration_and_creation),
    ("Attribute Surface & Defaults", test_attribute_surface_and_defaults),
    ("Empty Node Evaluates Cleanly", test_empty_node_evaluates_without_error),
    ("Dynamic Body Falls Under Gravity", test_dynamic_body_falls_under_gravity),
    ("Kinematic Anchor Drives Welded Body", test_kinematic_anchor_drives_welded_body),
    ("Config Edit Forces Rebuild", test_config_edit_forces_rebuild),
    (
        "Solver Location Irrelevant (World Space)",
        test_solver_location_does_not_affect_simulation,
    ),
    (
        "Whole-Skeleton Drag Rides Dynamic Chain Along (no physics step)",
        test_whole_skeleton_move_rides_dynamic_chain_along,
    ),
    (
        "Rewind Rebuild Keeps Character Move (no persistent jump)",
        test_rewind_rebuild_keeps_character_move,
    ),
]
