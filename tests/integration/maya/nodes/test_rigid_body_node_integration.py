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


def test_rewind_rebuild_pins_posed_skeleton():
    """A POSED (animated) skeleton must pin identically across a rewind
    rebuild — the rebuild must NOT snap the chains to rest nor bake the pose
    into the reset offset.

    Regression: with a VMD motion applied, the skeleton was POSED BEFORE the
    first solver evaluation.  The first build then used the posed anchors as
    its "rest" reference, so on first play the bodies sat at REST while the
    skeleton was posed (the 51° mismatch); a rewind rebuild read the anchors
    fresh, so rewind frame 1 ≠ first-play frame 1 — the persistent rewind
    jump.  The reset reference must be the joints' REST worlds (model
    constants), so first build and rewind rebuild pin identically to the
    posed skeleton, and both are move-invariant.
    """
    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    cmds.select(clear=True)
    joint_a = cmds.joint(name="posedAnchorJoint", p=(0, 0, 0))
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

    # Pose the skeleton BEFORE the first evaluation — as a VMD motion would
    # (rotate the anchor 30° about Z + move it up by 5).  The FIRST build must
    # see this posed skeleton, not an un-posed one.
    cmds.currentTime(1)
    cmds.setAttr(f"{joint_a}.rotate", 0.0, 0.0, 30.0)
    cmds.setAttr(f"{joint_a}.translateY", 5)
    cmds.dgdirty(joint_a)

    # First play: the welded body hangs 1 unit above the posed anchor, in the
    # anchor's rotated frame: Rz(30°)·(0,1,0) + (0,5,0) = (-0.5, 5.866, 0).
    # Under the old (posed-as-rest) reference it would sit at the un-posed
    # (0, 6, 0) — the 51°-class mismatch.
    first = _read_output(node, 1)
    assert_true(
        abs(first[0] - (-0.5)) < 0.1 and abs(first[1] - 5.866) < 0.1,
        f"first play pins to the posed skeleton (got ({first[0]:.3f}, "
        f"{first[1]:.3f}, {first[2]:.3f}), expected ≈(-0.5, 5.866, 0))",
    )

    # Rewind to frame 0 (scrub-back -> rebuild).  The rebuild must pin to the
    # SAME posed skeleton — identical to first play, NOT snapping back to rest
    # (0,1,0) or the un-posed (0,6,0).
    cmds.currentTime(0)
    rewind = _read_output(node, 1)
    assert_true(
        abs(rewind[0] - first[0]) < 0.05 and abs(rewind[1] - first[1]) < 0.05,
        f"rewind rebuild pins identically to first play (got "
        f"({rewind[0]:.3f}, {rewind[1]:.3f}, {rewind[2]:.3f}), first was "
        f"({first[0]:.3f}, {first[1]:.3f}, {first[2]:.3f}))",
    )

    # Replay to frame 2: still pinned to the posed skeleton — no jump.
    cmds.currentTime(2)
    replay = _read_output(node, 1)
    assert_true(
        abs(replay[0] - first[0]) < 0.15 and abs(replay[1] - first[1]) < 0.15,
        f"replay after rewind stays pinned to the posed skeleton (got "
        f"({replay[0]:.3f}, {replay[1]:.3f}, {replay[2]:.3f}))",
    )
    print(
        f"✓ rewind rebuild pinned to the posed skeleton: "
        f"({first[0]:.3f}, {first[1]:.3f}) first, "
        f"({rewind[0]:.3f}, {rewind[1]:.3f}) rewind, "
        f"({replay[0]:.3f}, {replay[1]:.3f}) replay"
    )
    return True


def test_parentless_body_write_back_survives_rotated_parent_chain():
    """A dynamic body whose PARENT bone has no body must land at the correct
    pose even when the gap bones between it and the solver-known ancestor are
    ROTATED (a posed / animated skeleton).

    Regression: the write-back fallback reconstructs the parent joint's world
    by composing the gap bones' LOCAL matrices on the nearest solver-known
    ancestor.  The locals are stored as btTransforms (TRANSPOSED Maya matrices,
    column-vector), so each must be POST-multiplied (parentWorld = parentWorld
    * local).  The old pre-multiply (local * parentWorld) was exact at rest —
    translation-only locals commute — but once the chain rotates it computes
    world(parent)*local instead of local*world(parent) and launched the
    parentless bones meters away during animation (Endmin shengzi_0_skin_jnt).
    """
    import maya.api.OpenMaya as om  # noqa: PLC0415

    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    # Chain: joint_a (bone 0, kinematic anchor) -> joint_b (bone 1, NO body)
    # -> joint_c (bone 2, dynamic body 1).  joint_b is the "gap" bone with no
    # body, so body 1's write-back must reconstruct joint_b's world.
    cmds.select(clear=True)
    joint_a = cmds.joint(name="reconAnchorJoint", p=(0, 0, 0))
    cmds.select(clear=True)
    joint_b = cmds.joint(name="reconGapJoint", p=(0, 1, 0))
    cmds.select(clear=True)
    joint_c = cmds.joint(name="reconDynJoint", p=(0, 1, 0))
    cmds.parent(joint_b, joint_a)
    cmds.parent(joint_c, joint_b)
    for j, bone_idx in ((joint_a, 0), (joint_b, 1), (joint_c, 2)):
        cmds.addAttr(j, longName="pmxBoneIndex", attributeType="long", defaultValue=-1)
        cmds.setAttr(f"{j}.pmxBoneIndex", bone_idx)
    # Local rest offsets: b is 1 above a, c is 1 above b => c rests at (0,2,0)
    # in a's frame.  Stamped so the node's derived K is exact.
    _stamp_joint_rest(joint_a, 0.0, 0.0, 0.0)
    _stamp_joint_rest(joint_b, 0.0, 1.0, 0.0)
    _stamp_joint_rest(joint_c, 0.0, 1.0, 0.0)

    # Body 0: kinematic anchor (followBone) on bone 0.
    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)
    cmds.connectAttr(f"{joint_a}.message", f"{p0}.bodyJoint")

    # Body 1: dynamic on bone 2 (joint_c) — its parent bone 1 (joint_b) has
    # NO body, so the write-back fallback reconstructs joint_b's world from
    # joint_a's solved world + the gap local.  Rest 2 above the anchor.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 2.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p1}.bodyMaskGroup0", False)  # fall through the anchor
    cmds.connectAttr(f"{joint_c}.message", f"{p1}.bodyJoint")

    # Rigid weld between the anchor and the dynamic body.
    j = f"{node}.joints[0]"
    cmds.setAttr(f"{j}.jointBodyA", 0)
    cmds.setAttr(f"{j}.jointBodyB", 1)
    cmds.setAttr(f"{j}.jointType", 0)
    cmds.setAttr(f"{j}.jointFrameTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointFrameRotate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearSpring", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularSpring", 0.0, 0.0, 0.0, type="double3")

    # Pose the skeleton BEFORE the first evaluation: rotate the anchor 30° Z
    # and move it up by 5 — the gap bone joint_b rotates WITH the chain.
    cmds.currentTime(1)
    cmds.setAttr(f"{joint_a}.rotate", 0.0, 0.0, 30.0)
    cmds.setAttr(f"{joint_a}.translateY", 5)
    cmds.dgdirty(joint_a)

    # Build and read the dynamic body's write-back local pose.
    out_t = _read_output(node, 1)
    out_r = tuple(cmds.getAttr(f"{node}.outRotate[1]")[0])

    # Expected: the body's solved WORLD pose (2 units above the posed anchor,
    # rotated 30° Z with the chain) expressed as joint_c's LOCAL pose relative
    # to joint_b's ACTUAL world.  Reconstruct that world and verify the body
    # actually landed at the expected world position.
    #
    # Expected world = body rest offset rigidly attached to the posed anchor:
    # Rz(30°)·(0,2,0) + (0,5,0).  Compute via Maya matrices so the convention
    # is exactly Maya's.
    def _world_matrix(name: str) -> om.MMatrix:
        raw = cmds.getAttr(f"{name}.worldMatrix[0]")
        # getAttr returns a list containing one 16-float list, or the flat
        # list directly depending on context — normalize to 16 floats.
        if isinstance(raw, (list, tuple)) and raw and isinstance(raw[0], (list, tuple)):
            raw = raw[0]
        return om.MMatrix(raw)

    anchor_world = _world_matrix(joint_a)
    body_rest = om.MTransformationMatrix()
    body_rest.setTranslation(om.MVector(0, 2, 0), om.MSpace.kTransform)
    expected_world = body_rest.asMatrix() * anchor_world  # row-vector
    expected = om.MTransformationMatrix(expected_world).translation(
        om.MSpace.kTransform
    )

    # Reconstruct the body's ACTUAL world from the write-back: the body's
    # local pose (outT/outR) composed with joint_b's real world.
    local = om.MTransformationMatrix()
    local.setTranslation(om.MVector(*out_t), om.MSpace.kTransform)
    import math  # noqa: PLC0415

    local.setRotation(
        om.MEulerRotation(
            math.radians(out_r[0]), math.radians(out_r[1]), math.radians(out_r[2])
        )
    )
    joint_b_world = _world_matrix(joint_b)
    actual_world = local.asMatrix() * joint_b_world
    actual = om.MTransformationMatrix(actual_world).translation(om.MSpace.kTransform)

    assert_true(
        abs(actual.x - expected.x) < 0.2
        and abs(actual.y - expected.y) < 0.2
        and abs(actual.z - expected.z) < 0.2,
        f"parentless body must land at the posed world pose (got "
        f"({actual.x:.3f}, {actual.y:.3f}, {actual.z:.3f}), expected "
        f"({expected.x:.3f}, {expected.y:.3f}, {expected.z:.3f}); the "
        f"pre-multiply bug launched it meters away under chain rotation)",
    )
    print(
        f"✓ parentless-body write-back survived a rotated parent chain: "
        f"landed at ({actual.x:.3f}, {actual.y:.3f}, {actual.z:.3f}) vs expected "
        f"({expected.x:.3f}, {expected.y:.3f}, {expected.z:.3f})"
    )
    return True


def test_rewind_rebuild_pins_body_with_later_reset_anchor():
    """A dynamic body whose reset-anchor kinematic body appears LATER in body
    order must re-pin to the CURRENT skeleton pose on a rewind rebuild.

    Regression: Endmin's skirt bones are anchored to a bone whose kinematic
    body FOLLOWS them in body order.  The reset offset was captured inside the
    body-creation loop, where mAnchorRest only held the kinematic bodies seen
    SO FAR — so such a body silently failed the resetAnchorIndex <
    mAnchorRest.size() check, got NO reset, and sat at the freshly-built
    world's rest pose (the origin) on every rewind after the character was
    moved, until the next forward step re-dragged it.  (The existing rewind
    tests put the anchor FIRST, so they never exercised this ordering.)
    """
    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    # DAG: joint_child (bone 1) is a CHILD of joint_anchor (bone 0).  Body 0
    # (dynamic) drives joint_child; its reset anchor is bone 0 = body 1 (the
    # kinematic body), which is registered LATER in body order.
    cmds.select(clear=True)
    joint_anchor = cmds.joint(name="laterAnchorJoint", p=(0, 0, 0))
    cmds.addAttr(
        joint_anchor, longName="pmxBoneIndex", attributeType="long", defaultValue=-1
    )
    cmds.setAttr(f"{joint_anchor}.pmxBoneIndex", 0)
    _stamp_joint_rest(joint_anchor, 0.0, 0.0, 0.0)
    cmds.select(clear=True)
    joint_child = cmds.joint(name="laterAnchorChildJoint", p=(0, 1, 0))
    cmds.addAttr(
        joint_child, longName="pmxBoneIndex", attributeType="long", defaultValue=-1
    )
    cmds.setAttr(f"{joint_child}.pmxBoneIndex", 1)
    _stamp_joint_rest(joint_child, 0.0, 1.0, 0.0)
    cmds.parent(joint_child, joint_anchor)

    # Body 0: DYNAMIC on bone 1 (joint_child) — appears BEFORE its anchor body.
    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyMass", 1.0)
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p0}.bodyMaskGroup0", False)  # fall through the anchor
    cmds.connectAttr(f"{joint_child}.message", f"{p0}.bodyJoint")

    # Body 1: KINEMATIC anchor on bone 0 (joint_anchor) — LATER in body order.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)
    cmds.connectAttr(f"{joint_anchor}.message", f"{p1}.bodyJoint")
    cmds.connectAttr(
        f"{joint_anchor}.worldMatrix[0]", f"{node}.bodies[1].bodyAnchorWorld"
    )

    # Rigid weld between the anchor and the dynamic body.
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

    # Settle at frame 1: the welded dynamic body hangs 1 unit above the anchor
    # -> joint_child's world is at y≈1.
    cmds.currentTime(1)
    cmds.dgeval(f"{node}.outTranslate")
    settled = tuple(
        cmds.xform(joint_child, query=True, worldSpace=True, translation=True)
    )
    assert_true(
        abs(settled[1] - 1.0) < 0.05,
        f"settled at y≈1 (got {settled[1]:.3f})",
    )

    # Move the whole character up by 5 -> the dynamic body rides along.
    cmds.setAttr(f"{joint_anchor}.translateY", 5)
    cmds.dgeval(f"{node}.outTranslate")
    moved = tuple(
        cmds.xform(joint_child, query=True, worldSpace=True, translation=True)
    )
    assert_true(
        abs(moved[1] - 6.0) < 0.1,
        f"ride-along at y≈6 (got {moved[1]:.3f})",
    )

    # Rewind to frame 0 (scrub-back -> rebuild).  The rebuild must re-pin the
    # dynamic body to the CURRENT skeleton pose (y≈6), NOT leave it at the
    # fresh-world rest pose (y≈1, the origin-drop regression).
    cmds.currentTime(0)
    cmds.dgeval(f"{node}.outTranslate")
    rewind = tuple(
        cmds.xform(joint_child, query=True, worldSpace=True, translation=True)
    )
    assert_true(
        abs(rewind[1] - 6.0) < 0.1,
        f"rewind rebuild must keep the body at the moved skeleton (got "
        f"y={rewind[1]:.3f}, expected y≈6)",
    )
    print(
        f"✓ rewind rebuild pinned a later-registered-anchor body: "
        f"y={settled[1]:.3f} settled, y={moved[1]:.3f} moved, "
        f"y={rewind[1]:.3f} rewind"
    )
    return True


def test_disabled_kinematic_body_does_not_shift_anchor_write_back():
    """A DISABLED kinematic body must not consume a kinematic-order slot in
    the write-back, so the anchors after it still read the correct raw world.

    Regression: writeOutputs pass 1 indexed lastAnchorWorld with a kinematic
    counter that incremented for disabled kinematic bodies too — but
    readRawAnchorWorlds only records ENABLED kinematic anchors.  A disabled
    kinematic body before an enabled one shifted every subsequent anchor read,
    so the enabled kinematic body's bone world was read from the wrong slot
    (or skipped) and a dynamic body driven on a child bone (whose write-back
    needs the anchor bone's solved world as its parent) landed at the wrong
    local pose.
    """
    setup_test_environment()
    node = _create_node()
    _connect_time(node)

    # DAG: joint_b (bone 1) is a CHILD of joint_a (bone 0).  Body 2 (dynamic)
    # drives joint_b; its write-back is relative to bone 0's solved world,
    # which comes from the kinematic anchor's kinematic-order slot.
    cmds.select(clear=True)
    joint_a = cmds.joint(name="disabledKinAnchorJoint", p=(0, 0, 0))
    cmds.addAttr(
        joint_a, longName="pmxBoneIndex", attributeType="long", defaultValue=-1
    )
    cmds.setAttr(f"{joint_a}.pmxBoneIndex", 0)
    _stamp_joint_rest(joint_a, 0.0, 0.0, 0.0)
    cmds.select(clear=True)
    joint_b = cmds.joint(name="disabledKinChildJoint", p=(0, 1, 0))
    cmds.addAttr(
        joint_b, longName="pmxBoneIndex", attributeType="long", defaultValue=-1
    )
    cmds.setAttr(f"{joint_b}.pmxBoneIndex", 1)
    _stamp_joint_rest(joint_b, 0.0, 1.0, 0.0)
    cmds.parent(joint_b, joint_a)

    # Body 0: DISABLED kinematic anchor — must NOT occupy a slot.
    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyEnabled", False)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)
    cmds.connectAttr(f"{joint_a}.message", f"{p0}.bodyJoint")

    # Body 1: ENABLED kinematic anchor on bone 0 — the FIRST enabled kinematic
    # body, so it must be kinematic-order slot 0.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)
    cmds.connectAttr(f"{joint_a}.message", f"{p1}.bodyJoint")
    cmds.connectAttr(f"{joint_a}.worldMatrix[0]", f"{node}.bodies[1].bodyAnchorWorld")

    # Body 2: dynamic on bone 1 (joint_b), welded to the anchor, 1 unit above.
    p2 = _set_body_common(node, 2)
    cmds.setAttr(f"{p2}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p2}.bodyMass", 1.0)
    cmds.setAttr(f"{p2}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p2}.bodyMaskGroup0", False)  # fall through the anchor
    cmds.connectAttr(f"{joint_b}.message", f"{p2}.bodyJoint")
    # The import command wires the solver output straight into the joint, so
    # the write-back (pass 2) is what moves joint_b — its LOCAL pose is
    # expressed relative to bone 0's solved world.
    cmds.connectAttr(f"{node}.outTranslate[2]", f"{joint_b}.translate")
    cmds.connectAttr(f"{node}.outRotate[2]", f"{joint_b}.rotate")

    j = f"{node}.joints[0]"
    cmds.setAttr(f"{j}.jointBodyA", 1)
    cmds.setAttr(f"{j}.jointBodyB", 2)
    cmds.setAttr(f"{j}.jointType", 0)
    cmds.setAttr(f"{j}.jointFrameTranslate", 0.0, 0.5, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointFrameRotate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMin", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularMax", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointLinearSpring", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{j}.jointAngularSpring", 0.0, 0.0, 0.0, type="double3")

    # Move the anchor up by 5 and settle: the welded dynamic body rides to
    # y≈6 in WORLD space, so joint_b's LOCAL write-back (relative to bone 0's
    # solved world at y=5) is (0,1,0) and its world is y≈6.  If the disabled
    # kinematic body had shifted the anchor slot, body 1's bone world would be
    # missing and the write-back would write the raw WORLD pose as the local
    # one -> joint_b would land at y≈11.
    cmds.currentTime(1)
    cmds.setAttr(f"{joint_a}.translateY", 5)
    cmds.dgeval(f"{node}.outTranslate")
    after = tuple(cmds.xform(joint_b, query=True, worldSpace=True, translation=True))
    assert_true(
        abs(after[1] - 6.0) < 0.1,
        f"joint_b world must ride to y≈6 past a disabled kinematic body "
        f"(got y={after[1]:.3f}, expected y≈6; a shifted anchor slot would "
        f"land it at y≈11)",
    )
    print(
        f"✓ disabled kinematic body did not shift the write-back: "
        f"joint_b y={after[1]:.3f}"
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
    (
        "Rewind Rebuild Pins Posed (Animated) Skeleton",
        test_rewind_rebuild_pins_posed_skeleton,
    ),
    (
        "Parentless-Body Write-Back Survives Rotated Parent Chain",
        test_parentless_body_write_back_survives_rotated_parent_chain,
    ),
    (
        "Rewind Rebuild Pins Body With Later-Registered Reset Anchor",
        test_rewind_rebuild_pins_body_with_later_reset_anchor,
    ),
    (
        "Disabled Kinematic Body Does Not Shift Anchor Write-Back",
        test_disabled_kinematic_body_does_not_shift_anchor_write_back,
    ),
]
