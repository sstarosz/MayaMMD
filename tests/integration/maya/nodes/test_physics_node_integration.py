"""
test_physics_node_integration.py

Integration tests for PhysicsNode (pmxPhysicsNode) — the native rigid-body
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


_NODE_TYPE = "pmxPhysicsNode"

# Attribute-enum values (PhysicsNode.h): collider type + physics mode.
_COLLIDER_SPHERE = 2  # kColliderSphere
_PHYSICS_MODE_FOLLOW_BONE = 0
_PHYSICS_MODE_PHYSICS = 1

# Maya row-vector convention: translation lives in the LAST row (m30..m32).
_IDENTITY_MATRIX = (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)


def _create_node(name: str = "testPhysicsNode") -> str:
    """Create a pmxPhysicsNode and assert it exists."""
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
    cmds.setAttr(f"{p}.bodyParentBodyIndex", -1)
    cmds.setAttr(f"{p}.bodyResetAnchorIndex", -1)


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
    cmds.setAttr(f"{p0}.bodyParentBodyIndex", -1)
    cmds.setAttr(f"{p0}.bodyResetAnchorIndex", -1)

    # Body 1: dynamic, 1 unit above the anchor.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p1}.bodyParentBodyIndex", -1)
    cmds.setAttr(f"{p1}.bodyResetAnchorIndex", -1)

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
        "anchorWorldMatrix",
        "bodyWriteBackOffset",
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
        "bodyParentBodyIndex",
        "bodyResetAnchorIndex",
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
    print("✓ empty pmxPhysicsNode evaluates cleanly (valid no-op)")
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
    cmds.setAttr(f"{p0}.bodyParentBodyIndex", -1)
    cmds.setAttr(f"{p0}.bodyResetAnchorIndex", -1)

    # Body 1: dynamic, 1 unit above the anchor.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p1}.bodyParentBodyIndex", -1)
    cmds.setAttr(f"{p1}.bodyResetAnchorIndex", -1)

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

    # Anchor world matrix: translate the kinematic anchor to y=3 (group at
    # the origin, so the derived group-inverse = identity and the derived
    # offset (K^-1 = bodyWriteBackOffset[0]^-1) = identity).
    cmds.setAttr(
        f"{node}.anchorWorldMatrix[0]",
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

    # Body 0: kinematic anchor (followBone) at the origin.
    p0 = _set_body_common(node, 0)
    cmds.setAttr(f"{p0}.bodyRestTranslate", 0.0, 0.0, 0.0, type="double3")
    cmds.setAttr(f"{p0}.bodyPhysicsMode", _PHYSICS_MODE_FOLLOW_BONE)
    cmds.setAttr(f"{p0}.bodyParentBodyIndex", -1)
    cmds.setAttr(f"{p0}.bodyResetAnchorIndex", -1)

    # Body 1: dynamic, 1 unit above the anchor, reset anchor = body 0, and it
    # does NOT collide with the anchor's group so it falls freely.
    p1 = _set_body_common(node, 1)
    cmds.setAttr(f"{p1}.bodyRestTranslate", 0.0, 1.0, 0.0, type="double3")
    cmds.setAttr(f"{p1}.bodyMass", 1.0)
    cmds.setAttr(f"{p1}.bodyPhysicsMode", _PHYSICS_MODE_PHYSICS)
    cmds.setAttr(f"{p1}.bodyParentBodyIndex", -1)
    cmds.setAttr(f"{p1}.bodyResetAnchorIndex", 0)
    cmds.setAttr(f"{p1}.bodyMaskGroup0", False)  # fall through the anchor

    # Anchor at the origin (identity) so its current pose is captured for reset.
    cmds.setAttr(f"{node}.anchorWorldMatrix[0]", *_IDENTITY_MATRIX, type="matrix")

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
    node = cmds.createNode(_NODE_TYPE, name="testPhysicsNode", parent=holder)
    _connect_time(node)
    _set_welded_chain(node)

    # World-space anchor at y=3 (there is no groupWorldMatrix input anymore).
    cmds.setAttr(
        f"{node}.anchorWorldMatrix[0]",
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
    print(f"✓ solver location ignored; welded body followed world anchor to y={last_y:.3f}")
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
]
