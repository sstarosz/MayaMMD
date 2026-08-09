"""
test_mmd_rigid_body_cmd_integration.py

Integration tests for the native C++ ``mmdRigidBody`` / ``mmdRigidBodyConstraint``
commands — pure Maya command testing WITHOUT any PMX model.  The scene is a
small mock: a transform group holding one ``mmdPhysicsNode`` solver plus a
couple of bare joints.

Tests cover (the "add a body / joint and configure it on addition" contract):

- Auto-append body index (0, 1, 2 ...) and stored body DATA (names, mass,
  damping, friction, restitution, group, mask, shape, physics mode, rest pose).
- FOLLOW_BONE + bone -> kinematic anchor binding (anchorWorldMatrix fed by the
  joint, anchorParentInverseMatrix fed by the group, baked anchorOffset).
- PHYSICS / PHYSICS_BONE -> DATA ONLY (no anchor, no write-back connections).
- FOLLOW_BONE without a bone -> a static collider pinned at its rest pose.
- clamp01 on damping/friction/restitution.
- Invalid shape / physicsMode values are rejected.
- ``mmdRigidBodyConstraint`` appends ONE joint (bodyA/B, type, frame, limits,
  springs) with the MMD -> Maya conversions.
- Solver resolution through a model-root ``pmxPhysicsNode`` attribute.
- Enum attributes: getAttr returns the numeric field value; the enum field
  list is exposed via attributeQuery(listEnum=True).

NOTE: MayaMMD.mll (which registers mmdPhysicsNode + both commands) is already
loaded by the test runner — no separate plugin loading here.
"""

# ── Maya standalone initialised by the test runner ───────────────────────

# Maya imports (safe after standalone.initialize())
from maya import cmds

# Test framework imports
from tests.integration.test_helpers import (
    approx_equal_tuple,
    assert_eq,
    assert_true,
    setup_test_environment,
)

# PMX collider type field values (MMDPhysicsNode::ColliderType).
COLLIDER_BOX = 1
COLLIDER_SPHERE = 2
COLLIDER_CAPSULE = 3
# BodyPhysicsMode field values (mmd_physics_node.h).
MODE_FOLLOW_BONE = 0
MODE_PHYSICS = 1
MODE_PHYSICS_BONE = 2


def _make_physics_scene():
    """Create a mock physics group + solver node + two bare joints.

    Returns ``(group, solver, joint_a, joint_b)`` (plain names, no PMX).
    """
    setup_test_environment()
    group = cmds.createNode("transform", name="testPhysicsGroup")
    solver = cmds.createNode("mmdPhysicsNode", name="testSolver", parent=group)
    cmds.select(clear=True)
    joint_a = cmds.joint(name="testJointA", p=[0, 0, 0])
    cmds.select(clear=True)
    joint_b = cmds.joint(name="testJointB", p=[1, 0, 0])
    return group, solver, joint_a, joint_b


# ──────────────────────────────────────────────────────────────────────────
# Test Cases
# ──────────────────────────────────────────────────────────────────────────


def test_append_body_data():
    """Appending a body stores the full body DATA at the next free index."""
    _group, solver, _ja, _jb = _make_physics_scene()

    idx = cmds.mmdRigidBody(
        solver,
        name="BodyA",
        nameUniversal="BodyA_univ",
        shape="box",
        size=(1.0, 2.0, 3.0),
        position=(0.5, 0.0, 0.0),
        mass=2.5,
        linearDamping=0.5,
        angularDamping=0.3,
        friction=0.7,
        restitution=0.1,
        group=3,
        nonCollisionGroup=0xFFFE,
        physicsMode="followBone",
    )
    assert_eq(idx, 0, f"first body index != 0 ({idx})")
    assert_eq(int(cmds.getAttr(f"{solver}.bodies", size=True)), 1, "bodies count != 1")

    base = f"{solver}.bodies[0]"
    assert_eq(cmds.getAttr(f"{base}.bodyNameLocal"), "BodyA", "bodyNameLocal")
    assert_eq(
        cmds.getAttr(f"{base}.bodyNameUniversal"), "BodyA_univ", "bodyNameUniversal"
    )
    assert_eq(float(cmds.getAttr(f"{base}.bodyMass")), 2.5, "bodyMass")
    assert_eq(
        float(cmds.getAttr(f"{base}.bodyLinearDamping")), 0.5, "bodyLinearDamping"
    )
    assert_eq(
        float(cmds.getAttr(f"{base}.bodyAngularDamping")), 0.3, "bodyAngularDamping"
    )
    assert_eq(float(cmds.getAttr(f"{base}.bodyFriction")), 0.7, "bodyFriction")
    assert_eq(float(cmds.getAttr(f"{base}.bodyRestitution")), 0.1, "bodyRestitution")
    assert_eq(int(cmds.getAttr(f"{base}.bodyGroupId")), 3, "bodyGroupId")
    assert_eq(
        int(cmds.getAttr(f"{base}.bodyNonCollisionGroup")), 0xFFFE, "nonCollisionGroup"
    )
    assert_eq(
        int(cmds.getAttr(f"{base}.bodyColliderType")),
        COLLIDER_BOX,
        "colliderType (box)",
    )
    assert_eq(
        int(cmds.getAttr(f"{base}.bodyPhysicsMode")), MODE_FOLLOW_BONE, "physicsMode"
    )
    # Box: radius = size.x, extents = size, length = size.y.  (float3 compound
    # children come back from getAttr as [(x, y, z)].)
    assert_eq(float(cmds.getAttr(f"{base}.bodyRadius")), 1.0, "bodyRadius (size.x)")
    assert_true(
        approx_equal_tuple(cmds.getAttr(f"{base}.bodyExtents")[0], (1.0, 2.0, 3.0)),
        f"bodyExtents != size {cmds.getAttr(f'{base}.bodyExtents')}",
    )
    assert_eq(float(cmds.getAttr(f"{base}.bodyLength")), 2.0, "bodyLength (size.y)")
    # Rest pose in group space: group is at the origin, so the MMD position
    # (Z-flip of 0 == 0) lands directly in group space.
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.bodyRestTranslate")[0], (0.5, 0.0, 0.0)
        ),
        f"bodyRestTranslate != (0.5, 0, 0) {cmds.getAttr(f'{base}.bodyRestTranslate')}",
    )
    print("✓ append body stores data at index 0")
    return True


def test_follow_bone_binds_anchor():
    """FOLLOW_BONE + bone: the kinematic anchor is fed by the related joint."""
    group, solver, joint_a, _jb = _make_physics_scene()

    idx = cmds.mmdRigidBody(
        solver,
        name="KinBody",
        bone=joint_a,
        shape="sphere",
        size=(0.4, 0.4, 0.4),
        position=(0.0, 5.0, 0.0),
        mass=1.0,
        physicsMode="followBone",
    )
    assert_eq(idx, 0, "index != 0")

    # Anchor world matrix fed by the joint's worldMatrix[0] (listConnections
    # returns the source NODE name).
    srcs = cmds.listConnections(f"{solver}.anchorWorldMatrix[0]", source=True) or []
    assert_true(
        any(joint_a in src for src in srcs),
        f"anchorWorldMatrix[0] not fed by joint ({srcs})",
    )
    # Parent inverse fed by the physics group's worldInverseMatrix[0].
    psrcs = (
        cmds.listConnections(f"{solver}.anchorParentInverseMatrix[0]", source=True)
        or []
    )
    assert_true(
        any(group in src for src in psrcs),
        f"anchorParentInverseMatrix[0] not fed by group ({psrcs})",
    )
    # Baked body<->bone offset present (16 floats).
    offset = cmds.getAttr(f"{solver}.anchorOffset[0]")
    assert_true(
        offset is not None and len(offset) == 16,
        f"anchorOffset[0] not baked ({offset})",
    )
    print("✓ FOLLOW_BONE body bound to joint via kinematic anchor")
    return True


def test_dynamic_body_data_only():
    """PHYSICS / PHYSICS_BONE: data only — no anchor, no write-back wiring."""
    _group, solver, joint_a, _jb = _make_physics_scene()

    idx = cmds.mmdRigidBody(
        solver,
        name="DynBody",
        bone=joint_a,
        shape="capsule",
        size=(0.3, 2.0, 0.3),
        mass=3.0,
        physicsMode="physics",
    )
    assert_eq(idx, 0, "index != 0")
    assert_eq(
        int(cmds.getAttr(f"{solver}.bodies[0].bodyPhysicsMode")), MODE_PHYSICS, "mode"
    )
    assert_eq(
        int(cmds.getAttr(f"{solver}.bodies[0].bodyColliderType")),
        COLLIDER_CAPSULE,
        "type",
    )

    # Dynamic bodies create NO anchor.
    assert_eq(
        int(cmds.getAttr(f"{solver}.anchorWorldMatrix", size=True) or 0), 0, "no anchor"
    )
    # No write-back: nothing drives the node outputs.
    driven = (cmds.listConnections(f"{solver}.outRotate", destination=True) or []) + (
        cmds.listConnections(f"{solver}.outTranslate", destination=True) or []
    )
    assert_true(not driven, f"simulation disabled but outputs drive: {driven}")

    # PHYSICS_BONE mode is stored the same way (data only).
    idx2 = cmds.mmdRigidBody(
        solver, name="RotBody", bone=joint_a, mass=1.0, physicsMode="physicsBone"
    )
    assert_eq(idx2, 1, "second index != 1")
    assert_eq(
        int(cmds.getAttr(f"{solver}.bodies[1].bodyPhysicsMode")),
        MODE_PHYSICS_BONE,
        "mode 2",
    )
    print("✓ dynamic bodies are data-only (no anchor, no write-back)")
    return True


def test_static_collider_no_bone():
    """FOLLOW_BONE without a bone: a static collider pinned at its rest pose."""
    _group, solver, _ja, _jb = _make_physics_scene()

    idx = cmds.mmdRigidBody(
        solver,
        name="StaticBody",
        shape="sphere",
        size=(1.0, 1.0, 1.0),
        position=(2.0, 0.0, 0.0),
        mass=0.0,
        physicsMode="followBone",
    )
    assert_eq(idx, 0, "index != 0")
    # Pinned anchor: no incoming connection on anchorWorldMatrix[0].
    srcs = cmds.listConnections(f"{solver}.anchorWorldMatrix[0]", source=True) or []
    assert_true(
        not srcs, f"static collider anchor should be pinned, got sources {srcs}"
    )
    # The pinned world matrix holds the body's rest pose in world space.
    pinned = cmds.getAttr(f"{solver}.anchorWorldMatrix[0]")
    assert_true(pinned is not None and len(pinned) == 16, "pinned anchor not set")
    print("✓ no-bone FOLLOW_BONE body is a pinned static collider")
    return True


def test_auto_increment_indices():
    """Bodies auto-append; an explicit index must be the next free index."""
    _group, solver, joint_a, _jb = _make_physics_scene()

    idx0 = cmds.mmdRigidBody(solver, name="B0", bone=joint_a, physicsMode="followBone")
    idx1 = cmds.mmdRigidBody(solver, name="B1", bone=joint_a, physicsMode="physics")
    idx2 = cmds.mmdRigidBody(solver, name="B2", bone=joint_a, physicsMode="followBone")
    assert_eq((idx0, idx1, idx2), (0, 1, 2), "indices not 0,1,2")
    assert_eq(int(cmds.getAttr(f"{solver}.bodies", size=True)), 3, "bodies count != 3")

    # Explicit index equal to the next free index is accepted.
    idx3 = cmds.mmdRigidBody(solver, index=3, name="B3", physicsMode="physics")
    assert_eq(idx3, 3, "explicit index 3 != 3")

    # Explicit index that is NOT the next free index is rejected.
    try:
        cmds.mmdRigidBody(solver, index=1, name="Bad", physicsMode="physics")
        assert_true(False, "stale explicit index was accepted")
    except RuntimeError:
        pass
    print("✓ indices auto-increment (0,1,2,3) and stale indices are rejected")
    return True


def test_clamp01():
    """PMX attenuation values are clamped to 0..1."""
    _group, solver, _ja, _jb = _make_physics_scene()

    cmds.mmdRigidBody(
        solver,
        name="Clamp",
        linearDamping=1.5,
        angularDamping=-0.5,
        friction=2.0,
        restitution=-0.2,
        physicsMode="physics",
    )
    base = f"{solver}.bodies[0]"
    assert_eq(
        float(cmds.getAttr(f"{base}.bodyLinearDamping")),
        1.0,
        "linearDamping clamped to 1",
    )
    assert_eq(
        float(cmds.getAttr(f"{base}.bodyAngularDamping")),
        0.0,
        "angularDamping clamped to 0",
    )
    assert_eq(float(cmds.getAttr(f"{base}.bodyFriction")), 1.0, "friction clamped to 1")
    assert_eq(
        float(cmds.getAttr(f"{base}.bodyRestitution")), 0.0, "restitution clamped to 0"
    )
    print("✓ damping/friction/restitution clamped to 0..1")
    return True


def test_invalid_shape_rejected():
    """An unknown shape string is rejected by the command."""
    _group, solver, _ja, _jb = _make_physics_scene()
    try:
        cmds.mmdRigidBody(solver, name="Bad", shape="torus", physicsMode="physics")
        assert_true(False, "invalid shape was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.bodies", size=True)),
        0,
        "no body should be appended",
    )
    print("✓ unknown shape rejected")
    return True


def test_invalid_physics_mode_rejected():
    """An unknown physicsMode string is rejected by the command."""
    _group, solver, _ja, _jb = _make_physics_scene()
    try:
        cmds.mmdRigidBody(solver, name="Bad", physicsMode="teleport")
        assert_true(False, "invalid physicsMode was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.bodies", size=True)),
        0,
        "no body should be appended",
    )
    print("✓ unknown physicsMode rejected")
    return True


def test_constraint_cmd_appends_joint():
    """mmdRigidBodyConstraint appends one joint with the MMD->Maya conversions."""
    _group, solver, _ja, _jb = _make_physics_scene()

    j_idx = cmds.mmdRigidBodyConstraint(
        solver,
        bodyA=0,
        bodyB=1,
        type=2,
        position=(0.0, 0.0, 0.0),
        rotation=(0.0, 0.0, 0.0),
        linearMin=(-1.0, -2.0, -3.0),
        linearMax=(1.0, 2.0, 3.0),
        angularMin=(-0.5, -0.5, -0.5),
        angularMax=(0.5, 0.5, 0.5),
        linearSpring=(0.1, 0.2, 0.3),
        angularSpring=(0.01, 0.02, 0.03),
    )
    assert_eq(j_idx, 0, "joint index != 0")
    assert_eq(int(cmds.getAttr(f"{solver}.joints", size=True)), 1, "joints count != 1")

    base = f"{solver}.joints[0]"
    assert_eq(int(cmds.getAttr(f"{base}.jointBodyA")), 0, "jointBodyA")
    assert_eq(int(cmds.getAttr(f"{base}.jointBodyB")), 1, "jointBodyB")
    assert_eq(int(cmds.getAttr(f"{base}.jointType")), 2, "jointType")
    # Frame translate is Z-flipped; frame rotate goes MMD radians -> Maya degrees.
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointFrameTranslate")[0], (0.0, 0.0, 0.0)
        ),
        f"jointFrameTranslate {cmds.getAttr(f'{base}.jointFrameTranslate')}",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointFrameRotate")[0], (0.0, 0.0, 0.0)
        ),
        f"jointFrameRotate {cmds.getAttr(f'{base}.jointFrameRotate')}",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointLinearMin")[0], (-1.0, -2.0, -3.0)
        ),
        f"jointLinearMin {cmds.getAttr(f'{base}.jointLinearMin')}",
    )
    assert_true(
        approx_equal_tuple(cmds.getAttr(f"{base}.jointLinearMax")[0], (1.0, 2.0, 3.0)),
        f"jointLinearMax {cmds.getAttr(f'{base}.jointLinearMax')}",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointAngularMin")[0], (-0.5, -0.5, -0.5)
        ),
        f"jointAngularMin {cmds.getAttr(f'{base}.jointAngularMin')}",
    )
    assert_true(
        approx_equal_tuple(cmds.getAttr(f"{base}.jointAngularMax")[0], (0.5, 0.5, 0.5)),
        f"jointAngularMax {cmds.getAttr(f'{base}.jointAngularMax')}",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointLinearSpring")[0], (0.1, 0.2, 0.3)
        ),
        f"jointLinearSpring {cmds.getAttr(f'{base}.jointLinearSpring')}",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointAngularSpring")[0], (0.01, 0.02, 0.03)
        ),
        f"jointAngularSpring {cmds.getAttr(f'{base}.jointAngularSpring')}",
    )
    print("✓ mmdRigidBodyConstraint appends one joint")
    return True


def test_model_root_resolution():
    """The solver is resolved through a model-root pmxPhysicsNode attribute."""
    _group, solver, joint_a, _jb = _make_physics_scene()
    root = cmds.createNode("transform", name="testModelRoot")
    cmds.addAttr(root, longName="pmxPhysicsNode", dataType="string")
    cmds.setAttr(f"{root}.pmxPhysicsNode", solver, type="string")

    idx = cmds.mmdRigidBody(
        root,
        name="ViaRoot",
        bone=joint_a,
        shape="sphere",
        physicsMode="followBone",
    )
    assert_eq(idx, 0, "index via model root != 0")
    assert_eq(
        cmds.getAttr(f"{solver}.bodies[0].bodyNameLocal"),
        "ViaRoot",
        "name via model root",
    )
    print("✓ solver resolved through model root pmxPhysicsNode")
    return True


def test_enum_fields_exposed():
    """The enum attributes expose their fields through attributeQuery."""
    _group, solver, _ja, _jb = _make_physics_scene()

    fields_collider = cmds.attributeQuery(
        "bodyColliderType", node=solver, listEnum=True
    )
    fields_mode = cmds.attributeQuery("bodyPhysicsMode", node=solver, listEnum=True)
    fields_group = cmds.attributeQuery("bodyGroupId", node=solver, listEnum=True)
    assert_true(
        fields_collider
        and "Box" in fields_collider[0]
        and "Sphere" in fields_collider[0]
        and "Capsule" in fields_collider[0],
        f"bodyColliderType fields missing {fields_collider}",
    )
    assert_true(
        fields_mode
        and "FollowBone" in fields_mode[0]
        and "Physics" in fields_mode[0]
        and "PhysicsBone" in fields_mode[0],
        f"bodyPhysicsMode fields missing {fields_mode}",
    )
    assert_true(
        fields_group
        and "Group 0" in fields_group[0]
        and "Group 15" in fields_group[0],
        f"bodyGroupId fields missing {fields_group}",
    )
    print(
        "✓ enum fields exposed (Box/Sphere/Capsule, FollowBone/Physics/PhysicsBone, Group 0..15)"
    )
    return True


def test_body_mask_group_toggles():
    """bodyMask is one boolean toggle per collision group (0..15)."""
    _group, solver, _ja, _jb = _make_physics_scene()

    cmds.mmdRigidBody(solver, name="Toggled", physicsMode="physics")

    # Default: every group enabled (matches the legacy 0xFFFF mask).
    for g in range(16):
        assert_eq(
            bool(cmds.getAttr(f"{solver}.bodies[0].bodyMaskGroup{g}")),
            True,
            f"bodyMaskGroup{g} default != True",
        )

    # Each toggle is independent — disabling one leaves the others alone.
    cmds.setAttr(f"{solver}.bodies[0].bodyMaskGroup3", False)
    cmds.setAttr(f"{solver}.bodies[0].bodyMaskGroup11", False)
    assert_eq(
        bool(cmds.getAttr(f"{solver}.bodies[0].bodyMaskGroup3")),
        False,
        "group 3 not off",
    )
    assert_eq(
        bool(cmds.getAttr(f"{solver}.bodies[0].bodyMaskGroup11")),
        False,
        "group 11 not off",
    )
    for g in (0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15):
        assert_eq(
            bool(cmds.getAttr(f"{solver}.bodies[0].bodyMaskGroup{g}")),
            True,
            f"group {g} unexpectedly off",
        )
    print("✓ bodyMask exposes one boolean toggle per collision group (0..15)")
    return True


# ──────────────────────────────────────────────────────────────────────────
# Test Registry (static — consumed by run_all_integration_tests.py)
# ──────────────────────────────────────────────────────────────────────────

_TESTS = [
    ("Append body stores data", test_append_body_data),
    ("FOLLOW_BONE binds anchor", test_follow_bone_binds_anchor),
    ("Dynamic body data-only", test_dynamic_body_data_only),
    ("Static collider no bone", test_static_collider_no_bone),
    ("Auto-increment indices", test_auto_increment_indices),
    ("Clamp01 on attenuation", test_clamp01),
    ("Invalid shape rejected", test_invalid_shape_rejected),
    ("Invalid physicsMode rejected", test_invalid_physics_mode_rejected),
    ("Constraint cmd appends joint", test_constraint_cmd_appends_joint),
    ("Model root resolution", test_model_root_resolution),
    ("Enum fields exposed", test_enum_fields_exposed),
    ("Body mask group toggles", test_body_mask_group_toggles),
]
