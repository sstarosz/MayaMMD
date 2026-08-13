"""
test_pmx_rigid_body_cmd_integration.py

Integration tests for the native C++ ``pmxRigidBody`` command — pure Maya
command testing WITHOUT any PMX model.  The scene is a small mock: a transform
group holding one ``pmxPhysicsNode`` solver plus a couple of bare joints.

Tests cover (the "add a body and configure it on addition" contract):

- Auto-append body index (0, 1, 2 ...) and stored body DATA (names, mass,
  damping, friction, restitution, group, mask, shape, physics mode, rest pose,
  PMX shape_size verbatim).
- FOLLOW_BONE + bone -> kinematic anchor binding (anchorWorldMatrix fed by
  the joint; the body<->bone offset is DERIVED by the node from
  bodyWriteBackOffset).
- PHYSICS / PHYSICS_BONE on a bone -> write-back wired AT CREATION: a dynamic
  body with a related joint gets its outTranslate/outRotate connected straight
  into the joint (PHYSICS_BONE rotation-only).  A body WITHOUT a joint (static
  collider) gets no wiring.
- FOLLOW_BONE without a bone -> a static collider pinned at its rest pose.
- clamp01 on damping/friction/restitution.
- Invalid shape / physicsMode values are rejected.
- Solver resolution through a model-root ``pmxPhysicsNode`` attribute.
- Enum attributes: getAttr returns the numeric field value; the enum field
  list is exposed via attributeQuery(listEnum=True).

NOTE: MayaMMD.mll (which registers pmxPhysicsNode + the pmxRigidBody command)
is already loaded by the test runner — no separate plugin loading here.

NOTE: the native ``pmxRigidBodyConstraint`` command is NOT covered here — it
lands in a separate PR (its own integration test file).
"""

# ── Maya standalone initialised by the test runner ────────────────────────

# Maya imports (safe after standalone.initialize())
from maya import cmds

# Test framework imports
from tests.integration.test_helpers import (
    approx_equal_tuple,
    assert_eq,
    assert_true,
    setup_test_environment,
)

# PMX collider type field values (PhysicsNode::ColliderType).
COLLIDER_BOX = 1
COLLIDER_SPHERE = 2
COLLIDER_CAPSULE = 3
# PhysicsMode field values (mmd::core::Simulation::PhysicsMode).
MODE_FOLLOW_BONE = 0
MODE_PHYSICS = 1
MODE_PHYSICS_BONE = 2


def _make_physics_scene():
    """Create a mock physics group + solver node + two bare joints.

    Returns ``(group, solver, joint_a, joint_b)`` (plain names, no PMX).
    """
    setup_test_environment()
    group = cmds.createNode("transform", name="testPhysicsGroup")
    solver = cmds.createNode("pmxPhysicsNode", name="testSolver", parent=group)
    cmds.select(clear=True)
    joint_a = cmds.joint(name="testJointA", p=[0, 0, 0])
    cmds.select(clear=True)
    joint_b = cmds.joint(name="testJointB", p=[1, 0, 0])
    return group, solver, joint_a, joint_b


# ─────────────────────────────────────────────────────────────────────────
# Test Cases
# ─────────────────────────────────────────────────────────────────────────


def test_append_body_data():
    """Appending a body stores the full body DATA at the next free index."""
    _group, solver, _ja, _jb = _make_physics_scene()

    idx = cmds.pmxRigidBody(
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
        mask=0xFFFE,
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
    # mask=0xFFFE = every group except 0 -> bodyMaskGroup bools (collides-with).
    assert_eq(
        bool(cmds.getAttr(f"{base}.bodyMaskGroup0")),
        False,
        "bodyMaskGroup0 (mask 0xFFFE clears bit 0)",
    )
    for g in range(1, 16):
        assert_eq(
            bool(cmds.getAttr(f"{base}.bodyMaskGroup{g}")),
            True,
            f"bodyMaskGroup{g} (mask 0xFFFE keeps bit {g})",
        )
    assert_eq(
        int(cmds.getAttr(f"{base}.bodyColliderType")),
        COLLIDER_BOX,
        "colliderType (box)",
    )
    assert_eq(
        int(cmds.getAttr(f"{base}.bodyPhysicsMode")), MODE_FOLLOW_BONE, "physicsMode"
    )
    # PMX shape_size VERBATIM (full size — box extents are full, not half).
    # (float3 compound children come back from getAttr as [(x, y, z)].)
    assert_true(
        approx_equal_tuple(cmds.getAttr(f"{base}.bodyShapeSize")[0], (1.0, 2.0, 3.0)),
        f"bodyShapeSize != size {cmds.getAttr(f'{base}.bodyShapeSize')}",
    )
    # Rest pose stored in world space: the group is at the origin, so the MMD
    # position (Z-flip of 0 == 0) lands directly in world space.
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
    _group, solver, joint_a, _jb = _make_physics_scene()

    idx = cmds.pmxRigidBody(
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
    # The body<->bone offset is DERIVED by the node (from bodyWriteBackOffset)
    # — the command no longer writes an anchorOffset input.
    print("✓ FOLLOW_BONE body bound to joint via kinematic anchor")
    return True


def test_no_bone_body_has_no_write_back():
    """A body without a related joint (static collider) gets no write-back."""
    _group, solver, _ja, _jb = _make_physics_scene()

    idx = cmds.pmxRigidBody(
        solver,
        name="Static",
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

    # No anchor, and no joint to connect to -> no write-back wiring.
    assert_eq(
        int(cmds.getAttr(f"{solver}.anchorWorldMatrix", size=True) or 0), 0, "no anchor"
    )
    driven = (cmds.listConnections(f"{solver}.outRotate", destination=True) or []) + (
        cmds.listConnections(f"{solver}.outTranslate", destination=True) or []
    )
    assert_true(not driven, f"static collider must not drive outputs: {driven}")
    print("✓ body without a joint (static collider) gets no write-back")
    return True


def test_static_collider_no_bone():
    """FOLLOW_BONE without a bone: a static collider pinned at its rest pose."""
    _group, solver, _ja, _jb = _make_physics_scene()

    idx = cmds.pmxRigidBody(
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

    idx0 = cmds.pmxRigidBody(solver, name="B0", bone=joint_a, physicsMode="followBone")
    idx1 = cmds.pmxRigidBody(solver, name="B1", bone=joint_a, physicsMode="physics")
    idx2 = cmds.pmxRigidBody(solver, name="B2", bone=joint_a, physicsMode="followBone")
    assert_eq((idx0, idx1, idx2), (0, 1, 2), "indices not 0,1,2")
    assert_eq(int(cmds.getAttr(f"{solver}.bodies", size=True)), 3, "bodies count != 3")

    # Explicit index equal to the next free index is accepted.
    idx3 = cmds.pmxRigidBody(solver, index=3, name="B3", physicsMode="physics")
    assert_eq(idx3, 3, "explicit index 3 != 3")

    # Explicit index that is NOT the next free index is rejected.
    try:
        cmds.pmxRigidBody(solver, index=1, name="Bad", physicsMode="physics")
        assert_true(False, "stale explicit index was accepted")
    except RuntimeError:
        pass
    print("✓ indices auto-increment (0,1,2,3) and stale indices are rejected")
    return True


def test_clamp01():
    """PMX attenuation values are clamped to 0..1."""
    _group, solver, _ja, _jb = _make_physics_scene()

    cmds.pmxRigidBody(
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
        cmds.pmxRigidBody(solver, name="Bad", shape="torus", physicsMode="physics")
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
        cmds.pmxRigidBody(solver, name="Bad", physicsMode="teleport")
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


def test_model_root_resolution():
    """The solver is resolved through a model-root pmxPhysicsNode attribute."""
    _group, solver, joint_a, _jb = _make_physics_scene()
    root = cmds.createNode("transform", name="testModelRoot")
    cmds.addAttr(root, longName="pmxPhysicsNode", dataType="string")
    cmds.setAttr(f"{root}.pmxPhysicsNode", solver, type="string")

    idx = cmds.pmxRigidBody(
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
        fields_group and "Group 0" in fields_group[0] and "Group 15" in fields_group[0],
        f"bodyGroupId fields missing {fields_group}",
    )
    print(
        "✓ enum fields exposed (Box/Sphere/Capsule, FollowBone/Physics/PhysicsBone, Group 0..15)"
    )
    return True


def test_body_mask_group_toggles():
    """bodyMask is one boolean toggle per collision group (0..15)."""
    _group, solver, _ja, _jb = _make_physics_scene()

    cmds.pmxRigidBody(solver, name="Toggled", physicsMode="physics")

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


def test_related_joint_connected():
    """pmxRigidBody connects the body's related joint as a MESSAGE."""
    _group, solver, joint_a, _jb = _make_physics_scene()

    # Give the mock joint a PMX bone index so the node can resolve it back.
    cmds.addAttr(
        joint_a, longName="pmxBoneIndex", attributeType="long", defaultValue=-1
    )
    cmds.setAttr(f"{joint_a}.pmxBoneIndex", 7)

    idx = cmds.pmxRigidBody(solver, name="Wired", bone=joint_a, physicsMode="physics")
    assert_eq(idx, 0, "index != 0")
    base = f"{solver}.bodies[0]"
    # The related joint's message plug is the source of bodies[0].bodyJoint.
    srcs = cmds.listConnections(f"{base}.bodyJoint", source=True) or []
    assert_true(
        bool(srcs) and joint_a in srcs, f"bodyJoint not connected to {joint_a} ({srcs})"
    )
    # A body without a bone stays unconnected (a static collider, no write-back).
    idx2 = cmds.pmxRigidBody(solver, name="NoBone", physicsMode="physics")
    assert_eq(idx2, 1, "index != 1")
    assert_eq(
        cmds.listConnections(f"{solver}.bodies[1].bodyJoint", source=True) or [],
        [],
        "no-bone bodyJoint must be unconnected",
    )
    # Enabled by default.
    assert_eq(bool(cmds.getAttr(f"{base}.bodyEnabled")), True, "bodyEnabled != True")
    print(
        "✓ bodyJoint message connected to the related joint; absent for no-bone bodies"
    )
    return True


def test_write_back_connected_when_drivable():
    """A dynamic body on a bone ALWAYS gets its outputs wired at creation."""
    _group, solver, joint_a, joint_b = _make_physics_scene()
    cmds.select(clear=True)
    joint_c = cmds.joint(name="testJointC", p=[2, 0, 0])
    # Mock joints get PMX bone indices + a DAG parent chain (b0 root, b1<-b0,
    # b2<-b1) — the node resolves the hierarchy from the DAG.
    for j, bone_idx in ((joint_a, 0), (joint_b, 1), (joint_c, 2)):
        cmds.addAttr(j, longName="pmxBoneIndex", attributeType="long", defaultValue=-1)
        cmds.setAttr(f"{j}.pmxBoneIndex", bone_idx)
    cmds.parent(joint_b, joint_a)
    cmds.parent(joint_c, joint_b)

    # Body 0 on bone 0 (kinematic — never driven).
    cmds.pmxRigidBody(solver, name="Anch", bone=joint_a, physicsMode="followBone")
    # Body 1 on bone 1 (dynamic) — wired unconditionally at creation.
    cmds.pmxRigidBody(solver, name="Dyn", bone=joint_b, physicsMode="physics")
    # Body 2 on bone 2 (PHYSICS_BONE) — its OWN joint, rotation-only.
    cmds.pmxRigidBody(solver, name="RotOnly", bone=joint_c, physicsMode="physicsBone")

    rot_dests = cmds.listConnections(f"{solver}.outRotate[1]", destination=True) or []
    assert_true(
        bool(rot_dests) and all("unitConversion" not in str(d) for d in rot_dests),
        "outRotate[1] not connected directly to the joint",
    )
    tr_dests = cmds.listConnections(f"{solver}.outTranslate[1]", destination=True) or []
    assert_true(
        bool(tr_dests) and all("unitConversion" not in str(d) for d in tr_dests),
        "outTranslate[1] not connected directly to the joint",
    )
    assert_true(
        not (cmds.listConnections(f"{solver}.outRotate[0]", destination=True) or []),
        "kinematic body must not be driven",
    )

    # PHYSICS_BONE is rotation-only (its own joint, so nothing is stolen).
    rot2 = cmds.listConnections(f"{solver}.outRotate[2]", destination=True) or []
    tr2 = cmds.listConnections(f"{solver}.outTranslate[2]", destination=True) or []
    assert_true(bool(rot2), "PHYSICS_BONE must drive rotate")
    assert_true(not tr2, "PHYSICS_BONE must NOT drive translate")
    print("✓ dynamic bodies on a bone always wired; kinematic/PHYSICS_BONE respected")
    return True


def test_write_back_offset_baked():
    """A body with a related joint gets its write-back K offset baked."""
    _group, solver, joint_a, _jb = _make_physics_scene()

    cmds.pmxRigidBody(
        solver,
        name="KBody",
        bone=joint_a,
        position=(0.0, 2.0, 0.0),
        physicsMode="physics",
    )
    k = cmds.getAttr(f"{solver}.bodyWriteBackOffset[0]")
    assert_true(
        k is not None and len(k) == 16, f"bodyWriteBackOffset[0] not baked ({k})"
    )
    # K = jointRestWorld * bodyRestWorld^-1 — with a non-zero rest offset the
    # baked matrix is NOT identity (Maya matrices are row-major: translation
    # lives in the last row, flat indices 12..14 — here the 2-unit Y offset
    # between the joint and the body rest shows up inverted in K's T).
    k_trans = [k[12], k[13], k[14]]
    assert_true(
        any(abs(v) > 1e-6 for v in k_trans),
        f"bodyWriteBackOffset[0] looks like identity ({k_trans})",
    )
    # A body WITHOUT a joint gets an identity K.
    cmds.pmxRigidBody(
        solver, name="NoJoint", position=(0.0, 3.0, 0.0), physicsMode="physics"
    )
    k2 = cmds.getAttr(f"{solver}.bodyWriteBackOffset[1]")
    assert_true(k2 is not None and len(k2) == 16, "bodyWriteBackOffset[1] not baked")
    assert_true(
        approx_equal_tuple((k2[12], k2[13], k2[14]), (0.0, 0.0, 0.0)),
        f"no-joint body K should be identity ({k2[12]},{k2[13]},{k2[14]})",
    )
    print("✓ bodyWriteBackOffset baked (K for joint bodies, identity otherwise)")
    return True


def test_shape_size_verbatim_per_collider():
    """-size is stored VERBATIM into bodyShapeSize for every collider type."""
    _group, solver, _ja, _jb = _make_physics_scene()

    cmds.pmxRigidBody(
        solver, name="Box", shape="box", size=(1.0, 2.0, 3.0), physicsMode="physics"
    )
    cmds.pmxRigidBody(
        solver,
        name="Sphere",
        shape="sphere",
        size=(0.4, 0.0, 0.0),
        physicsMode="physics",
    )
    cmds.pmxRigidBody(
        solver,
        name="Capsule",
        shape="capsule",
        size=(0.3, 2.0, 0.0),
        physicsMode="physics",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{solver}.bodies[0].bodyShapeSize")[0], (1.0, 2.0, 3.0)
        ),
        "box shapeSize not verbatim",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{solver}.bodies[1].bodyShapeSize")[0], (0.4, 0.0, 0.0)
        ),
        "sphere shapeSize not verbatim",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{solver}.bodies[2].bodyShapeSize")[0], (0.3, 2.0, 0.0)
        ),
        "capsule shapeSize not verbatim",
    )
    print("✓ bodyShapeSize stores PMX shape_size verbatim for box/sphere/capsule")
    return True


def test_rest_pose_conversion():
    """MMD rest pose is stored in world space (Z-flip + handedness flip)."""
    _group, solver, _ja, _jb = _make_physics_scene()

    # position z=2 flips to -2; rotation x=90deg (MMD radians) flips to -90.
    cmds.pmxRigidBody(
        solver,
        name="Rotated",
        position=(1.0, 2.0, 3.0),
        rotation=(1.5707963267948966, 0.0, 0.0),  # pi/2
        physicsMode="physics",
    )
    base = f"{solver}.bodies[0]"
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.bodyRestTranslate")[0], (1.0, 2.0, -3.0)
        ),
        f"bodyRestTranslate Z-flip wrong {cmds.getAttr(f'{base}.bodyRestTranslate')}",
    )
    # pi/2 rad -> 90 deg, negated on X (handedness flip).
    rest_rot = cmds.getAttr(f"{base}.bodyRestRotate")[0]
    assert_true(
        approx_equal_tuple(rest_rot, (-90.0, 0.0, 0.0), tolerance=1e-3),
        f"bodyRestRotate handedness flip wrong {rest_rot}",
    )
    print("✓ MMD rest pose stored in world space (Z-flip + handedness)")
    return True


def test_kinematic_anchor_ordering():
    """Kinematic anchors are indexed in FOLLOW_BONE body order (not PMX order)."""
    group, solver, joint_a, joint_b = _make_physics_scene()

    # B0: dynamic, B1: follow-bone, B2: dynamic, B3: follow-bone.
    cmds.pmxRigidBody(solver, name="Dyn0", bone=joint_a, physicsMode="physics")
    cmds.pmxRigidBody(solver, name="Kin1", bone=joint_a, physicsMode="followBone")
    cmds.pmxRigidBody(solver, name="Dyn2", bone=joint_a, physicsMode="physics")
    cmds.pmxRigidBody(solver, name="Kin3", bone=joint_b, physicsMode="followBone")

    # Two kinematic bodies -> two anchors; anchor[0] is the FIRST follow-bone
    # body (joint_a), anchor[1] is the second (joint_b).
    assert_eq(
        int(cmds.getAttr(f"{solver}.anchorWorldMatrix", size=True)), 2, "anchor count"
    )
    srcs0 = cmds.listConnections(f"{solver}.anchorWorldMatrix[0]", source=True) or []
    assert_true(
        any(joint_a in src for src in srcs0),
        f"anchor[0] not fed by joint_a ({srcs0})",
    )
    srcs1 = cmds.listConnections(f"{solver}.anchorWorldMatrix[1]", source=True) or []
    assert_true(
        any(joint_b in src for src in srcs1),
        f"anchor[1] not fed by joint_b ({srcs1})",
    )
    print("✓ kinematic anchors indexed in FOLLOW_BONE body order")
    return True


def test_numeric_bone_index():
    """-bone <pmxBoneIdx> resolves the related joint by its PMX bone index."""
    _group, solver, joint_a, _jb = _make_physics_scene()
    # Give the joint a PMX bone index (as the bone builder does on import).
    cmds.addAttr(joint_a, longName="pmxBoneIndex", attributeType="long")
    cmds.setAttr(f"{joint_a}.pmxBoneIndex", 7)

    idx = cmds.pmxRigidBody(solver, name="ByIdx", bone="7", physicsMode="followBone")
    assert_eq(idx, 0, "index != 0")
    # Anchor fed by the joint that carries pmxBoneIndex == 7.
    srcs = cmds.listConnections(f"{solver}.anchorWorldMatrix[0]", source=True) or []
    assert_true(
        any(joint_a in src for src in srcs),
        f"anchor[0] not fed by the pmxBoneIndex=7 joint ({srcs})",
    )
    print("✓ -bone <pmxBoneIdx> resolves the related joint")
    return True


def test_query_edit_rejected():
    """Query/edit modes are not implemented yet and are rejected."""
    _group, solver, _ja, _jb = _make_physics_scene()

    try:
        cmds.pmxRigidBody(solver, query=True)
        assert_true(False, "query mode was accepted")
    except RuntimeError:
        pass
    try:
        cmds.pmxRigidBody(solver, edit=True, name="Edited")
        assert_true(False, "edit mode was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.bodies", size=True) or 0),
        0,
        "rejected query/edit should not create a body",
    )
    print("✓ query/edit modes rejected (not implemented yet)")
    return True


def test_invalid_target_rejected():
    """A non-solver, non-model-root target is rejected."""
    _group, _solver, _ja, _jb = _make_physics_scene()
    plain = cmds.createNode("transform", name="plainTransform")

    try:
        cmds.pmxRigidBody(plain, name="Bad", physicsMode="physics")
        assert_true(False, "non-solver target was accepted")
    except RuntimeError:
        pass
    print("✓ non-solver target rejected")
    return True


def test_missing_solver_argument_rejected():
    """A missing solver argument is reported as an error."""
    _group, _solver, _ja, _jb = _make_physics_scene()

    try:
        cmds.pmxRigidBody()
        assert_true(False, "missing solver argument was accepted")
    except RuntimeError:
        pass
    print("✓ missing solver argument rejected")
    return True


def test_group_clamped():
    """-group is clamped to the PMX 0..15 range."""
    _group, solver, _ja, _jb = _make_physics_scene()

    # Out-of-range values clamp to the enum bounds instead of writing garbage.
    cmds.pmxRigidBody(solver, name="Low", group=-5, physicsMode="physics")
    cmds.pmxRigidBody(solver, name="High", group=99, physicsMode="physics")
    assert_eq(int(cmds.getAttr(f"{solver}.bodies[0].bodyGroupId")), 0, "group<0 -> 0")
    assert_eq(
        int(cmds.getAttr(f"{solver}.bodies[1].bodyGroupId")), 15, "group>15 -> 15"
    )
    print("✓ -group clamped to 0..15")
    return True


# ─────────────────────────────────────────────────────────────────────────
# Test Registry (static — consumed by run_all_integration_tests.py)
# ─────────────────────────────────────────────────────────────────────────

_TESTS = [
    ("Append body stores data", test_append_body_data),
    ("FOLLOW_BONE binds anchor", test_follow_bone_binds_anchor),
    ("No-bone body has no write-back", test_no_bone_body_has_no_write_back),
    ("Write-back connected when drivable", test_write_back_connected_when_drivable),
    ("Static collider no bone", test_static_collider_no_bone),
    ("Auto-increment indices", test_auto_increment_indices),
    ("Clamp01 on attenuation", test_clamp01),
    ("Invalid shape rejected", test_invalid_shape_rejected),
    ("Invalid physicsMode rejected", test_invalid_physics_mode_rejected),
    ("Model root resolution", test_model_root_resolution),
    ("Enum fields exposed", test_enum_fields_exposed),
    ("Body mask group toggles", test_body_mask_group_toggles),
    ("Related joint connected", test_related_joint_connected),
    ("Write-back offset baked", test_write_back_offset_baked),
    ("Shape size verbatim per collider", test_shape_size_verbatim_per_collider),
    ("Rest pose conversion", test_rest_pose_conversion),
    ("Kinematic anchor ordering", test_kinematic_anchor_ordering),
    ("Numeric bone index", test_numeric_bone_index),
    ("Query/edit rejected", test_query_edit_rejected),
    ("Invalid target rejected", test_invalid_target_rejected),
    ("Missing solver argument rejected", test_missing_solver_argument_rejected),
    ("Group clamped", test_group_clamped),
]
