"""
test_pmx_rigid_body_constraint_cmd_integration.py

Integration tests for the native C++ ``pmxRigidBodyConstraint`` command — pure
Maya command testing WITHOUT any PMX model.  The scene is a small mock: a
transform group holding one ``pmxRigidBodyNode`` solver plus two bare joints,
with a couple of rigid bodies appended through ``pmxRigidBody`` (constraints
reference bodies by index, so bodies must exist first).

Tests cover (the "add a joint and configure it on addition" contract):

- Auto-append joint index (0, 1, 2 ...) and stored joint DATA (bodyA/bodyB,
  type, frame translate/rotate, linear/angular limits, springs).
- MMD frame conversion: Z-flip on position, radians -> degrees handedness flip
  on rotation.
- Limits/springs stored VERBATIM (angular stays in PMX radians — the node
  hands angular values to Bullet unchanged).
- Joint type validated to the PMX 0..5 range.
- bodyA/bodyB validated against the current body count.
- -index must be the next free index (rejected otherwise).
- Solver resolution through a model-root ``pmxRigidBodyNode`` attribute.
- Query/edit modes rejected.

NOTE: MayaMMD.mll (which registers pmxRigidBodyNode + the pmxRigidBody and
pmxRigidBodyConstraint commands) is already loaded by the test runner — no
separate plugin loading here.
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


def _make_physics_scene():
    """Create a mock physics group + solver node + two bare joints.

    Returns ``(group, solver, joint_a, joint_b)`` (plain names, no PMX).
    """
    setup_test_environment()
    group = cmds.createNode("transform", name="testPhysicsGroup")
    solver = cmds.createNode("pmxRigidBodyNode", name="testSolver", parent=group)
    cmds.select(clear=True)
    joint_a = cmds.joint(name="testJointA", p=[0, 0, 0])
    cmds.select(clear=True)
    joint_b = cmds.joint(name="testJointB", p=[1, 0, 0])
    return group, solver, joint_a, joint_b


def _add_bodies(solver, count):
    """Append *count* dynamic bodies through ``pmxRigidBody`` (data only)."""
    for i in range(count):
        cmds.pmxRigidBody(solver, name=f"Body{i}", physicsMode="physics")


# ─────────────────────────────────────────────────────────────────────────
# Test Cases
# ─────────────────────────────────────────────────────────────────────────


def test_append_joint_data():
    """Appending a joint stores the full joint DATA at the next free index."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    idx = cmds.pmxRigidBodyConstraint(
        solver,
        name="LeftChest",
        nameUniversal="LeftChest_univ",
        bodyA=0,
        bodyB=1,
        type=0,
        position=(1.0, 2.0, 3.0),
        rotation=(0.0, 1.5707963267948966, 0.0),  # pi/2
        linearMin=(-2.0, -3.0, -4.0),
        linearMax=(2.0, 3.0, 4.0),
        angularMin=(-0.1, -0.2, -0.3),
        angularMax=(0.1, 0.2, 0.3),
        linearSpring=(5.0, 6.0, 7.0),
        angularSpring=(8.0, 9.0, 10.0),
    )
    assert_eq(idx, 0, f"first joint index != 0 ({idx})")
    assert_eq(int(cmds.getAttr(f"{solver}.joints", size=True)), 1, "joints count != 1")

    base = f"{solver}.joints[0]"
    assert_eq(cmds.getAttr(f"{base}.jointNameLocal"), "LeftChest", "jointNameLocal")
    assert_eq(
        cmds.getAttr(f"{base}.jointNameUniversal"),
        "LeftChest_univ",
        "jointNameUniversal",
    )
    assert_eq(int(cmds.getAttr(f"{base}.jointBodyA")), 0, "jointBodyA")
    assert_eq(int(cmds.getAttr(f"{base}.jointBodyB")), 1, "jointBodyB")
    assert_eq(int(cmds.getAttr(f"{base}.jointType")), 0, "jointType")
    print("✓ joint appended with names/bodyA/bodyB/type at the next free index")
    return True


def test_frame_conversion():
    """The MMD frame is converted to Maya space (Z-flip + handedness)."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    cmds.pmxRigidBodyConstraint(
        solver,
        bodyA=0,
        bodyB=1,
        type=0,
        position=(1.0, 2.0, 3.0),
        rotation=(1.5707963267948966, 0.0, 0.0),  # pi/2 rad on X
    )
    base = f"{solver}.joints[0]"
    # Z-flip: position z=3 -> -3.
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointFrameTranslate")[0], (1.0, 2.0, -3.0)
        ),
        f"jointFrameTranslate Z-flip wrong {cmds.getAttr(f'{base}.jointFrameTranslate')}",
    )
    # pi/2 rad -> -90 deg on X (handedness flip), Y/Z untouched.
    frame_rot = cmds.getAttr(f"{base}.jointFrameRotate")[0]
    assert_true(
        approx_equal_tuple(frame_rot, (-90.0, 0.0, 0.0), tolerance=1e-3),
        f"jointFrameRotate handedness flip wrong {frame_rot}",
    )
    print("✓ MMD joint frame converted (Z-flip + radians->degrees handedness)")
    return True


def test_limits_springs_converted():
    """Limits pass through the MMD->Maya reflection; springs stay verbatim.

    The MMD->Maya conversion is the reflection F = diag(1, 1, -1): linear Z
    negates + min/max swap, angular X/Y negate + min/max swap, angular Z and
    springs are invariant.  SYMMETRIC limits are unchanged by the flip, so
    this test uses ASYMMETRIC values to prove the conversion actually ran.
    """
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    cmds.pmxRigidBodyConstraint(
        solver,
        bodyA=0,
        bodyB=1,
        type=0,
        linearMin=(-1.0, -2.0, -3.0),
        linearMax=(4.0, 5.0, 6.0),
        angularMin=(-0.5, -0.2, -0.7),
        angularMax=(0.1, 0.6, 0.7),
        linearSpring=(4.0, 5.0, 6.0),
        angularSpring=(7.0, 8.0, 9.0),
    )
    base = f"{solver}.joints[0]"
    # Linear: X/Y unchanged; Z negated + min/max swapped.
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointLinearMin")[0], (-1.0, -2.0, -6.0)
        ),
        f"jointLinearMin flip wrong {cmds.getAttr(f'{base}.jointLinearMin')}",
    )
    assert_true(
        approx_equal_tuple(cmds.getAttr(f"{base}.jointLinearMax")[0], (4.0, 5.0, 3.0)),
        f"jointLinearMax flip wrong {cmds.getAttr(f'{base}.jointLinearMax')}",
    )
    # Angular: X/Y negated + min/max swapped; Z unchanged (radians, NOT
    # converted to degrees — the node hands angular values to Bullet).
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointAngularMin")[0], (-0.1, -0.6, -0.7)
        ),
        f"jointAngularMin flip wrong {cmds.getAttr(f'{base}.jointAngularMin')}",
    )
    assert_true(
        approx_equal_tuple(cmds.getAttr(f"{base}.jointAngularMax")[0], (0.5, 0.2, 0.7)),
        f"jointAngularMax flip wrong {cmds.getAttr(f'{base}.jointAngularMax')}",
    )
    # Springs are magnitudes — invariant under the reflection.
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointLinearSpring")[0], (4.0, 5.0, 6.0)
        ),
        "jointLinearSpring not verbatim",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointAngularSpring")[0], (7.0, 8.0, 9.0)
        ),
        "jointAngularSpring not verbatim",
    )
    print("✓ limits converted through the MMD->Maya reflection; springs verbatim")
    return True


def test_asymmetric_angular_limits_real_model():
    """A real chest-joint's asymmetric angular limits are NOT stored mirrored.

    The Endfield test model's chest joints carry min=(-0.524,-0.175,-0.087)
    max=(0.087,0.175,0.087).  Under the reflection the X interval must become
    [-0.087, +0.524] (negated + swapped) — storing it verbatim would let the
    joint rotate the wrong way in the sim.
    """
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    cmds.pmxRigidBodyConstraint(
        solver,
        bodyA=0,
        bodyB=1,
        type=0,
        angularMin=(-0.524, -0.175, -0.087),
        angularMax=(0.087, 0.175, 0.087),
    )
    base = f"{solver}.joints[0]"
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointAngularMin")[0], (-0.087, -0.175, -0.087)
        ),
        f"chest angularMin wrong {cmds.getAttr(f'{base}.jointAngularMin')}",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointAngularMax")[0], (0.524, 0.175, 0.087)
        ),
        f"chest angularMax wrong {cmds.getAttr(f'{base}.jointAngularMax')}",
    )
    print("✓ asymmetric chest-joint limits converted (not mirrored)")
    return True


def test_frame_in_world_space():
    """The joint frame is stored in WORLD space, independent of the rigid bodies group.

    The Bullet world runs in world space, so a joint at world (1,2,3) under a
    group translated to (10,0,0) is stored at the raw world position (1,2,-3)
    — NOT shifted by the group.  (Before, the frame was stored group-space and
    the solver's location mattered.)
    """
    group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)
    cmds.move(10.0, 0.0, 0.0, group)

    cmds.pmxRigidBodyConstraint(
        solver,
        bodyA=0,
        bodyB=1,
        type=0,
        position=(1.0, 2.0, 3.0),
    )
    base = f"{solver}.joints[0]"
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointFrameTranslate")[0], (1.0, 2.0, -3.0)
        ),
        f"jointFrameTranslate not in world space {cmds.getAttr(f'{base}.jointFrameTranslate')}",
    )
    print("✓ joint frame stored in world space (group transform ignored)")
    return True


def test_self_constraint_rejected():
    """A joint linking a body to ITSELF (bodyA == bodyB) is rejected."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    try:
        cmds.pmxRigidBodyConstraint(solver, bodyA=1, bodyB=1, type=0)
        assert_true(False, "bodyA == bodyB was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.joints", size=True) or 0),
        0,
        "rejected self-constraint should not create a joint",
    )
    print("✓ bodyA == bodyB self-constraint rejected")
    return True


def test_auto_increment_indices():
    """Joints auto-append at 0, 1, 2 ... in creation order."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    assert_eq(
        cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1, type=0), 0, "joint 0 idx"
    )
    assert_eq(
        cmds.pmxRigidBodyConstraint(solver, bodyA=1, bodyB=0, type=1), 1, "joint 1 idx"
    )
    assert_eq(
        cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1, type=2), 2, "joint 2 idx"
    )
    assert_eq(int(cmds.getAttr(f"{solver}.joints", size=True)), 3, "joints count")
    print("✓ joints auto-increment 0, 1, 2 ...")
    return True


def test_explicit_index_validation():
    """-index must be the next free index; otherwise it is rejected."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    assert_eq(
        cmds.pmxRigidBodyConstraint(solver, index=0, bodyA=0, bodyB=1, type=0),
        0,
        "explicit next-free index ok",
    )
    try:
        cmds.pmxRigidBodyConstraint(solver, index=5, bodyA=0, bodyB=1, type=0)
        assert_true(False, "-index 5 (not next free) was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.joints", size=True)),
        1,
        "rejected index should not create a joint",
    )
    print("✓ -index validated against the next free index")
    return True


def test_type_validated():
    """A joint type outside the PMX 0..5 range is rejected."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    for bad_type in (-1, 6):
        try:
            cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1, type=bad_type)
            assert_true(False, f"joint type {bad_type} was accepted")
        except RuntimeError:
            pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.joints", size=True) or 0),
        0,
        "rejected type should not create a joint",
    )
    print("✓ joint type validated to 0..5")
    return True


def test_joint_type_enum_fields_exposed():
    """jointType is an enum dropdown exposing the six PMX JointType fields."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    fields = cmds.attributeQuery("jointType", node=solver, listEnum=True)
    assert_true(
        fields
        and "Spring6Dof" in fields[0]
        and "SixDof" in fields[0]
        and "P2P" in fields[0]
        and "ConeTwist" in fields[0]
        and "Slider" in fields[0]
        and "Hinge" in fields[0],
        f"jointType fields missing {fields}",
    )
    # The field VALUES are the PMX JointType values (0..5) — writing 4 (Slider)
    # and reading back yields 4.
    cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1, type=4)
    assert_eq(int(cmds.getAttr(f"{solver}.joints[0].jointType")), 4, "enum value write")
    print("✓ jointType exposes the six PMX JointType fields as a dropdown")
    return True


def test_body_indices_validated():
    """bodyA/bodyB are validated against the node's current body count."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    for body_a, body_b in ((-1, 0), (0, -1), (2, 0), (0, 2), (99, 0)):
        try:
            cmds.pmxRigidBodyConstraint(solver, bodyA=body_a, bodyB=body_b, type=0)
            assert_true(False, f"bodyA/bodyB ({body_a},{body_b}) was accepted")
        except RuntimeError:
            pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.joints", size=True) or 0),
        0,
        "rejected body indices should not create a joint",
    )
    print("✓ bodyA/bodyB validated against the body count")
    return True


def test_model_root_resolution():
    """The solver is resolved through a model-root pmxRigidBodyNode attribute."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)
    root = cmds.createNode("transform", name="testModelRoot")
    cmds.addAttr(root, longName="pmxRigidBodyNode", dataType="string")
    cmds.setAttr(f"{root}.pmxRigidBodyNode", solver, type="string")

    idx = cmds.pmxRigidBodyConstraint(
        root, bodyA=0, bodyB=1, type=0, position=(4.0, 0.0, 0.0)
    )
    assert_eq(idx, 0, "index via model root != 0")
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{solver}.joints[0].jointFrameTranslate")[0], (4.0, 0.0, 0.0)
        ),
        "frame translate via model root",
    )
    print("✓ solver resolved through model root pmxRigidBodyNode")
    return True


def test_query_edit_rejected():
    """Query/edit modes are not implemented yet and are rejected."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    try:
        cmds.pmxRigidBodyConstraint(solver, query=True)
        assert_true(False, "query mode was accepted")
    except RuntimeError:
        pass
    try:
        cmds.pmxRigidBodyConstraint(solver, edit=True, bodyA=0, bodyB=1, type=0)
        assert_true(False, "edit mode was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.joints", size=True) or 0),
        0,
        "rejected query/edit should not create a joint",
    )
    print("✓ query/edit modes rejected (not implemented yet)")
    return True


def test_invalid_target_rejected():
    """A non-solver, non-model-root target is rejected."""
    _group, _solver, _ja, _jb = _make_physics_scene()
    plain = cmds.createNode("transform", name="plainTransform")

    try:
        cmds.pmxRigidBodyConstraint(plain, bodyA=0, bodyB=1, type=0)
        assert_true(False, "non-solver target was accepted")
    except RuntimeError:
        pass
    print("✓ non-solver target rejected")
    return True


def test_missing_solver_argument_rejected():
    """A missing solver argument is reported as an error."""
    _group, _solver, _ja, _jb = _make_physics_scene()

    try:
        cmds.pmxRigidBodyConstraint()
        assert_true(False, "missing solver argument was accepted")
    except RuntimeError:
        pass
    print("✓ missing solver argument rejected")
    return True


# ─────────────────────────────────────────────────────────────────────────
# Test Registry (static — consumed by run_all_integration_tests.py)
# ─────────────────────────────────────────────────────────────────────────

_TESTS = [
    ("Append joint stores data", test_append_joint_data),
    ("Joint frame conversion", test_frame_conversion),
    ("Limits converted through reflection", test_limits_springs_converted),
    (
        "Asymmetric angular limits (real model)",
        test_asymmetric_angular_limits_real_model,
    ),
    ("Frame stored in world space", test_frame_in_world_space),
    ("Self-constraint rejected", test_self_constraint_rejected),
    ("Auto-increment joint indices", test_auto_increment_indices),
    ("Explicit index validated", test_explicit_index_validation),
    ("Joint type validated", test_type_validated),
    ("Joint type enum dropdown", test_joint_type_enum_fields_exposed),
    ("Body indices validated", test_body_indices_validated),
    ("Model root resolution", test_model_root_resolution),
    ("Query/edit rejected", test_query_edit_rejected),
    ("Invalid target rejected", test_invalid_target_rejected),
    ("Missing solver argument rejected", test_missing_solver_argument_rejected),
]
