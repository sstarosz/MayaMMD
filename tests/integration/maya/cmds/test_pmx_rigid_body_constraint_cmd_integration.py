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
- Joint type validated: only PMX 2.0 Spring6Dof (0) is accepted.
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
    get_angle3_radians,
    set_angle3_radians,
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


def _joint_at(solver: str, index: int) -> str:
    """The pmxRigidBodyJoint node connected to solver.jointShapes[index]."""
    plugs = (
        cmds.listConnections(f"{solver}.jointShapes[{index}]", source=True, plugs=True)
        or []
    )
    assert_eq(len(plugs), 1, f"jointShapes[{index}] should have one source ({plugs})")
    return str(plugs[0]).split(".")[0]


def _joint_guide_at(solver: str, index: int) -> str:
    """The joint's guide transform (the FRAME) holding the shape at `index`."""
    parents = cmds.listRelatives(_joint_at(solver, index), parent=True) or []
    assert_eq(len(parents), 1, f"joint {index} should have one guide parent")
    return parents[0]


def _body_shape_at(solver: str, index: int) -> str:
    """The pmxRigidBodyShape connected to solver.bodyShapes[index]."""
    plugs = (
        cmds.listConnections(f"{solver}.bodyShapes[{index}]", source=True, plugs=True)
        or []
    )
    assert_eq(len(plugs), 1, f"bodyShapes[{index}] should have one source ({plugs})")
    return str(plugs[0]).split(".")[0]


def _linked_body(joint: str, end: str) -> str | None:
    """The body shape connected to ``joint.jointBody{end}`` (None if unconnected)."""
    plugs = (
        cmds.listConnections(f"{joint}.jointBody{end}", source=True, plugs=True) or []
    )
    return str(plugs[0]).split(".")[0] if plugs else None


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
        position=(1.0, 2.0, 3.0),
        rotation=(0.0, 1.5707963267948966, 0.0),  # pi/2
        linearMin=(-2.0, -3.0, -4.0),
        linearMax=(2.0, 3.0, 4.0),
        angularMin=(-0.1, -0.2, -0.3),
        angularMax=(0.1, 0.2, 0.3),
        linearStiffness=(5.0, 6.0, 7.0),
        angularStiffness=(8.0, 9.0, 10.0),
    )
    assert_eq(idx, 0, f"first joint index != 0 ({idx})")
    assert_eq(
        int(cmds.getAttr(f"{solver}.jointShapes", size=True)), 1, "joints count != 1"
    )

    base = _joint_at(solver, 0)
    assert_eq(cmds.getAttr(f"{base}.jointNameLocal"), "LeftChest", "jointNameLocal")
    assert_eq(
        cmds.getAttr(f"{base}.jointNameUniversal"),
        "LeftChest_univ",
        "jointNameUniversal",
    )
    # The bodies are linked by connection to their shapes, not by index.
    assert_eq(_linked_body(base, "A"), _body_shape_at(solver, 0), "jointBodyA link")
    assert_eq(_linked_body(base, "B"), _body_shape_at(solver, 1), "jointBodyB link")
    # New joints are created at the Ranges draw mode (0=Off, 1=Hub, 2=Ranges).
    assert_eq(
        int(cmds.getAttr(f"{base}.drawMode")),
        2,
        "joint drawMode default (0=Off, 1=Hub, 2=Ranges)",
    )
    # The simulation values, the frame included, live in the `joint` compound
    # fed to solver.joints[index].
    data_source = cmds.listConnections(f"{solver}.joints[0]", source=True, plugs=True)
    assert_eq(data_source, [f"{base}.joint"], "joints[0] not fed by the joint compound")
    print("✓ joint appended with names/bodyA/bodyB at the next free index")
    return True


def test_frame_conversion():
    """The MMD frame is converted to Maya space (Z-flip + handedness)."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    cmds.pmxRigidBodyConstraint(
        solver,
        bodyA=0,
        bodyB=1,
        position=(1.0, 2.0, 3.0),
        rotation=(1.5707963267948966, 0.0, 0.0),  # pi/2 rad on X
    )
    base = _joint_guide_at(solver, 0)
    joint = _joint_at(solver, 0)
    # The frame is authored on the node (world space): Z-flip on the
    # translation (z=3 -> -3), and pi/2 rad -> -90 deg on X (handedness flip).
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{joint}.jointRestTranslate")[0], (1.0, 2.0, -3.0)
        ),
        "joint rest translate Z-flip wrong "
        f"{cmds.getAttr(f'{joint}.jointRestTranslate')}",
    )
    rest_rot = cmds.getAttr(f"{joint}.jointRestRotate")[0]
    assert_true(
        approx_equal_tuple(rest_rot, (-90.0, 0.0, 0.0), tolerance=1e-3),
        f"joint rest rotate handedness flip wrong {rest_rot}",
    )
    # The guide is placed at the same frame before the first evaluation (and
    # is driven from there by the solver).
    assert_true(
        approx_equal_tuple(cmds.getAttr(f"{base}.translate")[0], (1.0, 2.0, -3.0)),
        f"joint guide not placed at the rest frame {cmds.getAttr(f'{base}.translate')}",
    )
    print("✓ MMD joint frame on the node (Z-flip + radians->degrees handedness)")
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
        linearMin=(-1.0, -2.0, -3.0),
        linearMax=(4.0, 5.0, 6.0),
        angularMin=(-0.5, -0.2, -0.7),
        angularMax=(0.1, 0.6, 0.7),
        linearStiffness=(4.0, 5.0, 6.0),
        angularStiffness=(7.0, 8.0, 9.0),
    )
    base = _joint_at(solver, 0)
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
    # Angular: X/Y negated + min/max swapped; Z unchanged.  The PMX radians
    # are stored as angle attributes (radians internally, UI unit in scripts).
    assert_true(
        approx_equal_tuple(
            get_angle3_radians(f"{base}.jointAngularMin"), (-0.1, -0.6, -0.7)
        ),
        f"jointAngularMin flip wrong {get_angle3_radians(f'{base}.jointAngularMin')}",
    )
    assert_true(
        approx_equal_tuple(
            get_angle3_radians(f"{base}.jointAngularMax"), (0.5, 0.2, 0.7)
        ),
        f"jointAngularMax flip wrong {get_angle3_radians(f'{base}.jointAngularMax')}",
    )
    # Stiffnesses are magnitudes — invariant under the reflection.
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointLinearStiffness")[0], (4.0, 5.0, 6.0)
        ),
        "jointLinearStiffness not verbatim",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{base}.jointAngularStiffness")[0], (7.0, 8.0, 9.0)
        ),
        "jointAngularStiffness not verbatim",
    )
    print("✓ limits converted through the MMD->Maya reflection; stiffnesses verbatim")
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
        angularMin=(-0.524, -0.175, -0.087),
        angularMax=(0.087, 0.175, 0.087),
    )
    base = _joint_at(solver, 0)
    assert_true(
        approx_equal_tuple(
            get_angle3_radians(f"{base}.jointAngularMin"), (-0.087, -0.175, -0.087)
        ),
        f"chest angularMin wrong {get_angle3_radians(f'{base}.jointAngularMin')}",
    )
    assert_true(
        approx_equal_tuple(
            get_angle3_radians(f"{base}.jointAngularMax"), (0.524, 0.175, 0.087)
        ),
        f"chest angularMax wrong {get_angle3_radians(f'{base}.jointAngularMax')}",
    )
    print("✓ asymmetric chest-joint limits converted (not mirrored)")
    return True


def test_frame_in_bodies_rest_space():
    """The joint frame is stored in the bodies' rest space (world at import).

    The bodies' rest poses are model/rest-space values and the Bullet world
    runs there, so the joint frame is stored as-is.  An offset
    {model}_RigidBodies group is organization only: the guide is placed (and
    then driven) at the frame's world position instead of being moved by the
    group, exactly like a body guide.
    """
    group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)
    cmds.move(10.0, 0.0, 0.0, group)

    cmds.pmxRigidBodyConstraint(
        solver,
        bodyA=0,
        bodyB=1,
        position=(1.0, 2.0, 3.0),  # Z-flip -> frame (1, 2, -3)
    )
    base = _joint_guide_at(solver, 0)
    local_pos = cmds.getAttr(f"{base}.translate")[0]
    assert_true(
        approx_equal_tuple(local_pos, (-9.0, 2.0, -3.0)),
        f"joint guide local position should compensate the group {local_pos}",
    )
    world_pos = cmds.xform(base, query=True, worldSpace=True, translation=True)
    assert_true(
        approx_equal_tuple(world_pos, (1.0, 2.0, -3.0)),
        f"joint frame should stay in the bodies' rest space {world_pos}",
    )
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{_joint_at(solver, 0)}.jointRestTranslate")[0],
            (1.0, 2.0, -3.0),
        ),
        "jointRestTranslate should hold the PMX frame in world space",
    )
    print("✓ joint frame stored in the bodies' rest space; guide placed there")
    return True


def test_self_constraint_rejected():
    """A joint linking a body to ITSELF (bodyA == bodyB) is rejected."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    try:
        cmds.pmxRigidBodyConstraint(solver, bodyA=1, bodyB=1)
        assert_true(False, "bodyA == bodyB was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.jointShapes", size=True) or 0),
        0,
        "rejected self-constraint should not create a joint",
    )
    print("✓ bodyA == bodyB self-constraint rejected")
    return True


def test_auto_increment_indices():
    """Joints auto-append at 0, 1, 2 ... in creation order."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    assert_eq(cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1), 0, "joint 0 idx")
    assert_eq(cmds.pmxRigidBodyConstraint(solver, bodyA=1, bodyB=0), 1, "joint 1 idx")
    assert_eq(cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1), 2, "joint 2 idx")
    assert_eq(int(cmds.getAttr(f"{solver}.jointShapes", size=True)), 3, "joints count")
    print("✓ joints auto-increment 0, 1, 2 ...")
    return True


def test_explicit_index_validation():
    """-index must be the next free index; otherwise it is rejected."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    assert_eq(
        cmds.pmxRigidBodyConstraint(solver, index=0, bodyA=0, bodyB=1),
        0,
        "explicit next-free index ok",
    )
    try:
        cmds.pmxRigidBodyConstraint(solver, index=5, bodyA=0, bodyB=1)
        assert_true(False, "-index 5 (not next free) was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.jointShapes", size=True)),
        1,
        "rejected index should not create a joint",
    )
    print("✓ -index validated against the next free index")
    return True


def test_body_indices_validated():
    """bodyA/bodyB are validated against the node's current body count."""
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)

    for body_a, body_b in ((-1, 0), (0, -1), (2, 0), (0, 2), (99, 0)):
        try:
            cmds.pmxRigidBodyConstraint(solver, bodyA=body_a, bodyB=body_b)
            assert_true(False, f"bodyA/bodyB ({body_a},{body_b}) was accepted")
        except RuntimeError:
            pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.jointShapes", size=True) or 0),
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

    idx = cmds.pmxRigidBodyConstraint(root, bodyA=0, bodyB=1, position=(4.0, 0.0, 0.0))
    assert_eq(idx, 0, "index via model root != 0")
    assert_true(
        approx_equal_tuple(
            cmds.getAttr(f"{_joint_guide_at(solver, 0)}.translate")[0], (4.0, 0.0, 0.0)
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
        cmds.pmxRigidBodyConstraint(solver, edit=True, bodyA=0, bodyB=1)
        assert_true(False, "edit mode was accepted")
    except RuntimeError:
        pass
    assert_eq(
        int(cmds.getAttr(f"{solver}.jointShapes", size=True) or 0),
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
        cmds.pmxRigidBodyConstraint(plain, bodyA=0, bodyB=1)
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
def test_joint_viewport_link_inputs():
    """A created joint exposes its owning solver and both current guide poses.

    The viewport follows joint.solver OUTGOING to jointShapes[]. An incoming
    lookup silently returns no solver, so neither solid nor X-ray links draw.
    """
    from maya.api import OpenMaya as om

    _group, solver, _ja, _jb = _make_physics_scene()
    cmds.pmxRigidBody(solver, name="LinkA", position=(-2, 0, 0), physicsMode="physics")
    cmds.pmxRigidBody(solver, name="LinkB", position=(2, 0, 0), physicsMode="physics")
    cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1)
    joint = _joint_at(solver, 0)

    assert_true(
        cmds.isConnected(f"{solver}.outJointDisplay[0]", f"{joint}.jointDisplay"),
        "joint display configuration must be a solver-driven input",
    )

    selection = om.MSelectionList()
    selection.add(joint)
    joint_fn = om.MFnDependencyNode(selection.getDependNode(0))
    message = joint_fn.findPlug("solver", False)
    assert_eq(
        len(message.connectedTo(True, False)),
        0,
        "solver message has no incoming source",
    )
    destinations = message.connectedTo(False, True)
    assert_eq(len(destinations), 1, "joint has one owning solver destination")
    solver_fn = om.MFnDependencyNode(destinations[0].node())
    assert_eq(solver_fn.name(), solver, "viewport resolves the owning solver")

    poses = solver_fn.findPlug("outGuideTranslate", False)
    assert_eq(poses.evaluateNumElements(), 2, "both guide poses are evaluated")
    positions = []
    for index in (0, 1):
        pose = poses.elementByLogicalIndex(index)
        positions.append(
            tuple(pose.child(axis).asMDistance().asCentimeters() for axis in range(3))
        )
    assert_true(positions[0] != positions[1], "link endpoints must be distinct")
    return True


def test_joint_display_configuration_updates():
    """Display inputs react to repeated paused edits in DG, Serial and Parallel."""
    from maya.api import OpenMaya as om

    original_mode = cmds.evaluationManager(query=True, mode=True)[0]
    try:
        for mode in ("off", "serial", "parallel"):
            _group, solver, _ja, _jb = _make_physics_scene()
            cmds.evaluationManager(mode=mode)
            cmds.connectAttr("time1.outTime", f"{solver}.time")
            for index, x in enumerate((-2.0, 2.0)):
                cmds.pmxRigidBody(
                    solver,
                    name=f"Display{index}",
                    position=(x, 0, 0),
                    physicsMode="physics",
                )
            cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1)
            joint = _joint_at(solver, 0)
            body_a = _body_shape_at(solver, 0)
            body_b = _body_shape_at(solver, 1)
            cmds.currentTime(1)
            cmds.getAttr(f"{solver}.outStep")
            radius = cmds.getAttr(f"{joint}.jointHubRadius")
            assert_true(radius > 0, f"{mode}: nonzero hub radius")
            assert_true(cmds.getAttr(f"{joint}.jointActive"), f"{mode}: active joint")

            for scale in (2.0, 0.5, 1.0):
                cmds.setAttr(f"{solver}.jointDisplayScale", scale)
                assert_true(
                    abs(cmds.getAttr(f"{joint}.jointHubRadius") - radius * scale)
                    < 1e-8,
                    f"{mode}: scale edit updates the display without a time change",
                )
            for enabled in (False, True, False, True):
                cmds.setAttr(f"{body_b}.bodyEnabled", enabled)
                assert_eq(cmds.getAttr(f"{joint}.jointActive"), enabled, mode)
            cmds.connectAttr(f"{body_a}.message", f"{joint}.jointBodyB", force=True)
            assert_true(
                not cmds.getAttr(f"{joint}.jointActive"), "same body is inactive"
            )
            cmds.connectAttr(f"{body_b}.message", f"{joint}.jointBodyB", force=True)
            assert_true(
                cmds.getAttr(f"{joint}.jointActive"), "reconnected joint is active"
            )

            cmds.setAttr(f"{joint}.jointLinearMin", -2, 0, 0, type="double3")
            cmds.setAttr(f"{joint}.jointLinearMax", 3, 0, 0, type="double3")
            selection = om.MSelectionList()
            selection.add(joint)
            bounds = om.MFnDagNode(selection.getDagPath(0)).boundingBox
            assert_true(bounds.min.x < -2 and bounds.max.x > 3, "bounds include limits")
            before = cmds.getAttr(f"{_joint_guide_at(solver, 0)}.translate")[0]
            for frame in range(2, 6):
                cmds.currentTime(frame)
                cmds.getAttr(f"{solver}.outStep")
                assert_eq(cmds.getAttr(f"{joint}.jointHubRadius"), radius, mode)
                assert_true(cmds.getAttr(f"{joint}.jointActive"), mode)
            after = cmds.getAttr(f"{_joint_guide_at(solver, 0)}.translate")[0]
            assert_true(after[1] < before[1], f"{mode}: guide follows the falling body")
            cmds.delete(cmds.listRelatives(body_b, parent=True))
            assert_true(
                not cmds.getAttr(f"{joint}.jointActive"), "deleted body is inactive"
            )
    finally:
        cmds.evaluationManager(mode=original_mode)
    return True


def test_joint_display_does_not_drive_simulation():
    """Display reads leave physics dirty; pose changes leave display inputs clean."""
    original_mode = cmds.evaluationManager(query=True, mode=True)[0]
    try:
        _group, solver, _ja, _jb = _make_physics_scene()
        cmds.evaluationManager(mode="off")
        _add_bodies(solver, 2)
        cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1)
        joint = _joint_at(solver, 0)
        cmds.getAttr(f"{solver}.outStep")
        cmds.getAttr(f"{joint}.jointHubRadius")
        cmds.setAttr(f"{solver}.time", 2)
        assert_true(
            cmds.isDirty(f"{solver}.outStep", datablock=True), "time dirties physics"
        )
        assert_true(
            not cmds.isDirty(f"{joint}.jointHubRadius"),
            "time must not dirty the hub's display connection",
        )
        cmds.setAttr(f"{solver}.jointDisplayScale", 2)
        cmds.getAttr(f"{joint}.jointHubRadius")
        cmds.getAttr(f"{joint}.jointBoundsMin")
        assert_true(
            cmds.isDirty(f"{solver}.outStep", datablock=True),
            "display evaluation must not advance physics",
        )
        cmds.getAttr(f"{solver}.outStep")
        cmds.setAttr(f"{solver}.jointDisplayScale", 3)
        assert_true(
            not cmds.isDirty(f"{solver}.outStep", datablock=True),
            "display scale must not dirty physics",
        )
    finally:
        cmds.evaluationManager(mode=original_mode)
    return True


def test_joint_display_survives_scene_reload():
    """New display connections persist; their derived values recompute after loading."""
    import tempfile
    from pathlib import Path

    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)
    cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1)
    joint = _joint_at(solver, 0)
    cmds.setAttr(f"{solver}.jointDisplayScale", 1.5)
    radius = cmds.getAttr(f"{joint}.jointHubRadius")
    with tempfile.TemporaryDirectory() as directory:
        scene = Path(directory) / "joint_display.ma"
        cmds.file(rename=str(scene))
        cmds.file(save=True, type="mayaAscii", force=True)
        cmds.file(new=True, force=True)
        cmds.file(str(scene), open=True, force=True)
    assert_true(
        cmds.isConnected(f"{solver}.outJointDisplay[0]", f"{joint}.jointDisplay")
    )
    assert_eq(cmds.getAttr(f"{joint}.jointHubRadius"), radius)
    assert_true(cmds.getAttr(f"{joint}.jointActive"))
    cmds.setAttr(f"{solver}.jointDisplayScale", 3)
    assert_eq(cmds.getAttr(f"{joint}.jointHubRadius"), 2 * radius)
    return True


def test_joint_node_naming():
    """Joint transforms follow the naming manager's policy.

    ``-nodeName`` (what the importer passes) is used as given.  Without it the
    command names the joint ``{model}_{joint}_Constraint``: the local name
    when it is ASCII-safe, else the universal name, else ``Joint_{index}``.
    A joint named like a body does not clash with it.
    """
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 2)
    cases = [
        ({"nodeName": "Given_Name"}, "Given_Name"),
        ({"name": "Body0"}, "testSolver_Body0_Constraint"),  # body 0's name
        (
            {"name": "スカート前", "nameUniversal": "Skirt front"},
            "testSolver_Skirt_front_Constraint",
        ),
        ({"name": "髪"}, "testSolver_Joint_3_Constraint"),
    ]
    for index, (flags, expected) in enumerate(cases):
        cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1, **flags)
        assert_eq(_joint_guide_at(solver, index), expected, f"joint {index} name")
    return True


def test_joint_links_follow_bodies():
    """A joint keeps its bodies when an earlier body goes away.

    Joints link body SHAPES by connection, so deleting body 0 neither
    re-targets a joint between bodies 1 and 2 nor breaks it; only the joint
    that named the deleted body loses that end.
    """
    _group, solver, _ja, _jb = _make_physics_scene()
    _add_bodies(solver, 3)
    cmds.pmxRigidBodyConstraint(solver, bodyA=0, bodyB=1)
    cmds.pmxRigidBodyConstraint(solver, bodyA=1, bodyB=2)
    body1, body2 = _body_shape_at(solver, 1), _body_shape_at(solver, 2)
    first, second = _joint_at(solver, 0), _joint_at(solver, 1)

    cmds.delete(cmds.listRelatives(_body_shape_at(solver, 0), parent=True))
    assert_eq(_linked_body(second, "A"), body1, "joint 1 still links body 1")
    assert_eq(_linked_body(second, "B"), body2, "joint 1 still links body 2")
    assert_eq(_linked_body(first, "A"), None, "joint 0 lost its deleted body")
    assert_eq(_linked_body(first, "B"), body1, "joint 0 keeps its other body")
    # The solver still evaluates with the gap (joint 0 is simply inactive).
    cmds.dgeval(f"{solver}.outTranslate")
    return True


def test_joint_gallery_effective_modes():
    """Exercise the real gallery and its readable parameter explanations."""
    import math
    import runpy
    from pathlib import Path

    setup_test_environment()
    sentinel = cmds.createNode("transform", name="preserveGalleryTest")
    api = runpy.run_path(
        str(Path(__file__).resolve().parents[4] / "scripts/maya/joint_glyph_demo.py"),
        run_name="gallery_library",
    )
    gallery = api["build"](show_ui=False)
    assert_eq(len(gallery["cases"]), 18, "gallery cases")
    assert_true(cmds.objExists(sentinel), "gallery preserves scene content")
    assert_true(
        "Rotate X: FREE" in api["describe_joint"](gallery["cases"][10]["joint"]),
        "free rotation explanation",
    )
    joint = gallery["cases"][8]["joint"]
    set_angle3_radians(joint + ".jointAngularMin", (-4, -0.35, -0.9))
    summary = api["describe_joint"](joint)
    assert_true(
        "Rotate X: FREE" in summary and "wrapped" in summary,
        "wrapped angular limit explanation",
    )
    # The explanation follows the viewport's model: a Y range past +/-90
    # degrees is capped, stiffness on a locked axis has no effect, and a
    # joint between two bone-driven bodies (case 18) is inactive.
    set_angle3_radians(joint + ".jointAngularMax", (0.6, 2.0, 0.9))
    assert_true(
        "+/-90" in api["describe_joint"](joint).split("Rotate Y:")[1].split("\n")[0],
        "Y range past 90 degrees is capped",
    )
    welded = gallery["cases"][0]["joint"]
    cmds.setAttr(welded + ".jointLinearStiffness", 5, 0, 0, type="double3")
    assert_true(
        "X=5 (no effect: locked)" in api["describe_joint"](welded),
        "stiffness on a locked axis has no effect",
    )
    assert_true(
        "INACTIVE: both bodies follow their bones"
        in api["describe_joint"](gallery["cases"][17]["joint"]),
        "case 18 is inactive",
    )
    assert_true(
        "INACTIVE" not in api["describe_joint"](gallery["cases"][1]["joint"]),
        "an ordinary case is active",
    )
    # _inactive_reason is a second copy of RigidBodySimulation::jointStatus, so
    # pin its remaining reasons too.
    two_bodies = [gallery["cases"][i]["joint"] for i in (0, 1)]
    solver_of_case0 = gallery["cases"][0]["solver"]
    body_a = _body_shape_at(solver_of_case0, 0)
    body_b = _body_shape_at(solver_of_case0, 1)
    # Point B at body A's shape, so both ends are the same body.
    cmds.connectAttr(f"{body_a}.message", two_bodies[0] + ".jointBodyB", force=True)
    assert_true(
        "INACTIVE: both ends are the same body" in api["describe_joint"](two_bodies[0]),
        "same body is inactive",
    )
    cmds.connectAttr(f"{body_b}.message", two_bodies[0] + ".jointBodyB", force=True)
    cmds.setAttr(body_b + ".bodyEnabled", False)
    assert_true(
        "INACTIVE: a body is disabled" in api["describe_joint"](two_bodies[0]),
        "disabled body is inactive",
    )
    cmds.setAttr(body_b + ".bodyEnabled", True)
    cmds.disconnectAttr(f"{body_b}.message", two_bodies[0] + ".jointBodyB")
    assert_true(
        "INACTIVE: a body is missing" in api["describe_joint"](two_bodies[0]),
        "missing body is inactive",
    )
    cmds.connectAttr(f"{body_b}.message", two_bodies[0] + ".jointBodyB", force=True)
    api["step"](gallery, 30)
    for entry in gallery["cases"]:
        for body in (0, 1):
            pose = cmds.getAttr(f"{entry['solver']}.outGuideTranslate[{body}]")[0]
            assert_true(
                all(math.isfinite(v) for v in pose),
                entry["spec"]["name"] + " finite pose",
            )
    api["reset"](gallery)
    assert_true(cmds.objExists(sentinel), "stepping/reset preserves scene content")
    return True


# Test Registry (static — consumed by run_all_integration_tests.py)
# ─────────────────────────────────────────────────────────────────────────

_TESTS = [
    ("Joint display configuration updates", test_joint_display_configuration_updates),
    (
        "Joint display independent of physics",
        test_joint_display_does_not_drive_simulation,
    ),
    ("Joint display survives scene reload", test_joint_display_survives_scene_reload),
    ("Joint gallery effective modes", test_joint_gallery_effective_modes),
    ("Joint viewport link inputs", test_joint_viewport_link_inputs),
    ("Append joint stores data", test_append_joint_data),
    ("Joint node naming", test_joint_node_naming),
    ("Joint links follow their bodies", test_joint_links_follow_bodies),
    ("Joint frame conversion", test_frame_conversion),
    ("Limits converted through reflection", test_limits_springs_converted),
    (
        "Asymmetric angular limits (real model)",
        test_asymmetric_angular_limits_real_model,
    ),
    ("Frame stored in the bodies' rest space", test_frame_in_bodies_rest_space),
    ("Self-constraint rejected", test_self_constraint_rejected),
    ("Auto-increment joint indices", test_auto_increment_indices),
    ("Explicit index validated", test_explicit_index_validation),
    ("Body indices validated", test_body_indices_validated),
    ("Model root resolution", test_model_root_resolution),
    ("Query/edit rejected", test_query_edit_rejected),
    ("Invalid target rejected", test_invalid_target_rejected),
    ("Missing solver argument rejected", test_missing_solver_argument_rejected),
]
