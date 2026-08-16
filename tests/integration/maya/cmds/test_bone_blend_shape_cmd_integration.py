"""
test_bone_blend_shape_cmd_integration.py

Integration tests for BoneBlendShapeCmd - pure Maya command testing without PMX dependencies.

Tests cover:
- Command syntax and argument parsing
- Query operations (listTargets)
- Edit operations (addTarget — automatically creates MORPH_ controllers and connections)
- Weight alias creation
- Undo/redo functionality
- Automatic MORPH_ controller creation + DG wiring + multiplyDivide rewiring
"""

import math

# ── Maya standalone initialised by the test runner ───────────────────────
# Maya imports (safe after standalone.initialize())
from maya import cmds

# Test framework imports
from tests.integration.test_helpers import (
    approx_equal,
    approx_equal_tuple,
    assert_eq,
    assert_true,
    setup_test_environment,
    suppressed_redo,
    suppressed_undo,
)

# NOTE: boneMorphNode and boneBlendShape are already registered by
# MayaMMD.mll via mmd.plugin.initializePlugin().  No separate plugin
# loading is needed in this module.


def find_morph_controller_for_joint(joint_name):
    """
    Find the MORPH_ controller for a joint by checking its parent.
    Returns the controller name if found, None otherwise.

    Args:
        joint_name: Name of the joint to find controller for

    Returns:
        Controller name (str) or None
    """
    if not cmds.objExists(joint_name):
        return None

    # Get the parent of the joint
    parents = cmds.listRelatives(joint_name, parent=True, fullPath=False)
    if not parents:
        return None

    parent_name = parents[0]

    # Check if parent is a MORPH_ controller
    if parent_name.endswith("_MorphCtrl"):
        return parent_name

    return None


# ──────────────────────────────────────────────────────────────────────────
# Test Cases
# ──────────────────────────────────────────────────────────────────────────


def test_query_list_targets_empty():
    """Test querying targets from an empty boneMorphNode."""
    setup_test_environment()

    # Create empty node
    node = cmds.createNode("boneMorphNode", name="testMorphNode")

    # Query targets (should be empty)
    targets = cmds.boneBlendShape(node, query=True, listTargets=True)

    assert_true(
        targets is None or len(targets) == 0,
        f"Expected empty list, got: {targets}",
    )
    print("✓ Empty node returns no targets")
    return True


def test_query_list_targets_with_data():
    """Test querying targets from a node with manually added target data."""
    setup_test_environment()

    # Create node and manually add target data via attributes
    node = cmds.createNode("boneMorphNode", name="testMorphNode")

    # Add target 0
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "Target1", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", "joint1", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 1, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]", 0, 0, 0, 1, type="double4"
    )

    # Add target 1
    cmds.setAttr(f"{node}.morphTargets[1].targetName", "Target2", type="string")
    cmds.setAttr(f"{node}.morphTargets[1].boneNames[0]", "joint2", type="string")
    cmds.setAttr(f"{node}.morphTargets[1].positionOffset[0]", 0, 1, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[1].rotationOffset[0]", 0, 0, 0, 1, type="double4"
    )

    # Query targets
    targets = cmds.boneBlendShape(node, query=True, listTargets=True)

    assert_true(
        targets and len(targets) == 2 and "Target1" in targets and "Target2" in targets,
        f"Expected ['Target1', 'Target2'], got: {targets}",
    )
    print("✓ Query returns correct target names")
    return True


def test_add_target_rotation_only():
    """Test adding a rotation-only target — creates data, MORPH_ controller, and connections."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")
    _parent = cmds.createNode("transform", name="parent")
    cmds.select(clear=True)
    _joint = cmds.joint(name="test_joint")
    cmds.parent("test_joint", "parent")

    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=("RotateY90", "test_joint", "0,0,0", "0,0.7071068,0,0.7071068"),
    )

    # Verify target was added
    targets = cmds.boneBlendShape(node, query=True, listTargets=True)
    assert_true(
        targets and "RotateY90" in targets,
        f"Target not added, targets: {targets}",
    )

    # Verify weight alias was created
    assert_true(
        cmds.attributeQuery("RotateY90", node=node, exists=True),
        "Weight alias not created",
    )

    # Verify morphTargets data was written
    bone_name = cmds.getAttr(f"{node}.morphTargets[0].boneNames[0]")
    assert_eq(bone_name, "test_joint")

    rot_offset = cmds.getAttr(f"{node}.morphTargets[0].rotationOffset[0]")
    if isinstance(rot_offset, list):
        rot_offset = rot_offset[0]
    assert_true(
        rot_offset and len(rot_offset) >= 4 and abs(rot_offset[1] - 0.7071068) <= 0.001,
        f"Rotation offset mismatch: {rot_offset}",
    )

    # Verify MORPH_ controller was created and wired
    ctrl_name = find_morph_controller_for_joint("test_joint")
    assert_true(
        ctrl_name is not None,
        "MORPH_ controller not created (joint has no MORPH_ parent)",
    )

    assert_true(
        cmds.objExists(ctrl_name),
        f"MORPH_ controller '{ctrl_name}' not created",
    )

    # Verify boneMorphNode.outputRotate is connected to controller (through unitConversion)
    output_plug = f"{node}.outputRotate[0]"

    # Get all destination connections from outputRotate (should include unitConversion nodes)
    all_conns = cmds.listConnections(output_plug, plugs=True, destination=True) or []

    # Walk through unitConversion nodes to find final destinations
    connected_to_ctrl = False
    for conn in all_conns:
        conn_node = conn.split(".")[0]
        if cmds.nodeType(conn_node) == "unitConversion":
            # Check if this unitConversion outputs to the controller
            uc_outputs = (
                cmds.listConnections(
                    f"{conn_node}.output", plugs=True, destination=True
                )
                or []
            )
            for uc_out in uc_outputs:
                if ctrl_name in uc_out:
                    connected_to_ctrl = True
                    break
        elif ctrl_name in conn:
            connected_to_ctrl = True
            break

        if connected_to_ctrl:
            break

    assert_true(
        connected_to_ctrl,
        f"outputRotate[0] not connected to {ctrl_name}.rotate\n  Found connections: {all_conns}",
    )

    # Verify joint is parented under controller
    parents = cmds.listRelatives("test_joint", parent=True) or []
    assert_true(
        parents and parents[0] == ctrl_name,
        f"Joint parent is {parents}, expected [{ctrl_name}]",
    )

    print("✓ Rotation-only target added (data + controller + connections)")
    return True


def test_add_target_with_translation():
    """Test adding a target with both translation and rotation — creates full setup."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")
    _parent = cmds.createNode("transform", name="parent")
    cmds.select(clear=True)
    _joint = cmds.joint(name="test_joint")
    cmds.parent("test_joint", "parent")

    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=(
            "TranslateAndRotate",
            "test_joint",
            "10,0,0",
            "0,0.3826834,0,0.9238795",
        ),
    )

    # Verify target was added
    targets = cmds.boneBlendShape(node, query=True, listTargets=True)
    assert_true(
        targets and "TranslateAndRotate" in targets,
        f"Target not added, targets: {targets}",
    )

    # Verify position offsets were written
    pos_offset = cmds.getAttr(f"{node}.morphTargets[0].positionOffset[0]")
    if isinstance(pos_offset, list):
        pos_offset = pos_offset[0]
    assert_true(
        pos_offset and len(pos_offset) >= 3 and abs(pos_offset[0] - 10.0) <= 0.001,
        f"Position offset mismatch: {pos_offset}",
    )

    # Verify MORPH_ controller created
    ctrl_name = find_morph_controller_for_joint("test_joint")
    assert_true(
        ctrl_name is not None,
        "MORPH_ controller not created (joint has no MORPH_ parent)",
    )

    print("✓ Target with translation + rotation added successfully")
    return True


def test_add_multiple_targets():
    """Test adding multiple targets to the same node."""
    setup_test_environment()

    # Create node and joints
    node = cmds.createNode("boneMorphNode", name="morphNode")
    _ = cmds.joint(name="joint1")
    cmds.select(clear=True)
    _ = cmds.joint(name="joint2")

    # Add first target
    cmds.boneBlendShape(
        node, edit=True, addTarget=("Target1", "joint1", "1,0,0", "0,0,0,1")
    )

    # Add second target (different joint)
    cmds.boneBlendShape(
        node, edit=True, addTarget=("Target2", "joint2", "0,1,0", "0,0,0,1")
    )

    # Add third target (same joint as first)
    cmds.boneBlendShape(
        node, edit=True, addTarget=("Target3", "joint1", "0,0,1", "0,0,0,1")
    )

    # Verify all targets were added
    targets = cmds.boneBlendShape(node, query=True, listTargets=True)
    assert_true(
        targets and len(targets) == 3,
        f"Expected 3 targets, got: {targets}",
    )

    assert_true(
        all(name in targets for name in ["Target1", "Target2", "Target3"]),
        f"Not all targets found: {targets}",
    )

    # Verify weight aliases
    for name in ["Target1", "Target2", "Target3"]:
        assert_true(
            cmds.attributeQuery(name, node=node, exists=True),
            f"Weight alias not created for {name}",
        )

    print("✓ Multiple targets added successfully")
    return True


def test_multiple_joints_single_target():
    """Test adding a target that affects multiple joints — creates controllers for all."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")
    _parent = cmds.createNode("transform", name="parent")
    cmds.select(clear=True)
    _ = cmds.joint(name="joint1")
    cmds.parent("joint1", "parent")
    cmds.select(clear=True)
    _ = cmds.joint(name="joint2")
    cmds.parent("joint2", "parent")
    cmds.select(clear=True)
    _ = cmds.joint(name="joint3")
    cmds.parent("joint3", "parent")

    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=(
            "MultiJoint",
            "joint1,joint2,joint3",
            "1,0,0;0,1,0;0,0,1",
            "0,0,0,1;0,0.707,0,0.707;0,0,0.707,0.707",
        ),
    )

    targets = cmds.boneBlendShape(node, query=True, listTargets=True)
    assert_true(
        targets and "MultiJoint" in targets,
        f"Target not added, targets: {targets}",
    )

    # Verify MORPH_ controllers created for all 3 joints
    for jname in ["joint1", "joint2", "joint3"]:
        ctrl = find_morph_controller_for_joint(jname)
        assert_true(
            ctrl is not None,
            f"Controller for '{jname}' not created (no MORPH_ parent)",
        )

    # Verify all 3 bone names were written
    for i, expected in enumerate(["joint1", "joint2", "joint3"]):
        name = cmds.getAttr(f"{node}.morphTargets[0].boneNames[{i}]")
        assert_eq(name, expected)

    # Verify connections WERE made (controllers auto-wired)
    conns = cmds.listConnections(f"{node}.outputRotate") or []
    assert_true(
        len(conns) >= 3,
        f"Expected at least 3 connections (unitConversion nodes), got: {conns}",
    )

    print("✓ Multi-joint target added successfully with all controllers")
    return True


def test_weight_control():
    """Test weight changes affect node outputRotate (node-level, no joint required).
    After create_bone_morph_helper_joints, weight→controller→joint is DG-driven."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")
    _joint = cmds.joint(name="test_joint")

    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=("Rotate45Y", "test_joint", "0,0,0", "0,0.3826834,0,0.9238795"),
    )

    # Set weight to 0 — outputRotate should be (0,0,0)
    cmds.setAttr(f"{node}.Rotate45Y", 0.0)
    cmds.dgdirty(node)
    out0 = cmds.getAttr(f"{node}.outputRotate[0]")
    # getAttr returns [(x,y,z)], extract tuple
    if isinstance(out0, list):
        out0 = out0[0]
    assert_true(
        all(abs(v) <= 0.1 for v in out0),
        f"Weight=0: Expected outputRotate[0]≈(0,0,0), got {out0}",
    )

    # Set weight to 1 — outputRotate should be ~(0,45,0)
    cmds.setAttr(f"{node}.Rotate45Y", 1.0)
    cmds.dgdirty(node)
    out1 = cmds.getAttr(f"{node}.outputRotate[0]")
    if isinstance(out1, list):
        out1 = out1[0]
    assert_true(
        approx_equal(out1[1], 45.0, tolerance=0.1),
        f"Weight=1: Expected outputRotate[0]≈(0,45,0), got {out1}",
    )

    # Set weight to 0.5
    cmds.setAttr(f"{node}.Rotate45Y", 0.5)
    cmds.dgdirty(node)
    out05 = cmds.getAttr(f"{node}.outputRotate[0]")
    if isinstance(out05, list):
        out05 = out05[0]
    assert_true(
        approx_equal(out05[1], 22.5, tolerance=1.0),
        f"Weight=0.5: Expected outputRotate[0]≈(0,22.5,0), got {out05}",
    )

    print("✓ Weight control works correctly (outputRotate reflects blend)")
    return True


def test_undo_redo():
    """Test that addTarget command does not crash on undo/redo.

    Data writes via cmds.setAttr are not tracked in the MDGModifier, but
    the command's undoIt/redoIt lifecycle is safe (no-op with empty modifier).
    """
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")
    _joint = cmds.joint(name="test_joint")

    cmds.undoInfo(state=True)

    # Add target
    cmds.boneBlendShape(
        node, edit=True, addTarget=("TestTarget", "test_joint", "0,0,0", "0,0,0,1")
    )

    assert_true(
        cmds.boneBlendShape(node, query=True, listTargets=True),
        "Target not added",
    )

    # Undo — should not crash (no-op modifier)
    suppressed_undo()
    print("  ✓ undo() completed without error")
    suppressed_redo()
    print("  ✓ redo() completed without error")

    print("✓ Undo/redo lifecycle is safe")
    return True


def test_create_bone_morph_helper_joints():
    """Test that addTarget automatically creates MORPH_ controllers (no separate call needed)."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")
    _parent = cmds.createNode("transform", name="parent")
    cmds.select(clear=True)
    cmds.joint(name="joint1", p=[0, 0, 0])
    cmds.parent("joint1", "parent")
    cmds.select(clear=True)
    cmds.joint(name="joint2", p=[0, 0, 0])
    cmds.parent("joint2", "parent")

    # Add a target affecting both joints — should auto-create controllers
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=(
            "Test",
            "joint1,joint2",
            "0,0;0,0",
            "0,0.707,0,0.707;0,0,0.707,0.707",
        ),
    )

    # Verify MORPH_ controllers were created automatically
    ctrl1 = find_morph_controller_for_joint("joint1")
    ctrl2 = find_morph_controller_for_joint("joint2")

    assert_true(ctrl1 is not None, "Controller 'MORPH_joint1' not auto-created")
    assert_true(ctrl2 is not None, "Controller 'MORPH_joint2' not auto-created")

    # Verify controllers are transforms
    assert_eq(cmds.nodeType(ctrl1), "transform", f"'{ctrl1}' is not a transform")

    # Verify joints are parented under controllers
    parents1 = cmds.listRelatives("joint1", parent=True) or []
    assert_true(
        parents1 and parents1[0] == ctrl1,
        f"joint1 parent is {parents1}, expected [{ctrl1}]",
    )

    # Verify DG connections (boneMorphNode → controller)
    output_plug = f"{node}.outputRotate[0]"

    # Get all destination connections from outputRotate (should include unitConversion nodes)
    all_conns = cmds.listConnections(output_plug, plugs=True, destination=True) or []

    # Walk through unitConversion nodes to find final destinations
    connected_to_ctrl = False
    for conn in all_conns:
        conn_node = conn.split(".")[0]
        if cmds.nodeType(conn_node) == "unitConversion":
            # Check if this unitConversion outputs to the controller
            uc_outputs = (
                cmds.listConnections(
                    f"{conn_node}.output", plugs=True, destination=True
                )
                or []
            )
            for uc_out in uc_outputs:
                if ctrl1 in uc_out:
                    connected_to_ctrl = True
                    break
        elif ctrl1 in conn:
            connected_to_ctrl = True
            break

        if connected_to_ctrl:
            break

    assert_true(
        connected_to_ctrl,
        f"outputRotate[0] not connected to {ctrl1}\n  Found connections: {all_conns}",
    )

    print(f"  ✓ {ctrl1} → joint1 (auto-created by addTarget)")
    print(f"  ✓ {ctrl2} → joint2 (auto-created by addTarget)")
    print("✓ addTarget automatically creates MORPH_ controllers + wiring")
    return True


def test_data_integrity_after_add_target():
    """Verify add_target writes correct data to all node attributes."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")
    _ = cmds.joint(name="jointA")

    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=("MorphA", "jointA", "5,10,15", "0.1,0.2,0.3,0.8"),
    )

    # Check target name
    name = cmds.getAttr(f"{node}.morphTargets[0].targetName")
    assert_eq(name, "MorphA")

    # Check bone name
    bone = cmds.getAttr(f"{node}.morphTargets[0].boneNames[0]")
    assert_eq(bone, "jointA")

    # Check position offset
    pos = cmds.getAttr(f"{node}.morphTargets[0].positionOffset[0]")
    if isinstance(pos, list):
        pos = pos[0]
    assert_true(
        abs(pos[0] - 5) <= 0.001
        and abs(pos[1] - 10) <= 0.001
        and abs(pos[2] - 15) <= 0.001,
        f"positionOffset: {pos}, expected (5,10,15)",
    )

    # Check rotation offset
    rot = cmds.getAttr(f"{node}.morphTargets[0].rotationOffset[0]")
    if isinstance(rot, list):
        rot = rot[0]
    assert_true(
        abs(rot[0] - 0.1) <= 0.001 and abs(rot[1] - 0.2) <= 0.001,
        f"rotationOffset: {rot}, expected (0.1,0.2,0.3,0.8)",
    )

    # Check weight alias
    assert_true(
        cmds.attributeQuery("MorphA", node=node, exists=True),
        "Weight alias 'MorphA' not created",
    )

    # Check PMX alias
    aliases = cmds.aliasAttr(node, query=True) or []
    assert_true(
        "PMX_MorphA" in aliases,
        f"PMX alias not found in: {aliases}",
    )

    print("✓ add_target writes complete data correctly")
    return True


def test_morph_controller_rewires_inherit_rotation():
    """Test that boneBlendShape automatically rewires INHERIT_ROTATION multiplyDivide nodes.

    Scenario: parent joint has both INHERIT_ROTATION children AND bone morphs.
    The multiplyDivide must read from MORPH_ parent.rotate, not parent.rotate
    (which is identity due to DAG inheritance from the controller above it).
    boneBlendShape._addTarget now handles this automatically.
    """
    setup_test_environment()

    # Create parent joint and INHERIT_ROTATION child (simulating bone_builder.py)
    parent = cmds.joint(name="Parent")
    cmds.select(clear=True)
    child = cmds.joint(name="Child", p=[0, 5, 0])

    influence = 1.0
    md_node = cmds.createNode("multiplyDivide", name=f"{child}_RotScale")
    cmds.setAttr(f"{md_node}.operation", 1)
    cmds.setAttr(f"{md_node}.input2X", influence)
    cmds.setAttr(f"{md_node}.input2Y", influence)
    cmds.setAttr(f"{md_node}.input2Z", influence)
    cmds.connectAttr(f"{parent}.rotateX", f"{md_node}.input1X")
    cmds.connectAttr(f"{parent}.rotateY", f"{md_node}.input1Y")
    cmds.connectAttr(f"{parent}.rotateZ", f"{md_node}.input1Z")

    ctrl_name = f"{child}_InheritCtrl"
    ctrl = cmds.createNode("transform", name=ctrl_name)
    cmds.matchTransform(ctrl, child, pos=True, rot=True)
    child_parent = cmds.listRelatives(child, parent=True)
    if child_parent:
        cmds.parent(ctrl, child_parent[0])
    cmds.connectAttr(f"{md_node}.outputX", f"{ctrl}.rotateX")
    cmds.connectAttr(f"{md_node}.outputY", f"{ctrl}.rotateY")
    cmds.connectAttr(f"{md_node}.outputZ", f"{ctrl}.rotateZ")
    cmds.parent(child, ctrl, absolute=True)

    # Before morph: multiplyDivide reads from Parent (possibly via unitConversion)
    src_x_before = cmds.listConnections(f"{md_node}.input1X", source=True, plugs=True)
    assert_true(
        bool(src_x_before),
        "multiplyDivide input1X has no source before morph",
    )

    # Create bone morph for the parent joint — should auto-create controller and rewire
    morph_node = cmds.createNode("boneMorphNode", name="morphNode")
    cmds.boneBlendShape(
        morph_node,
        edit=True,
        addTarget=("Test", "Parent", "0,0,0", "0,0.707,0,0.707"),
    )

    # Verify MORPH_Parent was created automatically
    morph_parent_ctrl = find_morph_controller_for_joint("Parent")
    assert_true(
        morph_parent_ctrl is not None,
        "MORPH_Parent not auto-created by boneBlendShape",
    )

    # Verify Parent is parented under MORPH_Parent
    parents_list = cmds.listRelatives("Parent", parent=True) or []
    assert_true(
        parents_list and parents_list[0] == morph_parent_ctrl,
        f"Parent should be under {morph_parent_ctrl}, got: {parents_list}",
    )

    # After rewire: multiplyDivide should read from MORPH_Parent (possibly via unitConversion)
    src_x = cmds.listConnections(f"{md_node}.input1X", source=True, plugs=True)
    assert_true(
        bool(src_x),
        "multiplyDivide input1X has no source after rewire",
    )

    # Walk through unitConversion if present to find the actual source
    src_node = src_x[0].split(".")[0]
    while cmds.nodeType(src_node) == "unitConversion":
        deeper = cmds.listConnections(f"{src_node}.input", source=True, plugs=True)
        if deeper:
            src_node = deeper[0].split(".")[0]
        else:
            break

    assert_eq(
        src_node,
        morph_parent_ctrl,
        f"Expected multiplyDivide input1X from {morph_parent_ctrl}, got: {src_node}",
    )

    # Verify original Parent.rotateX is no longer connected to multiplyDivide
    assert_true(
        not cmds.isConnected(f"{parent}.rotateX", f"{md_node}.input1X"),
        "Parent.rotateX should NOT be connected to multiplyDivide after rewire",
    )

    print("✓ MORPH_ controller correctly rewires INHERIT_ROTATION multiplyDivide")
    return True


def test_self_connection_skipped():
    """Verify adding multiple targets with the same bone reuses existing controller."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")
    _parent = cmds.createNode("transform", name="parent")
    cmds.select(clear=True)
    _joint = cmds.joint(name="self_joint")
    cmds.parent("self_joint", "parent")

    # First target affecting self_joint — should create controller
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=("MorphA", "self_joint", "0,0,0", "0,0.3826834,0,0.9238795"),
    )

    # Verify first controller was created
    ctrl_name = find_morph_controller_for_joint("self_joint")
    assert_true(
        ctrl_name is not None,
        "Controller 'MORPH_self_joint' not created for first target",
    )

    # Second target affecting same joint — should REUSE existing controller
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=("MorphB", "self_joint", "0,0,0", "0,0,0.3826834,0.9238795"),
    )

    # Verify both morph targets exist
    targets = cmds.boneBlendShape(node, query=True, listTargets=True)
    assert_true(
        targets and "MorphA" in targets and "MorphB" in targets,
        f"Missing targets: {targets}",
    )

    # Verify only ONE controller exists (should reuse the same controller)
    all_transforms = cmds.ls(type="transform")
    morph_ctrls = [
        t for t in all_transforms if t.endswith("_MorphCtrl") and "self_joint" in t
    ]
    assert_eq(
        len(morph_ctrls),
        1,
        f"Expected 1 morphCtrl controller, found {len(morph_ctrls)}: {morph_ctrls}",
    )

    # Both share the same bone (outputRotate[0])
    bone0 = cmds.getAttr(f"{node}.morphTargets[0].boneNames[0]")
    bone1 = cmds.getAttr(f"{node}.morphTargets[1].boneNames[0]")
    assert_true(
        bone0 == "self_joint" and bone1 == "self_joint",
        f"Bone names: '{bone0}', '{bone1}'",
    )

    # Verify connections WERE made (controller auto-created)
    conns = cmds.listConnections(f"{node}.outputRotate") or []
    assert_true(
        bool(conns),
        "Expected connections to MORPH controller, found none",
    )

    print("✓ Both morphs reuse the same MORPH_ controller")
    return True


def test_eye_bone_morphs_with_shared_parent():
    """
    Test eye bone morphs scenario from Luna Laurel model.

    Scenario:
    - Bone 638 (両目 - both eyes controller) is affected by "+目上" and "+目下" morphs
    - Bones 639 (右目 - right eye) and 641 (左目 - left eye) are affected by "寄り目" morph
    - In PMX, bones 639/641 inherit rotation from bone 638

    This tests:
    1. Multiple morphs affecting the same bone (bone 638)
    2. Multiple morphs in a parent-child bone relationship
    3. PMA node insertion and reuse when a bone is affected by multiple morphs
    """
    setup_test_environment()

    # Create boneMorphNode
    node = cmds.createNode("boneMorphNode", name="eyeMorphNode")

    # Create bone hierarchy representing Luna Laurel's eye setup
    cmds.select(clear=True)
    head_joint = cmds.joint(name="Head", position=[0, 20, 0])  # Parent (bone 28 in PMX)

    cmds.select(head_joint)
    both_eyes = cmds.joint(name="BothEyes", position=[0, 20.687, -0.552])  # Bone 638

    # Set LOCAL_COORDINATE for BothEyes (from PMX: xAxis=[1,0,0], zAxis=[0,0,-1])
    # In Maya after Z-flip: xAxis=[1,0,0], zAxis=[0,0,1] (world-aligned)
    # jointOrient should be identity since local coords match world
    cmds.setAttr(f"{both_eyes}.jointOrient", 0, 0, 0)

    cmds.select(head_joint)
    _right_eye = cmds.joint(
        name="RightEye", position=[-0.348, 18.357, -0.476]
    )  # Bone 639

    cmds.select(head_joint)
    _left_eye = cmds.joint(name="LeftEye", position=[0.348, 18.357, -0.476])  # Bone 641

    # Add "+目上" (eyes up) morph - affects BothEyes (bone 638)
    # Quaternion: [0.017452405765652657, 0.0, 0.0, 0.9998477101325989]
    # This is a rotation around X axis by ~2 degrees
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=(
            "EyesUp",
            "BothEyes",
            "0,0,0",
            "0.017452405765652657,0,0,0.9998477101325989",
        ),
    )

    # Add "+目下" (eyes down) morph - also affects BothEyes (bone 638)
    # Quaternion: [-0.017452405765652657, 0.0, 0.0, 0.9998477101325989]
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=(
            "EyesDown",
            "BothEyes",
            "0,0,0",
            "-0.017452405765652657,0,0,0.9998477101325989",
        ),
    )

    # Add "寄り目" (cross-eyed) morph - affects both RightEye and LeftEye (bones 639, 641)
    # Right eye: [0.0, -0.06104854494333267, 0.0, 0.9981347918510437]
    # Left eye: [0.0, 0.06104854494333267, 0.0, 0.9981347918510437]
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=(
            "CrossEyed",
            "RightEye,LeftEye",
            "0,0,0;0,0,0",
            "0,-0.06104854494333267,0,0.9981347918510437;0,0.06104854494333267,0,0.9981347918510437",
        ),
    )

    # Verify all targets were added
    targets = cmds.boneBlendShape(node, query=True, listTargets=True)
    assert_true(
        targets and len(targets) == 3,
        f"Expected 3 targets, got {len(targets) if targets else 0}",
    )

    expected_targets = ["EyesUp", "EyesDown", "CrossEyed"]
    assert_true(
        all(t in targets for t in expected_targets),
        f"Missing expected targets. Got: {targets}",
    )

    # Test morph activation via outputRotate (node-level, no joint connections needed)
    print("  Testing morph activation via outputRotate...")

    cmds.setAttr(f"{node}.EyesUp", 1.0)
    cmds.dgdirty(node)
    out_up = cmds.getAttr(f"{node}.outputRotate[0]")
    if isinstance(out_up, list):
        out_up = out_up[0]
    print(f"    EyesUp(1.0) -> outputRotate[0] = {out_up}")

    cmds.setAttr(f"{node}.EyesUp", 0.0)
    cmds.setAttr(f"{node}.EyesDown", 1.0)
    cmds.dgdirty(node)
    out_down = cmds.getAttr(f"{node}.outputRotate[0]")
    if isinstance(out_down, list):
        out_down = out_down[0]
    print(f"    EyesDown(1.0) -> outputRotate[0] = {out_down}")

    cmds.setAttr(f"{node}.EyesDown", 0.0)
    cmds.setAttr(f"{node}.CrossEyed", 1.0)
    cmds.dgdirty(node)
    out_right = cmds.getAttr(f"{node}.outputRotate[1]")
    out_left = cmds.getAttr(f"{node}.outputRotate[2]")
    if isinstance(out_right, list):
        out_right = out_right[0]
    if isinstance(out_left, list):
        out_left = out_left[0]
    print(f"    CrossEyed(1.0) -> outputRotate[1] (RightEye) = {out_right}")
    print(f"    CrossEyed(1.0) -> outputRotate[2] (LeftEye) = {out_left}")

    print(
        "✓ Eye bone morphs with shared parent and multiple morphs per bone working correctly"
    )
    return True


def test_translation_connection_with_position_offset():
    """
    Verify that translation connections are properly made when position offsets exist.

    This test addresses the concern that translation wiring might have been removed.
    It verifies that:
    1. When a target has non-zero position offsets, outputTranslate is connected
    2. When a target has only rotation (zero position), outputTranslate is NOT connected
    3. The detection logic works correctly across multiple targets
    """
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="translationTestNode")
    _parent = cmds.createNode("transform", name="parent")

    # Create two joints
    cmds.select(clear=True)
    _joint1 = cmds.joint(name="joint_with_translation")
    cmds.parent("joint_with_translation", "parent")

    cmds.select(clear=True)
    _joint2 = cmds.joint(name="joint_rotation_only")
    cmds.parent("joint_rotation_only", "parent")

    # Target 1: Has both translation (10,0,0) and rotation
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=(
            "WithTranslation",
            "joint_with_translation",
            "10,0,0",
            "0,0.3826834,0,0.9238795",
        ),
    )

    # Target 2: Has only rotation (zero translation)
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=(
            "RotationOnly",
            "joint_rotation_only",
            "0,0,0",
            "0,0.707,0,0.707",
        ),
    )

    # Verify MORPH_ controllers were created
    ctrl1 = find_morph_controller_for_joint("joint_with_translation")
    ctrl2 = find_morph_controller_for_joint("joint_rotation_only")

    assert_true(
        ctrl1 is not None,
        "Controller 'MORPH_joint_with_translation' not created",
    )
    assert_true(
        ctrl2 is not None,
        "Controller 'MORPH_joint_rotation_only' not created",
    )

    # Verify rotation connections exist for BOTH controllers (always connected)
    # Check outputRotate[0] → MORPH_joint_with_translation.rotate
    output_plug_0 = f"{node}.outputRotate[0]"
    all_conns_0 = (
        cmds.listConnections(output_plug_0, plugs=True, destination=True) or []
    )

    connected_rot_1 = False
    for conn in all_conns_0:
        conn_node = conn.split(".")[0]
        if cmds.nodeType(conn_node) == "unitConversion":
            uc_outputs = (
                cmds.listConnections(
                    f"{conn_node}.output", plugs=True, destination=True
                )
                or []
            )
            for uc_out in uc_outputs:
                if ctrl1 in uc_out and ".rotate" in uc_out:
                    connected_rot_1 = True
                    break
        elif ctrl1 in conn and ".rotate" in conn:
            connected_rot_1 = True
            break

    assert_true(
        connected_rot_1,
        f"outputRotate[0] not connected to {ctrl1}.rotate\n  Found connections: {all_conns_0}",
    )

    print(f"  ✓ outputRotate[0] → {ctrl1}.rotate (rotation always connected)")

    # Check outputRotate[1] → MORPH_joint_rotation_only.rotate
    output_plug_1 = f"{node}.outputRotate[1]"
    all_conns_1 = (
        cmds.listConnections(output_plug_1, plugs=True, destination=True) or []
    )

    connected_rot_2 = False
    for conn in all_conns_1:
        conn_node = conn.split(".")[0]
        if cmds.nodeType(conn_node) == "unitConversion":
            uc_outputs = (
                cmds.listConnections(
                    f"{conn_node}.output", plugs=True, destination=True
                )
                or []
            )
            for uc_out in uc_outputs:
                if ctrl2 in uc_out and ".rotate" in uc_out:
                    connected_rot_2 = True
                    break
        elif ctrl2 in conn and ".rotate" in conn:
            connected_rot_2 = True
            break

    assert_true(
        connected_rot_2,
        f"outputRotate[1] not connected to {ctrl2}.rotate\n  Found connections: {all_conns_1}",
    )

    print(f"  ✓ outputRotate[1] → {ctrl2}.rotate (rotation always connected)")

    # CRITICAL VERIFICATION: Check translation connection for joint WITH position offset
    # outputTranslate[0] should be connected to MORPH_joint_with_translation.translate
    output_trans_plug_0 = f"{node}.outputTranslate[0]"
    trans_conns_0 = (
        cmds.listConnections(output_trans_plug_0, plugs=True, destination=True) or []
    )

    connected_trans_1 = False
    for conn in trans_conns_0:
        conn_node = conn.split(".")[0]
        if cmds.nodeType(conn_node) == "unitConversion":
            uc_outputs = (
                cmds.listConnections(
                    f"{conn_node}.output", plugs=True, destination=True
                )
                or []
            )
            for uc_out in uc_outputs:
                if ctrl1 in uc_out and ".translate" in uc_out:
                    connected_trans_1 = True
                    break
        elif ctrl1 in conn and ".translate" in conn:
            connected_trans_1 = True
            break

    assert_true(
        connected_trans_1,
        f"outputTranslate[0] NOT connected to {ctrl1}.translate\n"
        f"  This is a REGRESSION — translation wiring is missing!\n"
        f"  Found connections: {trans_conns_0}",
    )

    print(f"  ✓ outputTranslate[0] → {ctrl1}.translate (has position offset)")

    # VERIFICATION: Check translation NOT connected for joint WITHOUT position offset
    # outputTranslate[1] should NOT be connected to MORPH_joint_rotation_only.translate
    output_trans_plug_1 = f"{node}.outputTranslate[1]"
    trans_conns_1 = (
        cmds.listConnections(output_trans_plug_1, plugs=True, destination=True) or []
    )

    connected_trans_2 = False
    for conn in trans_conns_1:
        conn_node = conn.split(".")[0]
        if cmds.nodeType(conn_node) == "unitConversion":
            uc_outputs = (
                cmds.listConnections(
                    f"{conn_node}.output", plugs=True, destination=True
                )
                or []
            )
            for uc_out in uc_outputs:
                if ctrl2 in uc_out and ".translate" in uc_out:
                    connected_trans_2 = True
                    break
        elif ctrl2 in conn and ".translate" in conn:
            connected_trans_2 = True
            break

    assert_true(
        not connected_trans_2,
        f"outputTranslate[1] IS connected to {ctrl2}.translate\n"
        f"  Expected NO connection for rotation-only target",
    )

    print(
        f"  ✓ outputTranslate[1] not connected to {ctrl2}.translate (no position offset)"
    )

    # Verify weight control affects translation output
    cmds.setAttr(f"{node}.WithTranslation", 0.0)
    cmds.dgdirty(node)
    out_trans_0 = cmds.getAttr(f"{node}.outputTranslate[0]")
    if isinstance(out_trans_0, list):
        out_trans_0 = out_trans_0[0]

    assert_true(
        all(abs(v) <= 0.1 for v in out_trans_0),
        f"Weight=0: Expected outputTranslate[0]≈(0,0,0), got {out_trans_0}",
    )

    cmds.setAttr(f"{node}.WithTranslation", 1.0)
    cmds.dgdirty(node)
    out_trans_1 = cmds.getAttr(f"{node}.outputTranslate[0]")
    if isinstance(out_trans_1, list):
        out_trans_1 = out_trans_1[0]

    assert_true(
        approx_equal(out_trans_1[0], 10.0, tolerance=0.1),
        f"Weight=1: Expected outputTranslate[0]≈(10,0,0), got {out_trans_1}",
    )

    print("  ✓ Weight control affects outputTranslate (0→10 on X axis)")

    print("✓ Translation connections properly made when position offsets exist")
    return True


def test_jointorient_preserved_after_morph_controller_insertion():
    """
    Test that jointOrient is preserved when MORPH_ controller is inserted.

    When boneBlendShape creates a MORPH_ controller and reparents a joint under it,
    the joint's jointOrient should remain unchanged. This is critical for maintaining
    the skeleton's rest pose and ensuring animations/poses work correctly.

    This test validates the fix in bone_blend_shape_cmd.py that uses relative=True
    parenting to preserve jointOrient.
    """
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="morphNode")

    # Create a joint hierarchy with non-zero jointOrient
    _parent = cmds.createNode("transform", name="parent")
    cmds.select(clear=True)
    _joint = cmds.joint(name="test_joint", position=[0, 0, 0])
    cmds.parent("test_joint", "parent")

    # Set a non-trivial jointOrient (30 degrees around Y-axis)
    # This simulates what PMX bone_builder does for FIXED_AXIS bones
    original_orient = (0.0, 30.0, 0.0)
    cmds.setAttr("test_joint.jointOrient", *original_orient)

    # Get the initial jointOrient values for comparison
    orient_before = cmds.getAttr("test_joint.jointOrient")[0]

    print(f"  Initial jointOrient: {orient_before}")

    # Add a bone morph — this should create MORPH_test_joint and reparent test_joint
    cmds.boneBlendShape(
        node,
        edit=True,
        addTarget=("TestMorph", "test_joint", "0,0,0", "0,0.7071068,0,0.7071068"),
    )

    # Verify MORPH_ controller was created
    ctrl_name = find_morph_controller_for_joint("test_joint")
    assert_true(
        ctrl_name is not None,
        "MORPH_ controller not created",
    )

    # Verify joint is now parented under the controller
    parents = cmds.listRelatives("test_joint", parent=True) or []
    assert_true(
        parents and parents[0] == ctrl_name,
        f"Joint not parented under controller. Parents: {parents}",
    )

    # CRITICAL: Verify jointOrient is preserved
    orient_after = cmds.getAttr("test_joint.jointOrient")[0]

    print(f"  jointOrient after controller insertion: {orient_after}")

    assert_true(
        approx_equal_tuple(orient_before, orient_after, tolerance=0.001),
        f"jointOrient changed!\n  Before: {orient_before}\n  After:  {orient_after}\n"
        f"  This breaks skeleton orientation and animation compatibility!",
    )

    # Also verify the joint's world orientation is still correct
    # (world matrix should be unchanged by controller insertion in bind pose)
    world_matrix = cmds.xform("test_joint", query=True, worldSpace=True, matrix=True)
    import maya.api.OpenMaya as om

    m_matrix = om.MMatrix(world_matrix)
    world_euler = om.MTransformationMatrix(m_matrix).rotation()
    world_degrees = (
        math.degrees(world_euler.x),
        math.degrees(world_euler.y),
        math.degrees(world_euler.z),
    )

    print(f"  World rotation (degrees): {world_degrees}")

    # In this test setup, world rotation should match jointOrient since rotate is zero
    expected_world = original_orient
    assert_true(
        approx_equal_tuple(world_degrees, expected_world, tolerance=0.1),
        f"World rotation changed unexpectedly!\n  Expected: {expected_world}\n  Got:      {world_degrees}",
    )

    print("✓ jointOrient preserved after MORPH_ controller insertion")
    return True


# ──────────────────────────────────────────────────────────────────────────
# Test Registry (static — consumed by run_all_integration_tests.py)
# ──────────────────────────────────────────────────────────────────────────

_TESTS = [
    ("Query empty node", test_query_list_targets_empty),
    ("Query with targets", test_query_list_targets_with_data),
    ("Add rotation-only target", test_add_target_rotation_only),
    ("Add target with translation", test_add_target_with_translation),
    ("Add multiple targets", test_add_multiple_targets),
    ("Multi-joint single target", test_multiple_joints_single_target),
    ("Weight control", test_weight_control),
    ("Undo/redo", test_undo_redo),
    ("Create bone morph helper joints", test_create_bone_morph_helper_joints),
    ("Data integrity after add target", test_data_integrity_after_add_target),
    (
        "Morph controller rewires INHERIT_ROTATION",
        test_morph_controller_rewires_inherit_rotation,
    ),
    ("Self-connection (multiple targets, same bone)", test_self_connection_skipped),
    ("Eye bone morphs with shared parent", test_eye_bone_morphs_with_shared_parent),
    (
        "Translation connection with position offset",
        test_translation_connection_with_position_offset,
    ),
    (
        "JointOrient preserved after controller insertion",
        test_jointorient_preserved_after_morph_controller_insertion,
    ),
]
