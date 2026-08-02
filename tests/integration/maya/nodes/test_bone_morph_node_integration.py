"""
test_bone_morph_node_integration.py

Integration tests for BoneMorphNode - pure Maya node testing without PMX dependencies.

Tests cover:
- Node creation and attribute initialization
- Direct attribute manipulation (no command dependencies)
- Weight-driven rotation blending (quaternion SLERP)
- Weight-driven translation blending (linear interpolation)
- Multi-target blending behavior
- Output array computation
"""

# ── Maya standalone initialised by the test runner ───────────────────────

# Maya imports (safe after standalone.initialize())
from maya import cmds  # noqa: E402

# Test framework imports
from tests.integration.test_helpers import (  # noqa: E402
    setup_test_environment,
    assert_true,
    assert_eq,
    skip_test,
    approx_equal,
    approx_equal_tuple,
)

# NOTE: boneMorphNode is already registered by MayaMMD.mll via
# mmd.plugin.initializePlugin().  No separate plugin loading is needed.


# ──────────────────────────────────────────────────────────────────────────
# Test Cases
# ──────────────────────────────────────────────────────────────────────────


def test_node_creation():
    """Test that boneMorphNode can be created and has expected attributes."""
    setup_test_environment()

    # Create node
    node = cmds.createNode("boneMorphNode", name="testBoneMorph")

    # Check node exists
    assert_true(cmds.objExists(node), "Node creation failed")

    # Check expected attributes exist
    required_attrs = ["weight", "morphTargets", "outputRotate", "outputTranslate"]

    for attr in required_attrs:
        assert_true(
            cmds.attributeQuery(attr, node=node, exists=True),
            f"Missing attribute: {attr}",
        )

    # Check that morphTargets has expected children
    compound_children = ["targetName", "boneNames", "positionOffset", "rotationOffset"]
    for child in compound_children:
        assert_true(
            cmds.attributeQuery(child, node=node, exists=True),
            f"Missing compound child: {child}",
        )

    print("✓ Node created with all required attributes")
    return True


def test_single_rotation_target():
    """Test single rotation morph target blending via direct node output reading."""
    setup_test_environment()

    # Create a test joint (prevents rotation order warnings)
    cmds.joint(name="test_joint")

    # Create bone morph node
    node = cmds.createNode("boneMorphNode", name="boneMorph")

    # Manually add a rotation target via attributes (no command)
    # Target: rotate test_joint by 45 degrees around Y axis
    # Quaternion for 45° Y rotation: (0, 0.3826834, 0, 0.9238795)
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "RotateY45", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", "test_joint", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 0, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]",
        0,
        0.3826834,
        0,
        0.9238795,
        type="double4",
    )

    # Create weight attribute element
    cmds.setAttr(f"{node}.weight[0]", 0.0)

    # Test weight = 0 (should output 0,0,0)
    cmds.setAttr(f"{node}.weight[0]", 0.0)
    output = cmds.getAttr(f"{node}.outputRotate[0]")
    assert_true(
        approx_equal_tuple(output[0], (0, 0, 0), tolerance=0.1),
        f"Weight=0: Expected (0,0,0), got {output[0]}",
    )

    # Test weight = 1 (full rotation - should be 45° on Y)
    cmds.setAttr(f"{node}.weight[0]", 1.0)
    output = cmds.getAttr(f"{node}.outputRotate[0]")
    assert_true(
        approx_equal(output[0][1], 45.0, tolerance=0.1),
        f"Weight=1: Expected Y≈45°, got {output[0]}",
    )

    # Test weight = 0.5 (half rotation - should be ~22.5° on Y due to SLERP)
    cmds.setAttr(f"{node}.weight[0]", 0.5)
    output = cmds.getAttr(f"{node}.outputRotate[0]")
    assert_true(
        approx_equal(output[0][1], 22.5, tolerance=1.0),
        f"Weight=0.5: Expected Y≈22.5°, got {output[0]}",
    )

    print("✓ Single rotation target blending works correctly")
    return True


def test_single_translation_target():
    """Test single translation morph target blending via direct output reading."""
    setup_test_environment()

    # Create bone morph node
    node = cmds.createNode("boneMorphNode", name="boneMorph")

    # Manually add a translation target via attributes
    # Target: move test_joint 10 units on X axis
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "TranslateX10", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", "test_joint", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 10, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]", 0, 0, 0, 1, type="double4"
    )

    # Create weight attribute element
    cmds.setAttr(f"{node}.weight[0]", 0.0)

    # Test weight = 0 (no translation)
    cmds.setAttr(f"{node}.weight[0]", 0.0)
    output = cmds.getAttr(f"{node}.outputTranslate[0]")
    assert_true(
        approx_equal_tuple(output[0], (0, 0, 0), tolerance=0.01),
        f"Weight=0: Expected (0,0,0), got {output[0]}",
    )

    # Test weight = 1 (full translation - should be 10 on X)
    cmds.setAttr(f"{node}.weight[0]", 1.0)
    output = cmds.getAttr(f"{node}.outputTranslate[0]")
    assert_true(
        approx_equal(output[0][0], 10.0, tolerance=0.01),
        f"Weight=1: Expected X≈10, got {output[0]}",
    )

    # Test weight = 0.5 (half translation - should be 5 on X, linear)
    cmds.setAttr(f"{node}.weight[0]", 0.5)
    output = cmds.getAttr(f"{node}.outputTranslate[0]")
    assert_true(
        approx_equal(output[0][0], 5.0, tolerance=0.01),
        f"Weight=0.5: Expected X≈5, got {output[0]}",
    )

    print("✓ Single translation target blending works correctly")
    return True


def test_multiple_targets_on_same_joint():
    """Test multiple morph targets affecting the same joint (additive blending)."""
    setup_test_environment()

    # Create a test joint (prevents rotation order warnings)
    cmds.joint(name="test_joint")

    # Create bone morph node
    node = cmds.createNode("boneMorphNode", name="boneMorph")

    # Add two rotation targets for the same joint
    # Target 1: Rotate 30° around Y
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "RotateY30", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", "test_joint", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 0, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]",
        0,
        0.2588190,
        0,
        0.9659258,
        type="double4",
    )

    # Target 2: Rotate 15° around X
    cmds.setAttr(f"{node}.morphTargets[1].targetName", "RotateX15", type="string")
    cmds.setAttr(f"{node}.morphTargets[1].boneNames[0]", "test_joint", type="string")
    cmds.setAttr(f"{node}.morphTargets[1].positionOffset[0]", 0, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[1].rotationOffset[0]",
        0.1305262,
        0,
        0,
        0.9914449,
        type="double4",
    )

    # Create weights
    cmds.setAttr(f"{node}.weight[0]", 1.0)  # RotateY30
    cmds.setAttr(f"{node}.weight[1]", 1.0)  # RotateX15

    # Get output rotation (both targets should be composited)
    output = cmds.getAttr(f"{node}.outputRotate[0]")
    rot = output[0]

    # When composing rotations via quaternion multiplication, the result won't be exactly
    # the sum of Euler angles. We just verify that both axes have non-zero rotation.
    assert_true(
        approx_equal(rot[0], 15.0, tolerance=3.0)
        and approx_equal(rot[1], 30.0, tolerance=3.0),
        f"Multiple targets: Expected X≈15°, Y≈30°, got {rot}",
    )

    print("✓ Multiple targets blend additively")
    return True


def test_multiple_joints():
    """Test morph affecting multiple joints simultaneously."""
    setup_test_environment()

    # Create test joints (prevents rotation order warnings)
    cmds.joint(name="joint1")
    cmds.joint(name="joint2")

    # Create bone morph node
    node = cmds.createNode("boneMorphNode", name="boneMorph")

    # Add target affecting both joints
    # joint1: 45° Y rotation, joint2: 45° Z rotation
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "BothJoints", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", "joint1", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[1]", "joint2", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 0, 0, 0, type="double3")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[1]", 0, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]",
        0,
        0.3826834,
        0,
        0.9238795,
        type="double4",
    )  # 45° Y
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[1]",
        0,
        0,
        0.3826834,
        0.9238795,
        type="double4",
    )  # 45° Z

    # Create weight
    cmds.setAttr(f"{node}.weight[0]", 1.0)

    # Check outputs - should have 2 output slots (one per unique joint)
    output0 = cmds.getAttr(f"{node}.outputRotate[0]")
    output1 = cmds.getAttr(f"{node}.outputRotate[1]")

    # First output should be joint1's 45° Y rotation
    assert_true(
        approx_equal(output0[0][1], 45.0, tolerance=0.1),
        f"Output[0] Y rotation: Expected 45°, got {output0[0]}",
    )

    # Second output should be joint2's 45° Z rotation
    assert_true(
        approx_equal(output1[0][2], 45.0, tolerance=0.1),
        f"Output[1] Z rotation: Expected 45°, got {output1[0]}",
    )

    print("✓ Multiple joints affected correctly")
    return True


def test_attribute_query():
    """Test querying morph target data from attributes."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="boneMorph")

    # Add multiple targets via attributes
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "Target1", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", "joint1", type="string")

    cmds.setAttr(f"{node}.morphTargets[1].targetName", "Target2", type="string")
    cmds.setAttr(f"{node}.morphTargets[1].boneNames[0]", "joint2", type="string")

    cmds.setAttr(f"{node}.morphTargets[2].targetName", "Target3", type="string")
    cmds.setAttr(f"{node}.morphTargets[2].boneNames[0]", "joint3", type="string")

    # Query morphTargets array size
    size = cmds.getAttr(f"{node}.morphTargets", size=True)
    assert_eq(size, 3, f"morphTargets size: Expected 3, got {size}")

    # Query specific target name
    target_name = cmds.getAttr(f"{node}.morphTargets[1].targetName")
    assert_eq(
        target_name,
        "Target2",
        f"Target name query: Expected 'Target2', got '{target_name}'",
    )

    # Query bone name
    bone_name = cmds.getAttr(f"{node}.morphTargets[1].boneNames[0]")
    assert_eq(
        bone_name,
        "joint2",
        f"Bone name query: Expected 'joint2', got '{bone_name}'",
    )

    print("✓ Attribute query functionality works")
    return True


def test_translation_root_joint_no_parent_correction():
    """Translation offset on a root joint (no parent joint) passes through unchanged.

    _get_parent_joint_name returns None for a joint whose DAG parent is not a
    joint (e.g. the world transform), so no inverse-orientation rotation should
    be applied to the position offset.
    """
    setup_test_environment()

    # Root joint — no parent joint in the scene
    cmds.joint(name="root_joint")

    node = cmds.createNode("boneMorphNode", name="boneMorph")
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "MoveX", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", "root_joint", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 10, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]", 0, 0, 0, 1, type="double4"
    )
    cmds.setAttr(f"{node}.weight[0]", 1.0)

    output = cmds.getAttr(f"{node}.outputTranslate[0]")
    assert_true(
        approx_equal_tuple(output[0], (10, 0, 0), tolerance=0.01),
        f"Root joint translation not preserved: expected (10,0,0), got {output[0]}",
    )

    print("✓ Root joint translation passes through without parent-space correction")
    return True


def test_translation_parent_space_correction():
    """Translation offset on a child joint is rotated by the inverse of the parent's
    world-rest orientation (new behaviour added to _compute_blended_translation_for_index).

    Setup:
      - parent_joint  at origin, jointOrientY = 90°
      - child_joint   parented under parent_joint, no jointOrient

    A world-space offset of (10, 0, 0) must be rotated by Q(Y, -90°) to produce
    the local-space value (0, 0, 10) that should be added to child_joint.translate.

    Math:  R_y(-90°) * [10,0,0]ᵀ  =  [0, 0, 10]ᵀ
    """
    setup_test_environment()

    # Build parent-child joint chain
    parent = cmds.joint(name="parent_joint")
    # Set a known world-rest orientation on the parent: 90° around Y
    cmds.setAttr(f"{parent}.jointOrientX", 0)
    cmds.setAttr(f"{parent}.jointOrientY", 90)
    cmds.setAttr(f"{parent}.jointOrientZ", 0)

    # Create child parented under parent
    cmds.select(parent)
    child = cmds.joint(name="child_joint")
    # Child has identity jointOrient (default)

    node = cmds.createNode("boneMorphNode", name="boneMorph")
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "MoveChildX", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", child, type="string")
    # World-space offset: 10 units along +X
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 10, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]", 0, 0, 0, 1, type="double4"
    )
    cmds.setAttr(f"{node}.weight[0]", 1.0)

    output = cmds.getAttr(f"{node}.outputTranslate[0]")
    # R_y(-90°) * (10,0,0) = (0,0,10)
    assert_true(
        approx_equal_tuple(output[0], (0, 0, 10), tolerance=0.1),
        f"Parent-space correction wrong: expected (0,0,10), got {output[0]}",
    )

    print("✓ Translation correctly transformed into parent's local space")
    return True


def test_rotation_child_joint_non_identity_parent_orient():
    """Rotation offset on a child joint under a parent with non-identity jointOrient
    is applied in the child's local space without parent-frame correction.

    This test locks in the PR's architecture assumption: morph rotation offsets are
    already expressed in local space and must NOT have the parent's orientation baked
    out of them.  If the bone morph node ever reintroduces a world-to-local rotation
    transform for rotation offsets, this test will catch the regression.

    Setup:
      - parent_joint  at origin, jointOrientY = 90°
      - child_joint   parented under parent_joint, no jointOrient

    Apply a 45° Y rotation offset to child_joint.
    Expected output: (0, 45, 0) degrees in child's local space — the parent
    orientation does NOT affect what the node outputs for rotation.

    If the old world-space conversion were active, the output would be
    rotated by the inverse of the parent's world orient (−90° Y), giving
    a different result rather than (0, 45, 0).
    """
    setup_test_environment()

    # Build parent-child joint chain
    parent = cmds.joint(name="parent_joint")
    cmds.setAttr(f"{parent}.jointOrientX", 0)
    cmds.setAttr(f"{parent}.jointOrientY", 90)
    cmds.setAttr(f"{parent}.jointOrientZ", 0)

    cmds.select(parent)
    child = cmds.joint(name="child_joint")
    # Child has identity jointOrient (enforced by new PR architecture)

    node = cmds.createNode("boneMorphNode", name="boneMorph")
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "RotateChildY45", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", child, type="string")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 0, 0, 0, type="double3")
    # Quaternion for 45° Y rotation: (0, sin(22.5°), 0, cos(22.5°))
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]",
        0,
        0.3826834,
        0,
        0.9238795,
        type="double4",
    )
    cmds.setAttr(f"{node}.weight[0]", 1.0)

    output = cmds.getAttr(f"{node}.outputRotate[0]")
    rot = output[0]  # (rx, ry, rz) in degrees

    # The rotation output must be the local-space equivalent of the offset
    # quaternion — 45° around Y — regardless of the parent's 90° Y orient.
    assert_true(
        approx_equal(rot[1], 45.0, tolerance=0.5),
        f"Child rotation Y: expected ≈45°, got {rot[1]:.4f}° "
        f"(full output: {rot}) — parent orient may be incorrectly applied",
    )

    assert_true(
        approx_equal(rot[0], 0.0, tolerance=0.5)
        and approx_equal(rot[2], 0.0, tolerance=0.5),
        f"Child rotation X/Z should be ~0, got X={rot[0]:.4f}°, Z={rot[2]:.4f}°",
    )

    print(
        "✓ Rotation on child joint is in local space — parent orient does not affect output"
    )
    return True


def test_zero_vs_nonzero_translation():
    """Test that zero translation outputs (0,0,0) and nonzero translation outputs correctly."""
    setup_test_environment()

    node = cmds.createNode("boneMorphNode", name="boneMorph")

    # Target with zero translation
    cmds.setAttr(f"{node}.morphTargets[0].targetName", "ZeroTranslation", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].boneNames[0]", "joint1", type="string")
    cmds.setAttr(f"{node}.morphTargets[0].positionOffset[0]", 0, 0, 0, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[0].rotationOffset[0]", 0, 0, 0, 1, type="double4"
    )
    cmds.setAttr(f"{node}.weight[0]", 1.0)

    # Check output is zero
    output = cmds.getAttr(f"{node}.outputTranslate[0]")
    assert_true(
        approx_equal_tuple(output[0], (0, 0, 0), tolerance=0.001),
        f"Zero translation: Expected (0,0,0), got {output[0]}",
    )

    # Target with non-zero translation
    cmds.setAttr(
        f"{node}.morphTargets[1].targetName", "NonZeroTranslation", type="string"
    )
    cmds.setAttr(f"{node}.morphTargets[1].boneNames[0]", "joint2", type="string")
    cmds.setAttr(f"{node}.morphTargets[1].positionOffset[0]", 5, 3, 2, type="double3")
    cmds.setAttr(
        f"{node}.morphTargets[1].rotationOffset[0]", 0, 0, 0, 1, type="double4"
    )
    cmds.setAttr(f"{node}.weight[1]", 1.0)

    # Check output is correct
    output = cmds.getAttr(f"{node}.outputTranslate[1]")
    assert_true(
        approx_equal_tuple(output[0], (5, 3, 2), tolerance=0.001),
        f"Non-zero translation: Expected (5,3,2), got {output[0]}",
    )

    print("✓ Translation output works for both zero and non-zero cases")
    return True


# ──────────────────────────────────────────────────────────────────────────
# Test Registry (static — consumed by run_all_integration_tests.py)
# ──────────────────────────────────────────────────────────────────────────

_TESTS = [
    ("Node Creation", test_node_creation),
    ("Single Rotation Target", test_single_rotation_target),
    ("Single Translation Target", test_single_translation_target),
    ("Multiple Targets (Same Joint)", test_multiple_targets_on_same_joint),
    ("Multiple Joints", test_multiple_joints),
    ("Attribute Query", test_attribute_query),
    ("Zero vs Non-Zero Translation", test_zero_vs_nonzero_translation),
    (
        "Root Joint Translation No Correction",
        test_translation_root_joint_no_parent_correction,
    ),
    ("Child Joint Translation Parent Space", test_translation_parent_space_correction),
    (
        "Rotation Child Joint Non-Identity Parent Orient",
        test_rotation_child_joint_non_identity_parent_orient,
    ),
]
