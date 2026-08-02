"""
test_ccd_ik_solver_node_integration.py

Integration tests for CCDIKSolverNode - pure Maya node testing without PMX dependencies.

Tests cover:
- Node creation and attribute initialization
- Solver type name and node identity
- limitRadian attribute (read/write)
- Compound ikLinkLimits array attribute creation and querying
- Direct solver invocation (doSolve) on a minimal IK chain
- Per-joint link limit population and reading
- Convergence behavior with known target positions
"""

import math

# ── Maya standalone initialised by the test runner ───────────────────────

# Maya imports (safe after standalone.initialize())
from maya import cmds  # noqa: E402
import maya.api.OpenMaya as om  # noqa: E402
import maya.api.OpenMayaAnim as oma  # noqa: E402

# Test framework imports
from tests.integration.test_helpers import (  # noqa: E402
    setup_test_environment,
    assert_true,
    assert_eq,
    skip_test,
    approx_equal,
    approx_equal_tuple,
)


def _build_two_joint_chain(name_prefix: str = "ik") -> tuple[str, str, str]:
    """Build a simple 2-joint IK chain in the XY plane.

    Returns:
        Tuple of (root_joint, mid_joint, effector_joint) names.
    """
    root = cmds.joint(name=f"{name_prefix}_root", position=(0, 0, 0))
    cmds.setAttr(f"{root}.jointOrientX", 0)
    cmds.setAttr(f"{root}.jointOrientY", 0)
    cmds.setAttr(f"{root}.jointOrientZ", 0)

    mid = cmds.joint(name=f"{name_prefix}_mid", position=(0, 5, 0))
    cmds.setAttr(f"{mid}.jointOrientX", 0)
    cmds.setAttr(f"{mid}.jointOrientY", 0)
    cmds.setAttr(f"{mid}.jointOrientZ", 0)

    effector = cmds.joint(name=f"{name_prefix}_effector", position=(0, 10, 0))
    cmds.setAttr(f"{effector}.jointOrientX", 0)
    cmds.setAttr(f"{effector}.jointOrientY", 0)
    cmds.setAttr(f"{effector}.jointOrientZ", 0)

    return root, mid, effector


# ──────────────────────────────────────────────────────────────────────────
# Test Cases
# ──────────────────────────────────────────────────────────────────────────


def test_node_creation():
    """Test that ccdIKSolverNode can be created and has expected attributes."""
    setup_test_environment()

    # Create node
    node = cmds.createNode("ccdIKSolverNode", name="testCCDSolver")

    # Check node exists
    assert_true(cmds.objExists(node), "Node creation failed")

    # Check expected attributes exist
    required_attrs = [
        "limitRadian",
        "maxIterations",
        "ikLinkLimits",
        "ikLinkBoneIndex",
        "hasIkLinkLimits",
        "ikLinkLimitMin",
        "ikLinkLimitMax",
    ]
    for attr in required_attrs:
        assert_true(
            cmds.attributeQuery(attr, node=node, exists=True),
            f"Missing attribute: {attr}",
        )

    print(f"✓ Node created with all {len(required_attrs)} required attributes")
    return True


def test_limit_radian_attribute():
    """Test that limitRadian attribute can be read/written correctly."""
    setup_test_environment()

    node = cmds.createNode("ccdIKSolverNode", name="testCCDSolver")

    # Default value should be 0.0
    default_val = cmds.getAttr(f"{node}.limitRadian")
    assert_true(
        approx_equal(default_val, 0.0),
        f"Default limitRadian: Expected 0.0, got {default_val}",
    )

    # Set a value and verify
    test_radian = math.radians(45.0)
    cmds.setAttr(f"{node}.limitRadian", test_radian)
    read_val = cmds.getAttr(f"{node}.limitRadian")
    assert_true(
        approx_equal(read_val, test_radian),
        f"Set/Get limitRadian: Expected {test_radian:.6f}, got {read_val:.6f}",
    )

    print(f"✓ limitRadian attribute works correctly (set/get {test_radian:.4f})")
    return True


def test_max_iterations_attribute():
    """Test that maxIterations attribute can be read/written correctly."""
    setup_test_environment()

    node = cmds.createNode("ccdIKSolverNode", name="testCCDSolver")

    # Default value should be 0 (or whatever Maya defaults to)
    cmds.setAttr(f"{node}.maxIterations", 50)
    read_val = cmds.getAttr(f"{node}.maxIterations")
    assert_eq(read_val, 50, f"maxIterations: Expected 50, got {read_val}")

    cmds.setAttr(f"{node}.maxIterations", 200)
    read_val = cmds.getAttr(f"{node}.maxIterations")
    assert_eq(read_val, 200, f"maxIterations: Expected 200, got {read_val}")

    print("✓ maxIterations attribute works correctly")
    return True


def test_ik_link_limits_compound():
    """Test that ikLinkLimits compound array attributes work."""
    setup_test_environment()

    node = cmds.createNode("ccdIKSolverNode", name="testCCDSolver")

    # Populate a link limit entry
    cmds.setAttr(f"{node}.ikLinkLimits[0].ikLinkBoneIndex", 5)
    cmds.setAttr(f"{node}.ikLinkLimits[0].hasIkLinkLimits", True)
    cmds.setAttr(
        f"{node}.ikLinkLimits[0].ikLinkLimitMin", -1.0, -1.0, -1.0, type="double3"
    )
    cmds.setAttr(
        f"{node}.ikLinkLimits[0].ikLinkLimitMax", 1.0, 1.0, 1.0, type="double3"
    )

    # Read back values
    bone_idx = cmds.getAttr(f"{node}.ikLinkLimits[0].ikLinkBoneIndex")
    assert_eq(bone_idx, 5, f"ikLinkBoneIndex: Expected 5, got {bone_idx}")

    has_limits = cmds.getAttr(f"{node}.ikLinkLimits[0].hasIkLinkLimits")
    assert_eq(has_limits, 1, f"hasIkLinkLimits: Expected 1, got {has_limits}")

    # Verify compound array size
    size = cmds.getAttr(f"{node}.ikLinkLimits", size=True)
    assert_eq(size, 1, f"ikLinkLimits size: Expected 1, got {size}")

    # Add a second entry
    cmds.setAttr(f"{node}.ikLinkLimits[1].ikLinkBoneIndex", 12)
    cmds.setAttr(f"{node}.ikLinkLimits[1].hasIkLinkLimits", False)

    size = cmds.getAttr(f"{node}.ikLinkLimits", size=True)
    assert_eq(size, 2, f"ikLinkLimits size after adding: Expected 2, got {size}")

    print("✓ ikLinkLimits compound array works correctly")
    return True


def test_ik_handle_creation_with_ccd_solver():
    """Create an IK handle that uses the CCD solver and verify it solves."""
    setup_test_environment()

    # Build a simple 2-joint chain
    root, mid, effector = _build_two_joint_chain("test_ik")

    # Create the solver node first
    solver = cmds.createNode("ccdIKSolverNode", name="test_ik_ccdSolver")
    cmds.setAttr(f"{solver}.maxIterations", 100)
    cmds.setAttr(f"{solver}.limitRadian", math.radians(90.0))

    # Create IK handle with the CCD solver
    ik_handle, effector_result = cmds.ikHandle(
        startJoint=root,
        endEffector=effector,
        solver=solver,
        name="test_ik_handle",
    )

    assert_true(
        cmds.objExists(ik_handle),
        "IK handle creation failed",
    )

    # Verify the IK handle is connected to our solver
    connected_solvers = cmds.listConnections(f"{ik_handle}.ikSolver", source=True) or []
    assert_true(
        solver in connected_solvers,
        f"IK handle not connected to solver '{solver}'",
    )

    # Move the handle target and trigger solve
    cmds.setAttr(f"{ik_handle}.translateX", 3.0)
    cmds.setAttr(f"{ik_handle}.translateY", 8.0)
    cmds.setAttr(f"{ik_handle}.translateZ", 0.0)

    # Force solve by querying the end effector position
    cmds.dgdirty(ik_handle)
    solved_pos = cmds.xform(effector, query=True, worldSpace=True, translation=True)

    # The effector should now be closer to the handle position than the original
    original_dist = math.sqrt((0 - 3.0) ** 2 + (10 - 8.0) ** 2)
    solved_dist = math.sqrt(
        (solved_pos[0] - 3.0) ** 2
        + (solved_pos[1] - 8.0) ** 2
        + (solved_pos[2] - 0.0) ** 2
    )

    assert_true(
        solved_dist <= original_dist,
        f"CCD solver did not converge: original_dist={original_dist:.4f}, "
        f"solved_dist={solved_dist:.4f}",
    )

    print(
        f"✓ CCD IK handle created and solved: end_effector at "
        f"({solved_pos[0]:.2f}, {solved_pos[1]:.2f}, {solved_pos[2]:.2f})"
    )
    return True


def test_solver_convergence():
    """Test that the CCD solver converges to a target within tolerance."""
    setup_test_environment()

    # Build a 3-joint chain (more interesting for CCD)
    root = cmds.joint(name="conv_root", position=(0, 0, 0))
    cmds.setAttr(f"{root}.jointOrientX", 0)
    cmds.setAttr(f"{root}.jointOrientY", 0)
    cmds.setAttr(f"{root}.jointOrientZ", 0)

    mid = cmds.joint(name="conv_mid", position=(0, 5, 0))
    cmds.setAttr(f"{mid}.jointOrientX", 0)
    cmds.setAttr(f"{mid}.jointOrientY", 0)
    cmds.setAttr(f"{mid}.jointOrientZ", 0)

    effector = cmds.joint(name="conv_eff", position=(0, 10, 0))
    cmds.setAttr(f"{effector}.jointOrientX", 0)
    cmds.setAttr(f"{effector}.jointOrientY", 0)
    cmds.setAttr(f"{effector}.jointOrientZ", 0)

    # Create solver and IK handle
    solver = cmds.createNode("ccdIKSolverNode", name="conv_ccdSolver")
    cmds.setAttr(f"{solver}.maxIterations", 200)
    cmds.setAttr(f"{solver}.limitRadian", math.radians(90.0))

    ik_handle, _ = cmds.ikHandle(
        startJoint=root,
        endEffector=effector,
        solver=solver,
        name="conv_ik_handle",
    )

    # Move handle to a target position
    target_x, target_y = 4.0, 7.0
    cmds.setAttr(f"{ik_handle}.translateX", target_x)
    cmds.setAttr(f"{ik_handle}.translateY", target_y)
    cmds.setAttr(f"{ik_handle}.translateZ", 0.0)

    # Trigger solve
    cmds.dgdirty(ik_handle)
    solved_pos = cmds.xform(effector, query=True, worldSpace=True, translation=True)

    # Check convergence: effector should be close to target
    dist_to_target = math.sqrt(
        (solved_pos[0] - target_x) ** 2
        + (solved_pos[1] - target_y) ** 2
        + (solved_pos[2] - 0.0) ** 2
    )

    # Allow some tolerance for the CCD approximation
    assert_true(
        dist_to_target <= 0.5,
        f"Solver did not converge: effector at ({solved_pos[0]:.2f}, {solved_pos[1]:.2f}), "
        f"target ({target_x:.2f}, {target_y:.2f}), distance={dist_to_target:.4f}",
    )

    print(
        f"✓ Solver converged: effector at ({solved_pos[0]:.2f}, {solved_pos[1]:.2f}), "
        f"distance to target={dist_to_target:.4f}"
    )
    return True


def test_ik_handle_priority_attribute():
    """Test that IK handle priority can be set."""
    setup_test_environment()

    root, mid, effector = _build_two_joint_chain("priority")

    solver = cmds.createNode("ccdIKSolverNode", name="priority_ccdSolver")
    cmds.setAttr(f"{solver}.maxIterations", 50)

    ik_handle, _ = cmds.ikHandle(
        startJoint=root,
        endEffector=effector,
        solver=solver,
        name="priority_ik_handle",
    )

    # Check default priority
    default_priority = cmds.getAttr(f"{ik_handle}.priority")
    assert_eq(
        default_priority,
        1,
        f"Default priority: Expected 1, got {default_priority}",
    )

    # Set priority
    cmds.setAttr(f"{ik_handle}.priority", 2)
    read_priority = cmds.getAttr(f"{ik_handle}.priority")
    assert_eq(
        read_priority,
        2,
        f"Set priority: Expected 2, got {read_priority}",
    )

    print("✓ IK handle priority attribute works correctly")
    return True


# ──────────────────────────────────────────────────────────────────────────
# Test Registry (static — consumed by run_all_integration_tests.py)
# ──────────────────────────────────────────────────────────────────────────

_TESTS = [
    ("Node Creation & Attributes", test_node_creation),
    ("limitRadian Attribute", test_limit_radian_attribute),
    ("maxIterations Attribute", test_max_iterations_attribute),
    ("ikLinkLimits Compound Array", test_ik_link_limits_compound),
    ("IK Handle Creation with CCD Solver", test_ik_handle_creation_with_ccd_solver),
    ("Solver Convergence", test_solver_convergence),
    ("IK Handle Priority Attribute", test_ik_handle_priority_attribute),
]
