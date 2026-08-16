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
from maya import cmds

# Test framework imports
from tests.integration.test_helpers import (
    approx_equal,
    assert_eq,
    assert_true,
    setup_test_environment,
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
    root, _mid, effector = _build_two_joint_chain("test_ik")

    # Create the solver node first
    solver = cmds.createNode("ccdIKSolverNode", name="test_ik_ccdSolver")
    cmds.setAttr(f"{solver}.maxIterations", 100)
    cmds.setAttr(f"{solver}.limitRadian", math.radians(90.0))

    # Create IK handle with the CCD solver
    ik_handle, _ = cmds.ikHandle(
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

    root, _mid, effector = _build_two_joint_chain("priority")

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


# ══════════════════════════════════════════════════════════════════════════
# Real-data helpers — Left leg (PMX bones 14/15/16)
#
# These replicate exactly what the PMX importer writes for the model's
# left leg chain, so the CCD solver is tested against real-world geometry:
#
#   bone 14 左足     (thigh) → mayaJoint (0.802, 11.703,  0.220)
#   bone 15 左ひざ   (knee)  → mayaJoint (0.801,  6.872,  0.075)
#   bone 16 左足首   (ankle) → mayaJoint (0.800,  1.674, -0.375)
#
# PMX positions are stored in MMD space (left-handed, +Z forward). The
# importer negates Z when converting to Maya (right-handed, -Z forward),
# so the Maya positions below already have Z flipped.
# ══════════════════════════════════════════════════════════════════════════

_LEG_THIGH_POS = (0.8016511797904968, 11.70343017578125, 0.21956150233745575)
_LEG_KNEE_POS = (0.8010708093643188, 6.872162818908691, 0.07484938204288483)
_LEG_ANKLE_POS = (0.8004451990127563, 1.6735248565673828, -0.37491273880004883)

# 左足ＩＫ (bone 24) — the IK target. At rest it sits on the ankle.
_LEG_IK_TARGET_X = 0.8004451990127563
_LEG_IK_TARGET_Z = -0.37491273880004883
_LEG_REST_Y = _LEG_ANKLE_POS[1]

# Real IK parameters from the PMX file (bone 24's ik block).
_LEG_IK_LOOP_COUNT = 40
_LEG_IK_LIMIT_RADIAN = 2.0

# Knee link (bone 15) rotation limit, in MMD space — set exactly as the
# importer does. The C++ solver maps MMD [-π, -0.0087] → Maya [0.0087, π],
# i.e. the knee must rotate positively (the natural forward bend) and can
# never rotate negatively (backward / outward bend).
_LEG_KNEE_LIMIT_MIN = (-math.pi, 0.0, 0.0)
_LEG_KNEE_LIMIT_MAX = (-0.008726646192371845, 0.0, 0.0)


def _add_pmx_bone_index(joint: str, bone_idx: int) -> None:
    """Add the ``pmxBoneIndex`` int attribute used by the solver's link lookup."""
    if not cmds.attributeQuery("pmxBoneIndex", node=joint, exists=True):
        cmds.addAttr(
            joint, longName="pmxBoneIndex", shortName="pmxIdx", attributeType="long"
        )
    cmds.setAttr(f"{joint}.pmxBoneIndex", bone_idx)


def _build_real_leg_chain(name_prefix: str = "tololo") -> tuple[str, str, str]:
    """Build left leg chain: thigh → knee → ankle.

    Returns:
        Tuple of (thigh, knee, ankle) joint names.
    """
    thigh = cmds.joint(name=f"{name_prefix}_thigh", position=_LEG_THIGH_POS)
    cmds.setAttr(f"{thigh}.jointOrientX", 0)
    cmds.setAttr(f"{thigh}.jointOrientY", 0)
    cmds.setAttr(f"{thigh}.jointOrientZ", 0)
    _add_pmx_bone_index(thigh, 14)

    knee = cmds.joint(name=f"{name_prefix}_knee", position=_LEG_KNEE_POS)
    cmds.setAttr(f"{knee}.jointOrientX", 0)
    cmds.setAttr(f"{knee}.jointOrientY", 0)
    cmds.setAttr(f"{knee}.jointOrientZ", 0)
    _add_pmx_bone_index(knee, 15)

    ankle = cmds.joint(name=f"{name_prefix}_ankle", position=_LEG_ANKLE_POS)
    cmds.setAttr(f"{ankle}.jointOrientX", 0)
    cmds.setAttr(f"{ankle}.jointOrientY", 0)
    cmds.setAttr(f"{ankle}.jointOrientZ", 0)
    _add_pmx_bone_index(ankle, 16)

    return thigh, knee, ankle


def _create_real_leg_solver(
    thigh: str, knee: str, ankle: str, name_prefix: str = "tololo"
) -> tuple[str, str]:
    """Create the CCD solver + IK handle with real IK data.

    Args:
        thigh: Thigh joint name.
        knee:  Knee joint name.
        ankle: Ankle (end effector) joint name.
        name_prefix: Naming prefix for the solver/handle.

    Returns:
        Tuple of (solver, ik_handle) node names.
    """
    solver = cmds.createNode("ccdIKSolverNode", name=f"{name_prefix}_ccdSolver")
    cmds.setAttr(f"{solver}.maxIterations", _LEG_IK_LOOP_COUNT)
    cmds.setAttr(f"{solver}.limitRadian", _LEG_IK_LIMIT_RADIAN)

    # Link 0: knee (bone 15) — single-axis X limit (MMD space values).
    cmds.setAttr(f"{solver}.ikLinkLimits[0].ikLinkBoneIndex", 15)
    cmds.setAttr(f"{solver}.ikLinkLimits[0].hasIkLinkLimits", True)
    cmds.setAttr(
        f"{solver}.ikLinkLimits[0].ikLinkLimitMin",
        *_LEG_KNEE_LIMIT_MIN,
        type="double3",
    )
    cmds.setAttr(
        f"{solver}.ikLinkLimits[0].ikLinkLimitMax",
        *_LEG_KNEE_LIMIT_MAX,
        type="double3",
    )

    # Link 1: thigh (bone 14) — unconstrained.
    cmds.setAttr(f"{solver}.ikLinkLimits[1].ikLinkBoneIndex", 14)
    cmds.setAttr(f"{solver}.ikLinkLimits[1].hasIkLinkLimits", False)

    ik_handle, _ = cmds.ikHandle(
        startJoint=thigh,
        endEffector=ankle,
        solver=solver,
        name=f"{name_prefix}_ikHandle",
    )
    return solver, ik_handle


def _snap_ik_handle_to_rest(ik_handle: str) -> None:
    """Place the IK handle back at the rest ankle position."""
    cmds.setAttr(f"{ik_handle}.translateX", _LEG_IK_TARGET_X)
    cmds.setAttr(f"{ik_handle}.translateY", _LEG_REST_Y)
    cmds.setAttr(f"{ik_handle}.translateZ", _LEG_IK_TARGET_Z)


def test_real_leg_knee_bends_forward():
    """Left leg: raising the IK target must bend the knee forward.

    Reproduces the real model's leg chain (bones 14/15/16) and IK data
    (loopCount=40, limitRadian=2.0, knee X limit [-π, -0.0087] in MMD).

    The target is raised incrementally (like dragging the IK handle or
    issuing repeated ``move -r 0 <dy> 0`` commands — each move triggers a
    fresh solve while the joints are already rotated from the previous
    solve).  For every height the knee must:
      1. Bend in the natural forward direction (positive X rotation,
         i.e. inside the Maya-space limit [0.0087, π]) — never backward.
      2. Bring the ankle to the target without overshooting above it.

    Before the fix this test fails: after a couple of small raises the
    ankle overshoots the handler, and at roughly knee height the knee
    flips and bends the wrong way.
    """
    setup_test_environment()

    thigh, knee, ankle = _build_real_leg_chain("tololo")
    _, ik_handle = _create_real_leg_solver(thigh, knee, ankle)
    _snap_ik_handle_to_rest(ik_handle)

    # Heights to test, in the same spirit as the user's reported moves
    # (cumulative +Y offsets above the rest ankle: 0.761, 0.309, 1.942).
    target_heights = [2.0, 2.435, 2.744, 3.0, 3.5, 4.0, 4.686, 5.0, 5.5, 6.0, 6.5]

    reach_tol = 0.25
    overshoot_tol = 0.15
    last_ok = _LEG_REST_Y
    for target_y in target_heights:
        cmds.setAttr(f"{ik_handle}.translateY", target_y)
        cmds.dgdirty(ik_handle)

        ankle_pos = cmds.xform(ankle, query=True, worldSpace=True, translation=True)
        knee_rot_deg = cmds.getAttr(f"{knee}.rotateX")
        knee_rot = math.radians(knee_rot_deg)

        dist = math.sqrt(
            (ankle_pos[0] - _LEG_IK_TARGET_X) ** 2
            + (ankle_pos[1] - target_y) ** 2
            + (ankle_pos[2] - _LEG_IK_TARGET_Z) ** 2
        )

        # 1) Knee must bend forward: Maya-space limit maps to [0.0087, π].
        assert_true(
            knee_rot >= -0.02,
            f"Knee bent BACKWARD at target_y={target_y:.3f}: "
            f"knee.rotateX={knee_rot_deg:.4f}° ({knee_rot:.4f} rad); must be >= 0 "
            f"(natural forward bend)",
        )

        # 2) No overshoot above the IK handler.
        assert_true(
            ankle_pos[1] <= target_y + overshoot_tol,
            f"Ankle overshot above IK target at target_y={target_y:.3f}: "
            f"ankle_y={ankle_pos[1]:.4f} > target+{overshoot_tol}",
        )

        # 3) Ankle must track the target.
        assert_true(
            dist <= reach_tol,
            f"Ankle did not reach IK target at target_y={target_y:.3f}: "
            f"ankle={tuple(round(v, 4) for v in ankle_pos)}, dist={dist:.4f}",
        )
        last_ok = target_y

    print(
        f"✓ Real leg knee bends forward from rest (y={_LEG_REST_Y:.2f}) up to "
        f"y={last_ok:.2f} across {len(target_heights)} target heights"
    )
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
    ("Real Leg Knee Bend Direction", test_real_leg_knee_bends_forward),
]
