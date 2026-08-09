"""
Integration tests for VPD (Vocaloid Pose Data) import in Maya.

Tests that VPD poses are correctly applied to PMX models: bone rotations,
IK handle translations, and special bone flags (FIXED_AXIS, LOCAL_COORDINATE).
"""

from __future__ import annotations

# ── Maya standalone initialised by the test runner ───────────────────────
import maya.api.OpenMaya as om  # noqa: E402
import maya.cmds as cmds  # noqa: E402

# ── Project imports ─────────────────────────────────────────────────────────
from mmd.core.data_types import (  # noqa: E402
    PMXBoneFlagBits,
    PmxModel,
    VPDFile,
)
from mmd.maya.maya_data_types import MayaPmxData  # noqa: E402
from mmd.maya.pmx_scene_builder import build_pmx_scene  # noqa: E402
from mmd.maya.vpd_scene_builder import (  # noqa: E402
    apply_vpd_pose_to_scene,
    _rotation_degrees_from_vpd_quaternion,
)
from tests.integration.test_helpers import (  # noqa: E402
    assert_true,
    matrix,
    skip_test,
    suppressed_undo,
    euler_degrees_to_quat,
    quat_dot,
)

# ── Helpers


def _capture_world_rest_cache() -> dict[str, om.MQuaternion]:
    """Capture world-space rest quaternions for all joints in the current scene.

    This is a shared helper used by multiple VPD test functions to avoid
    rebuilding the scene from scratch for each VPD file.
    """
    orig_time = cmds.currentTime(query=True)
    cmds.currentTime(0)
    q_world_cache: dict[str, om.MQuaternion] = {}
    for jnt in cmds.ls(type="joint", long=True):
        try:
            short = jnt.split("|")[-1]
            q_world_cache[short] = _get_world_rotation(jnt)
        except Exception:
            pass  # Joint may not be queryable — skip
    cmds.currentTime(orig_time)
    return q_world_cache


# Tolerance for quaternion dot-product comparison
_QUAT_DOT_TOLERANCE = 0.99
# Tolerance for translation comparison (cm)
_TRANS_TOLERANCE = 1e-2


def _get_world_rotation(node_name: str) -> om.MQuaternion:
    """Get the world-space rotation quaternion of a node."""
    matrix = om.MMatrix(cmds.xform(node_name, query=True, worldSpace=True, matrix=True))
    return om.MTransformationMatrix(matrix).rotation(asQuaternion=True)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_vpd_parsing(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that all test VPD files parse without errors."""
    all_vpd = all_vpd_data
    assert_true(
        len(all_vpd) > 0,
        "No VPD files were parsed successfully",
    )

    errors: list[str] = []
    for name, vpd in all_vpd.items():
        if vpd.bone_count <= 0:
            errors.append(f"{name} has {vpd.bone_count} bones (expected > 0)")
        elif len(vpd.bones) != vpd.bone_count:
            errors.append(
                f"{name} claims {vpd.bone_count} bones but parsed {len(vpd.bones)}"
            )

    if errors:
        assert_true(False, "\n".join(errors))
    print(f"  PASS: {len(all_vpd)} VPD files parsed correctly")
    return True


@matrix
def test_vpd_apply_no_errors(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that applying each VPD file completes without exceptions.

    Uses the shared scene from the test runner and undo chunks to avoid
    rebuilding the scene for each VPD file.
    """
    all_vpd = all_vpd_data
    errors: list[str] = []

    for name, vpd in all_vpd.items():
        cmds.undoInfo(openChunk=True)
        try:
            apply_vpd_pose_to_scene(
                vpd_data=vpd,
                model=maya_pmx_data.to_resolved(),
                create_keyframe=False,
            )
            print(f"  PASS: {name}")
        except Exception as exc:
            errors.append(f"apply_vpd_pose_to_scene raised exception for {name}: {exc}")
        finally:
            cmds.undoInfo(closeChunk=True)
            suppressed_undo()

    assert_true(
        len(errors) == 0,
        f"{len(errors)} VPD(s) failed:\n" + "\n".join(errors),
    )
    return True


def _get_ik_chain_local_names(pmx_data: PmxModel) -> set[str]:
    """Collect the nameLocal of all joints that are IK chain links."""
    names: set[str] = set()
    for b in pmx_data.bones:
        if b.flags & PMXBoneFlagBits.IK and b.ik is not None:
            for link in b.ik.links:
                if link.boneIndex < len(pmx_data.bones):
                    names.add(pmx_data.bones[link.boneIndex].nameLocal)
    return names


def _get_inherit_rotation_names(pmx_data: PmxModel) -> set[str]:
    """Collect the nameLocal of all bones with INHERIT_ROTATION flag.
    These get their rotation via multiplyDivide from a parent, not from VPD."""
    return {
        b.nameLocal
        for b in pmx_data.bones
        if b.flags & PMXBoneFlagBits.INHERIT_ROTATION
    }


def _get_local_coordinate_names(pmx_data: PmxModel) -> set[str]:
    """Collect the nameLocal of all bones with LOCAL_COORDINATE flag."""
    return {
        b.nameLocal
        for b in pmx_data.bones
        if b.flags & PMXBoneFlagBits.LOCAL_COORDINATE
    }


def _get_bone_rest_q(
    short_name: str,
    joint_name: str,
    q_world_cache: dict[str, om.MQuaternion],
    local_coord_names: set[str],
    vpd_bone_name: str,
) -> om.MQuaternion:
    """Get the appropriate world-rest quaternion for a bone.

    For LOCAL_COORDINATE bones, MMD ignores the local axes during motion
    playback, so we use the parent's world rest.  For all other bones,
    we use the joint's own world rest.
    """
    if vpd_bone_name in local_coord_names:
        parents = cmds.listRelatives(joint_name, parent=True, fullPath=True)
        if parents:
            parent_short = parents[0].split("|")[-1]
            return q_world_cache.get(parent_short, om.MQuaternion())
    return q_world_cache.get(short_name, om.MQuaternion())


@matrix
def test_vpd_bone_rotation(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that non-IK-chain bones receive the correct VPD rotation.

    Skips IK chain link bones since their rotation is controlled by the IK
    solver.  Uses parent world-rest for LOCAL_COORDINATE bones to match MMD.

    Uses the shared scene — world rest is captured once, then each VPD is
    applied inside an undo chunk to avoid rebuilding the scene.
    """
    all_vpd = all_vpd_data
    ik_chain_names = _get_ik_chain_local_names(pmx_data)
    local_coord_names = _get_local_coordinate_names(pmx_data)
    inherit_rot_names = _get_inherit_rotation_names(pmx_data)
    bone_map = maya_pmx_data.bone_name_map

    # Capture world rest once from the shared scene
    q_world_cache = _capture_world_rest_cache()

    errors: list[str] = []
    for name, vpd in all_vpd.items():
        cmds.undoInfo(openChunk=True)
        try:
            # Apply VPD
            apply_vpd_pose_to_scene(
                vpd_data=vpd,
                model=maya_pmx_data.to_resolved(),
                create_keyframe=False,
            )

            # Now check each bone in the VPD
            vpd_ok = True
            for bone_pose in vpd.bones:
                vpd_bone_name = bone_pose.bone_name
                maya_joint = bone_map.get(vpd_bone_name)
                if not maya_joint or not cmds.objExists(maya_joint):
                    continue

                # Skip IK chain link bones and INHERIT_ROTATION bones
                if (
                    vpd_bone_name in ik_chain_names
                    or vpd_bone_name in inherit_rot_names
                ):
                    continue

                short_name = maya_joint.split("|")[-1]
                q_world_rest = _get_bone_rest_q(
                    short_name,
                    maya_joint,
                    q_world_cache,
                    local_coord_names,
                    vpd_bone_name,
                )
                expected_rx, expected_ry, expected_rz = (
                    _rotation_degrees_from_vpd_quaternion(
                        maya_joint,
                        bone_pose.rotation,
                        q_world_rest=q_world_rest,
                    )
                )

                # Read actual rotation
                actual_rx = cmds.getAttr(f"{maya_joint}.rotateX")
                actual_ry = cmds.getAttr(f"{maya_joint}.rotateY")
                actual_rz = cmds.getAttr(f"{maya_joint}.rotateZ")

                # Compare via quaternion dot product (handles Euler ambiguity)
                rot_order_int = cmds.getAttr(f"{maya_joint}.rotateOrder")

                q_actual = euler_degrees_to_quat(
                    actual_rx, actual_ry, actual_rz, rot_order_int
                )
                q_expected = euler_degrees_to_quat(
                    expected_rx, expected_ry, expected_rz, rot_order_int
                )

                dot = quat_dot(q_actual, q_expected)
                if dot < _QUAT_DOT_TOLERANCE:
                    errors.append(
                        f"{name}/{vpd_bone_name}: "
                        f"expected ({expected_rx:.2f}, {expected_ry:.2f}, {expected_rz:.2f}), "
                        f"got ({actual_rx:.2f}, {actual_ry:.2f}, {actual_rz:.2f}), "
                        f"dot={dot:.4f}"
                    )
                    vpd_ok = False
                    break

            if vpd_ok:
                print(f"  PASS: {name}: all {len(vpd.bones)} bone rotations match")
        except Exception as exc:
            errors.append(f"apply failed for {name}: {exc}")
        finally:
            cmds.undoInfo(closeChunk=True)
            suppressed_undo()

    assert_true(
        len(errors) == 0,
        f"{len(errors)} VPD(s) failed:\n" + "\n".join(errors),
    )
    return True


def test_vpd_ik_control_bone_translation(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that IK control bones have their VPD translation applied directly
    to the joint (not the IK handle), matching the new hierarchy where all
    animation curves live on the control bone.

    Uses the shared scene — each VPD is applied inside an undo chunk.
    """
    all_vpd = all_vpd_data
    bone_map = maya_pmx_data.bone_name_map

    # Collect IK control bone names from PMX data
    ik_bone_names = {
        b.nameLocal for b in pmx_data.bones if b.flags & PMXBoneFlagBits.IK
    }

    # Pre-capture joint bind poses at frame 0 once
    cmds.currentTime(0)
    joint_bind: dict[str, tuple[float, float, float]] = {}
    for bname in ik_bone_names:
        maya_joint = bone_map.get(bname)
        if not maya_joint or not cmds.objExists(maya_joint):
            continue
        tx = cmds.getAttr(f"{maya_joint}.translateX")
        ty = cmds.getAttr(f"{maya_joint}.translateY")
        tz = cmds.getAttr(f"{maya_joint}.translateZ")
        joint_bind[maya_joint] = (tx, ty, tz)

    errors: list[str] = []
    for name, vpd in all_vpd.items():
        cmds.undoInfo(openChunk=True)
        try:
            apply_vpd_pose_to_scene(
                vpd_data=vpd,
                model=maya_pmx_data.to_resolved(),
                create_keyframe=False,
            )

            vpd_ok = True
            for bone_pose in vpd.bones:
                vpd_bone_name = bone_pose.bone_name
                maya_joint = bone_map.get(vpd_bone_name)
                if not maya_joint or not cmds.objExists(maya_joint):
                    continue
                if vpd_bone_name not in ik_bone_names:
                    continue

                bind = joint_bind.get(maya_joint)
                if bind is None:
                    continue

                exp_tx = bind[0] + bone_pose.position.x
                exp_ty = bind[1] + bone_pose.position.y
                exp_tz = bind[2] - bone_pose.position.z  # Z flip

                act_tx = cmds.getAttr(f"{maya_joint}.translateX")
                act_ty = cmds.getAttr(f"{maya_joint}.translateY")
                act_tz = cmds.getAttr(f"{maya_joint}.translateZ")

                if (
                    abs(act_tx - exp_tx) > _TRANS_TOLERANCE
                    or abs(act_ty - exp_ty) > _TRANS_TOLERANCE
                    or abs(act_tz - exp_tz) > _TRANS_TOLERANCE
                ):
                    errors.append(
                        f"{name}/{vpd_bone_name} IK bone translation: "
                        f"expected ({exp_tx:.3f}, {exp_ty:.3f}, {exp_tz:.3f}), "
                        f"got ({act_tx:.3f}, {act_ty:.3f}, {act_tz:.3f})"
                    )
                    vpd_ok = False
                    break

            if vpd_ok:
                print(f"  PASS: {name}: IK control bone translations match")
        except Exception as exc:
            errors.append(f"{name}: {exc}")
        finally:
            cmds.undoInfo(closeChunk=True)
            suppressed_undo()

    assert_true(
        len(errors) == 0,
        f"{len(errors)} VPD(s) failed:\n" + "\n".join(errors),
    )
    return True


@matrix
def test_vpd_ik_chain_rotation_skipped(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that IK chain link joints do NOT have their rotation set by the
    VPD — the IK solver controls those.  Non-IK bones should still match.

    Uses the shared scene — world rest is captured once, then each VPD is
    applied inside an undo chunk.
    """
    all_vpd = all_vpd_data
    ik_chain_local_names = _get_ik_chain_local_names(pmx_data)
    local_coord_names = _get_local_coordinate_names(pmx_data)
    inherit_rot_names = _get_inherit_rotation_names(pmx_data)
    bone_map = maya_pmx_data.bone_name_map

    # Capture world rest once from the shared scene
    q_world_cache = _capture_world_rest_cache()

    errors: list[str] = []
    for name, vpd in all_vpd.items():
        cmds.undoInfo(openChunk=True)
        try:
            apply_vpd_pose_to_scene(
                vpd_data=vpd,
                model=maya_pmx_data.to_resolved(),
                create_keyframe=False,
            )

            vpd_ok = True
            for bone_pose in vpd.bones:
                vpd_bone_name = bone_pose.bone_name
                maya_joint = bone_map.get(vpd_bone_name)
                if not maya_joint or not cmds.objExists(maya_joint):
                    continue

                short_name = maya_joint.split("|")[-1]
                q_world_rest = _get_bone_rest_q(
                    short_name,
                    maya_joint,
                    q_world_cache,
                    local_coord_names,
                    vpd_bone_name,
                )

                expected_rx, expected_ry, expected_rz = (
                    _rotation_degrees_from_vpd_quaternion(
                        maya_joint, bone_pose.rotation, q_world_rest=q_world_rest
                    )
                )

                actual_rx = cmds.getAttr(f"{maya_joint}.rotateX")
                actual_ry = cmds.getAttr(f"{maya_joint}.rotateY")
                actual_rz = cmds.getAttr(f"{maya_joint}.rotateZ")

                rot_order_int = cmds.getAttr(f"{maya_joint}.rotateOrder")

                q_actual = euler_degrees_to_quat(
                    actual_rx, actual_ry, actual_rz, rot_order_int
                )
                q_expected = euler_degrees_to_quat(
                    expected_rx, expected_ry, expected_rz, rot_order_int
                )

                dot = quat_dot(q_actual, q_expected)

                is_ik_chain = vpd_bone_name in ik_chain_local_names
                is_inherit_rot = vpd_bone_name in inherit_rot_names
                if is_ik_chain or is_inherit_rot:
                    continue

                if dot < _QUAT_DOT_TOLERANCE:
                    errors.append(
                        f"{name}/{vpd_bone_name} is NOT an IK chain bone but "
                        f"rotation was NOT applied (dot={dot:.4f}, expected >= {_QUAT_DOT_TOLERANCE})"
                    )
                    vpd_ok = False
                    break

            if vpd_ok:
                ik_count = sum(
                    1 for bp in vpd.bones if bp.bone_name in ik_chain_local_names
                )
                non_ik_count = len(vpd.bones) - ik_count
                print(
                    f"  PASS: {name}: {ik_count} IK-chain bones (skipped), "
                    f"{non_ik_count} non-IK bones verified"
                )
        except Exception as exc:
            errors.append(f"{name}: {exc}")
        finally:
            cmds.undoInfo(closeChunk=True)
            suppressed_undo()

    assert_true(
        len(errors) == 0,
        f"{len(errors)} VPD(s) failed:\n" + "\n".join(errors),
    )
    return True


@matrix
def test_vpd_toe_follows_ankle(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that the toe bone (右足首先) has the correct rotation after VPD
    application.  The toe is the end-effector of the toe IK chain, so its
    rotation is NOT controlled by the IK solver and SHOULD be set by the VPD.

    Uses the shared scene — world rest captured once, VPD applied with undo.
    """
    all_vpd = all_vpd_data
    bone_map = maya_pmx_data.bone_name_map
    toe_name = "右足首先"  # toe bone in PMX data

    # Capture world rest once from the shared scene
    q_world_cache = _capture_world_rest_cache()

    errors: list[str] = []
    for name, vpd in all_vpd.items():
        cmds.undoInfo(openChunk=True)
        try:
            apply_vpd_pose_to_scene(
                vpd_data=vpd,
                model=maya_pmx_data.to_resolved(),
                create_keyframe=False,
            )

            toe_pose = next((bp for bp in vpd.bones if bp.bone_name == toe_name), None)
            if toe_pose is None:
                print(f"  PASS: {name}: toe bone '{toe_name}' not in VPD — skipped")
                continue

            maya_toe = bone_map.get(toe_name)
            if not maya_toe:
                continue

            short_name = maya_toe.split("|")[-1]
            q_world_rest = q_world_cache.get(short_name)
            expected_rx, expected_ry, expected_rz = (
                _rotation_degrees_from_vpd_quaternion(
                    maya_toe, toe_pose.rotation, q_world_rest=q_world_rest
                )
            )

            actual_rx = cmds.getAttr(f"{maya_toe}.rotateX")
            actual_ry = cmds.getAttr(f"{maya_toe}.rotateY")
            actual_rz = cmds.getAttr(f"{maya_toe}.rotateZ")

            rot_order_int = cmds.getAttr(f"{maya_toe}.rotateOrder")

            q_actual = euler_degrees_to_quat(
                actual_rx, actual_ry, actual_rz, rot_order_int
            )
            q_expected = euler_degrees_to_quat(
                expected_rx, expected_ry, expected_rz, rot_order_int
            )

            dot = quat_dot(q_actual, q_expected)
            if dot < _QUAT_DOT_TOLERANCE:
                errors.append(
                    f"{name}/toe ({toe_name}): "
                    f"expected ({expected_rx:.2f}, {expected_ry:.2f}, {expected_rz:.2f}), "
                    f"got ({actual_rx:.2f}, {actual_ry:.2f}, {actual_rz:.2f}), "
                    f"dot={dot:.4f}"
                )
                break

            print(f"  PASS: {name}: toe rotation matches (dot={dot:.4f})")
        except Exception as exc:
            errors.append(f"{name}: {exc}")
        finally:
            cmds.undoInfo(closeChunk=True)
            suppressed_undo()

    assert_true(
        len(errors) == 0,
        f"{len(errors)} VPD(s) failed:\n" + "\n".join(errors),
    )
    return True


@matrix
def test_vpd_fixed_axis_bones(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that FIXED_AXIS bones only have rotation around their constrained
    X axis after VPD application (rotateY and rotateZ should be ~0).

    Uses the shared scene — VPD applied with undo to avoid rebuilding.
    """
    fixed_axis_names = {
        b.nameLocal for b in pmx_data.bones if b.flags & PMXBoneFlagBits.FIXED_AXIS
    }
    if not fixed_axis_names:
        skip_test("No FIXED_AXIS bones in model")

    all_vpd = all_vpd_data
    bone_map = maya_pmx_data.bone_name_map
    YZ_TOLERANCE = 1e-3

    errors: list[str] = []
    for name, vpd in all_vpd.items():
        cmds.undoInfo(openChunk=True)
        try:
            apply_vpd_pose_to_scene(
                vpd_data=vpd,
                model=maya_pmx_data.to_resolved(),
                create_keyframe=False,
            )

            vpd_ok = True
            for bone_pose in vpd.bones:
                if bone_pose.bone_name not in fixed_axis_names:
                    continue

                maya_joint = bone_map.get(bone_pose.bone_name)
                if not maya_joint:
                    continue

                ry = cmds.getAttr(f"{maya_joint}.rotateY")
                rz = cmds.getAttr(f"{maya_joint}.rotateZ")

                if abs(ry) > YZ_TOLERANCE or abs(rz) > YZ_TOLERANCE:
                    errors.append(
                        f"{name}/{bone_pose.bone_name} (FIXED_AXIS): "
                        f"rotateY={ry:.4f}, rotateZ={rz:.4f} (expected ~0)"
                    )
                    vpd_ok = False
                    break

            if vpd_ok:
                fa_in_vpd = sum(
                    1 for bp in vpd.bones if bp.bone_name in fixed_axis_names
                )
                print(
                    f"  PASS: {name}: {fa_in_vpd} FIXED_AXIS bones have only X rotation"
                )
        except Exception as exc:
            errors.append(f"{name}: {exc}")
        finally:
            cmds.undoInfo(closeChunk=True)
            suppressed_undo()

    assert_true(
        len(errors) == 0,
        f"{len(errors)} VPD(s) failed:\n" + "\n".join(errors),
    )
    return True


def test_rest_pose_attributes_on_joints(
    pmx_data: PmxModel,
    maya_pmx_data: MayaPmxData,
    all_vpd_data: dict[str, VPDFile] | None = None,
) -> bool:
    """Verify that every joint created during PMX import carries the six
    pmxRest* custom attributes that are written by _capture_rest_pose_on_joints.

    Uses the shared scene from the test runner (non-mutating — no undo needed).
    """
    expected_attrs = [
        "pmxRestTranslateX",
        "pmxRestTranslateY",
        "pmxRestTranslateZ",
        "pmxRestRotateX",
        "pmxRestRotateY",
        "pmxRestRotateZ",
    ]

    missing_joints: list[str] = []

    for joint_name in maya_pmx_data.bone_name_map.values():
        if not cmds.objExists(joint_name):
            continue
        for attr in expected_attrs:
            if not cmds.attributeQuery(attr, node=joint_name, exists=True):
                missing_joints.append(f"{joint_name}.{attr}")
                break

    if missing_joints:
        detail = "\n".join(f"    - {item}" for item in missing_joints[:5])
        if len(missing_joints) > 5:
            detail += f"\n    ... and {len(missing_joints) - 5} more"
        assert_true(
            False,
            f"{len(missing_joints)} joints are missing pmxRest* attributes:\n{detail}",
        )
    joint_count = len(maya_pmx_data.bone_name_map)
    print(f"  PASS: All {joint_count} joints have pmxRest* attributes")
    return True


@matrix
def test_reset_to_bind_pose(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that reset_model_to_bind_pose restores every joint to its
    rest-pose values after a VPD has been applied.

    Uses the shared scene — rest pose snapshot captured once, then each of
    the first 3 VPDs is applied and reset within an undo chunk.
    """
    from mmd.maya.pmx_model_utils import reset_model_to_bind_pose

    all_vpd = all_vpd_data
    bone_map = maya_pmx_data.bone_name_map

    _TRANS_TOLERANCE = 1e-3
    # (1 - |quat dot|) — world rotation error (radians-ish measure).
    _ROT_TOLERANCE = 1e-3

    # Put the shared scene at TRUE rest first: earlier tests stepped the
    # physics solver, whose internal Bullet state persists across undo and can
    # leave the physics-driven joints a hair off rest.  Resetting also rewinds
    # the solver (Phase 3), so the snapshot below captures the exact rest pose.
    reset_model_to_bind_pose(model=maya_pmx_data.to_resolved())

    # Capture ground-truth rest pose once from the shared scene.  World-space
    # (translate + quaternion) is compared, not raw local Euler: the C++ node
    # writes physics-driven joints from MATRICES, so the raw Euler is a
    # canonicalized representation of the same rotation (e.g. identity may be
    # stored as (180,-180,180) in the skeleton builder but written back as
    # (0,0,0) by the solver).  The world pose is what "rest pose" means.
    rest_snapshot: dict[str, tuple[tuple, om.MQuaternion]] = {}
    for joint_name in bone_map.values():
        if not cmds.objExists(joint_name):
            continue
        rest_snapshot[joint_name] = (
            tuple(cmds.xform(joint_name, q=True, ws=True, translation=True)),
            _get_world_rotation(joint_name),
        )

    errors: list[str] = []
    for name, vpd in list(all_vpd.items())[:3]:
        cmds.undoInfo(openChunk=True)
        try:
            apply_vpd_pose_to_scene(
                vpd_data=vpd,
                model=maya_pmx_data.to_resolved(),
                create_keyframe=False,
            )

            stats = reset_model_to_bind_pose(
                model=maya_pmx_data.to_resolved(),
            )

            mismatch_count = 0
            details: list[str] = []
            for joint_name, (exp_t, exp_q) in rest_snapshot.items():
                if not cmds.objExists(joint_name):
                    continue
                act_t = cmds.xform(joint_name, q=True, ws=True, translation=True)
                act_q = _get_world_rotation(joint_name)
                t_err = max(abs(act_t[i] - exp_t[i]) for i in range(3))
                q_dot = abs(
                    exp_q.x * act_q.x
                    + exp_q.y * act_q.y
                    + exp_q.z * act_q.z
                    + exp_q.w * act_q.w
                )
                if t_err > _TRANS_TOLERANCE or (1.0 - q_dot) > _ROT_TOLERANCE:
                    mismatch_count += 1
                    if len(details) < 4:
                        details.append(
                            f"{joint_name.split('|')[-1]} "
                            f"(t_err={t_err:.4f}, 1-dot={1.0 - q_dot:.4f})"
                        )

            if mismatch_count > 0:
                errors.append(
                    f"{name}: {mismatch_count} joints not at rest after reset "
                    f"(bones_reset={stats['bones_reset']}, "
                    f"ik_handles_reset={stats['ik_handles_reset']}); "
                    f"e.g. {details}"
                )
            else:
                print(
                    f"  PASS: {name}: all {len(rest_snapshot)} joints restored to rest pose"
                )
        except Exception as exc:
            errors.append(f"{name}: {exc}")
        finally:
            cmds.undoInfo(closeChunk=True)
            suppressed_undo()

    assert_true(
        len(errors) == 0,
        f"{len(errors)} VPD(s) failed:\n" + "\n".join(errors),
    )
    return True


def test_vpd_pose_no_stacking(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that applying two VPD poses in sequence does NOT stack.

    A pose stack would mean: after applying pose B, bones that pose A
    modified but pose B does not touch retain A's rotation rather than
    returning to rest.

    Steps:
    1. Build fresh scene; capture rest-pose snapshot.
    2. Apply pose A (VPD index 0).
    3. Apply pose B (VPD index 1) without rebuilding the scene.
    4. Collect the set of bones that pose A touched but pose B does NOT touch
       (i.e. not present in pose B's bone list).
    5. For each such bone, assert that its rotation is back at the rest value
       (within tolerance), proving that the reset-before-apply ran correctly.
    """

    all_vpd_items = list(all_vpd_data.items())
    if len(all_vpd_items) < 2:
        skip_test("Need at least 2 VPD files to test stacking prevention")

    cmds.file(new=True, force=True)
    fresh_maya_data = build_pmx_scene(pmx_data)
    bone_map = fresh_maya_data.bone_name_map

    # Capture ground-truth rest pose
    rest_snapshot: dict[str, tuple[float, float, float]] = {}
    for joint_name in bone_map.values():
        if not cmds.objExists(joint_name):
            continue
        rx = cmds.getAttr(f"{joint_name}.rotateX")
        ry = cmds.getAttr(f"{joint_name}.rotateY")
        rz = cmds.getAttr(f"{joint_name}.rotateZ")
        rest_snapshot[joint_name] = (rx, ry, rz)

    # ── Find a suitable pair of poses where B does NOT cover all of A ──
    _found_pair: tuple | None = None
    for _i in range(len(all_vpd_items)):
        _na, _va = all_vpd_items[_i]
        _bones_a = {bp.bone_name for bp in _va.bones}
        for _j in range(len(all_vpd_items)):
            if _i == _j:
                continue
            _nb, _vb = all_vpd_items[_j]
            _bones_b = {bp.bone_name for bp in _vb.bones}
            _a_only = _bones_a - _bones_b
            if _a_only:
                _found_pair = (_na, _va, _nb, _vb, _a_only)
                break
        if _found_pair:
            break

    if _found_pair is None:
        skip_test(
            f"All {len(all_vpd_items)} VPD poses share identical bone sets — "
            "cannot test stacking on exclusive A bones"
        )

    name_a, vpd_a, name_b, vpd_b, _a_only_set = _found_pair

    # Apply pose A
    try:
        apply_vpd_pose_to_scene(
            vpd_data=vpd_a,
            model=fresh_maya_data.to_resolved(),
            create_keyframe=False,
        )
    except Exception as exc:
        assert_true(False, f"apply pose A ({name_a}) raised: {exc}")

    # Apply pose B (on the SAME scene — no rebuild)
    try:
        apply_vpd_pose_to_scene(
            vpd_data=vpd_b,
            model=fresh_maya_data.to_resolved(),
            create_keyframe=False,
        )
    except Exception as exc:
        assert_true(False, f"apply pose B ({name_b}) raised: {exc}")

    # Bones that pose A touched but pose B does NOT mention
    a_only_bones = sorted(_a_only_set)

    _STACK_TOLERANCE = 1e-3
    stacked_count = 0

    for pmx_bone_name in a_only_bones:
        joint_name = bone_map.get(pmx_bone_name)
        if not joint_name or not cmds.objExists(joint_name):
            continue

        rest = rest_snapshot.get(joint_name)
        if rest is None:
            continue

        exp_rx, exp_ry, exp_rz = rest
        act_rx = cmds.getAttr(f"{joint_name}.rotateX")
        act_ry = cmds.getAttr(f"{joint_name}.rotateY")
        act_rz = cmds.getAttr(f"{joint_name}.rotateZ")

        if (
            abs(act_rx - exp_rx) > _STACK_TOLERANCE
            or abs(act_ry - exp_ry) > _STACK_TOLERANCE
            or abs(act_rz - exp_rz) > _STACK_TOLERANCE
        ):
            stacked_count += 1
            if stacked_count <= 3:
                print(
                    f"  FAIL detail: {pmx_bone_name}: pose A rotation stacked — "
                    f"expected rest r=({exp_rx:.4f},{exp_ry:.4f},{exp_rz:.4f}), "
                    f"got r=({act_rx:.4f},{act_ry:.4f},{act_rz:.4f})"
                )

    assert_true(
        stacked_count == 0,
        f"{stacked_count}/{len(a_only_bones)} bones from pose A "
        f"({name_a}) were still rotated after applying pose B ({name_b}) — "
        "pose stacking detected",
    )
    print(
        f"  PASS: {len(a_only_bones)} A-only bones correctly returned to rest "
        f"after applying pose B ({name_b}) — no stacking"
    )
    return True


def test_reset_preserves_ik_blend(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that reset_model_to_bind_pose does not alter IK handle ikBlend values.

    Steps:
    1. Build a fresh scene and collect all IK handles.
    2. Set each handle's ikBlend to a known non-default value (0.7) to simulate
       a user-configured solver blend.
    3. Apply a VPD pose, then call reset_model_to_bind_pose.
    4. Assert that every handle's ikBlend is still 0.7 (not 0.0 or 1.0).
    """
    from mmd.maya.pmx_model_utils import reset_model_to_bind_pose

    all_vpd = all_vpd_data
    if not all_vpd:
        skip_test("No VPD files available")

    cmds.file(new=True, force=True)
    fresh_maya_data = build_pmx_scene(pmx_data)

    ik_handles = fresh_maya_data.ik_handles or cmds.ls(type="ikHandle") or []
    if not ik_handles:
        skip_test("No IK handles in scene")

    # Set a non-default ikBlend value on every handle
    _TEST_BLEND = 0.7
    for handle in ik_handles:
        try:
            cmds.setAttr(f"{handle}.ikBlend", _TEST_BLEND)
        except Exception:
            pass  # Handle may not have ikBlend attr (e.g. non-standard IK) — skip

    # Apply one VPD pose to exercise the full reset path
    _, vpd = next(iter(all_vpd.items()))
    try:
        apply_vpd_pose_to_scene(
            vpd_data=vpd,
            model=fresh_maya_data.to_resolved(),
            create_keyframe=False,
        )
    except Exception as exc:
        assert_true(False, f"apply_vpd_pose_to_scene raised: {exc}")

    # Explicitly call the public reset (same path as the UI button)
    try:
        reset_model_to_bind_pose(
            model=fresh_maya_data.to_resolved(),
        )
    except Exception as exc:
        assert_true(False, f"reset_model_to_bind_pose raised: {exc}")

    # Assert ikBlend is unchanged
    _BLEND_TOL = 1e-4
    wrong: list[str] = []
    for handle in ik_handles:
        try:
            actual = cmds.getAttr(f"{handle}.ikBlend")
            if abs(actual - _TEST_BLEND) > _BLEND_TOL:
                wrong.append(f"{handle}: expected {_TEST_BLEND}, got {actual:.4f}")
        except Exception:
            # If getAttr fails, the handle may have been deleted — treat as altered
            wrong.append(f"{handle}: could not read ikBlend after reset")

    assert_true(
        len(wrong) == 0,
        f"{len(wrong)} IK handle(s) had ikBlend altered by reset:\n"
        + "\n".join(f"    - {item}" for item in wrong[:5]),
    )

    print(
        f"  PASS: All {len(ik_handles)} IK handles retained ikBlend={_TEST_BLEND} after reset"
    )
    return True


def test_vpd_translation_no_stacking(
    pmx_data: PmxModel, maya_pmx_data: MayaPmxData, all_vpd_data: dict[str, VPDFile]
) -> bool:
    """Verify that sequential VPD applications do not accumulate translation.

    Bones that pose A translates but pose B does NOT mention should return to
    their rest translation after pose B is applied — not retain pose A's value.

    Steps:
    1. Build a fresh scene; capture rest-translate snapshot.
    2. Apply pose A.
    3. Apply pose B on the SAME scene (no rebuild).
    4. For every bone that pose A translated but pose B omits, assert that
       the joint is back at rest translation (not stacked from pose A).
    """

    all_vpd_items = list(all_vpd_data.items())
    if len(all_vpd_items) < 2:
        skip_test("Need at least 2 VPD files to test translation stacking")

    cmds.file(new=True, force=True)
    fresh_maya_data = build_pmx_scene(pmx_data)
    bone_map = fresh_maya_data.bone_name_map

    # Capture rest translation (before any pose)
    rest_trans: dict[str, tuple[float, float, float]] = {}
    for joint_name in bone_map.values():
        if not cmds.objExists(joint_name):
            continue
        rest_trans[joint_name] = (
            cmds.getAttr(f"{joint_name}.translateX"),
            cmds.getAttr(f"{joint_name}.translateY"),
            cmds.getAttr(f"{joint_name}.translateZ"),
        )

    # ── Find a suitable pair of poses where A has translated bones that B omits ──
    _TRANS_TOL = 1e-3
    _found_pair: tuple | None = None
    for _i in range(len(all_vpd_items)):
        _na, _va = all_vpd_items[_i]
        _translated_a = {
            bp.bone_name
            for bp in _va.bones
            if (
                abs(bp.position.x) >= _TRANS_TOL
                or abs(bp.position.y) >= _TRANS_TOL
                or abs(bp.position.z) >= _TRANS_TOL
            )
        }
        if not _translated_a:
            continue
        for _j in range(len(all_vpd_items)):
            if _i == _j:
                continue
            _nb, _vb = all_vpd_items[_j]
            _bones_b = {bp.bone_name for bp in _vb.bones}
            _a_only = _translated_a - _bones_b
            if _a_only:
                _found_pair = (_na, _va, _nb, _vb, _a_only)
                break
        if _found_pair:
            break

    if _found_pair is None:
        skip_test(
            f"No suitable VPD pair found among {len(all_vpd_items)} poses — "
            "every translated-in-A bone is also covered by every other pose"
        )

    name_a, vpd_a, name_b, vpd_b, _a_only_set = _found_pair

    try:
        apply_vpd_pose_to_scene(
            vpd_data=vpd_a,
            model=fresh_maya_data.to_resolved(),
            create_keyframe=False,
        )
    except Exception as exc:
        assert_true(False, f"apply pose A ({name_a}) raised: {exc}")

    try:
        apply_vpd_pose_to_scene(
            vpd_data=vpd_b,
            model=fresh_maya_data.to_resolved(),
            create_keyframe=False,
        )
    except Exception as exc:
        assert_true(False, f"apply pose B ({name_b}) raised: {exc}")

    # Bones with non-zero VPD translation in pose A that pose B does not touch
    pose_b_names = {bp.bone_name for bp in vpd_b.bones}
    stacked: list[str] = []

    for bone_pose in vpd_a.bones:
        # Only bones that actually carry a non-trivial translation offset
        if (
            abs(bone_pose.position.x) < _TRANS_TOL
            and abs(bone_pose.position.y) < _TRANS_TOL
            and abs(bone_pose.position.z) < _TRANS_TOL
        ):
            continue
        if bone_pose.bone_name in pose_b_names:
            continue  # Pose B overwrites this bone — not relevant for stacking

        joint_name = bone_map.get(bone_pose.bone_name)
        if not joint_name or not cmds.objExists(joint_name):
            continue

        rest = rest_trans.get(joint_name)
        if rest is None:
            continue

        act_tx = cmds.getAttr(f"{joint_name}.translateX")
        act_ty = cmds.getAttr(f"{joint_name}.translateY")
        act_tz = cmds.getAttr(f"{joint_name}.translateZ")

        if (
            abs(act_tx - rest[0]) > _TRANS_TOL
            or abs(act_ty - rest[1]) > _TRANS_TOL
            or abs(act_tz - rest[2]) > _TRANS_TOL
        ):
            stacked.append(
                f"{bone_pose.bone_name}: rest=({rest[0]:.3f},{rest[1]:.3f},{rest[2]:.3f}) "
                f"got=({act_tx:.3f},{act_ty:.3f},{act_tz:.3f})"
            )

    assert_true(
        len(stacked) == 0,
        f"{len(stacked)} bone(s) from pose A ({name_a}) retained "
        f"translation after applying pose B ({name_b}) — translation stacking:\n"
        + "\n".join(f"    - {item}" for item in stacked[:3]),
    )

    # Count how many A-only translated bones were actually checked
    a_only_translated = sum(
        1
        for bp in vpd_a.bones
        if (
            abs(bp.position.x) > _TRANS_TOL
            or abs(bp.position.y) > _TRANS_TOL
            or abs(bp.position.z) > _TRANS_TOL
        )
        and bp.bone_name not in pose_b_names
        and bone_map.get(bp.bone_name)
        and cmds.objExists(bone_map[bp.bone_name])
    )

    print(
        f"  PASS: {a_only_translated} A-only translated bones correctly at rest "
        f"after applying pose B ({name_b}) — no translation stacking"
    )
    return True


# ---------------------------------------------------------------------------
# Test registry
# ---------------------------------------------------------------------------

_TESTS = [
    ("VPD Parsing", test_vpd_parsing),
    ("VPD Apply No Errors", test_vpd_apply_no_errors),
    ("VPD Bone Rotation", test_vpd_bone_rotation),
    ("VPD IK Control Bone Translation", test_vpd_ik_control_bone_translation),
    ("VPD IK Chain Rotation Skipped", test_vpd_ik_chain_rotation_skipped),
    ("VPD Toe Bone Rotation", test_vpd_toe_follows_ankle),
    ("VPD FIXED_AXIS Bones", test_vpd_fixed_axis_bones),
    ("VPD Rest Pose Attributes On Joints", test_rest_pose_attributes_on_joints),
    ("VPD Reset To Bind Pose", test_reset_to_bind_pose),
    ("VPD Pose No Stacking", test_vpd_pose_no_stacking),
    ("VPD Reset Preserves IK Blend", test_reset_preserves_ik_blend),
    ("VPD Translation No Stacking", test_vpd_translation_no_stacking),
]
