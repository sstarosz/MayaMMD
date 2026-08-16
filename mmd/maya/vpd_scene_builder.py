"""VPD scene builder for Maya.

This module applies VPD (Vocaloid Pose Data) to PMX models that
were imported into Maya. VPD files contain static pose data for bones.
"""

import logging
import math
from typing import NamedTuple

import maya.api.OpenMaya as om
from maya import cmds

from mmd.core.data_types import Vec3, Vec4, VPDFile
from mmd.maya.maya_data_types import ResolvedModelData
from mmd.maya.pmx_model_utils import (
    collect_ik_chain_joints,
    reset_all_bones_to_rest_pose,
    set_joint_rotate_safe,
    set_joint_translate_safe,
)

log = logging.getLogger(__name__)


class _JointRestState(NamedTuple):
    bind_pose: Vec3
    world_rest: om.MQuaternion
    parent_world_rest: om.MQuaternion
    has_fixed_axis: bool


def _get_world_rotation(node_name: str) -> om.MQuaternion:
    matrix = om.MMatrix(cmds.xform(node_name, query=True, worldSpace=True, matrix=True))
    return om.MTransformationMatrix(matrix).rotation(asQuaternion=True)


def _bone_has_local_coordinate(joint_name: str) -> bool:
    """Check if the bone carries the LOCAL_COORDINATE flag."""
    try:
        if cmds.attributeQuery("pmxUseLocalCoordinate", node=joint_name, exists=True):
            return cmds.getAttr(f"{joint_name}.pmxUseLocalCoordinate")
    except Exception as exc:
        log.debug("Error querying local coordinate for %s: %s", joint_name, exc)
    return False


def _bone_has_fixed_axis(joint_name: str) -> bool:
    """Check if the bone uses Fixed Axis (single-axis rotation constraint)."""
    try:
        if cmds.attributeQuery("pmxUseFixedAxis", node=joint_name, exists=True):
            return cmds.getAttr(f"{joint_name}.pmxUseFixedAxis")
    except Exception as exc:
        log.debug("Error querying fixed axis for %s: %s", joint_name, exc)
    return False


def _capture_joint_rest_state(joint_name: str) -> _JointRestState:
    bind_pose = Vec3(
        cmds.getAttr(f"{joint_name}.translateX"),
        cmds.getAttr(f"{joint_name}.translateY"),
        cmds.getAttr(f"{joint_name}.translateZ"),
    )
    parent = cmds.listRelatives(joint_name, parent=True, fullPath=True)
    parent_world_rest = _get_world_rotation(parent[0]) if parent else om.MQuaternion()

    # For LOCAL_COORDINATE bones, MMD applies VMD motion ignoring the bone's local axes.
    # Therefore, its effective rest orientation for the purpose of the similarity transform
    # should be its parent's world rest.
    is_local = _bone_has_local_coordinate(joint_name)
    world_rest = parent_world_rest if is_local else _get_world_rotation(joint_name)

    return _JointRestState(
        bind_pose=bind_pose,
        world_rest=world_rest,
        parent_world_rest=parent_world_rest,
        has_fixed_axis=_bone_has_fixed_axis(joint_name),
    )


def _extract_local_x_twist_degrees(quat: om.MQuaternion) -> float:
    """Extract the signed twist angle around the joint's local X axis.

    FIXED_AXIS bones are authored so their constrained axis is aligned to the
    Maya joint's local X axis during PMX import. For these bones we want the
    pure twist component only; converting the projected quaternion back through
    Euler can still reintroduce an equivalent-angle flip near 180 degrees.
    """
    length = math.hypot(quat.x, quat.w)
    if length <= 1e-8:
        return 0.0

    twist_x = quat.x / length
    twist_w = quat.w / length
    return math.degrees(2.0 * math.atan2(twist_x, twist_w))


def _normalize_quaternion_components(
    quat: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Normalize quaternion components."""
    x, y, z, w = quat
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if magnitude <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / magnitude, y / magnitude, z / magnitude, w / magnitude)


def _convert_vpd_quaternion_to_maya_components(
    quat: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Convert a VPD quaternion from MMD space (left-handed, Y-up, Z-forward)
    to Maya space (right-handed, Y-up, Z-forward).

    Reflecting a rotation across the XY plane (flipping Z coordinate) transforms
    the rotation axis direction AND reverses rotation handedness, which mathematically
    negates the X and Y components of the quaternion: (-x, -y, z, w).
    """
    x, y, z, w = quat
    return _normalize_quaternion_components((-x, -y, z, w))


def _get_joint_rotation_order_constant(rot_order: int) -> int:
    """Convert Maya rotation order integer to MEulerRotation constant."""
    rotation_order_map = [
        om.MEulerRotation.kXYZ,
        om.MEulerRotation.kYZX,
        om.MEulerRotation.kZXY,
        om.MEulerRotation.kXZY,
        om.MEulerRotation.kYXZ,
        om.MEulerRotation.kZYX,
    ]
    if 0 <= rot_order < len(rotation_order_map):
        return rotation_order_map[rot_order]
    return om.MEulerRotation.kXYZ


def _rotation_degrees_from_vpd_quaternion(
    joint_name: str,
    quat: Vec4,
    q_world_rest: om.MQuaternion | None = None,
    has_fixed_axis: bool = False,
) -> tuple[float, float, float]:
    """Convert a VPD quaternion into Maya rotate-channel Euler degrees.

    VPD rotations appear to be stored as offsets/deltas from the bone's bind pose
    in MMD's space. We need to:
    1. Convert from MMD quaternion to Maya quaternion
    2. Apply similarity transform to get the rotation in Maya's world space
    3. Convert to Euler angles

    Returns:
        Tuple of (rx, ry, rz) in degrees
    """

    if q_world_rest is None:
        current_time = cmds.currentTime(query=True)
        try:
            cmds.currentTime(0)
            q_world_rest = _get_world_rotation(joint_name)
        except Exception:
            q_world_rest = om.MQuaternion()
        finally:
            try:
                cmds.currentTime(current_time)
            except Exception:
                pass

    # Convert VPD quaternion from MMD space to Maya space
    q_target = om.MQuaternion(
        *_convert_vpd_quaternion_to_maya_components((quat.x, quat.y, quat.z, quat.w))
    )

    # Apply similarity transform: convert from rest-space to Maya joint space
    q_R = q_world_rest * q_target * q_world_rest.inverse()

    if has_fixed_axis:
        return (_extract_local_x_twist_degrees(q_R), 0.0, 0.0)

    # Convert to Euler angles using the joint's rotation order
    euler = q_R.asEulerRotation()
    rot_order = cmds.getAttr(f"{joint_name}.rotateOrder")
    euler = euler.reorder(_get_joint_rotation_order_constant(rot_order))

    return (
        math.degrees(euler.x),
        math.degrees(euler.y),
        math.degrees(euler.z),
    )


def _calculate_local_translation(
    joint_name: str,
    position: Vec3,
) -> tuple[float, float, float]:
    """Convert VPD translation to Maya local space.

    VPD translations appear to be offsets/deltas from bind pose in MMD space.
    Simply convert coordinate system and add to bind pose.

    Returns:
        Tuple of (tx, ty, tz) in local space
    """
    # Get current bind pose (rest translation)
    bind_pose = Vec3(
        cmds.getAttr(f"{joint_name}.translateX"),
        cmds.getAttr(f"{joint_name}.translateY"),
        cmds.getAttr(f"{joint_name}.translateZ"),
    )

    # VPD position is offset in MMD space - convert to Maya space (flip Z)
    # and add to bind pose
    return (
        bind_pose.x + position.x,
        bind_pose.y + position.y,
        bind_pose.z - position.z,  # Flip Z for MMD->Maya conversion
    )


def apply_vpd_pose_to_scene(
    vpd_data: VPDFile,
    model: "ResolvedModelData",
    create_keyframe: bool = False,
) -> None:
    """
    Apply VPD pose data to a PMX model in the Maya scene.

    All model data (bone map, IK handles) must be resolved beforehand into
    *model* — this function does NOT query the Maya scene for metadata.

    Args:
        vpd_data: VPD file data containing bone poses.
        model: Resolved model data (bone map, IK handles).
        create_keyframe: If True, create a keyframe at the current frame; if False, just set the pose
    """
    bone_map = model.bone_map
    log.debug(
        "Applying VPD pose (%d bones) to model: %s (create_keyframe=%s)",
        len(bone_map),
        model.root_name,
        create_keyframe,
    )

    if not bone_map:
        log.error("Bone map is empty - cannot apply pose")
        return

    unmatched_bones: set[str] = set()

    _ik_handles_list = model.ik_handles

    # Capture original ikBlend values so we restore exactly what the user had
    # (ikBlend is user-visible / may be keyed; always writing 1.0 would corrupt it).
    _ik_blend_orig: dict[str, float] = {}
    for _ik_h in _ik_handles_list:
        try:
            _ik_blend_orig[_ik_h] = cmds.getAttr(f"{_ik_h}.ikBlend")
        except Exception:
            _ik_blend_orig[_ik_h] = 1.0

    auto_key_state = cmds.autoKeyframe(query=True, state=True)
    # Sentinel: holds the saved timeline position while we're away from it;
    # set to None once successfully restored so the finally block is a no-op.
    orig_time: float | None = None

    # Suspend viewport refresh to prevent T-pose flash during reset/apply.
    # The viewport will only update once at the end with the final pose.
    cmds.refresh(suspend=True)

    try:
        # Disable auto-key temporarily
        if auto_key_state:
            cmds.autoKeyframe(state=False)

        # Disable IK blend so FK setAttr values are uncontested by the solver
        for _ik_h in _ik_handles_list:
            try:
                cmds.setAttr(f"{_ik_h}.ikBlend", 0.0)
            except Exception:
                pass

        # Reset all bones (IK disabled → no IK override on chain joints)
        reset_all_bones_to_rest_pose(bone_map)

        # Restore original IK blend — chain is at rest so the solver is
        # trivially satisfied and the user's blend value is preserved.
        for _ik_h, _orig_blend in _ik_blend_orig.items():
            try:
                cmds.setAttr(f"{_ik_h}.ikBlend", _orig_blend)
            except Exception:
                pass

        # Collect joints that are part of IK chains — the IK solver controls
        # their rotation and we must NOT override it with FK values.
        ik_chain_joints = collect_ik_chain_joints(_ik_handles_list)

        # Pre-capture ALL joint rest states at frame 0 before the VPD loop.
        # If we capture incrementally inside the loop, earlier VPD operations
        # on parent bones will corrupt the similarity transform for children.
        orig_time = cmds.currentTime(query=True)
        cmds.currentTime(0)
        rest_states: dict[str, _JointRestState] = {}
        for maya_joint_name in set(bone_map.values()):
            try:
                rest_states[maya_joint_name] = _capture_joint_rest_state(
                    maya_joint_name
                )
            except Exception as exc:
                log.warning(
                    "Could not capture rest state for %s: %s", maya_joint_name, exc
                )
        cmds.currentTime(orig_time)
        orig_time = None  # successfully restored; clear sentinel

        for bone_pose in vpd_data.bones:
            vpd_bone_name = bone_pose.bone_name

            # Find matching Maya joint
            maya_joint_name = bone_map.get(vpd_bone_name)

            if not maya_joint_name or not cmds.objExists(maya_joint_name):
                if vpd_bone_name not in unmatched_bones:
                    log.debug("VPD bone not found in scene: %s", vpd_bone_name)
                    unmatched_bones.add(vpd_bone_name)
                continue

            # Try to apply translation and rotation
            try:
                joint_rest_state = rest_states.get(maya_joint_name)

                # Convert VPD rotation to Maya Euler angles
                local_rot = _rotation_degrees_from_vpd_quaternion(
                    joint_name=maya_joint_name,
                    quat=bone_pose.rotation,
                    q_world_rest=(
                        joint_rest_state.world_rest if joint_rest_state else None
                    ),
                    has_fixed_axis=(
                        joint_rest_state.has_fixed_axis if joint_rest_state else False
                    ),
                )

                # Apply translation — all bones (including IK control bones)
                # are animated directly as joints. IK handles are parented
                # under the control bone and are not animated directly.
                local_trans = _calculate_local_translation(
                    maya_joint_name, bone_pose.position
                )
                trans_applied = set_joint_translate_safe(
                    maya_joint_name,
                    local_trans[0],
                    local_trans[1],
                    local_trans[2],
                )

                # Apply rotation — skip for IK chain joints because the IK solver
                # controls their rotation directly; setting FK values would conflict.
                use_ik_chain = maya_joint_name in ik_chain_joints
                rot_applied = False
                if not use_ik_chain:
                    rot_applied = set_joint_rotate_safe(
                        maya_joint_name, local_rot[0], local_rot[1], local_rot[2]
                    )

                # Create keyframe if requested and something was applied
                if create_keyframe:
                    if trans_applied:
                        try:
                            cmds.setKeyframe(
                                maya_joint_name,
                                attribute=["translateX", "translateY", "translateZ"],
                            )
                        except Exception:
                            log.warning(
                                "Failed to set keyframe for translation on %s",
                                maya_joint_name,
                            )
                    if rot_applied:
                        try:
                            cmds.setKeyframe(
                                maya_joint_name,
                                attribute=["rotateX", "rotateY", "rotateZ"],
                            )
                        except Exception:
                            log.warning(
                                "Failed to set keyframe for rotation on %s",
                                maya_joint_name,
                            )

                if not (trans_applied or rot_applied):
                    log.debug("Joint attributes locked, skipping: %s", maya_joint_name)

            except Exception as e:
                log.warning("Failed to apply pose to bone %s: %s", maya_joint_name, e)

        if unmatched_bones:
            log.debug(
                "VPD bones not found in scene (%d): %s",
                len(unmatched_bones),
                ", ".join(sorted(list(unmatched_bones)[:5]))
                + (
                    f" and {len(unmatched_bones) - 5} more"
                    if len(unmatched_bones) > 5
                    else ""
                ),
            )

    finally:
        # Restore timeline if an exception interrupted rest-state capture
        if orig_time is not None:
            try:
                cmds.currentTime(orig_time)
            except Exception:
                pass

        # Restore original ikBlend values — guards against exceptions that
        # prevented the mid-flow restore (after bone reset) from running.
        for _ik_h, _orig_blend in _ik_blend_orig.items():
            try:
                cmds.setAttr(f"{_ik_h}.ikBlend", _orig_blend)
            except Exception:
                pass

        # Restore auto-key state
        if auto_key_state:
            cmds.autoKeyframe(state=True)

        # Resume viewport refresh - now the viewport updates once with the final pose
        cmds.refresh(suspend=False)
