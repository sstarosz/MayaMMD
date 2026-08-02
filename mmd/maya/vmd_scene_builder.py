"""VMD scene builder for Maya.

This module applies VMD (Vocaloid Motion Data) animation to PMX models that
were imported into Maya.
"""

import math
import logging
from collections import defaultdict
from typing import Dict, List, NamedTuple, Optional, Tuple

import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
import maya.cmds as cmds

from mmd.core.data_types import Vec3, Vec4, VMDFile
from mmd.maya.maya_data_types import ResolvedModelData

log = logging.getLogger(__name__)

_ROTATION_ORDER_TO_EULER = [
    om.MEulerRotation.kXYZ,
    om.MEulerRotation.kYZX,
    om.MEulerRotation.kZXY,
    om.MEulerRotation.kXZY,
    om.MEulerRotation.kYXZ,
    om.MEulerRotation.kZYX,
]


class _JointRestState(NamedTuple):
    bind_pose: Vec3
    world_rest: om.MQuaternion
    parent_world_rest: om.MQuaternion
    has_fixed_axis: bool


def _normalize_quaternion_components(
    quat: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    import math

    x, y, z, w = quat
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if magnitude <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / magnitude, y / magnitude, z / magnitude, w / magnitude)


def _convert_vmd_quaternion_to_maya_components(
    quat: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Convert a VMD quaternion from MMD space (left-handed, Y-up, Z-forward)
    to Maya space (right-handed, Y-up, Z-forward).

    Reflecting a rotation across the XY plane (flipping Z coordinate) transforms
    the rotation axis direction AND reverses rotation handedness, which mathematically
    negates the X and Y components of the quaternion: (-x, -y, z, w).
    """
    x, y, z, w = quat
    return _normalize_quaternion_components((-x, -y, z, w))


def _quaternion_from_vmd(
    joint_name: str,
    quat: Vec4,
    q_world_rest: Optional[om.MQuaternion] = None,
    has_fixed_axis: bool = False,
) -> Tuple[float, float, float, float]:
    """Convert a VMD quaternion to raw Maya quaternion components (qx, qy, qz, qw).

    This applies the similarity transform to convert from world-space to
    the joint's local space, matching ``_rotation_degrees_from_vmd_quaternion``
    but returning quaternion components instead of Euler angles.

    Returns:
        Tuple of (qx, qy, qz, qw) in Maya coordinate space, normalized.
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

    q_target = om.MQuaternion(
        *_convert_vmd_quaternion_to_maya_components((quat.x, quat.y, quat.z, quat.w))
    )

    # Similarity transform: convert from world to joint's local space
    q_R = q_world_rest * q_target * q_world_rest.inverse()

    if has_fixed_axis:
        # Fixed-axis bones: only extract X-axis twist
        length = math.hypot(q_R.x, q_R.w)
        if length <= 1e-8:
            return (0.0, 0.0, 0.0, 1.0)
        # Represent the twist as a quaternion around local X
        half_angle = math.atan2(q_R.x / length, q_R.w / length)
        return (math.sin(half_angle), 0.0, 0.0, math.cos(half_angle))

    return (q_R.x, q_R.y, q_R.z, q_R.w)


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


def _get_joint_rotation_order_constant(rot_order: int) -> int:
    if 0 <= rot_order < len(_ROTATION_ORDER_TO_EULER):
        return _ROTATION_ORDER_TO_EULER[rot_order]
    return om.MEulerRotation.kXYZ


def _bone_has_fixed_axis(joint_name: str) -> bool:
    """Check if the bone uses Fixed Axis (single-axis rotation constraint)."""
    try:
        if cmds.attributeQuery("pmxUseFixedAxis", node=joint_name, exists=True):
            return cmds.getAttr(f"{joint_name}.pmxUseFixedAxis")
    except Exception as exc:
        log.debug("Error querying fixed axis for %s: %s", joint_name, exc)
    return False


def _bone_has_local_coordinate(joint_name: str) -> bool:
    """Check if the bone carries the LOCAL_COORDINATE flag."""
    try:
        if cmds.attributeQuery("pmxUseLocalCoordinate", node=joint_name, exists=True):
            return cmds.getAttr(f"{joint_name}.pmxUseLocalCoordinate")
    except Exception as exc:
        log.debug("Error querying local coordinate for %s: %s", joint_name, exc)
    return False


def _get_world_rotation(node_name: str) -> om.MQuaternion:
    matrix = om.MMatrix(cmds.xform(node_name, query=True, worldSpace=True, matrix=True))
    return om.MTransformationMatrix(matrix).rotation(asQuaternion=True)


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


def _calculate_local_translation(
    position: Vec3,
    bind_pose: Optional[Vec3],
    parent_world_rest: om.MQuaternion,
) -> Tuple[float, float, float]:
    """Calculate local translation from VMD world-space position.

    Returns tuple for direct use with Maya batch keyframe API.
    """
    local_offset = om.MVector(position.x, position.y, -position.z).rotateBy(
        parent_world_rest.inverse()
    )
    if bind_pose is None:
        return (local_offset.x, local_offset.y, local_offset.z)
    return (
        bind_pose.x + local_offset.x,
        bind_pose.y + local_offset.y,
        bind_pose.z + local_offset.z,
    )


# NOTE: Kept as a reference implementation — used by integration tests
# (tests/integration/maya/test_vmd_integration.py) to compute expected Euler
# values. Not called from the production keyframe path (quaternion SLERP
# curves are always used).
def _rotation_degrees_from_vmd_quaternion(
    joint_name: str,
    quat: Vec4,
    q_world_rest: Optional[om.MQuaternion] = None,
    has_fixed_axis: bool = False,
) -> Tuple[float, float, float]:
    """Convert a VMD quaternion into Maya rotate-channel Euler degrees.

    Returns tuple for direct use with Maya batch keyframe API.
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

    q_target = om.MQuaternion(
        *_convert_vmd_quaternion_to_maya_components((quat.x, quat.y, quat.z, quat.w))
    )

    # Apply similarity transform to convert from world space to joint's local space.
    # This compensates for the jointOrient that was set during bone creation.
    q_R = q_world_rest * q_target * q_world_rest.inverse()

    if has_fixed_axis:
        return (_extract_local_x_twist_degrees(q_R), 0.0, 0.0)

    # Normal and Local Coordinate bones: Use standard Euler conversion
    euler = q_R.asEulerRotation()
    euler = euler.reorder(
        _get_joint_rotation_order_constant(cmds.getAttr(f"{joint_name}.rotateOrder"))
    )

    return (
        math.degrees(euler.x),
        math.degrees(euler.y),
        math.degrees(euler.z),
    )


def get_or_create_anim_curve(node_name: str, attribute: str) -> oma.MFnAnimCurve:
    """Get existing animation curve or create a new one.

    Checks if the attribute already has an animCurve connected and returns it.
    If not, creates a new animCurve connected directly to the plug.

    Args:
        node_name: Name of the node (e.g., joint)
        attribute: Attribute name (e.g., 'translateX', 'rotateY')

    Returns:
        MFnAnimCurve function set for the animation curve
    """
    # Get the plug for this attribute
    sel = om.MSelectionList()
    try:
        sel.add(f"{node_name}.{attribute}")
        plug = sel.getPlug(0)
    except Exception as e:
        log.error(f"[BATCH] Could not get plug for {node_name}.{attribute}: {e}")
        raise

    # Check if the plug already has an animCurve connected
    if plug.isDestination:
        src_plug = plug.source()
        src_node = src_plug.node()
        if src_node.hasFn(om.MFn.kAnimCurve):
            log.debug("Found existing animation curve for %s.%s", node_name, attribute)
            return oma.MFnAnimCurve(src_node)

    # Create a new animation curve directly on the plug
    if attribute.startswith("translate"):
        curve_type = oma.MFnAnimCurve.kAnimCurveTL
    elif attribute.startswith("rotate"):
        curve_type = oma.MFnAnimCurve.kAnimCurveTA
    else:
        curve_type = oma.MFnAnimCurve.kAnimCurveTU

    dg_mod = om.MDGModifier()
    anim_curve_fn = oma.MFnAnimCurve()
    anim_curve_fn.create(plug, curve_type, dg_mod)
    dg_mod.doIt()
    log.debug("Created new animation curve for %s.%s", node_name, attribute)
    return anim_curve_fn


def _ensure_quaternion_continuity(
    quat_series: List[Tuple[float, float, float, float]],
) -> List[Tuple[float, float, float, float]]:
    """Ensure quaternion sign continuity across keyframes.

    Quaternions ``q`` and ``-q`` represent the same rotation.  Maya's
    quaternion SLERP takes the shortest path between two keyframes, which
    can flip hemispheres if consecutive quaternions live on opposite sides
    of the 4D hypersphere.  This function negates any quaternion whose dot
    product with the previous result is negative, keeping all keyframes on
    the same hemisphere so SLERP always takes the intended path.

    Args:
        quat_series: List of (qx, qy, qz, qw) tuples sorted by frame.

    Returns:
        Sign-continuous list of quaternion tuples.
    """
    if len(quat_series) <= 1:
        return list(quat_series)

    result: List[Tuple[float, float, float, float]] = [quat_series[0]]
    for i in range(1, len(quat_series)):
        prev = result[-1]
        curr = quat_series[i]
        dot = (
            prev[0] * curr[0]
            + prev[1] * curr[1]
            + prev[2] * curr[2]
            + prev[3] * curr[3]
        )
        if dot < 0.0:
            result.append((-curr[0], -curr[1], -curr[2], -curr[3]))
        else:
            result.append(curr)
    return result


def batch_set_quaternion_keyframes(
    joint_name: str,
    frames: List[int],
    translate_values: List[Tuple[float, float, float]],
    quaternion_values: List[Tuple[float, float, float, float]],
    skip_translation: bool = False,
    skip_rotation: bool = False,
) -> None:
    """Batch set quaternion-SLERP keyframes for a joint.

    Stores raw quaternion components in ``kAnimCurveTA`` curves and
    configures Maya to SLERP between them instead of linear Euler
    interpolation.  This eliminates foot-through-thigh artifacts that
    occur when sparse Euler keyframes take a physically impossible path.

    Requires Maya 2024+ for ``rotationInterpolation(convert='quaternionSlerp')``.

    Args:
        joint_name: Name of the Maya joint.
        frames: List of frame numbers (ints).
        translate_values: List of (tx, ty, tz) tuples (unchanged, linear).
        quaternion_values: List of (qx, qy, qz, qw) tuples (per frame).
        skip_translation: If True, do not create translate animation curves.
        skip_rotation: If True, do not create rotate animation curves.
    """
    if not frames:
        return

    frame_time_pairs = sorted(
        zip(frames, translate_values, quaternion_values), key=lambda x: x[0]
    )
    sorted_frames = [f for f, _, _ in frame_time_pairs]
    sorted_translates = [t for _, t, _ in frame_time_pairs]
    sorted_quats = [q for _, _, q in frame_time_pairs]

    time_array = om.MTimeArray()
    for frame in sorted_frames:
        time_array.append(om.MTime(frame, om.MTime.uiUnit()))

    # --- Translation (linear interpolation — avoids overshoot from auto-tangent) ---
    if not skip_translation:
        tx_values = om.MDoubleArray([tv[0] for tv in sorted_translates])
        ty_values = om.MDoubleArray([tv[1] for tv in sorted_translates])
        tz_values = om.MDoubleArray([tv[2] for tv in sorted_translates])
        trans_attributes = [
            ("translateX", tx_values),
            ("translateY", ty_values),
            ("translateZ", tz_values),
        ]
        for attr_name, values in trans_attributes:
            try:
                anim_curve = get_or_create_anim_curve(joint_name, attr_name)
                _clear_anim_curve_keys(anim_curve)
                anim_curve.addKeys(
                    time_array,
                    values,
                    oma.MFnAnimCurve.kTangentLinear,
                    oma.MFnAnimCurve.kTangentLinear,
                )
            except Exception as e:
                log.error(
                    "[QUAT-BATCH] Failed to set %s keys for %s: %s",
                    attr_name,
                    joint_name,
                    e,
                )

    # --- Rotation (quaternion SLERP) ---
    if not skip_rotation:
        qx_values = om.MDoubleArray([qv[0] for qv in sorted_quats])
        qy_values = om.MDoubleArray([qv[1] for qv in sorted_quats])
        qz_values = om.MDoubleArray([qv[2] for qv in sorted_quats])
        qw_list = [qv[3] for qv in sorted_quats]

        rotate_attrs = [
            ("rotateX", qx_values),
            ("rotateY", qy_values),
            ("rotateZ", qz_values),
        ]

        # Create / get the three rotate curves and switch to quaternion mode.
        curves: List[oma.MFnAnimCurve] = []
        for attr_name, values in rotate_attrs:
            try:
                anim_curve = get_or_create_anim_curve(joint_name, attr_name)
                _clear_anim_curve_keys(anim_curve)
                curves.append(anim_curve)
            except Exception as e:
                log.error(
                    "[QUAT-BATCH] Failed to get curve for %s.%s: %s",
                    joint_name,
                    attr_name,
                    e,
                )
                curves.clear()
                break

        if len(curves) == 3:
            try:
                cmds.rotationInterpolation(
                    f"{joint_name}.rotateX",
                    f"{joint_name}.rotateY",
                    f"{joint_name}.rotateZ",
                    convert="quaternionSlerp",
                )
            except Exception as e:
                log.warning(
                    "[QUAT-BATCH] rotationInterpolation failed for %s: %s",
                    joint_name,
                    e,
                )

            # Store quaternion components — convertUnits=False prevents Maya
            # from treating the values as radians.
            for idx, (attr_name, values) in enumerate(rotate_attrs):
                try:
                    curves[idx].addKeysWithTangents(
                        time_array,
                        values,
                        oma.MFnAnimCurve.kTangentLinear,
                        oma.MFnAnimCurve.kTangentLinear,
                        convertUnits=False,
                    )
                except Exception as e:
                    log.error(
                        "[QUAT-BATCH] addKeysWithTangents failed for %s.%s: %s",
                        joint_name,
                        attr_name,
                        e,
                    )

            # Per-key quaternion W — must be set on ALL THREE rotate curves.
            for i, w in enumerate(qw_list):
                for curve in curves:
                    try:
                        curve.setQuaternionW(i, w)
                    except Exception as e:
                        log.error(
                            "[QUAT-BATCH] setQuaternionW(%d) failed for %s: %s",
                            i,
                            joint_name,
                            e,
                        )


def _clear_anim_curve_keys(anim_curve: oma.MFnAnimCurve) -> None:
    """Remove all existing keys from an animation curve."""
    num_keys = anim_curve.numKeys
    if num_keys > 0:
        for i in range(num_keys - 1, -1, -1):
            try:
                anim_curve.remove(i)
            except Exception as exc:
                log.debug("Failed to remove key %d from anim curve: %s", i, exc)


def apply_bone_animation(
    vmd_data: VMDFile,
    bone_map: Dict[str, str],
    start_frame: int = 1,
    frame_scale: float = 1.0,
) -> None:
    """
    Apply bone animation from VMD data to Maya joints.

    All bones receive FK rotation curves. IK chain joints get the same
    treatment as regular bones — the IK solver overrides their rotation
    when IK is enabled (pmxIkToggle=1), and the FK curves take over when
    IK is disabled (pmxIkToggle=0), matching Blender mmd_tools behavior.

    Args:
        vmd_data: VMD file data containing bone keyframes
        bone_map: Dictionary mapping VMD bone names to Maya joint names
        start_frame: Starting frame in Maya timeline (default: 1)
        frame_scale: Scale factor for frame numbers (default: 1.0)
    """
    keyframe_count = 0
    unmatched_bones = set()
    refresh_suspended = False
    auto_key_state = None

    log.debug("Applying bone animation: %d keyframes", len(vmd_data.bone_keyframes))
    current_time = cmds.currentTime(query=True)

    try:
        try:
            cmds.refresh(suspend=True)
            refresh_suspended = True
            auto_key_state = cmds.autoKeyframe(query=True, state=True)
            cmds.autoKeyframe(state=False)
        except Exception:
            pass

        cmds.currentTime(0)

        joint_rest_states: Dict[str, _JointRestState] = {}
        for maya_joint_name in set(bone_map.values()):
            try:
                joint_rest_states[maya_joint_name] = _capture_joint_rest_state(
                    maya_joint_name
                )
            except Exception as exc:
                log.warning(
                    "Could not capture rest state for %s: %s", maya_joint_name, exc
                )

        log.debug("Captured rest state for %d joints", len(joint_rest_states))

        joint_keyframes = defaultdict(
            lambda: {"frames": [], "translates": [], "quaternions": []}
        )
        joints_with_translation: set = set()
        _TRANSLATION_EPSILON = 1e-5

        log.debug("Starting keyframe collection...")

        for keyframe in vmd_data.bone_keyframes:
            joint_name = bone_map.get(keyframe.bone_name)
            if joint_name is None:
                if keyframe.bone_name not in unmatched_bones:
                    log.debug("Bone not found in mapping: '%s'", keyframe.bone_name)
                    unmatched_bones.add(keyframe.bone_name)
                continue

            target_frame = int(start_frame + (keyframe.frame_number * frame_scale))
            rest_state = joint_rest_states.get(joint_name)

            trans_rest_bind = rest_state.bind_pose if rest_state else None
            trans_parent_rest = (
                rest_state.parent_world_rest if rest_state else om.MQuaternion()
            )

            final_translate = _calculate_local_translation(
                keyframe.position,
                trans_rest_bind,
                trans_parent_rest,
            )
            qx, qy, qz, qw = _quaternion_from_vmd(
                joint_name,
                keyframe.rotation,
                q_world_rest=rest_state.world_rest if rest_state else None,
                has_fixed_axis=rest_state.has_fixed_axis if rest_state else False,
            )

            if (
                abs(keyframe.position.x) > _TRANSLATION_EPSILON
                or abs(keyframe.position.y) > _TRANSLATION_EPSILON
                or abs(keyframe.position.z) > _TRANSLATION_EPSILON
            ):
                joints_with_translation.add(joint_name)

            joint_keyframes[joint_name]["frames"].append(target_frame)
            joint_keyframes[joint_name]["translates"].append(final_translate)
            joint_keyframes[joint_name]["quaternions"].append((qx, qy, qz, qw))
            keyframe_count += 1

        log.debug("Setting keyframes for %d bones...", len(joint_keyframes))
        for joint_name, data in joint_keyframes.items():
            try:
                continuous_quats = _ensure_quaternion_continuity(data["quaternions"])
                batch_set_quaternion_keyframes(
                    joint_name=joint_name,
                    frames=data["frames"],
                    translate_values=data["translates"],
                    quaternion_values=continuous_quats,
                    skip_translation=joint_name not in joints_with_translation,
                )
            except Exception as exc:
                log.error("Failed to set keyframes for %s: %s", joint_name, exc)

        if unmatched_bones:
            log.warning(
                "Could not match %d bone names from VMD to Maya joints",
                len(unmatched_bones),
            )
    finally:
        try:
            cmds.currentTime(current_time)
        except Exception:
            pass

        try:
            if refresh_suspended:
                cmds.refresh(suspend=False)
            if auto_key_state is not None:
                cmds.autoKeyframe(state=auto_key_state)
            if refresh_suspended:
                cmds.refresh()
        except Exception:
            pass


def apply_morph_animation(
    vmd_data: VMDFile,
    model: "ResolvedModelData",
    start_frame: int = 1,
    frame_scale: float = 1.0,
) -> None:
    """
    Apply morph (blend shape) animation from VMD data to Maya blend shapes.

    Handles both vertex morphs (blend shape targets) and bone morphs (boneMorphNode weights).
    VMD morph keyframes can drive either type based on the morph name mapping.
    All model data is resolved beforehand into *model*.

    Args:
        vmd_data: VMD file data containing morph keyframes
        model: Resolved model data (morph map, node names)
        start_frame: Starting frame in Maya timeline (default: 1)
        frame_scale: Scale factor for frame numbers (default: 1.0)

    Returns:
        Number of keyframes set
    """
    morph_map = model.morph_map
    blend_shape_node = model.blend_shape_node
    bone_morph_node = model.bone_morph_node

    keyframe_count = 0
    unmatched_morphs = set()
    refresh_suspended = False
    auto_key_state = None

    log.debug("Applying morph animation: %d keyframes", len(vmd_data.morph_keyframes))
    current_time = cmds.currentTime(query=True)

    try:
        try:
            cmds.refresh(suspend=True)
            refresh_suspended = True
            auto_key_state = cmds.autoKeyframe(query=True, state=True)
            cmds.autoKeyframe(state=False)
        except Exception:
            pass

        # Collect keyframes per morph target
        morph_keyframes = defaultdict(lambda: {"frames": [], "weights": []})

        log.debug("Starting morph keyframe collection...")

        for keyframe in vmd_data.morph_keyframes:
            target_name = morph_map.get(keyframe.morph_name)
            if target_name is None:
                if keyframe.morph_name not in unmatched_morphs:
                    unmatched_morphs.add(keyframe.morph_name)
                    log.debug(
                        "No mapping for VMD morph '%s', skipping", keyframe.morph_name
                    )
                continue

            target_frame = int(start_frame + (keyframe.frame_number * frame_scale))

            morph_keyframes[target_name]["frames"].append(target_frame)
            morph_keyframes[target_name]["weights"].append(keyframe.weight)
            keyframe_count += 1

        log.debug("Setting keyframes for %d morph targets...", len(morph_keyframes))

        for target_name, data in morph_keyframes.items():
            try:
                # Sort keyframes by frame number
                frame_weight_pairs = sorted(
                    zip(data["frames"], data["weights"]), key=lambda x: x[0]
                )
                sorted_frames = [f for f, _ in frame_weight_pairs]
                sorted_weights = [w for _, w in frame_weight_pairs]

                # Convert frames to MTimeArray
                time_array = om.MTimeArray()
                for frame in sorted_frames:
                    time_array.append(om.MTime(frame, om.MTime.uiUnit()))

                # Convert weights to MDoubleArray
                weight_values = om.MDoubleArray(sorted_weights)

                # Determine which node drives this morph target
                # Check if it's a blend shape target or bone morph weight
                node_name: Optional[str] = None

                if blend_shape_node and cmds.objExists(blend_shape_node):
                    # For blend shapes, check if target exists using aliasAttr
                    # Blend shape targets are stored as weight[i] with aliases
                    aliases = cmds.aliasAttr(blend_shape_node, query=True) or []
                    # aliasAttr returns pairs: [alias, real_attr, alias, real_attr, ...]
                    blend_shape_targets = [
                        aliases[i] for i in range(0, len(aliases), 2)
                    ]
                    if target_name in blend_shape_targets:
                        node_name = blend_shape_node

                if (
                    node_name is None
                    and bone_morph_node
                    and cmds.objExists(bone_morph_node)
                ):
                    # For bone morph nodes, check direct attributes
                    bone_morph_attrs = (
                        cmds.listAttr(bone_morph_node, keyable=True, scalar=True) or []
                    )
                    if target_name in bone_morph_attrs:
                        node_name = bone_morph_node

                if node_name is None:
                    log.warning(
                        "Morph target '%s' not found on blend shape or bone morph node, skipping",
                        target_name,
                    )
                    continue

                # Get or create animation curve for this morph weight
                # Morph weights are scalar attributes, so we handle them differently
                # from compound attributes (translate/rotate)
                sel = om.MSelectionList()
                sel.add(f"{node_name}.{target_name}")
                plug = sel.getPlug(0)

                # Check if plug already has an animation curve
                anim_curve = None
                if plug.isDestination:
                    src_plug = plug.source()
                    src_node = om.MFnDependencyNode(src_plug.node())
                    if src_node.hasFn(om.MFn.kAnimCurve):
                        anim_curve = oma.MFnAnimCurve(src_node.object())
                    else:
                        log.warning(
                            "Morph target '%s' has incoming connection from %s, cannot animate",
                            target_name,
                            src_node.name(),
                        )
                        continue

                # Create new animation curve if needed
                if anim_curve is None:
                    dg_mod = om.MDGModifier()
                    anim_curve = oma.MFnAnimCurve()
                    anim_curve.create(plug, oma.MFnAnimCurve.kAnimCurveTU, dg_mod)
                    dg_mod.doIt()
                else:
                    # Clear existing keyframes if reusing curve
                    num_keys = anim_curve.numKeys
                    if num_keys > 0:
                        # Remove keys in reverse order to avoid index shifting
                        for i in range(num_keys - 1, -1, -1):
                            anim_curve.remove(i)

                # Add all keyframes at once
                anim_curve.addKeys(
                    time_array,
                    weight_values,
                    oma.MFnAnimCurve.kTangentAuto,
                    oma.MFnAnimCurve.kTangentAuto,
                )

            except Exception as exc:
                log.error("Failed to set keyframes for %s: %s", target_name, exc)

        if unmatched_morphs:
            log.warning(
                "Could not match %d morph names from VMD to Maya blend shapes",
                len(unmatched_morphs),
            )
    finally:
        try:
            cmds.currentTime(current_time)
        except Exception:
            pass

        try:
            if refresh_suspended:
                cmds.refresh(suspend=False)
            if auto_key_state is not None:
                cmds.autoKeyframe(state=auto_key_state)
            if refresh_suspended:
                cmds.refresh()
        except Exception:
            pass


def apply_vmd_to_scene(
    vmd_data: VMDFile,
    model: "ResolvedModelData",
    start_frame: int = 1,
    frame_scale: float = 1.0,
    apply_bone_anim: bool = True,
    apply_morph_anim: bool = True,
) -> None:
    """
    Apply VMD animation data to a PMX model in the Maya scene.

    All model data (bone map, morph map, node names) is resolved beforehand
    into *model* — this function does NOT query the Maya scene for metadata.

    Args:
        vmd_data: VMD file data containing animation keyframes.
        model: Resolved model data (bone map, morph map, node names).
        start_frame: Starting frame in Maya timeline (default: 1)
        frame_scale: Scale factor for frame numbers (default: 1.0)
        apply_bone_anim: Whether to apply bone animation (default: True)
        apply_morph_anim: Whether to apply morph animation (default: True)
    """
    bone_map = model.bone_map

    log.debug(
        "Applying VMD animation (%d bones, %d morphs)",
        len(bone_map),
        len(model.morph_map),
    )
    log.debug("VMD model name: %s", vmd_data.model_name)
    log.debug(
        "Bone keyframes: %d, Morph keyframes: %d",
        len(vmd_data.bone_keyframes),
        len(vmd_data.morph_keyframes),
    )

    # Apply bone animation
    if apply_bone_anim and vmd_data.bone_keyframes:
        if not bone_map:
            log.warning("No bone mapping available")
        else:
            apply_bone_animation(
                vmd_data=vmd_data,
                bone_map=bone_map,
                start_frame=start_frame,
                frame_scale=frame_scale,
            )

    # Apply morph animation
    if apply_morph_anim and vmd_data.morph_keyframes:
        if not model.morph_map:
            log.warning("No morph mapping available")
        elif not model.blend_shape_node and not model.bone_morph_node:
            log.warning("No blend shape or bone morph node found in scene")
        else:
            apply_morph_animation(
                vmd_data=vmd_data,
                model=model,
                start_frame=start_frame,
                frame_scale=frame_scale,
            )

    # Set playback range to match animation
    if vmd_data.bone_keyframes or vmd_data.morph_keyframes:
        max_frame = 0
        if vmd_data.bone_keyframes:
            max_frame = max(
                max_frame, max(kf.frame_number for kf in vmd_data.bone_keyframes)
            )
        if vmd_data.morph_keyframes:
            max_frame = max(
                max_frame, max(kf.frame_number for kf in vmd_data.morph_keyframes)
            )

        end_frame = start_frame + (max_frame * frame_scale)
        try:
            cmds.playbackOptions(
                minTime=start_frame,
                maxTime=int(end_frame),
                animationStartTime=start_frame,
                animationEndTime=int(end_frame),
            )
            log.debug("Set playback range: %d - %d", start_frame, int(end_frame))
        except Exception as e:
            log.warning("Failed to set playback range: %s", e)

    log.debug("VMD animation applied successfully!")
