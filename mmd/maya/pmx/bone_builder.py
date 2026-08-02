"""
bone_builder.py

Responsible for creating Maya joint hierarchies, IK handles, pole vectors, and
rotation-inheritance constraints from PMX bone data.

This module is part of the mmd.maya.pmx package and is designed to run inside
Autodesk Maya (requires maya.api.OpenMaya, maya.api.OpenMayaAnim, maya.cmds).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

from mmd.core.data_types import (
    IKLink,
    PMXBone,
    PMXBoneFlagBits,
    PmxModel,
    Vec3,
)

# Maya modules are only available when running inside Maya.
# Importing this module outside Maya works for the pure logic functions
# (get_ik_chain_info, etc.) which are tested without Maya.
# The Maya-dependent functions raise NameError if called without Maya.
try:
    import maya.api.OpenMaya as om
    import maya.api.OpenMayaAnim as oma
    import maya.cmds as cmds
except ImportError:
    om = None  # type: ignore
    oma = None  # type: ignore
    cmds = None  # type: ignore

from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Pure helper functions and types – no Maya API dependency
# ---------------------------------------------------------------------------


@dataclass
class IKChainInfo:
    """Validated IK chain data extracted from a single PMX IK bone.

    Attributes:
        target_bone_idx: Index of the effector bone the IK tries to reach.
        start_bone_idx:  Index of the root joint of the IK chain
                         (the last link in PMX's link list).
        links:           Ordered list of IKLink objects (knee → thigh, etc.).
    """

    target_bone_idx: int
    start_bone_idx: int
    links: list[IKLink] = field(default_factory=list[IKLink])


def get_ik_chain_info(
    bone: PMXBone,
    total_bone_count: int,
) -> Optional[IKChainInfo]:
    """Validate a PMX bone's IK data and return a structured summary.

    Performs all bounds-checking so the caller does not have to.  Returns
    ``None`` for any of the following conditions:

    * The bone does not have the IK flag set.
    * ``bone.ik`` is ``None``.
    * ``targetBoneIndex`` is out of range.
    * The links list is empty (no chain to solve).
    * The start bone index (last link) is out of range.
    """
    if not (bone.flags & PMXBoneFlagBits.IK) or bone.ik is None:
        return None

    target_bone_idx = bone.ik.targetBoneIndex
    if target_bone_idx < 0 or target_bone_idx >= total_bone_count:
        return None

    if not bone.ik.links:
        return None

    start_bone_idx = bone.ik.links[-1].boneIndex
    if start_bone_idx < 0 or start_bone_idx >= total_bone_count:
        return None

    return IKChainInfo(
        target_bone_idx=target_bone_idx,
        start_bone_idx=start_bone_idx,
        links=bone.ik.links,
    )


@dataclass
class RotationInheritInfo:
    """Validated rotation-inheritance data for a PMX bone.

    Attributes:
        parent_bone_idx: Index of the bone whose rotation is copied.
        influence:       Weight in the range [0, 1].  1.0 = full copy.
    """

    parent_bone_idx: int
    influence: float


def get_rotation_inherit_info(
    bone_idx: int,
    bone: PMXBone,
    total_bone_count: int,
) -> Optional[RotationInheritInfo]:
    """Validate a PMX bone's rotation-inheritance data.

    Returns ``None`` when:

    * The INHERIT_ROTATION flag is not set.
    * ``bone.inheritBone`` is ``None``.
    * ``parentBoneIndex`` points outside the valid range.
    """

    if not (bone.flags & PMXBoneFlagBits.INHERIT_ROTATION):
        return None

    if not bone.inheritBone:
        return None

    parent_bone_idx = bone.inheritBone.parentBoneIndex
    if parent_bone_idx < 0 or parent_bone_idx >= total_bone_count:
        log.warning(
            "Bone %d ('%s') has invalid inherit parent index: %d",
            bone_idx,
            getattr(bone, "nameLocal", "?"),
            parent_bone_idx,
        )
        return None

    return RotationInheritInfo(
        parent_bone_idx=parent_bone_idx,
        influence=bone.inheritBone.influenceFactor,
    )


def build_bone_name_map(
    bones: list[PMXBone],
    bone_idx_to_maya_name: dict[int, str],
) -> dict[str, str]:
    """Build the PMX-name → Maya-joint-name lookup table.

    Maps *both* ``nameLocal`` and ``nameUniversal`` to the same Maya name so
    that VMD files can match either variant.  When the two names are
    identical only one entry is created.

    Args:
        bones: Ordered list of PMXBone objects.
        bone_idx_to_maya_name: Dict mapping each bone index to the *actual*
            Maya joint name (i.e. what Maya called it after deduplication).

    Returns:
        Dict[str, str]: ``{pmx_bone_name: maya_joint_name, ...}``
    """
    bone_name_map: dict[str, str] = {}
    for bone_idx, bone in enumerate(bones):
        maya_joint_name = bone_idx_to_maya_name[bone_idx]

        if bone.nameLocal:
            bone_name_map[bone.nameLocal] = maya_joint_name

        if bone.nameUniversal and bone.nameUniversal != bone.nameLocal:
            bone_name_map[bone.nameUniversal] = maya_joint_name

    return bone_name_map


class ConstraintType(Enum):
    """Type of Maya constraint to create for bone inheritance."""

    NONE = "none"  # No constraint needed; bone does not inherit rotation or translation
    PARENT = "parent"  # Both rotation and translation (standard Maya parentConstraint)
    ORIENT = "orient"  # Rotation only (expression node)
    POINT = "point"  # Translation only (Maya pointConstraint)


def get_inheritance_constraint_type(bone: PMXBone) -> ConstraintType:
    """Determine which type of constraint to create for an inheriting bone.

    Returns:
        InheritanceConstraintType indicating the constraint approach to use.
    """
    if not bone.inheritBone:
        return ConstraintType.NONE

    has_rot = bool(bone.flags & PMXBoneFlagBits.INHERIT_ROTATION)
    has_trans = bool(bone.flags & PMXBoneFlagBits.INHERIT_TRANSLATION)

    if has_rot and has_trans:
        return ConstraintType.PARENT
    elif has_rot:
        # Use expression for local rotation scaling
        return ConstraintType.ORIENT
    elif has_trans:
        return ConstraintType.POINT
    else:
        return ConstraintType.NONE


# ---------------------------------------------------------------------------
# Maya-dependent helpers
# ---------------------------------------------------------------------------


def _set_ik_handle_priority(ik_handle: str, bone: PMXBone, pmx_data: PmxModel) -> None:
    """Set IK handle priority for correct solve ordering.

    Lower-priority handles are solved first.  This ensures that parent IK
    chains (e.g. leg IK) are evaluated before child IK chains that share
    joints (e.g. toe IK under leg IK), preventing solver conflicts on the
    shared joint.

    The current heuristic sets:
      - Priority 1 — parent bone is NOT an IK bone (root chain).
      - Priority 2 — parent bone IS an IK bone (child chain).

    NOTE: This heuristic relies on the PMX parent-bone hierarchy, which
    matches the standard MMD leg+toe pattern but may not cover all cases:
    - IK chains that share joints without a parent-child relationship in
      the PMX hierarchy would both get priority 1 (possible conflict).
    - Three-or-more nested IK chains would collapse priorities 2+.
    A more robust approach would compute priority from actual joint overlap
    between IK chains, but that is not yet implemented.
    TODO: Implement overlap-based IK priority calculation.
    """
    parent_bone_idx = bone.parentIndex
    priority = 1  # root default
    if parent_bone_idx >= 0:
        parent_bone = pmx_data.bones[parent_bone_idx]
        if parent_bone.flags & PMXBoneFlagBits.IK:
            priority = 2  # child IK (e.g. toe IK under leg IK)
    try:
        cmds.setAttr(f"{ik_handle}.priority", priority)
    except Exception as e:
        log.warning("Could not set priority on IK handle %s: %s", ik_handle, e)


def _add_pmx_bone_attributes(
    joint_fn: om.MFnIkJoint,
    maya_joint_idx: int,
    bone_idx: int,
    bone: PMXBone,
) -> None:
    """Attach all PMX bone metadata as a single compound custom attribute on *joint_fn*.

    All children live under one compound attribute (``pmxBoneData`` /
    ``pmxBone``) so the Attribute Editor groups them cleanly.  Optional
    fields (fixedAxis, localCoordinate, inheritBone, externalParent) are
    omitted entirely when ``None`` – no sentinel values are stored.

    Attribute naming convention:

    * Long name  – ``pmx<CamelCase>``  (human-readable in Attribute Editor)
    * Short name – ``pmx<Abbrev>``     (compact, for MEL / Python scripting)

    Args:
        joint_fn:       Wraps the joint node to receive the attributes.
        maya_joint_idx: Sequential index in the ``joints`` list.
        bone_idx:       PMX bone index (0-based).
        bone:           Source :class:`PMXBone` dataclass instance.
    """

    # ── Compound container ────────────────────────────────────────────────
    compound_fn = om.MFnCompoundAttribute()
    compound_attr = compound_fn.create("pmxBoneData", "pmxBone")
    compound_fn.keyable = False
    compound_fn.storable = True

    # ── Helper closures – add children to the compound, not to the joint ──
    # String attributes need a post-add setString call; collect them here.
    _pending_strings: list[
        Tuple[str, str]
    ] = []  # List of (long_name, value) for string attributes to set after adding to joint.
    # Vector attributes also need post-add value setting (k3Float has no default in create).
    _pending_vectors: list[Tuple[str, float, float, float]] = []

    def _int(long: str, short: str, value: int) -> None:
        fn = om.MFnNumericAttribute()
        attr = fn.create(long, short, om.MFnNumericData.kInt, value)
        fn.keyable = False
        fn.storable = True
        compound_fn.addChild(attr)

    def _float(long: str, short: str, value: float) -> None:
        fn = om.MFnNumericAttribute()
        attr = fn.create(long, short, om.MFnNumericData.kFloat, value)
        fn.keyable = False
        fn.storable = True
        compound_fn.addChild(attr)

    def _bool(long: str, short: str, value: bool) -> None:
        fn = om.MFnNumericAttribute()
        attr = fn.create(long, short, om.MFnNumericData.kBoolean, int(value))
        fn.keyable = False
        fn.storable = True
        compound_fn.addChild(attr)

    def _str(long: str, short: str, value: str) -> None:
        fn = om.MFnTypedAttribute()
        default_obj = om.MFnStringData().create(value or "")
        attr = fn.create(long, short, om.MFnData.kString, default_obj)
        fn.keyable = False
        fn.storable = True
        compound_fn.addChild(attr)
        _pending_strings.append((long, value or ""))

    def _vec3(long: str, short: str, x: float, y: float, z: float) -> None:
        fn = om.MFnNumericAttribute()
        attr = fn.create(long, short, om.MFnNumericData.k3Float)
        fn.keyable = False
        fn.storable = True
        compound_fn.addChild(attr)
        _pending_vectors.append((long, x, y, z))

    # ── Identification ────────────────────────────────────────────────────
    _int("mayaJointIndex", "mayaIdx", maya_joint_idx)
    _int("pmxBoneIndex", "pmxIdx", bone_idx)
    _int("pmxParentBoneIndex", "pmxParentIdx", bone.parentIndex)
    _int("pmxLevel", "pmxLvl", bone.level)

    # ── Names ─────────────────────────────────────────────────────────────
    _str("pmxNameLocal", "pmxNameLoc", bone.nameLocal)
    _str("pmxNameUniversal", "pmxNameUni", bone.nameUniversal)

    # ── World position (MMD space – Z intentionally NOT flipped here) ─────
    _vec3(
        "pmxWorldPosition", "pmxPos", bone.position.x, bone.position.y, bone.position.z
    )

    # ── Tail info ─────────────────────────────────────────────────────────
    if isinstance(bone.tailInfo, int):
        _int("pmxTailIndex", "pmxTailIdx", bone.tailInfo)
    else:  # Vec3 offset (MMD space)
        _vec3(
            "pmxTailOffset",
            "pmxTailOfs",
            bone.tailInfo.x,
            bone.tailInfo.y,
            bone.tailInfo.z,
        )

    # ── Flags ─────────────────────────────────────────────────────────────
    f = bone.flags
    _bool("pmxRotatable", "pmxRot", bool(f & PMXBoneFlagBits.ROTATABLE))
    _bool("pmxTranslatable", "pmxTrans", bool(f & PMXBoneFlagBits.TRANSLATABLE))
    _bool("pmxVisible", "pmxVis", bool(f & PMXBoneFlagBits.VISIBLE))
    _bool("pmxEnabled", "pmxEnabled", bool(f & PMXBoneFlagBits.ENABLED))
    _bool("pmxHasIK", "pmxIK", bool(f & PMXBoneFlagBits.IK))
    _bool("pmxInheritRotation", "pmxInhRot", bool(f & PMXBoneFlagBits.INHERIT_ROTATION))
    _bool(
        "pmxInheritTranslation",
        "pmxInhTrans",
        bool(f & PMXBoneFlagBits.INHERIT_TRANSLATION),
    )
    _bool("pmxUseFixedAxis", "pmxFixAxis", bool(f & PMXBoneFlagBits.FIXED_AXIS))
    _bool(
        "pmxUseLocalCoordinate",
        "pmxLocCoord",
        bool(f & PMXBoneFlagBits.LOCAL_COORDINATE),
    )
    _bool(
        "pmxPhysicsAfterDeform",
        "pmxPhysDeform",
        bool(f & PMXBoneFlagBits.PHYSICS_AFTER_DEFORM),
    )
    _bool(
        "pmxExternalParentDeform",
        "pmxExtParDeform",
        bool(f & PMXBoneFlagBits.EXTERNAL_PARENT_DEFORM),
    )

    # ── Optional: fixed axis ──────────────────────────────────────────────
    if bone.fixedAxis is not None:
        ax = bone.fixedAxis.axis
        _vec3("pmxFixedAxis", "pmxFixAx", ax.x, ax.y, ax.z)

    # ── Optional: local coordinate axes ───────────────────────────────────
    if bone.localCoordinate is not None:
        lc = bone.localCoordinate
        _vec3("pmxLocalCoordX", "pmxLocX", lc.xAxis.x, lc.xAxis.y, lc.xAxis.z)
        _vec3("pmxLocalCoordZ", "pmxLocZ", lc.zAxis.x, lc.zAxis.y, lc.zAxis.z)

    # ── Optional: inherit bone ────────────────────────────────────────────
    if bone.inheritBone is not None:
        ib = bone.inheritBone
        _int("pmxInheritParentIndex", "pmxInhParIdx", ib.parentBoneIndex)
        _float("pmxInheritFactor", "pmxInhFac", ib.influenceFactor)

    # ── Optional: external parent ─────────────────────────────────────────
    if bone.externalParent is not None:
        _int(
            "pmxExternalParentIndex",
            "pmxExtParIdx",
            bone.externalParent.parentBoneIndex,
        )

    # ── Rest pose storage (captured after skeleton build) ─────────────────
    _float("pmxRestTranslateX", "pmxRestTx", 0.0)
    _float("pmxRestTranslateY", "pmxRestTy", 0.0)
    _float("pmxRestTranslateZ", "pmxRestTz", 0.0)
    _float("pmxRestRotateX", "pmxRestRx", 0.0)
    _float("pmxRestRotateY", "pmxRestRy", 0.0)
    _float("pmxRestRotateZ", "pmxRestRz", 0.0)

    # ── Commit compound to the joint, then finalise string values ─────────
    joint_fn.addAttribute(compound_attr)
    for long, value in _pending_strings:
        joint_fn.findPlug(long, True).setString(value)
    for long, x, y, z in _pending_vectors:
        plug = joint_fn.findPlug(long, True)
        plug.child(0).setFloat(x)
        plug.child(1).setFloat(y)
        plug.child(2).setFloat(z)


def _set_bone_radius(joint_fn: oma.MFnIkJoint, radius: float) -> None:
    """Set the radius of a joint for better visibility in the viewport.

    Args:
        joint_fn: MFnIkJoint wrapping the target joint.
        radius: Desired radius value (e.g. 0.1 for main joints, 0.08 for tails).
    """
    try:
        radius_plug = joint_fn.findPlug("radius", True)
        radius_plug.setFloat(radius)
    except Exception as e:
        log.warning("Could not set radius for joint %s: %s", joint_fn.name(), e)


def _set_joint_orient_from_axes(
    bone_name: str, x_axis: om.MVector, y_axis: om.MVector, z_axis: om.MVector
) -> None:
    """Set jointOrient on a joint from three orthonormal basis vectors (Maya space).

    Builds a rotation matrix from the basis vectors as ROWS,
    decomposes it to Euler angles, and writes the result to the joint's
    jointOrient attribute.

    Args:
        bone_name: Maya joint name (e.g. "Root").
        x_axis:    Local X basis vector (om.MVector).
        y_axis:    Local Y basis vector (om.MVector).
        z_axis:    Local Z basis vector (om.MVector).
    """
    # Build rotation matrix with basis vectors as ROWS.
    # Maya uses row-vector convention (v' = v * M), so the orthonormal
    # basis vectors go in rows 0-2:
    #     [Xx Xy Xz 0]   ← local X basis
    #     [Yx Yy Yz 0]   ← local Y basis
    #     [Zx Zy Zz 0]   ← local Z basis
    #     [0  0  0  1]
    rotation_matrix = om.MMatrix(
        [
            [x_axis.x, x_axis.y, x_axis.z, 0],
            [y_axis.x, y_axis.y, y_axis.z, 0],
            [z_axis.x, z_axis.y, z_axis.z, 0],
            [0, 0, 0, 1],
        ]
    )

    euler = om.MTransformationMatrix(rotation_matrix).rotation()
    deg_x = math.degrees(euler.x)
    deg_y = math.degrees(euler.y)
    deg_z = math.degrees(euler.z)
    cmds.setAttr(f"{bone_name}.jointOrient", deg_x, deg_y, deg_z, type="double3")
    log.debug("  -> Set jointOrient to degrees [%f, %f, %f]", deg_x, deg_y, deg_z)


# ---------------------------------------------------------------------------
# Pass functions – each encapsulates one stage of bone scene construction
# ---------------------------------------------------------------------------


def _pass1_create_joints(
    pmx_data: PmxModel,
    bone_group_obj: om.MObject,
    name_registry: PMXNamingManager,
) -> Tuple[List[om.MObject], dict[int, om.MObject]]:
    """Pass 1: Create all joints, set positions, add custom attributes, and tail joints.

    Args:
        pmx_data:        Parsed PMX model.
        bone_group_obj:  Parent MObject for the bone group transform.
        name_registry:   Provides unique Maya-safe names for all nodes.

    Returns:
        Tuple of (joints list of MObjects, pmx_bones_to_maya_joints mapping dict).
    """
    joints: list[om.MObject] = []
    pmx_bones_to_maya_joints: dict[
        int, om.MObject
    ] = {}  # bone index → joint MObject mapping for Pass 2+ access

    for bone_idx, bone in enumerate(pmx_data.bones):
        joint_fn = oma.MFnIkJoint()
        joint_obj = joint_fn.create(bone_group_obj)
        joints.append(joint_obj)

        bone_name = name_registry.get_bone_name(bone_idx)
        joint_fn.setName(bone_name)
        pmx_bones_to_maya_joints[bone_idx] = joint_obj

        joint_pos = om.MVector(bone.position.x, bone.position.y, -bone.position.z)
        joint_fn.setTranslation(joint_pos, om.MSpace.kTransform)
        _set_bone_radius(joint_fn, 0.1)

        try:
            _add_pmx_bone_attributes(joint_fn, len(joints) - 1, bone_idx, bone)
        except Exception as e:
            log.warning("Could not add custom attributes to joint %s: %s", bone_name, e)

        # FIXED AXIS: visually highlight the joint only.
        # jointOrient is intentionally NOT set — bone morph quaternions are applied
        # directly to .rotate in the joint's local frame. Setting a non-zero JO would
        # misalign that frame from world space and break morph rotations.
        bone_path = joint_fn.fullPathName()
        if bone.flags & PMXBoneFlagBits.FIXED_AXIS and bone.fixedAxis:
            cmds.setAttr(f"{bone_path}.overrideEnabled", 1)
            cmds.setAttr(f"{bone_path}.overrideColor", 24)
            log.debug(
                "  -> Highlighted FIXED_AXIS bone %s (jointOrient not set)", bone_path
            )

        # LOCAL COORDINATE: jointOrient is intentionally NOT set.
        # These bones keep world-aligned frames (jointOrient = 0,0,0) so that
        # inheritance (multiplyDivide) and morph outputs operate in the same
        # coordinate space.  The LOCAL_COORDINATE data is stored in custom
        # attributes for reference.
        if (bone.flags & PMXBoneFlagBits.LOCAL_COORDINATE) and not (
            bone.flags & PMXBoneFlagBits.FIXED_AXIS
        ):
            if bone.localCoordinate is not None:
                cmds.setAttr(f"{bone_path}.overrideEnabled", 1)
                cmds.setAttr(f"{bone_path}.overrideColor", 17)
                log.debug(
                    "  -> Skipped jointOrient (localCoordinate) for bone %s", bone_path
                )

        # Offset-mode tail joint (Vec3 offset rather than index)
        if not (bone.flags & PMXBoneFlagBits.INDEXED_TAIL_POSITION):
            if not isinstance(
                bone.tailInfo, int
            ):  # Vec3 — avoid isinstance(Vec3) which breaks after plugin reload
                try:
                    tail_joint_fn = oma.MFnIkJoint()
                    _ = tail_joint_fn.create(joint_obj)
                    tail_bone_name = name_registry.get_tail_bone_name(bone_idx)
                    tail_joint_fn.setName(tail_bone_name)
                    tail_pos = om.MVector(
                        bone.tailInfo.x, bone.tailInfo.y, -bone.tailInfo.z
                    )
                    tail_joint_fn.setTranslation(tail_pos, om.MSpace.kTransform)
                    _set_bone_radius(tail_joint_fn, 0.08)
                except Exception as e:
                    log.warning(
                        "Could not create tail joint for bone %s: %s", bone_name, e
                    )

    return joints, pmx_bones_to_maya_joints


def _pass2_build_hierarchy(
    pmx_data: PmxModel,
    joints: List[om.MObject],
    pmx_bones_to_maya_joints: dict[int, om.MObject],
) -> None:
    """Pass 2: Parent joints according to PMX parentIndex.

    Joints whose local axes are authored by PMX (FIXED_AXIS or LOCAL_COORDINATE)
    already have their jointOrient set in Pass 1 based on the PMX data. Plain joints
    (which lack explicit orientation data) intentionally remain world-aligned
    (jointOrient = 0, 0, 0) to match MMD behavior; Pass 2 does not invoke Maya's
    automatic orientJoint for any bone type and only establishes the joint hierarchy.

    This is a change from older versions where Pass 2 could apply automatic
    orientation. Now, all orientation logic lives in Pass 1, and Pass 2 is purely
    responsible for parenting.
    Args:
        pmx_data:                Parsed PMX model.
        joints:                  Ordered list of joint MObjects from Pass 1.
        pmx_bones_to_maya_joints: Bone-index → MObject mapping from Pass 1.
    """
    log.debug("Building bone hierarchy")
    for bone_idx, joint_obj in enumerate(joints):
        try:
            joint_fn = oma.MFnIkJoint(joint_obj)
            parent_idx = pmx_data.bones[bone_idx].parentIndex

            if parent_idx < 0 or parent_idx not in pmx_bones_to_maya_joints:
                continue

            parent_joint_fn = oma.MFnIkJoint(pmx_bones_to_maya_joints[parent_idx])
            joint_name = joint_fn.fullPathName()
            parent_name = parent_joint_fn.fullPathName()

            try:
                cmds.parent(joint_name, parent_name, absolute=True)
            except Exception as e:
                log.warning(
                    "Failed to parent joint %d to parent %d: %s",
                    bone_idx,
                    parent_idx,
                    e,
                )

        except Exception as e:
            log.warning("Error building hierarchy for joint %d: %s", bone_idx, e)


def _pass3_create_ik_handles(
    pmx_data: PmxModel,
    pmx_bones_to_maya_joints: dict[int, om.MObject],
    name_registry: PMXNamingManager,
) -> Tuple[List[str], dict[str, str]]:
    """Pass 3: Create ccdIKSolverNode handles and collect IK-controlled joint names.

    Each MMD IK chain gets its own ``ccdIKSolverNode`` with the chain's
    parameters (loopCount, limitRadian) and per-joint angle limits.

    Each IK handle is parented directly under its IK control bone (the joint
    with the PMX IK flag), not under a parent bone or another IK handle.
    Since bones with IK data are now treated as control bones, the handle
    stays organized with its controller without nesting inside the skeleton
    hierarchy of other joints.

    IK handles are assigned a priority to control solve order:
      - Root IK chains (parent is not an IK bone) → priority 1
      - Child IK chains (parent is also an IK bone, e.g. toe IK) → priority 2

    Args:
        pmx_data:                Parsed PMX model.
        pmx_bones_to_maya_joints: Bone-index → MObject mapping from Pass 1.
        name_registry:           Provides unique Maya-safe names for all nodes.

    Returns:
        Tuple of:
        - ik_controlled_bones (List[str]): Maya names of the IK target + all link joints.
        - ik_bone_to_handle (dict):        IK bone Maya name → IK handle name.
    """
    ik_controlled_bones: set[str] = set()
    ik_bone_to_handle: dict[str, str] = {}

    for bone_idx, bone in enumerate(pmx_data.bones):
        ik_chain = get_ik_chain_info(bone, len(pmx_data.bones))
        if ik_chain is None:
            continue

        try:
            ik_joint_long_name = oma.MFnIkJoint(
                pmx_bones_to_maya_joints[bone_idx]
            ).fullPathName()
            target_joint_name = oma.MFnIkJoint(
                pmx_bones_to_maya_joints[ik_chain.target_bone_idx]
            ).fullPathName()
            start_joint_name = oma.MFnIkJoint(
                pmx_bones_to_maya_joints[ik_chain.start_bone_idx]
            ).fullPathName()

            # Create a custom CCD IK solver node for each chain.
            # Each MMD IK chain has its own parameters (loopCount, limitRadian,
            # per-link rotation limits), so one solver per chain is required.

            solver_name = name_registry.get_ik_solver_name(bone_idx)
            log.debug("Creating CCD IK solver node: %s", solver_name)
            cmds.createNode("ccdIKSolverNode", name=solver_name)

            # Set solver params before ikHandle() — it triggers the first solve
            ik_data = bone.ik
            if ik_data:
                cmds.setAttr(f"{solver_name}.maxIterations", ik_data.loopCount)
                cmds.setAttr(f"{solver_name}.limitRadian", ik_data.limitRadian)

            ik_handle_name = name_registry.get_ik_handle_name(bone_idx)
            ik_result = cmds.ikHandle(
                startJoint=start_joint_name,
                endEffector=target_joint_name,
                solver=solver_name,
                name=ik_handle_name,
            )

            if ik_result:
                ik_handle = ik_result[0]

                # Populate per-joint link limits for doSolve()
                if ik_data:
                    try:
                        for i, link in enumerate(ik_data.links):
                            prefix = f"{solver_name}.ikLinkLimits[{i}]"
                            cmds.setAttr(f"{prefix}.ikLinkBoneIndex", link.boneIndex)
                            has_limits = (
                                link.rotationLimitMin is not None
                                and link.rotationLimitMax is not None
                            )
                            cmds.setAttr(f"{prefix}.hasIkLinkLimits", has_limits)
                            if has_limits:
                                cmds.setAttr(
                                    f"{prefix}.ikLinkLimitMin",
                                    link.rotationLimitMin.x,
                                    link.rotationLimitMin.y,
                                    link.rotationLimitMin.z,
                                )
                                cmds.setAttr(
                                    f"{prefix}.ikLinkLimitMax",
                                    link.rotationLimitMax.x,
                                    link.rotationLimitMax.y,
                                    link.rotationLimitMax.z,
                                )
                    except Exception as link_err:
                        log.warning(
                            "Could not set link limits on %s: %s",
                            solver_name,
                            link_err,
                        )

                # Parent the IK handle under the bone that owns the IK data.
                # This keeps the handle organized with its IK bone and avoids
                # nesting inside the skeleton hierarchy of other joints.
                try:
                    cmds.parent(ik_handle, ik_joint_long_name, absolute=True)
                except Exception as e:
                    log.warning(
                        "Failed to parent IK handle %s to %s: %s",
                        ik_handle,
                        ik_joint_long_name,
                        e,
                    )

                # ── IK evaluation order (priority) ──────────────────────
                # Lower priority = solved first.  Root IK handles (leg IK)
                # must solve before child IK handles (toe IK) so that the
                # leg IK positions the ankle and the toe IK fine-tunes it.
                _set_ik_handle_priority(ik_handle, bone, pmx_data)

                ik_bone_to_handle[ik_joint_long_name] = ik_handle

                ik_controlled_bones.add(target_joint_name)
                for link in ik_chain.links:
                    link_obj = pmx_bones_to_maya_joints.get(link.boneIndex)
                    if link_obj is not None:
                        link_name = oma.MFnIkJoint(link_obj).fullPathName()
                        ik_controlled_bones.add(link_name)

        except Exception as e:
            log.error("Failed to create IK handle for bone %d: %s", bone_idx, e)

    return list(ik_controlled_bones), ik_bone_to_handle


def _pass4_create_inheritance_constraints(
    pmx_data: PmxModel,
    pmx_bones_to_maya_joints: dict[int, om.MObject],
    name_registry: PMXNamingManager,
) -> None:
    """Pass 4: Create orient/point/parent constraints for bones with inherit-bone data.

    * INHERIT_ROTATION only  → rotation scaled by influence via a hidden DAG controller
    * INHERIT_TRANSLATION only → pointConstraint
    * Both flags set         → parentConstraint

    For ``INHERIT_ROTATION``, PMX behavior is local-channel inheritance:
    child rotate channels receive parent rotate channels scaled by influence.
    We intentionally create a hidden controller in the DAG hierarchy instead of
    a DG connection to child.rotate, so the child joint remains freely keyable
    and manipulable with the Rotate Tool.

    Args:
        pmx_data:                Parsed PMX model.
        pmx_bones_to_maya_joints: Bone-index → MObject mapping from Pass 1.
        name_registry:           Provides unique Maya-safe names for all nodes.
    """
    for bone_idx, bone in enumerate(pmx_data.bones):
        try:
            constraint_type = get_inheritance_constraint_type(bone)
            if constraint_type == ConstraintType.NONE:
                continue

            # Type narrowing: if we reach here, bone.inheritBone must exist
            assert bone.inheritBone is not None
            parent_bone_idx = bone.inheritBone.parentBoneIndex
            influence = bone.inheritBone.influenceFactor

            if parent_bone_idx < 0 or parent_bone_idx >= len(pmx_data.bones):
                log.warning(
                    "Bone %d ('%s') has invalid inherit parent index: %d",
                    bone_idx,
                    bone.nameLocal,
                    parent_bone_idx,
                )
                continue

            child_joint_path = oma.MFnIkJoint(
                pmx_bones_to_maya_joints[bone_idx]
            ).fullPathName()
            parent_joint_path = oma.MFnIkJoint(
                pmx_bones_to_maya_joints[parent_bone_idx]
            ).fullPathName()

            if constraint_type == ConstraintType.ORIENT:
                # In MMD, inherit rotation means child.local_rot = child.local_rot * (source.local_rot * influence).
                # It does NOT blend world space rotations like an orientConstraint.
                # Instead of connecting multiplyDivide directly to child_joint.rotate
                # (which blocks the rotate gizmo), we insert a hidden DAG controller.
                # The multiplyDivide drives the controller, and the child is parented under
                # it — inheriting rotation through the hierarchy naturally.
                # child_joint.rotate stays FREE for gizmo and keyframing.
                md_name = name_registry.get_inherit_rotation_multiplydivide_name(
                    bone_idx
                )
                md_node = cmds.createNode("multiplyDivide", name=md_name)
                cmds.setAttr(f"{md_node}.operation", 1)  # 1 = multiply
                cmds.setAttr(f"{md_node}.input2X", influence)
                cmds.setAttr(f"{md_node}.input2Y", influence)
                cmds.setAttr(f"{md_node}.input2Z", influence)
                cmds.connectAttr(f"{parent_joint_path}.rotateX", f"{md_node}.input1X")
                cmds.connectAttr(f"{parent_joint_path}.rotateY", f"{md_node}.input1Y")
                cmds.connectAttr(f"{parent_joint_path}.rotateZ", f"{md_node}.input1Z")

                # Create hidden controller in the DAG hierarchy using naming manager
                ctrl_name = name_registry.get_inherit_rotation_controller_name(bone_idx)
                ctrl = cmds.createNode("transform", name=ctrl_name)

                # Copy the child joint's local translate so the controller sits
                # at the same position relative to the shared parent.
                child_trans = cmds.getAttr(f"{child_joint_path}.translate")[0]
                cmds.setAttr(
                    f"{ctrl}.translate", child_trans[0], child_trans[1], child_trans[2]
                )

                # Parent under the child's current parent with relative=True.
                # This keeps the controller's local transform intact
                # (translate = joint offset, rotate = identity).
                child_parent = cmds.listRelatives(
                    child_joint_path, parent=True, fullPath=True
                )
                if child_parent:
                    cmds.parent(ctrl, child_parent[0], relative=True)

                # Connect multiplyDivide → controller.rotate (controller is driven, child is not)
                cmds.connectAttr(f"{md_node}.outputX", f"{ctrl}.rotateX")
                cmds.connectAttr(f"{md_node}.outputY", f"{ctrl}.rotateY")
                cmds.connectAttr(f"{md_node}.outputZ", f"{ctrl}.rotateZ")

                # Parent child under controller — child inherits rotation through hierarchy.
                # jointOrient is preserved:
                #   new_jointOrient = P^-1 × (P × JO) = JO
                cmds.parent(child_joint_path, ctrl, absolute=True)
            elif constraint_type == ConstraintType.POINT:
                cmds.pointConstraint(
                    parent_joint_path,
                    child_joint_path,
                    maintainOffset=True,
                    weight=influence,
                )
            elif constraint_type == ConstraintType.PARENT:
                cmds.parentConstraint(
                    parent_joint_path,
                    child_joint_path,
                    maintainOffset=True,
                    weight=influence,
                )

        except Exception as e:
            log.warning(
                "Failed to create inheritance constraint for bone %d: %s", bone_idx, e
            )


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


def _capture_rest_pose_on_joints(
    joints: List[om.MObject],
) -> None:
    """Capture and store rest pose values in custom attributes on all joints.

    Uses the MObject list so that the correct Maya name is always resolved,
    even when Maya auto-renames a joint (e.g. 'default' → 'default1').

    This must be called after the full scene build (including MORPH_ controller
    insertion) so the captured values reflect the final settled hierarchy.

    Args:
        joints: Ordered list of joint MObjects from bone builder pass 1.
    """
    log.debug("Capturing rest pose on %d bones", len(joints))

    for joint_obj in joints:
        joint_fn = oma.MFnIkJoint(joint_obj)
        joint_name = joint_fn.partialPathName()

        # Check if joint exists
        if not cmds.objExists(joint_name):
            continue

        try:
            # Read current values (these are the "rest" values)
            tx = cmds.getAttr(f"{joint_name}.translateX")
            ty = cmds.getAttr(f"{joint_name}.translateY")
            tz = cmds.getAttr(f"{joint_name}.translateZ")
            rx = cmds.getAttr(f"{joint_name}.rotateX")
            ry = cmds.getAttr(f"{joint_name}.rotateY")
            rz = cmds.getAttr(f"{joint_name}.rotateZ")

            # Store in custom attributes
            cmds.setAttr(f"{joint_name}.pmxRestTranslateX", tx)
            cmds.setAttr(f"{joint_name}.pmxRestTranslateY", ty)
            cmds.setAttr(f"{joint_name}.pmxRestTranslateZ", tz)
            cmds.setAttr(f"{joint_name}.pmxRestRotateX", rx)
            cmds.setAttr(f"{joint_name}.pmxRestRotateY", ry)
            cmds.setAttr(f"{joint_name}.pmxRestRotateZ", rz)

            log.debug(
                "Captured rest pose for %s: t=(%.3f, %.3f, %.3f) r=(%.3f, %.3f, %.3f)",
                joint_name,
                tx,
                ty,
                tz,
                rx,
                ry,
                rz,
            )

        except Exception as exc:
            log.warning("Failed to capture rest pose for joint %s: %s", joint_name, exc)

    log.debug("Rest pose captured for %d bones", len(joints))


def _capture_rest_pose_on_ik_handles(ik_bone_to_handle: dict[str, str]) -> None:
    """Capture and store rest pose values in custom attributes on all IK handles.

    IK handles receive transformations during VPD/VMD application and must be
    reset to rest pose between applications to prevent stacking.

    Args:
        ik_bone_to_handle: Dictionary mapping IK bone name to IK handle name
    """
    log.debug("Capturing rest pose on %d IK handles", len(ik_bone_to_handle))

    for handle_name in ik_bone_to_handle.values():
        if not cmds.objExists(handle_name):
            log.warning(
                "IK handle %s not found, skipping rest pose capture", handle_name
            )
            continue

        try:
            # Add custom attributes if they don't exist
            for attr_long, attr_short, default_val in [
                ("pmxIkRestTranslateX", "pmxIkRestTx", 0.0),
                ("pmxIkRestTranslateY", "pmxIkRestTy", 0.0),
                ("pmxIkRestTranslateZ", "pmxIkRestTz", 0.0),
                ("pmxIkRestRotateX", "pmxIkRestRx", 0.0),
                ("pmxIkRestRotateY", "pmxIkRestRy", 0.0),
                ("pmxIkRestRotateZ", "pmxIkRestRz", 0.0),
            ]:
                if not cmds.attributeQuery(attr_long, node=handle_name, exists=True):
                    cmds.addAttr(
                        handle_name,
                        longName=attr_long,
                        shortName=attr_short,
                        attributeType="float",
                        defaultValue=default_val,
                        keyable=False,
                    )

            # Read current values (these are the "rest" values)
            tx = cmds.getAttr(f"{handle_name}.translateX")
            ty = cmds.getAttr(f"{handle_name}.translateY")
            tz = cmds.getAttr(f"{handle_name}.translateZ")
            rx = cmds.getAttr(f"{handle_name}.rotateX")
            ry = cmds.getAttr(f"{handle_name}.rotateY")
            rz = cmds.getAttr(f"{handle_name}.rotateZ")

            # Store in custom attributes
            cmds.setAttr(f"{handle_name}.pmxIkRestTranslateX", tx)
            cmds.setAttr(f"{handle_name}.pmxIkRestTranslateY", ty)
            cmds.setAttr(f"{handle_name}.pmxIkRestTranslateZ", tz)
            cmds.setAttr(f"{handle_name}.pmxIkRestRotateX", rx)
            cmds.setAttr(f"{handle_name}.pmxIkRestRotateY", ry)
            cmds.setAttr(f"{handle_name}.pmxIkRestRotateZ", rz)

            log.debug(
                "Captured rest pose for IK handle %s: t=(%.3f, %.3f, %.3f) r=(%.3f, %.3f, %.3f)",
                handle_name,
                tx,
                ty,
                tz,
                rx,
                ry,
                rz,
            )

        except Exception as exc:
            log.warning(
                "Failed to capture rest pose for IK handle %s: %s", handle_name, exc
            )

    log.debug("Rest pose captured for %d IK handles", len(ik_bone_to_handle))


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def create_bones_from_pmx_bones(
    pmx_data: PmxModel,
    root_transform_obj: om.MObject,
    name_registry: PMXNamingManager,
) -> Tuple[list[om.MObject], dict[str, str], List[str], dict[str, str]]:
    """Create Maya joint hierarchies, IK handles, and constraints from PMX bone data.

    Orchestrates four passes over the bone list:

    1. ``_pass1_create_joints``                – joints, positions, custom attrs, tails.
    2. ``_pass2_build_hierarchy``              – parent–child relationships + re-orientation.
    3. ``_pass3_create_ik_handles``            – ikSCsolver handles.
    4. ``_pass4_create_inheritance_constraints`` – orient/point/parent constraints.

    Args:
        pmx_data:            Parsed PMX model.
        root_transform_obj:  MObject used as the scene root parent.
        name_registry:       Provides unique Maya-safe names for all nodes.

    Returns:
        Tuple of:
        - joints (List[MObject]):          One entry per PMX bone, in order.
        - bone_name_map (dict):            PMX bone name → Maya joint name.
        - ik_controlled_bones (List[str]): Maya names of IK target + link joints.
        - ik_bone_to_handle (dict):        IK bone Maya name → IK handle name.
    """
    bone_group_transform_fn = om.MFnTransform()
    bone_group_obj = bone_group_transform_fn.create(root_transform_obj)
    bone_group_transform_fn.setName(name_registry.get_bone_group_name())

    joints, pmx_bones_to_maya_joints = _pass1_create_joints(
        pmx_data, bone_group_obj, name_registry
    )
    _pass2_build_hierarchy(pmx_data, joints, pmx_bones_to_maya_joints)
    ik_controlled_bones, ik_bone_to_handle = _pass3_create_ik_handles(
        pmx_data, pmx_bones_to_maya_joints, name_registry
    )
    _pass4_create_inheritance_constraints(
        pmx_data, pmx_bones_to_maya_joints, name_registry
    )

    # Build name map using the ACTUAL Maya names (Maya may deduplicate with _1, _2…)
    bone_idx_to_maya_name = {
        idx: oma.MFnIkJoint(pmx_bones_to_maya_joints[idx]).name()
        for idx in range(len(pmx_data.bones))
    }
    bone_name_map = build_bone_name_map(pmx_data.bones, bone_idx_to_maya_name)
    log.debug("Created bone name mapping with %d entries", len(bone_name_map))

    # Capture rest pose after skeleton build (MORPH_ controllers no longer
    # change joint local translates, so this is safe to run here).
    _capture_rest_pose_on_joints(joints)
    _capture_rest_pose_on_ik_handles(ik_bone_to_handle)

    return joints, bone_name_map, ik_controlled_bones, ik_bone_to_handle
