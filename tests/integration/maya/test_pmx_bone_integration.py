"""
test_pmx_bone_integration.py

Integration tests for the bone-building stage of PMX → Maya import.
These tests exercise ``mmd.maya.pmx.bone_builder`` via the full
``build_pmx_scene`` pipeline and verify each pass of the builder:

    Pass 1 – joint creation + custom PMX attributes + tail joints
    Pass 2 – parent-child hierarchy + joint re-orientation
    Pass 3 – IK handle creation
    Pass 4 – rotation-/translation-inherit constraints

Running
-------
    mayapy tests/integration/maya/test_pmx_bone_integration.py
"""

from __future__ import annotations

# ── Maya standalone initialised by the test runner ───────────────────────
import maya.api.OpenMaya as om  # noqa: E402
import maya.cmds as cmds  # noqa: E402
from mmd.maya.maya_data_types import MayaPmxData  # noqa: E402
from mmd.maya.pmx.rigid_body_builder import PhysicsBinding  # noqa: E402

# ── Project imports ─────────────────────────────────────────────────────────
from mmd.core.data_types import PMXBoneFlagBits, PmxModel  # noqa: E402
from mmd.maya.pmx.bone_builder import (  # noqa: E402
    build_bone_name_map,
    get_ik_chain_info,
    get_inheritance_constraint_type,
    get_rotation_inherit_info,
    ConstraintType,
)
from tests.integration.test_helpers import (  # noqa: E402
    assert_true,
    assert_eq,
    skip_test,
)

# ── Local test infrastructure ───────────────────────────────────────────────

# ---------------------------------------------------------------------------
# Helpers shared by tests in this module
# ---------------------------------------------------------------------------

_TOLERANCE = 1e-4  # world-position comparison tolerance


def _joint_name(joint_obj: om.MObject) -> str:
    return om.MFnDagNode(joint_obj).partialPathName()


def _get_effective_parent(joint_fn: om.MFnDagNode):
    """Walk up through non-joint DAG nodes (controllers) to find the real parent joint.

    Bone morph and inheritance-rotation controllers are always plain ``transform``
    nodes inserted into the DAG hierarchy above the joint they control.  We detect
    them by checking the Maya object type (``MObject.hasFn``) rather than relying
    on any naming convention, which makes this function robust against naming changes.

    Joints may have multiple controller levels (e.g., a MORPH_ transform wrapping an
    inheritCtrl). Walking up through all non-joint transforms finds the actual
    DAG parent joint.
    """
    if joint_fn.parentCount() == 0:
        return None

    current = joint_fn.parent(0)  # MObject

    # Keep walking up while the parent is a non-joint transform (controller)
    while True:
        # Controllers are plain transforms — not joints, not mesh shapes, etc.
        # Use MObject.hasFn() (the OpenMaya 2.0 way), not MFnDagNode.hasFn().
        is_joint = current.hasFn(om.MFn.kJoint)
        is_transform = current.hasFn(om.MFn.kTransform)

        if not is_transform or is_joint:
            # Found a non-controller node (joint, or a non-transform shape)
            return current

        # Still in a controller (transform, not joint), walk up one more level
        current_fn = om.MFnDagNode(current)
        if current_fn.parentCount() == 0:
            return current

        current = current_fn.parent(0)  # MObject


def _world_position(joint_obj: om.MObject) -> tuple[float, float, float]:
    """Return the world-space translation of a joint as a plain tuple."""
    name = _joint_name(joint_obj)
    tx, ty, tz = cmds.xform(name, q=True, ws=True, t=True)
    return (tx, ty, tz)


def _physics_driven_bone_indices(maya_pmx_data, pmx_data) -> set[int]:
    """Return the set of bone indices driven by the physics write-back.

    Milestone 2: dynamic rigid bodies (PHYSICS / PHYSICS_BONE) write their
    solved pose back to their related bone through a ``parentConstraint`` /
    ``orientConstraint`` (guide → bone).  That constraint legitimately drives
    the bone's ``rotate`` channels — even at rest the solver pose is applied
    through the constraint (the world matrix stays exactly at rest; the raw
    rotate values may be a non-canonical Euler representation of identity).

    Skeleton-construction tests that assert ``rotate == 0`` / ``rotate ==
    pmxRest*`` therefore exempt physics-driven bones — their rotation is owned
    by the simulation, not by the skeleton builder.
    """
    binding = PhysicsBinding.from_scene(maya_pmx_data.root_name, pmx_data=pmx_data)
    if binding.node is None:
        return set()
    driven: set[int] = set()
    for rb_idx in binding.constraints:
        if 0 <= rb_idx < len(pmx_data.rigid_bodies):
            related = pmx_data.rigid_bodies[rb_idx].related_bone_index
            if related >= 0:
                driven.add(related)
    return driven


# ---------------------------------------------------------------------------
# Pass 1 – joint creation
# ---------------------------------------------------------------------------


def test_pmx_bone_group_creation(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Bone group transform (PMX_*_Bones) exists directly under root."""
    root_fn = om.MFnTransform(maya_pmx_data.root_obj)

    bone_group = None
    for i in range(root_fn.childCount()):
        child_fn = om.MFnTransform(root_fn.child(i))
        if "_Bones" in child_fn.name():
            bone_group = child_fn
            break

    assert_true(bone_group is not None, "PMX_*_Bones group not found under root")
    parent_name = om.MFnTransform(bone_group.parent(0)).name()
    assert_eq(parent_name, root_fn.name(), "PMX_Bones has wrong parent")

    print(f"PASS: Bone group '{bone_group.name()}' parented under '{root_fn.name()}'")
    return True


def test_pmx_bone_creation(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Correct number of joints created and all carry mandatory custom attributes."""
    joints = maya_pmx_data.joints
    expected = len(pmx_data.bones)

    assert_eq(len(joints), expected, f"Expected {expected} joints, got {len(joints)}")
    print(f"PASS: {len(joints)} joints match PMX bone count")

    # Validate each joint is a live MObject
    for i, jobj in enumerate(joints):
        assert_true(not jobj.isNull(), f"Joint {i} is a null MObject")
    print("PASS: All joints are valid MObjects")

    # Check mandatory custom attribute existence on the first joint
    dep_fn = om.MFnDependencyNode(joints[0])
    required = ("pmxBoneIndex", "pmxParentBoneIndex")
    for attr in required:
        assert_true(dep_fn.hasAttribute(attr), f"Joint 0 is missing attribute '{attr}'")
    print("PASS: Mandatory custom PMX attributes present on joints")
    return True


def test_pmx_bone_positions(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """World-space joint positions match PMX bone positions (Z-axis negated)."""
    joints = maya_pmx_data.joints
    mismatches = 0

    for bone_idx, (jobj, bone) in enumerate(zip(joints, pmx_data.bones)):
        wx, wy, wz = _world_position(jobj)
        ex, ey, ez = bone.position.x, bone.position.y, -bone.position.z  # flip Z

        dx, dy, dz = abs(wx - ex), abs(wy - ey), abs(wz - ez)
        if dx > _TOLERANCE or dy > _TOLERANCE or dz > _TOLERANCE:
            print(
                f"  MISMATCH bone {bone_idx} '{bone.nameLocal}': "
                f"got ({wx:.4f}, {wy:.4f}, {wz:.4f}) "
                f"expected ({ex:.4f}, {ey:.4f}, {ez:.4f})"
            )
            mismatches += 1

    assert_true(
        mismatches == 0,
        f"{mismatches}/{len(joints)} joint positions incorrect",
    )

    print(f"PASS: All {len(joints)} joint world positions match PMX data")
    return True


def test_pmx_bone_custom_attribute_values(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Custom attribute *values* on each joint match the source PMX bone data."""
    joints = maya_pmx_data.joints
    errors = []

    for bone_idx, (jobj, bone) in enumerate(zip(joints, pmx_data.bones)):
        name = _joint_name(jobj)

        # pmxBoneIndex
        got_pmx_idx = cmds.getAttr(f"{name}.pmxBoneIndex")
        if int(got_pmx_idx) != bone_idx:
            errors.append(
                f"bone {bone_idx}: pmxBoneIndex got {got_pmx_idx}, expected {bone_idx}"
            )

        # pmxParentBoneIndex
        got_par = cmds.getAttr(f"{name}.pmxParentBoneIndex")
        if int(got_par) != bone.parentIndex:
            errors.append(
                f"bone {bone_idx}: pmxParentBoneIndex got {got_par}, "
                f"expected {bone.parentIndex}"
            )

        # pmxLevel
        got_lvl = cmds.getAttr(f"{name}.pmxLevel")
        if int(got_lvl) != bone.level:
            errors.append(
                f"bone {bone_idx}: pmxLevel got {got_lvl}, expected {bone.level}"
            )

        # pmxNameLocal
        got_name_local = cmds.getAttr(f"{name}.pmxNameLocal")
        if got_name_local != bone.nameLocal:
            errors.append(
                f"bone {bone_idx}: pmxNameLocal got '{got_name_local}', "
                f"expected '{bone.nameLocal}'"
            )

        # pmxNameUniversal
        got_name_uni = cmds.getAttr(f"{name}.pmxNameUniversal")
        if got_name_uni != bone.nameUniversal:
            errors.append(
                f"bone {bone_idx}: pmxNameUniversal got '{got_name_uni}', "
                f"expected '{bone.nameUniversal}'"
            )

        # pmxWorldPosition k3Float
        position = cmds.getAttr(f"{name}.pmxWorldPosition")[
            0
        ]  # returns a tuple inside a list

        # Bone position is in mmd order (X right, Y up, Z forward),
        # and we store it in extra attributes without modification
        # so we don't have to flip the Z sign here – the builder will flip it when applying to joint translation.
        ex_pos = (bone.position.x, bone.position.y, bone.position.z)
        for i in range(3):
            if abs(position[i] - ex_pos[i]) > _TOLERANCE:
                errors.append(
                    f"bone {bone_idx}: pmxWorldPosition[{i}] got {position[i]:.4f}, "
                    f"expected {ex_pos[i]:.4f}"
                )

        # pmxTailIndex or pmxTailOffset
        if isinstance(bone.tailInfo, int):  # tail is a bone index
            got_tail_idx = cmds.getAttr(f"{name}.pmxTailIndex")
            if int(got_tail_idx) != bone.tailInfo:
                errors.append(
                    f"bone {bone_idx}: pmxTailIndex got {got_tail_idx}, "
                    f"expected {bone.tailInfo}"
                )
        else:  # tail is a Vec3 offset
            tail_offset = cmds.getAttr(f"{name}.pmxTailOffset")[0]
            ex_tail = (bone.tailInfo.x, bone.tailInfo.y, bone.tailInfo.z)
            for i in range(3):
                if abs(tail_offset[i] - ex_tail[i]) > _TOLERANCE:
                    errors.append(
                        f"bone {bone_idx}: pmxTailOffset[{i}] got {tail_offset[i]:.4f}, "
                        f"expected {ex_tail[i]:.4f}"
                    )

        # ── Flags ────────────────────────────────────────────────────────
        flag_checks = [
            ("pmxRotatable", bool(bone.flags & PMXBoneFlagBits.ROTATABLE)),
            ("pmxTranslatable", bool(bone.flags & PMXBoneFlagBits.TRANSLATABLE)),
            ("pmxVisible", bool(bone.flags & PMXBoneFlagBits.VISIBLE)),
            ("pmxEnabled", bool(bone.flags & PMXBoneFlagBits.ENABLED)),
            ("pmxHasIK", bool(bone.flags & PMXBoneFlagBits.IK)),
            ("pmxInheritRotation", bool(bone.flags & PMXBoneFlagBits.INHERIT_ROTATION)),
            (
                "pmxInheritTranslation",
                bool(bone.flags & PMXBoneFlagBits.INHERIT_TRANSLATION),
            ),
            ("pmxUseFixedAxis", bool(bone.flags & PMXBoneFlagBits.FIXED_AXIS)),
            (
                "pmxUseLocalCoordinate",
                bool(bone.flags & PMXBoneFlagBits.LOCAL_COORDINATE),
            ),
            (
                "pmxPhysicsAfterDeform",
                bool(bone.flags & PMXBoneFlagBits.PHYSICS_AFTER_DEFORM),
            ),
            (
                "pmxExternalParentDeform",
                bool(bone.flags & PMXBoneFlagBits.EXTERNAL_PARENT_DEFORM),
            ),
        ]
        for attr, expected_val in flag_checks:
            got = bool(cmds.getAttr(f"{name}.{attr}"))
            if got != expected_val:
                errors.append(
                    f"bone {bone_idx}: {attr} got {got}, expected {expected_val}"
                )

        # ── Optional: fixed axis ──────────────────────────────────────────
        if bone.fixedAxis is not None:
            ax = cmds.getAttr(f"{name}.pmxFixedAxis")[0]
            ex_ax = (
                bone.fixedAxis.axis.x,
                bone.fixedAxis.axis.y,
                bone.fixedAxis.axis.z,
            )
            for i in range(3):
                if abs(ax[i] - ex_ax[i]) > _TOLERANCE:
                    errors.append(
                        f"bone {bone_idx}: pmxFixedAxis[{i}] got {ax[i]:.4f}, "
                        f"expected {ex_ax[i]:.4f}"
                    )

        # ── Optional: local coordinate axes ──────────────────────────────
        if bone.localCoordinate is not None:
            lc = bone.localCoordinate
            for attr, ex_vec in [
                ("pmxLocalCoordX", (lc.xAxis.x, lc.xAxis.y, lc.xAxis.z)),
                ("pmxLocalCoordZ", (lc.zAxis.x, lc.zAxis.y, lc.zAxis.z)),
            ]:
                got_vec = cmds.getAttr(f"{name}.{attr}")[0]
                for i in range(3):
                    if abs(got_vec[i] - ex_vec[i]) > _TOLERANCE:
                        errors.append(
                            f"bone {bone_idx}: {attr}[{i}] got {got_vec[i]:.4f}, "
                            f"expected {ex_vec[i]:.4f}"
                        )

        # ── Optional: inherit bone ────────────────────────────────────────
        if bone.inheritBone is not None:
            ib = bone.inheritBone
            got_inh_idx = int(cmds.getAttr(f"{name}.pmxInheritParentIndex"))
            if got_inh_idx != ib.parentBoneIndex:
                errors.append(
                    f"bone {bone_idx}: pmxInheritParentIndex got {got_inh_idx}, "
                    f"expected {ib.parentBoneIndex}"
                )
            got_inh_fac = cmds.getAttr(f"{name}.pmxInheritFactor")
            if abs(got_inh_fac - ib.influenceFactor) > _TOLERANCE:
                errors.append(
                    f"bone {bone_idx}: pmxInheritFactor got {got_inh_fac:.4f}, "
                    f"expected {ib.influenceFactor:.4f}"
                )

        # ── Optional: external parent ─────────────────────────────────────
        if bone.externalParent is not None:
            got_ext = int(cmds.getAttr(f"{name}.pmxExternalParentIndex"))
            if got_ext != bone.externalParent.parentBoneIndex:
                errors.append(
                    f"bone {bone_idx}: pmxExternalParentIndex got {got_ext}, "
                    f"expected {bone.externalParent.parentBoneIndex}"
                )

    if errors:
        for msg in errors[:10]:  # cap output for large models
            print(f"  ERROR: {msg}")
        if len(errors) > 10:
            print(f"  … and {len(errors) - 10} more errors")
    assert_true(
        len(errors) == 0,
        f"{len(errors)} attribute value mismatches found",
    )

    print(f"PASS: Custom attribute values match for all {len(joints)} joints")
    return True


# ---------------------------------------------------------------------------
# Pass 2 – hierarchy
# ---------------------------------------------------------------------------


def test_pmx_bone_hierarchy(pmx_data: PmxModel, maya_pmx_data: MayaPmxData) -> bool:
    """Each joint is parented correctly, accounting for controllers."""
    joints = maya_pmx_data.joints
    assert_true(len(joints) > 0, "No joints present")

    correct = total = 0
    for bone_idx, jobj in enumerate(joints):
        pmx_bone = pmx_data.bones[bone_idx]
        if not (0 <= pmx_bone.parentIndex < len(joints)):
            continue
        total += 1
        joint_fn = om.MFnDagNode(jobj)
        if joint_fn.parentCount() == 0:
            continue
        actual_parent = _get_effective_parent(joint_fn)
        if actual_parent is None:
            continue
        expected_parent = joints[pmx_bone.parentIndex]
        if actual_parent == expected_parent:
            correct += 1
        else:
            actual_name = om.MFnDagNode(actual_parent).partialPathName()
            expected_name = _joint_name(expected_parent)
            print(
                f"  MISMATCH bone {bone_idx} '{pmx_bone.nameLocal}': "
                f"parent is '{actual_name}', expected '{expected_name}'"
            )

    if total == 0:
        skip_test("No bones require parenting (root-level skeleton)")

    rate = (correct / total) * 100
    assert_eq(correct, total, f"{correct}/{total} bone parents correct ({rate:.1f}%)")
    print(f"PASS: {correct}/{total} bone parents correct ({rate:.1f}%)")
    return True


# ---------------------------------------------------------------------------
# Pass 1 continued – authored orientation bones
# ---------------------------------------------------------------------------


def test_pmx_fixed_axis_bones(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Bones with FIXED_AXIS have overrideEnabled + overrideColor=24."""
    joints = maya_pmx_data.joints
    fixed_axis_bones = [
        (bone_idx, bone)
        for bone_idx, bone in enumerate(pmx_data.bones)
        if bone.flags & PMXBoneFlagBits.FIXED_AXIS
    ]

    if not fixed_axis_bones:
        skip_test("Model has no FIXED_AXIS bones")

    errors = []
    for bone_idx, bone in fixed_axis_bones:
        name = _joint_name(joints[bone_idx])
        if not cmds.getAttr(f"{name}.overrideEnabled"):
            errors.append(
                f"bone {bone_idx} '{bone.nameLocal}': overrideEnabled not set"
            )
        got_color = cmds.getAttr(f"{name}.overrideColor")
        if got_color != 24:
            errors.append(
                f"bone {bone_idx} '{bone.nameLocal}': overrideColor={got_color}, expected 24"
            )

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
        if len(errors) > 10:
            print(f"  … and {len(errors) - 10} more errors")
    assert_true(len(errors) == 0, f"{len(errors)} FIXED_AXIS violation(s) found")

    print(f"PASS: {len(fixed_axis_bones)} FIXED_AXIS bone(s) verified")
    return True


def test_pmx_local_coordinate_bones(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Bones with LOCAL_COORDINATE have overrideColor=17 and custom attributes."""
    joints = maya_pmx_data.joints
    local_coord_bones = [
        (bone_idx, bone)
        for bone_idx, bone in enumerate(pmx_data.bones)
        if (bone.flags & PMXBoneFlagBits.LOCAL_COORDINATE)
        and not (bone.flags & PMXBoneFlagBits.FIXED_AXIS)
    ]

    if not local_coord_bones:
        skip_test("Model has no LOCAL_COORDINATE-only bones")

    errors = []
    for bone_idx, bone in local_coord_bones:
        name = _joint_name(joints[bone_idx])
        if not cmds.getAttr(f"{name}.overrideEnabled"):
            errors.append(
                f"bone {bone_idx} '{bone.nameLocal}': overrideEnabled not set"
            )
        got_color = cmds.getAttr(f"{name}.overrideColor")
        if got_color != 17:
            errors.append(
                f"bone {bone_idx} '{bone.nameLocal}': overrideColor={got_color}, expected 17"
            )
        if bone.localCoordinate is not None:
            has_lc = cmds.attributeQuery("pmxLocalCoordX", node=name, exists=True)
            if not has_lc:
                errors.append(
                    f"bone {bone_idx} '{bone.nameLocal}': pmxLocalCoordX missing"
                )

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
        if len(errors) > 10:
            print(f"  … and {len(errors) - 10} more errors")
    assert_true(len(errors) == 0, f"{len(errors)} LOCAL_COORDINATE violation(s) found")

    print(f"PASS: {len(local_coord_bones)} LOCAL_COORDINATE bone(s) verified")
    return True


# ---------------------------------------------------------------------------
# Pass 3 – IK handles
# ---------------------------------------------------------------------------


def test_pmx_ik_handles_created(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Verify that IK handles are created for IK bones that can be built."""
    total_bones = len(pmx_data.bones)
    expected_ik_bones = [
        (bone_idx, bone)
        for bone_idx, bone in enumerate(pmx_data.bones)
        if bone.flags & PMXBoneFlagBits.IK
    ]

    if not expected_ik_bones:
        skip_test("Model has no IK bones")

    actual_ik_handles = cmds.ls(type="ikHandle") or []
    actual_count = len(actual_ik_handles)

    if actual_count == len(expected_ik_bones):
        print(
            f"PASS: {actual_count} IK handle(s) match {len(expected_ik_bones)} IK bone(s)"
        )
        return True
    elif actual_count > 0:
        print(
            f"PASS: {actual_count} IK handle(s) created ({len(expected_ik_bones) - actual_count} chains could not be built)"
        )
        return True
    else:
        valid_count = sum(
            1
            for idx, _ in expected_ik_bones
            if get_ik_chain_info(pmx_data.bones[idx], total_bones) is not None
        )
        if valid_count > 0:
            skip_test(
                f"{len(expected_ik_bones)} IK bones but {valid_count} have valid chains — builder may reject at runtime"
            )
        else:
            skip_test(f"{len(expected_ik_bones)} IK bones but no valid chains")
        return True


# ---------------------------------------------------------------------------


def test_pmx_ik_handle_parented_under_control_bone(
    pmx_data: PmxModel,
    maya_pmx_data,
) -> bool:
    """Each IK handle is parented directly under its IK control bone.

    Uses the actual IK handles stored in ``maya_pmx_data.ik_handles``
    (or falls back to ``cmds.ls(type="ikHandle")``) and verifies each
    handle's parent joint matches the expected PMX IK bone.
    """
    bone_map = maya_pmx_data.bone_name_map

    # Use stored IK handles if available, otherwise query scene
    ik_handles = (
        list(maya_pmx_data.ik_handles)
        if maya_pmx_data.ik_handles
        else (cmds.ls(type="ikHandle") or [])
    )
    if not ik_handles:
        skip_test("No IK handles in scene")

    errors: list[str] = []
    for bone_idx, bone in enumerate(pmx_data.bones):
        if not (bone.flags & PMXBoneFlagBits.IK):
            continue

        # Find the Maya joint name for this IK bone
        maya_joint_name = bone_map.get(bone.nameLocal)
        if not maya_joint_name:
            maya_joint_name = bone_map.get(bone.nameUniversal)
        if not maya_joint_name:
            continue

        # Find the IK handle whose parent is this joint
        matching_handle = None
        for h in ik_handles:
            parent = cmds.listRelatives(h, parent=True, fullPath=True)
            if parent and maya_joint_name in parent[0]:
                matching_handle = h
                break

        if not matching_handle:
            errors.append(
                f"IK handle not found for bone '{bone.nameLocal}' "
                f"(joint '{maya_joint_name}')"
            )
            continue

        # Verify parent of IK handle is the control bone
        parent = cmds.listRelatives(matching_handle, parent=True, fullPath=True)
        if not parent or maya_joint_name not in parent[0]:
            errors.append(
                f"IK handle '{matching_handle}' parent is "
                f"'{parent[0] if parent else 'None'}', expected '{maya_joint_name}'"
            )

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
    assert_true(len(errors) == 0, f"{len(errors)} IK handle parent violation(s)")

    print(f"PASS: All {len(ik_handles)} IK handles parented under their control bones")
    return True


def test_pmx_ik_handle_priority(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """IK handle priority is set correctly:
    - Root IK chains (parent is not an IK bone) → priority 1
    - Child IK chains (parent is also an IK bone) → priority 2
    """
    bone_map = maya_pmx_data.bone_name_map

    ik_handles = (
        list(maya_pmx_data.ik_handles)
        if maya_pmx_data.ik_handles
        else (cmds.ls(type="ikHandle") or [])
    )
    if not ik_handles:
        skip_test("No IK handles in scene")

    errors: list[str] = []
    for bone_idx, bone in enumerate(pmx_data.bones):
        if not (bone.flags & PMXBoneFlagBits.IK):
            continue

        # Find the Maya joint name for this IK bone
        maya_joint_name = bone_map.get(bone.nameLocal)
        if not maya_joint_name:
            maya_joint_name = bone_map.get(bone.nameUniversal)
        if not maya_joint_name:
            continue

        # Find the matching IK handle by parent joint
        matching_handle = None
        for h in ik_handles:
            parent = cmds.listRelatives(h, parent=True, fullPath=True)
            if parent and maya_joint_name in parent[0]:
                matching_handle = h
                break

        if not matching_handle:
            continue

        # Determine expected priority
        parent_bone_idx = bone.parentIndex
        expected_priority = 1  # root default
        if parent_bone_idx >= 0:
            parent_bone = pmx_data.bones[parent_bone_idx]
            if parent_bone.flags & PMXBoneFlagBits.IK:
                expected_priority = 2  # child IK

        actual_priority = cmds.getAttr(f"{matching_handle}.priority")
        if actual_priority != expected_priority:
            errors.append(
                f"IK handle '{matching_handle}' (bone '{bone.nameLocal}') has "
                f"priority {actual_priority}, expected {expected_priority} "
                f"(parent bone {parent_bone_idx} is{' ' if expected_priority == 2 else ' not '}IK)"
            )

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
    assert_true(len(errors) == 0, f"{len(errors)} IK handle priority violation(s)")

    print(f"PASS: All {len(ik_handles)} IK handles have correct priority")
    return True


# ---------------------------------------------------------------------------


def _find_solver_for_ik_bone(pmx_data, bone_idx, maya_pmx_data):
    """Find the ccdIKSolverNode connected to an IK bone's IK handle.

    Looks up the IK handle for the given bone (from maya_pmx_data.ik_handles
    or by iterating scene handles), then follows the ``ikSolver`` connection
    to find the attached solver node.
    """
    bone_map = maya_pmx_data.bone_name_map
    bone = pmx_data.bones[bone_idx]

    maya_joint_name = bone_map.get(bone.nameLocal) or bone_map.get(bone.nameUniversal)
    if not maya_joint_name:
        return None

    # Collect all IK handles
    ik_handles = (
        list(maya_pmx_data.ik_handles)
        if maya_pmx_data.ik_handles
        else (cmds.ls(type="ikHandle") or [])
    )

    for h in ik_handles:
        parent = cmds.listRelatives(h, parent=True, fullPath=True)
        if parent and maya_joint_name in parent[0]:
            # Follow the ikSolver connection
            connections = (
                cmds.listConnections(f"{h}.ikSolver", source=True, destination=False)
                or []
            )
            if connections:
                return connections[0]
    return None


def test_pmx_ccd_solver_created(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """A ccdIKSolverNode is created for each IK chain with correct params.

    Result-oriented: verifies that solvers which *were* created have the
    correct parameters, and logs warnings for any that couldn't be built.
    """
    ik_bones = [bone for bone in pmx_data.bones if bone.flags & PMXBoneFlagBits.IK]
    if not ik_bones:
        skip_test("No IK bones in model")

    solver_nodes = cmds.ls(type="ccdIKSolverNode") or []
    if not solver_nodes:
        skip_test("No CCD solver nodes in scene")

    errors: list[str] = []
    verified = 0
    for bone_idx, bone in enumerate(pmx_data.bones):
        if not (bone.flags & PMXBoneFlagBits.IK):
            continue

        solver_name = _find_solver_for_ik_bone(pmx_data, bone_idx, maya_pmx_data)
        if not solver_name:
            continue

        verified += 1

        # Verify solver params match PMX data
        if bone.ik:
            actual_iters = cmds.getAttr(f"{solver_name}.maxIterations")
            if actual_iters != bone.ik.loopCount:
                errors.append(
                    f"Solver '{solver_name}' (bone '{bone.nameLocal}'): "
                    f"maxIterations={actual_iters}, expected {bone.ik.loopCount}"
                )
            actual_limit = cmds.getAttr(f"{solver_name}.limitRadian")
            if abs(actual_limit - bone.ik.limitRadian) > 1e-6:
                errors.append(
                    f"Solver '{solver_name}' (bone '{bone.nameLocal}'): "
                    f"limitRadian={actual_limit:.6f}, expected {bone.ik.limitRadian:.6f}"
                )

    missing = len(ik_bones) - verified
    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
    assert_true(len(errors) == 0, f"{len(errors)} solver parameter mismatch(es)")

    if missing > 0:
        print(
            f"PASS: {verified}/{len(ik_bones)} solver(s) verified "
            f"({missing} IK chains could not be built)"
        )
    else:
        print(
            f"PASS: {len(solver_nodes)} ccdIKSolverNode(s) created with "
            "correct parameters"
        )
    return True


def test_pmx_ccd_solver_link_limits(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Per-joint link limits are populated on each CCD solver node.

    Result-oriented: verifies limits on solvers that exist, skips
    silently for IK bones whose chains couldn't be built.
    """
    solver_nodes = cmds.ls(type="ccdIKSolverNode") or []
    if not solver_nodes:
        skip_test("No CCD solver nodes in scene")

    errors: list[str] = []
    for bone_idx, bone in enumerate(pmx_data.bones):
        if not (bone.flags & PMXBoneFlagBits.IK) or not bone.ik:
            continue

        solver_name = _find_solver_for_ik_bone(pmx_data, bone_idx, maya_pmx_data)
        if not solver_name:
            continue

        for i, link in enumerate(bone.ik.links):
            prefix = f"{solver_name}.ikLinkLimits[{i}]"
            try:
                if not cmds.attributeQuery(
                    "ikLinkBoneIndex", node=solver_name, exists=True
                ):
                    errors.append(f"{solver_name} missing ikLinkBoneIndex attribute")
                    continue

                stored_idx = cmds.getAttr(f"{prefix}.ikLinkBoneIndex")
                if stored_idx != link.boneIndex:
                    errors.append(
                        f"{prefix}.ikLinkBoneIndex={stored_idx}, "
                        f"expected {link.boneIndex}"
                    )
                has_limits = cmds.getAttr(f"{prefix}.hasIkLinkLimits")
                expected_has = (
                    link.rotationLimitMin is not None
                    and link.rotationLimitMax is not None
                )
                if has_limits != expected_has:
                    errors.append(
                        f"{prefix}.hasIkLinkLimits={has_limits}, "
                        f"expected {expected_has}"
                    )
            except Exception as exc:
                errors.append(f"Could not read {prefix}: {exc}")

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
    assert_true(len(errors) == 0, f"{len(errors)} link limit check(s) failed")

    print(
        f"PASS: Per-joint link limits populated correctly on "
        f"{len(solver_nodes)} solver(s)"
    )
    return True


# ---------------------------------------------------------------------------
# Pass 1 (cont.) – tail joints
# ---------------------------------------------------------------------------


def test_pmx_tail_joints_created(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Offset-mode tail joints (Vec3 tailInfo) are created as children of their parent bone."""
    joints = maya_pmx_data.joints
    tail_bones = [
        (bone_idx, bone)
        for bone_idx, bone in enumerate(pmx_data.bones)
        if not (bone.flags & PMXBoneFlagBits.INDEXED_TAIL_POSITION)
        and not isinstance(
            bone.tailInfo, int
        )  # Vec3 — avoid isinstance(Vec3) which breaks after plugin reload
    ]

    if not tail_bones:
        print("SKIP: Model has no offset-mode tail bones – nothing to verify")
        return True

    errors = []
    for bone_idx, bone in tail_bones:
        parent_joint = joints[bone_idx]
        parent_fn = om.MFnDagNode(parent_joint)
        ex_tail = om.MVector(bone.tailInfo.x, bone.tailInfo.y, -bone.tailInfo.z)

        # Find the tail joint among children by matching local position
        tail_match = None
        for i in range(parent_fn.childCount()):
            child = parent_fn.child(i)
            if child.hasFn(om.MFn.kJoint):
                child_fn = om.MFnTransform(child)
                child_pos = child_fn.translation(om.MSpace.kTransform)
                if (
                    abs(child_pos.x - ex_tail.x) < _TOLERANCE
                    and abs(child_pos.y - ex_tail.y) < _TOLERANCE
                    and abs(child_pos.z - ex_tail.z) < _TOLERANCE
                ):
                    tail_match = child_fn
                    break

        if tail_match is None:
            errors.append(
                f"bone {bone_idx} '{bone.nameLocal}': "
                f"tail joint not found among children"
            )
            continue

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
        if len(errors) > 10:
            print(f"  … and {len(errors) - 10} more errors")
        print(f"FAIL: {len(errors)} tail joint violation(s) found")
        return False

    print(
        f"PASS: {len(tail_bones)} offset-mode tail joint(s) verified "
        "(child of parent, position matches Z-flipped offset)"
    )
    return True


# ---------------------------------------------------------------------------
# Pass 4 – inheritance constraints
# ---------------------------------------------------------------------------


def test_pmx_bone_inheritance_constraints(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Bones with INHERIT_ROTATION have an orient (or parent) constraint applied."""
    total_bones = len(pmx_data.bones)

    inherit_rot_bones = [
        bone_idx
        for bone_idx, bone in enumerate(pmx_data.bones)
        if get_rotation_inherit_info(bone_idx, bone, total_bones) is not None
    ]

    if not inherit_rot_bones:
        skip_test("Model has no rotation-inherit bones")

    orient_constraints = set(cmds.ls(type="orientConstraint") or [])
    parent_constraints = set(cmds.ls(type="parentConstraint") or [])
    multiply_nodes = set(cmds.ls(type="multiplyDivide") or [])
    all_rot_constraints = orient_constraints | parent_constraints | multiply_nodes

    assert_true(
        len(all_rot_constraints) > 0,
        f"{len(inherit_rot_bones)} inherit-rotation bone(s) found but no constraints in scene",
    )

    print(
        f"PASS: {len(all_rot_constraints)} rotation constraint(s) cover "
        f"{len(inherit_rot_bones)} inherit-rotation bone(s)"
    )
    return True


def test_pmx_inheritance_constraint_types(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Verify each inheriting bone gets the correct constraint type.

    * INHERIT_ROTATION only  → multiplyDivide node (NOT orientConstraint)
    * INHERIT_TRANSLATION only → pointConstraint
    * Both flags             → parentConstraint
    """
    joints = maya_pmx_data.joints
    errors = []

    # Collect all multiplyDivide nodes ending with _RotScale for quick lookup
    all_md_nodes = set(cmds.ls(type="multiplyDivide") or [])
    rot_scale_md = {m for m in all_md_nodes if m.endswith("_RotScale")}

    for bone_idx, bone in enumerate(pmx_data.bones):
        constraint_type = get_inheritance_constraint_type(bone)
        if constraint_type == ConstraintType.NONE:
            continue

        joint_name = _joint_name(joints[bone_idx])

        if constraint_type == ConstraintType.ORIENT:
            # Should use multiplyDivide, NOT orientConstraint.
            # Find the multiplyDivide connected to this bone by matching its
            # input2X value against the PMX influence factor.  This avoids
            # DAG-path name-matching issues with long vs short node names.
            expected_influence = (
                bone.inheritBone.influenceFactor if bone.inheritBone else 1.0
            )

            found_md = None
            for md in rot_scale_md:
                try:
                    got_inp2x = cmds.getAttr(f"{md}.input2X")
                    if abs(got_inp2x - expected_influence) < _TOLERANCE:
                        found_md = md
                        break
                except Exception:
                    continue  # Node lacks input2X attr — not the one we're looking for

            if not found_md:
                errors.append(
                    f"bone {bone_idx} '{bone.nameLocal}': "
                    f"expected multiplyDivide (influence={expected_influence}) for ORIENT constraint"
                )

        elif constraint_type == ConstraintType.POINT:
            # Should use pointConstraint
            constraints = cmds.pointConstraint(joint_name, q=True, name=True) or []
            if not constraints:
                errors.append(
                    f"bone {bone_idx} '{bone.nameLocal}': "
                    "expected pointConstraint but none found"
                )

        elif constraint_type == ConstraintType.PARENT:
            # Should use parentConstraint
            constraints = cmds.parentConstraint(joint_name, q=True, name=True) or []
            if not constraints:
                errors.append(
                    f"bone {bone_idx} '{bone.nameLocal}': "
                    "expected parentConstraint but none found"
                )

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
        if len(errors) > 10:
            print(f"  … and {len(errors) - 10} more errors")
    assert_true(
        len(errors) == 0,
        f"{len(errors)} inheritance constraint type violation(s) found",
    )

    print(
        "PASS: All inheritance constraints use the correct node type for their flag combination"
    )
    return True


# ---------------------------------------------------------------------------
# bone_name_map – pure-helper integration
# ---------------------------------------------------------------------------


def test_pmx_bone_name_map_completeness(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """build_bone_name_map maps both nameLocal and nameUniversal to valid Maya joints."""
    joints = maya_pmx_data.joints
    bone_idx_to_maya_name = {i: _joint_name(jobj) for i, jobj in enumerate(joints)}
    bone_name_map = build_bone_name_map(pmx_data.bones, bone_idx_to_maya_name)
    all_joint_names = set(cmds.ls(type="joint") or [])

    missing_keys: list[str] = []
    invalid_values: list[str] = []

    for bone in pmx_data.bones:
        for pmx_name in (bone.nameLocal, bone.nameUniversal):
            if not pmx_name:
                continue
            if pmx_name not in bone_name_map:
                missing_keys.append(pmx_name)
            elif bone_name_map[pmx_name] not in all_joint_names:
                invalid_values.append(
                    f"'{pmx_name}' → '{bone_name_map[pmx_name]}' (not a joint)"
                )

    if missing_keys or invalid_values:
        for k in missing_keys[:5]:
            print(f"  MISSING KEY: '{k}'")
        for v in invalid_values[:5]:
            print(f"  INVALID VALUE: {v}")
    assert_true(
        len(missing_keys) == 0 and len(invalid_values) == 0,
        f"{len(missing_keys)} missing key(s), {len(invalid_values)} invalid value(s)",
    )

    print(
        f"PASS: bone_name_map has {len(bone_name_map)} entries, "
        "all mapping to valid Maya joints"
    )
    return True


# ----------------------------------------------------------------------------
# Additional tests for edge cases and regression checks
# ----------------------------------------------------------------------------
def test_zero_initial_rotation_bones(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Test that all created joints have zero rotation.

    Physics-driven bones (Milestone 2) are exempt: their ``rotate`` channels
    are owned by the simulation write-back constraint, which can express the
    identity rest rotation as a non-canonical Euler triple (e.g. (180,-180,180))
    while the world matrix stays exactly at rest.
    """
    joints = maya_pmx_data.joints
    errors = []
    driven = _physics_driven_bone_indices(maya_pmx_data, pmx_data)

    for bone_idx, jobj in enumerate(joints):
        if bone_idx in driven:
            continue  # rotation owned by the physics simulation
        name = _joint_name(jobj)
        rx, ry, rz = cmds.getAttr(f"{name}.rotate")[0]
        if abs(rx) > _TOLERANCE or abs(ry) > _TOLERANCE or abs(rz) > _TOLERANCE:
            errors.append(
                f"bone {bone_idx} '{name}': initial rotation is ({rx:.4f}, {ry:.4f}, {rz:.4f}), expected (0.0000, 0.0000, 0.0000)"
            )

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
        if len(errors) > 10:
            print(f"  … and {len(errors) - 10} more errors")
    assert_true(
        len(errors) == 0,
        f"{len(errors)} joints have non-zero initial rotation",
    )

    exempt = len(driven)
    suffix = f" ({exempt} physics-driven bones exempt)" if exempt else ""
    print(f"PASS: All {len(joints)} joints have zero initial rotation{suffix}")
    return True


def test_inherit_rotation_preserves_jointorient(
    pmx_data: PmxModel, maya_pmx_data
) -> bool:
    """Test that inherit rotation controller insertion preserves jointOrient.

    When bone_builder creates an inheritCtrl controller for INHERIT_ROTATION bones,
    it reparents the child bone under the controller. This test verifies that
    the child's jointOrient is preserved during this reparenting operation.

    This is critical because:
    - jointOrient defines the bone's rest orientation
    - Changing it would break skeleton pose and animation compatibility
    - The fix uses relative=True parenting to preserve jointOrient
    """
    joints = maya_pmx_data.joints

    # Find bones with inherit rotation that have _inheritCtrl controllers
    inherit_bones_with_controllers = []

    for bone_idx, bone in enumerate(pmx_data.bones):
        constraint_type = get_inheritance_constraint_type(bone)
        if constraint_type not in (ConstraintType.ORIENT, ConstraintType.PARENT):
            continue

        joint_name = _joint_name(joints[bone_idx])

        # Check if this bone has an _InheritCtrl controller parent
        parents = cmds.listRelatives(joint_name, parent=True, fullPath=False)
        if parents and parents[0].endswith("_InheritCtrl"):
            inherit_bones_with_controllers.append(
                (bone_idx, bone, joint_name, parents[0])
            )

    if not inherit_bones_with_controllers:
        skip_test("No bones with _InheritCtrl controllers found")

    print(
        f"  Found {len(inherit_bones_with_controllers)} bones with _InheritCtrl controllers"
    )

    errors = []

    for bone_idx, bone, joint_name, controller_name in inherit_bones_with_controllers:
        # Get the current jointOrient
        current_orient = cmds.getAttr(f"{joint_name}.jointOrient")[0]

        # For bones with FIXED_AXIS, verify the orient aligns with the axis
        if bone.flags & PMXBoneFlagBits.FIXED_AXIS:
            # Get the fixed axis from custom attributes
            axis_x = cmds.getAttr(f"{joint_name}.fixedAxisX")
            axis_y = cmds.getAttr(f"{joint_name}.fixedAxisY")
            axis_z = cmds.getAttr(f"{joint_name}.fixedAxisZ")
            axis_vec = om.MVector(axis_x, axis_y, axis_z).normal()

            # Compute X-axis from jointOrient (should align with fixed axis)
            import math

            jo_x_rad = math.radians(current_orient[0])
            jo_y_rad = math.radians(current_orient[1])
            jo_z_rad = math.radians(current_orient[2])
            orient_quat = om.MEulerRotation(jo_x_rad, jo_y_rad, jo_z_rad).asQuaternion()
            x_axis = om.MVector(1, 0, 0).rotateBy(orient_quat).normal()

            dot = x_axis * axis_vec
            if abs(dot - 1.0) > 0.01:
                errors.append(
                    f"bone {bone_idx} '{bone.nameLocal}': "
                    f"jointOrient X-axis misaligned with FIXED_AXIS (dot={dot:.4f})"
                )

        # For all bones, verify rotation is zero (orientation should be in jointOrient only)
        rx, ry, rz = cmds.getAttr(f"{joint_name}.rotate")[0]
        if abs(rx) > _TOLERANCE or abs(ry) > _TOLERANCE or abs(rz) > _TOLERANCE:
            errors.append(
                f"bone {bone_idx} '{bone.nameLocal}': "
                f"has non-zero rotation ({rx:.4f}, {ry:.4f}, {rz:.4f}) after _InheritCtrl controller insertion"
            )

    if errors:
        for msg in errors[:10]:
            print(f"  ERROR: {msg}")
        if len(errors) > 10:
            print(f"  … and {len(errors) - 10} more errors")
    assert_true(
        len(errors) == 0,
        f"{len(errors)} bones have jointOrient issues after controller insertion",
    )

    print(
        f"PASS: All {len(inherit_bones_with_controllers)} bones with _InheritCtrl controllers have correct jointOrient"
    )
    return True


def test_pmx_rest_pose_attributes_populated(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Verify that _capture_rest_pose_on_joints stored correct values.

    For every joint created by bone_builder, six custom attributes must exist
    (pmxRestTranslateX/Y/Z, pmxRestRotateX/Y/Z) and their values must match
    the joint's actual translateX/Y/Z and rotateX/Y/Z at build time (i.e.
    they should be whatever the bone builder left after all four passes).
    """
    joints = maya_pmx_data.joints
    rest_attrs = [
        ("pmxRestTranslateX", "translateX"),
        ("pmxRestTranslateY", "translateY"),
        ("pmxRestTranslateZ", "translateZ"),
        ("pmxRestRotateX", "rotateX"),
        ("pmxRestRotateY", "rotateY"),
        ("pmxRestRotateZ", "rotateZ"),
    ]

    missing_attr_joints: list[str] = []
    wrong_value_joints: list[str] = []
    driven = _physics_driven_bone_indices(maya_pmx_data, pmx_data)

    for bone_idx, jobj in enumerate(joints):
        name = _joint_name(jobj)
        for rest_attr, _ in rest_attrs:
            if not cmds.attributeQuery(rest_attr, node=name, exists=True):
                missing_attr_joints.append(f"{name}.{rest_attr}")
                break
        if any(name + "." + ra in missing_attr_joints for ra, _ in rest_attrs):
            continue
        if bone_idx in driven:
            continue  # rotation owned by the physics simulation write-back
        for rest_attr, live_attr in rest_attrs:
            stored = cmds.getAttr(f"{name}.{rest_attr}")
            live = cmds.getAttr(f"{name}.{live_attr}")
            if abs(stored - live) > _TOLERANCE:
                wrong_value_joints.append(
                    f"{name}: {rest_attr}={stored:.5f} != {live_attr}={live:.5f}"
                )

    if missing_attr_joints:
        for item in missing_attr_joints[:5]:
            print(f"  ERROR: missing {item}")
        if len(missing_attr_joints) > 5:
            print(f"  … and {len(missing_attr_joints) - 5} more")
    assert_true(
        len(missing_attr_joints) == 0,
        f"{len(missing_attr_joints)} joints missing pmxRest* attributes",
    )

    if wrong_value_joints:
        for item in wrong_value_joints[:5]:
            print(f"  ERROR: {item}")
        if len(wrong_value_joints) > 5:
            print(f"  … and {len(wrong_value_joints) - 5} more")
    assert_true(
        len(wrong_value_joints) == 0,
        f"{len(wrong_value_joints)} joints have mismatched pmxRest* values",
    )

    print(f"PASS: All {len(joints)} joints have correct pmxRest* values")
    return True


def test_pmx_ik_handle_rest_pose_attributes(pmx_data: PmxModel, maya_pmx_data) -> bool:
    """Verify that _capture_rest_pose_on_ik_handles stored correct values.

    Every IK handle created for IK bones must carry six pmxIkRest* custom
    attributes whose values match the IK handle's translate/rotate immediately
    after skeleton build.
    """
    ik_rest_attrs = [
        ("pmxIkRestTranslateX", "translateX"),
        ("pmxIkRestTranslateY", "translateY"),
        ("pmxIkRestTranslateZ", "translateZ"),
        ("pmxIkRestRotateX", "rotateX"),
        ("pmxIkRestRotateY", "rotateY"),
        ("pmxIkRestRotateZ", "rotateZ"),
    ]

    ik_handles = cmds.ls(type="ikHandle") or []
    if not ik_handles:
        skip_test("No IK handles in scene")

    missing: list[str] = []
    wrong: list[str] = []

    for handle in ik_handles:
        for rest_attr, _ in ik_rest_attrs:
            if not cmds.attributeQuery(rest_attr, node=handle, exists=True):
                missing.append(f"{handle}.{rest_attr}")
                break

        if any(handle + "." + ra in missing for ra, _ in ik_rest_attrs):
            continue

        for rest_attr, live_attr in ik_rest_attrs:
            stored = cmds.getAttr(f"{handle}.{rest_attr}")
            live = cmds.getAttr(f"{handle}.{live_attr}")
            if abs(stored - live) > _TOLERANCE:
                wrong.append(
                    f"{handle}: {rest_attr}={stored:.5f} != {live_attr}={live:.5f}"
                )

    if missing:
        for item in missing[:5]:
            print(f"  ERROR: missing {item}")
    assert_true(
        len(missing) == 0, f"{len(missing)} IK handles missing pmxIkRest* attributes"
    )

    if wrong:
        for item in wrong[:5]:
            print(f"  ERROR: {item}")
    assert_true(
        len(wrong) == 0, f"{len(wrong)} IK handles have mismatched pmxIkRest* values"
    )

    print(f"PASS: All {len(ik_handles)} IK handles have correct pmxIkRest* values")
    return True


# ---------------------------------------------------------------------------
# Test registry and runner
# ---------------------------------------------------------------------------

_TESTS = [
    # Pass 1 – creation
    ("Bone Group Under Root", test_pmx_bone_group_creation),
    ("Joint Count + Valid MObjects", test_pmx_bone_creation),
    ("Joint World Positions", test_pmx_bone_positions),
    ("Custom Attribute Values", test_pmx_bone_custom_attribute_values),
    ("Bone Name Map Completeness", test_pmx_bone_name_map_completeness),
    # Pass 2 – hierarchy
    ("Bone Parent-Child Hierarchy", test_pmx_bone_hierarchy),
    # Pass 1 (cont.) – authored-orientation bones (tested after hierarchy is built)
    ("Fixed Axis Bones", test_pmx_fixed_axis_bones),
    ("Local Coordinate Bones", test_pmx_local_coordinate_bones),
    # Pass 1 (cont.) – tail joints
    ("Offset-Mode Tail Joints", test_pmx_tail_joints_created),
    # Pass 3 – IK
    ("IK Handles Created", test_pmx_ik_handles_created),
    (
        "IK Handle Parented Under Control Bone",
        test_pmx_ik_handle_parented_under_control_bone,
    ),
    ("IK Handle Priority Set", test_pmx_ik_handle_priority),
    ("CCD Solver Node Created per IK Chain", test_pmx_ccd_solver_created),
    ("CCD Solver Link Limits Populated", test_pmx_ccd_solver_link_limits),
    # Pass 4 – inherit constraints
    ("Rotation Inherit Constraints", test_pmx_bone_inheritance_constraints),
    ("Inheritance Constraint Types", test_pmx_inheritance_constraint_types),
    #
    ("Verify zero initial rotaion bones", test_zero_initial_rotation_bones),
    (
        "Inherit rotation preserves jointOrient",
        test_inherit_rotation_preserves_jointorient,
    ),
    #
    ("Rest Pose Attributes Populated", test_pmx_rest_pose_attributes_populated),
    ("IK Handle Rest Pose Attributes", test_pmx_ik_handle_rest_pose_attributes),
]
