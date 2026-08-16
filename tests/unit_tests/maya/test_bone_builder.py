"""
test_bone_builder.py

Unit tests for pure logic functions in mmd.maya.pmx.bone_builder.

This file tests only the PURE functions that don't depend on Maya API.
These functions contain the decision logic and validation that can be
tested without mocking Maya.

Maya API calls (joint creation, parenting, constraints, etc.) are tested
in integration tests that run inside Maya, not here.
"""

import unittest

from mmd.core.data_types import (
    IK,
    IKLink,
    InheritBone,
    PMXBone,
    PMXBoneFlagBits,
    Vec3,
)
from mmd.maya.pmx.bone_builder import (
    ConstraintType,
    IKChainInfo,
    RotationInheritInfo,
    build_bone_name_map,
    get_ik_chain_info,
    get_inheritance_constraint_type,
    get_rotation_inherit_info,
)

# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------


def _make_bone(
    name_local: str = "bone",
    name_universal: str = "bone",
    parent_index: int = -1,
    position: Vec3 = None,
    flags: PMXBoneFlagBits = PMXBoneFlagBits.ROTATABLE | PMXBoneFlagBits.VISIBLE,
    tail_info=None,
    ik=None,
    inherit_bone=None,
    fixed_axis=None,
    local_coordinate=None,
) -> PMXBone:
    """Convenience factory for test PMXBone instances."""
    if position is None:
        position = Vec3(0.0, 0.0, 0.0)
    if tail_info is None:
        tail_info = 0
    return PMXBone(
        nameLocal=name_local,
        nameUniversal=name_universal,
        position=position,
        parentIndex=parent_index,
        level=0,
        flags=flags | PMXBoneFlagBits.INDEXED_TAIL_POSITION,
        tailInfo=tail_info,
        inheritBone=inherit_bone,
        fixedAxis=fixed_axis,
        localCoordinate=local_coordinate,
        externalParent=None,
        ik=ik,
    )


# ---------------------------------------------------------------------------
# Pure function tests
# ---------------------------------------------------------------------------


class TestGetIKChainInfo(unittest.TestCase):
    """get_ik_chain_info validates IK data and returns IKChainInfo or None."""

    def _make_ik_bone(
        self,
        target_idx,
        links,
        flags_extra: PMXBoneFlagBits | None = None,
    ):
        ik_data = IK(
            targetBoneIndex=target_idx,
            loopCount=20,
            limitRadian=0.1,
            links=links,
        )
        flags = PMXBoneFlagBits.IK | PMXBoneFlagBits.ROTATABLE
        if flags_extra is not None:
            flags = flags | flags_extra
        return _make_bone(name_local="ik_bone", flags=flags, ik=ik_data)

    def test_valid_chain_returns_ik_chain_info(self):
        links = [IKLink(boneIndex=1), IKLink(boneIndex=2)]
        bone = self._make_ik_bone(target_idx=0, links=links)
        result = get_ik_chain_info(bone, total_bone_count=3)
        self.assertIsInstance(result, IKChainInfo)
        self.assertEqual(result.target_bone_idx, 0)
        self.assertEqual(result.start_bone_idx, 2)

    def test_no_ik_flag_returns_none(self):
        bone = _make_bone(flags=PMXBoneFlagBits.ROTATABLE)
        self.assertIsNone(get_ik_chain_info(bone, 5))

    def test_ik_flag_but_ik_none_returns_none(self):
        bone = _make_bone(flags=PMXBoneFlagBits.IK | PMXBoneFlagBits.ROTATABLE)
        self.assertIsNone(get_ik_chain_info(bone, 5))

    def test_target_index_negative_returns_none(self):
        bone = self._make_ik_bone(target_idx=-1, links=[IKLink(boneIndex=0)])
        self.assertIsNone(get_ik_chain_info(bone, 5))

    def test_target_index_out_of_range_returns_none(self):
        bone = self._make_ik_bone(target_idx=10, links=[IKLink(boneIndex=0)])
        self.assertIsNone(get_ik_chain_info(bone, 5))

    def test_empty_links_returns_none(self):
        bone = self._make_ik_bone(target_idx=0, links=[])
        self.assertIsNone(get_ik_chain_info(bone, 5))

    def test_start_bone_index_out_of_range_returns_none(self):
        bone = self._make_ik_bone(target_idx=0, links=[IKLink(boneIndex=10)])
        self.assertIsNone(get_ik_chain_info(bone, 5))

    def test_start_bone_is_last_link(self):
        links = [IKLink(boneIndex=1), IKLink(boneIndex=2), IKLink(boneIndex=3)]
        bone = self._make_ik_bone(target_idx=0, links=links)
        result = get_ik_chain_info(bone, 5)
        self.assertIsNotNone(result)
        self.assertEqual(result.start_bone_idx, 3)


class TestGetRotationInheritInfo(unittest.TestCase):
    """get_rotation_inherit_info validates inherit data and returns RotationInheritInfo or None."""

    def _make_inherit_bone(self, parent_idx, influence):
        inherit_data = InheritBone(
            parentBoneIndex=parent_idx, influenceFactor=influence
        )
        flags = PMXBoneFlagBits.INHERIT_ROTATION | PMXBoneFlagBits.ROTATABLE
        return _make_bone(
            name_local="inherit_bone", flags=flags, inherit_bone=inherit_data
        )

    def test_valid_returns_rotation_inherit_info(self):
        bone = self._make_inherit_bone(parent_idx=1, influence=0.5)
        result = get_rotation_inherit_info(bone_idx=2, bone=bone, total_bone_count=5)
        self.assertIsInstance(result, RotationInheritInfo)
        self.assertEqual(result.parent_bone_idx, 1)
        self.assertEqual(result.influence, 0.5)

    def test_no_inherit_flag_returns_none(self):
        bone = _make_bone(flags=PMXBoneFlagBits.ROTATABLE)
        self.assertIsNone(get_rotation_inherit_info(0, bone, 5))

    def test_inherit_flag_but_no_inherit_bone_returns_none(self):
        bone = _make_bone(
            flags=PMXBoneFlagBits.INHERIT_ROTATION | PMXBoneFlagBits.ROTATABLE
        )
        self.assertIsNone(get_rotation_inherit_info(0, bone, 5))

    def test_parent_index_negative_returns_none(self):
        bone = self._make_inherit_bone(parent_idx=-1, influence=1.0)
        self.assertIsNone(get_rotation_inherit_info(2, bone, 5))

    def test_parent_index_out_of_range_returns_none(self):
        bone = self._make_inherit_bone(parent_idx=10, influence=1.0)
        self.assertIsNone(get_rotation_inherit_info(2, bone, 5))

    def test_parent_index_equal_to_count_returns_none(self):
        bone = self._make_inherit_bone(parent_idx=5, influence=1.0)
        self.assertIsNone(get_rotation_inherit_info(2, bone, 5))

    def test_full_influence(self):
        bone = self._make_inherit_bone(parent_idx=0, influence=1.0)
        result = get_rotation_inherit_info(1, bone, 5)
        self.assertEqual(result.influence, 1.0)

    def test_zero_influence(self):
        bone = self._make_inherit_bone(parent_idx=0, influence=0.0)
        result = get_rotation_inherit_info(1, bone, 5)
        self.assertEqual(result.influence, 0.0)


class TestBuildBoneNameMap(unittest.TestCase):
    """build_bone_name_map produces correct PMX-name → Maya-joint-name lookup."""

    def test_local_and_universal_both_mapped(self):
        bones = [_make_bone(name_local="Local", name_universal="Universal")]
        mapping = build_bone_name_map(bones, {0: "joint1"})
        self.assertEqual(mapping["Local"], "joint1")
        self.assertEqual(mapping["Universal"], "joint1")

    def test_equal_names_produce_single_entry(self):
        bones = [_make_bone(name_local="Same", name_universal="Same")]
        mapping = build_bone_name_map(bones, {0: "joint1"})
        self.assertEqual(len(mapping), 1)
        self.assertEqual(mapping["Same"], "joint1")

    def test_empty_local_name_not_mapped(self):
        bones = [_make_bone(name_local="", name_universal="Universal")]
        mapping = build_bone_name_map(bones, {0: "joint1"})
        self.assertNotIn("", mapping)

    def test_empty_universal_name_not_mapped(self):
        bones = [_make_bone(name_local="Local", name_universal="")]
        mapping = build_bone_name_map(bones, {0: "joint1"})
        self.assertEqual(len(mapping), 1)

    def test_empty_bone_list_returns_empty_dict(self):
        mapping = build_bone_name_map([], {})
        self.assertEqual(len(mapping), 0)

    def test_multiple_bones(self):
        bones = [
            _make_bone(name_local="Bone1L", name_universal="Bone1U"),
            _make_bone(name_local="Bone2L", name_universal="Bone2U"),
        ]
        idx_to_maya = {0: "maya_bone1", 1: "maya_bone2"}
        mapping = build_bone_name_map(bones, idx_to_maya)
        self.assertEqual(mapping["Bone1L"], "maya_bone1")
        self.assertEqual(mapping["Bone1U"], "maya_bone1")
        self.assertEqual(mapping["Bone2L"], "maya_bone2")
        self.assertEqual(mapping["Bone2U"], "maya_bone2")

    def test_maya_name_is_actual_not_pmx(self):
        bones = [_make_bone(name_local="PMXName", name_universal="PMXName")]
        mapping = build_bone_name_map(bones, {0: "MayaName_1"})
        self.assertEqual(mapping["PMXName"], "MayaName_1")


class TestGetInheritanceConstraintType(unittest.TestCase):
    """get_inheritance_constraint_type determines the correct constraint approach."""

    def test_no_inherit_bone_returns_none(self):
        bone = _make_bone()
        result = get_inheritance_constraint_type(bone)
        self.assertEqual(result, ConstraintType.NONE)

    def test_rotation_only_returns_expression(self):
        inherit_data = InheritBone(parentBoneIndex=0, influenceFactor=1.0)
        bone = _make_bone(
            flags=PMXBoneFlagBits.INHERIT_ROTATION, inherit_bone=inherit_data
        )
        result = get_inheritance_constraint_type(bone)
        self.assertEqual(result, ConstraintType.ORIENT)

    def test_translation_only_returns_point(self):
        inherit_data = InheritBone(parentBoneIndex=0, influenceFactor=1.0)
        bone = _make_bone(
            flags=PMXBoneFlagBits.INHERIT_TRANSLATION, inherit_bone=inherit_data
        )
        result = get_inheritance_constraint_type(bone)
        self.assertEqual(result, ConstraintType.POINT)

    def test_both_rotation_and_translation_returns_parent(self):
        inherit_data = InheritBone(parentBoneIndex=0, influenceFactor=1.0)
        bone = _make_bone(
            flags=PMXBoneFlagBits.INHERIT_ROTATION
            | PMXBoneFlagBits.INHERIT_TRANSLATION,
            inherit_bone=inherit_data,
        )
        result = get_inheritance_constraint_type(bone)
        self.assertEqual(result, ConstraintType.PARENT)

    def test_neither_flag_returns_none(self):
        inherit_data = InheritBone(parentBoneIndex=0, influenceFactor=1.0)
        bone = _make_bone(inherit_bone=inherit_data)
        result = get_inheritance_constraint_type(bone)
        self.assertEqual(result, ConstraintType.NONE)


if __name__ == "__main__":
    unittest.main()
