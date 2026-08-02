# pylint: disable=missing-function-docstring,missing-module-docstring
import struct

import pytest

from mmd.core.binary_reader import BinaryReader
from mmd.core.data_types import (
    BoneFrameElement,
    FrameData,
    FrameType,
    JointType,
    MorphFrameElement,
    MorphType,
    PhysicsMode,
    PMXBoneFlagBits,
    ShapeType,
    Vec3,
    WeightType,
)
from mmd.core.pmx_importer import (
    PMXParseError,
    create_pmx_reader,
    parse_pmx,
    read_bones_data,
    read_display_frames_data,
    read_indices_data,
    read_joint_data,
    read_morphs_data,
    read_pmx_header,
    read_rigid_body_data,
    read_vertex_data,
)
from mmd.core.pmx_reader import PMXConfig, PMXReader

# Helper to create a minimal valid PMX header binary


def make_pmx_header_bytes():
    magic = b"PMX "
    version = struct.pack("<f", 2.0)
    global_count = 8
    globals_list = [1, 0, 1, 2, 2, 1, 1, 1]
    globals_bytes = bytes(globals_list)
    # Model names/comments: 4 empty utf8 strings
    empty_str = struct.pack("<I", 0)
    return magic + version + bytes([global_count]) + globals_bytes + empty_str * 4


def test_read_pmx_header_valid():
    data = make_pmx_header_bytes()
    reader = BinaryReader(data)
    header = read_pmx_header(reader)
    assert header.magic == b"PMX "
    assert header.version == 2.0
    assert header.global_count == 8
    assert header.globals == [1, 0, 1, 2, 2, 1, 1, 1]
    assert header.model_name_local == ""
    assert header.model_name_universal == ""
    assert header.model_comment_local == ""
    assert header.model_comment_universal == ""


def test_read_pmx_header_invalid_magic():
    data = (
        b"BAD!"
        + struct.pack("<f", 2.0)
        + bytes([8])
        + bytes([1, 0, 1, 2, 2, 1, 1, 1])
        + struct.pack("<I", 0) * 4
    )
    reader = BinaryReader(data)
    with pytest.raises(PMXParseError, match="Invalid PMX file: incorrect magic number"):
        read_pmx_header(reader)


def test_parse_pmx_file_not_found():
    with pytest.raises(PMXParseError, match="Failed to open PMX file"):
        parse_pmx("nonexistent_file.pmx")


def test_parse_pmx_wraps_parse_errors(tmp_path):
    pmx_path = tmp_path / "invalid_header.pmx"
    pmx_path.write_bytes(b"BAD!")

    with pytest.raises(PMXParseError, match="Failed to parse PMX file"):
        parse_pmx(str(pmx_path))


def test_create_pmx_reader():
    data = make_pmx_header_bytes()
    reader = BinaryReader(data)
    header = read_pmx_header(reader)
    pmx_reader = create_pmx_reader(reader, header)
    assert isinstance(pmx_reader, PMXReader)
    assert isinstance(pmx_reader.config, PMXConfig)


def test_read_vertex_data_bdef1():
    # Vertex count = 1
    # position, normal, uv: 3*3 + 3*3 + 2*4 bytes
    # weight type: BDEF1 (0)
    # bone index: int32 (default config)
    # edge scale: float
    vertex_count = 1
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]  # index sizes = 4
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)
    # Build binary data
    data = struct.pack("<I", vertex_count)
    # position
    data += struct.pack("<fff", 1.0, 2.0, 3.0)
    # normal
    data += struct.pack("<fff", 0.1, 0.2, 0.3)
    # uv
    data += struct.pack("<ff", 0.5, 0.6)
    # weight type
    data += struct.pack("<b", WeightType.BDEF1.value)
    # bone index
    data += struct.pack("<i", 42)
    # edge scale
    data += struct.pack("<f", 1.5)
    pmx_reader.reader = BinaryReader(data)
    vertices = read_vertex_data(pmx_reader)
    assert len(vertices) == 1
    v = vertices[0]
    assert v.position.x == pytest.approx(1.0)
    assert v.position.y == pytest.approx(2.0)
    assert v.position.z == pytest.approx(3.0)

    # normal
    assert v.normal.x == pytest.approx(0.1)
    assert v.normal.y == pytest.approx(0.2)
    assert v.normal.z == pytest.approx(0.3)

    # uv
    assert v.uv.x == pytest.approx(0.5)
    assert v.uv.y == pytest.approx(0.6)

    # weight
    assert v.weight_type == WeightType.BDEF1
    assert v.weight.bone_index == [42]
    assert v.weight.weight == pytest.approx([1.0])
    assert v.edge_scale == pytest.approx(1.5)


def test_read_vertex_data_bdef2():
    vertex_count = 1
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)
    data = struct.pack("<I", vertex_count)
    data += struct.pack("<fff", 1.0, 2.0, 3.0)  # position
    data += struct.pack("<fff", 0.1, 0.2, 0.3)  # normal
    data += struct.pack("<ff", 0.5, 0.6)  # uv
    data += struct.pack("<b", WeightType.BDEF2.value)  # weight type
    data += struct.pack("<i", 10)  # bone 0
    data += struct.pack("<i", 20)  # bone 1
    data += struct.pack("<f", 0.7)  # weight
    data += struct.pack("<f", 2.0)  # edge scale
    pmx_reader.reader = BinaryReader(data)
    vertices = read_vertex_data(pmx_reader)
    v = vertices[0]
    assert v.weight_type == WeightType.BDEF2
    assert v.weight.bone_index == [10, 20]
    assert v.weight.weight == pytest.approx([0.7, 0.3])
    assert v.edge_scale == pytest.approx(2.0)


def test_read_vertex_data_bdef4():
    vertex_count = 1
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)
    data = struct.pack("<I", vertex_count)
    data += struct.pack("<fff", 1.0, 2.0, 3.0)  # position
    data += struct.pack("<fff", 0.1, 0.2, 0.3)  # normal
    data += struct.pack("<ff", 0.5, 0.6)  # uv
    data += struct.pack("<b", WeightType.BDEF4.value)  # weight type
    data += struct.pack("<i", 1)  # bone 0
    data += struct.pack("<i", 2)  # bone 1
    data += struct.pack("<i", 3)  # bone 2
    data += struct.pack("<i", 4)  # bone 3
    data += struct.pack("<f", 0.1)  # w0
    data += struct.pack("<f", 0.2)  # w1
    data += struct.pack("<f", 0.3)  # w2
    data += struct.pack("<f", 0.4)  # w3
    data += struct.pack("<f", 3.0)  # edge scale
    pmx_reader.reader = BinaryReader(data)
    vertices = read_vertex_data(pmx_reader)
    v = vertices[0]
    assert v.weight_type == WeightType.BDEF4
    assert v.weight.bone_index == [1, 2, 3, 4]
    assert v.weight.weight == pytest.approx([0.1, 0.2, 0.3, 0.4])
    assert v.edge_scale == pytest.approx(3.0)


def test_read_vertex_data_sdef():
    vertex_count = 1
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)
    data = struct.pack("<I", vertex_count)
    data += struct.pack("<fff", 1.0, 2.0, 3.0)  # position
    data += struct.pack("<fff", 0.1, 0.2, 0.3)  # normal
    data += struct.pack("<ff", 0.5, 0.6)  # uv
    data += struct.pack("<b", WeightType.SDEF.value)  # weight type
    data += struct.pack("<i", 5)  # bone 0
    data += struct.pack("<i", 6)  # bone 1
    data += struct.pack("<f", 0.25)  # weight
    # SDEF params: C, R0, R1 (each Vec3)
    data += struct.pack("<fff", 1.1, 1.2, 1.3)  # C
    data += struct.pack("<fff", 2.1, 2.2, 2.3)  # R0
    data += struct.pack("<fff", 3.1, 3.2, 3.3)  # R1
    data += struct.pack("<f", 4.0)  # edge scale
    pmx_reader.reader = BinaryReader(data)
    vertices = read_vertex_data(pmx_reader)
    v = vertices[0]
    assert v.weight_type == WeightType.SDEF
    assert v.weight.bone_index == [5, 6]
    assert v.weight.weight == pytest.approx([0.25, 0.75])
    assert v.edge_scale == pytest.approx(4.0)


def test_read_vertex_data_qdef():
    vertex_count = 1
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)
    data = struct.pack("<I", vertex_count)
    data += struct.pack("<fff", 1.0, 2.0, 3.0)  # position
    data += struct.pack("<fff", 0.1, 0.2, 0.3)  # normal
    data += struct.pack("<ff", 0.5, 0.6)  # uv
    data += struct.pack("<b", WeightType.QDEF.value)  # weight type
    data += struct.pack("<i", 11)  # bone 0
    data += struct.pack("<i", 12)  # bone 1
    data += struct.pack("<i", 13)  # bone 2
    data += struct.pack("<i", 14)  # bone 3
    data += struct.pack("<f", 0.1)  # w0
    data += struct.pack("<f", 0.2)  # w1
    data += struct.pack("<f", 0.3)  # w2
    data += struct.pack("<f", 0.4)  # w3
    data += struct.pack("<f", 5.0)  # edge scale
    pmx_reader.reader = BinaryReader(data)
    vertices = read_vertex_data(pmx_reader)
    v = vertices[0]
    assert v.weight_type == WeightType.QDEF
    assert v.weight.bone_index == [11, 12, 13, 14]
    assert v.weight.weight == pytest.approx([0.1, 0.2, 0.3, 0.4])
    assert v.edge_scale == pytest.approx(5.0)


def test_read_indices_data():
    # index_count = 3, indices = 1,2,3
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)
    data = struct.pack("<I", 3) + struct.pack("<iii", 1, 2, 3)
    pmx_reader.reader = BinaryReader(data)
    indices = read_indices_data(pmx_reader)
    assert indices == [1, 2, 3]


# --- Bone Data Tests ---
def test_read_bones_data_simple_bone():
    """Test reading a single bone with minimal flags"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]  # bone index size = 4
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    # Build binary data
    data = struct.pack("<I", 1)  # bone_count = 1

    # Bone name local (UTF-8)
    name_local = "RootBone"
    data += struct.pack("<I", len(name_local.encode("utf-8")))
    data += name_local.encode("utf-8")

    # Bone name universal (UTF-8)
    name_universal = "Root"
    data += struct.pack("<I", len(name_universal.encode("utf-8")))
    data += name_universal.encode("utf-8")

    # Position
    data += struct.pack("<fff", 0.0, 10.0, 0.0)

    # Parent index
    data += struct.pack("<i", -1)

    # Level
    data += struct.pack("<i", 0)

    # Bone flags (no flags set - use tail position)
    data += struct.pack("<H", 0)

    # Tail position (not indexed)
    data += struct.pack("<fff", 0.0, 5.0, 0.0)

    pmx_reader.reader = BinaryReader(data)
    bones = read_bones_data(pmx_reader)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.nameLocal == "RootBone"
    assert bone.nameUniversal == "Root"
    assert bone.position.x == pytest.approx(0.0)
    assert bone.position.y == pytest.approx(10.0)
    assert bone.position.z == pytest.approx(0.0)
    assert bone.parentIndex == -1
    assert bone.level == 0
    assert bone.flags == 0
    assert bone.tailInfo.x == pytest.approx(0.0)
    assert bone.tailInfo.y == pytest.approx(5.0)
    assert bone.tailInfo.z == pytest.approx(0.0)


def test_read_bones_data_indexed_tail():
    """Test reading bone with indexed tail position"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)  # bone_count = 1

    # Names
    data += struct.pack("<I", 0)  # empty local name
    data += struct.pack("<I", 0)  # empty universal name

    # Position
    data += struct.pack("<fff", 1.0, 2.0, 3.0)

    # Parent index
    data += struct.pack("<i", 0)

    # Level
    data += struct.pack("<i", 1)

    # Bone flags - INDEXED_TAIL_POSITION
    flags = PMXBoneFlagBits.INDEXED_TAIL_POSITION
    data += struct.pack("<H", flags)

    # Tail bone index
    data += struct.pack("<i", 5)

    pmx_reader.reader = BinaryReader(data)
    bones = read_bones_data(pmx_reader)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.flags & PMXBoneFlagBits.INDEXED_TAIL_POSITION
    assert bone.tailInfo == 5  # Indexed tail


def test_read_bones_data_inherit_rotation():
    """Test reading bone with inherit rotation"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)  # bone_count = 1

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)

    # Parent index
    data += struct.pack("<i", 0)

    # Level
    data += struct.pack("<i", 1)

    # Bone flags - INHERIT_ROTATION
    flags = PMXBoneFlagBits.INHERIT_ROTATION
    data += struct.pack("<H", flags)

    # Tail position (not indexed)
    data += struct.pack("<fff", 0.0, 1.0, 0.0)

    # Inherit bone index
    data += struct.pack("<i", 3)

    # Inherit weight
    data += struct.pack("<f", 0.75)

    pmx_reader.reader = BinaryReader(data)
    bones = read_bones_data(pmx_reader)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.flags & PMXBoneFlagBits.INHERIT_ROTATION
    assert bone.inheritBone is not None
    assert bone.inheritBone.parentBoneIndex == 3
    assert bone.inheritBone.influenceFactor == pytest.approx(0.75)


def test_read_bones_data_fixed_axis():
    """Test reading bone with fixed axis"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)

    # Parent index
    data += struct.pack("<i", -1)

    # Level
    data += struct.pack("<i", 0)

    # Bone flags - FIXED_AXIS
    flags = PMXBoneFlagBits.FIXED_AXIS
    data += struct.pack("<H", flags)

    # Tail position
    data += struct.pack("<fff", 0.0, 1.0, 0.0)

    # Fixed axis vector
    data += struct.pack("<fff", 0.0, 1.0, 0.0)

    pmx_reader.reader = BinaryReader(data)
    bones = read_bones_data(pmx_reader)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.flags & PMXBoneFlagBits.FIXED_AXIS
    assert bone.fixedAxis is not None
    assert bone.fixedAxis.axis.x == pytest.approx(0.0)
    assert bone.fixedAxis.axis.y == pytest.approx(1.0)
    assert bone.fixedAxis.axis.z == pytest.approx(0.0)


def test_read_bones_data_local_coordinate():
    """Test reading bone with local coordinate system"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)

    # Parent index
    data += struct.pack("<i", -1)

    # Level
    data += struct.pack("<i", 0)

    # Bone flags - LOCAL_COORDINATE
    flags = PMXBoneFlagBits.LOCAL_COORDINATE
    data += struct.pack("<H", flags)

    # Tail position
    data += struct.pack("<fff", 0.0, 1.0, 0.0)

    # Local X axis
    data += struct.pack("<fff", 1.0, 0.0, 0.0)

    # Local Z axis
    data += struct.pack("<fff", 0.0, 0.0, 1.0)

    pmx_reader.reader = BinaryReader(data)
    bones = read_bones_data(pmx_reader)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.flags & PMXBoneFlagBits.LOCAL_COORDINATE
    assert bone.localCoordinate is not None
    assert bone.localCoordinate.xAxis.x == pytest.approx(1.0)
    assert bone.localCoordinate.zAxis.z == pytest.approx(1.0)


def test_read_bones_data_external_parent():
    """Test reading bone with external parent deform"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)

    # Parent index
    data += struct.pack("<i", -1)

    # Level
    data += struct.pack("<i", 0)

    # Bone flags - EXTERNAL_PARENT_DEFORM
    flags = PMXBoneFlagBits.EXTERNAL_PARENT_DEFORM
    data += struct.pack("<H", flags)

    # Tail position
    data += struct.pack("<fff", 0.0, 1.0, 0.0)

    # External parent key
    data += struct.pack("<i", 10)

    pmx_reader.reader = BinaryReader(data)
    bones = read_bones_data(pmx_reader)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.flags & PMXBoneFlagBits.EXTERNAL_PARENT_DEFORM
    assert bone.externalParent is not None
    assert bone.externalParent.parentBoneIndex == 10


def test_read_bones_data_ik():
    """Test reading bone with IK"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)

    # Parent index
    data += struct.pack("<i", -1)

    # Level
    data += struct.pack("<i", 0)

    # Bone flags - IK
    flags = PMXBoneFlagBits.IK
    data += struct.pack("<H", flags)

    # Tail position
    data += struct.pack("<fff", 0.0, 1.0, 0.0)

    # IK target bone index
    data += struct.pack("<i", 5)

    # IK loop count
    data += struct.pack("<i", 40)

    # IK limit radian
    data += struct.pack("<f", 1.57)

    # IK link count
    data += struct.pack("<i", 2)

    # IK link 1 (with limits)
    data += struct.pack("<i", 3)  # bone index
    data += struct.pack("<b", 1)  # has limits
    data += struct.pack("<fff", -3.14, -0.1, -0.5)  # lower limit
    data += struct.pack("<fff", 3.14, 0.1, 0.5)  # upper limit

    # IK link 2 (without limits)
    data += struct.pack("<i", 4)  # bone index
    data += struct.pack("<b", 0)  # no limits

    pmx_reader.reader = BinaryReader(data)
    bones = read_bones_data(pmx_reader)

    assert len(bones) == 1
    bone = bones[0]
    assert bone.flags & PMXBoneFlagBits.IK
    assert bone.ik is not None
    assert bone.ik.targetBoneIndex == 5
    assert bone.ik.loopCount == 40
    assert bone.ik.limitRadian == pytest.approx(1.57)
    assert len(bone.ik.links) == 2

    # Check first link (with limits)
    link1 = bone.ik.links[0]
    assert link1.boneIndex == 3
    assert link1.rotationLimitMin is not None
    assert link1.rotationLimitMax is not None
    assert link1.rotationLimitMin.x == pytest.approx(-3.14)

    # Check second link (without limits)
    link2 = bone.ik.links[1]
    assert link2.boneIndex == 4
    assert link2.rotationLimitMin is None
    assert link2.rotationLimitMax is None


def test_read_bones_data_multiple_bones():
    """Test reading multiple bones"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 3)  # bone_count = 3

    # Bone 1
    data += struct.pack("<I", 0)  # empty names
    data += struct.pack("<I", 0)
    data += struct.pack("<fff", 0.0, 0.0, 0.0)  # position
    data += struct.pack("<i", -1)  # parent
    data += struct.pack("<i", 0)  # level
    data += struct.pack("<H", 0)  # flags
    data += struct.pack("<fff", 0.0, 1.0, 0.0)  # tail

    # Bone 2
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)
    data += struct.pack("<fff", 1.0, 1.0, 1.0)
    data += struct.pack("<i", 0)  # parent is bone 0
    data += struct.pack("<i", 1)
    data += struct.pack("<H", 0)
    data += struct.pack("<fff", 1.0, 2.0, 1.0)

    # Bone 3
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)
    data += struct.pack("<fff", 2.0, 2.0, 2.0)
    data += struct.pack("<i", 1)  # parent is bone 1
    data += struct.pack("<i", 2)
    data += struct.pack("<H", 0)
    data += struct.pack("<fff", 2.0, 3.0, 2.0)

    pmx_reader.reader = BinaryReader(data)
    bones = read_bones_data(pmx_reader)

    assert len(bones) == 3
    assert bones[0].parentIndex == -1
    assert bones[1].parentIndex == 0
    assert bones[2].parentIndex == 1
    assert bones[0].level == 0
    assert bones[1].level == 1
    assert bones[2].level == 2


# --- Morph Data Tests ---
def test_read_morphs_data_group():
    """Test reading a group morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]  # morph index size = 4
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    # Build binary data
    data = struct.pack("<I", 1)  # morph_count = 1

    # Morph name local (UTF-8)
    name_local = "GroupMorph"
    data += struct.pack("<I", len(name_local.encode("utf-8")))
    data += name_local.encode("utf-8")

    # Morph name universal (UTF-8)
    name_universal = "Group"
    data += struct.pack("<I", len(name_universal.encode("utf-8")))
    data += name_universal.encode("utf-8")

    # Panel type
    data += struct.pack("<b", 1)

    # Morph type - GROUP
    data += struct.pack("<b", MorphType.GROUP.value)

    # Element count
    data += struct.pack("<i", 2)

    # Group element 1
    data += struct.pack("<i", 5)  # morph index
    data += struct.pack("<f", 0.75)  # weight

    # Group element 2
    data += struct.pack("<i", 10)  # morph index
    data += struct.pack("<f", 0.5)  # weight

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.name_local == "GroupMorph"
    assert morph.name_universal == "Group"
    assert morph.panel_type == 1
    assert morph.morph_type == MorphType.GROUP
    assert len(morph.data) == 2
    assert morph.data[0].morph_index == 5
    assert morph.data[0].morph_value == pytest.approx(0.75)
    assert morph.data[1].morph_index == 10
    assert morph.data[1].morph_value == pytest.approx(0.5)


def test_read_morphs_data_vertex():
    """Test reading a vertex morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)  # morph_count = 1

    # Names
    data += struct.pack("<I", 0)  # empty local name
    data += struct.pack("<I", 0)  # empty universal name

    # Panel type
    data += struct.pack("<b", 2)

    # Morph type - VERTEX
    data += struct.pack("<b", MorphType.VERTEX.value)

    # Element count
    data += struct.pack("<i", 1)

    # Vertex element
    data += struct.pack("<i", 42)  # vertex index
    data += struct.pack("<fff", 1.5, 2.5, 3.5)  # position offset

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.morph_type == MorphType.VERTEX
    assert len(morph.data) == 1
    assert morph.data[0].vertex_index == 42
    assert morph.data[0].offset.x == pytest.approx(1.5)
    assert morph.data[0].offset.y == pytest.approx(2.5)
    assert morph.data[0].offset.z == pytest.approx(3.5)


def test_read_morphs_data_bone():
    """Test reading a bone morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)  # morph_count = 1

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Panel type
    data += struct.pack("<b", 3)

    # Morph type - BONE
    data += struct.pack("<b", MorphType.BONE.value)

    # Element count
    data += struct.pack("<i", 1)

    # Bone element
    data += struct.pack("<i", 15)  # bone index
    data += struct.pack("<fff", 0.1, 0.2, 0.3)  # position offset
    data += struct.pack("<ffff", 0.0, 0.707, 0.0, 0.707)  # rotation offset (quaternion)

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.morph_type == MorphType.BONE
    assert len(morph.data) == 1
    assert morph.data[0].bone_index == 15
    assert morph.data[0].position_offset.x == pytest.approx(0.1)
    assert morph.data[0].position_offset.y == pytest.approx(0.2)
    assert morph.data[0].position_offset.z == pytest.approx(0.3)
    assert morph.data[0].rotation_offset.x == pytest.approx(0.0)
    assert morph.data[0].rotation_offset.y == pytest.approx(0.707)
    assert morph.data[0].rotation_offset.z == pytest.approx(0.0)
    assert morph.data[0].rotation_offset.w == pytest.approx(0.707)


def test_read_morphs_data_uv():
    """Test reading a UV morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)  # morph_count = 1

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Panel type
    data += struct.pack("<b", 4)

    # Morph type - UV
    data += struct.pack("<b", MorphType.UV.value)

    # Element count
    data += struct.pack("<i", 1)

    # UV element
    data += struct.pack("<i", 100)  # vertex index
    data += struct.pack("<ffff", 0.5, 0.25, 0.0, 0.0)  # uv offset

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.morph_type == MorphType.UV
    assert len(morph.data) == 1
    assert morph.data[0].vertex_index == 100
    assert morph.data[0].offset.x == pytest.approx(0.5)
    assert morph.data[0].offset.y == pytest.approx(0.25)


def test_read_morphs_data_uv1():
    """Test reading a UV1 morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Panel type
    data += struct.pack("<b", 4)

    # Morph type - UV1
    data += struct.pack("<b", MorphType.UV1.value)

    # Element count
    data += struct.pack("<i", 1)

    # UV1 element
    data += struct.pack("<i", 200)  # vertex index
    data += struct.pack("<ffff", 0.1, 0.2, 0.3, 0.4)  # uv offset

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.morph_type == MorphType.UV1
    assert len(morph.data) == 1
    assert morph.data[0].vertex_index == 200
    assert morph.data[0].offset.x == pytest.approx(0.1)
    assert morph.data[0].offset.y == pytest.approx(0.2)
    assert morph.data[0].offset.z == pytest.approx(0.3)
    assert morph.data[0].offset.w == pytest.approx(0.4)


def test_read_morphs_data_uv2():
    """Test reading a UV2 morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Panel type
    data += struct.pack("<b", 4)

    # Morph type - UV2
    data += struct.pack("<b", MorphType.UV2.value)

    # Element count
    data += struct.pack("<i", 1)

    # UV2 element
    data += struct.pack("<i", 300)  # vertex index
    data += struct.pack("<ffff", 0.2, 0.3, 0.4, 0.5)  # uv offset

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.morph_type == MorphType.UV2
    assert len(morph.data) == 1
    assert morph.data[0].vertex_index == 300
    assert morph.data[0].offset.x == pytest.approx(0.2)
    assert morph.data[0].offset.y == pytest.approx(0.3)


def test_read_morphs_data_uv3():
    """Test reading a UV3 morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Panel type
    data += struct.pack("<b", 4)

    # Morph type - UV3
    data += struct.pack("<b", MorphType.UV3.value)

    # Element count
    data += struct.pack("<i", 1)

    # UV3 element
    data += struct.pack("<i", 400)  # vertex index
    data += struct.pack("<ffff", 0.3, 0.4, 0.5, 0.6)  # uv offset

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.morph_type == MorphType.UV3
    assert len(morph.data) == 1
    assert morph.data[0].vertex_index == 400
    assert morph.data[0].offset.z == pytest.approx(0.5)
    assert morph.data[0].offset.w == pytest.approx(0.6)


def test_read_morphs_data_uv4():
    """Test reading a UV4 morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Panel type
    data += struct.pack("<b", 4)

    # Morph type - UV4
    data += struct.pack("<b", MorphType.UV4.value)

    # Element count
    data += struct.pack("<i", 1)

    # UV4 element
    data += struct.pack("<i", 500)  # vertex index
    data += struct.pack("<ffff", 0.4, 0.5, 0.6, 0.7)  # uv offset

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.morph_type == MorphType.UV4
    assert len(morph.data) == 1
    assert morph.data[0].vertex_index == 500
    assert morph.data[0].offset.x == pytest.approx(0.4)
    assert morph.data[0].offset.w == pytest.approx(0.7)


def test_read_morphs_data_material():
    """Test reading a material morph"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 1)  # morph_count = 1

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Panel type
    data += struct.pack("<b", 1)

    # Morph type - MATERIAL (8)
    data += struct.pack("<b", 8)

    # Element count
    data += struct.pack("<i", 1)

    # Material element
    data += struct.pack("<i", 3)  # material index
    data += struct.pack("<b", 1)  # offset operation
    data += struct.pack("<ffff", 1.0, 0.8, 0.6, 1.0)  # diffuse
    data += struct.pack("<fff", 0.5, 0.5, 0.5)  # specular
    data += struct.pack("<f", 10.0)  # specular power
    data += struct.pack("<fff", 0.2, 0.2, 0.2)  # ambient
    data += struct.pack("<ffff", 0.0, 0.0, 0.0, 1.0)  # edge color
    data += struct.pack("<f", 1.5)  # edge size
    data += struct.pack("<ffff", 1.0, 1.0, 1.0, 1.0)  # texture tint
    data += struct.pack("<ffff", 1.0, 1.0, 1.0, 1.0)  # sphere texture tint
    data += struct.pack("<ffff", 1.0, 1.0, 1.0, 1.0)  # toon texture tint

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 1
    morph = morphs[0]
    assert morph.morph_type == MorphType.MATERIAL
    assert len(morph.data) == 1
    mat = morph.data[0]
    assert mat.material_index == 3
    assert mat.offset_operation == 1
    assert mat.diffuse.x == pytest.approx(1.0)
    assert mat.diffuse.y == pytest.approx(0.8)
    assert mat.specular.x == pytest.approx(0.5)
    assert mat.specular_power == pytest.approx(10.0)
    assert mat.ambient.x == pytest.approx(0.2)
    assert mat.edge_size == pytest.approx(1.5)


def test_read_morphs_data_multiple():
    """Test reading multiple morphs of different types"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<I", 3)  # morph_count = 3

    # Morph 1 - Group
    data += struct.pack("<I", 0)  # name local
    data += struct.pack("<I", 0)  # name universal
    data += struct.pack("<b", 1)  # panel type
    data += struct.pack("<b", MorphType.GROUP.value)
    data += struct.pack("<i", 1)  # element count
    data += struct.pack("<i", 0)  # morph index
    data += struct.pack("<f", 1.0)  # weight

    # Morph 2 - Vertex
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)
    data += struct.pack("<b", 2)
    data += struct.pack("<b", MorphType.VERTEX.value)
    data += struct.pack("<i", 1)
    data += struct.pack("<i", 5)  # vertex index
    data += struct.pack("<fff", 0.1, 0.2, 0.3)  # offset

    # Morph 3 - Bone
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)
    data += struct.pack("<b", 3)
    data += struct.pack("<b", MorphType.BONE.value)
    data += struct.pack("<i", 1)
    data += struct.pack("<i", 10)  # bone index
    data += struct.pack("<fff", 1.0, 2.0, 3.0)  # position offset
    data += struct.pack("<ffff", 0.0, 0.0, 0.0, 1.0)  # rotation offset

    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)

    assert len(morphs) == 3
    assert morphs[0].morph_type == MorphType.GROUP
    assert morphs[1].morph_type == MorphType.VERTEX
    assert morphs[2].morph_type == MorphType.BONE


def test_read_morphs_data_empty():
    """Test reading zero morphs"""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)
    data = struct.pack("<I", 0)  # morph_count = 0
    pmx_reader.reader = BinaryReader(data)
    morphs = read_morphs_data(pmx_reader)
    assert len(morphs) == 0


# --- Display Frame Data Tests ---


def test_read_display_frames_data_empty():
    """Test reading display frames when none are present."""
    # display_frame_count = 0
    data = struct.pack("<i", 0)
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(data), config)
    frames = read_display_frames_data(pmx_reader)
    assert frames == []


def test_read_display_frames_data_single_bone():
    """Test reading a display frame with a single bone element."""
    # display_frame_count = 1
    # names: "Frame", "FrameEN"
    # special_flag = 0
    # frame_element_count = 1
    # element_type = BONE (0), element_index = 5
    local_name = "Frame"
    universal_name = "FrameEN"
    special_flag = 0
    frame_element_count = 1
    element_type = 0  # BONE
    element_index = 5
    data = b""
    data += struct.pack("<i", 1)  # display_frame_count
    # local_name
    local_name_bytes = local_name.encode("utf-8")
    data += struct.pack("<I", len(local_name_bytes)) + local_name_bytes
    # universal_name
    universal_name_bytes = universal_name.encode("utf-8")
    data += struct.pack("<I", len(universal_name_bytes)) + universal_name_bytes
    data += struct.pack("<b", special_flag)
    data += struct.pack("<i", frame_element_count)
    data += struct.pack("<b", element_type)  # FrameType.BONE
    data += struct.pack("<i", element_index)
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(data), config)
    frames = read_display_frames_data(pmx_reader)
    assert len(frames) == 1
    frame = frames[0]
    assert frame.name_local == local_name
    assert frame.name_universal == universal_name
    assert frame.special_flag == special_flag
    assert frame.frame_elements == [
        FrameData(
            frame_type=FrameType.BONE, data=BoneFrameElement(bone_index=element_index)
        )
    ]


def test_read_display_frames_data_single_morph():
    """Test reading a display frame with a single morph element."""
    # display_frame_count = 1
    # names: "MorphFrame", "MorphFrameEN"
    # special_flag = 1
    # frame_element_count = 1
    # element_type = MORPH (1), element_index = 3
    local_name = "MorphFrame"
    universal_name = "MorphFrameEN"
    special_flag = 1
    frame_element_count = 1
    element_type = 1  # MORPH
    element_index = 3
    data = b""
    data += struct.pack("<i", 1)  # display_frame_count
    # local_name
    local_name_bytes = local_name.encode("utf-8")
    data += struct.pack("<I", len(local_name_bytes)) + local_name_bytes
    # universal_name
    universal_name_bytes = universal_name.encode("utf-8")
    data += struct.pack("<I", len(universal_name_bytes)) + universal_name_bytes
    data += struct.pack("<b", special_flag)
    data += struct.pack("<i", frame_element_count)
    data += struct.pack("<b", element_type)  # FrameType.MORPH
    data += struct.pack("<i", element_index)
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(data), config)
    frames = read_display_frames_data(pmx_reader)
    assert len(frames) == 1
    frame = frames[0]
    assert frame.name_local == local_name
    assert frame.name_universal == universal_name
    assert frame.special_flag == special_flag
    assert frame.frame_elements == [
        FrameData(
            frame_type=FrameType.MORPH,
            data=MorphFrameElement(morph_index=element_index),
        )
    ]


def test_read_display_frames_data_multiple_elements():
    """Test reading a display frame with multiple elements (bone and morph)."""
    local_name = "Combo"
    universal_name = "ComboEN"
    special_flag = 0
    frame_element_count = 2
    # element 0: BONE, 7
    # element 1: MORPH, 2
    data = b""
    data += struct.pack("<i", 1)  # display_frame_count
    # local_name
    local_name_bytes = local_name.encode("utf-8")
    data += struct.pack("<I", len(local_name_bytes)) + local_name_bytes
    # universal_name
    universal_name_bytes = universal_name.encode("utf-8")
    data += struct.pack("<I", len(universal_name_bytes)) + universal_name_bytes
    data += struct.pack("<b", special_flag)
    data += struct.pack("<i", frame_element_count)
    data += struct.pack("<b", 0)  # FrameType.BONE
    data += struct.pack("<i", 7)
    data += struct.pack("<b", 1)  # FrameType.MORPH
    data += struct.pack("<i", 2)
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(data), config)
    frames = read_display_frames_data(pmx_reader)
    assert len(frames) == 1
    frame = frames[0]
    assert frame.name_local == local_name
    assert frame.name_universal == universal_name
    assert frame.special_flag == special_flag
    assert frame.frame_elements == [
        FrameData(frame_type=FrameType.BONE, data=BoneFrameElement(bone_index=7)),
        FrameData(frame_type=FrameType.MORPH, data=MorphFrameElement(morph_index=2)),
    ]


# --- Rigid Body Data Tests ---
def test_read_rigid_body_data_single():
    """Test reading a single rigid body with basic properties."""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<i", 1)  # rigid_body_count = 1

    # Rigid body name local (UTF-8)
    name_local = "RigidBody01"
    data += struct.pack("<I", len(name_local.encode("utf-8")))
    data += name_local.encode("utf-8")

    # Rigid body name universal (UTF-8)
    name_universal = "RigidBody01EN"
    data += struct.pack("<I", len(name_universal.encode("utf-8")))
    data += name_universal.encode("utf-8")

    # Bone index
    data += struct.pack("<i", 5)

    # Group
    data += struct.pack("<b", 1)

    # Non-collision group
    data += struct.pack("<H", 0)

    # Shape type (0 = Sphere)
    data += struct.pack("<b", 0)

    # Shape size
    data += struct.pack("<fff", 1.0, 1.0, 1.0)

    # Shape position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)

    # Shape rotation
    data += struct.pack("<fff", 0.0, 0.0, 0.0)

    # Mass
    data += struct.pack("<f", 1.0)

    # Linear damping
    data += struct.pack("<f", 0.04)

    # Angular damping
    data += struct.pack("<f", 0.1)

    # Restitution
    data += struct.pack("<f", 0.0)

    # Friction
    data += struct.pack("<f", 0.5)

    # Physics mode (0 = Static)
    data += struct.pack("<b", 0)

    pmx_reader.reader = BinaryReader(data)
    rigid_bodies = read_rigid_body_data(pmx_reader)

    assert len(rigid_bodies) == 1
    rb = rigid_bodies[0]
    assert rb.name_local == "RigidBody01"
    assert rb.name_universal == "RigidBody01EN"
    assert rb.related_bone_index == 5
    assert rb.group_id == 1
    assert rb.non_collision_group == 0
    assert rb.shape == ShapeType.SPHERE
    assert rb.shape_size == (1.0, 1.0, 1.0)
    assert rb.shape_position == (0.0, 0.0, 0.0)
    assert rb.shape_rotation == (0.0, 0.0, 0.0)
    assert rb.mass == 1.0
    assert rb.move_attenuation == pytest.approx(0.04)
    assert rb.rotation_damping == pytest.approx(0.1)
    assert rb.repulsion == 0.0
    assert rb.friction_force == pytest.approx(0.5)
    assert rb.physics_mode == PhysicsMode.FOLLOW_BONE


def test_read_rigid_body_data_box_shape():
    """Test reading a rigid body with box shape."""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<i", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Bone index
    data += struct.pack("<i", 3)

    # Group
    data += struct.pack("<b", 0)

    # Non-collision group
    data += struct.pack("<H", 1)

    # Shape type (1 = Box)
    data += struct.pack("<b", 1)

    # Shape size (box dimensions)
    data += struct.pack("<fff", 2.0, 3.0, 4.0)

    # Shape position
    data += struct.pack("<fff", 1.0, 2.0, 3.0)

    # Shape rotation
    data += struct.pack("<fff", 0.1, 0.2, 0.3)

    # Mass
    data += struct.pack("<f", 5.0)

    # Linear damping
    data += struct.pack("<f", 0.01)

    # Angular damping
    data += struct.pack("<f", 0.05)

    # Restitution
    data += struct.pack("<f", 0.5)

    # Friction
    data += struct.pack("<f", 0.8)

    # Physics mode (1 = Dynamic)
    data += struct.pack("<b", 1)

    pmx_reader.reader = BinaryReader(data)
    rigid_bodies = read_rigid_body_data(pmx_reader)

    assert len(rigid_bodies) == 1
    rb = rigid_bodies[0]
    assert rb.shape == ShapeType.BOX
    assert rb.shape_size == (2.0, 3.0, 4.0)
    assert rb.shape_position == (1.0, 2.0, 3.0)
    assert rb.shape_rotation.x == pytest.approx(0.1)
    assert rb.shape_rotation.y == pytest.approx(0.2)
    assert rb.shape_rotation.z == pytest.approx(0.3)
    assert rb.mass == 5.0
    assert rb.move_attenuation == pytest.approx(0.01)
    assert rb.rotation_damping == pytest.approx(0.05)
    assert rb.repulsion == pytest.approx(0.5)
    assert rb.friction_force == pytest.approx(0.8)
    assert rb.physics_mode == PhysicsMode.PHYSICS


def test_read_rigid_body_data_capsule_shape():
    """Test reading a rigid body with capsule shape."""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<i", 1)

    # Names
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)

    # Bone index
    data += struct.pack("<i", 2)

    # Group
    data += struct.pack("<b", 2)

    # Non-collision group
    data += struct.pack("<H", 0xFFFF)

    # Shape type (2 = Capsule)
    data += struct.pack("<b", 2)

    # Shape size (radius and height for capsule)
    data += struct.pack("<fff", 0.5, 2.0, 0.5)

    # Shape position
    data += struct.pack("<fff", 0.5, 1.0, 0.5)

    # Shape rotation
    data += struct.pack("<fff", 0.0, 1.57, 0.0)

    # Mass
    data += struct.pack("<f", 2.5)

    # Linear damping
    data += struct.pack("<f", 0.02)

    # Angular damping
    data += struct.pack("<f", 0.08)

    # Restitution
    data += struct.pack("<f", 0.2)

    # Friction
    data += struct.pack("<f", 0.6)

    # Physics mode (2 = Physics + Bone)
    data += struct.pack("<b", 2)

    pmx_reader.reader = BinaryReader(data)
    rigid_bodies = read_rigid_body_data(pmx_reader)

    assert len(rigid_bodies) == 1
    rb = rigid_bodies[0]
    assert rb.shape == ShapeType.CAPSULE
    assert rb.shape_size == (0.5, 2.0, 0.5)
    # PMX stores non-collision group as unsigned, but Python struct '<H' gives 65535, while readInt16 gives -1
    # Accept both representations
    assert rb.non_collision_group in (0xFFFF, -1)


def test_read_rigid_body_data_multiple():
    """Test reading multiple rigid bodies with different properties."""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<i", 3)  # rigid_body_count = 3

    # Rigid body 1
    data += struct.pack("<I", 0)  # empty name local
    data += struct.pack("<I", 0)  # empty name universal
    data += struct.pack("<i", 1)  # bone index
    data += struct.pack("<b", 0)  # group
    data += struct.pack("<H", 0)  # non-collision group
    data += struct.pack("<b", 0)  # shape type (sphere)
    data += struct.pack("<fff", 1.0, 1.0, 1.0)  # shape size
    data += struct.pack("<fff", 0.0, 0.0, 0.0)  # position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)  # rotation
    data += struct.pack("<f", 1.0)  # mass
    data += struct.pack("<f", 0.04)  # linear damping
    data += struct.pack("<f", 0.1)  # angular damping
    data += struct.pack("<f", 0.0)  # restitution
    data += struct.pack("<f", 0.5)  # friction
    data += struct.pack("<b", 0)  # physics mode

    # Rigid body 2
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)
    data += struct.pack("<i", 2)  # bone index
    data += struct.pack("<b", 1)  # group
    data += struct.pack("<H", 0)  # non-collision group
    data += struct.pack("<b", 1)  # shape type (box)
    data += struct.pack("<fff", 2.0, 2.0, 2.0)  # shape size
    data += struct.pack("<fff", 1.0, 1.0, 1.0)  # position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)  # rotation
    data += struct.pack("<f", 3.0)  # mass
    data += struct.pack("<f", 0.05)  # linear damping
    data += struct.pack("<f", 0.15)  # angular damping
    data += struct.pack("<f", 0.3)  # restitution
    data += struct.pack("<f", 0.7)  # friction
    data += struct.pack("<b", 1)  # physics mode

    # Rigid body 3
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)
    data += struct.pack("<i", 3)  # bone index
    data += struct.pack("<b", 2)  # group
    data += struct.pack("<H", 0xFFFF)  # non-collision group
    data += struct.pack("<b", 2)  # shape type (capsule)
    data += struct.pack("<fff", 0.5, 1.5, 0.5)  # shape size
    data += struct.pack("<fff", 2.0, 2.0, 2.0)  # position
    data += struct.pack("<fff", 0.0, 0.0, 0.0)  # rotation
    data += struct.pack("<f", 2.0)  # mass
    data += struct.pack("<f", 0.03)  # linear damping
    data += struct.pack("<f", 0.12)  # angular damping
    data += struct.pack("<f", 0.1)  # restitution
    data += struct.pack("<f", 0.6)  # friction
    data += struct.pack("<b", 2)  # physics mode

    pmx_reader.reader = BinaryReader(data)
    rigid_bodies = read_rigid_body_data(pmx_reader)

    assert len(rigid_bodies) == 3

    # Verify first rigid body
    rb1 = rigid_bodies[0]
    assert rb1.related_bone_index == 1
    assert rb1.group_id == 0
    assert rb1.non_collision_group == 0
    assert rb1.shape == ShapeType.SPHERE

    # Verify second rigid body
    rb2 = rigid_bodies[1]
    assert rb2.related_bone_index == 2
    assert rb2.group_id == 1
    assert rb2.shape == ShapeType.BOX
    assert rb2.mass == 3.0

    # Verify third rigid body
    rb3 = rigid_bodies[2]
    assert rb3.related_bone_index == 3
    assert rb3.group_id == 2
    assert rb3.non_collision_group == -1  # 0xFFFF stored as signed short
    assert rb3.shape == ShapeType.CAPSULE


def test_read_rigid_body_data_no_collision_groups():
    """Test reading a rigid body with various non-collision group settings."""
    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    pmx_reader = PMXReader(BinaryReader(b""), config)

    data = struct.pack("<i", 2)  # rigid_body_count = 2

    # Rigid body 1 - no collision with any group
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)
    data += struct.pack("<i", 0)
    data += struct.pack("<b", 0)
    data += struct.pack("<H", 0xFFFF)  # all bits set
    data += struct.pack("<b", 0)
    data += struct.pack("<fff", 1.0, 1.0, 1.0)
    data += struct.pack("<fff", 0.0, 0.0, 0.0)
    data += struct.pack("<fff", 0.0, 0.0, 0.0)
    data += struct.pack("<f", 1.0)
    data += struct.pack("<f", 0.04)
    data += struct.pack("<f", 0.1)
    data += struct.pack("<f", 0.0)
    data += struct.pack("<f", 0.5)
    data += struct.pack("<b", 0)

    # Rigid body 2 - collision with specific groups
    data += struct.pack("<I", 0)
    data += struct.pack("<I", 0)
    data += struct.pack("<i", 1)
    data += struct.pack("<b", 1)
    data += struct.pack("<H", 0b0000000000001010)  # bits 1 and 3 set
    data += struct.pack("<b", 0)
    data += struct.pack("<fff", 1.0, 1.0, 1.0)
    data += struct.pack("<fff", 0.0, 0.0, 0.0)
    data += struct.pack("<fff", 0.0, 0.0, 0.0)
    data += struct.pack("<f", 1.0)
    data += struct.pack("<f", 0.04)
    data += struct.pack("<f", 0.1)
    data += struct.pack("<f", 0.0)
    data += struct.pack("<f", 0.5)
    data += struct.pack("<b", 0)

    pmx_reader.reader = BinaryReader(data)
    rigid_bodies = read_rigid_body_data(pmx_reader)

    assert len(rigid_bodies) == 2
    assert rigid_bodies[0].non_collision_group == -1  # 0xFFFF stored as signed short
    assert rigid_bodies[1].non_collision_group == 0b0000000000001010


# --- Joint Data Tests ---
def _pack_index(value, size):
    if size == 1:
        return struct.pack("<b", value)
    elif size == 2:
        return struct.pack("<h", value)
    elif size == 4:
        return struct.pack("<i", value)
    else:
        raise ValueError(f"Unsupported index size: {size}")


def test_read_joint_data_single():
    joint_count = 1
    name_local = "JointA"
    name_universal = "JointA_U"
    joint_type = JointType.SPRING_6DOF.value
    rigid_body_index_a = 2
    rigid_body_index_b = 3
    vec_zero = (0.0, 0.0, 0.0)

    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]  # rigid body index size = 4 bytes
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    index_size = config_globals[7]

    data = struct.pack("<I", joint_count)
    # name_local
    data += struct.pack("<I", len(name_local.encode("utf-8"))) + name_local.encode(
        "utf-8"
    )
    # name_universal
    data += struct.pack(
        "<I", len(name_universal.encode("utf-8"))
    ) + name_universal.encode("utf-8")
    # joint_type
    data += struct.pack("<b", joint_type)
    # rigid_body_index_a
    data += _pack_index(rigid_body_index_a, index_size)
    # rigid_body_index_b
    data += _pack_index(rigid_body_index_b, index_size)
    # position, rotation, movement_limit_min, movement_limit_max, rotation_limit_min, rotation_limit_max, spring_position_factor, spring_rotation_factor
    for _ in range(8):
        data += struct.pack("<fff", *vec_zero)

    reader = PMXReader(BinaryReader(data), config)
    joints = read_joint_data(reader)
    assert len(joints) == 1
    joint = joints[0]
    assert joint.name_local == name_local
    assert joint.name_universal == name_universal
    assert joint.type == JointType.SPRING_6DOF
    assert joint.rigid_body_index_a == rigid_body_index_a
    assert joint.rigid_body_index_b == rigid_body_index_b
    assert joint.position == Vec3(0.0, 0.0, 0.0)
    assert joint.rotation == Vec3(0.0, 0.0, 0.0)
    assert joint.position_min == Vec3(0.0, 0.0, 0.0)
    assert joint.position_max == Vec3(0.0, 0.0, 0.0)
    assert joint.rotation_min == Vec3(0.0, 0.0, 0.0)
    assert joint.rotation_max == Vec3(0.0, 0.0, 0.0)
    assert joint.position_spring_constant == Vec3(0.0, 0.0, 0.0)
    assert joint.rotation_spring_constant == Vec3(0.0, 0.0, 0.0)


def test_read_joint_data_multiple():
    joint_count = 2
    names = ["JointA", "JointB"]
    joint_types = [JointType.SPRING_6DOF.value, JointType.HINGE.value]
    rigid_body_indices = [(2, 3), (4, 5)]
    vec_zero = (0.0, 0.0, 0.0)

    config_globals = [1, 0, 4, 4, 4, 4, 4, 4]  # rigid body index size = 4 bytes
    header = type("H", (), {"globals": config_globals})()
    config = PMXConfig(header)
    index_size = config_globals[7]

    data = struct.pack("<I", joint_count)
    for i in range(joint_count):
        # name_local
        data += struct.pack("<I", len(names[i].encode("utf-8"))) + names[i].encode(
            "utf-8"
        )
        # name_universal
        universal = names[i] + "_U"
        data += struct.pack("<I", len(universal.encode("utf-8"))) + universal.encode(
            "utf-8"
        )
        # joint_type
        data += struct.pack("<b", joint_types[i])
        # rigid_body_index_a
        data += _pack_index(rigid_body_indices[i][0], index_size)
        # rigid_body_index_b
        data += _pack_index(rigid_body_indices[i][1], index_size)
        # 8x Vec3
        for _ in range(8):
            data += struct.pack("<fff", *vec_zero)

    reader = PMXReader(BinaryReader(data), config)
    joints = read_joint_data(reader)
    assert len(joints) == 2
    for i, joint in enumerate(joints):
        assert joint.name_local == names[i]
        assert joint.name_universal == names[i] + "_U"
        assert joint.type.value == joint_types[i]
        assert joint.rigid_body_index_a == rigid_body_indices[i][0]
        assert joint.rigid_body_index_b == rigid_body_indices[i][1]
        assert joint.position == Vec3(0.0, 0.0, 0.0)
        assert joint.rotation == Vec3(0.0, 0.0, 0.0)
        assert joint.position_min == Vec3(0.0, 0.0, 0.0)
        assert joint.position_max == Vec3(0.0, 0.0, 0.0)
        assert joint.rotation_min == Vec3(0.0, 0.0, 0.0)
        assert joint.rotation_max == Vec3(0.0, 0.0, 0.0)
        assert joint.position_spring_constant == Vec3(0.0, 0.0, 0.0)
        assert joint.rotation_spring_constant == Vec3(0.0, 0.0, 0.0)
