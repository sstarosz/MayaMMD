# pylint: disable=missing-function-docstring,missing-module-docstring
import struct
import tempfile

import pytest

from mmd.core.binary_reader import BinaryReader
from mmd.core.data_types import (
    VMDVersion,
)
from mmd.core.vmd_importer import (
    VMDHeaderParseError,
    VMDParseError,
    determine_vmd_version,
    parse_vmd_file,
    read_bone_keyframe,
    read_bone_keyframes,
    read_camera_keyframes,
    read_header,
    read_light_keyframes,
    read_model_name,
    read_morph_keyframes,
    read_property_keyframes,
    read_self_shadow_keyframes,
)

# ---------------------------------------------------------------------------#
#  read_header
# ---------------------------------------------------------------------------#


def test_read_header_vmd1():
    """Header string 'Vocaloid Motion Data file' padded to 30 bytes with nulls."""
    header_str = b"Vocaloid Motion Data file\x00"
    padded = header_str.ljust(30, b"\x00")
    reader = BinaryReader(padded)
    result = read_header(reader)
    assert result == "Vocaloid Motion Data file"
    assert reader.get_offset() == 30


def test_read_header_vmd2():
    header_str = b"Vocaloid Motion Data 0002\x00"
    padded = header_str.ljust(30, b"\x00")
    reader = BinaryReader(padded)
    result = read_header(reader)
    assert result == "Vocaloid Motion Data 0002"
    assert reader.get_offset() == 30


def test_read_header_no_null():
    """If no null terminator is found, the full 30 bytes are consumed."""
    data = b"A" * 30
    reader = BinaryReader(data)
    result = read_header(reader)
    assert result == "A" * 30
    assert reader.get_offset() == 30


# ---------------------------------------------------------------------------#
#  determine_vmd_version
# ---------------------------------------------------------------------------#


def test_determine_vmd_version_v1():
    assert determine_vmd_version("Vocaloid Motion Data file") == VMDVersion.VMD_1_0


def test_determine_vmd_version_v2():
    assert determine_vmd_version("Vocaloid Motion Data 0002") == VMDVersion.VMD_2_0


def test_determine_vmd_version_unknown():
    with pytest.raises(VMDHeaderParseError, match="Unknown VMD file version"):
        determine_vmd_version("Some other header")


# ---------------------------------------------------------------------------#
#  read_model_name
# ---------------------------------------------------------------------------#


def test_read_model_name_v1():
    """VMD 1.0: model name stored in 10 bytes, Shift-JIS encoded."""
    model = "TestModel"
    encoded = model.encode("shift_jis")
    data = encoded.ljust(10, b"\x00")
    reader = BinaryReader(data)
    result = read_model_name(reader, VMDVersion.VMD_1_0)
    assert result == model


def test_read_model_name_v2():
    """VMD 2.0: model name stored in 20 bytes, Shift-JIS encoded."""
    model = "TestModelLongName"
    encoded = model.encode("shift_jis")
    data = encoded.ljust(20, b"\x00")
    reader = BinaryReader(data)
    result = read_model_name(reader, VMDVersion.VMD_2_0)
    assert result == model


def test_read_model_name_leading_nulls():
    """Leading nulls should be stripped before decoding."""
    data = b"\x00\x00" + "ABC".encode("shift_jis")
    data = data.ljust(20, b"\x00")
    reader = BinaryReader(data)
    result = read_model_name(reader, VMDVersion.VMD_2_0)
    assert result == "ABC"


# ---------------------------------------------------------------------------#
#  read_bone_keyframe
# ---------------------------------------------------------------------------#


def _build_bone_keyframe_bytes(
    bone_name: str = "test_bone",
    frame_number: int = 42,
    pos: tuple[float, float, float] = (1.0, 2.0, 3.0),
    rot: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> bytes:
    name_encoded = bone_name.encode("shift_jis")
    name_padded = name_encoded.ljust(15, b"\x00")[:15]
    data = name_padded
    data += struct.pack("<I", frame_number)
    data += struct.pack("<fff", *pos)
    data += struct.pack("<ffff", *rot)
    data += b"\x00" * 64  # interpolation (64 bytes)
    return data


def test_read_bone_keyframe():
    data = _build_bone_keyframe_bytes()
    reader = BinaryReader(data)
    kf = read_bone_keyframe(reader)
    assert kf.bone_name == "test_bone"
    assert kf.frame_number == 42
    assert kf.position.x == pytest.approx(1.0)
    assert kf.position.y == pytest.approx(2.0)
    assert kf.position.z == pytest.approx(3.0)
    assert kf.rotation.x == pytest.approx(0.0)
    assert kf.rotation.y == pytest.approx(0.0)
    assert kf.rotation.z == pytest.approx(0.0)
    assert kf.rotation.w == pytest.approx(1.0)
    assert len(kf.interpolation) == 64


# ---------------------------------------------------------------------------#
#  read_bone_keyframes
# ---------------------------------------------------------------------------#


def test_read_bone_keyframes_empty():
    data = struct.pack("<I", 0)
    reader = BinaryReader(data)
    keyframes = read_bone_keyframes(reader)
    assert keyframes == []


def test_read_bone_keyframes_multiple():
    kf1 = _build_bone_keyframe_bytes("bone_a", 0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    kf2 = _build_bone_keyframe_bytes(
        "bone_b", 10, (0.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    )
    count = struct.pack("<I", 2)
    reader = BinaryReader(count + kf1 + kf2)
    keyframes = read_bone_keyframes(reader)
    assert len(keyframes) == 2
    assert keyframes[0].bone_name == "bone_a"
    assert keyframes[0].frame_number == 0
    assert keyframes[1].bone_name == "bone_b"
    assert keyframes[1].frame_number == 10


# ---------------------------------------------------------------------------#
#  read_morph_keyframes
# ---------------------------------------------------------------------------#


def _build_morph_keyframe_bytes(
    morph_name: str = "blink",
    frame_number: int = 5,
    weight: float = 0.5,
) -> bytes:
    name_encoded = morph_name.encode("shift_jis")
    name_padded = name_encoded.ljust(15, b"\x00")[:15]
    data = name_padded
    data += struct.pack("<I", frame_number)
    data += struct.pack("<f", weight)
    return data


def test_read_morph_keyframes_empty():
    data = struct.pack("<I", 0)
    reader = BinaryReader(data)
    keyframes = read_morph_keyframes(reader)
    assert keyframes == []


def test_read_morph_keyframes_multiple():
    m1 = _build_morph_keyframe_bytes("blink", 0, 1.0)
    m2 = _build_morph_keyframe_bytes("mouth", 30, 0.75)
    count = struct.pack("<I", 2)
    reader = BinaryReader(count + m1 + m2)
    keyframes = read_morph_keyframes(reader)
    assert len(keyframes) == 2
    assert keyframes[0].morph_name == "blink"
    assert keyframes[0].frame_number == 0
    assert keyframes[0].weight == pytest.approx(1.0)
    assert keyframes[1].morph_name == "mouth"
    assert keyframes[1].frame_number == 30
    assert keyframes[1].weight == pytest.approx(0.75)


# ---------------------------------------------------------------------------#
#  read_camera_keyframes
# ---------------------------------------------------------------------------#


def _build_camera_keyframe_bytes(
    frame_number: int = 0,
    distance: float = 50.0,
    pos: tuple[float, float, float] = (0.0, 10.0, -30.0),
    rot: tuple[float, float, float] = (0.0, 0.0, 0.0),
    viewing_angle: int = 30,
    perspective: bool = True,
) -> bytes:
    data = struct.pack("<I", frame_number)
    data += struct.pack("<f", distance)
    data += struct.pack("<fff", *pos)
    data += struct.pack("<fff", *rot)
    data += b"\x00" * 24  # interpolation (24 bytes)
    data += struct.pack("<I", viewing_angle)
    data += struct.pack("<B", 0 if perspective else 1)
    return data


def test_read_camera_keyframes_empty():
    data = struct.pack("<I", 0)
    reader = BinaryReader(data)
    keyframes = read_camera_keyframes(reader)
    assert keyframes == []


def test_read_camera_keyframes_single():
    data = _build_camera_keyframe_bytes()
    count = struct.pack("<I", 1)
    reader = BinaryReader(count + data)
    keyframes = read_camera_keyframes(reader)
    assert len(keyframes) == 1
    cam = keyframes[0]
    assert cam.frame_number == 0
    assert cam.distance == pytest.approx(50.0)
    assert cam.position.x == pytest.approx(0.0)
    assert cam.position.y == pytest.approx(10.0)
    assert cam.position.z == pytest.approx(-30.0)
    assert cam.viewing_angle == 30
    assert cam.perspective is True
    assert len(cam.interpolation) == 24


def test_read_camera_keyframes_orthographic():
    data = _build_camera_keyframe_bytes(perspective=False)
    count = struct.pack("<I", 1)
    reader = BinaryReader(count + data)
    keyframes = read_camera_keyframes(reader)
    assert keyframes[0].perspective is False


# ---------------------------------------------------------------------------#
#  read_light_keyframes
# ---------------------------------------------------------------------------#


def test_read_light_keyframes_empty():
    data = struct.pack("<I", 0)
    reader = BinaryReader(data)
    keyframes = read_light_keyframes(reader)
    assert keyframes == []


def test_read_light_keyframes_single():
    count = struct.pack("<I", 1)
    data = count
    data += struct.pack("<I", 10)  # frame_number
    data += struct.pack("<fff", 0.5, 0.6, 0.7)  # color RGB
    data += struct.pack("<fff", 0.0, -1.0, 0.0)  # direction
    reader = BinaryReader(data)
    keyframes = read_light_keyframes(reader)
    assert len(keyframes) == 1
    light = keyframes[0]
    assert light.frame_number == 10
    assert light.color.x == pytest.approx(0.5)
    assert light.color.y == pytest.approx(0.6)
    assert light.color.z == pytest.approx(0.7)
    assert light.direction.x == pytest.approx(0.0)
    assert light.direction.y == pytest.approx(-1.0)
    assert light.direction.z == pytest.approx(0.0)


# ---------------------------------------------------------------------------#
#  read_self_shadow_keyframes
# ---------------------------------------------------------------------------#


def test_read_self_shadow_keyframes_empty():
    data = struct.pack("<I", 0)
    reader = BinaryReader(data)
    keyframes = read_self_shadow_keyframes(reader)
    assert keyframes == []


def test_read_self_shadow_keyframes_single():
    count = struct.pack("<I", 1)
    data = count
    data += struct.pack("<I", 5)  # frame_number
    data += struct.pack("<B", 2)  # mode
    data += struct.pack("<f", 100.0)  # distance
    reader = BinaryReader(data)
    keyframes = read_self_shadow_keyframes(reader)
    assert len(keyframes) == 1
    shadow = keyframes[0]
    assert shadow.frame_number == 5
    assert shadow.mode == 2
    assert shadow.distance == pytest.approx(100.0)


# ---------------------------------------------------------------------------#
#  read_property_keyframes
# ---------------------------------------------------------------------------#


def _build_property_keyframe_bytes(
    frame_number: int = 0,
    visible: bool = True,
    ik_states: list[tuple[str, bool]] | None = None,
) -> bytes:
    if ik_states is None:
        ik_states = []
    data = struct.pack("<I", frame_number)
    data += struct.pack("<B", 1 if visible else 0)
    data += struct.pack("<I", len(ik_states))
    for ik_name, enabled in ik_states:
        name_encoded = ik_name.encode("shift_jis")
        name_padded = name_encoded.ljust(20, b"\x00")[:20]
        data += name_padded
        data += struct.pack("<B", 1 if enabled else 0)
    return data


def test_read_property_keyframes_empty():
    data = struct.pack("<I", 0)
    reader = BinaryReader(data)
    keyframes = read_property_keyframes(reader)
    assert keyframes == []


def test_read_property_keyframes_with_ik():
    ik_states = [("leg_ik", True), ("arm_ik", False)]
    prop_data = _build_property_keyframe_bytes(
        frame_number=15, visible=True, ik_states=ik_states
    )
    count = struct.pack("<I", 1)
    reader = BinaryReader(count + prop_data)
    keyframes = read_property_keyframes(reader)
    assert len(keyframes) == 1
    prop = keyframes[0]
    assert prop.frame_number == 15
    assert prop.visible is True
    assert len(prop.ik_states) == 2
    assert prop.ik_states[0].ik_name == "leg_ik"
    assert prop.ik_states[0].enabled is True
    assert prop.ik_states[1].ik_name == "arm_ik"
    assert prop.ik_states[1].enabled is False


def test_read_property_keyframes_invisible():
    prop_data = _build_property_keyframe_bytes(frame_number=0, visible=False)
    count = struct.pack("<I", 1)
    reader = BinaryReader(count + prop_data)
    keyframes = read_property_keyframes(reader)
    assert keyframes[0].visible is False


# ---------------------------------------------------------------------------#
#  parse_vmd_file (integration via temp file)
# ---------------------------------------------------------------------------#


def _build_full_vmd_bytes(
    header_text: str = "Vocaloid Motion Data file",
    model_name: str = "TestModel",
    bone_count: int = 0,
    morph_count: int = 0,
    camera_count: int = 0,
    light_count: int = 0,
    shadow_count: int = 0,
    property_count: int = 0,
) -> bytes:
    """Construct a complete (minimal) VMD binary blob."""
    data = bytearray()

    # Header: 30 bytes, null-terminated
    header_encoded = header_text.encode("ascii") + b"\x00"
    header_padded = header_encoded.ljust(30, b"\x00")
    data.extend(header_padded)

    # Model name: 10 bytes for VMD 1.0
    model_encoded = model_name.encode("shift_jis")
    model_padded = model_encoded.ljust(10, b"\x00")
    data.extend(model_padded)

    # Bone keyframes
    data.extend(struct.pack("<I", bone_count))
    for _ in range(bone_count):
        name_enc = b"bone".ljust(15, b"\x00")
        data.extend(name_enc)
        data.extend(struct.pack("<I", 0))  # frame
        data.extend(struct.pack("<fff", 0.0, 0.0, 0.0))  # position
        data.extend(struct.pack("<ffff", 0.0, 0.0, 0.0, 1.0))  # rotation
        data.extend(b"\x00" * 64)  # interpolation

    # Morph keyframes
    data.extend(struct.pack("<I", morph_count))
    for _ in range(morph_count):
        name_enc = b"morph".ljust(15, b"\x00")
        data.extend(name_enc)
        data.extend(struct.pack("<I", 0))  # frame
        data.extend(struct.pack("<f", 1.0))  # weight

    # Camera keyframes
    data.extend(struct.pack("<I", camera_count))
    for _ in range(camera_count):
        data.extend(struct.pack("<I", 0))  # frame
        data.extend(struct.pack("<f", 50.0))  # distance
        data.extend(struct.pack("<fff", 0.0, 0.0, 0.0))  # position
        data.extend(struct.pack("<fff", 0.0, 0.0, 0.0))  # rotation
        data.extend(b"\x00" * 24)  # interpolation
        data.extend(struct.pack("<I", 30))  # viewing_angle
        data.extend(struct.pack("<B", 0))  # perspective

    # Light keyframes (optional)
    data.extend(struct.pack("<I", light_count))
    for _ in range(light_count):
        data.extend(struct.pack("<I", 0))  # frame
        data.extend(struct.pack("<fff", 0.5, 0.6, 0.7))  # color
        data.extend(struct.pack("<fff", 0.0, -1.0, 0.0))  # direction

    # Shadow keyframes (optional)
    data.extend(struct.pack("<I", shadow_count))
    for _ in range(shadow_count):
        data.extend(struct.pack("<I", 0))  # frame
        data.extend(struct.pack("<B", 1))  # mode
        data.extend(struct.pack("<f", 100.0))  # distance

    # Property keyframes (optional)
    data.extend(struct.pack("<I", property_count))
    for _ in range(property_count):
        data.extend(struct.pack("<I", 0))  # frame
        data.extend(struct.pack("<B", 1))  # visible
        data.extend(struct.pack("<I", 0))  # ik count (none)

    return bytes(data)


def test_parse_vmd_file_basic():
    vmd_bytes = _build_full_vmd_bytes(
        header_text="Vocaloid Motion Data file",
        model_name="TestModel",
        bone_count=2,
        morph_count=1,
        camera_count=0,
    )
    with tempfile.NamedTemporaryFile(delete=False, suffix=".vmd") as f:
        f.write(vmd_bytes)
        tmp_path = f.name

    try:
        vmd = parse_vmd_file(tmp_path)
        assert vmd.header == "Vocaloid Motion Data file"
        assert vmd.version == VMDVersion.VMD_1_0
        assert vmd.model_name == "TestModel"
        assert len(vmd.bone_keyframes) == 2
        assert len(vmd.morph_keyframes) == 1
        assert len(vmd.camera_keyframes) == 0
        # Optional sections should be empty
        assert len(vmd.light_keyframes) == 0
        assert len(vmd.shadow_keyframes) == 0
        assert len(vmd.property_keyframes) == 0
    finally:
        import os

        os.unlink(tmp_path)


def test_parse_vmd_file_v2():
    """Test parsing a VMD 2.0 file (20-byte model name)."""
    # Build header (30 bytes)
    header_encoded = b"Vocaloid Motion Data 0002\x00"
    header_padded = header_encoded.ljust(30, b"\x00")

    model_encoded = "LongModelName".encode("shift_jis")
    model_padded = model_encoded.ljust(20, b"\x00")

    data = bytearray(header_padded)
    data.extend(model_padded)
    # No keyframes at all
    data.extend(struct.pack("<I", 0))  # bone count
    data.extend(struct.pack("<I", 0))  # morph count
    data.extend(struct.pack("<I", 0))  # camera count

    with tempfile.NamedTemporaryFile(delete=False, suffix=".vmd") as f:
        f.write(bytes(data))
        tmp_path = f.name

    try:
        vmd = parse_vmd_file(tmp_path)
        assert vmd.version == VMDVersion.VMD_2_0
        assert vmd.model_name == "LongModelName"
    finally:
        import os

        os.unlink(tmp_path)


def test_parse_vmd_file_with_optional_sections():
    """Include all optional sections: light, shadow, property."""
    vmd_bytes = _build_full_vmd_bytes(
        header_text="Vocaloid Motion Data file",
        model_name="Miku",
        bone_count=0,
        morph_count=0,
        camera_count=0,
        light_count=1,
        shadow_count=1,
        property_count=1,
    )
    with tempfile.NamedTemporaryFile(delete=False, suffix=".vmd") as f:
        f.write(vmd_bytes)
        tmp_path = f.name

    try:
        vmd = parse_vmd_file(tmp_path)
        # Optional sections should be populated
        assert len(vmd.light_keyframes) == 1
        assert vmd.light_keyframes[0].frame_number == 0
        assert len(vmd.shadow_keyframes) == 1
        assert vmd.shadow_keyframes[0].mode == 1
        assert len(vmd.property_keyframes) == 1
        assert vmd.property_keyframes[0].frame_number == 0
    finally:
        import os

        os.unlink(tmp_path)


def test_parse_vmd_file_not_found():
    with pytest.raises(VMDParseError, match="Failed to parse VMD file"):
        parse_vmd_file("nonexistent_file.vmd")
