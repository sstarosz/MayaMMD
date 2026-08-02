# pylint: disable=missing-function-docstring, missing-module-docstring

import struct

import pytest

from mmd.core.binary_reader import BinaryReader


def test_read_uint8():
    data = bytes([0x12, 0xFF])
    reader = BinaryReader(data)
    assert reader.read_uint8() == 0x12
    assert reader.read_uint8() == 0xFF


def test_read_int8():
    data = struct.pack("<bb", 0x7F, -0x80)
    reader = BinaryReader(data)
    assert reader.read_int8() == 127
    assert reader.read_int8() == -128


def test_read_uint16():
    data = struct.pack("<H", 0xABCD)
    reader = BinaryReader(data)

    assert reader.read_uint16() == 0xABCD


def test_read_int16():
    data = struct.pack("<h", -12345)
    reader = BinaryReader(data)
    assert reader.read_int16() == -12345


def test_read_uint32():
    data = struct.pack("<I", 0xDEADBEEF)
    reader = BinaryReader(data)
    assert reader.read_uint32() == 0xDEADBEEF


def test_read_int32():
    data = struct.pack("<i", -123456789)
    reader = BinaryReader(data)
    assert reader.read_int32() == -123456789


def test_read_float():
    data = struct.pack("<f", 3.14159)
    reader = BinaryReader(data)
    assert abs(reader.read_float() - 3.14159) < 1e-6


def test_read_text_utf8():
    text = "hello"
    encoded = text.encode("utf-8")
    data = struct.pack("<I", len(encoded)) + encoded
    reader = BinaryReader(data)
    assert reader.read_text(is_utf8=True) == text


def test_read_text_utf16():
    text = "hello"
    encoded = text.encode("utf-16le")
    data = struct.pack("<I", len(encoded)) + encoded
    reader = BinaryReader(data)
    assert reader.read_text(is_utf8=False) == text


def test_read_vec2():
    data = struct.pack("<ff", 1.0, 2.0)
    reader = BinaryReader(data)
    v = reader.read_vec2()
    assert v.x == 1.0 and v.y == 2.0


def test_read_vec3():
    data = struct.pack("<fff", 1.0, 2.0, 3.0)
    reader = BinaryReader(data)
    v = reader.read_vec3()
    assert v.x == 1.0 and v.y == 2.0 and v.z == 3.0


def test_read_vec4():
    data = struct.pack("<ffff", 1.0, 2.0, 3.0, 4.0)
    reader = BinaryReader(data)
    v = reader.read_vec4()
    assert v.x == 1.0 and v.y == 2.0 and v.z == 3.0 and v.w == 4.0


def test_can_read_and_remaining():
    data = bytes([1, 2, 3, 4])
    reader = BinaryReader(data)
    assert reader.can_read(4)
    reader.read_uint8()
    assert reader.remaining() == 3
    reader.read_uint8()
    reader.read_uint8()
    reader.read_uint8()
    assert reader.remaining() == 0
    assert not reader.can_read(1)


def test_read_text_empty():
    data = struct.pack("<I", 0)
    reader = BinaryReader(data)
    assert reader.read_text() == ""


# Exception tests
def test_read_uint8_raises():
    reader = BinaryReader(b"")
    with pytest.raises(RuntimeError, match="Not enough data to read uint8"):
        reader.read_uint8()


def test_read_int8_raises():
    reader = BinaryReader(b"")
    with pytest.raises(RuntimeError, match="Not enough data to read int8"):
        reader.read_int8()


def test_read_uint16_raises():
    reader = BinaryReader(b"\x01")
    with pytest.raises(RuntimeError, match="Not enough data to read uint16"):
        reader.read_uint16()


def test_read_int16_raises():
    reader = BinaryReader(b"\x01")
    with pytest.raises(RuntimeError, match="Not enough data to read int16"):
        reader.read_int16()


def test_read_uint32_raises():
    reader = BinaryReader(b"\x01\x02")
    with pytest.raises(RuntimeError, match="Not enough data to read uint32"):
        reader.read_uint32()


def test_read_int32_raises():
    reader = BinaryReader(b"\x01\x02")
    with pytest.raises(RuntimeError, match="Not enough data to read int32"):
        reader.read_int32()


def test_read_float_raises():
    reader = BinaryReader(b"\x01\x02")
    with pytest.raises(RuntimeError, match="Not enough data to read float"):
        reader.read_float()


def test_read_text_length_raises():
    # Not enough data for length field
    reader = BinaryReader(b"")
    with pytest.raises(RuntimeError, match="Not enough data for length field"):
        reader.read_text()


def test_read_text_content_raises():
    # Enough for length, but not for content
    data = struct.pack("<I", 5) + b"abc"
    reader = BinaryReader(data)
    with pytest.raises(RuntimeError, match="Not enough data for string content"):
        reader.read_text()
