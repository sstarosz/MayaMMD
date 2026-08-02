import struct

import pytest

from mmd.core.binary_reader import BinaryReader
from mmd.core.data_types import Vec2, Vec3, Vec4, WeightType
from mmd.core.pmx_reader import PMXConfig, PMXReader


class DummyHeader:
    def __init__(self, globals_):
        self.globals = globals_


def make_reader(data: bytes, header_globals=None):
    if header_globals is None:
        header_globals = [1, 0, 1, 2, 2, 1, 1, 1]  # reasonable defaults
    header = DummyHeader(header_globals)
    config = PMXConfig(header)
    return PMXReader(BinaryReader(data), config)


def test_readInt8():
    r = make_reader(struct.pack("<b", -5))
    assert r.readInt8() == -5


def test_readInt16():
    r = make_reader(struct.pack("<h", -1234))
    assert r.readInt16() == -1234


def test_readUInt16():
    r = make_reader(struct.pack("<H", 65535))
    assert r.readUInt16() == 65535


def test_readInt32():
    r = make_reader(struct.pack("<i", -123456))
    assert r.readInt32() == -123456


def test_readUInt32():
    r = make_reader(struct.pack("<I", 123456789))
    assert r.readUInt32() == 123456789


def test_readFloat():
    r = make_reader(struct.pack("<f", 1.5))
    assert abs(r.readFloat() - 1.5) < 1e-6


def test_readVec2():
    r = make_reader(struct.pack("<ff", 1.0, 2.0))
    v = r.readVec2()
    assert isinstance(v, Vec2)
    assert v.x == 1.0 and v.y == 2.0


def test_readVec3():
    r = make_reader(struct.pack("<fff", 1.0, 2.0, 3.0))
    v = r.readVec3()
    assert isinstance(v, Vec3)
    assert v.x == 1.0 and v.y == 2.0 and v.z == 3.0


def test_readVec4():
    r = make_reader(struct.pack("<ffff", 1.0, 2.0, 3.0, 4.0))
    v = r.readVec4()
    assert isinstance(v, Vec4)
    assert v.x == 1.0 and v.y == 2.0 and v.z == 3.0 and v.w == 4.0


def test_readIndex_signed():
    # size=2, signed
    r = make_reader(struct.pack("<h", -42), header_globals=[1, 0, 1, 2, 2, 2, 1, 1])
    assert r.readIndex(2, signed=True) == -42


def test_readIndex_unsigned():
    # size=2, unsigned
    r = make_reader(struct.pack("<H", 42), header_globals=[1, 0, 2, 2, 2, 2, 1, 1])
    assert r.readIndex(2, signed=False) == 42


def test_readIndex_invalid_size():
    r = make_reader(b"")
    with pytest.raises(RuntimeError):
        r.readIndex(3, signed=True)
    with pytest.raises(RuntimeError):
        r.readIndex(3, signed=False)


def test_readText_utf8():
    text = "hello"
    encoded = text.encode("utf-8")
    data = struct.pack("<I", len(encoded)) + encoded
    r = make_reader(data)
    assert r.readText() == text


def test_readWeightType():
    r = make_reader(struct.pack("<b", WeightType.BDEF1.value))
    assert r.readWeightType() == WeightType.BDEF1


def test_isAtEnd():
    r = make_reader(b"\x01")
    assert not r.isAtEnd()
    r.readInt8()
    assert r.isAtEnd()


# --- Additional tests for index readers ---


@pytest.mark.parametrize(
    "size, value, expected, signed",
    [
        (1, 0x7F, 127, True),
        (1, -0x7F, -127, True),
        (2, 0x7FFF, 32767, True),
        (2, -0x7FFF, -32767, True),
        (4, 0x7FFFFFFF, 2147483647, True),
        (4, -0x7FFFFFFF, -2147483647, True),
        (1, 0xFF, 255, False),
        (2, 0xFFFF, 65535, False),
        (4, 0xFFFFFFFF, 4294967295, False),
    ],
)
def test_readIndex_various(size, value, expected, signed):
    # Pack value according to size and signedness
    if size == 1:
        fmt = "<b" if signed else "<B"
    elif size == 2:
        fmt = "<h" if signed else "<H"
    elif size == 4:
        fmt = "<i" if signed else "<I"
    else:
        pytest.skip("Unsupported size for test")
    data = struct.pack(fmt, value)
    header_globals = [1, 0, size, size, size, size, size, size]
    r = make_reader(data, header_globals=header_globals)
    assert r.readIndex(size, signed=signed) == expected


@pytest.mark.parametrize(
    "size, value, expected",
    [
        (1, 42, 42),
        (2, 1234, 1234),
        (4, 56789, 56789),
    ],
)
def test_readTextureIndex(size, value, expected):
    # textureIndexSize is at index 3
    header_globals = [1, 0, 1, size, 1, 1, 1, 1]
    if size == 1:
        data = struct.pack("<b", value)
    elif size == 2:
        data = struct.pack("<h", value)
    elif size == 4:
        data = struct.pack("<i", value)
    else:
        pytest.skip("Unsupported size")
    r = make_reader(data, header_globals=header_globals)
    assert r.readTextureIndex() == expected


@pytest.mark.parametrize(
    "size, value, expected",
    [
        (1, -5, -5),
        (2, -123, -123),
        (4, -45678, -45678),
    ],
)
def test_readMaterialIndex(size, value, expected):
    # materialIndexSize is at index 4
    header_globals = [1, 0, 1, 1, size, 1, 1, 1]
    if size == 1:
        data = struct.pack("<b", value)
    elif size == 2:
        data = struct.pack("<h", value)
    elif size == 4:
        data = struct.pack("<i", value)
    else:
        pytest.skip("Unsupported size")
    r = make_reader(data, header_globals=header_globals)
    assert r.readMaterialIndex() == expected


@pytest.mark.parametrize(
    "size, value, expected",
    [
        (1, 7, 7),
        (2, 321, 321),
        (4, 7654321, 7654321),
    ],
)
def test_readMorphIndex(size, value, expected):
    # morphIndexSize is at index 6
    header_globals = [1, 0, 1, 1, 1, 1, size, 1]
    if size == 1:
        data = struct.pack("<b", value)
    elif size == 2:
        data = struct.pack("<h", value)
    elif size == 4:
        data = struct.pack("<i", value)
    else:
        pytest.skip("Unsupported size")
    r = make_reader(data, header_globals=header_globals)
    assert r.readMorphIndex() == expected


@pytest.mark.parametrize(
    "size, value, expected",
    [
        (1, -1, -1),
        (2, -222, -222),
        (4, -333333, -333333),
    ],
)
def test_readRigidBodyIndex(size, value, expected):
    # rigidBodyIndexSize is at index 7
    header_globals = [1, 0, 1, 1, 1, 1, 1, size]
    if size == 1:
        data = struct.pack("<b", value)
    elif size == 2:
        data = struct.pack("<h", value)
    elif size == 4:
        data = struct.pack("<i", value)
    else:
        pytest.skip("Unsupported size")
    r = make_reader(data, header_globals=header_globals)
    assert r.readRigidBodyIndex() == expected
