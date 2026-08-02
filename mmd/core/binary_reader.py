# BinaryReader: Python version inspired by the provided C++ code
import struct

from mmd.core.data_types import Vec2, Vec3, Vec4


class BinaryReader:
    def __init__(self, data: bytes):
        self.data = data
        self.offset = 0

    def can_read(self, size: int) -> bool:
        return self.offset + size <= len(self.data)

    def read_uint8(self) -> int:
        if not self.can_read(1):
            raise RuntimeError("Not enough data to read uint8")
        value = self.data[self.offset]
        self.offset += 1
        return value

    def read_int8(self) -> int:
        if not self.can_read(1):
            raise RuntimeError("Not enough data to read int8")
        value = struct.unpack_from("<b", self.data, self.offset)[0]
        self.offset += 1
        return value

    def read_int16(self) -> int:
        if not self.can_read(2):
            raise RuntimeError("Not enough data to read int16")
        value = struct.unpack_from("<h", self.data, self.offset)[0]
        self.offset += 2
        return value

    def read_uint16(self) -> int:
        if not self.can_read(2):
            raise RuntimeError("Not enough data to read uint16")
        value = struct.unpack_from("<H", self.data, self.offset)[0]
        self.offset += 2
        return value

    def read_int32(self) -> int:
        if not self.can_read(4):
            raise RuntimeError("Not enough data to read int32")
        value = struct.unpack_from("<i", self.data, self.offset)[0]
        self.offset += 4
        return value

    def read_uint32(self) -> int:
        if not self.can_read(4):
            raise RuntimeError("Not enough data to read uint32")
        value = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return value

    def read_float(self) -> float:
        if not self.can_read(4):
            raise RuntimeError("Not enough data to read float")
        value = struct.unpack_from("<f", self.data, self.offset)[0]
        self.offset += 4
        return value

    def read_text(self, is_utf8: bool = True) -> str:
        # Read length (uint32)
        if not self.can_read(4):
            raise RuntimeError("Not enough data for length field")
        length = self.read_uint32()
        if length == 0:
            return ""
        if not self.can_read(length):
            raise RuntimeError(f"Not enough data for string content (length: {length})")
        raw = self.data[self.offset : self.offset + length]
        self.offset += length
        if is_utf8:
            return raw.decode("utf-8")
        else:
            # UTF-16LE to UTF-8
            return raw.decode("utf-16le")

    def read_bytes(self, length: int) -> bytes:
        if not self.can_read(length):
            raise RuntimeError(f"Not enough data to read {length} bytes")
        value = self.data[self.offset : self.offset + length]
        self.offset += length
        return value

    def get_offset(self) -> int:
        return self.offset

    def remaining(self) -> int:
        return len(self.data) - self.offset

    def read_vec2(self) -> Vec2:
        return Vec2(self.read_float(), self.read_float())

    def read_vec3(self) -> Vec3:
        return Vec3(
            self.read_float(),
            self.read_float(),
            self.read_float(),
        )

    def read_vec4(self) -> Vec4:
        return Vec4(
            self.read_float(),
            self.read_float(),
            self.read_float(),
            self.read_float(),
        )
