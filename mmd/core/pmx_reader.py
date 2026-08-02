# PMXReader: Python version inspired by the provided C++ code
from mmd.core.binary_reader import BinaryReader
from mmd.core.data_types import (
    FrameType,
    JointType,
    MaterialFlagBits,
    MorphType,
    PhysicsMode,
    PMXBoneFlagBits,
    PmxHeader,
    ShapeType,
    Vec2,
    Vec3,
    Vec4,
    WeightType,
)


class PMXConfig:
    def __init__(self, header: PmxHeader):
        # These are the index sizes as per PMX spec, from header.globals
        # [0]=isUTF8, [1]=additionalUVs,
        # [2]=vertex, [3]=texture, [4]=material,
        # [5]=bone, [6]=morph, [7]=rigidbody
        self.is_utf8 = header.globals[0] == 1 if len(header.globals) > 0 else True
        self.additionalUVCount = header.globals[1] if len(header.globals) > 1 else 0
        self.vertexIndexSize = header.globals[2] if len(header.globals) > 2 else 4
        self.textureIndexSize = header.globals[3] if len(header.globals) > 3 else 4
        self.materialIndexSize = header.globals[4] if len(header.globals) > 4 else 4
        self.boneIndexSize = header.globals[5] if len(header.globals) > 5 else 4
        self.morphIndexSize = header.globals[6] if len(header.globals) > 6 else 4
        self.rigidBodyIndexSize = header.globals[7] if len(header.globals) > 7 else 4


class PMXReader:
    def __init__(self, reader: BinaryReader, config: PMXConfig):
        self.reader = reader
        self.config = config

    def readInt8(self) -> int:
        return self.reader.read_int8()

    def readInt16(self) -> int:
        return self.reader.read_int16()

    def readUInt16(self) -> int:
        return self.reader.read_uint16()

    def readInt32(self) -> int:
        return self.reader.read_int32()

    def readUInt32(self) -> int:
        return self.reader.read_uint32()

    def readFloat(self) -> float:
        return self.reader.read_float()

    def readVec2(self) -> Vec2:
        return self.reader.read_vec2()

    def readVec3(self) -> Vec3:
        return self.reader.read_vec3()

    def readVec4(self) -> Vec4:
        return self.reader.read_vec4()

    def readIndex(self, size: int, signed: bool = True) -> int:
        if signed:
            if size == 1:
                return self.reader.read_int8()
            elif size == 2:
                return self.reader.read_int16()
            elif size == 4:
                return self.reader.read_int32()
            else:
                raise RuntimeError(f"Unsupported signed index size: {size}")
        else:
            if size == 1:
                return self.reader.read_uint8()
            elif size == 2:
                return self.reader.read_uint16()
            elif size == 4:
                return self.reader.read_uint32()
            else:
                raise RuntimeError(f"Unsupported unsigned index size: {size}")

    def readVertexIndex(self) -> int:
        return self.readIndex(self.config.vertexIndexSize, signed=False)

    def readBoneIndex(self) -> int:
        return self.readIndex(self.config.boneIndexSize, signed=True)

    def readTextureIndex(self) -> int:
        return self.readIndex(self.config.textureIndexSize, signed=True)

    def readMaterialIndex(self) -> int:
        return self.readIndex(self.config.materialIndexSize, signed=True)

    def readMorphIndex(self) -> int:
        return self.readIndex(self.config.morphIndexSize, signed=True)

    def readRigidBodyIndex(self) -> int:
        return self.readIndex(self.config.rigidBodyIndexSize, signed=True)

    def readText(self) -> str:
        return self.reader.read_text(self.config.is_utf8)

    def readWeightType(self) -> WeightType:
        value = self.readInt8()
        return WeightType(value)

    def readMaterialFlagBits(self) -> MaterialFlagBits:
        value = self.readInt8()
        return MaterialFlagBits(value)

    def readBoneFlagBits(self) -> PMXBoneFlagBits:
        value = self.readUInt16()
        return PMXBoneFlagBits(value)

    def readMorphType(self) -> MorphType:
        value = self.readInt8()
        return MorphType(value)

    def readFrameType(self) -> FrameType:
        value = self.readInt8()
        return FrameType(value)

    def readShapeType(self) -> ShapeType:
        value = self.readInt8()
        return ShapeType(value)

    def readPhysicsMode(self) -> PhysicsMode:
        value = self.readInt8()
        return PhysicsMode(value)

    def readJointType(self) -> JointType:
        value = self.readInt8()
        return JointType(value)

    def isAtEnd(self):
        return self.reader.remaining() == 0
