import math
from dataclasses import dataclass, field
from enum import Enum, IntFlag
from typing import TypeGuard, cast


def index_type_to_string(value: int, allow_unsigned: bool = True) -> str:
    if value == 1:
        return "uint8" if allow_unsigned else "int8"
    elif value == 2:
        return "uint16" if allow_unsigned else "int16"
    elif value == 4:
        return "uint32" if allow_unsigned else "int32"
    else:
        return f"Unknown({value})"


# ----------------- Data Classes for PMX Model ----------------#


def _is_float2(t: object) -> TypeGuard[tuple[float, float]]:
    if not isinstance(t, tuple):
        return False
    tup: tuple[object, ...] = cast("tuple[object, ...]", t)
    return len(tup) == 2 and all(isinstance(v, (float, int)) for v in tup)


def _is_float3(t: object) -> TypeGuard[tuple[float, float, float]]:
    if not isinstance(t, tuple):
        return False
    tup: tuple[object, ...] = cast("tuple[object, ...]", t)
    return len(tup) == 3 and all(isinstance(v, (float, int)) for v in tup)


def _is_float4(t: object) -> TypeGuard[tuple[float, float, float, float]]:
    if not isinstance(t, tuple):
        return False
    tup: tuple[object, ...] = cast("tuple[object, ...]", t)
    return len(tup) == 4 and all(isinstance(v, (float, int)) for v in tup)


@dataclass
class Vec2:
    x: float
    y: float

    def __eq__(self, other: object) -> bool:
        """
        Compare Vec2 objects or 2-tuples with tolerant float equality.
        Uses math.isclose with a default tolerance of 1e-6.
        """
        tol = 1e-6
        if isinstance(other, Vec2):
            return math.isclose(
                self.x, other.x, rel_tol=tol, abs_tol=tol
            ) and math.isclose(self.y, other.y, rel_tol=tol, abs_tol=tol)
        elif _is_float2(other):
            x, y = other
            return math.isclose(self.x, x, rel_tol=tol, abs_tol=tol) and math.isclose(
                self.y, y, rel_tol=tol, abs_tol=tol
            )
        else:
            return False


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def __eq__(self, other: object) -> bool:
        """
        Compare Vec3 objects or 3-tuples with tolerant float equality.
        Uses math.isclose with a default tolerance of 1e-6.
        """
        tol = 1e-6
        if isinstance(other, Vec3):
            return (
                math.isclose(self.x, other.x, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.y, other.y, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.z, other.z, rel_tol=tol, abs_tol=tol)
            )
        elif _is_float3(other):
            x, y, z = other
            return (
                math.isclose(self.x, x, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.y, y, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.z, z, rel_tol=tol, abs_tol=tol)
            )
        else:
            return False


@dataclass
class Vec4:
    x: float
    y: float
    z: float
    w: float

    def __eq__(self, other: object) -> bool:
        """
        Compare Vec4 objects or 4-tuples with tolerant float equality.
        Uses math.isclose with a default tolerance of 1e-6.
        """
        tol = 1e-6
        if isinstance(other, Vec4):
            return (
                math.isclose(self.x, other.x, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.y, other.y, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.z, other.z, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.w, other.w, rel_tol=tol, abs_tol=tol)
            )
        elif _is_float4(other):
            x, y, z, w = other
            return (
                math.isclose(self.x, x, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.y, y, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.z, z, rel_tol=tol, abs_tol=tol)
                and math.isclose(self.w, w, rel_tol=tol, abs_tol=tol)
            )
        else:
            return False


# ------------------------------------------------------#
# ----------------- Vertex Data Classes ----------------#
# ------------------------------------------------------#
class WeightType(Enum):
    BDEF1 = 0
    BDEF2 = 1
    BDEF4 = 2
    SDEF = 3
    QDEF = 4


@dataclass
class VertexWeight:
    bone_index: list[int] = field(default_factory=list[int])
    weight: list[float] = field(default_factory=list[float])


@dataclass
class PmxVertex:
    position: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    normal: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    uv: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))
    additional_uvs: list[Vec4] = field(default_factory=list[Vec4])
    weight_type: WeightType = WeightType.BDEF1
    weight: VertexWeight = field(default_factory=VertexWeight)
    edge_scale: float = 1.0


# --------------------------------------------------------#
# ----------------- Material Data Classes ----------------#
# --------------------------------------------------------#


class MaterialFlagBits(IntFlag):
    NO_CULL = 1 << 0
    GROUND_SHADOW = 1 << 1
    DRAW_SHADOW = 1 << 2
    RECEIVE_SHADOW = 1 << 3
    HAS_EDGE = 1 << 4
    VERTEX_COLOR = 1 << 5  # 2.1
    POINT_DRAWING = 1 << 6  # 2.1
    LINE_DRAWING = 1 << 7  # 2.1

    def __str__(self):
        names = [
            ("NoCull", self.NO_CULL),
            ("GroundShadow", self.GROUND_SHADOW),
            ("DrawShadow", self.DRAW_SHADOW),
            ("ReceiveShadow", self.RECEIVE_SHADOW),
            ("HasEdge", self.HAS_EDGE),
            ("VertexColor", self.VERTEX_COLOR),
            ("PointDrawing", self.POINT_DRAWING),
            ("LineDrawing", self.LINE_DRAWING),
        ]
        flag_str = "[" + " ".join(name for name, bit in names if self & bit) + "]"
        return flag_str


@dataclass
class PmxMaterial:
    name_local: str = ""
    name_universal: str = ""
    diffuse_color: Vec4 = field(default_factory=lambda: Vec4(1.0, 1.0, 1.0, 1.0))
    specular_color: Vec3 = field(default_factory=lambda: Vec3(1.0, 1.0, 1.0))
    specular_strength: float = 1.0
    ambient_color: Vec3 = field(default_factory=lambda: Vec3(1.0, 1.0, 1.0))

    draw_flag: MaterialFlagBits = MaterialFlagBits(0)

    edge_color: Vec4 = field(default_factory=lambda: Vec4(0.0, 0.0, 0.0, 1.0))
    edge_size: float = 1.0

    texture_index: int = -1  # 32bit

    sphere_texture_index: int = -1  # 32bit
    sphere_mode: int = 0  # 8bit

    toon_flag: int = 0  # 8bit
    toon_value: int = 0  # 32bit

    meta_data: str = ""

    face_vertex_count: int = 0  # 32bit

    def has_flag(self, flag: MaterialFlagBits) -> bool:
        return bool(self.draw_flag & flag)

    def get_flags(self) -> list[str]:
        return [
            name
            for name, bit in MaterialFlagBits.__members__.items()
            if self.has_flag(bit)
        ]


# ----------------------------------------------------#
# ----------------- Bone Data Classes ----------------#
# ----------------------------------------------------#
class PMXBoneFlagBits(IntFlag):
    INDEXED_TAIL_POSITION = 1 << 0
    ROTATABLE = 1 << 1
    TRANSLATABLE = 1 << 2
    VISIBLE = 1 << 3
    ENABLED = 1 << 4
    IK = 1 << 5
    UNKNOWN_BIT_6 = 1 << 6
    UNKNOWN_BIT_7 = 1 << 7
    INHERIT_ROTATION = 1 << 8
    INHERIT_TRANSLATION = 1 << 9
    FIXED_AXIS = 1 << 10
    LOCAL_COORDINATE = 1 << 11
    PHYSICS_AFTER_DEFORM = 1 << 12
    EXTERNAL_PARENT_DEFORM = 1 << 13
    UNKNOWN_BIT_14 = 1 << 14
    UNKNOWN_BIT_15 = 1 << 15

    def __str__(self):
        names = [
            ("IndexedTailPosition", self.INDEXED_TAIL_POSITION),
            ("Rotatable", self.ROTATABLE),
            ("Translatable", self.TRANSLATABLE),
            ("Visible", self.VISIBLE),
            ("Enabled", self.ENABLED),
            ("IK", self.IK),
            ("UnknownBit6", self.UNKNOWN_BIT_6),
            ("UnknownBit7", self.UNKNOWN_BIT_7),
            ("InheritRotation", self.INHERIT_ROTATION),
            ("InheritTranslation", self.INHERIT_TRANSLATION),
            ("FixedAxis", self.FIXED_AXIS),
            ("LocalCoordinate", self.LOCAL_COORDINATE),
            ("PhysicsAfterDeform", self.PHYSICS_AFTER_DEFORM),
            ("ExternalParentDeform", self.EXTERNAL_PARENT_DEFORM),
            ("UnknownBit14", self.UNKNOWN_BIT_14),
            ("UnknownBit15", self.UNKNOWN_BIT_15),
        ]
        flag_str = "[" + " ".join(name for name, bit in names if self & bit) + "]"
        return flag_str


@dataclass
class InheritBone:
    parentBoneIndex: int
    influenceFactor: float


@dataclass
class FixedAxis:
    axis: Vec3


@dataclass
class BoneLocalCoordinate:
    xAxis: Vec3
    zAxis: Vec3


@dataclass
class BoneExternalParent:
    parentBoneIndex: int


@dataclass
class IKLink:
    boneIndex: int
    rotationLimitMin: Vec3 | None = None
    rotationLimitMax: Vec3 | None = None


def _default_ik_links() -> list[IKLink]:
    return []


@dataclass
class IK:
    targetBoneIndex: int
    loopCount: int
    limitRadian: float
    links: list[IKLink] = field(default_factory=_default_ik_links)


@dataclass
class PMXBone:
    nameLocal: str
    nameUniversal: str
    position: Vec3
    parentIndex: int
    level: int
    flags: PMXBoneFlagBits
    tailInfo: Vec3 | int
    inheritBone: InheritBone | None = None
    fixedAxis: FixedAxis | None = None
    localCoordinate: BoneLocalCoordinate | None = None
    externalParent: BoneExternalParent | None = None
    ik: IK | None = None


# ---------------------------------------------------------#
# ------------------- Morph Data Classes ------------------#
# ---------------------------------------------------------#
@dataclass
class MorphGroup:
    morph_index: int
    morph_value: float


@dataclass
class MorphVertex:
    vertex_index: int
    offset: Vec3


@dataclass
class MorphBone:
    bone_index: int
    position_offset: Vec3
    rotation_offset: Vec4


@dataclass
class MorphUV:
    vertex_index: int
    offset: Vec4


@dataclass
class MorphUV1:
    vertex_index: int
    offset: Vec4


@dataclass
class MorphUV2:
    vertex_index: int
    offset: Vec4


@dataclass
class MorphUV3:
    vertex_index: int
    offset: Vec4


@dataclass
class MorphUV4:
    vertex_index: int
    offset: Vec4


@dataclass
class MorphMaterial:
    material_index: int
    offset_operation: int
    diffuse: Vec4
    specular: Vec3
    specular_power: float
    ambient: Vec3
    edge_color: Vec4
    edge_size: float
    texture_tint: Vec4
    sphere_texture_tint: Vec4
    toon_texture_tint: Vec4


# The MorphData can be any of these lists
MorphData = (
    list[MorphGroup]
    | list[MorphVertex]
    | list[MorphBone]
    | list[MorphUV]
    | list[MorphUV1]
    | list[MorphUV2]
    | list[MorphUV3]
    | list[MorphUV4]
    | list[MorphMaterial]
)


class MorphType(Enum):
    GROUP = 0
    VERTEX = 1
    BONE = 2
    UV = 3
    UV1 = 4
    UV2 = 5
    UV3 = 6
    UV4 = 7
    MATERIAL = 8


@dataclass
class PMXMorph:
    name_local: str = ""
    name_universal: str = ""
    panel_type: int = 0
    morph_type: MorphType = MorphType.GROUP
    data: MorphData = field(default_factory=lambda: cast(MorphData, []))


# -------------------------------------------------------------#
# ----------------- Display Frame Data Classes ----------------#
# -------------------------------------------------------------#
class FrameType(Enum):
    BONE = 0
    MORPH = 1


@dataclass
class BoneFrameElement:
    bone_index: int


@dataclass
class MorphFrameElement:
    morph_index: int


@dataclass
class FrameData:
    frame_type: FrameType
    data: BoneFrameElement | MorphFrameElement


@dataclass
class PMXDisplayFrame:
    name_local: str = ""
    name_universal: str = ""
    special_flag: int = 0
    frame_elements: list[FrameData] = field(default_factory=list[FrameData])


# -----------------------------------------------------------#
# ----------------- Rigid Body Data Classes -----------------#
# -----------------------------------------------------------#


class ShapeType(Enum):
    SPHERE = 0
    BOX = 1
    CAPSULE = 2


class PhysicsMode(Enum):
    FOLLOW_BONE = 0  # Rigid body sticks to bone
    PHYSICS = 1  # Physics drives rigid body
    PHYSICS_BONE = 2  # Physics drives rigid body; result is applied to bone


@dataclass
class PMXRigidBody:
    name_local: str = ""
    name_universal: str = ""
    related_bone_index: int = -1
    group_id: int = 0  # byte
    non_collision_group: int = 0
    shape: ShapeType = ShapeType.SPHERE
    shape_size: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    shape_position: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    shape_rotation: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    mass: float = 1.0
    move_attenuation: float = 0.0
    rotation_damping: float = 0.0
    repulsion: float = 0.0
    friction_force: float = 0.0
    physics_mode: PhysicsMode = PhysicsMode.FOLLOW_BONE


# -------------------------------------------------------------#
# ----------------- Joint Data Classes -----------------------#
# ------------------------------------------------------------#


class JointType(Enum):
    SPRING_6DOF = 0  # 2.0
    SIX_DOF = 1  # 2.1 (6DOF)
    P2P = 2  # 2.1
    CONETWIST = 3  # 2.1
    SLIDER = 4  # 2.1
    HINGE = 5  # 2.1


@dataclass
class PMXJoint:
    name_local: str = ""
    name_universal: str = ""
    type: JointType = JointType.SPRING_6DOF
    rigid_body_index_a: int = -1
    rigid_body_index_b: int = -1
    position: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    rotation: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    position_min: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    position_max: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    rotation_min: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    rotation_max: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    position_spring_constant: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))
    rotation_spring_constant: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))


# -------------------------------------------------------------#
# ------------------ Main PMX Model Data Class ----------------#
# -------------------------------------------------------------#


@dataclass
class PmxHeader:
    magic: bytes = b""
    version: float = 0.0
    global_count: int = 0
    globals: list[int] = field(default_factory=list[int])
    model_name_local: str = ""
    model_name_universal: str = ""
    model_comment_local: str = ""
    model_comment_universal: str = ""

    def __str__(self) -> str:
        def indent_multiline(text: str, prefix: str) -> str:
            return "\n".join(
                prefix + line if line else prefix for line in text.splitlines()
            )

        lines: list[str] = []
        lines.append("PMX Header Information:")
        # Magic as string (printable)
        try:
            magic_str = self.magic.decode("ascii")
        except Exception:
            magic_str = str(self.magic)
        lines.append(f"  Magic: {magic_str}")
        lines.append(f"  Version: {self.version:.1f}")
        lines.append(f"  Global Count: {self.global_count}")
        lines.append("  Globals:")
        if self.globals and len(self.globals) >= 8:
            is_utf8 = self.globals[0] == 1
            lines.append(f"    [0] Encoding: {'UTF-8' if is_utf8 else 'UTF-16LE'}")
            lines.append(f"    [1] Additional Vec Count: {self.globals[1]}")
            lines.append(
                f"    [2] Vertex Index Size: {index_type_to_string(self.globals[2], False)}"
            )
            lines.append(
                f"    [3] Texture Index Size: {index_type_to_string(self.globals[3], True)}"
            )
            lines.append(
                f"    [4] Material Index Size: {index_type_to_string(self.globals[4], True)}"
            )
            lines.append(
                f"    [5] Bone Index Size: {index_type_to_string(self.globals[5], True)}"
            )
            lines.append(
                f"    [6] Morph Index Size: {index_type_to_string(self.globals[6], True)}"
            )
            lines.append(
                f"    [7] Rigid Body Index Size: {index_type_to_string(self.globals[7], True)}"
            )
        else:
            lines.append(f"    Globals: {self.globals}")
        lines.append(f"  Model Name (Local): {self.model_name_local}")
        lines.append(f"  Model Name (Universal): {self.model_name_universal}")
        lines.append("  Comment (Local):")
        lines.append(indent_multiline(self.model_comment_local, "    "))
        lines.append("  Comment (Universal):")
        lines.append(indent_multiline(self.model_comment_universal, "    "))
        return "\n".join(lines)


@dataclass
class PmxModel:
    model_name: str
    file_path: str
    absolute_path: str

    header: PmxHeader

    vertices: list[PmxVertex] = field(default_factory=list[PmxVertex])
    indices: list[int] = field(default_factory=list[int])

    textures_paths: list[str] = field(default_factory=list[str])
    materials: list[PmxMaterial] = field(default_factory=list[PmxMaterial])

    bones: list[PMXBone] = field(default_factory=list[PMXBone])

    morphs: list[PMXMorph] = field(default_factory=list[PMXMorph])

    display_frames: list[PMXDisplayFrame] = field(default_factory=list[PMXDisplayFrame])

    rigid_bodies: list[PMXRigidBody] = field(default_factory=list[PMXRigidBody])

    joints: list[PMXJoint] = field(default_factory=list[PMXJoint])


# -------------------------------------------------------------#
# ------------------ VMD Motion Data Classes ------------------#
# -------------------------------------------------------------#


@dataclass
class VMDMotion:
    bone_name: str
    frame_number: int
    position: Vec3
    rotation: Vec4  # Quaternion (x, y, z, w)
    interpolation: bytes  # 64 bytes


@dataclass
class VMDMorph:
    morph_name: str
    frame_number: int
    weight: float


@dataclass
class VMDCamera:
    frame_number: int
    distance: float
    position: Vec3
    rotation: Vec3  # Euler angles
    interpolation: bytes  # 24 bytes
    viewing_angle: int
    perspective: bool  # True = perspective, False = orthographic


@dataclass
class VMDLight:
    frame_number: int
    color: Vec3  # RGB
    direction: Vec3  # Direction vector


@dataclass
class VMDShadow:
    frame_number: int
    mode: int  # 0: none, 1: mode1, 2: mode2
    distance: float


@dataclass
class VMDIKState:
    ik_name: str
    enabled: bool


@dataclass
class VMDProperty:
    frame_number: int
    visible: bool
    ik_states: list[VMDIKState]


class VMDVersion(Enum):
    VMD_1_0 = "Vocaloid Motion Data file"
    VMD_2_0 = "Vocaloid Motion Data 0002"


@dataclass
class VMDFile:
    header: str
    version: VMDVersion
    model_name: str
    bone_keyframes: list[VMDMotion] = field(default_factory=list[VMDMotion])
    morph_keyframes: list[VMDMorph] = field(default_factory=list[VMDMorph])
    camera_keyframes: list[VMDCamera] = field(default_factory=list[VMDCamera])
    light_keyframes: list[VMDLight] = field(default_factory=list[VMDLight])
    shadow_keyframes: list[VMDShadow] = field(default_factory=list[VMDShadow])
    property_keyframes: list[VMDProperty] = field(default_factory=list[VMDProperty])


# -----------------------------------------------------------#
# ------------------ VPD Pose Data Classes ------------------#
# -----------------------------------------------------------#


@dataclass
class VPDBonePose:
    bone_idx: str  # Original bone name as string (not index) e.g "Bone1"
    bone_name: str  # "Bone1" but converted to match PMX bone names (e.g. "センター")
    position: Vec3
    rotation: Vec4  # Quaternion (x, y, z, w)


@dataclass
class VPDFile:
    header: str
    model_name: str
    bone_count: int
    bones: list[VPDBonePose] = field(default_factory=list[VPDBonePose])
