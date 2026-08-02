import enum
import logging
import os
import json

from mmd.core.binary_reader import BinaryReader
from mmd.core.data_types import (
    Vec3,
    Vec4,
    VMDCamera,
    VMDFile,
    VMDIKState,
    VMDLight,
    VMDMorph,
    VMDMotion,
    VMDProperty,
    VMDShadow,
    VMDVersion,
)

log = logging.getLogger(__name__)


class VMDParseError(RuntimeError):
    """Base exception for VMD parsing failures."""


class VMDStepParseError(VMDParseError):
    """Base exception for VMD step-level parse failures."""


class VMDReadError(VMDStepParseError):
    """Exception raised when VMD bytes cannot be read from disk."""


class VMDHeaderParseError(VMDStepParseError):
    """Exception raised while parsing the VMD header/version block."""


class VMDModelNameParseError(VMDStepParseError):
    """Exception raised while parsing the VMD model name."""


class VMDBoneParseError(VMDStepParseError):
    """Exception raised while parsing VMD bone keyframes."""


class VMDMorphParseError(VMDStepParseError):
    """Exception raised while parsing VMD morph keyframes."""


class VMDCameraParseError(VMDStepParseError):
    """Exception raised while parsing VMD camera keyframes."""


class VMDLightParseError(VMDStepParseError):
    """Exception raised while parsing VMD light keyframes."""


class VMDShadowParseError(VMDStepParseError):
    """Exception raised while parsing VMD self-shadow keyframes."""


class VMDPropertyParseError(VMDStepParseError):
    """Exception raised while parsing VMD property/IK keyframes."""


def read_header(reader: BinaryReader) -> str:
    """Read VMD header (30 bytes, null-terminated string)."""
    try:
        header_bytes: list[int] = []
        for _ in range(30):
            byte = reader.read_uint8()
            if byte == 0:
                break
            header_bytes.append(byte)

        # Consume all 30 bytes, skip remaining if we stopped early
        bytes_read = len(header_bytes) + 1
        if bytes_read < 30:
            reader.read_bytes(30 - bytes_read)

        return bytes(header_bytes).decode("utf-8", errors="replace")
    except VMDHeaderParseError:
        raise
    except Exception as e:
        raise VMDHeaderParseError(f"Failed to read VMD header: {e}") from e


def determine_vmd_version(header: str) -> VMDVersion:
    """Determine VMD file version from header string."""
    if header.startswith("Vocaloid Motion Data file"):
        return VMDVersion.VMD_1_0
    elif header.startswith("Vocaloid Motion Data 0002"):
        return VMDVersion.VMD_2_0
    else:
        raise VMDHeaderParseError(f"Unknown VMD file version. Header: {header}")


def read_model_name(reader: BinaryReader, version: VMDVersion) -> str:
    """Read model name (10 or 20 bytes depending on version)."""
    try:
        if version == VMDVersion.VMD_1_0:
            model_name_bytes = reader.read_bytes(10)
        elif version == VMDVersion.VMD_2_0:
            model_name_bytes = reader.read_bytes(20)
        else:
            raise VMDModelNameParseError(f"Unknown VMD version: {version}")

        # Skip leading nulls and decode as Shift-JIS
        trimmed_bytes = model_name_bytes.lstrip(b"\x00")
        return trimmed_bytes.split(b"\x00", 1)[0].decode("shift_jis", errors="replace")
    except VMDModelNameParseError:
        raise
    except Exception as e:
        raise VMDModelNameParseError(f"Failed to read VMD model name: {e}") from e


def read_bone_keyframe(reader: BinaryReader) -> VMDMotion:
    """Read a single bone keyframe."""
    try:
        bone_name_bytes = reader.read_bytes(15)
        bone_name = (
            bone_name_bytes.lstrip(b"\x00")
            .split(b"\x00", 1)[0]
            .decode("shift_jis", errors="replace")
        )
        frame_number = reader.read_uint32()
        position = Vec3(reader.read_float(), reader.read_float(), reader.read_float())
        rotation = Vec4(
            reader.read_float(),
            reader.read_float(),
            reader.read_float(),
            reader.read_float(),
        )
        interpolation = reader.read_bytes(64)

        return VMDMotion(
            bone_name=bone_name,
            frame_number=frame_number,
            position=position,
            rotation=rotation,
            interpolation=interpolation,
        )
    except VMDBoneParseError:
        raise
    except Exception as e:
        raise VMDBoneParseError(f"Failed to read VMD bone keyframe: {e}") from e


def read_bone_keyframes(reader: BinaryReader) -> list[VMDMotion]:
    """Read all bone keyframes."""
    try:
        count = reader.read_uint32()
        return [read_bone_keyframe(reader) for _ in range(count)]
    except VMDBoneParseError:
        raise
    except Exception as e:
        raise VMDBoneParseError(f"Failed to read VMD bone keyframes: {e}") from e


def read_morph_keyframes(reader: BinaryReader) -> list[VMDMorph]:
    """Read all morph/shape keyframes."""
    try:
        count = reader.read_uint32()
        morph_keyframes: list[VMDMorph] = []

        for _ in range(count):
            morph_name_bytes = reader.read_bytes(15)
            morph_name = (
                morph_name_bytes.lstrip(b"\x00")
                .split(b"\x00", 1)[0]
                .decode("shift_jis", errors="replace")
            )
            frame_number = reader.read_uint32()
            weight = reader.read_float()

            morph_keyframes.append(
                VMDMorph(
                    morph_name=morph_name,
                    frame_number=frame_number,
                    weight=weight,
                )
            )

        return morph_keyframes
    except VMDMorphParseError:
        raise
    except Exception as e:
        raise VMDMorphParseError(f"Failed to read VMD morph keyframes: {e}") from e


def read_camera_keyframes(reader: BinaryReader) -> list[VMDCamera]:
    """Read all camera keyframes."""
    try:
        count = reader.read_uint32()
        camera_keyframes: list[VMDCamera] = []

        for _ in range(count):
            frame_number = reader.read_uint32()
            distance = reader.read_float()
            position = Vec3(
                reader.read_float(), reader.read_float(), reader.read_float()
            )
            rotation = Vec3(
                reader.read_float(), reader.read_float(), reader.read_float()
            )
            interpolation = reader.read_bytes(24)
            viewing_angle = reader.read_uint32()
            perspective = reader.read_uint8() == 0

            camera_keyframes.append(
                VMDCamera(
                    frame_number=frame_number,
                    distance=distance,
                    position=position,
                    rotation=rotation,
                    interpolation=interpolation,
                    viewing_angle=viewing_angle,
                    perspective=perspective,
                )
            )

        return camera_keyframes
    except VMDCameraParseError:
        raise
    except Exception as e:
        raise VMDCameraParseError(f"Failed to read VMD camera keyframes: {e}") from e


def read_light_keyframes(reader: BinaryReader) -> list[VMDLight]:
    """Read all light keyframes."""
    try:
        count = reader.read_uint32()
        light_keyframes: list[VMDLight] = []

        for _ in range(count):
            frame_number = reader.read_uint32()
            color = Vec3(reader.read_float(), reader.read_float(), reader.read_float())
            direction = Vec3(
                reader.read_float(), reader.read_float(), reader.read_float()
            )

            light_keyframes.append(
                VMDLight(
                    frame_number=frame_number,
                    color=color,
                    direction=direction,
                )
            )

        return light_keyframes
    except VMDLightParseError:
        raise
    except Exception as e:
        raise VMDLightParseError(f"Failed to read VMD light keyframes: {e}") from e


def read_self_shadow_keyframes(reader: BinaryReader) -> list[VMDShadow]:
    """Read all self-shadow keyframes."""
    try:
        count = reader.read_uint32()
        shadow_keyframes: list[VMDShadow] = []

        for _ in range(count):
            frame_number = reader.read_uint32()
            mode = reader.read_uint8()
            distance = reader.read_float()

            shadow_keyframes.append(
                VMDShadow(
                    frame_number=frame_number,
                    mode=mode,
                    distance=distance,
                )
            )

        return shadow_keyframes
    except VMDShadowParseError:
        raise
    except Exception as e:
        raise VMDShadowParseError(
            f"Failed to read VMD self-shadow keyframes: {e}"
        ) from e


def read_property_keyframes(reader: BinaryReader) -> list[VMDProperty]:
    """Read all property/IK keyframes."""
    try:
        count = reader.read_uint32()
        property_keyframes: list[VMDProperty] = []

        for _ in range(count):
            frame_number = reader.read_uint32()
            visible = bool(reader.read_uint8())
            ik_count = reader.read_uint32()

            ik_states: list[VMDIKState] = []
            for _ in range(ik_count):
                ik_name_bytes = reader.read_bytes(20)
                ik_name = ik_name_bytes.split(b"\x00", 1)[0].decode(
                    "shift_jis", errors="replace"
                )
                ik_enable = bool(reader.read_uint8())
                ik_states.append(VMDIKState(ik_name=ik_name, enabled=ik_enable))

            property_keyframes.append(
                VMDProperty(
                    frame_number=frame_number,
                    visible=visible,
                    ik_states=ik_states,
                )
            )

        return property_keyframes
    except VMDPropertyParseError:
        raise
    except Exception as e:
        raise VMDPropertyParseError(
            f"Failed to read VMD property/IK keyframes: {e}"
        ) from e


def parse_vmd_file(file_path: str) -> VMDFile:
    """Parse a VMD file and extract all motion data.

    Args:
        file_path: Path to the VMD file

    Returns:
        VMDFile object containing all parsed data
    """
    try:
        try:
            with open(file_path, "rb") as f:
                data = f.read()
        except Exception as e:
            raise VMDReadError(f"Failed to read VMD file '{file_path}': {e}") from e

        reader = BinaryReader(data)
        header = read_header(reader)
        version = determine_vmd_version(header)
        model_name = read_model_name(reader, version)
        bone_keyframes = read_bone_keyframes(reader)
        morph_keyframes = read_morph_keyframes(reader)
        camera_keyframes = read_camera_keyframes(reader)

        # Optional sections (may not exist in all files)
        light_keyframes = read_light_keyframes(reader) if reader.can_read(4) else []
        shadow_keyframes = (
            read_self_shadow_keyframes(reader) if reader.can_read(4) else []
        )
        property_keyframes = (
            read_property_keyframes(reader) if reader.can_read(4) else []
        )

        # Verify all data was consumed
        if reader.get_offset() != len(data):
            log.warning(
                "VMD file not fully consumed. Offset: %d, Length: %d",
                reader.get_offset(),
                len(data),
            )

        return VMDFile(
            header=header,
            version=version,
            model_name=model_name,
            bone_keyframes=bone_keyframes,
            morph_keyframes=morph_keyframes,
            camera_keyframes=camera_keyframes,
            light_keyframes=light_keyframes,
            shadow_keyframes=shadow_keyframes,
            property_keyframes=property_keyframes,
        )
    except VMDStepParseError as e:
        raise VMDParseError(f"Failed to parse VMD file '{file_path}': {e}") from e
    except Exception as e:
        raise VMDParseError(
            f"Unexpected error while parsing VMD file '{file_path}': {e}"
        ) from e


# --- JSON serialization helper for custom types ---
def dump_vmd_to_json(vmd_file: VMDFile, output_path: str) -> None:
    def serialize_for_json(obj: object) -> object:
        if isinstance(obj, enum.Enum):
            return obj.name
        elif isinstance(obj, bytes):
            # Convert bytes to hex string for safe JSON serialization
            return obj.hex()
        elif hasattr(obj, "x") and hasattr(obj, "y"):
            # Vec2/Vec3/Vec4 — must be checked before __dict__ because dataclasses have __dict__
            return [getattr(obj, a) for a in ("x", "y", "z", "w") if hasattr(obj, a)]
        elif isinstance(obj, (list, tuple)):
            from typing import cast

            seq = cast("list[object] | tuple[object, ...]", obj)
            return [serialize_for_json(v) for v in seq]
        elif isinstance(obj, (int, float, str, type(None))):
            return obj
        elif hasattr(obj, "__dict__"):
            if isinstance(obj, type):
                return str(obj)
            fields: dict[str, object] = obj.__dict__
            return {k: serialize_for_json(v) for k, v in fields.items()}
        else:
            return str(obj)

    os.makedirs(output_path, exist_ok=True)

    # Header
    header_json_path = os.path.join(output_path, "header.json")
    try:
        log.debug("Saving VMD header to JSON: %s", header_json_path)
        with open(header_json_path, "w", encoding="utf-8") as json_file:
            json.dump(vmd_file.header, json_file, ensure_ascii=False, indent=4)
    except Exception as e:
        log.warning("Failed to save VMD header to JSON: %s", e)

    # Model name
    model_name_json_path = os.path.join(output_path, "model_name.json")
    try:
        log.debug("Saving VMD model name to JSON: %s", model_name_json_path)
        with open(model_name_json_path, "w", encoding="utf-8") as json_file:
            json.dump(vmd_file.model_name, json_file, ensure_ascii=False, indent=4)
    except Exception as e:
        log.warning("Failed to save VMD model name to JSON: %s", e)

    # Bone keyframes
    bone_keyframes_json_path = os.path.join(output_path, "bone_keyframes.json")
    try:
        log.debug("Saving VMD bone keyframes to JSON: %s", bone_keyframes_json_path)
        with open(bone_keyframes_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(bk) for bk in vmd_file.bone_keyframes],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save VMD bone keyframes to JSON: %s", e)

    # Morph keyframes
    morph_keyframes_json_path = os.path.join(output_path, "morph_keyframes.json")
    try:
        log.debug("Saving VMD morph keyframes to JSON: %s", morph_keyframes_json_path)
        with open(morph_keyframes_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(mk) for mk in vmd_file.morph_keyframes],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save VMD morph keyframes to JSON: %s", e)

    # Camera keyframes
    camera_keyframes_json_path = os.path.join(output_path, "camera_keyframes.json")
    try:
        log.debug("Saving VMD camera keyframes to JSON: %s", camera_keyframes_json_path)
        with open(camera_keyframes_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(ck) for ck in vmd_file.camera_keyframes],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save VMD camera keyframes to JSON: %s", e)

    # Light keyframes
    light_keyframes_json_path = os.path.join(output_path, "light_keyframes.json")
    try:
        log.debug("Saving VMD light keyframes to JSON: %s", light_keyframes_json_path)
        with open(light_keyframes_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(lk) for lk in vmd_file.light_keyframes],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save VMD light keyframes to JSON: %s", e)

    # Shadow keyframes
    shadow_keyframes_json_path = os.path.join(output_path, "shadow_keyframes.json")
    try:
        log.debug("Saving VMD shadow keyframes to JSON: %s", shadow_keyframes_json_path)
        with open(shadow_keyframes_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(sk) for sk in vmd_file.shadow_keyframes],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save VMD shadow keyframes to JSON: %s", e)

    # Property keyframes
    property_keyframes_json_path = os.path.join(output_path, "property_keyframes.json")
    try:
        log.debug(
            "Saving VMD property keyframes to JSON: %s", property_keyframes_json_path
        )
        with open(property_keyframes_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(pk) for pk in vmd_file.property_keyframes],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save VMD property keyframes to JSON: %s", e)
