import enum
import json
import logging
import os
import re

from mmd.core.data_types import Vec3, Vec4, VPDBonePose, VPDFile

log = logging.getLogger(__name__)


class VPDParseError(RuntimeError):
    """Base exception for VPD parsing failures."""


class VPDStepParseError(VPDParseError):
    """Base exception for VPD step-level parse failures."""


class VPDReadError(VPDStepParseError):
    """Exception raised when VPD bytes cannot be read from disk."""


class VPDDecodeError(VPDStepParseError):
    """Exception raised when VPD bytes cannot be decoded to text."""


class VPDHeaderParseError(VPDStepParseError):
    """Exception raised while parsing VPD header data."""


class VPDBoneParseError(VPDStepParseError):
    """Exception raised while parsing VPD bone pose data."""


def _clean_data(data: str) -> str:
    """Clean the raw data from a VPD file by removing unnecessary whitespace and comments.

    Args:
        data: The raw content of the VPD file as a string
    Returns:
        A cleaned version of the data ready for parsing
    """
    # Remove comments (lines starting with //) and trim whitespace
    lines = []
    for line in data.split("\n"):
        if "//" in line:
            line = line[: line.index("//")]
        lines.append(line)

    # Remove empty lines and trim whitespace
    lines = [line.strip() for line in lines if line.strip()]

    return "\n".join(lines)


def _parse_header(data: str) -> dict[str, str | int]:
    """Parse the header of a VPD file to extract metadata.

    Args:
        data: The raw content of the VPD file as a string

    Returns:
        A dictionary containing parsed header information
    """
    # First line should start with "Vocaloid Pose Data File"
    # Second line is the model name
    # Third line is the bone count
    # Note: VPD files do NOT have a morph count line
    lines = data.split("\n")
    header_info = {}
    if len(lines) < 3:
        raise VPDHeaderParseError(
            "VPD file is too short to contain valid header information."
        )
    if not lines[0].lower().startswith("vocaloid pose data file"):
        raise VPDHeaderParseError("VPD file does not start with expected header line.")

    header_info["model_name"] = lines[1].strip()

    try:
        # Remove ';' at the end of bone count if present
        line = lines[2].strip()
        line = line.removesuffix(";")
        header_info["bone_count"] = int(line)
    except ValueError:
        raise VPDHeaderParseError("Bone count in VPD header is not a valid integer.")

    return header_info


def _parse_bones(data: str) -> list[VPDBonePose]:
    """Parse bone poses from the VPD file content.

    Args:
        data: The cleaned VPD file content

    Returns:
        List of VPDBonePose objects
    """
    bones = []

    # Regex pattern to match bone entries
    # Pattern explanation:
    # Bone\d+\{   - Match "BoneN{" where N is a number
    # ([^{}]+)    - Capture bone name (everything until next brace)
    # ([\d\.\-,;\s]+) - Capture translation values
    # ([\d\.\-,;\s]+) - Capture quaternion values
    # \}          - Match closing brace
    bone_pattern = re.compile(
        r"Bone(\d+)\{([^\n]+)\n\s*([0-9\.\-,;\s]+)\n\s*([0-9\.\-,;\s]+)\s*\}",
        re.MULTILINE,
    )

    for match in bone_pattern.finditer(data):
        bone_idx = match.group(1)  # Bone number
        bone_name = match.group(2).strip()  # Bone name
        trans_str = match.group(3).strip()  # Translation line
        rot_str = match.group(4).strip()  # Rotation line

        # Parse translation values (x, y, z)
        trans_values_str = [x.strip() for x in trans_str.rstrip(";").split(",")]
        trans_values = []
        for v in trans_values_str:
            try:
                trans_values.append(float(v))
            except ValueError as e:
                raise VPDBoneParseError(
                    f"Invalid translation values for {bone_name}: {trans_str}"
                ) from e
        if len(trans_values) != 3:
            raise VPDBoneParseError(
                f"Invalid translation values for {bone_name}: {trans_str}"
            )

        # Parse rotation values (quaternion x, y, z, w)
        rot_values_str = [x.strip() for x in rot_str.rstrip(";").split(",")]
        rot_values = []
        for v in rot_values_str:
            try:
                rot_values.append(float(v))
            except ValueError as e:
                raise VPDBoneParseError(
                    f"Invalid rotation values for {bone_name}: {rot_str}"
                ) from e
        if len(rot_values) != 4:
            raise VPDBoneParseError(
                f"Invalid rotation values for {bone_name}: {rot_str}"
            )

        bone_pose = VPDBonePose(
            bone_idx=f"Bone{bone_idx}",
            bone_name=bone_name,
            position=Vec3(*trans_values),
            rotation=Vec4(*rot_values),
        )
        bones.append(bone_pose)

    return bones


def parse_vpd_file(file_path: str) -> VPDFile:
    """Parse a VPD file and extract all motion data.

    Args:
        file_path: Path to the VPD file

    Returns:
        VPDFile object containing all parsed data
    """
    try:
        try:
            # VPD file is a text file
            # Encoding is usually Shift-JIS (cp932) for Japanese files
            # TODO: we should detect encoding instead of hardcoding it
            with open(file_path, "r", encoding="cp932") as f:
                data = f.read()
        except UnicodeDecodeError:
            # Fallback to UTF-8 if Shift-JIS fails
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    data = f.read()
            except Exception as e:
                raise VPDDecodeError(
                    f"Failed to decode VPD file '{file_path}' with cp932 and utf-8: {e}"
                ) from e
        except Exception as e:
            raise VPDReadError(f"Failed to read VPD file '{file_path}': {e}") from e

        # Clean the data
        cleaned_data = _clean_data(data)

        # Parse header
        header_info = _parse_header(cleaned_data)

        # Parse bone poses
        bones = _parse_bones(cleaned_data)

        # Validate bone count
        if len(bones) != header_info["bone_count"]:
            log.warning(
                "Bone count mismatch in '%s': header says %d, parsed %d",
                file_path,
                header_info["bone_count"],
                len(bones),
            )

        # Create and return VPDFile object
        vpd_file = VPDFile(
            header="Vocaloid Pose Data file",
            model_name=header_info["model_name"].rstrip(";"),
            bone_count=header_info["bone_count"],
            bones=bones,
        )

        return vpd_file
    except VPDStepParseError as e:
        raise VPDParseError(f"Failed to parse VPD file '{file_path}': {e}") from e
    except Exception as e:
        raise VPDParseError(
            f"Unexpected error while parsing VPD file '{file_path}': {e}"
        ) from e


def dump_vpd_to_json(vpd_file: VPDFile, output_path: str) -> None:
    """Serialize a parsed VPDFile object to JSON files in the given output directory.

    Produces the following files inside *output_path*:
    - header.json        — VPD header string
    - model_name.json    — model name string
    - bone_keyframes.json — list of bone poses (bone_idx, bone_name, position, rotation)

    Args:
        vpd_file: The parsed VPDFile object to serialize.
        output_path: Directory path where the JSON files will be written.
    """

    def serialize_for_json(obj: object) -> object:
        if isinstance(obj, enum.Enum):
            return obj.name
        elif isinstance(obj, bytes):
            return obj.hex()
        elif hasattr(obj, "x") and hasattr(obj, "y"):
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
        log.debug("Saving VPD header to JSON: %s", header_json_path)
        with open(header_json_path, "w", encoding="utf-8") as json_file:
            json.dump(vpd_file.header, json_file, ensure_ascii=False, indent=4)
    except Exception as e:
        log.warning("Failed to save VPD header to JSON: %s", e)

    # Model name
    model_name_json_path = os.path.join(output_path, "model_name.json")
    try:
        log.debug("Saving VPD model name to JSON: %s", model_name_json_path)
        with open(model_name_json_path, "w", encoding="utf-8") as json_file:
            json.dump(vpd_file.model_name, json_file, ensure_ascii=False, indent=4)
    except Exception as e:
        log.warning("Failed to save VPD model name to JSON: %s", e)

    # Bone keyframes
    bone_keyframes_json_path = os.path.join(output_path, "bone_keyframes.json")
    try:
        log.debug("Saving VPD bone keyframes to JSON: %s", bone_keyframes_json_path)
        with open(bone_keyframes_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(bp) for bp in vpd_file.bones],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save VPD bone keyframes to JSON: %s", e)
