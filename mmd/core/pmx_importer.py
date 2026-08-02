import enum
import json
import logging
import os
from typing import cast

from mmd.core.binary_reader import BinaryReader
from mmd.core.data_types import (
    IK,
    BoneExternalParent,
    BoneFrameElement,
    BoneLocalCoordinate,
    FixedAxis,
    FrameData,
    FrameType,
    MorphFrameElement,
    IKLink,
    InheritBone,
    MorphBone,
    MorphData,
    MorphGroup,
    MorphMaterial,
    MorphType,
    MorphUV,
    MorphUV1,
    MorphUV2,
    MorphUV3,
    MorphUV4,
    MorphVertex,
    PMXBone,
    PMXBoneFlagBits,
    PMXDisplayFrame,
    PmxHeader,
    PMXJoint,
    PmxMaterial,
    PmxModel,
    PMXMorph,
    PMXRigidBody,
    PmxVertex,
    Vec4,
    VertexWeight,
    WeightType,
)
from mmd.core.pmx_reader import PMXConfig, PMXReader

log = logging.getLogger(__name__)


class PMXParseError(Exception):
    """Base exception for PMX parsing failures."""


class PMXHeaderParseError(PMXParseError):
    """Exception raised for errors in parsing PMX header data."""


class PMXVertexParseError(PMXParseError):
    """Exception raised for errors in parsing PMX vertex data."""


class PMXIndexParseError(PMXParseError):
    """Exception raised for errors in parsing PMX index data."""


class PMXTextureParseError(PMXParseError):
    """Exception raised for errors in parsing PMX texture data."""


class PMXMaterialParseError(PMXParseError):
    """Exception raised for errors in parsing PMX material data."""


class PMXBoneParseError(PMXParseError):
    """Exception raised for errors in parsing PMX bone data."""


class PMXMorphParseError(PMXParseError):
    """Exception raised for errors in parsing PMX morph data."""


class PMXDisplayFrameParseError(PMXParseError):
    """Exception raised for errors in parsing PMX display frame data."""


class PMXJointParseError(PMXParseError):
    """Exception raised for errors in parsing PMX joint data."""


class PMXRigidBodyParseError(PMXParseError):
    """Exception raised for errors in parsing PMX rigid body data."""


def read_pmx_header(reader: BinaryReader) -> PmxHeader:
    """
    Reads and validates the PMX header. Raises descriptive errors on invalid or unexpected data.
    """
    try:
        # Read magic (4 bytes)
        magic = b""
        for _ in range(4):
            magic += bytes([reader.read_uint8()])
        if magic != b"PMX ":
            raise PMXHeaderParseError(
                f"Invalid PMX file: incorrect magic number {magic!r}"
            )

        # Read version (float)
        version = reader.read_float()
        if not (2.0 <= version <= 2.1):
            raise PMXHeaderParseError(f"Unsupported PMX version: {version}")

        if version == 2.1:
            # TODO: add full support for PMX 2.1 features
            log.warning("PMX version %s may have limited support.", version)

        # Read global count (uint8)
        global_count = reader.read_uint8()
        if global_count != 8:
            raise PMXHeaderParseError(
                f"Unexpected global count: {global_count} (expected 8)"
            )

        # Read globals (list of uint8)
        globals_list: list[int] = []
        for _ in range(global_count):
            globals_list.append(reader.read_uint8())
        if len(globals_list) != 8:
            raise PMXHeaderParseError(
                f"Globals list length mismatch: {len(globals_list)}"
            )

        # Read model names and comments (local and universal)
        is_utf8 = globals_list[0] == 1
        model_name_local = reader.read_text(is_utf8)
        model_name_universal = reader.read_text(is_utf8)
        model_comment_local = reader.read_text(is_utf8)
        model_comment_universal = reader.read_text(is_utf8)

        pmx_header = PmxHeader(
            magic=magic,
            version=version,
            global_count=global_count,
            globals=globals_list,
            model_name_local=model_name_local,
            model_name_universal=model_name_universal,
            model_comment_local=model_comment_local,
            model_comment_universal=model_comment_universal,
        )
        return pmx_header
    except PMXHeaderParseError:
        raise
    except Exception as e:
        raise PMXHeaderParseError(f"Failed to read PMX header: {e}") from e


def create_pmx_reader(reader: BinaryReader, header: PmxHeader) -> PMXReader:
    """
    Creates a PMXReader instance using the provided BinaryReader and PMXHeader.

    Args:
        reader (BinaryReader): The binary reader positioned after the PMX header.
        header (PmxHeader): The PMX header information.
    """
    config = PMXConfig(header)
    pmx_reader = PMXReader(reader, config)
    return pmx_reader


def _read_single_vertex(
    reader: PMXReader, additional_uv_count: int, i: int
) -> PmxVertex:
    try:
        position = reader.readVec3()
        normal = reader.readVec3()
        uv = reader.readVec2()

        # Read additional UVs
        additional_uvs: list[Vec4] = []
        for _ in range(additional_uv_count):
            additional_uvs.append(reader.readVec4())

        # Read weight deform type
        weight_deform_type: WeightType = reader.readWeightType()
        weight_bones: VertexWeight = VertexWeight()
        if weight_deform_type == WeightType.BDEF1:
            weight_bones.bone_index.append(reader.readBoneIndex())
            weight_bones.weight.append(1.0)
        elif weight_deform_type == WeightType.BDEF2:
            weight_bones.bone_index.append(reader.readBoneIndex())
            weight_bones.bone_index.append(reader.readBoneIndex())
            w = reader.readFloat()
            weight_bones.weight.append(w)
            weight_bones.weight.append(1.0 - w)
        elif weight_deform_type == WeightType.BDEF4:
            for _ in range(4):
                weight_bones.bone_index.append(reader.readBoneIndex())
            for _ in range(4):
                w = reader.readFloat()
                weight_bones.weight.append(w)
        elif weight_deform_type == WeightType.SDEF:
            weight_bones.bone_index.append(reader.readBoneIndex())
            weight_bones.bone_index.append(reader.readBoneIndex())
            w = reader.readFloat()
            weight_bones.weight.append(w)
            weight_bones.weight.append(1.0 - w)
            # Read SDEF parameters (C, R0, R1)
            reader.readVec3()
            reader.readVec3()
            reader.readVec3()
        elif weight_deform_type == WeightType.QDEF:
            for _ in range(4):
                weight_bones.bone_index.append(reader.readBoneIndex())
            for _ in range(4):
                w = reader.readFloat()
                weight_bones.weight.append(w)
        else:
            raise PMXVertexParseError(
                f"Unsupported weight deform type: {weight_deform_type}"
            )

        # Read edge scale
        edge_scale = reader.readFloat()

        return PmxVertex(
            position=position,
            normal=normal,
            uv=uv,
            additional_uvs=additional_uvs,
            weight_type=weight_deform_type,
            weight=weight_bones,
            edge_scale=edge_scale,
        )
    except Exception as ve:
        raise PMXVertexParseError(f"Error parsing vertex {i}: {ve}") from ve


def read_vertex_data(reader: PMXReader) -> list[PmxVertex]:
    """
    Reads all vertex data.
    Raises PMXVertexParseError on any issues that make parsing unreliable
    """
    try:
        vertex_count = reader.readInt32()
        if vertex_count < 0 or vertex_count > 10**7:
            raise PMXVertexParseError(f"Unreasonable vertex count: {vertex_count}")

        additional_uv_count = reader.config.additionalUVCount
        if additional_uv_count < 0 or additional_uv_count > 4:
            raise PMXVertexParseError(
                f"Invalid additional UV count: {additional_uv_count}"
            )

        vertices: list[PmxVertex] = []
        for i in range(vertex_count):
            vertex = _read_single_vertex(reader, additional_uv_count, i)
            vertices.append(vertex)
        return vertices
    except PMXVertexParseError:
        raise
    except Exception as e:
        raise PMXVertexParseError(f"Failed to read vertex data: {e}") from e


def read_indices_data(reader: PMXReader) -> list[int]:
    """
    Reads all index data. Raises PMXIndexParseError on any issues that make parsing unreliable.
    """
    try:
        index_count = reader.readInt32()
        if index_count < 0 or index_count > 10**8:
            raise PMXIndexParseError(f"Unreasonable index count: {index_count}")
        indices: list[int] = []
        for _ in range(index_count):
            indices.append(reader.readVertexIndex())
        return indices
    except PMXIndexParseError:
        raise
    except Exception as e:
        raise PMXIndexParseError(f"Failed to read indices data: {e}") from e


def read_texture_data(reader: PMXReader) -> list[str]:
    """
    Reads all texture paths.
    Raises PMXTextureParseError on any issues that make parsing unreliable.
    """
    try:
        textures: list[str] = []
        texture_count = (
            reader.readUInt32()
        )  # TODO: validate if this is signed or unsigned
        for _ in range(texture_count):
            textures.append(reader.readText())
        return textures
    except PMXTextureParseError:
        raise
    except Exception as e:
        raise PMXTextureParseError(f"Failed to read texture data: {e}") from e


def read_material_data(reader: PMXReader) -> list[PmxMaterial]:
    try:
        materials: list[PmxMaterial] = []
        material_count = reader.readUInt32()
        for _ in range(material_count):
            name_local = reader.readText()
            name_universal = reader.readText()
            diffuse_color = reader.readVec4()
            specular_color = reader.readVec3()
            specular_strength = reader.readFloat()
            ambient_color = reader.readVec3()
            draw_flag = reader.readMaterialFlagBits()
            edge_color = reader.readVec4()
            edge_size = reader.readFloat()
            texture_index = reader.readTextureIndex()
            sphere_texture_index = reader.readTextureIndex()
            sphere_mode = reader.readInt8()
            toon_flag = reader.readInt8()

            if toon_flag == 0:
                toon_value = reader.readTextureIndex()
            else:
                toon_value = reader.readInt8()

            meta_data = reader.readText()
            face_vertex_count = reader.readUInt32()
            material = PmxMaterial(
                name_local=name_local,
                name_universal=name_universal,
                diffuse_color=diffuse_color,
                specular_color=specular_color,
                specular_strength=specular_strength,
                ambient_color=ambient_color,
                draw_flag=draw_flag,
                edge_color=edge_color,
                edge_size=edge_size,
                texture_index=texture_index,
                sphere_texture_index=sphere_texture_index,
                sphere_mode=sphere_mode,
                toon_flag=toon_flag,
                toon_value=toon_value,
                meta_data=meta_data,
                face_vertex_count=face_vertex_count,
            )
            materials.append(material)

        return materials
    except PMXMaterialParseError:
        raise
    except Exception as e:
        raise PMXMaterialParseError(f"Failed to read material data: {e}") from e


def read_bones_data(reader: PMXReader) -> list[PMXBone]:
    try:
        bones: list[PMXBone] = []
        bone_count = reader.readInt32()
        for _ in range(bone_count):
            name_local = reader.readText()
            name_universal = reader.readText()
            position = reader.readVec3()
            parent_index = reader.readBoneIndex()
            level = reader.readInt32()
            bone_flags = reader.readBoneFlagBits()

            # Tail info
            if bone_flags & PMXBoneFlagBits.INDEXED_TAIL_POSITION:
                tail_info = reader.readBoneIndex()
            else:
                tail_info = reader.readVec3()

            # Inherit Bone
            inherit_bone: InheritBone | None = None
            if (
                bone_flags & PMXBoneFlagBits.INHERIT_ROTATION
                or bone_flags & PMXBoneFlagBits.INHERIT_TRANSLATION
            ):
                inherit_bone = InheritBone(
                    parentBoneIndex=reader.readBoneIndex(),
                    influenceFactor=reader.readFloat(),
                )

            # Fixed Axis
            fixed_axis: FixedAxis | None = None
            if bone_flags & PMXBoneFlagBits.FIXED_AXIS:
                fixed_axis = FixedAxis(axis=reader.readVec3())

            # Local Coordinates
            local_coordinate: BoneLocalCoordinate | None = None
            if bone_flags & PMXBoneFlagBits.LOCAL_COORDINATE:
                local_coordinate = BoneLocalCoordinate(
                    xAxis=reader.readVec3(),
                    zAxis=reader.readVec3(),
                )

            # External Parent Deform
            external_parent: BoneExternalParent | None = None
            if bone_flags & PMXBoneFlagBits.EXTERNAL_PARENT_DEFORM:
                external_parent = BoneExternalParent(
                    parentBoneIndex=reader.readBoneIndex()
                )

            # IK
            ik_obj: IK | None = None
            if bone_flags & PMXBoneFlagBits.IK:
                ik_target_bone_index = reader.readBoneIndex()
                ik_loop_count = reader.readInt32()
                ik_limit_radian = reader.readFloat()

                ik_link_count = reader.readInt32()
                ik_links: list[IKLink] = []
                for _ in range(ik_link_count):
                    ik_link_bone_index = reader.readBoneIndex()
                    ik_has_limit = reader.readInt8()
                    if ik_has_limit:
                        ik_lower_limit = reader.readVec3()
                        ik_upper_limit = reader.readVec3()
                    else:
                        ik_lower_limit = None
                        ik_upper_limit = None
                    ik_links.append(
                        IKLink(
                            boneIndex=ik_link_bone_index,
                            rotationLimitMin=ik_lower_limit,
                            rotationLimitMax=ik_upper_limit,
                        )
                    )
                ik_obj = IK(
                    targetBoneIndex=ik_target_bone_index,
                    loopCount=ik_loop_count,
                    limitRadian=ik_limit_radian,
                    links=ik_links,
                )

            bone = PMXBone(
                nameLocal=name_local,
                nameUniversal=name_universal,
                position=position,
                parentIndex=parent_index,
                level=level,
                flags=bone_flags,
                tailInfo=tail_info,
                inheritBone=inherit_bone,
                fixedAxis=fixed_axis,
                localCoordinate=local_coordinate,
                externalParent=external_parent,
                ik=ik_obj,
            )

            bones.append(bone)

        return bones
    except PMXBoneParseError:
        raise
    except Exception as e:
        raise PMXBoneParseError(f"Failed to read bone data: {e}") from e


def read_morphs_data(reader: PMXReader) -> list[PMXMorph]:
    """
    Reads all morph data, ensuring validity and consistency. Raises errors on invalid morphs.
    """
    try:
        morphs: list[PMXMorph] = []
        morph_count = reader.readUInt32()
        for _ in range(morph_count):
            # Implement morph reading here
            name_local = reader.readText()
            name_universal = reader.readText()
            panel_type = reader.readInt8()
            morph_type = reader.readMorphType()

            element_count = reader.readInt32()
            morph_data: MorphData
            if morph_type == MorphType.GROUP:  # Group Morph
                group_elements: list[MorphGroup] = []
                for _ in range(element_count):
                    morph_index = reader.readMorphIndex()
                    weight = reader.readFloat()
                    group_elements.append(
                        MorphGroup(morph_index=morph_index, morph_value=weight)
                    )
                morph_data = group_elements

            elif morph_type == MorphType.VERTEX:  # Vertex Morph
                vertex_elements: list[MorphVertex] = []
                for _ in range(element_count):
                    vertex_index = reader.readVertexIndex()
                    position_offset = reader.readVec3()
                    vertex_elements.append(
                        MorphVertex(vertex_index=vertex_index, offset=position_offset)
                    )
                morph_data = vertex_elements

            elif morph_type == MorphType.BONE:  # Bone Morph
                bone_elements: list[MorphBone] = []
                for _ in range(element_count):
                    bone_index = reader.readBoneIndex()
                    position_offset = reader.readVec3()
                    rotation_offset = reader.readVec4()
                    bone_elements.append(
                        MorphBone(
                            bone_index=bone_index,
                            position_offset=position_offset,
                            rotation_offset=rotation_offset,
                        )
                    )
                morph_data = bone_elements

            elif morph_type == MorphType.UV:  # UV Morph
                uv_elements: list[MorphUV] = []
                for _ in range(element_count):
                    vertex_index = reader.readVertexIndex()
                    uv_offset = reader.readVec4()
                    uv_elements.append(
                        MorphUV(vertex_index=vertex_index, offset=uv_offset)
                    )
                morph_data = uv_elements

            elif morph_type == MorphType.UV1:  # UV1 Morph
                uv1_elements: list[MorphUV1] = []
                for _ in range(element_count):
                    vertex_index = reader.readVertexIndex()
                    uv_offset = reader.readVec4()
                    uv1_elements.append(
                        MorphUV1(vertex_index=vertex_index, offset=uv_offset)
                    )
                morph_data = uv1_elements

            elif morph_type == MorphType.UV2:  # UV2 Morph
                uv2_elements: list[MorphUV2] = []
                for _ in range(element_count):
                    vertex_index = reader.readVertexIndex()
                    uv_offset = reader.readVec4()
                    uv2_elements.append(
                        MorphUV2(vertex_index=vertex_index, offset=uv_offset)
                    )
                morph_data = uv2_elements

            elif morph_type == MorphType.UV3:  # UV3 Morph
                uv3_elements: list[MorphUV3] = []
                for _ in range(element_count):
                    vertex_index = reader.readVertexIndex()
                    uv_offset = reader.readVec4()
                    uv3_elements.append(
                        MorphUV3(vertex_index=vertex_index, offset=uv_offset)
                    )
                morph_data = uv3_elements

            elif morph_type == MorphType.UV4:  # UV4 Morph
                uv4_elements: list[MorphUV4] = []
                for _ in range(element_count):
                    vertex_index = reader.readVertexIndex()
                    uv_offset = reader.readVec4()
                    uv4_elements.append(
                        MorphUV4(vertex_index=vertex_index, offset=uv_offset)
                    )
                morph_data = uv4_elements

            elif morph_type == MorphType.MATERIAL:  # Material Morph
                material_elements: list[MorphMaterial] = []
                for _ in range(element_count):
                    material_index = reader.readMaterialIndex()
                    offset_operation = reader.readInt8()
                    diffuse = reader.readVec4()
                    specular = reader.readVec3()
                    specular_power = reader.readFloat()
                    ambient = reader.readVec3()
                    edge_color = reader.readVec4()
                    edge_size = reader.readFloat()
                    texture_tint = reader.readVec4()
                    sphere_texture_tint = reader.readVec4()
                    toon_texture_tint = reader.readVec4()
                    material_elements.append(
                        MorphMaterial(
                            material_index=material_index,
                            offset_operation=offset_operation,
                            diffuse=diffuse,
                            specular=specular,
                            specular_power=specular_power,
                            ambient=ambient,
                            edge_color=edge_color,
                            edge_size=edge_size,
                            texture_tint=texture_tint,
                            sphere_texture_tint=sphere_texture_tint,
                            toon_texture_tint=toon_texture_tint,
                        )
                    )
                morph_data = material_elements

            else:
                raise PMXMorphParseError(f"Unknown morph type: {morph_type}")

            morph = PMXMorph(
                name_local=name_local,
                name_universal=name_universal,
                panel_type=panel_type,
                morph_type=morph_type,
                data=morph_data,
            )

            morphs.append(morph)

        return morphs
    except PMXMorphParseError:
        raise
    except Exception as e:
        raise PMXMorphParseError(f"Failed to read morph data: {e}") from e


def read_display_frames_data(reader: PMXReader) -> list[PMXDisplayFrame]:
    try:
        display_frame_count = reader.readInt32()
        display_frames: list[PMXDisplayFrame] = []
        for _ in range(display_frame_count):
            local_name = reader.readText()
            universal_name = reader.readText()

            special_flag = reader.readInt8()
            frame_element_count = reader.readInt32()
            frame_elements: list[FrameData] = []
            for _ in range(frame_element_count):
                element_type: FrameType = reader.readFrameType()
                if element_type is FrameType.BONE:
                    element_data: BoneFrameElement | MorphFrameElement = (
                        BoneFrameElement(bone_index=reader.readBoneIndex())
                    )
                elif element_type is FrameType.MORPH:
                    element_data = MorphFrameElement(
                        morph_index=reader.readMorphIndex()
                    )
                else:
                    raise PMXDisplayFrameParseError(
                        f"Invalid display frame element type: {element_type}"
                    )
                frame_elements.append(
                    FrameData(frame_type=element_type, data=element_data)
                )

            display_frame = PMXDisplayFrame(
                name_local=local_name,
                name_universal=universal_name,
                special_flag=special_flag,
                frame_elements=frame_elements,
            )
            display_frames.append(display_frame)

        return display_frames
    except PMXDisplayFrameParseError:
        raise
    except Exception as e:
        raise PMXDisplayFrameParseError(
            f"Failed to read display frame data: {e}"
        ) from e


def read_joint_data(reader: PMXReader) -> list[PMXJoint]:
    try:
        joints: list[PMXJoint] = []
        joint_count = reader.readInt32()
        for _ in range(joint_count):
            name_local = reader.readText()
            name_universal = reader.readText()

            joint_type = reader.readJointType()

            rigid_body_index_a = reader.readRigidBodyIndex()
            rigid_body_index_b = reader.readRigidBodyIndex()
            position = reader.readVec3()
            rotation = reader.readVec3()
            movement_limit_min = reader.readVec3()
            movement_limit_max = reader.readVec3()
            rotation_limit_min = reader.readVec3()
            rotation_limit_max = reader.readVec3()
            spring_position_factor = reader.readVec3()
            spring_rotation_factor = reader.readVec3()

            joint = PMXJoint(
                name_local=name_local,
                name_universal=name_universal,
                type=joint_type,
                rigid_body_index_a=rigid_body_index_a,
                rigid_body_index_b=rigid_body_index_b,
                position=position,
                rotation=rotation,
                position_min=movement_limit_min,
                position_max=movement_limit_max,
                rotation_min=rotation_limit_min,
                rotation_max=rotation_limit_max,
                position_spring_constant=spring_position_factor,
                rotation_spring_constant=spring_rotation_factor,
            )
            joints.append(joint)

        return joints
    except PMXJointParseError:
        raise
    except Exception as e:
        raise PMXJointParseError(f"Failed to read joint data: {e}") from e


def read_rigid_body_data(reader: PMXReader) -> list[PMXRigidBody]:
    try:
        pmx_rigid_bodies: list[PMXRigidBody] = []
        rigid_body_count = reader.readInt32()
        for _ in range(rigid_body_count):
            name_local = reader.readText()
            name_universal = reader.readText()

            bone_index = reader.readBoneIndex()
            group = reader.readInt8()
            non_collision_group = reader.readInt16()
            shape_type = reader.readShapeType()
            shape_size = reader.readVec3()
            shape_position = reader.readVec3()
            shape_rotation = reader.readVec3()
            mass = reader.readFloat()
            linear_damping = reader.readFloat()
            angular_damping = reader.readFloat()
            restitution = reader.readFloat()
            friction = reader.readFloat()
            physics_mode = reader.readPhysicsMode()

            rigid_body = PMXRigidBody(
                name_local=name_local,
                name_universal=name_universal,
                related_bone_index=bone_index,
                group_id=group,
                non_collision_group=non_collision_group,
                shape=shape_type,
                shape_size=shape_size,
                shape_position=shape_position,
                shape_rotation=shape_rotation,
                mass=mass,
                move_attenuation=linear_damping,
                rotation_damping=angular_damping,
                repulsion=restitution,
                friction_force=friction,
                physics_mode=physics_mode,
            )

            pmx_rigid_bodies.append(rigid_body)
        return pmx_rigid_bodies
    except PMXRigidBodyParseError:
        raise
    except Exception as e:
        raise PMXRigidBodyParseError(f"Failed to read rigid body data: {e}") from e


def parse_pmx(file_path: str) -> PmxModel:
    """
    Parses a PMX file, returning a PmxModel. Handles file I/O and parsing errors robustly.
    """
    try:
        with open(file_path, "rb") as file:
            data = file.read()
    except Exception as e:
        raise PMXHeaderParseError(f"Failed to open PMX file '{file_path}': {e}") from e
    log.debug("Opened PMX file: %s", file_path)

    # Get absolute path
    model_name = os.path.basename(file_path)
    file_path = os.path.abspath(file_path)
    absolute_path = os.path.dirname(file_path)
    log.debug("Absolute path: %s", absolute_path)
    try:
        # Reading pass
        reader = BinaryReader(data)
        header = read_pmx_header(reader)
        pmx_reader = create_pmx_reader(reader, header)
        # Read all PMX data sections
        vertex_data = read_vertex_data(pmx_reader)
        indices_data = read_indices_data(pmx_reader)
        textures_data = read_texture_data(pmx_reader)
        material_data = read_material_data(pmx_reader)
        bones_data = read_bones_data(pmx_reader)
        morphs_data = read_morphs_data(pmx_reader)
        display_frames_data = read_display_frames_data(pmx_reader)
        rigid_bodies_data = read_rigid_body_data(pmx_reader)
        joints_data = read_joint_data(pmx_reader)

        # Validation pass
        # Check if we have read all data
        if not pmx_reader.isAtEnd():
            log.warning(
                "Not all data was read from PMX file '%s'. Remaining bytes: %d",
                file_path,
                pmx_reader.reader.remaining(),
            )

        pmx_data = PmxModel(
            model_name=model_name,
            file_path=file_path,
            absolute_path=absolute_path,
            header=header,
            vertices=vertex_data,
            indices=indices_data,
            textures_paths=textures_data,
            materials=material_data,
            bones=bones_data,
            morphs=morphs_data,
            display_frames=display_frames_data,
            rigid_bodies=rigid_bodies_data,
            joints=joints_data,
        )

        return pmx_data
    except PMXParseError as e:
        raise PMXParseError(f"Failed to parse PMX file '{file_path}': {e}") from e
    except Exception as e:
        raise PMXParseError(
            f"Unexpected error while parsing PMX file '{file_path}': {e}"
        ) from e


# --- JSON serialization helper for custom types ---
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
        seq = cast("list[object] | tuple[object, ...]", obj)
        return [serialize_for_json(v) for v in seq]
    elif isinstance(obj, (int, float, str, type(None))):
        return obj
    elif hasattr(obj, "__dict__"):
        # Avoid recursing into class/type objects
        if isinstance(obj, type):
            return str(obj)
        fields: dict[str, object] = obj.__dict__
        return {k: serialize_for_json(v) for k, v in fields.items()}
    else:
        return str(obj)


def dump_pmx_to_json(pmx_data: PmxModel, output_path: str):
    """
    Dumps the entire PMX data to a JSON files at the specified output path.
    Each major component is saved in a separate JSON file.
    """
    os.makedirs(output_path, exist_ok=True)
    # Header data
    header_json_path = os.path.join(output_path, "header.json")
    try:
        log.debug("Saving header data to JSON: %s", header_json_path)
        with open(header_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                serialize_for_json(pmx_data.header),
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save header data to JSON: %s", e)

    # Vertexes data
    vertices_json_path = os.path.join(output_path, "vertices.json")
    try:
        log.debug("Saving vertices data to JSON: %s", vertices_json_path)
        with open(vertices_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(vertex) for vertex in pmx_data.vertices],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save vertices data to JSON: %s", e)
    # Indices data
    indices_json_path = os.path.join(output_path, "indices.json")
    try:
        log.debug("Saving indices data to JSON: %s", indices_json_path)
        with open(indices_json_path, "w", encoding="utf-8") as json_file:
            json.dump(pmx_data.indices, json_file, ensure_ascii=False, indent=4)
    except Exception as e:
        log.warning("Failed to save indices data to JSON: %s", e)
    # Additional components can be dumped similarly if needed
    # Textures data
    textures_json_path = os.path.join(output_path, "textures.json")
    try:
        log.debug("Saving textures data to JSON: %s", textures_json_path)
        with open(textures_json_path, "w", encoding="utf-8") as json_file:
            json.dump(pmx_data.textures_paths, json_file, ensure_ascii=False, indent=4)
    except Exception as e:
        log.warning("Failed to save textures data to JSON: %s", e)
    # Materials data
    materials_json_path = os.path.join(output_path, "materials.json")
    try:
        log.debug("Saving materials data to JSON: %s", materials_json_path)
        with open(materials_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(material) for material in pmx_data.materials],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save materials data to JSON: %s", e)
    # Bones data
    bones_json_path = os.path.join(output_path, "bones.json")
    try:
        log.debug("Saving bones data to JSON: %s", bones_json_path)
        with open(bones_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(bone) for bone in pmx_data.bones],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save bones data to JSON: %s", e)
    # Morphs data
    morphs_json_path = os.path.join(output_path, "morphs.json")
    try:
        log.debug("Saving morphs data to JSON: %s", morphs_json_path)
        with open(morphs_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(morph) for morph in pmx_data.morphs],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save morphs data to JSON: %s", e)

    # Display Frames data
    display_frames_json_path = os.path.join(output_path, "display_frames.json")
    try:
        log.debug("Saving display frames data to JSON: %s", display_frames_json_path)
        with open(display_frames_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(frame) for frame in pmx_data.display_frames],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save display frames data to JSON: %s", e)

    # Rigid Bodies data
    rigid_bodies_json_path = os.path.join(output_path, "rigid_bodies.json")
    try:
        log.debug("Saving rigid bodies data to JSON: %s", rigid_bodies_json_path)
        with open(rigid_bodies_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(rb) for rb in pmx_data.rigid_bodies],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save rigid bodies data to JSON: %s", e)

    # Joints data
    joints_json_path = os.path.join(output_path, "joints.json")
    try:
        log.debug("Saving joints data to JSON: %s", joints_json_path)
        with open(joints_json_path, "w", encoding="utf-8") as json_file:
            json.dump(
                [serialize_for_json(joint) for joint in pmx_data.joints],
                json_file,
                ensure_ascii=False,
                indent=4,
            )
    except Exception as e:
        log.warning("Failed to save joints data to JSON: %s", e)


# --- End of JSON serialization helper ---
