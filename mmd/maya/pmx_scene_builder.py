import logging
import os
import traceback

import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
import maya.cmds as cmds

# Import after path is set
from mmd.core.data_types import PmxModel
from mmd.maya.maya_data_types import MayaPmxData
from mmd.maya.pmx.bone_builder import create_bones_from_pmx_bones  # noqa: F401
from mmd.maya.pmx.rigid_body_builder import create_physics_from_pmx_data  # noqa: F401
from mmd.maya.pmx.morph_builder import create_blendshapes_from_pmx_data  # noqa: F401
from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


def is_valid_pmx_model_name(name: str) -> bool:
    """
    Checks if the given PMX model name is valid for use in Maya.
    Maya can't handle not ascii characters in object names and some special characters.

    Args:
        name (str): The PMX model name to check.
    Returns:
        bool: True if valid, False otherwise.
    """

    if not name:
        return False

    # Check for non-ascii characters
    try:
        name.encode("ascii")
    except UnicodeEncodeError:
        return False

    # TODO: Maya change spaces to underscores, do we want to allow spaces?

    # Check for special characters not allowed in Maya names
    invalid_chars = set(r'\/:*?"<>|')
    if any(char in invalid_chars for char in name):
        return False

    return True


def create_root_node_for_pmx_model(
    pmx_data: PmxModel, name_registry: PMXNamingManager
) -> om.MObject:
    """
    Creates a root transform node for the PMX model.

    Args:
        pmx_model_name (str, optional): Name of the PMX model. Defaults to None.

    Returns:
        MObject: The created root transform object.
    """
    # Create root transform node using MFnTransform
    root_transform_fn = om.MFnTransform()
    root_transform_obj = root_transform_fn.create()

    # Rename the transform node
    root_transform_fn.setName(name_registry.get_root_name())

    return root_transform_obj


def create_mesh_nodes_from_pmx_data(
    pmx_data: PmxModel, root_transform_obj, name_registry: PMXNamingManager
) -> dict:
    """
    Creates mesh nodes from PMX data.

    Args:
        pmx_data (dict): PMX data containing vertices and indices
            - pmx_vertices: list of dicts with position, normal, uv
            - indices: list of triangle indices
        root_transform_obj (MObject): Root transform object
        pmx_model_name (str, optional): Name of the PMX model. Defaults to None.

    Returns:
        dict: Dictionary containing created Maya objects
    """
    maya_data = {
        "root_obj": root_transform_obj,
        "mesh_node": None,
        "mesh_dag_path": None,
    }

    geo_group_transform_fn = om.MFnTransform()
    geo_group_obj = geo_group_transform_fn.create(root_transform_obj)
    geo_group_transform_fn.setName(name_registry.get_geo_group_name())

    # Prepare mesh data
    vertices = []
    normals = om.MVectorArray()
    u_array = []
    v_array = []

    # Extract vertex data (flip Z and V coordinates as in C++ code)
    for vertex in pmx_data.vertices:
        # Position (flip Z)
        pos = vertex.position
        vertices.append(om.MPoint(pos.x, pos.y, -pos.z))

        # Normal (flip Z)
        normal = vertex.normal
        normals.append(om.MVector(normal.x, normal.y, -normal.z))

        # UV (flip V)
        uv = vertex.uv
        u_array.append(float(uv.x))
        v_array.append(float(1.0 - uv.y))

    # Create face connectivity array
    # When flipping Z-axis, we need to reverse triangle winding order
    # to keep normals facing outward
    face_connects = []
    for i in range(0, len(pmx_data.indices), 3):
        # Reverse winding order: [A, B, C] -> [A, C, B]
        face_connects.append(int(pmx_data.indices[i]))
        face_connects.append(int(pmx_data.indices[i + 2]))  # Swap B and C
        face_connects.append(int(pmx_data.indices[i + 1]))

    # Calculate number of faces
    num_faces = len(pmx_data.indices) // 3

    # Create mesh transform
    mesh_transform_fn = om.MFnTransform()
    mesh_transform_obj = mesh_transform_fn.create(geo_group_obj)

    # Set mesh name
    mesh_transform_fn.setName(name_registry.get_mesh_name())

    # Create face counts array (all triangles)
    face_counts = []
    for _ in range(num_faces):
        face_counts.append(int(3))

    # Create the mesh
    mesh_fn = om.MFnMesh()

    try:
        # Create the mesh with vertices and faces
        mesh_obj = mesh_fn.create(
            vertices,
            face_counts,
            face_connects,
            u_array,
            v_array,
            mesh_transform_obj,
        )

        if not mesh_obj.isNull():
            # Set mesh shape name
            mesh_fn.setName(name_registry.get_shape_name())

            # Store in maya_data
            maya_data["mesh_node"] = mesh_obj

            # Get DAG path
            dag_path = om.MDagPath.getAPathTo(mesh_obj)
            maya_data["mesh_dag_path"] = dag_path

            # Set normals
            vertex_list = om.MIntArray([i for i in range(len(vertices))])
            mesh_fn.setVertexNormals(normals, vertex_list)

            # Set UVs
            uv_set_name = "map1"  # TODO: do we need to care about UV set names?
            mesh_fn.setUVs(u_array, v_array, uvSet=uv_set_name)

            # Assign UVs to vertices - need to create a copy of face_connects
            uv_ids = om.MIntArray(face_connects)  # Create a copy for UVs
            mesh_fn.assignUVs(face_counts, uv_ids, uvSet=uv_set_name)

            # Force update of the mesh to ensure proper display
            mesh_fn.updateSurface()

        else:
            log.error("Failed to create mesh for PMX model.")

    except Exception as e:
        log.error("Error creating mesh: %s", e)
        log.debug(traceback.format_exc())

    return maya_data


def create_materials_from_pmx_materials(
    pmx_data: PmxModel, name_registry: PMXNamingManager
) -> list:
    """
    Creates Maya OpenPBRSurface shaders and shading groups from PMX materials.
    Args:
        pmx_materials (list): List of PmxMaterial objects.
        pmx_textures (list): List of texture file names (optional).
        pmx_base_path (str): Base path for textures (optional).
    Returns:
        list: List of (shader_obj, shading_group_obj) tuples.
    """
    # Create texture nodes for all textures first, so they can be reused
    texture_nodes = {}
    place2d_nodes = {}
    for tex_idx, tex_path in enumerate(pmx_data.textures_paths):
        full_tex_path = os.path.join(pmx_data.absolute_path, tex_path)
        if os.path.isfile(full_tex_path):
            texture_name = name_registry.get_texture_name(tex_idx)
            file_node = cmds.shadingNode(
                "file", asTexture=True, isColorManaged=True, name=texture_name
            )
            place2d_node_name = name_registry.get_place2d_name(tex_idx)
            place2d_node = cmds.shadingNode(
                "place2dTexture", asUtility=True, name=place2d_node_name
            )
            cmds.setAttr(f"{file_node}.fileTextureName", full_tex_path, type="string")
            try:
                cmds.setAttr(f"{file_node}.alphaIsLuminance", 0)
            except Exception as e:
                log.warning("Could not set alphaIsLuminance for %s: %s", file_node, e)
            tex_attrs = [
                ("coverage", "coverage"),
                ("translateFrame", "translateFrame"),
                ("rotateFrame", "rotateFrame"),
                ("mirrorU", "mirrorU"),
                ("mirrorV", "mirrorV"),
                ("stagger", "stagger"),
                ("wrapU", "wrapU"),
                ("wrapV", "wrapV"),
                ("repeatUV", "repeatUV"),
                ("offset", "offset"),
                ("rotateUV", "rotateUV"),
                ("noiseUV", "noiseUV"),
                ("vertexUvOne", "vertexUvOne"),
                ("vertexUvTwo", "vertexUvTwo"),
                ("vertexUvThree", "vertexUvThree"),
                ("vertexCameraOne", "vertexCameraOne"),
            ]
            for src, dest in tex_attrs:
                try:
                    cmds.connectAttr(
                        f"{place2d_node}.{src}", f"{file_node}.{dest}", force=True
                    )
                except Exception as e:
                    log.warning(
                        "Could not connect %s.%s to %s.%s: %s",
                        place2d_node,
                        src,
                        file_node,
                        dest,
                        e,
                    )
            try:
                cmds.connectAttr(
                    f"{place2d_node}.outUV", f"{file_node}.uvCoord", force=True
                )
            except Exception as e:
                log.warning(
                    "Could not connect %s.outUV to %s.uvCoord: %s",
                    place2d_node,
                    file_node,
                    e,
                )
            try:
                cmds.connectAttr(
                    f"{place2d_node}.outUvFilterSize",
                    f"{file_node}.uvFilterSize",
                    force=True,
                )
            except Exception as e:
                log.warning(
                    "Could not connect %s.outUvFilterSize to %s.uvFilterSize: %s",
                    place2d_node,
                    file_node,
                    e,
                )
            texture_nodes[tex_idx] = file_node
            place2d_nodes[tex_idx] = place2d_node
        elif os.path.isdir(full_tex_path):
            log.debug("Texture path is a directory, skipping: %s", full_tex_path)
        else:
            log.warning("Texture file does not exist, skipping: %s", full_tex_path)

    sg_obj_list = []
    for idx, pmx_mat in enumerate(pmx_data.materials):
        mat_name = name_registry.get_material_name(idx)
        log.debug("Creating material: %s", mat_name)
        shading_node = cmds.shadingNode(
            "openPBRSurface", asShader=True, name=f"{mat_name}"
        )
        shading_node = cmds.ls(shading_node)[0]
        shading_group = name_registry.get_shading_group_name(idx)
        cmds.sets(renderable=True, noSurfaceShader=True, empty=True, name=shading_group)
        cmds.connectAttr(
            f"{shading_node}.outColor", f"{shading_group}.surfaceShader", force=True
        )
        cmds.setAttr(
            f"{shading_node}.baseColor",
            float(pmx_mat.diffuse_color.x),
            float(pmx_mat.diffuse_color.y),
            float(pmx_mat.diffuse_color.z),
            type="float3",
        )
        # PMX specular_strength is specified as a 0-1 intensity, but many models
        # (e.g. YYB, Lyfe, Endmin) store out-of-range values (5, 30, 100, ...).
        # openPBRSurface.specularWeight is strictly 0-1 — values > 1 make the
        # surface render dark/black in the viewport, so clamp before assigning.
        spec_weight = max(0.0, min(pmx_mat.specular_strength, 1.0))
        cmds.setAttr(f"{shading_node}.specularWeight", spec_weight)
        cmds.setAttr(
            f"{shading_node}.specularColor",
            pmx_mat.specular_color.x,
            pmx_mat.specular_color.y,
            pmx_mat.specular_color.z,
            type="float3",
        )
        if pmx_mat.diffuse_color.w < 1.0:
            cmds.setAttr(f"{shading_node}.geometryOpacity", pmx_mat.diffuse_color.w)
        if pmx_mat.texture_index >= 0 and pmx_mat.texture_index < len(
            pmx_data.textures_paths
        ):
            file_node = texture_nodes.get(pmx_mat.texture_index)
            if file_node:
                try:
                    cmds.connectAttr(
                        f"{file_node}.outColor", f"{shading_node}.baseColor", force=True
                    )
                except Exception as e:
                    log.warning(
                        "Could not connect %s.outColor to %s.baseColor: %s",
                        file_node,
                        shading_node,
                        e,
                    )
                material_alpha = pmx_mat.diffuse_color.w
                # Always set geometryOpacity for semi-transparent materials
                if material_alpha < 1.0:
                    cmds.setAttr(f"{shading_node}.geometryOpacity", material_alpha)
                # Connect outAlpha only if texture has alpha
                try:
                    texture_has_alpha = cmds.getAttr(f"{file_node}.fileHasAlpha")
                except Exception as e:
                    log.debug("Could not query fileHasAlpha for %s: %s", file_node, e)
                    texture_has_alpha = False

                if texture_has_alpha:
                    try:
                        cmds.connectAttr(
                            f"{file_node}.outAlpha",
                            f"{shading_node}.geometryOpacity",
                            force=True,
                        )
                        log.debug(
                            "Connected texture alpha to geometryOpacity for material: %s (alpha=%.2f)",
                            mat_name,
                            material_alpha,
                        )
                    except Exception as e:
                        log.warning(
                            "Could not connect %s.outAlpha to %s.geometryOpacity: %s",
                            file_node,
                            shading_node,
                            e,
                        )
                else:
                    log.debug(
                        "Texture has no alpha channel, skipping alpha connection for material: %s",
                        mat_name,
                    )
        sel_sg = om.MSelectionList()
        sel_sg.add(shading_group)
        sg_mobj = sel_sg.getDependNode(0)
        sg_obj_list.append(sg_mobj)
    return sg_obj_list


def assign_materials_to_mesh_faces(mesh_dag_path, pmx_materials, maya_materials):
    """
    Assigns each material's shading group to the correct faces on the mesh.

    PMX materials specify **contiguous** face ranges (via ``face_vertex_count``),
    so we can use Maya's range notation ``f[start:end]`` instead of expanding
    to one string per face — a 15 000× reduction in string allocations for
    typical models.

    Args:
        mesh_dag_path (MDagPath): The DAG path to the mesh shape or transform.
        pmx_materials (list): List of PMX material objects (must have .face_vertex_count).
        maya_materials (list): List of shading-group MObjects (same order as pmx_materials).
    """
    # Resolve mesh name once
    dag_path = mesh_dag_path
    if dag_path.node().hasFn(om.MFn.kMesh) and dag_path.length() > 0:
        dag_path2 = om.MDagPath(dag_path)
        dag_path2.pop()
        mesh_name = dag_path2.fullPathName()
    else:
        mesh_name = dag_path.fullPathName()

    # PMX materials specify contiguous face ranges — use Maya range syntax.
    face_start = 0
    for mat_idx, pmx_mat in enumerate(pmx_materials):
        face_vertex_count = getattr(pmx_mat, "face_vertex_count", 0)
        num_faces = face_vertex_count // 3
        if num_faces == 0:
            continue

        face_end = face_start + num_faces - 1
        sg_obj = maya_materials[mat_idx]
        sg_name = om.MFnDependencyNode(sg_obj).name()

        # Single range string per material instead of one string per face.
        cmds.sets(
            f"{mesh_name}.f[{face_start}:{face_end}]",
            edit=True,
            forceElement=sg_name,
        )

        face_start = face_end + 1


# create_bones_from_pmx_bones moved to mmd.maya.pmx.bone_builder; imported above.


def find_used_bones(pmx_data: PmxModel) -> set:
    """
    Finds all bone indices that are actually used by vertices.

    Args:
        pmx_data (PmxModel): PMX model data containing vertices.

    Returns:
        set: Set of bone indices that are used by at least one vertex.
    """
    used_bone_indices = set()

    for vertex in pmx_data.vertices:
        # Add all bone indices from this vertex's weights
        for bone_idx in vertex.weight.bone_index:
            if bone_idx >= 0:  # Valid bone index
                used_bone_indices.add(bone_idx)

    return used_bone_indices


def create_skin_cluster_for_mesh(
    pmx_data: PmxModel,
    mesh_dag_path,
    joints: list,
    name_registry: PMXNamingManager,
) -> om.MObject:
    """
    Creates a skin cluster for the mesh with the given joints.

    Args:
        pmx_data (PmxModel): PMX model data.
        mesh_dag_path (MDagPath): DAG path to the mesh.
        joints (list): List of joint MObjects.
        name_registry (PMXNamingManager): Naming manager for unique names.

    Returns:
        MObject: The created skin cluster MObject, or None if creation failed.
    """
    # Find which bones are actually used by vertices
    used_bone_indices = find_used_bones(pmx_data)

    if not used_bone_indices:
        log.warning("No bones are used by vertices, skipping skin cluster creation")
        return None

    # Get mesh name (use full DAG path for uniqueness across multiple model imports)
    mesh_name = mesh_dag_path.fullPathName()

    # Collect joint names (MFnDagNode is lighter than MFnIkJoint).
    joint_names = []
    for bone_idx in sorted(used_bone_indices):
        if bone_idx < len(joints):
            joint_obj = joints[bone_idx]
            if not joint_obj.isNull():
                joint_names.append(om.MFnDagNode(joint_obj).fullPathName())
        else:
            log.warning(
                "Bone index %d is out of range (total joints: %d)",
                bone_idx,
                len(joints),
            )

    # Execute MEL command
    log.debug(
        "Creating skin cluster for mesh %s with joints: %s", mesh_name, joint_names
    )
    try:
        result = cmds.skinCluster(
            joint_names,
            mesh_name,
            maximumInfluences=4,
            toSelectedBones=True,
        )

        if not result:
            log.error("Failed to create skin cluster")
            return None

        # Get skin cluster name (result is a list with the skin cluster name)
        skin_cluster_name = result[0] if isinstance(result, list) else result

        # Get skin cluster MObject
        sel = om.MSelectionList()
        sel.add(skin_cluster_name)
        skin_cluster_obj = sel.getDependNode(0)

        log.debug("Created skin cluster: %s", skin_cluster_name)
        return skin_cluster_obj

    except Exception as e:
        log.error("Failed to create skin cluster: %s", e)
        log.debug(traceback.format_exc())
        return None


def apply_skin_weights(
    pmx_data: PmxModel,
    mesh_dag_path,
    skin_cluster_obj: om.MObject,
    joints: list,
):
    """
    Applies skin weights from PMX data to the Maya skin cluster.

    Optimised paths:
    - O(I+J) influence->bone mapping via fullPathName dict (was O(IxJ)).
    - Skips the expensive ``getWeights`` read-back; allocates the target
      ``MDoubleArray`` directly.
    - Replaces per-vertex ``dict`` allocation with a fixed-size BDEF4 scan.
    - Accumulates total weight inline; removes the redundant O(VxI)
      zero-weight validation pass.

    Args:
        pmx_data (PmxModel): PMX model data containing vertex weights.
        mesh_dag_path (MDagPath): DAG path to the mesh.
        skin_cluster_obj (MObject): The skin cluster MObject.
        joints (list): List of joint MObjects (indexed by PMX bone index).
    """
    if not skin_cluster_obj or skin_cluster_obj.isNull():
        log.error("Invalid skin cluster object")
        return

    try:
        skin_cluster_fn = oma.MFnSkinCluster(skin_cluster_obj)

        # -- Phase 1: influence lookup tables --
        influence_paths = skin_cluster_fn.influenceObjects()
        influence_count = len(influence_paths)
        log.debug("Found %d influence joints for skin cluster", influence_count)

        # O(J):  joint fullPathName -> PMX bone index
        joint_name_to_bone_idx: dict[str, int] = {}
        for bone_idx, joint_obj in enumerate(joints):
            if not joint_obj.isNull():
                name = om.MFnDagNode(joint_obj).fullPathName()
                joint_name_to_bone_idx[name] = bone_idx

        # O(I):  PMX bone index -> skin-cluster influence index
        pmx_bone_to_influence_map: dict[int, int] = {}
        influence_indices = om.MIntArray()

        for influence_path in influence_paths:
            influence_idx = skin_cluster_fn.indexForInfluenceObject(influence_path)
            influence_indices.append(influence_idx)

            inf_name = influence_path.fullPathName()
            bone_idx = joint_name_to_bone_idx.get(inf_name)
            if bone_idx is not None:
                pmx_bone_to_influence_map[bone_idx] = influence_idx

        log.debug(
            "Mapped %d PMX bones to influence indices",
            len(pmx_bone_to_influence_map),
        )

        # -- Phase 2: build weight array --
        mesh_vertex_count = len(pmx_data.vertices)
        total_elements = mesh_vertex_count * influence_count

        # Allocate directly -- skip the expensive getWeights() read-back.
        weights_to_apply = om.MDoubleArray(total_elements, 0.0)

        # Fixed-size BDEF4 combiner (avoids per-vertex dict allocation).
        # PMX guarantees <= 4 bone weights per vertex.
        _w_bones = [0, 0, 0, 0]
        _w_vals = [0.0, 0.0, 0.0, 0.0]

        zero_weight_vertices: list[int] = []
        base_offset = 0  # = v_idx * influence_count, advanced per vertex

        for v_idx, vertex in enumerate(pmx_data.vertices):
            weight_count = len(vertex.weight.bone_index)
            w_count = 0  # unique bones for this vertex

            # Accumulate weights via linear scan (max 4 iterations).
            for i in range(weight_count):
                bone_idx = vertex.weight.bone_index[i]
                weight = vertex.weight.weight[i]

                if weight <= 0.0:
                    continue

                # Look for an existing entry for this bone.
                found = False
                for j in range(w_count):
                    if _w_bones[j] == bone_idx:
                        _w_vals[j] += weight
                        found = True
                        break

                if not found:
                    if w_count < 4:
                        _w_bones[w_count] = bone_idx
                        _w_vals[w_count] = weight
                        w_count += 1
                    else:
                        log.warning(
                            "Vertex %d has >4 unique bone weights; "
                            "extra weight ignored",
                            v_idx,
                        )

            if w_count == 0:
                zero_weight_vertices.append(v_idx)
                base_offset += influence_count
                continue

            # Compute total and reciprocal in one pass.
            total_weight = _w_vals[0]
            for j in range(1, w_count):
                total_weight += _w_vals[j]

            inv_total = 1.0 / total_weight if total_weight > 0.0 else 0.0

            # Write normalised weights.
            for j in range(w_count):
                bone_idx = _w_bones[j]
                influence_idx = pmx_bone_to_influence_map.get(bone_idx)
                if influence_idx is not None:
                    weights_to_apply[base_offset + influence_idx] = (
                        _w_vals[j] * inv_total
                    )

            base_offset += influence_count

        # -- Phase 3: patch zero-weight vertices --
        for v_idx in zero_weight_vertices:
            log.warning(
                "Vertex %d has zero total skin weight, assigning to first influence",
                v_idx,
            )
            weights_to_apply[v_idx * influence_count] = 1.0

        # -- Phase 4: apply --
        skin_cluster_fn.setWeights(
            mesh_dag_path,
            om.MObject.kNullObj,
            influence_indices,
            weights_to_apply,
            True,  # normalize
        )

        log.debug("Applied skin weights to mesh")

    except Exception as e:
        log.error("Failed to apply skin weights: %s", e)
        log.debug(traceback.format_exc())


def build_pmx_scene(
    pmx_data: PmxModel, build_physics: bool = True
) -> MayaPmxData:
    """
    Builds the PMX scene in Maya from the given PMX data.

    Args:
        pmx_data (PmxModel): The PMX model data.
        build_physics (bool): If True (default), also create the mayaBullet
            physics binding (rigid body colliders + joints, per-frame
            write-back) via ``create_physics_from_pmx_data``.  Colliders are
            shown by mayaBullet's wireframe drawing, tinted per collision
            group.  See docs/PhysicsImplementation.md.
    Returns:
        MayaPmxData: The Maya PMX data containing created objects.
    """
    # TODO: Pre-process PMX data to detect naming issues or any other problems which can
    # cause issues during scene building in Maya.
    # TODO: pre-process should also detect duplicated names and rename them accordingly
    name_registry = PMXNamingManager(pmx_data)

    # Create root node
    root_obj = create_root_node_for_pmx_model(
        pmx_data=pmx_data, name_registry=name_registry
    )

    # Create mesh nodes
    maya_data = create_mesh_nodes_from_pmx_data(
        pmx_data, root_transform_obj=root_obj, name_registry=name_registry
    )

    # Create materials
    maya_materials = create_materials_from_pmx_materials(
        pmx_data, name_registry=name_registry
    )

    # Assign materials to mesh faces
    assign_materials_to_mesh_faces(
        maya_data["mesh_dag_path"], pmx_data.materials, maya_materials
    )

    # Create bones
    joints, bone_name_map, _, ik_bone_to_handle = create_bones_from_pmx_bones(
        pmx_data, root_transform_obj=root_obj, name_registry=name_registry
    )

    # Create blend shapes from morphs BEFORE skin cluster so the deformation
    # order is: blendShape → skinCluster.  This ensures vertex morph offsets
    # are computed on the base (rest-pose) mesh, not on already-skinned vertices.
    blend_shape_node_name, bone_morph_node_name = create_blendshapes_from_pmx_data(
        pmx_data,
        mesh_name=name_registry.get_mesh_name(),
        name_registry=name_registry,
        joints=joints,
    )

    # Create skin cluster
    skin_cluster = create_skin_cluster_for_mesh(
        pmx_data,
        mesh_dag_path=maya_data["mesh_dag_path"],
        joints=joints,
        name_registry=name_registry,
    )

    # Apply skin weights
    if skin_cluster:
        apply_skin_weights(
            pmx_data,
            mesh_dag_path=maya_data["mesh_dag_path"],
            skin_cluster_obj=skin_cluster,
            joints=joints,
        )

    # Build morph name map (PMX morph name -> Maya blend shape target name)
    morph_name_map = {}
    for idx, morph in enumerate(pmx_data.morphs):
        pmx_morph_name = morph.name_local if morph.name_local else morph.name_universal
        maya_target_name = name_registry.get_blendshape_target_name(idx)
        morph_name_map[pmx_morph_name] = maya_target_name

    # ── Store model display name for fast root discovery and UI display ──
    root_name = name_registry.get_root_name()
    model_display_name = (
        name_registry.model_name_local
        or name_registry.model_name_universal
        or pmx_data.model_name
    )
    if not cmds.attributeQuery("pmxModelName", node=root_name, exists=True):
        cmds.addAttr(root_name, longName="pmxModelName", dataType="string")
    cmds.setAttr(
        f"{root_name}.pmxModelName",
        model_display_name,
        type="string",
    )
    log.debug(
        "Stored pmxModelName='%s' on %s",
        model_display_name,
        root_name,
    )

    # Rigid bodies are represented by the physics guide meshes created by
    # rigid_body_builder, which shades them per collision group with one
    # unique material per group.  No separate mesh guides are created here.
    # See docs/PhysicsImplementation.md.

    # Rigid bodies + joints via the native mmdPhysicsNode (embedded Bullet).
    # No handle is kept in memory — the scene is the source of truth; discover
    # physics state later via mmd.maya.pmx_model_utils (wrapped by
    # ModelContext.physics* getters).
    if build_physics:
        solver_node = create_physics_from_pmx_data(
            pmx_data,
            joints=joints,
            name_registry=name_registry,
            root_transform_obj=root_obj,
        )
        # Stamp the solver on the root so discovery can find it directly.
        if solver_node:
            if not cmds.attributeQuery("pmxPhysicsNode", node=root_name, exists=True):
                cmds.addAttr(root_name, longName="pmxPhysicsNode", dataType="string")
            cmds.setAttr(f"{root_name}.pmxPhysicsNode", solver_node, type="string")

    return MayaPmxData(
        root_obj=root_obj,
        mesh_node=maya_data["mesh_node"],
        joints=joints,
        skin_cluster=skin_cluster,
        bone_name_map=bone_name_map,
        morph_name_map=morph_name_map,
        root_name=root_name,
        mesh_name=name_registry.get_mesh_name(),
        bone_morph_node_name=bone_morph_node_name or "",
        blend_shape_node_name=blend_shape_node_name or "",
        ik_handles=list(ik_bone_to_handle.values()),
    )
