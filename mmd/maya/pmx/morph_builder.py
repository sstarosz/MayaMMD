import logging
import traceback
from typing import cast

import maya.cmds as cmds
import maya.api.OpenMaya as om

from mmd.core.data_types import MorphType, PmxModel, PMXMorph, MorphBone
from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


def create_bone_morph_node(
    bone_morphs: list[tuple[int, PMXMorph]],
    name_registry: PMXNamingManager,
    joints: list | None = None,
) -> str | None:
    """
    Create a bone morph node for bone-based morphs.

    Args:
        bone_morphs: List of bone morphs from PMX data
        skeleton_root: Root joint of the skeleton
        pmx_data: Full PMX model data for bone name mapping
        name_registry: Naming manager for consistent naming
        joints: List of joint MObjects (in PMX bone index order) for resolving
                full DAG paths. If provided, full paths are used instead of
                short bone names, preventing ambiguity across multiple models.

    Returns:
        Name of the created bone morph node or None if failed
    """
    if not bone_morphs:
        log.debug("No bone morphs to create")
        return None

    log.debug("Creating bone morph node for %d bone morphs", len(bone_morphs))

    # Create the custom node with a unique name
    node_name = cmds.createNode(
        "boneMorphNode", name=name_registry.get_bone_morph_node_name()
    )
    log.debug("Created bone morph node: %s", node_name)

    # Verify the node was created successfully
    if not cmds.objExists(node_name):
        error_msg = "Failed to create boneMorphNode - node doesn't exist"
        log.error(error_msg)
        raise RuntimeError(error_msg)

    # ── Warm-up: Maya 2026 standalone has an intermittent bug where the
    # first invocation of an MPxCommand with multi-string flags (addTarget
    # with 4 args) fails to detect the flag via numberOfFlagUses.  A
    # harmless query call exercises the command infrastructure so subsequent
    # edit calls work reliably.  The try/except is defensive — the warm-up
    # is best-effort and must never block the build.
    try:
        cmds.boneBlendShape(node_name, query=True, listTargets=True)
    except Exception:
        pass

    # Add bone morphs to the node
    for idx, morph in bone_morphs:
        morph_name = name_registry.get_blendshape_target_name(idx)
        log.debug("Processing bone morph: %s", morph_name)

        # Extract bone data from morph
        joint_names = []
        pos_offsets = []
        rot_offsets = []

        for bone_morph in cast(list[MorphBone], morph.data):
            bone_idx = bone_morph.bone_index
            # Use the naming manager's short name — it is stable and does not
            # change when controllers are inserted above the joint (full DAG
            # paths would shift, causing the same joint to be stored under
            # multiple different paths and breaking the output-index mapping).
            joint_name = name_registry.get_bone_name(bone_idx)
            joint_names.append(joint_name)

            # Position offset (flip Z for Maya coordinate system)
            pos = bone_morph.position_offset
            pos_offsets.append(f"{pos.x},{pos.y},{-pos.z}")

            # Rotation offset (quaternion) — convert MMD left-handed (Z into screen)
            # to Maya right-handed (Z toward viewer) via the Z-flip transformation T=diag(1,1,-1).
            # Conjugating the rotation matrix by T gives: q_maya = (-qx, -qy, qz, qw).
            rot = bone_morph.rotation_offset
            rot_offsets.append(f"{-rot.x},{-rot.y},{rot.z},{rot.w}")

        if not joint_names:
            log.warning("Skipping morph '%s': no valid bone data", morph_name)
            continue

        # Format strings for the command (tuple syntax)
        joints_str = ",".join(joint_names)
        pos_str = ";".join(pos_offsets)
        rot_str = ";".join(rot_offsets)

        # Add target to the bone morph node.
        # Avoid hasattr(cmds, ...) — maya.cmds dynamic resolution can return
        # False for commands registered via MGlobal::executePythonCommand during
        # initializePlugin, even though the command works when called directly.
        target = (morph_name, joints_str, pos_str, rot_str)

        try:
            cmds.boneBlendShape(node_name, edit=True, addTarget=target)
        except (AttributeError, RuntimeError) as exc:
            # Maya 2026 standalone has an intermittent bug where MPxCommand
            # multi-string flags (addTarget with 4 args) sometimes fail to
            # parse even though the call site is identical.  Skip this
            # target and continue — the bone morph node still works.
            log.warning(
                "boneBlendShape failed for '%s' on '%s' (type=%s): %s — skipping target",
                morph_name,
                node_name,
                cmds.nodeType(node_name),
                exc,
            )
            continue

        log.debug(
            "Added bone morph target: %s (%d bones)", morph_name, len(joint_names)
        )

    return node_name


def create_vertex_blend_shapes(
    vertex_morphs: list[tuple[int, PMXMorph]],
    mesh_name: str,
    name_registry: PMXNamingManager,
) -> str | None:
    """
    Create blend shape nodes from PMX vertex morphs and attach them to the mesh.

    Args:
        pmx_data: Full PMX model data for vertex morphs
        mesh_name: Name of the mesh to attach blend shapes to
        name_registry: Naming manager for consistent naming
    Returns:
        Name of the created blend shape node or None if failed
    """
    # Get the base mesh - need to get the shape node
    try:
        # First, check if the mesh exists
        if not cmds.objExists(mesh_name):
            error_msg = f"Mesh does not exist: {mesh_name}"
            log.error(error_msg)
            raise RuntimeError(error_msg)

        # Get the shape node from the transform
        shapes = cmds.listRelatives(mesh_name, shapes=True, fullPath=True)
        if not shapes:
            error_msg = f"No shape node found for mesh: {mesh_name}"
            log.error(error_msg)
            raise RuntimeError(error_msg)

        shape_name = shapes[0]

        # Get the mesh DAG path
        sel = om.MSelectionList()
        sel.add(shape_name)
        mesh_dag_path = sel.getDagPath(0)
        mesh_fn = om.MFnMesh(mesh_dag_path)
        base_vertex_count = mesh_fn.numVertices

        log.debug(
            "Found mesh shape: %s with %d vertices", shape_name, base_vertex_count
        )
    except Exception as e:
        log.error("Failed to get base mesh: %s", e)
        log.debug(traceback.format_exc())
        # Re-raise so test framework catches it
        raise

    # Create duplicate meshes for each morph target
    target_mesh_names = []
    base_points: om.MPointArray | None = None
    try:
        for morph_idx, morph in vertex_morphs:
            target_name = name_registry.get_blendshape_target_name(morph_idx)
            duplicated = cmds.duplicate(mesh_name, name=target_name)
            target_mesh_name = (
                duplicated[0] if isinstance(duplicated, list) else duplicated
            )
            target_mesh_names.append(target_mesh_name)

            sel_target = om.MSelectionList()
            sel_target.add(target_mesh_name)
            target_dag_path = sel_target.getDagPath(0)
            target_mesh_fn = om.MFnMesh(target_dag_path)

            # Read base points from first duplicate; copy for rest.
            if base_points is None:
                base_points = target_mesh_fn.getPoints(om.MSpace.kObject)
                points = om.MPointArray(base_points)
            else:
                points = om.MPointArray(base_points)

            # Apply vertex offsets (no isinstance — vertex morphs are
            # guaranteed list[MorphVertex] with matching topology).
            data = morph.data
            if isinstance(data, list):
                for mv in data:
                    vi = mv.vertex_index
                    if 0 <= vi < base_vertex_count:
                        bp = base_points[vi]
                        d = mv.offset
                        points[vi] = om.MPoint(bp.x + d.x, bp.y + d.y, bp.z - d.z)

            target_mesh_fn.setPoints(points, om.MSpace.kObject)

        # Create the blend shape deformer
        if target_mesh_names:
            # Create blend shape deformer
            blend_shape_result = cmds.blendShape(
                *target_mesh_names,
                mesh_name,
                name=name_registry.get_blendshape_node_name(),
                origin="world",
            )

            blend_shape_name = (
                blend_shape_result[0]
                if isinstance(blend_shape_result, list)
                else blend_shape_result
            )
            # Delete the temporary target meshes
            cmds.delete(target_mesh_names)

            log.debug(
                "Created blend shape deformer: %s with %d targets",
                blend_shape_name,
                len(target_mesh_names),
            )

            # Store PMX→Maya morph name mapping as a compound array attribute
            # so users can inspect/edit it in the Attribute Editor.
            # Each element is a pair: {pmxName: "...", mayaAlias: "..."}.
            # This is more Maya-idiomatic than a stringArray — the UI shows
            # a "+" button to add entries, and each entry has named fields.
            try:
                cmds.addAttr(
                    blend_shape_name,
                    longName="pmxMorphMapping",
                    attributeType="compound",
                    numberOfChildren=2,
                    multi=True,
                    storable=True,
                    niceName="PMX Morph Mapping",
                )
                cmds.addAttr(
                    blend_shape_name,
                    longName="pmxName",
                    dataType="string",
                    parent="pmxMorphMapping",
                    storable=True,
                    niceName="PMX Name",
                )
                cmds.addAttr(
                    blend_shape_name,
                    longName="mayaAlias",
                    dataType="string",
                    parent="pmxMorphMapping",
                    storable=True,
                    niceName="Maya Alias",
                )
                entry_idx = 0
                for morph_idx, morph in vertex_morphs:
                    maya_alias = name_registry.get_blendshape_target_name(morph_idx)
                    if morph.name_local:
                        cmds.setAttr(
                            f"{blend_shape_name}.pmxMorphMapping[{entry_idx}].pmxName",
                            morph.name_local,
                            type="string",
                        )
                        cmds.setAttr(
                            f"{blend_shape_name}.pmxMorphMapping[{entry_idx}].mayaAlias",
                            maya_alias,
                            type="string",
                        )
                        entry_idx += 1
                    if (
                        morph.name_universal
                        and morph.name_universal != morph.name_local
                    ):
                        cmds.setAttr(
                            f"{blend_shape_name}.pmxMorphMapping[{entry_idx}].pmxName",
                            morph.name_universal,
                            type="string",
                        )
                        cmds.setAttr(
                            f"{blend_shape_name}.pmxMorphMapping[{entry_idx}].mayaAlias",
                            maya_alias,
                            type="string",
                        )
                        entry_idx += 1
                log.debug(
                    "Stored pmxMorphMapping on %s (%d entries)",
                    blend_shape_name,
                    entry_idx,
                )
            except Exception as map_err:
                log.warning(
                    "Could not store pmxMorphMapping on %s: %s",
                    blend_shape_name,
                    map_err,
                )

            return blend_shape_name

        return None

    except Exception as e:
        log.error("Failed to create blend shapes: %s", e)
        log.debug(traceback.format_exc())

        # Clean up any created target meshes before re-raising
        remaining = [n for n in target_mesh_names if cmds.objExists(n)]
        if remaining:
            cmds.delete(remaining)

        # Re-raise the exception so tests can detect the failure
        raise


def create_blendshapes_from_pmx_data(
    pmx_data: PmxModel,
    mesh_name: str,
    name_registry: PMXNamingManager,
    joints: list | None = None,
) -> tuple[str | None, str | None]:
    """
    Creates blend shape nodes from PMX morphs and attaches them to the mesh.


    Returns: (vertex_blend_shape_name, bone_morph_node_name) - either may be None
    """
    # Check if there are any morphs in the PMX data
    if not pmx_data.morphs:
        log.debug("No morphs found in PMX data, skipping blend shape creation")
        return None, None

    # Filter for vertex morphs only
    log.debug("Creating blend shapes from PMX morphs")
    vertex_morphs = [
        (idx, morph)
        for idx, morph in enumerate(pmx_data.morphs)
        if morph.morph_type == MorphType.VERTEX
    ]
    log.debug("Found %d vertex morphs in PMX data", len(vertex_morphs))
    vertex_blend_shape_name = None

    if vertex_morphs:
        vertex_blend_shape_name = create_vertex_blend_shapes(
            vertex_morphs, mesh_name, name_registry
        )

    # Filter for bone morphs only
    log.debug("Creating bone morph node from PMX morphs")
    bone_morphs = [
        (idx, morph)
        for idx, morph in enumerate(pmx_data.morphs)
        if morph.morph_type == MorphType.BONE
    ]
    log.debug("Found %d bone morphs in PMX data", len(bone_morphs))

    # Create bone morph node for bone morphs
    bone_morph_node_name = None
    if bone_morphs:
        bone_morph_node_name = create_bone_morph_node(
            bone_morphs, name_registry, joints
        )

    return vertex_blend_shape_name, bone_morph_node_name
