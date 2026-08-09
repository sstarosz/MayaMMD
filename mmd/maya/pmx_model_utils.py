"""Utility functions for manipulating PMX models in Maya.

This module provides general-purpose utilities for working with imported PMX models,
such as pose resets, transformations, and other model-level operations.
"""

import json
import logging
import re
from typing import Dict, List, Optional

import maya.api.OpenMaya as om
import maya.cmds as cmds
from mmd.maya.maya_data_types import ResolvedModelData

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Scene discovery utilities — selection-based model targeting
# ---------------------------------------------------------------------------


def _is_pmx_root(node_name: str) -> bool:
    """Check if *node_name* is a PMX model root transform.

    Two-tier check:
    1. **Fast path** — ``pmxModelName`` custom attribute exists on the
       transform itself (set during PMX import).  O(1), no descendant scan.
    2. **Legacy fallback** — name ends with ``_Root`` and at least one
       descendant joint has ``pmxNameLocal``.  Used for scenes imported
       before ``pmxModelName`` was added.

    Args:
        node_name: Maya transform name to check.

    Returns:
        ``True`` if the node appears to be a PMX model root.
    """
    # Fast path: self-describing attribute (set during PMX import)
    try:
        if cmds.attributeQuery("pmxModelName", node=node_name, exists=True):
            return True
    except Exception:
        pass

    # Legacy fallback: name pattern + descendant joint attribute check
    if not re.search(r"_Root(_\d+)?$", node_name):
        return False
    joints = (
        cmds.listRelatives(node_name, allDescendents=True, type="joint", fullPath=True)
        or []
    )
    return any(
        cmds.attributeQuery("pmxNameLocal", node=j, exists=True) for j in joints[:5]
    )


def find_model_root_from_selection() -> Optional[str]:
    """Walk up the DAG from the current selection to find a PMX model root.

    If the selection is empty, or the selected object does not belong to any
    PMX model, returns ``None``.

    Returns:
        The full DAG path of the PMX root transform, or ``None``.
    """
    sel = cmds.ls(selection=True, long=True)
    if not sel:
        return None

    # Use the first selected object
    obj = sel[0]
    parent = obj
    while parent is not None:
        if _is_pmx_root(parent):
            return parent
        parents = cmds.listRelatives(parent, parent=True, fullPath=True)
        parent = parents[0] if parents else None

    return None


def find_all_model_roots_from_selection() -> List[str]:
    """Return the full DAG paths of ALL unique PMX model roots spanned by the
    current Maya selection.

    If the selection is empty, or no selected object belongs to a PMX model,
    returns an empty list.

    Unlike :func:`find_model_root_from_selection` (which returns only the
    root of the *first* selected object), this collects roots for every
    selected object so callers can apply operations to multiple models at
    once.

    Returns:
        List of unique root transform paths (may be empty).
    """
    sel = cmds.ls(selection=True, long=True)
    if not sel:
        return []

    roots: List[str] = []
    for obj in sel:
        parent = obj
        while parent is not None:
            if _is_pmx_root(parent):
                if parent not in roots:
                    roots.append(parent)
                break
            parents = cmds.listRelatives(parent, parent=True, fullPath=True)
            parent = parents[0] if parents else None

    return roots


def discover_model_roots_in_scene() -> List[str]:
    """Return the full DAG paths of all PMX model roots in the current scene.

    A node is a PMX root if it carries the ``pmxModelName`` custom attribute
    (set during PMX import).  No naming conventions are used.

    Returns:
        List of root transform paths (may be empty).
    """
    return [t for t in (cmds.ls(type="transform", long=True) or []) if _is_pmx_root(t)]


def build_bone_map_from_scene(root_name: str) -> Dict[str, str]:
    """Reconstruct a PMX-name→Maya-joint-name mapping from scene attributes.

    Reads ``pmxNameLocal`` and ``pmxNameUniversal`` from every joint
    descendant of *root_name*, mirroring what :func:`build_bone_name_map`
    does during import.

    Args:
        root_name: Full DAG path of the PMX model root.

    Returns:
        Dict mapping PMX bone names to Maya joint names.
    """
    try:
        joints = (
            cmds.listRelatives(
                root_name, allDescendents=True, type="joint", fullPath=True
            )
            or []
        )
    except Exception:
        log.warning("Root '%s' does not exist — cannot build bone map", root_name)
        return {}
    bone_map: Dict[str, str] = {}
    for joint in joints:
        if cmds.attributeQuery("pmxNameLocal", node=joint, exists=True):
            name_local = cmds.getAttr(f"{joint}.pmxNameLocal")
            if name_local:
                bone_map[name_local] = joint
        if cmds.attributeQuery("pmxNameUniversal", node=joint, exists=True):
            name_uni = cmds.getAttr(f"{joint}.pmxNameUniversal")
            if name_uni and name_uni not in bone_map:
                bone_map[name_uni] = joint
    return bone_map


def build_morph_map_from_scene(root_name: str) -> Dict[str, str]:
    """Reconstruct a PMX-morph-name→Maya-alias mapping from the blendShape node.

    Reads the self-describing ``pmxMorphMapping`` compound multi-attribute
    on the blendShape node.  Each array element has two string children:
    ``pmxName`` (the original PMX morph name) and ``mayaAlias`` (the Maya
    blendShape target alias).  Falls back to ``aliasAttr()`` which returns
    Maya aliases only (PMX names lost — map is identity in this case).

    Args:
        root_name: Full DAG path of the PMX model root.

    Returns:
        Dict mapping PMX morph names to Maya blendShape target aliases.
    """
    blend_shape = _find_blend_shape_node(root_name)
    if not blend_shape:
        return {}

    # Read self-describing compound array attribute
    if cmds.attributeQuery("pmxMorphMapping", node=blend_shape, exists=True):
        try:
            size = cmds.getAttr(f"{blend_shape}.pmxMorphMapping", size=True) or 0
            if size > 0:
                result: Dict[str, str] = {}
                for i in range(size):
                    pmx_name = cmds.getAttr(
                        f"{blend_shape}.pmxMorphMapping[{i}].pmxName"
                    )
                    maya_alias = cmds.getAttr(
                        f"{blend_shape}.pmxMorphMapping[{i}].mayaAlias"
                    )
                    if pmx_name and maya_alias:
                        result[pmx_name] = maya_alias
                if result:
                    return result
        except Exception as exc:
            log.warning("Failed to read pmxMorphMapping on %s: %s", blend_shape, exc)

    # Fallback: return Maya aliases only (identity map)
    aliases = cmds.aliasAttr(blend_shape, query=True) or []
    names = [aliases[i] for i in range(0, len(aliases), 2)]
    log.warning(
        "No pmxMorphMapping attribute on %s — using identity map (%d names)",
        blend_shape,
        len(names),
    )
    return {n: n for n in names}


def _find_blend_shape_node(root_name: str) -> Optional[str]:
    """Find the blendShape deformer node for a PMX model.

    Finds the mesh transform and scans its deformation history for a
    ``blendShape`` node.

    Args:
        root_name: Full DAG path of the PMX model root.

    Returns:
        Name of the blendShape node, or ``None``.
    """
    mesh = _find_mesh_node(root_name)
    if mesh:
        shapes = cmds.listRelatives(mesh, shapes=True, type="mesh", fullPath=True)
        if shapes:
            history = cmds.listHistory(shapes[0], pruneDagObjects=True) or []
            for node in history:
                if cmds.nodeType(node) == "blendShape":
                    return node
    return None


def _find_mesh_node(root_name: str) -> Optional[str]:
    """Find the mesh transform node under a PMX root.

    The mesh is expected to be named ``{model}_Mesh`` and sit under
    ``{model}_Geo``.  When that pattern fails, falls back to scanning
    all descendants for a transform with a ``mesh`` shape child.

    Args:
        root_name: Full DAG path of the PMX model root.

    Returns:
        Name of the mesh transform node, or ``None``.
    """
    # Try naming convention first (fast path)
    base = root_name.split("|")[-1]
    if base.endswith("_Root") or base.endswith("_Root_"):
        # Strip suffix to get model name prefix
        import re

        model_prefix = re.sub(r"_Root(_\d+)?$", "", base)
        mesh_name = f"{model_prefix}_Mesh"
        if cmds.objExists(mesh_name):
            return mesh_name

    # Fallback: scan descendants for a transform with a mesh shape
    descendants = (
        cmds.listRelatives(root_name, allDescendents=True, type="transform") or []
    )
    for t in descendants:
        shapes = cmds.listRelatives(t, shapes=True, type="mesh") or []
        if shapes:
            return t
    return None


def find_blend_shape_node(root_name: str) -> Optional[str]:
    """Public wrapper for :func:`_find_blend_shape_node`."""
    return _find_blend_shape_node(root_name)


def find_bone_morph_node(root_name: str) -> Optional[str]:
    """Find the ``boneMorphNode`` DG node associated with a PMX model.

    Scans all ``boneMorphNode`` nodes and traces their ``outputRotate``
    connections through intermediate utility nodes (e.g. ``unitConversion``)
    to MORPH_ controllers under *root_name*.

    Args:
        root_name: Full DAG path of the PMX model root.

    Returns:
        Name of the ``boneMorphNode``, or ``None``.
    """
    for bmn in cmds.ls(type="boneMorphNode") or []:
        try:
            # Trace outputRotate connections (including array elements) to
            # MORPH_ controllers under this root.  Maya may insert
            # unitConversion nodes between outputRotate and the controller's
            # rotate attribute, so we follow the chain.
            outputs: list[str] = []
            # Check the array parent first
            outputs.extend(
                cmds.listConnections(
                    f"{bmn}.outputRotate", destination=True, plugs=True
                )
                or []
            )
            # Also check individual array elements (listConnections on the
            # parent array may miss element-level connections).
            num_elements = cmds.getAttr(f"{bmn}.outputRotate", size=True) or 0
            for i in range(num_elements):
                outputs.extend(
                    cmds.listConnections(
                        f"{bmn}.outputRotate[{i}]", destination=True, plugs=True
                    )
                    or []
                )
            for plug in outputs:
                node_name = plug.split(".")[0]
                # If the immediate destination is a utility node (e.g.
                # unitConversion), follow its output to find the real
                # controller transform.
                node_type = cmds.nodeType(node_name)
                if node_type == "unitConversion":
                    out_plugs = (
                        cmds.listConnections(
                            f"{node_name}.output", destination=True, plugs=True
                        )
                        or []
                    )
                    for out_plug in out_plugs:
                        candidate = out_plug.split(".")[0]
                        ctrl_long = cmds.ls(candidate, long=True)
                        if ctrl_long and root_name in ctrl_long[0]:
                            return bmn
                else:
                    ctrl_long = cmds.ls(node_name, long=True)
                    if ctrl_long and root_name in ctrl_long[0]:
                        return bmn
        except Exception:
            continue
    return None


def find_ik_handles(root_name: str) -> List[str]:
    """Return all IK handle names that are descendants of a PMX model root.

    Args:
        root_name: Full DAG path of the PMX model root.

    Returns:
        List of IK handle names (may be empty).
    """
    return cmds.listRelatives(root_name, allDescendents=True, type="ikHandle") or []


# ---------------------------------------------------------------------------
# Physics discovery — the scene is the source of truth.  These read the
# node's own data (the ``{model}_Physics`` group, the root's ``pmxPhysicsNode``
# attr, and the node's ``bodies[i]`` / ``anchorWorldMatrix[k]`` /
# ``outRotate[i]`` connections) so the binding can be reconstructed from any
# saved scene.  ModelContext wraps these as lazy getters
# (see mmd/maya/model_context.py).
# ---------------------------------------------------------------------------


def find_physics_group(root_name: str) -> Optional[str]:
    """Return the physics group transform for a PMX model root, or ``None``.

    The physics group is the first child transform of *root_name* whose name
    ends in ``_Physics`` (created by
    ``mmd.maya.pmx.rigid_body_builder.create_physics_from_pmx_data``).
    """
    for child in cmds.listRelatives(root_name, children=True, type="transform") or []:
        if child.endswith("_Physics"):
            return child
    return None


def find_physics_node(root_name: str) -> Optional[str]:
    """Return the ``pmxPhysicsNode`` solver for a PMX model root, or ``None``.

    Prefers the ``pmxPhysicsNode`` attribute stamped on the root at import;
    falls back to scanning the physics group's children (the solver is a
    locator shape parented under the group — scenes imported before the
    attribute existed).
    """
    if cmds.attributeQuery("pmxPhysicsNode", node=root_name, exists=True):
        node = cmds.getAttr(f"{root_name}.pmxPhysicsNode")
        if node and cmds.objExists(node):
            return node
    group = find_physics_group(root_name)
    if group is None:
        return None
    for child in cmds.listRelatives(group, children=True, fullPath=True) or []:
        if cmds.nodeType(child) == "pmxPhysicsNode":
            return child
    return None


def _body_physics_mode(node: str, rb_idx: int) -> int:
    """Read a body's PMX physics mode (0 FOLLOW_BONE, 1 PHYSICS, 2 PHYSICS_BONE)."""
    return int(cmds.getAttr(f"{node}.bodies[{rb_idx}].bodyPhysicsMode"))


def _driven_joint_from_out(node: str, i: int, mode: int) -> Optional[str]:
    """Return the joint driven by body *i*'s write-back output (or ``None``).

    Mode 1 (PHYSICS) drives translate AND rotate: the translate connection
    goes STRAIGHT to the joint (no unit conversion), so it is the reliable
    discovery path.  Mode 2 (PHYSICS_BONE) drives rotate only, and Maya
    auto-inserts a ``unitConversion`` between the raw double3
    ``outRotateValue`` and the angle-unit ``joint.rotate`` — follow its
    ``output`` plug.
    """
    if mode == 1:
        for dest in (
            cmds.listConnections(
                f"{node}.outTranslate[{i}].outTranslateValue", destination=True
            )
            or []
        ):
            if dest and dest != node:
                return dest
        return None
    # mode 2: rotation-only, through the auto-inserted unitConversion.
    for dest in (
        cmds.listConnections(f"{node}.outRotate[{i}].outRotateValue", destination=True)
        or []
    ):
        if cmds.nodeType(dest) == "unitConversion":
            for j in (
                cmds.listConnections(f"{dest}.output", destination=True) or []
            ):
                if j and j != node:
                    return j
        elif dest and dest != node:
            return dest
    return None


def find_physics_rigid_bodies(root_name: str) -> Dict[int, str]:
    """Return ``{pmx_rigid_body_index: related_joint}`` for a model root.

    Phase 3: guide transforms are gone — each body's related joint is traced
    from the node's own connections:
      * kinematic (FOLLOW_BONE) bodies via ``anchorWorldMatrix[k]`` (the
        anchors are in kinematic order), and
      * dynamic bodies via the write-back output connections (``outTranslate``
        for mode 1, ``outRotate`` for mode 2 — the node writes the solved pose
        straight into the related joint).
    """
    node = find_physics_node(root_name)
    if node is None:
        return {}
    try:
        n = int(cmds.getAttr(f"{node}.bodies", size=True) or 0)
    except Exception:
        return {}
    bodies: Dict[int, str] = {}
    kin_idx = 0
    for i in range(n):
        if _body_physics_mode(node, i) == 0:
            srcs = (
                cmds.listConnections(f"{node}.anchorWorldMatrix[{kin_idx}]", source=True)
                or []
            )
            kin_idx += 1
        else:
            srcs = [_driven_joint_from_out(node, i, _body_physics_mode(node, i))]
        if srcs and srcs[0]:
            bodies[i] = srcs[0]
    return bodies


def find_physics_driven_joints(root_name: str) -> Dict[int, str]:
    """Return ``{pmx_rigid_body_index: joint}`` for DYNAMIC bodies (write-back).

    Phase 3: the node writes the solved pose straight into these joints, so
    they are the write-back targets (this replaces the old
    parentConstraint/orientConstraint discovery — no constraints exist any
    more).  The related joint is traced from the write-back output connections
    (``outTranslate`` for mode 1, ``outRotate`` through any auto-inserted
    unitConversion for mode 2).
    """
    node = find_physics_node(root_name)
    if node is None:
        return {}
    try:
        n = int(cmds.getAttr(f"{node}.bodies", size=True) or 0)
    except Exception:
        return {}
    result: Dict[int, str] = {}
    for i in range(n):
        if _body_physics_mode(node, i) == 0:
            continue
        joint = _driven_joint_from_out(node, i, _body_physics_mode(node, i))
        if joint:
            result[i] = joint
    return result


# ---------------------------------------------------------------------------
# Joint transform utilities
# ---------------------------------------------------------------------------


def set_joint_rotate_safe(
    joint_name: str,
    rx: float,
    ry: float,
    rz: float,
) -> bool:
    """Set rotation directly on a joint.

    All joints use MORPH_ transform controllers so joint.rotate is always
    a free (undriven) attribute.  Logs a warning and returns False if the
    attribute is unexpectedly driven or does not exist.

    Returns True if the rotation was successfully set, False otherwise.
    """
    try:
        sel = om.MSelectionList()
        sel.add(f"{joint_name}.rotateX")
        plug_rx = sel.getPlug(0)
    except Exception:
        log.warning("Joint %s does not exist or has no rotate attribute", joint_name)
        return False

    if plug_rx.isDestination:
        log.debug("Joint %s rotate is unexpectedly driven — skipping", joint_name)
        return False

    try:
        cmds.setAttr(f"{joint_name}.rotateX", rx)
        cmds.setAttr(f"{joint_name}.rotateY", ry)
        cmds.setAttr(f"{joint_name}.rotateZ", rz)
        log.debug(
            "Set rotation on %s: (%.2f, %.2f, %.2f) degrees",
            joint_name,
            rx,
            ry,
            rz,
        )
        return True
    except Exception as exc:
        log.warning("Failed to set rotation on %s: %s", joint_name, exc)
        return False


def set_joint_translate_safe(
    joint_name: str,
    tx: float,
    ty: float,
    tz: float,
) -> bool:
    """Set translation directly on a joint.

    All joints use MORPH_ transform controllers so joint.translate is always
    a free (undriven) attribute.  Logs a warning and returns False if the
    attribute is unexpectedly driven or does not exist.

    Returns True if the translation was successfully set, False otherwise.
    """
    try:
        sel = om.MSelectionList()
        sel.add(f"{joint_name}.translateX")
        plug_tx = sel.getPlug(0)
    except Exception:
        log.warning("Joint %s does not exist or has no translate attribute", joint_name)
        return False

    if plug_tx.isDestination:
        log.debug("Joint %s translate is unexpectedly driven — skipping", joint_name)
        return False

    try:
        cmds.setAttr(f"{joint_name}.translateX", tx)
        cmds.setAttr(f"{joint_name}.translateY", ty)
        cmds.setAttr(f"{joint_name}.translateZ", tz)
        log.debug(
            "Set translation on %s: (%.2f, %.2f, %.2f)",
            joint_name,
            tx,
            ty,
            tz,
        )
        return True
    except Exception as exc:
        log.warning("Failed to set translation on %s: %s", joint_name, exc)
        return False


def collect_ik_chain_joints(ik_handles: Optional[list[str]] = None) -> set[str]:
    """Return Maya joint names whose rotation is DIRECTLY controlled by an IK solver.

    In Maya's ikSCsolver, the solver rotates the link joints (from startJoint to
    the parent of the end effector) but does NOT rotate the end effector joint
    itself.

    Args:
        ik_handles: IK handle names to inspect. When provided, only handles in
            this list are examined (model-scoped). When ``None``, all IK handles
            in the scene are used as a fallback.

    Returns:
        Set of Maya joint names whose rotation is directly driven by an IK solver.
    """
    chain_joints: set[str] = set()
    if ik_handles is None:
        ik_handles = cmds.ls(type="ikHandle") or []
    for handle in ik_handles:
        try:
            start_joint = cmds.ikHandle(handle, query=True, startJoint=True)
            end_eff = cmds.ikHandle(handle, query=True, endEffector=True)
            if not start_joint or not end_eff:
                continue
            eff_parents = cmds.listRelatives(end_eff, parent=True, fullPath=True) or []
            if not eff_parents:
                continue
            end_joint = eff_parents[0]
            end_long = cmds.ls(end_joint, long=True)[0]

            # Walk the hierarchy from start down to (but NOT including) the end effector.
            # Convert start_joint short name to a full DAG path.  With 785 joints many
            # share the same short name; picking [0] from an ambiguous ls() result would
            # silently traverse the wrong branch and produce an empty (or wrong) chain.
            start_joint_long_list = cmds.ls(start_joint, long=True) or []
            if not start_joint_long_list:
                continue
            joint_long = start_joint_long_list[0]
            while joint_long:
                if joint_long == end_long:
                    break
                # Store the short name so the returned set is consistent with how
                # bone_map values (joint names) are stored everywhere else.
                chain_joints.add(joint_long.split("|")[-1])
                # Request children as full DAG paths to avoid the short-name
                # ambiguity that caused wrong branches to be followed.
                children_long = (
                    cmds.listRelatives(
                        joint_long, children=True, type="joint", fullPath=True
                    )
                    or []
                )
                next_joint_long: str | None = None
                for child_long in children_long:
                    if child_long == end_long or end_long.startswith(child_long + "|"):
                        next_joint_long = child_long
                        break
                if next_joint_long is None:
                    break
                joint_long = next_joint_long

            # The end effector's parent joint (end_long) is also driven by the IK
            # solver — it is the last link in the chain whose rotation the solver
            # controls to position the effector.  The walker loop stops before
            # adding it, so we include it explicitly here.
            chain_joints.add(end_long.split("|")[-1])
        except Exception as exc:
            log.debug("Failed to process IK handle %s: %s", handle, exc)

    return chain_joints


# ---------------------------------------------------------------------------
# Pose reset utilities
# ---------------------------------------------------------------------------


def reset_all_bones_to_rest_pose(bone_map: dict[str, str]) -> None:
    """Reset ALL bones in the model to their rest pose.

    Reads rest pose values from pmxRest* custom attributes and applies them
    using the safe setters to handle DG connections properly.

    Args:
        bone_map: Dictionary mapping PMX bone names to Maya joint names
    """
    all_joints = set(bone_map.values())
    reset_count = 0
    skip_count = 0

    log.debug("Resetting %d bones to rest pose", len(all_joints))

    for joint_name in all_joints:
        # Check if joint has rest pose attributes
        if not cmds.attributeQuery("pmxRestRotateX", node=joint_name, exists=True):
            skip_count += 1
            continue

        try:
            # Read rest values from custom attributes
            rest_tx = cmds.getAttr(f"{joint_name}.pmxRestTranslateX")
            rest_ty = cmds.getAttr(f"{joint_name}.pmxRestTranslateY")
            rest_tz = cmds.getAttr(f"{joint_name}.pmxRestTranslateZ")
            rest_rx = cmds.getAttr(f"{joint_name}.pmxRestRotateX")
            rest_ry = cmds.getAttr(f"{joint_name}.pmxRestRotateY")
            rest_rz = cmds.getAttr(f"{joint_name}.pmxRestRotateZ")

            # Apply rest values using existing safe setters
            translate_ok = set_joint_translate_safe(
                joint_name, rest_tx, rest_ty, rest_tz
            )
            rotate_ok = set_joint_rotate_safe(joint_name, rest_rx, rest_ry, rest_rz)

            if translate_ok and rotate_ok:
                reset_count += 1
            else:
                skip_count += 1

        except Exception as exc:
            log.warning("Failed to reset joint %s to rest pose: %s", joint_name, exc)
            skip_count += 1

    log.debug(
        "Reset %d bones to rest pose (%d skipped/failed)", reset_count, skip_count
    )


def reset_ik_handles_to_rest_pose(ik_handles: Optional[list[str]] = None) -> None:
    """Reset IK handles to their rest pose.

    IK handles receive transformations during VPD pose application and must be
    reset to prevent stacking when multiple poses are applied sequentially.

    Args:
        ik_handles: IK handle names to reset. When provided, only handles in
            this list are reset (model-scoped). When ``None``, all IK handles
            in the scene are used as a fallback.
    """
    if ik_handles is None:
        ik_handles = cmds.ls(type="ikHandle") or []
    reset_count = 0
    skip_count = 0

    log.debug("Resetting %d IK handles to rest pose", len(ik_handles))

    for handle_name in ik_handles:
        # Check if handle has rest pose attributes
        if not cmds.attributeQuery("pmxIkRestRotateX", node=handle_name, exists=True):
            skip_count += 1
            continue

        try:
            # Read rest values from custom attributes
            rest_tx = cmds.getAttr(f"{handle_name}.pmxIkRestTranslateX")
            rest_ty = cmds.getAttr(f"{handle_name}.pmxIkRestTranslateY")
            rest_tz = cmds.getAttr(f"{handle_name}.pmxIkRestTranslateZ")
            rest_rx = cmds.getAttr(f"{handle_name}.pmxIkRestRotateX")
            rest_ry = cmds.getAttr(f"{handle_name}.pmxIkRestRotateY")
            rest_rz = cmds.getAttr(f"{handle_name}.pmxIkRestRotateZ")

            # Apply rest values directly (IK handles don't have DG connections like bone morphs)
            cmds.setAttr(f"{handle_name}.translateX", rest_tx)
            cmds.setAttr(f"{handle_name}.translateY", rest_ty)
            cmds.setAttr(f"{handle_name}.translateZ", rest_tz)
            cmds.setAttr(f"{handle_name}.rotateX", rest_rx)
            cmds.setAttr(f"{handle_name}.rotateY", rest_ry)
            cmds.setAttr(f"{handle_name}.rotateZ", rest_rz)

            reset_count += 1

        except Exception as exc:
            log.warning(
                "Failed to reset IK handle %s to rest pose: %s", handle_name, exc
            )
            skip_count += 1

    log.debug(
        "Reset %d IK handles to rest pose (%d skipped/failed)", reset_count, skip_count
    )


def _reset_bone_morphs(
    bone_morph_node_name: Optional[str] = None,
    pmx_root_name: Optional[str] = None,
) -> None:
    """Reset bone morph weights to zero and clear MORPH_ controller transforms.

    Bone morphs apply offsets through MORPH_ controller transforms connected via DG.
    When resetting to bind pose, these must be cleared to prevent stacking with the
    rest pose values.

    Args:
        bone_morph_node_name: Name of the model's boneMorphNode.
            When provided only that node is reset; otherwise all scene boneMorphNodes
            are reset (scene-global fallback).
        pmx_root_name: Name of the PMX root transform.  When provided, only MORPH_
            controller transforms that are descendants of this root are reset;
            otherwise all MORPH_* transforms in the scene are reset.
    """
    # --- Resolve the boneMorphNode list (model-scoped when possible) ---
    if bone_morph_node_name and cmds.objExists(bone_morph_node_name):
        bone_morph_nodes = [bone_morph_node_name]
        log.debug("Using model-scoped boneMorphNode: %s", bone_morph_node_name)
    else:
        bone_morph_nodes = cmds.ls(type="boneMorphNode") or []
        if bone_morph_node_name:
            log.debug(
                "boneMorphNode '%s' not found; falling back to scene-global scan",
                bone_morph_node_name,
            )

    if not bone_morph_nodes:
        log.debug("No boneMorphNode nodes found - skipping bone morph reset")
        return

    weights_reset = 0
    controllers_reset = 0

    for node_name in bone_morph_nodes:
        log.debug("Resetting bone morph node: %s", node_name)

        # Reset all weight attributes to 0
        try:
            # Get all weight array indices
            weight_indices = (
                cmds.getAttr(f"{node_name}.weight", multiIndices=True) or []
            )
            for idx in weight_indices:
                cmds.setAttr(f"{node_name}.weight[{idx}]", 0.0)
                weights_reset += 1
            log.debug("Reset %d weights on %s", len(weight_indices), node_name)
        except Exception as exc:
            log.warning("Failed to reset weights on %s: %s", node_name, exc)

    # --- Resolve MORPH_ controller list (model-scoped when possible) ---
    if pmx_root_name and cmds.objExists(pmx_root_name):
        descendants = (
            cmds.listRelatives(pmx_root_name, allDescendents=True, type="transform")
            or []
        )
        morph_controllers = [
            t for t in descendants if t.split("|")[-1].endswith("_MorphCtrl")
        ]
        log.debug(
            "Scanning %d descendants of '%s' for morphCtrl controllers",
            len(descendants),
            pmx_root_name,
        )
    else:
        all_transforms = cmds.ls(type="transform") or []
        morph_controllers = [t for t in all_transforms if t.endswith("_MorphCtrl")]
        if pmx_root_name:
            log.debug(
                "Root '%s' not found; falling back to scene-global MORPH_ scan",
                pmx_root_name,
            )

    for ctrl_name in morph_controllers:
        try:
            # Reset translate and rotate to zero
            for attr in ["translateX", "translateY", "translateZ"]:
                if not cmds.getAttr(f"{ctrl_name}.{attr}", lock=True):
                    cmds.setAttr(f"{ctrl_name}.{attr}", 0.0)
            for attr in ["rotateX", "rotateY", "rotateZ"]:
                if not cmds.getAttr(f"{ctrl_name}.{attr}", lock=True):
                    cmds.setAttr(f"{ctrl_name}.{attr}", 0.0)
            controllers_reset += 1
        except Exception as exc:
            log.debug("Failed to reset MORPH_ controller %s: %s", ctrl_name, exc)

    log.debug(
        "Reset bone morphs: %d weights zeroed, %d controllers cleared",
        weights_reset,
        controllers_reset,
    )


def reset_model_to_bind_pose(
    model: "ResolvedModelData",
) -> dict[str, int]:
    """Reset a PMX model to its bind pose (rest pose captured at import).

    Resets all joints to their rest pose values stored in custom
    attributes and clears bone morph weights. This is useful for clearing
    poses before applying a new one, or for returning to the base pose
    during animation workflow.

    IK handles are parented under their control bones and follow
    automatically — no separate reset is needed.

    Args:
        model: Resolved model data (bone map, bone morph node name, root name).

    Returns:
        Dictionary with statistics: {"bones_reset": count, "ik_handles_reset": count}
    """
    bone_map = model.bone_map
    bone_morph_node_name = model.bone_morph_node
    pmx_root_name = model.root_name

    log.debug("Resetting PMX model to bind pose")

    if not bone_map:
        log.error("Bone map is empty - cannot reset pose")
        return {"bones_reset": 0, "ik_handles_reset": 0}

    # Reset bone morphs first (so weights don't interfere with joint resets)
    _reset_bone_morphs(
        bone_morph_node_name=bone_morph_node_name or None,
        pmx_root_name=pmx_root_name or None,
    )

    # Capture original ikBlend values before disabling IK temporarily.
    # This preserves user-set / keyed values and ensures the rig state is
    # identical before and after a public reset (e.g. from the UI button).
    # TODO: Improve ik handle discovery to only include handles that are descendants of the model root.
    # Note: We query *all* IK handles in the scene (not just those of the
    # targeted model) because IK handles from sibling models can still
    # affect chain joints if they share the same DAG space.  Disabling all
    # IK blend ensures FK values are uncontested during the reset.
    _ik_handles = cmds.ls(type="ikHandle") or []
    _ik_blend_orig: dict[str, float] = {}
    for _ik_h in _ik_handles:
        try:
            _ik_blend_orig[_ik_h] = cmds.getAttr(f"{_ik_h}.ikBlend")
            cmds.setAttr(f"{_ik_h}.ikBlend", 0.0)
        except Exception:
            pass

    # Reset all bones (IK disabled → no IK override on chain joints)
    reset_all_bones_to_rest_pose(bone_map)

    # Phase 3: physics-driven joints are owned by the pmxPhysicsNode (their
    # translate/rotate are connected to its outputs, so setAttr cannot touch
    # them).  Rewind the solver so it rebuilds from the CURRENT (now rest)
    # skeleton pose and writes the exact rest pose into the driven joints —
    # without this, a model that was mid-simulation keeps its last solved pose
    # on those joints after a reset.
    solver = find_physics_node(pmx_root_name or "")
    if solver:
        try:
            from mmd.maya.pmx.rigid_body_builder import step_physics

            # Phase 3: the node owns the physics-driven joints (their
            # translate/rotate are connected to its outputs, so setAttr cannot
            # touch them).  Force a config-change REBUILD (Phase 4) by toggling
            # fps: the node destroys + rebuilds its Bullet world and re-anchors
            # every dynamic body to the CURRENT (now rest) skeleton pose, so
            # the driven joints land EXACTLY on their rest pose.  This is
            # deterministic — it does not depend on the current time or a
            # rewind (dt < 0).
            fps = float(cmds.getAttr(f"{solver}.fps"))
            cmds.setAttr(f"{solver}.fps", fps + 0.001)
            step_physics(solver)  # signature changed -> rebuild at rest pose
            cmds.setAttr(f"{solver}.fps", fps)
            step_physics(solver)  # restore fps -> rebuild again (still at rest)
        except Exception as exc:
            log.debug("Could not reset physics solver %s: %s", solver, exc)

    # Restore original ikBlend values so the rig solver state is unchanged.
    for _ik_h, _orig_blend in _ik_blend_orig.items():
        try:
            cmds.setAttr(f"{_ik_h}.ikBlend", _orig_blend)
        except Exception:
            pass

    # Count after reset for return stats
    bones_after = len(
        [
            j
            for j in bone_map.values()
            if cmds.attributeQuery("pmxRestRotateX", node=j, exists=True)
        ]
    )
    ik_handles_after = len(
        [
            h
            for h in _ik_handles
            if cmds.attributeQuery("pmxIkRestRotateX", node=h, exists=True)
        ]
    )

    log.debug("Reset complete: %d bones, %d IK handles", bones_after, ik_handles_after)

    return {"bones_reset": bones_after, "ik_handles_reset": ik_handles_after}
