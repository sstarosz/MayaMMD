"""Viewport visibility control for the parts of an imported PMX model.

A PMX model is built from container groups under its root transform (see
``docs/NamingConventions.md``)::

    {model}_Root
    ├── {model}_Geo            mesh
    ├── {model}_Bones          joints
    └── {model}_RigidBodies
        ├── {model}_Bodies     rigid-body guides
        └── {model}_Joints     rigid-body joints (constraints)

Each container is a plain transform, so toggling its ``visibility`` attribute
hides or shows every node of that kind at once: Maya inherits visibility down
the DAG.  Hiding rigid bodies/joints only affects drawing — the solver is a DG
node driven by ``time1``, so the simulation keeps running.

Containers are resolved structurally (by the shape types they hold) instead of
by name, so duplicated or renamed imports still work.
"""

from __future__ import annotations

import logging

from maya import cmds

log = logging.getLogger(__name__)

# ── Part keys ──────────────────────────────────────────────────────────────

MESH = "mesh"
BONES = "bones"
RIGID_BODIES = "rigid_bodies"
JOINTS = "joints"

PARTS = (MESH, BONES, RIGID_BODIES, JOINTS)

PART_LABELS = {
    MESH: "Mesh",
    BONES: "Bones",
    RIGID_BODIES: "Rigid Bodies",
    JOINTS: "Joints",
}

DEFAULT_VISIBILITY = {
    MESH: True,
    BONES: True,
    RIGID_BODIES: False,
    JOINTS: False,
}

_TYPE_MESH = "mesh"
_TYPE_JOINT = "joint"
_TYPE_RIGID_BODY_SOLVER = "pmxRigidBodyNode"
_TYPE_RIGID_BODY_SHAPE = "pmxRigidBodyShape"
_TYPE_RIGID_BODY_JOINT = "pmxRigidBodyJoint"

# The node type that *is* each part.  Everything else under a container is
# import scaffolding (``_InheritCtrl``, ``_MorphCtrl``, IK handles, ...) which
# is hidden on purpose and must not be counted or revealed.
_PART_NODE_TYPES = {
    MESH: _TYPE_MESH,
    BONES: _TYPE_JOINT,
    RIGID_BODIES: _TYPE_RIGID_BODY_SHAPE,
    JOINTS: _TYPE_RIGID_BODY_JOINT,
}


# ── Scene queries ──────────────────────────────────────────────────────────


def _children(node: str) -> list[str]:
    """Full DAG paths of ``node``'s direct children (``[]`` on failure)."""
    try:
        return cmds.listRelatives(node, children=True, fullPath=True) or []
    except RuntimeError:
        return []


def _hidden_among(nodes: list[str]) -> int:
    """How many of ``nodes`` have their *own* ``visibility`` switched off."""
    if not nodes:
        return 0
    try:
        values = cmds.getAttr([f"{node}.visibility" for node in nodes])
    except RuntimeError:
        return 0
    return sum(1 for value in values if value is False)


def _part_nodes(container: str, part: str) -> list[str]:
    """The DAG nodes that *are* ``part`` (joints, or the shapes' transforms).

    Internal helpers under the container (``_InheritCtrl``, ``_MorphCtrl``,
    IK handles) are hidden by the importer on purpose, so they are excluded -
    they are neither reported as "hidden individually" nor revealed.
    """
    wanted = _PART_NODE_TYPES[part]
    try:
        shapes = (
            cmds.listRelatives(
                container, allDescendents=True, type=wanted, fullPath=True
            )
            or []
        )
    except RuntimeError:
        return []
    if not shapes:
        return []

    if part == BONES:
        # Joints are their own transform, so they are the nodes to look at.
        return shapes

    # Every other part node is a shape; the transform holding it is what the
    # Outliner hides, so both count.
    try:
        parents = cmds.listRelatives(shapes, parent=True, fullPath=True) or []
    except RuntimeError:
        parents = []
    return list(dict.fromkeys([*shapes, *parents]))


def _holds_shape(node: str, shape_type: str) -> bool:
    """``True`` if ``node`` owns or has a descendant of ``shape_type``."""
    try:
        if cmds.listRelatives(node, shapes=True, type=shape_type):
            return True
        return bool(cmds.listRelatives(node, allDescendents=True, type=shape_type))
    except RuntimeError:
        return False


def _find_container(parent: str, shape_type: str) -> str | None:
    """First direct child of ``parent`` that holds a ``shape_type`` shape."""
    return next((c for c in _children(parent) if _holds_shape(c, shape_type)), None)


def resolve_part_nodes(root_name: str) -> dict[str, str | None]:
    """Map each part to the DAG node whose ``visibility`` controls it.

    Args:
        root_name: Full DAG path of a PMX model root.

    Returns:
        Dict keyed by :data:`PARTS`; missing parts map to ``None``.
    """
    nodes: dict[str, str | None] = dict.fromkeys(PARTS)
    if not root_name or not cmds.objExists(root_name):
        return nodes

    nodes[MESH] = _find_container(root_name, _TYPE_MESH)
    nodes[BONES] = _find_container(root_name, _TYPE_JOINT)

    physics = _find_container(root_name, _TYPE_RIGID_BODY_SOLVER)
    if physics:
        nodes[RIGID_BODIES] = _find_container(physics, _TYPE_RIGID_BODY_SHAPE)
        nodes[JOINTS] = _find_container(physics, _TYPE_RIGID_BODY_JOINT)
    return nodes


def _read_visibility(node: str) -> bool | None:
    """Current ``visibility`` of ``node``, or ``None`` when it cannot be read."""
    try:
        return bool(cmds.getAttr(f"{node}.visibility"))
    except RuntimeError as exc:
        log.debug("Could not read visibility of %s: %s", node, exc)
        return None


def get_part_visibility(root_name: str) -> dict[str, bool | None]:
    """Current visibility of every part of ``root_name``.

    Returns:
        Dict keyed by :data:`PARTS`; parts the model does not have map to
        ``None``.
    """
    return {
        part: _read_visibility(node) if node else None
        for part, node in resolve_part_nodes(root_name).items()
    }


def _set_visibility(node: str, visible: bool) -> bool:
    """Write ``visibility`` on ``node``; ``False`` if Maya rejected it."""
    try:
        cmds.setAttr(f"{node}.visibility", visible)
    except RuntimeError as exc:
        log.warning("Could not set visibility of %s: %s", node, exc)
        return False
    return True


def set_part_visibility(root_name: str, part: str, visible: bool) -> bool:
    """Show or hide one part of ``root_name``.

    Args:
        root_name: Full DAG path of a PMX model root.
        part: One of :data:`PARTS`.
        visible: ``True`` to show the part, ``False`` to hide it.

    Returns:
        ``True`` when the part was toggled, ``False`` when the model does not
        have it (or Maya rejected the change).
    """
    if part not in PARTS:
        raise ValueError(f"Unknown model part: {part!r}")

    node = resolve_part_nodes(root_name).get(part)
    if not node:
        log.debug("Model '%s' has no %s to toggle", root_name, part)
        return False

    return _set_visibility(node, visible)


def set_all_parts_visibility(root_name: str, visible: bool) -> int:
    """Show or hide every part of ``root_name`` at once.

    Resolves the containers once, like :func:`apply_default_visibility`.
    Nodes hidden *individually* keep their own ``visibility``: hiding a
    container already hides them, and showing it must not undo a deliberate
    per-node choice (Maya's display layers behave the same way).  Use
    :func:`reveal_all` when everything, including those, should reappear.

    Returns:
        How many containers were written (0 when the model is gone).
    """
    return sum(
        _set_visibility(node, visible)
        for node in resolve_part_nodes(root_name).values()
        if node
    )


def hidden_descendant_counts(root_name: str) -> dict[str, int]:
    """Per part, how many of its nodes are hidden by their own ``visibility``.

    A part toggle only writes its container.  Nodes hidden individually (e.g.
    a joint whose eye icon was switched off by hand) therefore stay hidden
    when the part is shown again - correct, but the panel has to say so.
    Import scaffolding that is hidden on purpose is not counted.

    Args:
        root_name: Full DAG path of a PMX model root.

    Returns:
        Dict keyed by :data:`PARTS`; parts the model does not have report 0.
    """
    counts: dict[str, int] = {}
    for part, container in resolve_part_nodes(root_name).items():
        counts[part] = _hidden_among(_part_nodes(container, part)) if container else 0
    return counts


def reveal_all(root_name: str) -> int:
    """Make every part, and every node inside it, visible.

    Unlike :func:`set_part_visibility` this also clears per-node hiding, so
    nodes hidden individually reappear - the "show everything" reset behind
    the panel's Show All button.  Import scaffolding that is hidden on purpose
    stays hidden.

    Returns:
        How many attributes were switched back on (0 when nothing was hidden).
    """
    changed = 0
    for part, container in resolve_part_nodes(root_name).items():
        if not container:
            continue
        targets = [container, *_part_nodes(container, part)]
        try:
            values = cmds.getAttr([f"{target}.visibility" for target in targets])
        except RuntimeError:
            values = [True] * len(targets)
        for target, value in zip(targets, values):
            if value is False and _set_visibility(target, True):
                changed += 1
    return changed


def apply_default_visibility(root_name: str) -> dict[str, bool | None]:
    """Apply :data:`DEFAULT_VISIBILITY` to ``root_name``.

    Resolves the containers once instead of once per part: a fresh resolution
    is a set of descendant scans, and import already walks the scene heavily.

    Returns:
        The resulting per-part visibility (see :func:`get_part_visibility`).
    """
    states: dict[str, bool | None] = {}
    for part, node in resolve_part_nodes(root_name).items():
        if not node:
            states[part] = None
            continue
        _set_visibility(node, DEFAULT_VISIBILITY[part])
        states[part] = _read_visibility(node)
    return states
