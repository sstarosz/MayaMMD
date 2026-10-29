"""Selection-neutral scene edits for the importers.

Several Maya commands select what they touch: ``createNode`` and
``shadingNode`` select the node they create, ``parent`` selects what it
moves, and ``ikHandle`` / ``skinCluster`` select their result.  A PMX import
runs those commands hundreds of times, so that side effect used to leave the
last batch of nodes (e.g. every rigid-body joint) selected after an import
and to feed the selection-driven viewport overrides a selection the user
never made.

The helpers here perform the same edits while leaving the active selection
alone.  Node creation uses the documented ``skipSelect`` flag; reparenting
uses ``MDagModifier`` where its semantics match, and only commands that have
no non-selecting form are wrapped in :func:`keep_selection`.
"""

from __future__ import annotations

import logging
from collections.abc import Iterator, Sequence
from contextlib import contextmanager

try:
    import maya.api.OpenMaya as om
    from maya import cmds
except ImportError:  # Imported outside Maya by the pure-logic tests.
    om = None  # type: ignore[assignment]
    cmds = None  # type: ignore[assignment]

log = logging.getLogger(__name__)


@contextmanager
def keep_selection() -> Iterator[None]:
    """Restore the active selection after the block.

    For commands that always select their result and offer no flag to avoid
    it: ``parent`` with world-preservation, ``ikHandle`` and ``skinCluster``.
    Names the block deleted or renamed are dropped, and a restore that fails
    never masks the block's own exception.
    """
    selection = cmds.ls(selection=True, long=True) or []
    try:
        yield
    finally:
        try:
            existing = cmds.ls(selection, long=True) if selection else []
            if existing:
                cmds.select(existing, replace=True)
            else:
                cmds.select(clear=True)
        except Exception:
            log.debug("Could not restore the selection", exc_info=True)


def create_node(node_type: str, **kwargs) -> str:
    """``cmds.createNode`` that leaves the active selection alone."""
    return cmds.createNode(node_type, skipSelect=True, **kwargs)


def create_shading_node(node_type: str, **kwargs) -> str:
    """``cmds.shadingNode`` that leaves the active selection alone."""
    return cmds.shadingNode(node_type, skipSelect=True, **kwargs)


def reparent_preserving_local(nodes: str | Sequence[str], parent_node: str) -> None:
    """``cmds.parent(..., relative=True)`` that leaves the active selection alone.

    ``MDagModifier.reparentNode`` keeps the node's LOCAL transform values and
    selects nothing — exactly the ``relative=True`` behaviour.  It does not
    preserve the world transform, so use :func:`reparent_preserving_world` for
    the ``absolute=True`` case.
    """
    if isinstance(nodes, str):
        nodes = [nodes]
    parent_selection = om.MSelectionList()
    parent_selection.add(parent_node)
    parent_obj = parent_selection.getDependNode(0)
    modifier = om.MDagModifier()
    for node in nodes:
        node_selection = om.MSelectionList()
        node_selection.add(node)
        modifier.reparentNode(node_selection.getDependNode(0), parent_obj)
    modifier.doIt()


def reparent_preserving_world(node: str, parent_node: str) -> None:
    """``cmds.parent(..., absolute=True)`` that leaves the active selection alone.

    ``cmds.parent`` selects what it moves and has no flag to avoid it, and
    ``MDagModifier.reparentNode`` does not reproduce its world-preserving
    handling of joints (``jointOrient`` is recomputed, local values change),
    so the selection is restored around the command instead.
    """
    with keep_selection():
        cmds.parent(node, parent_node, absolute=True)
