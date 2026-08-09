"""Selection-driven, lazy-evaluated context for the active PMX model.

This module provides the :class:`ModelContext` class which tracks which PMX model
the user has targeted (via Maya scene selection) and lazily discovers all model
data from the scene through custom attributes stored during PMX import.

Usage::

    ctx = ModelContext()
    ctx.modelChanged.connect(on_model_changed)

    # On selection change:
    ctx.refresh_from_selection()

    # Lazy getters (query scene once, cached until next refresh):
    bone_map = ctx.boneMap()
    morph_map = ctx.morphMap()
    ik_handles = ctx.ikHandles()
    bone_morph_node = ctx.boneMorphNode()
    blend_shape_node = ctx.blendShapeNode()
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List

import maya.cmds as cmds
from PySide6.QtCore import QObject, Signal

from mmd.maya.pmx_model_utils import (
    build_bone_map_from_scene,
    build_morph_map_from_scene,
    find_blend_shape_node,
    find_bone_morph_node,
    find_ik_handles,
    find_model_root_from_selection,
    find_physics_driven_joints,
    find_physics_group,
    find_physics_node,
    find_physics_rigid_bodies,
)

# Sentinel used to distinguish "failed to resolve" from "empty result" in the
# lazy-getter cache.  If a getter raises, we store this sentinel so we don't
# retry the expensive scene query on every subsequent call.
_SENTINEL = object()

log = logging.getLogger(__name__)


class ModelContext(QObject):
    """Selection-driven, lazy-evaluated context for the active PMX model.

    The context tracks which PMX model is currently targeted (based on Maya
    scene selection) and provides lazy getters that query the scene once and
    cache results until the active model changes.

    Emits :attr:`modelChanged` when the active model switches.
    """

    modelChanged = Signal(str)
    """Emitted when the active model root changes.

    The slot receives the new root name (``str``) or ``""`` when no
    model is targeted.
    """

    def __init__(self, parent: QObject = None) -> None:
        super().__init__(parent=parent)
        self._root_name: str = ""
        self._cache: Dict[str, Any] = {}

    # ── Public properties ────────────────────────────────────────────────

    @property
    def rootName(self) -> str:
        """Name of the active PMX model root transform (``""`` if none)."""
        return self._root_name

    @property
    def isValid(self) -> bool:
        """``True`` when a valid PMX model root is currently targeted."""
        return bool(self._root_name) and cmds.objExists(self._root_name)

    # ── Lifecycle ────────────────────────────────────────────────────────

    def refresh_from_selection(self) -> bool:
        """Re-detect the active model from the current Maya selection.

        Returns:
            ``True`` if the active model changed, ``False`` otherwise.
        """
        root = find_model_root_from_selection()
        if root == self._root_name:
            return False
        self._set_active(root or "")
        return True

    def set_active_root(self, root_name: str) -> None:
        """Manually set the active model root (for dropdown or testing).

        Args:
            root_name: Full DAG path of a PMX model root, or ``""`` to clear.
        """
        if root_name != self._root_name:
            self._set_active(root_name)

    def clear(self) -> None:
        """Clear the active model and all cached data."""
        if self._root_name:
            self._set_active("")

    def invalidate_cache(self) -> None:
        """Force all lazy getters to re-query the scene on next access."""
        self._cache.clear()

    # ── Lazy getters ─────────────────────────────────────────────────────
    #
    # Each getter follows the same pattern:
    #   1. Cache lookup — return cached value if already resolved (including
    #      sentinels) to avoid repeating expensive scene queries.
    #   2. Scene query — wrapped in try/except so a single broken custom
    #      attribute doesn't take down the whole operation silently.
    #   3. Cache the result (or sentinel on failure) and return.

    def boneMap(self) -> Dict[str, str]:
        """PMX-name → Maya-joint-name mapping for the active model.

        Lazily queries scene joints for ``pmxNameLocal`` / ``pmxNameUniversal``.
        Returns ``{}`` if the root has no joints or the query fails.
        """
        if "bone_map" not in self._cache and self._root_name:
            try:
                self._cache["bone_map"] = build_bone_map_from_scene(self._root_name)
            except Exception as exc:
                log.error("Failed to build bone map for %s: %s", self._root_name, exc)
                self._cache["bone_map"] = _SENTINEL
        result = self._cache.get("bone_map", {})
        return {} if result is _SENTINEL else result

    def morphMap(self) -> Dict[str, str]:
        """PMX-morph-name → Maya-blendShape-alias mapping for the active model.

        Reads the ``pmxMorphMapping`` compound attribute from the blendShape node.
        Returns ``{}`` if no blendShape exists or the query fails.
        """
        if "morph_map" not in self._cache and self._root_name:
            try:
                self._cache["morph_map"] = build_morph_map_from_scene(self._root_name)
            except Exception as exc:
                log.error("Failed to build morph map for %s: %s", self._root_name, exc)
                self._cache["morph_map"] = _SENTINEL
        result = self._cache.get("morph_map", {})
        return {} if result is _SENTINEL else result

    def boneMorphNode(self) -> str:
        """Name of the ``boneMorphNode`` DG node, or ``""`` if none exists.

        Returns ``""`` if the query fails (caller should treat "no bone morph
        node" and "failed to check" the same way).
        """
        if "bone_morph" not in self._cache and self._root_name:
            try:
                bone_morph = find_bone_morph_node(self._root_name)
                self._cache["bone_morph"] = bone_morph or ""
            except Exception as exc:
                log.error(
                    "Failed to find boneMorphNode for %s: %s",
                    self._root_name,
                    exc,
                )
                self._cache["bone_morph"] = _SENTINEL
        result = self._cache.get("bone_morph", "")
        return "" if result is _SENTINEL else result

    def blendShapeNode(self) -> str:
        """Name of the ``blendShape`` deformer node, or ``""`` if none exists.

        Returns ``""`` if the query fails (caller should treat "no blend shape
        node" and "failed to check" the same way).
        """
        if "blend_shape" not in self._cache and self._root_name:
            try:
                blend_shape = find_blend_shape_node(self._root_name)
                self._cache["blend_shape"] = blend_shape or ""
            except Exception as exc:
                log.error(
                    "Failed to find blendShape node for %s: %s",
                    self._root_name,
                    exc,
                )
                self._cache["blend_shape"] = _SENTINEL
        result = self._cache.get("blend_shape", "")
        return "" if result is _SENTINEL else result

    def ikHandles(self) -> List[str]:
        """List of IK handle names belonging to the active model.

        Returns ``[]`` if the root has no IK handles or the query fails.
        """
        if "ik_handles" not in self._cache and self._root_name:
            try:
                self._cache["ik_handles"] = find_ik_handles(self._root_name)
            except Exception as exc:
                log.error("Failed to find IK handles for %s: %s", self._root_name, exc)
                self._cache["ik_handles"] = _SENTINEL
        result = self._cache.get("ik_handles", [])
        return [] if result is _SENTINEL else result

    def physicsGroup(self) -> str:
        """Name of the active model's physics group transform ("" if none)."""
        if "physics_group" not in self._cache and self._root_name:
            try:
                self._cache["physics_group"] = (
                    find_physics_group(self._root_name) or ""
                )
            except Exception as exc:
                log.error(
                    "Failed to find physics group for %s: %s", self._root_name, exc
                )
                self._cache["physics_group"] = _SENTINEL
        result = self._cache.get("physics_group", "")
        return "" if result is _SENTINEL else result

    def physicsNode(self) -> str:
        """Name of the active model's ``mmdPhysicsNode`` solver ("" if none)."""
        if "physics_node" not in self._cache and self._root_name:
            try:
                self._cache["physics_node"] = (
                    find_physics_node(self._root_name) or ""
                )
            except Exception as exc:
                log.error(
                    "Failed to find physics node for %s: %s", self._root_name, exc
                )
                self._cache["physics_node"] = _SENTINEL
        result = self._cache.get("physics_node", "")
        return "" if result is _SENTINEL else result

    def physicsRigidBodies(self) -> Dict[int, str]:
        """Map of PMX rigid-body index -> related joint for the active model.

        Returns ``{}`` if the model has no physics.
        """
        if "physics_bodies" not in self._cache and self._root_name:
            try:
                self._cache["physics_bodies"] = find_physics_rigid_bodies(
                    self._root_name
                )
            except Exception as exc:
                log.error(
                    "Failed to find physics bodies for %s: %s", self._root_name, exc
                )
                self._cache["physics_bodies"] = _SENTINEL
        result = self._cache.get("physics_bodies", {})
        return {} if result is _SENTINEL else result

    def physicsDrivenJoints(self) -> Dict[int, str]:
        """Map of PMX rigid-body index -> joint driven by the node (write-back).

        Phase 3: the node writes the solved pose directly into these joints
        (no write-back constraints exist any more).  Returns ``{}`` if the
        model has no physics.
        """
        if "physics_driven" not in self._cache and self._root_name:
            try:
                self._cache["physics_driven"] = find_physics_driven_joints(
                    self._root_name
                )
            except Exception as exc:
                log.error(
                    "Failed to find physics-driven joints for %s: %s",
                    self._root_name,
                    exc,
                )
                self._cache["physics_driven"] = _SENTINEL
        result = self._cache.get("physics_driven", {})
        return {} if result is _SENTINEL else result

    # ── Resolve ──────────────────────────────────────────────────────────

    def resolve(self) -> "ResolvedModelData":
        """Eagerly evaluate all lazy getters and return a snapshot.

        Raises:
            ValueError: If the context has no active model (:attr:`rootName`
                is ``""``).  Callers should guard with :attr:`isValid` or
                :meth:`_ensure_model_targeted`.

        Returns:
            :class:`~mmd.maya.maya_data_types.ResolvedModelData` with all
            fields populated from scene custom attributes.
        """
        if not self._root_name:
            raise ValueError("ModelContext has no active model — cannot resolve data")
        # Import here to avoid circular dependency at module level.
        from mmd.maya.maya_data_types import ResolvedModelData

        return ResolvedModelData(
            root_name=self._root_name,
            bone_map=self.boneMap(),
            morph_map=self.morphMap(),
            blend_shape_node=self.blendShapeNode(),
            bone_morph_node=self.boneMorphNode(),
            ik_handles=self.ikHandles(),
        )

    # ── Internal ─────────────────────────────────────────────────────────

    def _set_active(self, root_name: str) -> None:
        self._root_name = root_name
        self._cache.clear()
        try:
            self.modelChanged.emit(root_name)
        except Exception as exc:
            log.warning("ModelContext listener error: %s", exc)
