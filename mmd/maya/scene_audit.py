"""
Scene audit utilities for testing and debugging PMX imports.

Provides a before/after snapshot pattern so you can validate what was
created during ``build_pmx_scene()`` without adding audit logic to
production builder code.

Typical usage in integration tests::

    from mmd.maya.scene_audit import SceneSnapshot
    from mmd.maya.pmx_scene_builder import build_pmx_scene

    pmx_data = parse_pmx("model.pmx")

    before = SceneSnapshot.take()

    maya_data = build_pmx_scene(pmx_data)

    after = SceneSnapshot.take()
    diff = after - before

    # Check categories
    assert "PMX_" in diff.transforms[0]           # root named correctly
    assert len(diff.nodes_of_type("joint")) == len(pmx_data.bones)

    # Print everything created
    print(diff.summary())
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Snapshot data
# ---------------------------------------------------------------------------


@dataclass
class SceneDiff:
    """The set of objects added to a Maya scene between two snapshots."""

    transforms: list[str] = field(default_factory=list)
    """All transform (DAG) nodes that appeared, by full path name."""

    shapes: list[str] = field(default_factory=list)
    """Shape nodes (mesh, nurbs, etc.) that appeared."""

    dag_nodes: list[str] = field(default_factory=list)
    """All DAG nodes (transforms + shapes) that appeared."""

    dg_nodes: list[str] = field(default_factory=list)
    """DG (non-DAG) nodes that appeared (materials, utilities, deformers, …)."""

    all_nodes: set[str] = field(default_factory=set)
    """Every Maya node name that appeared."""

    def nodes_of_type(self, maya_type: str) -> list[str]:
        """Return created node names that match a Maya object type (e.g. ``"joint"``, ``"mesh"``)."""
        from maya import cmds

        return [n for n in sorted(self.all_nodes) if cmds.nodeType(n) == maya_type]

    def count_of_type(self, maya_type: str) -> int:
        """Number of created nodes of a given Maya type."""
        return len(self.nodes_of_type(maya_type))

    def summary(self) -> str:
        """Return a human-readable summary of the diff."""
        lines = ["=== Scene Import Diff ==="]
        lines.append(f"  Transforms: {len(self.transforms)}")
        lines.append(f"  Shapes:     {len(self.shapes)}")
        lines.append(f"  DG nodes:   {len(self.dg_nodes)}")
        lines.append(f"  Total:      {len(self.all_nodes)}")
        if self.transforms:
            lines.append("  Transforms:")
            for n in sorted(self.transforms):
                lines.append(f"    - {n}")
        if self.dg_nodes:
            lines.append("  DG nodes:")
            for n in sorted(self.dg_nodes):
                lines.append(f"    - {n}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Snapshot
# ---------------------------------------------------------------------------


class SceneSnapshot:
    """A point-in-time record of all Maya nodes in the scene.

    Use ``SceneSnapshot.take()`` to capture, then subtract two snapshots
    to get a ``SceneDiff`` of everything created in between.
    """

    def __init__(
        self,
        transforms: set[str],
        shapes: set[str],
        dag_nodes: set[str],
        dg_nodes: set[str],
    ):
        self.transforms = transforms
        self.shapes = shapes
        self.dag_nodes = dag_nodes
        self.dg_nodes = dg_nodes
        self.all_nodes = transforms | shapes | dg_nodes

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @staticmethod
    def take() -> SceneSnapshot:
        """Capture the current state of the Maya scene."""
        from maya import cmds

        # All DAG nodes (transforms + shapes)
        dag_all = set(cmds.ls(dagObjects=True, long=True) or [])

        # Separate transforms (including joints) from shapes
        transforms = set()
        shapes = set()
        for node in dag_all:
            ntype = cmds.nodeType(node)
            if ntype in ("transform", "joint"):
                transforms.add(node)
            else:
                shapes.add(node)

        # All non-DAG DG nodes (materials, utilities, deformers, …)
        # Exclude DAG nodes (already counted), default nodes, and UI nodes.
        all_nodes = set(cmds.ls(dagObjects=False, long=True) or [])
        dg_nodes = all_nodes - dag_all

        return SceneSnapshot(
            transforms=transforms,
            shapes=shapes,
            dag_nodes=dag_all,
            dg_nodes=dg_nodes,
        )

    # ------------------------------------------------------------------
    # Operators
    # ------------------------------------------------------------------

    def __sub__(self, other: SceneSnapshot) -> SceneDiff:
        """Return everything that exists in *self* but not in *other*."""
        return SceneDiff(
            transforms=sorted(self.transforms - other.transforms),
            shapes=sorted(self.shapes - other.shapes),
            dag_nodes=sorted(self.dag_nodes - other.dag_nodes),
            dg_nodes=sorted(self.dg_nodes - other.dg_nodes),
            all_nodes=self.all_nodes - other.all_nodes,
        )

    def __repr__(self) -> str:
        return (
            f"SceneSnapshot("
            f"transforms={len(self.transforms)}, "
            f"shapes={len(self.shapes)}, "
            f"dg_nodes={len(self.dg_nodes)})"
        )


# ---------------------------------------------------------------------------
# Convenience helpers for common validation patterns
# ---------------------------------------------------------------------------


def diff_after_import(pmx_data, build_fn) -> SceneDiff:
    """Convenience: take a before snapshot, run *build_fn(pmx_data)*,
    take an after snapshot, and return the diff.

    Args:
        pmx_data: Parsed PMX model data.
        build_fn: Callable that takes *pmx_data* and builds the Maya scene
                  (typically ``build_pmx_scene``).

    Returns:
        SceneDiff of everything created during the build.
    """
    before = SceneSnapshot.take()
    build_fn(pmx_data)
    after = SceneSnapshot.take()
    return after - before
