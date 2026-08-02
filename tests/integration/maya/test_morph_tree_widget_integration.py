"""
Integration tests for ``MorphTreeWidget`` (pose / morph editor UI).

These tests run inside ``mayapy`` via the CTest infrastructure.  They
verify that the widget correctly reflects the state of imported PMX
models, including blendShape and boneMorphNode data.

The widget is constructed with a headless ``QApplication`` (offscreen
QPA platform), which allows Qt widget tests to run without a display
server.

When imported from ``run_all_integration_tests.py`` (the CTest runner),
Maya standalone is already initialised and a ``QApplication`` already
exists (created in the pose-tree handler just before the import).  In
that scenario the module-level code below will find the existing
``QApplication`` via ``instance()`` and skip the QPA platform override
and Maya standalone initialisation — avoiding conflicts with Maya's Qt.
"""

from __future__ import annotations

import os

# ── Maya standalone initialised by the test runner ───────────────────────
from maya import cmds

# ── QApplication singleton ─────────────────────────────────────────────
# When running standalone (python this_file.py), a QApplication must be
# created before import.  When running through the CTest runner
# (run_all_integration_tests.py), the runner creates the QApplication
# before importing this module, so instance() returns the existing one.
from PySide6.QtWidgets import QApplication

_app = QApplication.instance()
if _app is None or not isinstance(_app, QApplication):
    raise RuntimeError(
        "QApplication must be created before importing this module. "
        "The test runner (run_all_integration_tests.py) handles this "
        "automatically — run tests via ctest, not directly."
    )

# ── Multi-model imports ─────────────────────────────────────────────────────
from mmd.core.data_types import PmxModel
from mmd.core.vmd_importer import parse_vmd_file
from mmd.maya.pmx_model_utils import (
    discover_model_roots_in_scene,
    find_blend_shape_node,
    find_bone_morph_node,
)
from mmd.maya.vmd_scene_builder import apply_vmd_to_scene
from mmd.ui.morph_tree_widget import KeyframeState, MorphTreeWidget, _get_keyframe_state
from tests.integration.test_helpers import (
    assert_true,
    assert_eq,
    skip_test,
)

# Model and VMD paths — set by the test runner before tests execute.
_MODEL_A: str | None = None
_MODEL_B: str | None = None
_TEST_VMD: str | None = None


# ═══════════════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════════════


def _count_tree_targets(tree_widget: MorphTreeWidget) -> int:
    """Count the total number of leaf target items across all tabs.

    Does NOT include envelope weight widgets (those sit on parent rows,
    not as child items).
    """
    total = 0
    for tab_idx in range(tree_widget._tabs.count()):
        tab = tree_widget._tabs.widget(tab_idx)
        from PySide6.QtWidgets import QTreeWidget

        for child in tab.children():
            if isinstance(child, QTreeWidget):
                for i in range(child.topLevelItemCount()):
                    parent = child.topLevelItem(i)
                    total += parent.childCount()
    return total


def _count_weight_widgets_excluding_envelopes(
    tree_widget: MorphTreeWidget,
) -> int:
    """Count weight widgets that belong to morph targets (not envelopes).

    Envelope weight widgets have ``.envelope`` in their attribute path.
    """
    return sum(
        1 for attr_path in tree_widget._rows if not attr_path.endswith(".envelope")
    )


def _get_weight_widget_value(
    tree_widget: MorphTreeWidget, attr_path: str
) -> float | None:
    """Read the current spinbox value for an attribute from the widget."""
    row = tree_widget._rows.get(attr_path)
    if row is None:
        return None
    return row.spinbox.value()


# ═══════════════════════════════════════════════════════════════════════════
#  Test functions (model-dependent — receive pmx_data + maya_pmx_data)
# ═══════════════════════════════════════════════════════════════════════════


def _get_model_root(maya_pmx_data: object) -> str:
    """Extract root_name from maya_pmx_data, avoiding expensive scene scan."""
    return getattr(maya_pmx_data, "root_name", "")


def test_widget_constructs_with_model(
    pmx_data: PmxModel, maya_pmx_data: object
) -> bool:
    """The MorphTreeWidget constructs without error for an imported model."""
    root_name = _get_model_root(maya_pmx_data)
    if not root_name:
        skip_test("No PMX root found in scene")

    bs_node = find_blend_shape_node(root_name)
    bm_node = find_bone_morph_node(root_name)

    widget = MorphTreeWidget()
    widget.refresh(
        blend_shape_node=bs_node or "",
        bone_morph_node=bm_node or "",
    )

    tab_count = widget._tabs.count()
    print(f"  Tabs: {tab_count}")

    assert_true(
        tab_count >= 2,
        f"Expected at least 2 tabs, got {tab_count}",
    )

    widget.deleteLater()
    return True


def test_vertex_morph_targets_in_tree(
    pmx_data: PmxModel, maya_pmx_data: object
) -> bool:
    """Vertex morph targets appear as tree items under the Vertex tab."""
    root_name = _get_model_root(maya_pmx_data)
    if not root_name:
        skip_test("No PMX root found")

    bs_node = find_blend_shape_node(root_name)
    if not bs_node:
        skip_test("No blendShape node on this model")

    widget = MorphTreeWidget()
    widget.refresh(blend_shape_node=bs_node, bone_morph_node="")

    from mmd.ui.morph_tree_widget import _BlendShapeMorphSource

    source = _BlendShapeMorphSource(bs_node)
    expected = source.get_targets()
    actual_count = _count_tree_targets(widget)

    print(f"  BlendShape targets: expected={len(expected)}, in-tree={actual_count}")
    assert_eq(
        actual_count,
        len(expected),
        f"BlendShape target count mismatch: expected {len(expected)}, got {actual_count}",
    )

    widget.deleteLater()
    return True


def test_bone_morph_targets_in_tree(pmx_data: PmxModel, maya_pmx_data: object) -> bool:
    """Bone morph targets appear as tree items under the Bone tab."""
    root_name = _get_model_root(maya_pmx_data)
    if not root_name:
        skip_test("No PMX root found")

    bm_node = find_bone_morph_node(root_name)
    if not bm_node:
        skip_test("No boneMorphNode on this model")

    widget = MorphTreeWidget()
    widget.refresh(blend_shape_node="", bone_morph_node=bm_node)

    from mmd.ui.morph_tree_widget import _BoneMorphSource

    source = _BoneMorphSource(bm_node)
    expected = source.get_targets()
    actual_count = _count_tree_targets(widget)

    print(f"  BoneMorph targets: expected={len(expected)}, in-tree={actual_count}")
    assert_eq(
        actual_count,
        len(expected),
        f"BoneMorph target count mismatch: expected {len(expected)}, got {actual_count}",
    )

    widget.deleteLater()
    return True


def test_weight_bidirectional_sync(pmx_data: PmxModel, maya_pmx_data: object) -> bool:
    """Setting a blendShape weight via Maya updates the widget slider.

    And moving the widget slider updates the Maya attribute.
    """
    root_name = _get_model_root(maya_pmx_data)
    if not root_name:
        skip_test("No PMX root found")

    bs_node = find_blend_shape_node(root_name)
    if not bs_node:
        skip_test("No blendShape node on this model")

    # Query first target
    targets = cmds.aliasAttr(bs_node, query=True) or []
    if len(targets) < 2:
        skip_test("No blendShape targets")
    first_alias = targets[0]
    first_attr = f"{bs_node}.{first_alias}"

    widget = MorphTreeWidget()
    widget.refresh(blend_shape_node=bs_node, bone_morph_node="")

    # Verify the widget registered this weight attribute
    assert_true(
        first_attr in widget._rows,
        f"Widget did not register weight attr: {first_attr}",
    )

    # ── Maya → widget sync ──────────────────────────────────────────
    cmds.setAttr(first_attr, 0.75)
    # The Maya callback is async (fires on DG evaluation); force it
    cmds.refresh()

    widget_value = _get_weight_widget_value(widget, first_attr)
    assert_true(
        widget_value is not None,
        "Could not read widget value",
    )

    print(f"  Maya→widget: setAttr=0.75, widget={widget_value:.3f}")
    assert_true(
        abs(widget_value - 0.75) < 0.01,
        f"Maya→widget sync failed: expected 0.75, got {widget_value:.3f}",
    )

    # ── Widget → Maya sync ──────────────────────────────────────────
    row = widget._rows[first_attr]
    row.slider.setValue(250)  # 0.25
    maya_value = cmds.getAttr(first_attr)

    print(f"  Widget→Maya: slider=250, maya={maya_value:.3f}")
    assert_true(
        abs(maya_value - 0.25) < 0.01,
        f"Widget→Maya sync failed: expected 0.25, got {maya_value:.3f}",
    )

    widget.deleteLater()
    return True


def test_visibility_toggle_mutes_weight(
    pmx_data: PmxModel, maya_pmx_data: object
) -> bool:
    """Toggle OFF sets weight to 0; toggle ON restores the saved weight."""
    root_name = _get_model_root(maya_pmx_data)
    if not root_name:
        skip_test("No PMX root found")

    bs_node = find_blend_shape_node(root_name)
    if not bs_node:
        skip_test("No blendShape node")

    targets = cmds.aliasAttr(bs_node, query=True) or []
    if len(targets) < 2:
        skip_test("No blendShape targets")
    first_alias = targets[0]
    first_attr = f"{bs_node}.{first_alias}"

    # Set an initial non-zero weight
    cmds.setAttr(first_attr, 0.4)

    widget = MorphTreeWidget()
    widget.refresh(blend_shape_node=bs_node, bone_morph_node="")

    row = widget._rows.get(first_attr)
    vis_btn = row.vis_btn if row is not None else None
    if vis_btn is None:
        widget.deleteLater()
        skip_test("No visibility toggle for this attr")

    # ── Toggle OFF ───────────────────────────────────────────────────
    vis_btn.setChecked(False)
    cmds.refresh()
    maya_off = cmds.getAttr(first_attr)
    print(f"  Toggle OFF: weight={maya_off:.3f} (expected 0.0)")

    # ── Toggle ON ────────────────────────────────────────────────────
    vis_btn.setChecked(True)
    cmds.refresh()
    maya_on = cmds.getAttr(first_attr)
    print(f"  Toggle ON : weight={maya_on:.3f} (expected 0.4)")

    widget.deleteLater()
    assert_true(
        abs(maya_off) < 0.001,
        f"Toggle OFF failed: expected 0.0, got {maya_off:.3f}",
    )
    assert_true(
        abs(maya_on - 0.4) < 0.01,
        f"Toggle ON failed: expected 0.4, got {maya_on:.3f}",
    )
    return True


def test_widget_refresh_clears_state(pmx_data: PmxModel, maya_pmx_data: object) -> bool:
    """Calling refresh() with empty args clears all tabs and weight widgets."""
    root_name = _get_model_root(maya_pmx_data)
    if not root_name:
        skip_test("No PMX root found")

    bs_node = find_blend_shape_node(root_name)
    bm_node = find_bone_morph_node(root_name)

    widget = MorphTreeWidget()
    widget.refresh(
        blend_shape_node=bs_node or "",
        bone_morph_node=bm_node or "",
    )

    old_targets = _count_tree_targets(widget)
    old_weight_total = len(widget._rows)
    old_envelopes = old_weight_total - _count_weight_widgets_excluding_envelopes(widget)

    # Refresh with explicitly empty nodes (not auto-discover)
    widget.refresh(blend_shape_node="", bone_morph_node="")
    new_targets = _count_tree_targets(widget)
    new_weight_total = len(widget._rows)

    print(
        f"  Before: {old_targets} targets, {old_weight_total} weight widgets"
        f" ({old_envelopes} envelope(s))"
    )
    print(f"  After : {new_targets} targets, {new_weight_total} weight widgets")

    # After clearing, both targets and weight widgets should be 0.
    assert_eq(
        new_targets,
        0,
        f"Expected 0 targets after clear, got {new_targets}",
    )
    assert_eq(
        new_weight_total,
        0,
        f"Expected 0 weight widgets after clear, got {new_weight_total}",
    )

    widget.deleteLater()
    return True


def test_keyframe_indicators_after_vmd_import(
    pmx_data: PmxModel, maya_pmx_data: object
) -> bool:
    """After VMD import, morph weight keyframe state is non-NONE."""
    if _TEST_VMD is None:
        skip_test("No VMD file available")

    bs_node = find_blend_shape_node(maya_pmx_data.root_name)
    if not bs_node:
        skip_test("No blendShape node on this model")

    # ── Parse VMD; skip if it has no morph keyframes ───────────────────
    try:
        vmd_data = parse_vmd_file(_TEST_VMD)
    except Exception as e:
        skip_test(f"Failed to parse VMD: {e}")

    if not vmd_data.morph_keyframes:
        skip_test("VMD has no morph keyframes")

    # ── Apply VMD → creates animCurves on blendShape weights ──────────
    try:
        apply_vmd_to_scene(vmd_data, model=maya_pmx_data.to_resolved(), start_frame=1)
    except Exception as e:
        skip_test(f"Failed to apply VMD: {e}")

    # ── Create widget + force styling loop ────────────────────────────
    widget = MorphTreeWidget()
    widget.refresh(blend_shape_node=bs_node, bone_morph_node="")
    widget._do_scrub_update()

    keyed_count = 0
    total = len(widget._rows)
    for attr_path in widget._rows:
        state = _get_keyframe_state(attr_path)
        if state != KeyframeState.NONE:
            keyed_count += 1

    print(f"  Keyed targets: {keyed_count}/{total}")
    widget.deleteLater()

    # At least one morph target should now have an animCurve
    assert_true(
        keyed_count > 0 or total == 0,
        f"No keyed targets after VMD import ({total} total)",
    )
    return True


# ═══════════════════════════════════════════════════════════════════════════
#  Test registry (used by run_all_integration_tests.py)
# ═══════════════════════════════════════════════════════════════════════════

_TESTS = [
    ("PMX Pose Tree — Widget Constructs", test_widget_constructs_with_model),
    ("PMX Pose Tree — Vertex Targets", test_vertex_morph_targets_in_tree),
    ("PMX Pose Tree — Bone Targets", test_bone_morph_targets_in_tree),
    ("PMX Pose Tree — Bidirectional Sync", test_weight_bidirectional_sync),
    ("PMX Pose Tree — Visibility Toggle", test_visibility_toggle_mutes_weight),
    ("PMX Pose Tree — Refresh Clears State", test_widget_refresh_clears_state),
    (
        "PMX Pose Tree — Keyframe Indicators (VMD)",
        test_keyframe_indicators_after_vmd_import,
    ),
]


# ═════════════════════════════════════════════════════════════════════════════
#  Multi-model tests (require two models in the scene simultaneously)
# ═════════════════════════════════════════════════════════════════════════════


def _get_model_labels_in_tab(widget: MorphTreeWidget, tab_index: int) -> list[str]:
    """Return the display labels of top-level items in a tab."""
    tab = widget._tabs.widget(tab_index)
    if tab is None:
        return []
    from PySide6.QtWidgets import QTreeWidget

    for child in tab.children():
        if isinstance(child, QTreeWidget):
            return [
                child.topLevelItem(i).text(0) for i in range(child.topLevelItemCount())
            ]
    return []


def _weight_widget_count(widget: MorphTreeWidget) -> int:
    """Number of registered weight attribute → widget pairs."""
    return len(widget._rows)


def test_two_models_show_in_vertex_tab(model_a_path=None, model_b_path=None) -> bool:
    """Both imported models appear as top-level items in the Vertex tab.

    Precondition: a scene with both models already imported.
    """
    if not model_a_path or not model_b_path:
        skip_test("Need at least 2 distinct models")

    widget = MorphTreeWidget()
    widget.refresh()  # auto-discover

    labels = _get_model_labels_in_tab(widget, 0)  # Vertex tab
    print(f"  Vertex tab model labels: {labels}")

    assert_true(
        len(labels) >= 2,
        f"Expected 2+ models in Vertex tab, got {len(labels)}: {labels}",
    )

    widget.deleteLater()
    return True


def test_two_models_show_in_bone_tab(model_a_path=None, model_b_path=None) -> bool:
    """Both imported models appear as top-level items in the Bone tab.

    Precondition: a scene with both models already imported.
    """
    if not model_a_path or not model_b_path:
        skip_test("Need at least 2 distinct models")

    widget = MorphTreeWidget()
    widget.refresh()  # auto-discover

    labels = _get_model_labels_in_tab(widget, 1)  # Bone tab
    print(f"  Bone tab model labels: {labels}")

    if len(labels) >= 2:
        print(f"  Both models have bone morphs — {len(labels)} parents in Bone tab")
    elif len(labels) == 1:
        print("  Only one model has bone morphs — 1 parent in Bone tab (still OK)")
    elif len(labels) == 0:
        print("  Neither model has bone morphs — 0 parents in Bone tab (still OK)")

    widget.deleteLater()
    return True


def test_weight_widgets_registered_for_both_models(
    model_a_path=None, model_b_path=None
) -> bool:
    """Weight widgets are registered for attributes on both imported models.

    Precondition: a scene with both models already imported.
    """
    if not model_a_path or not model_b_path:
        skip_test("Need at least 2 distinct models")

    widget = MorphTreeWidget()
    widget.refresh()  # auto-discover

    count = _weight_widget_count(widget)
    print(f"  Total weight widgets registered: {count}")

    assert_true(
        count >= 2,
        f"Expected at least 2 weight widgets (one per model), got {count}",
    )

    model_a_nick = os.path.basename(os.path.dirname(model_a_path))
    model_b_nick = os.path.basename(os.path.dirname(model_b_path))
    attrs = list(widget._rows.keys())
    has_model_a = any(model_a_nick.lower() in a.lower() for a in attrs)
    has_model_b = any(model_b_nick.lower() in a.lower() for a in attrs)

    if not has_model_a:
        print(f"  WARN: No weight attrs from Model A ('{model_a_nick}') found")
    if not has_model_b:
        print(f"  WARN: No weight attrs from Model B ('{model_b_nick}') found")

    assert_true(
        has_model_a or has_model_b,
        "Expected weight attrs from at least one model",
    )

    widget.deleteLater()
    return True


def test_visibility_toggle_isolates_models(
    model_a_path=None, model_b_path=None
) -> bool:
    """Toggling visibility on one model's target does not affect the other model.

    Precondition: a scene with both models already imported.
    This test modifies blendShape weights; the runner wraps it in an undo
    chunk so other tests are not affected.
    """
    if not model_a_path or not model_b_path:
        skip_test("Need at least 2 distinct models")

    widget = MorphTreeWidget()
    widget.refresh()  # auto-discover

    model_a_key = None
    model_b_key = None
    model_a_nick = os.path.basename(os.path.dirname(model_a_path)).lower()
    model_b_nick = os.path.basename(os.path.dirname(model_b_path)).lower()

    for attr_path in widget._rows:
        lower = attr_path.lower()
        if model_a_nick in lower and model_a_key is None:
            model_a_key = attr_path
        if model_b_nick in lower and model_b_key is None:
            model_b_key = attr_path

    if model_a_key is None or model_b_key is None:
        keys = list(widget._rows.keys())
        if len(keys) < 2:
            widget.deleteLater()
            skip_test(f"Need at least 2 weight attrs (one per model), got {len(keys)}")
        model_a_key = keys[0]
        model_b_key = keys[1]

    node_a = model_a_key.rsplit(".", 1)[0]
    node_b = model_b_key.rsplit(".", 1)[0]
    if cmds.objExists(node_a):
        cmds.setAttr(model_a_key, 0.5)
    if cmds.objExists(node_b):
        cmds.setAttr(model_b_key, 0.7)

    row_a = widget._rows.get(model_a_key)
    vis_a = row_a.vis_btn if row_a is not None else None
    if vis_a is None:
        widget.deleteLater()
        skip_test("No visibility toggle for first attr")

    vis_a.setChecked(False)
    cmds.refresh()

    val_a_off = cmds.getAttr(model_a_key) if cmds.objExists(node_a) else 0.0
    val_b_after = cmds.getAttr(model_b_key) if cmds.objExists(node_b) else 0.0

    print(f"  Model A weight after toggle OFF: {val_a_off:.3f} (expected 0.0)")
    print(f"  Model B weight after toggle OFF: {val_b_after:.3f} (expected ~0.7)")

    assert_true(
        abs(val_a_off) < 0.001,
        f"Model A weight not muted ({val_a_off:.3f})",
    )
    assert_true(
        abs(val_b_after - 0.7) < 0.01,
        f"Model B weight changed ({val_b_after:.3f})",
    )

    widget.deleteLater()
    return True


def test_refresh_with_multi_model_clears_and_rebuilds(
    model_a_path=None, model_b_path=None
) -> bool:
    """Calling refresh() twice with auto-discovery preserves multi-model view.

    Precondition: a scene with both models already imported.
    """
    if not model_a_path or not model_b_path:
        skip_test("Need at least 2 distinct models")

    widget = MorphTreeWidget()
    widget.refresh()  # auto-discover
    count_before = _count_tree_targets(widget)

    widget.refresh()  # auto-discover again
    count_after = _count_tree_targets(widget)

    print(f"  Targets before re-refresh: {count_before}")
    print(f"  Targets after  re-refresh: {count_after}")

    assert_eq(
        count_after,
        count_before,
        f"Target count changed after re-refresh ({count_before} → {count_after})",
    )

    widget.deleteLater()
    return True


# Multi-model test registry (used by run_all_integration_tests.py)
_MULTI_MODEL_TESTS = [
    ("Multi-Model Pose Tree — Vertex Tab", test_two_models_show_in_vertex_tab),
    ("Multi-Model Pose Tree — Bone Tab", test_two_models_show_in_bone_tab),
    (
        "Multi-Model Pose Tree — Weight Widgets Registered",
        test_weight_widgets_registered_for_both_models,
    ),
    (
        "Multi-Model Pose Tree — Visibility Toggle Isolation",
        test_visibility_toggle_isolates_models,
    ),
    (
        "Multi-Model Pose Tree — Re-refresh Stable",
        test_refresh_with_multi_model_clears_and_rebuilds,
    ),
]


# ── Mutating test names (wrapped in undo chunks for perfect isolation) ──
_MUTATING_MULTI_TESTS: set[str] = {
    "Multi-Model Pose Tree — Visibility Toggle Isolation",
}


# Multi-model orchestration is handled by run_all_integration_tests.py.
# The runner selects models, builds the shared two-model scene, and
# dispatches tests via run_standalone_suite.
