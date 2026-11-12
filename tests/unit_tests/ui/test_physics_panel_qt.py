"""Qt headless tests for the Physics panel and its Maya section.

Covers the list/inspector widget, the four groupings, selection syncing in both
directions, the filter, delete requests, and the tree's column setup.

Maya is stubbed and Qt runs on the ``offscreen`` platform.
"""

from __future__ import annotations

import os
import unittest
from collections.abc import Sequence
from dataclasses import replace
from typing import Any
from unittest.mock import patch

# ── Force offscreen platform BEFORE Qt imports ─────────────────────────
os.environ["QT_QPA_PLATFORM"] = "offscreen"

# ── Install Maya stub (for import passthrough) ─────────────────────────
from tests.unit_tests.maya.maya_stub import install_maya_stub

install_maya_stub(profile="headless")

from PySide6 import QtCore, QtGui, QtTest, QtWidgets
from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QApplication
from shiboken6 import isValid

# ``tool_main_widget`` hot-reloads its dependencies in dev mode; keep the
# class identities stable for this process.
os.environ["MAYAMMD_DEV"] = "0"

from mmd.maya.rigid_body_info import (
    JointInfo,
    JointStatus,
    PhysicsInfo,
    RigidBodyInfo,
)
from mmd.ui import physics_panel as pp
from mmd.ui import tool_main_widget as tw
from mmd.ui.collapsible_section import CollapsibleSection
from mmd.ui.physics_panel import Grouping, PhysicsPanel, grouped_rows

del os.environ["MAYAMMD_DEV"]

_app: QCoreApplication | None = None

GUIDE = "|Miku_Root|Miku_RigidBodies|Miku_Bodies|Miku_{name}"
JOINT_GUIDE = "|Miku_Root|Miku_RigidBodies|Miku_Joints|Miku_{name}_Constraint"


def setUpModule() -> None:
    global _app
    _app = QApplication.instance()
    if _app is None:
        _app = QApplication([])


def tearDownModule() -> None:
    global _app
    if _app is not None:
        _app.quit()
        _app = None


# ── Fixtures ───────────────────────────────────────────────────────────────

_BASE_BODY = RigidBodyInfo(
    physics_mode=1,
    physics_mode_label="Physics",
    collider_label="Sphere",
    mass=1.5,
    collision_mask=tuple(i != 3 for i in range(16)),
    draw_mode_label="Wire",
)

_BASE_JOINT = JointInfo(
    body_a_index=0,
    body_a="hip",
    body_b_index=1,
    body_b="knee",
    draw_mode_label="Wire",
)

_BASE_INFO = PhysicsInfo(
    root="|Miku_Root", solver="|Miku_Root|Miku_RigidBodies|Miku_RigidBodySolver"
)


def _body(index: int, name: str, **overrides: Any) -> RigidBodyInfo:
    guide = GUIDE.format(name=name)
    values: dict[str, Any] = {
        "index": index,
        "node": f"{guide}|pmxRigidBodyShape{index + 1}",
        "guide": guide,
        "name_local": name,
        "group": index,
    }
    values.update(overrides)
    return replace(_BASE_BODY, **values)


def _joint(index: int, name: str, **overrides: Any) -> JointInfo:
    guide = JOINT_GUIDE.format(name=name)
    return replace(
        _BASE_JOINT,
        index=index,
        node=f"{guide}|pmxRigidBodyJoint{index + 1}",
        guide=guide,
        name_local=name,
        **overrides,
    )


def _info(
    bodies: Sequence[RigidBodyInfo] = (),
    joints: Sequence[JointInfo] = (),
    **overrides: Any,
) -> PhysicsInfo:
    return replace(_BASE_INFO, bodies=tuple(bodies), joints=tuple(joints), **overrides)


def _child(item: QtWidgets.QTreeWidgetItem, index: int) -> QtWidgets.QTreeWidgetItem:
    child = item.child(index)
    assert child is not None
    return child


def _labels(widget: QtWidgets.QWidget) -> list[str]:
    """Every label text in ``widget``, in layout order."""
    return [label.text() for label in widget.findChildren(QtWidgets.QLabel)]


def _dispose(widget: QtWidgets.QWidget) -> None:
    """Delete a test widget, tolerating one already destroyed by a parent."""
    if isValid(widget):
        widget.deleteLater()


class _PanelTestBase(unittest.TestCase):
    def setUp(self) -> None:
        self.panel = PhysicsPanel()
        self.addCleanup(_dispose, self.panel)

    def groups(self) -> list[QtWidgets.QTreeWidgetItem]:
        tree = self.panel.tree
        return [
            item
            for index in range(tree.topLevelItemCount())
            if (item := tree.topLevelItem(index)) is not None
        ]

    def rows(self, group: int) -> list[QtWidgets.QTreeWidgetItem]:
        return [self.panel._items[index] for index in range(len(self.panel._items))]

    def texts(self) -> list[str]:
        """``heading / row`` names, for asserting list content compactly."""
        names: list[str] = []
        for group in self.groups():
            names.append(group.text(0))
            for index in range(group.childCount()):
                child = _child(group, index)
                names.append(child.text(1) or child.text(0))
        return names


# ── Grouping (pure) ────────────────────────────────────────────────────────


class TestGroupingPure(unittest.TestCase):
    def setUp(self) -> None:
        self.info = _info(
            [
                _body(
                    0,
                    "waist",
                    group=0,
                    bone="waist_Jnt",
                    physics_mode=0,
                    physics_mode_label="FollowBone",
                ),
                _body(1, "skirt", group=0, bone="skirt_Jnt", physics_mode=1),
                _body(2, "hair", group=3, bone="hair_Jnt", physics_mode=1),
            ],
            [
                _joint(0, "j_waist", body_a_index=0),
                _joint(1, "j_hair", body_a_index=2),
            ],
        )

    def test_pmx_order_is_flat(self) -> None:
        sections = grouped_rows(self.info, Grouping.PMX_ORDER)
        self.assertEqual(
            [title for title, _ in sections], ["Rigid Bodies (3)", "Joints (2)"]
        )
        self.assertEqual([bucket for bucket, _ in sections[0][1]], [""])
        self.assertEqual(
            [row[1] for row in sections[0][1][0][1]], ["waist", "skirt", "hair"]
        )

    def test_collision_group_buckets_numerically(self) -> None:
        sections = grouped_rows(self.info, Grouping.COLLISION_GROUP)
        self.assertEqual(
            [bucket for bucket, _ in sections[0][1]], ["Group 0 (2)", "Group 3 (1)"]
        )
        self.assertEqual([row[1] for row in sections[0][1][0][1]], ["waist", "skirt"])

    def test_joints_follow_their_body_a(self) -> None:
        sections = grouped_rows(self.info, Grouping.COLLISION_GROUP)
        self.assertEqual(
            [bucket for bucket, _ in sections[1][1]],
            ["Group 0 (1)", "Group 3 (1)"],
        )
        self.assertEqual([row[1] for row in sections[1][1][1][1]], ["j_hair"])

    def test_physics_mode_buckets(self) -> None:
        sections = grouped_rows(self.info, Grouping.PHYSICS_MODE)
        self.assertEqual(
            [bucket for bucket, _ in sections[0][1]], ["FollowBone (1)", "Physics (2)"]
        )

    def test_bone_buckets_are_alphabetical(self) -> None:
        sections = grouped_rows(self.info, Grouping.BONE)
        self.assertEqual(
            [bucket for bucket, _ in sections[0][1]],
            ["hair_Jnt (1)", "skirt_Jnt (1)", "waist_Jnt (1)"],
        )

    def test_bone_bucket_for_a_body_without_one(self) -> None:
        info = _info([_body(0, "loose", bone="")])
        sections = grouped_rows(info, Grouping.BONE)
        self.assertEqual([bucket for bucket, _ in sections[0][1]], ["(no bone) (1)"])

    def test_joint_without_a_body_lands_in_its_own_bucket(self) -> None:
        info = _info(joints=[_joint(0, "orphan", body_a_index=-1)])
        sections = grouped_rows(info, Grouping.BONE)
        self.assertEqual([bucket for bucket, _ in sections[1][1]], ["(no body) (1)"])

    def test_row_detail_reports_an_inactive_joint(self) -> None:
        info = _info(joints=[_joint(0, "j", status=JointStatus.SAME_BODY)])
        row = grouped_rows(info, Grouping.PMX_ORDER)[1][1][0][1][0]
        self.assertEqual(row[2], "0 → 1 · same body")


# ── Panel ──────────────────────────────────────────────────────────────────


class TestEmptyStates(_PanelTestBase):
    def test_no_model(self) -> None:
        self.panel.showModel(PhysicsInfo())
        self.assertEqual(self.texts(), [])
        self.assertIn("The scene has no rigid bodies", " ".join(_labels(self.panel)))

    def test_model_without_physics(self) -> None:
        self.panel.showModel(_info())
        self.assertIn("no rigid bodies or joints", " ".join(_labels(self.panel)))

    def test_placeholder_asks_for_a_selection(self) -> None:
        self.panel.showModel(_info([_body(0, "hip")]))
        self.assertIn("Select a rigid body", " ".join(_labels(self.panel.inspector)))


class TestListContent(_PanelTestBase):
    def setUp(self) -> None:
        super().setUp()
        self.panel.showModel(
            _info(
                [_body(0, "hip"), _body(1, "knee", collider_label="Box")],
                [_joint(0, "knee_joint")],
            )
        )

    def test_tabs_count_their_rows(self) -> None:
        self.assertEqual(
            [self.panel.tabs.tabText(i) for i in range(2)],
            ["Rigid Bodies (2)", "Joints (1)"],
        )

    def test_rows_show_index_name_and_detail(self) -> None:
        body = self.panel._items[0]
        self.assertEqual(
            [body.text(column) for column in range(3)],
            ["0", "hip", "Sphere · G0 · Physics"],
        )

    def test_joint_row_shows_its_two_bodies(self) -> None:
        self.panel.tabs.setCurrentIndex(1)
        joint = self.panel._items[0]
        self.assertEqual(
            [joint.text(column) for column in range(3)], ["0", "knee_joint", "0 → 1"]
        )

    def test_switching_grouping_rebuilds_the_list(self) -> None:
        self.panel.grouping_box.setCurrentIndex(
            self.panel.grouping_box.findText(Grouping.BONE.value)
        )
        self.assertIn("(no bone) (2)", self.texts())

    def test_grouping_survives_a_refresh(self) -> None:
        self.panel.grouping_box.setCurrentIndex(
            self.panel.grouping_box.findText(Grouping.COLLISION_GROUP.value)
        )
        self.panel.showModel(_info([_body(4, "hip")]))
        self.assertIn("Group 4 (1)", self.texts())


class TestScenePhysics(_PanelTestBase):
    def setUp(self) -> None:
        super().setUp()
        self.first = _info([_body(0, "hip", group=1)], [_joint(0, "joint")])
        self.second = replace(
            self.first, root="|Rin_Root",
            bodies=tuple(replace(row, node=row.node.replace("Miku", "Rin"),
                                 guide=row.guide.replace("Miku", "Rin"), group=7)
                         for row in self.first.bodies),
            joints=tuple(replace(row, node=row.node.replace("Miku", "Rin"),
                                 guide=row.guide.replace("Miku", "Rin"))
                         for row in self.first.joints),
        )
        self.panel.showScene([self.first, self.second])

    def test_models_have_separate_parents_and_scene_counts(self) -> None:
        self.assertEqual([item.text(0) for item in self.groups()], ["Miku_Root", "Rin_Root"])
        self.assertEqual(self.panel.tabs.tabText(0), "Rigid Bodies (2)")
        self.assertEqual(self.panel.tabs.tabText(1), "Joints (2)")

    def test_joint_groups_resolve_indices_within_their_model(self) -> None:
        self.panel.grouping_box.setCurrentIndex(1)
        self.panel.tabs.setCurrentIndex(1)
        self.assertEqual(self.groups()[0].child(0).text(0), "Group 1 (1)")
        self.assertEqual(self.groups()[1].child(0).text(0), "Group 7 (1)")

    def test_selection_preserves_tree_and_collapsed_models(self) -> None:
        roots = self.groups()
        roots[1].setExpanded(False)
        with patch.object(self.panel, "_rebuild", wraps=self.panel._rebuild) as rebuild:
            self.panel.syncSelection([self.second.bodies[0].guide])
            rebuild.assert_not_called()
        self.assertEqual(self.groups(), roots)
        self.assertFalse(roots[1].isExpanded())
        self.assertEqual(self.panel.selected_node, self.second.bodies[0].node)
        self.panel.revealSelected()
        self.assertTrue(roots[1].isExpanded())

    def test_refresh_retains_model_expansion_and_selection(self) -> None:
        self.panel.syncSelection([self.first.bodies[0].guide])
        self.groups()[1].setExpanded(False)
        self.panel.showScene([self.first, self.second])
        self.assertFalse(self.groups()[1].isExpanded())
        self.assertEqual(self.panel.selected_paths(), [self.first.bodies[0].guide])

    def test_edit_only_changes_owning_model(self) -> None:
        self.panel.applyAttributeEdit(self.second.bodies[0].node, "bodyGroupId", 9)
        self.panel.grouping_box.setCurrentIndex(1)
        self.assertEqual(self.groups()[0].child(0).text(0), "Group 1 (1)")
        self.assertEqual(self.groups()[1].child(0).text(0), "Group 9 (1)")

    def test_deleted_model_is_removed(self) -> None:
        self.panel.syncSelection([self.second.bodies[0].guide])
        self.panel.showScene([self.first])
        self.assertEqual(len(self.groups()), 1)
        self.assertEqual(self.panel.selected_paths(), [])
        self.assertFalse(self.panel.delete_button.isEnabled())


class TestSelection(_PanelTestBase):
    def setUp(self) -> None:
        super().setUp()
        self.body = _body(0, "hip")
        self.joint = _joint(0, "knee_joint")
        self.panel.showModel(_info([self.body], [self.joint]))
        self.emitted: list[str] = []
        self.panel.selectionChanged.connect(self.emitted.append)

    def _select(self, group: int, row: int) -> None:
        self.panel.tabs.setCurrentIndex(group)
        self.panel.tree.setCurrentItem(self.panel._items[row])

    def test_picking_a_row_emits_its_data_node(self) -> None:
        self._select(0, 0)
        self.assertEqual(self.emitted, [self.body.node])
        self.assertEqual(self.panel.selected_node, self.body.node)

    def test_picking_a_joint_emits_its_data_node(self) -> None:
        self._select(1, 0)
        self.assertEqual(self.emitted, [self.joint.node])

    def test_inspector_shows_the_body_fields(self) -> None:
        self._select(0, 0)
        text = " ".join(_labels(self.panel.inspector))
        self.assertIn("hip", text)
        from mmd.ui.physics_controls import ScalarControl

        self.assertEqual(
            self.panel.inspector.findChild(ScalarControl, "Mass").field.value(), 1.5
        )
        self.assertIn("Collides With", text)

    def test_inspector_explains_joint_freedom(self) -> None:
        self._select(1, 0)
        text = " ".join(_labels(self.panel.inspector))
        self.assertIn("welded", text)
        self.assertIn("locked", text)

    def test_sync_highlights_without_re_emitting(self) -> None:
        self.panel.syncSelection([self.joint.node])
        self.assertEqual(self.panel.selected_node, self.joint.node)
        self.assertEqual(self.emitted, [])

    def test_sync_accepts_the_guide_transform(self) -> None:
        """The viewport selects the guide, not the shape."""
        self.panel.syncSelection([self.body.guide])
        self.assertEqual(self.panel.selected_node, self.body.node)

    def test_sync_accepts_a_node_under_the_guide(self) -> None:
        self.panel.syncSelection([f"{self.body.guide}|Something"])
        self.assertEqual(self.panel.selected_node, self.body.node)

    def test_sync_clears_unknown_nodes(self) -> None:
        self._select(0, 0)
        self.panel.syncSelection(["|Miku_Root|Miku_Bones|hip_Jnt"])
        self.assertEqual(self.panel.selected_node, "")
        self.assertIn("Select a rigid body", " ".join(_labels(self.panel.inspector)))

    def test_sync_selects_only_rows_in_the_first_matching_tab(self) -> None:
        self.panel.syncSelection([self.body.node, self.joint.node, "|unknown"])
        self.assertEqual(self.panel.tabs.currentIndex(), 0)
        self.assertEqual(self.panel.selected_paths(), [self.body.guide])

    def test_selecting_a_heading_only_resets_the_inspector(self) -> None:
        self._select(0, 0)
        self.panel.grouping_box.setCurrentIndex(1)
        heading = self.groups()[0]
        self.panel.tree.setCurrentItem(heading)
        self.assertEqual(self.emitted, [self.body.node])
        self.assertIn("Select a rigid body", " ".join(_labels(self.panel.inspector)))

    def test_refresh_keeps_the_selected_row(self) -> None:
        self._select(1, 0)
        self.emitted.clear()
        self.panel.showModel(_info([self.body], [self.joint]))
        self.assertEqual(self.panel.selected_node, self.joint.node)
        self.assertEqual(self.emitted, [])

    def test_tabs_restore_selection_without_selecting_in_maya(self) -> None:
        self._select(0, 0)
        self.panel.tabs.setCurrentIndex(1)
        self.assertEqual(self.panel.selected_node, "")
        self.assertFalse(self.panel.delete_button.isEnabled())
        self._select(1, 0)
        self.emitted.clear()
        self.panel.tabs.setCurrentIndex(0)
        self.assertEqual(self.panel.selected_node, self.body.node)
        self.assertEqual(self.panel.selected_paths(), [self.body.guide])
        self.panel.tabs.setCurrentIndex(1)
        self.assertEqual(self.panel.selected_node, self.joint.node)
        self.assertEqual(self.emitted, [])

    def test_tabs_restore_scroll_and_collapsed_groups(self) -> None:
        self.panel.showModel(
            _info([_body(i, f"body{i}", group=i % 3) for i in range(100)], [self.joint])
        )
        self.panel.grouping_box.setCurrentIndex(1)
        self.panel.resize(700, 400)
        self.panel.show()
        QApplication.processEvents()
        self.groups()[0].child(0).setExpanded(False)
        bar = self.panel.tree.verticalScrollBar()
        bar.setValue(15)
        scroll = bar.value()
        self.assertGreater(scroll, 0)
        self.panel.tabs.setCurrentIndex(1)
        self.panel.tabs.setCurrentIndex(0)
        self.assertFalse(self.groups()[0].child(0).isExpanded())
        self.assertEqual(bar.value(), scroll)

    def test_refresh_updates_counts_and_preserves_joint_tab(self) -> None:
        self._select(1, 0)
        self.panel.showModel(_info([self.body], []))
        self.assertEqual(self.panel.tabs.currentIndex(), 1)
        self.assertEqual(self.panel.tabs.tabText(1), "Joints (0)")
        self.assertEqual(self.panel.tree.topLevelItemCount(), 0)
        self.assertFalse(self.panel.delete_button.isEnabled())


class TestFilter(_PanelTestBase):
    def setUp(self) -> None:
        super().setUp()
        self.panel.showModel(
            _info([_body(0, "hip"), _body(1, "knee")], [_joint(0, "knee_joint")])
        )

    def _visible(self, group: int) -> list[str]:
        self.panel.tabs.setCurrentIndex(group)
        return [item.text(1) for item in self.panel._items if not item.isHidden()]

    def test_filters_rows_by_name(self) -> None:
        self.panel.filter_edit.setText("knee")
        self.assertEqual(self._visible(0), ["knee"])
        self.assertEqual(self._visible(1), ["knee_joint"])

    def test_hides_a_heading_with_no_match(self) -> None:
        self.panel.grouping_box.setCurrentIndex(1)
        self.panel.filter_edit.setText("knee")
        self.assertFalse(self.groups()[0].child(1).isHidden())
        self.panel.filter_edit.setText("hip")
        self.assertTrue(self.groups()[0].child(1).isHidden())

    def test_clearing_restores_every_row(self) -> None:
        self.panel.filter_edit.setText("knee")
        self.panel.filter_edit.clear()
        self.assertEqual(self._visible(0), ["hip", "knee"])
        self.assertEqual(self._visible(1), ["knee_joint"])

    def test_matches_the_detail_column(self) -> None:
        self.panel.filter_edit.setText("sphere · g1")
        self.assertEqual(self._visible(0), ["knee"])


class TestDelete(_PanelTestBase):
    def setUp(self) -> None:
        super().setUp()
        self.body = _body(0, "hip")
        self.joint = _joint(0, "knee_joint")
        self.panel.showModel(_info([self.body], [self.joint]))
        self.requested: list[list[str]] = []
        self.panel.deleteRequested.connect(self.requested.append)

    def test_button_is_disabled_without_a_selection(self) -> None:
        self.assertFalse(self.panel.delete_button.isEnabled())

    def test_button_enables_with_a_selection(self) -> None:
        self.panel.tree.setCurrentItem(self.panel._items[0])
        self.assertTrue(self.panel.delete_button.isEnabled())

    def test_requests_the_guide_of_the_selected_row(self) -> None:
        self.panel.tree.setCurrentItem(self.panel._items[0])
        self.panel.delete_button.click()
        self.assertEqual(self.requested, [[self.body.guide]])

    def test_requests_every_selected_row(self) -> None:
        other = _body(1, "knee")
        self.panel.showModel(_info([self.body, other], [self.joint]))
        self.panel.tree.setCurrentItem(self.panel._items[0])
        self.panel._items[1].setSelected(True)
        self.panel.delete_button.click()
        self.assertEqual(self.requested, [[self.body.guide, other.guide]])

    def test_delete_key_requests_deletion(self) -> None:
        self.panel.tree.setCurrentItem(self.panel._items[0])
        shortcuts = self.panel.tree.findChildren(QtGui.QShortcut)
        self.assertEqual(len(shortcuts), 1)
        shortcuts[0].activated.emit()
        self.assertEqual(self.requested, [[self.body.guide]])


class TestFooter(_PanelTestBase):
    def test_counts(self) -> None:
        self.panel.showModel(
            _info([_body(0, "hip"), _body(1, "knee")], [_joint(0, "knee_joint")])
        )
        self.assertEqual(self.panel.tabs.tabText(0), "Rigid Bodies (2)")
        self.assertEqual(self.panel.tabs.tabText(1), "Joints (1)")
        self.assertTrue(self.panel._footer.isHidden())

    def test_warnings_replace_the_counts(self) -> None:
        self.panel.showModel(_info(warnings=("No pmxRigidBodyNode",)))


class TestLayout(_PanelTestBase):
    """The panel's own furniture: where Delete lives and how columns behave."""

    def setUp(self) -> None:
        super().setUp()
        self.panel.showModel(_info([_body(0, "hip")], [_joint(0, "j")]))

    def test_delete_sits_under_the_tree(self) -> None:
        """The button acts on the rows above it, not on the filter row."""
        column = self.panel.delete_button.parentWidget()
        assert column is not None
        self.assertIsNot(column, self.panel)
        self.assertIsNone(column.findChild(QtWidgets.QLineEdit))

    def test_controls_hold_only_the_filter_and_grouping(self) -> None:
        controls = [
            widget
            for widget in self.panel.findChildren(QtWidgets.QWidget)
            if isinstance(widget, (QtWidgets.QPushButton, QtWidgets.QLineEdit))
            and widget is not self.panel.delete_button
        ]
        self.assertEqual(len(controls), 2)  # filter and explicit Reveal Selected
        self.assertIs(controls[0], self.panel.filter_edit)

    def test_no_refresh_button(self) -> None:
        labels = [
            button.text() for button in self.panel.findChildren(QtWidgets.QPushButton)
        ]
        self.assertNotIn("Refresh", labels)

    def test_columns_are_resizable_and_the_last_stretches(self) -> None:
        header = self.panel.tree.header()
        for column in range(header.count()):
            self.assertEqual(
                header.sectionResizeMode(column),
                QtWidgets.QHeaderView.ResizeMode.Interactive,
                f"column {column} must be draggable",
            )
        self.assertTrue(header.stretchLastSection())

    def test_index_column_fits_three_digit_indices(self) -> None:
        """Ten bodies used to elide the index to ``…``."""
        self.panel.showModel(_info([_body(index, f"b{index}") for index in range(120)]))
        widest = str(max(int(item.text(0)) for item in self.panel._items))
        self.assertEqual(widest, "119")
        metrics = self.panel.tree.fontMetrics()
        available = (
            self.panel.tree.columnWidth(0)
            - self.panel.tree.indentation()
            - pp.CELL_PADDING
        )
        self.assertEqual(
            metrics.elidedText(widest, QtCore.Qt.TextElideMode.ElideRight, available),
            widest,
        )

    def test_a_manual_resize_survives_a_refresh(self) -> None:
        """Auto-fitting must never undo a separator the user dragged."""
        header = self.panel.tree.header()
        header.resizeSection(1, 240)
        self.panel.showModel(_info([_body(0, "hip")]))
        self.assertEqual(self.panel.tree.columnWidth(1), 240)

    def test_index_cells_are_right_aligned(self) -> None:
        item = self.panel._items[0]
        alignment = int(item.textAlignment(0))
        self.assertTrue(alignment & int(QtCore.Qt.AlignmentFlag.AlignRight))

    def test_headings_span_the_index_column(self) -> None:
        self.panel.grouping_box.setCurrentIndex(1)
        group = self.panel.tree.topLevelItem(0)
        assert group is not None
        self.assertTrue(group.isFirstColumnSpanned())


class TestNumericPrecision(_PanelTestBase):
    def test_display_edit_and_focus_preserve_precision(self) -> None:
        from mmd.ui.physics_controls import ScalarControl

        for language in (
            QtCore.QLocale.Language.English,
            QtCore.QLocale.Language.Polish,
        ):
            with self.subTest(language=language):
                edits = []
                control = ScalarControl(0.41768, 1.0, edits.append, self.panel)
                field = control.field
                field.setLocale(QtCore.QLocale(language))
                self.panel.show()
                control.show()
                self.panel.filter_edit.setFocus()
                QApplication.processEvents()
                decimal = field.locale().decimalPoint()
                self.assertEqual(field.text(), f"0{decimal}418")
                self.assertEqual(field.value(), 0.41768)
                field.setFocus()
                QApplication.processEvents()
                self.assertEqual(field.text(), f"0{decimal}41768")
                self.panel.filter_edit.setFocus()
                QApplication.processEvents()
                self.assertEqual(field.value(), 0.41768)
                self.assertEqual(edits, [])
                field.setFocus()
                field.selectAll()
                QtTest.QTest.keyClicks(field, f"0{decimal}123456789")
                self.panel.filter_edit.setFocus()
                QApplication.processEvents()
                self.assertEqual(field.value(), 0.123456789)
                self.assertEqual(field.text(), f"0{decimal}123")
                self.assertEqual(edits, [0.123456789])
                control.hide()

    def test_editing_one_vector_axis_preserves_the_other_axes(self) -> None:
        from mmd.ui.physics_controls import attribute_control

        edits = []
        values = (0.41768123456789, 1.02750012345, 1e-20)
        control = attribute_control(
            _body(0, "body", rest_translate=values),
            "Rest Pose",
            "Rest Translate",
            self.panel,
            lambda *args: edits.append(args),
        )
        fields = control.findChildren(QtWidgets.QDoubleSpinBox)
        self.assertEqual(tuple(field.value() for field in fields), values)
        fields[0].editingFinished.emit()
        self.assertEqual(edits, [])
        fields[0].setValue(0.987654321)
        fields[0].editingFinished.emit()
        self.assertEqual(edits, [("bodyRestTranslate", (0.987654321, *values[1:]))])


class TestInspectorStyle(_PanelTestBase):
    """The detail pane reads like the Attribute Editor: title + sections."""

    def setUp(self) -> None:
        super().setUp()
        self.body = _body(0, "hip")
        self.joint = _joint(0, "knee_joint")
        self.panel.showModel(_info([self.body], [self.joint]))

    def test_empty_inspector_has_no_sections(self) -> None:
        self.assertEqual(self.panel.inspector.sections, {})

    def test_body_shows_its_node_name_as_the_title(self) -> None:
        self.panel.syncSelection([self.body.node])
        self.assertTrue(self.panel.inspector.title.endswith("pmxRigidBodyShape1"))

    def test_sections_group_the_fields(self) -> None:
        self.panel.syncSelection([self.body.node])
        self.assertEqual(
            list(self.panel.inspector.sections),
            ["Body", "Follows", "Rest Pose", "Physics", "Collision Mask", "Display"],
        )

    def test_joint_reports_freedom_values(self) -> None:
        self.panel.syncSelection([self.joint.node])
        freedom = self.panel.inspector.sections["Freedom"]
        self.assertEqual(freedom[0], "welded")

    def test_every_section_shares_one_label_and_field_column(self) -> None:
        """Labels right, values left, and one x for every field edge."""
        self.panel.resize(700, 800)
        self.panel.show()
        self.panel.syncSelection([self.body.node])
        QApplication.processEvents()

        sections = self.panel.inspector._sections
        self.assertGreater(len(sections), 1)
        fields = [
            field
            for section in sections
            for field in section.body.findChildren(pp._ValueField)
        ]
        self.assertTrue(fields)
        labels = [
            label
            for section in sections
            for label in section.body.findChildren(QtWidgets.QLabel)
            if not isinstance(label, pp._ValueField)
        ]
        self.assertTrue(labels)

        lefts = {field.mapTo(self.panel, QtCore.QPoint(0, 0)).x() for field in fields}
        rights = {
            field.mapTo(self.panel, QtCore.QPoint(field.width(), 0)).x()
            for field in fields
        }
        self.assertEqual(len(lefts), 1, f"fields start at different x: {lefts}")
        self.assertEqual(len(rights), 1, f"fields end at different x: {rights}")
        for label in labels:
            self.assertTrue(label.alignment() & QtCore.Qt.AlignmentFlag.AlignRight)
        for field in fields:
            self.assertTrue(field.alignment() & QtCore.Qt.AlignmentFlag.AlignLeft)
        for field in self.panel.inspector.findChildren(QtWidgets.QLineEdit):
            if field.objectName() in ("Local Name", "Universal Name"):
                self.assertTrue(field.alignment() & QtCore.Qt.AlignmentFlag.AlignLeft)
        for field in self.panel.inspector.findChildren(QtWidgets.QDoubleSpinBox):
            self.assertTrue(field.alignment() & QtCore.Qt.AlignmentFlag.AlignLeft)

    def test_refresh_never_promotes_discarded_widgets_to_windows(self) -> None:
        self.panel.show()
        self.panel.syncSelection([self.body.node])
        QApplication.processEvents()
        inspector = self.panel.inspector
        old_widgets = [
            inspector._layout.itemAt(i).widget()
            for i in range(inspector._layout.count())
            if inspector._layout.itemAt(i).widget() is not None
        ]
        self.assertTrue(old_widgets)
        inspector.showMessage("No selection")
        # Inspect before DeferredDelete: even during this interval there must
        # be no detached, potentially flashing top-level widgets.
        for widget in old_widgets:
            self.assertTrue(widget.isHidden())
            self.assertFalse(widget.isWindow())
            self.assertIs(widget.parentWidget(), inspector._body)
        QtCore.QCoreApplication.sendPostedEvents(
            None, QtCore.QEvent.Type.DeferredDelete
        )
        self.assertTrue(all(not isValid(widget) for widget in old_widgets))

    def test_typed_editors_emit_scene_attributes(self) -> None:
        from mmd.ui.physics_controls import CollisionMaskControl, ScalarControl

        self.panel.syncSelection([self.body.node])
        inspector = self.panel.inspector
        edits = []
        inspector.attributeEdited.connect(lambda *args: edits.append(args))
        universal = inspector.findChild(QtWidgets.QLineEdit, "Universal Name")
        self.assertEqual(universal.text(), "")
        universal.setText("Hip")
        universal.editingFinished.emit()
        self.assertEqual(edits.pop(), (self.body.node, "bodyNameUniversal", "Hip"))
        enabled = inspector.findChild(QtWidgets.QCheckBox, "Enabled")
        enabled.click()
        self.assertEqual(edits.pop(), (self.body.node, "bodyEnabled", False))
        self.assertIsNone(inspector.findChild(QtWidgets.QComboBox, "Mode"))
        mode = inspector.findChild(QtWidgets.QLineEdit, "Mode")
        self.assertTrue(mode.isReadOnly())
        self.assertEqual(mode.text(), self.body.physics_mode_label)
        self.assertIn("re-import", mode.toolTip())
        mode.editingFinished.emit()
        self.assertEqual(edits, [])
        mass = inspector.findChild(ScalarControl, "Mass")
        mass.field.setValue(2.5)
        mass.field.editingFinished.emit()
        self.assertEqual(edits.pop(), (self.body.node, "bodyMass", 2.5))
        mask = inspector.findChild(CollisionMaskControl)
        self.assertEqual(len(mask.checkboxes), 16)
        self.assertFalse(mask.checkboxes[3].isChecked())
        mask.checkboxes[3].click()
        self.assertEqual(edits.pop(), (self.body.node, "bodyMaskGroup3", True))

    def test_title_stays_fixed_when_details_scroll(self) -> None:
        self.panel.resize(850, 360)
        self.panel.show()
        self.panel.syncSelection([self.body.node])
        QApplication.processEvents()
        inspector = self.panel.inspector
        position = inspector._header.mapTo(self.panel, QtCore.QPoint())
        bar = inspector.scroll_area.verticalScrollBar()
        self.assertGreater(bar.maximum(), 0)
        bar.setValue(bar.maximum())
        QApplication.processEvents()
        self.assertEqual(inspector._header.mapTo(self.panel, QtCore.QPoint()), position)
        self.assertEqual(inspector._header.text(), self.body.name)
        self.assertTrue(inspector._header.isVisible())

    def test_slider_drag_commits_only_on_release(self) -> None:
        from mmd.ui.physics_controls import ScalarControl

        self.panel.syncSelection([self.body.node])
        inspector = self.panel.inspector
        edits = []
        inspector.attributeEdited.connect(lambda *args: edits.append(args))
        mass = inspector.findChild(ScalarControl, "Mass")
        mass.slider.setSliderDown(True)
        mass.slider.setValue(200)
        mass.slider.setValue(300)
        self.assertFalse(edits)
        mass.slider.setSliderDown(False)
        self.assertEqual(edits, [(self.body.node, "bodyMass", 3.0)])

    def test_group_color_matches_viewport_state(self) -> None:
        from mmd.ui.physics_controls import group_color

        active = replace(self.body, group=4, physics_mode=1)
        self.assertEqual(group_color(4, active), QtGui.QColor.fromRgbF(0.7, 0.25, 0.7))
        self.assertEqual(
            group_color(4, replace(active, enabled=False)),
            QtGui.QColor.fromRgbF(0.45, 0.45, 0.45),
        )
        self.assertEqual(
            group_color(4, replace(active, physics_mode=0)),
            QtGui.QColor.fromRgbF(0.7 * 0.6, 0.25 * 0.6, 0.7 * 0.6),
        )

    def test_sections_collapse(self) -> None:
        from mmd.ui.collapsible_section import SectionHeader

        self.panel.syncSelection([self.body.node])
        headers = self.panel.inspector.findChildren(SectionHeader)
        self.assertTrue(headers)
        self.assertTrue(headers[0].isChecked())
        headers[0].click()
        self.assertFalse(headers[0].isChecked())


# ── Tool widget wiring ─────────────────────────────────────────────────────


class TestToolWidgetWiring(unittest.TestCase):
    def test_cleanup_after_workspace_has_destroyed_qt_children(self) -> None:
        import shiboken6

        widget = self._make_widget()
        callbacks = list(widget._physics_callbacks)
        self.assertEqual(len(callbacks), 11)
        shiboken6.delete(widget)
        with patch.object(tw.om.MMessage, "removeCallback") as remove:
            widget.cleanup()
            self.assertEqual(remove.call_count, len(callbacks))
            widget.cleanup()
            self.assertEqual(remove.call_count, len(callbacks))
        self.assertEqual(widget._physics_callbacks, [])
        self.assertIsNone(tw._OPEN_PANEL)

    def setUp(self) -> None:
        discovery = patch.object(tw, "discover_model_roots_in_scene", return_value=["|Miku_Root"])
        discovery.start()
        self.addCleanup(discovery.stop)

    def _make_widget(self) -> tw.MMMToolWidget:
        widget = tw.MMMToolWidget()
        self.addCleanup(_dispose, widget)
        self.addCleanup(widget.cleanup)
        return widget

    def test_hosts_a_physics_panel(self) -> None:
        self.assertIsInstance(self._make_widget().physics_panel, PhysicsPanel)

    def test_reopened_plugin_uses_current_widget_and_writes_editor_changes(
        self,
    ) -> None:
        from mmd import plugin
        from mmd.ui.physics_controls import ScalarControl

        host = QtWidgets.QWidget()
        self.addCleanup(_dispose, host)
        current_class = tw.MMMToolWidget
        with (
            patch.object(plugin, "_mmd_widget", None),
            patch.object(plugin, "MMMToolWidget", create=True) as stale_class,
            patch.object(tw, "MMMToolWidget", wraps=current_class) as current_factory,
            patch.object(plugin.cmds, "workspaceControl", return_value=False),
            patch("maya.OpenMayaUI.MQtUtil.findControl", return_value=1),
            patch("shiboken6.wrapInstance", return_value=host),
        ):
            plugin.show_dockable_widget()
            stale_class.assert_not_called()
            current_factory.assert_called_once_with(parent=host)
            widget = plugin._mmd_widget
            self.addCleanup(widget.cleanup)
            body = _body(0, "hip")
            widget.physics_panel.showModel(_info([body]))
            widget.physics_panel.syncSelection([body.node])
            mass = widget.physics_panel.inspector.findChild(ScalarControl, "Mass")
            with (
                patch.object(tw.cmds, "objExists", return_value=True),
                patch.object(tw.cmds, "getAttr", side_effect=[True, body.mass]),
                patch.object(tw.cmds, "setAttr") as write,
                patch.object(QtCore.QTimer, "singleShot"),
            ):
                mass.field.setValue(2.5)
                mass.field.editingFinished.emit()
                write.assert_called_once_with(f"{body.node}.bodyMass", 2.5)

    def test_editor_writes_use_maya_attribute_types(self) -> None:
        widget = self._make_widget()
        for attr, value, args, kwargs in (
            ("bodyMass", 2.5, (2.5,), {}),
            ("bodyEnabled", False, (False,), {}),
            ("bodyGroupId", 4, (4,), {}),
            ("bodyMaskGroup3", False, (False,), {}),
            ("bodyNameUniversal", "", ("",), {"type": "string"}),
            ("bodyShapeSize", (1.0, 2.0, 3.0), (1.0, 2.0, 3.0), {"type": "double3"}),
        ):
            with (
                self.subTest(attribute=attr),
                patch.object(tw.cmds, "objExists", return_value=True),
                patch.object(tw.cmds, "getAttr", side_effect=[True, None]),
                patch.object(tw.cmds, "setAttr") as write,
                patch.object(QtCore.QTimer, "singleShot"),
            ):
                widget._on_physics_attribute_edited("|body", attr, value)
                write.assert_called_once_with(f"|body.{attr}", *args, **kwargs)

    def test_successful_edit_keeps_controls_scroll_and_tree_in_place(self) -> None:
        from mmd.ui.physics_controls import ScalarControl

        widget = self._make_widget()
        body = _body(0, "hip")
        panel = widget.physics_panel
        panel.showModel(_info([body]))
        panel.syncSelection([body.node])
        widget.resize(850, 550)
        widget.show()
        QApplication.processEvents()
        inspector = panel.inspector
        sections = list(inspector._sections)
        sections[0].header.setChecked(False)
        QApplication.processEvents()
        bar = inspector.scroll_area.verticalScrollBar()
        bar.setValue(bar.maximum() // 2)
        scroll = bar.value()
        item = panel.tree.currentItem()
        mass = inspector.findChild(ScalarControl, "Mass")
        with (
            patch.object(tw.cmds, "objExists", return_value=True),
            patch.object(tw.cmds, "getAttr", side_effect=[True, body.mass]),
            patch.object(tw.cmds, "setAttr") as write,
            patch.object(tw, "read_physics") as read,
            patch.object(QtCore.QTimer, "singleShot") as schedule,
        ):
            mass.field.setValue(2.5)
            mass.field.editingFinished.emit()
            QApplication.processEvents()
            write.assert_called_once_with(f"{body.node}.bodyMass", 2.5)
            read.assert_not_called()
            schedule.assert_not_called()
        self.assertIs(inspector.findChild(ScalarControl, "Mass"), mass)
        self.assertEqual(inspector._sections, sections)
        self.assertFalse(sections[0].header.isChecked())
        self.assertEqual(bar.value(), scroll)
        self.assertIs(panel.tree.currentItem(), item)
        self.assertEqual(panel.info.row_at(body.node).mass, 2.5)
        self.assertIn("2.5", inspector.sections["Physics"])

    def test_in_place_edit_updates_names_and_grouped_rows(self) -> None:
        widget = self._make_widget()
        panel = widget.physics_panel
        body = _body(0, "hip")
        panel.showModel(_info([body]))
        panel.syncSelection([body.node])
        sections = list(panel.inspector._sections)
        panel.applyAttributeEdit(body.node, "bodyNameLocal", "renamed")
        self.assertEqual(panel.tree.currentItem().text(1), "renamed")
        self.assertEqual(panel.inspector._header.text(), "renamed")
        panel.grouping_box.setCurrentIndex(
            panel.grouping_box.findData(Grouping.COLLISION_GROUP)
        )
        panel.applyAttributeEdit(body.node, "bodyGroupId", 4)
        self.assertIn("Group 4", panel.tree.currentItem().parent().text(0))
        self.assertEqual(panel.inspector._sections, sections)
        self.assertEqual(panel.info.row_at(body.node).group, 4)

    def test_body_edits_refresh_joint_details_without_scene_reload(self) -> None:
        panel = self._make_widget().physics_panel
        a, b = _body(0, "hip"), _body(2, "ankle")
        joint = _joint(
            3, "link", body_a_index=0, body_b_index=2,
            body_a="hip", body_b="ankle",
            body_a_node=a.node, body_b_node=b.node,
        )
        # Another model may have exactly the same PMX indices.
        other_a = replace(a, node="|Other|a|shape", guide="|Other|a")
        other_b = replace(b, node="|Other|b|shape", guide="|Other|b")
        other_joint = replace(
            joint, node="|Other|joint|shape", guide="|Other|joint",
            body_a_node=other_a.node, body_b_node=other_b.node,
        )
        panel.showScene([
            _info([a, b], [joint]),
            _info([other_a, other_b], [other_joint], root="|Other"),
        ])
        panel.syncSelection([a.node])
        panel.applyAttributeEdit(a.node, "bodyEnabled", False)
        panel.applyAttributeEdit(a.node, "bodyNameLocal", "renamed")
        panel.syncSelection([joint.node])
        self.assertIn("disabled body", panel.inspector.sections["Joint"])
        self.assertEqual(panel.inspector.sections["Bodies"], ["0: renamed", "2: ankle"])
        self.assertIn("disabled body", panel.tree.currentItem().text(2))
        self.assertEqual(panel.info.row_at(other_joint.node), other_joint)

        panel.syncSelection([a.node])
        panel.applyAttributeEdit(a.node, "bodyEnabled", True)
        panel.applyAttributeEdit(a.node, "bodyPhysicsMode", 0)
        panel.syncSelection([b.node])
        panel.applyAttributeEdit(b.node, "bodyPhysicsMode", 0)
        panel.syncSelection([joint.node])
        self.assertIn("both follow bone", panel.inspector.sections["Joint"])
        panel.syncSelection([b.node])
        panel.applyAttributeEdit(b.node, "bodyPhysicsMode", 1)
        panel.syncSelection([joint.node])
        self.assertIn("active", panel.inspector.sections["Joint"])

    def test_locked_attribute_is_not_written_and_refreshes(self) -> None:
        widget = self._make_widget()
        with (
            patch.object(tw.cmds, "objExists", return_value=True),
            patch.object(tw.cmds, "getAttr", return_value=False),
            patch.object(tw.cmds, "setAttr") as write,
            patch.object(tw.cmds, "warning") as warning,
            patch.object(QtCore.QTimer, "singleShot") as schedule,
        ):
            widget._on_physics_attribute_edited("|body", "bodyMass", 5.0)
            write.assert_not_called()
            warning.assert_called_once()
            schedule.assert_called_once()

    def test_bottom_of_panel_and_scroll_handle_remain_reachable(self) -> None:
        widget = self._make_widget()
        widget.resize(700, 550)
        widget.show()
        QApplication.processEvents()
        area = widget._scroll_area
        bar = area.verticalScrollBar()
        bar.setValue(bar.maximum())
        QApplication.processEvents()

        footer = widget.physics_panel._footer
        bottom = footer.mapTo(area.viewport(), footer.rect().bottomRight())
        self.assertTrue(area.viewport().rect().contains(bottom))
        self.assertTrue(bar.isVisible())
        option = QtWidgets.QStyleOptionSlider()
        bar.initStyleOption(option)
        handle = bar.style().subControlRect(
            QtWidgets.QStyle.ComplexControl.CC_ScrollBar,
            option,
            QtWidgets.QStyle.SubControl.SC_ScrollBarSlider,
            bar,
        )
        self.assertTrue(bar.visibleRegion().contains(handle.center()))
        before = bar.value()
        QtTest.QTest.mousePress(
            bar, QtCore.Qt.MouseButton.LeftButton, pos=handle.center()
        )
        QtTest.QTest.mouseMove(bar, handle.center() - QtCore.QPoint(0, 60))
        QtTest.QTest.mouseRelease(
            bar,
            QtCore.Qt.MouseButton.LeftButton,
            pos=handle.center() - QtCore.QPoint(0, 60),
        )
        self.assertLess(bar.value(), before)

    def test_fixed_sections_scroll_and_collapse_without_a_gap(self) -> None:
        widget = self._make_widget()
        self.assertFalse(hasattr(widget, "sections_splitter"))
        widget.resize(600, 450)
        widget.show()
        QApplication.processEvents()
        morphs, physics = widget.morphs_section, widget.physics_section
        heights = (morphs.height(), physics.height())
        self.assertEqual(morphs.body.height(), tw.MORPHS_SECTION_HEIGHT)
        self.assertEqual(physics.body.height(), tw.PHYSICS_SECTION_HEIGHT)
        self.assertGreater(widget._scroll_area.verticalScrollBar().maximum(), 0)
        widget.resize(600, 1400)
        QApplication.processEvents()
        self.assertEqual((morphs.height(), physics.height()), heights)
        morphs.setExpanded(False)
        physics.setExpanded(False)
        QApplication.processEvents()
        self.assertEqual(
            physics.y() - morphs.geometry().bottom() - 1,
            widget.content_layout.spacing(),
        )
        morphs.setExpanded(True)
        physics.setExpanded(True)
        QApplication.processEvents()
        self.assertEqual((morphs.height(), physics.height()), heights)

    def test_sections_are_ordered_like_pmx(self) -> None:
        widget = self._make_widget()
        titles = [
            widget.content_layout.itemAt(i).widget().title
            for i in range(widget.content_layout.count())
            if isinstance(widget.content_layout.itemAt(i).widget(), CollapsibleSection)
        ]
        self.assertEqual(titles, ["Visibility", "Morphs", "Physics"])

    def test_expanding_physics_re_reads_the_scene(self) -> None:
        widget = self._make_widget()
        with (
            patch.object(tw, "read_physics", return_value=PhysicsInfo()) as read,
            patch.object(tw.cmds, "ls", return_value=[]),
        ):
            widget.physics_section.setExpanded(False)
            read.assert_not_called()
            widget.physics_section.setExpanded(True)
        read.assert_called_once()

    def test_the_physics_section_refreshes_when_expanded(self) -> None:
        """The Maya frame's expandCommand is wired to the open panel."""
        self.assertIn("refresh_open_panel", tw.PHYSICS_EXPAND_COMMAND)

    def test_refresh_reads_models_from_the_scene(self) -> None:
        widget = self._make_widget()
        info = _info([_body(0, "hip")])
        with (
            patch.object(tw, "read_physics", return_value=info) as read,
            patch.object(tw.cmds, "ls", return_value=[]),
        ):
            widget._refresh_physics_panel()
        read.assert_called_once_with("|Miku_Root")
        self.assertEqual(widget.physics_panel.info.bodies, info.bodies)

    def test_context_change_does_not_reload_physics(self) -> None:
        widget = self._make_widget()
        with patch.object(widget, "_refresh_physics_panel") as refresh:
            widget._on_model_changed("|Rin_Root")
        refresh.assert_not_called()

    def test_scene_notifications_are_coalesced(self) -> None:
        widget = self._make_widget()
        with patch.object(widget, "_refresh_physics_panel") as refresh:
            widget._schedule_physics_refresh()
            widget._schedule_physics_refresh()
            refresh.assert_not_called()
            QtTest.QTest.qWait(150)
            refresh.assert_called_once()

    def test_picking_a_row_selects_the_node_in_maya(self) -> None:
        widget = self._make_widget()
        node = "|Miku_Root|Miku_Bodies|pmxRigidBodyShape1"
        with (
            patch.object(tw.cmds, "objExists", return_value=True),
            patch.object(tw.cmds, "select") as select,
        ):
            widget._on_physics_selection_changed(node)
        select.assert_called_once_with(node, replace=True)

    def test_picking_a_row_does_not_echo_its_own_selection(self) -> None:
        widget = self._make_widget()
        widget._applying_physics_selection = True
        with patch.object(tw.cmds, "select") as select:
            widget._on_physics_selection_changed("|Miku_Root|Miku_Bodies|x")
        select.assert_not_called()

    def test_picking_a_deleted_row_re_reads_instead(self) -> None:
        widget = self._make_widget()
        with (
            patch.object(tw.cmds, "objExists", return_value=False),
            patch.object(tw.cmds, "select") as select,
            patch.object(tw, "read_physics", return_value=PhysicsInfo()) as read,
            patch.object(tw.cmds, "ls", return_value=[]),
        ):
            widget._on_physics_selection_changed("|Gone|Deleted")
        select.assert_not_called()
        read.assert_called_once()

    def test_delete_is_cancelled_by_the_confirmation(self) -> None:
        widget = self._make_widget()
        with (
            patch.object(tw.cmds, "objExists", return_value=True),
            patch.object(tw.cmds, "confirmDialog", return_value="Cancel"),
            patch.object(tw.cmds, "delete") as delete,
        ):
            widget._on_physics_delete_requested(["|Miku_Root|a"])
        delete.assert_not_called()

    def test_delete_removes_the_nodes_and_refreshes(self) -> None:
        widget = self._make_widget()
        with (
            patch.object(tw.cmds, "objExists", return_value=True),
            patch.object(tw.cmds, "confirmDialog", return_value="Delete"),
            patch.object(tw.cmds, "delete") as delete,
            patch.object(tw, "read_physics", return_value=PhysicsInfo()) as read,
            patch.object(tw.cmds, "ls", return_value=[]),
        ):
            widget._on_physics_delete_requested(["|Miku_Root|a", "|Miku_Root|b"])
        delete.assert_called_once_with(["|Miku_Root|a", "|Miku_Root|b"])
        read.assert_called_once()

    def test_delete_skips_nodes_that_are_already_gone(self) -> None:
        widget = self._make_widget()
        with (
            patch.object(tw.cmds, "objExists", return_value=False),
            patch.object(tw.cmds, "confirmDialog") as confirm,
            patch.object(tw, "read_physics", return_value=PhysicsInfo()),
            patch.object(tw.cmds, "ls", return_value=[]),
        ):
            widget._on_physics_delete_requested(["|Gone|a"])
        confirm.assert_not_called()


if __name__ == "__main__":
    unittest.main()
