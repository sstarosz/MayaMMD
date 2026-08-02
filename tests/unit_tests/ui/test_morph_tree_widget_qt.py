"""
Qt headless unit tests for ``MorphTreeWidget`` and its delegate classes.

These tests run against a real ``QApplication`` using the ``offscreen``
QPA platform (no display required).  Maya calls are stubbed via
``maya_stub`` so the widget can be imported and partially constructed.
"""

from __future__ import annotations

import os
import unittest
from unittest.mock import MagicMock, patch

# ── Force offscreen platform BEFORE Qt imports ─────────────────────────
os.environ["QT_QPA_PLATFORM"] = "offscreen"

# ── Install Maya stub (for import passthrough) ─────────────────────────
from tests.unit_tests.maya.maya_stub import install_maya_stub

install_maya_stub(profile="headless")

from PySide6 import QtCore, QtWidgets
from PySide6.QtCore import QPoint, QRect, Qt
from PySide6.QtGui import QColor, QPainter, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QStyleOptionViewItem,
    QTreeWidget,
    QTreeWidgetItem,
)

# Import the module under test
from mmd.ui import morph_tree_widget as _mod  # noqa: E402
from mmd.ui.morph_tree_widget import (  # noqa: E402
    KeyframeState,
    MorphTreeWidget,
    MayaStyleTreeDelegate,
    _CircleIndicator,
    _C,
    _MorphRow,
    _create_round_toggle,
    _create_keyframe_dot,
    _wrap_centered,
)


# ═══════════════════════════════════════════════════════════════════════════
#  Global QApplication (shared across all test classes in this module)
# ═══════════════════════════════════════════════════════════════════════════

_app: QApplication | None = None


def setUpModule() -> None:
    """Create a single QApplication for all tests in this module."""
    global _app
    _app = QApplication.instance()
    if _app is None:
        _app = QApplication([])


def tearDownModule() -> None:
    """Clean up the QApplication."""
    global _app
    if _app is not None:
        _app.quit()
        _app = None


# ═══════════════════════════════════════════════════════════════════════════
#  _CircleIndicator
# ═══════════════════════════════════════════════════════════════════════════


class TestCircleIndicator(unittest.TestCase):
    """Tests for ``_CircleIndicator`` widget."""

    def test_default_size(self) -> None:
        dot = _CircleIndicator()
        self.assertEqual(dot.width(), 13)
        self.assertEqual(dot.height(), 13)

    def test_custom_size(self) -> None:
        dot = _CircleIndicator(size=20)
        self.assertEqual(dot.width(), 20)
        self.assertEqual(dot.height(), 20)

    def test_default_color_is_black(self) -> None:
        dot = _CircleIndicator()
        self.assertEqual(dot.color(), QColor(0, 0, 0))

    def test_custom_color(self) -> None:
        red = QColor(255, 0, 0)
        dot = _CircleIndicator(color=red)
        self.assertEqual(dot.color(), red)

    def test_set_color_updates(self) -> None:
        dot = _CircleIndicator()
        green = QColor(0, 255, 0)
        dot.setColor(green)
        self.assertEqual(dot.color(), green)

    def test_set_color_noop_when_same(self) -> None:
        """Setting the same color should not trigger a repaint."""
        dot = _CircleIndicator()
        dot.setColor(QColor(0, 0, 0))  # same as default — no crash

    def test_default_no_border(self) -> None:
        dot = _CircleIndicator()
        # Border color is None by default (no pen)
        self.assertIsNone(dot._border_color)

    def test_set_border_color(self) -> None:
        dot = _CircleIndicator()
        border = QColor(100, 100, 100)
        dot.setBorderColor(border)
        self.assertEqual(dot._border_color, border)

    def test_set_border_color_none(self) -> None:
        dot = _CircleIndicator()
        dot.setBorderColor(QColor(255, 0, 0))
        dot.setBorderColor(None)
        self.assertIsNone(dot._border_color)

    def test_paint_produces_output(self) -> None:
        """paintEvent should render to a pixmap without crashing."""
        dot = _CircleIndicator(size=32, color=QColor(255, 0, 0))
        dot.resize(32, 32)
        pixmap = QPixmap(32, 32)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        _CircleIndicator.paintEvent(dot, MagicMock())
        painter.end()
        self.assertFalse(pixmap.isNull())


# ═══════════════════════════════════════════════════════════════════════════
#  MayaStyleTreeDelegate
# ═══════════════════════════════════════════════════════════════════════════


class TestMayaStyleTreeDelegate(unittest.TestCase):
    """Tests for ``MayaStyleTreeDelegate`` paint and editor events."""

    def setUp(self) -> None:
        self.tree = QTreeWidget()
        self.tree.setColumnCount(2)
        self.tree.setHeaderLabels(["Name", "Value"])
        self.tree.setIndentation(30)
        self.delegate = MayaStyleTreeDelegate(self.tree)
        self.tree.setItemDelegateForColumn(0, self.delegate)

    def tearDown(self) -> None:
        self.tree.deleteLater()

    def test_delegate_binds_tree(self) -> None:
        self.assertIs(self.delegate.tree_widget, self.tree)

    def test_paint_non_column_zero_falls_through(self) -> None:
        """Painting column != 0 should use the default super().paint()."""
        item = QTreeWidgetItem(self.tree, ["Test", "42"])
        index = self.tree.indexFromItem(item, 1)  # column 1

        pixmap = QPixmap(200, 40)
        painter = QPainter(pixmap)
        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 200, 40)
        option.state = QtWidgets.QStyle.State_None

        # Should not crash
        self.delegate.paint(painter, option, index)
        painter.end()

    def test_paint_no_tree_noop(self) -> None:
        """If delegate has no tree_widget, paint falls through."""
        delegate = MayaStyleTreeDelegate(None)
        self.assertIsNone(delegate.tree_widget)

        # Create a temporary tree to get a valid QModelIndex for column != 0
        tmp_tree = QTreeWidget()
        tmp_tree.setColumnCount(2)
        item = QTreeWidgetItem(tmp_tree, ["A", "B"])
        # Use column 1 so we hit the early return (column != 0)
        index = tmp_tree.indexFromItem(item, 1)

        pixmap = QPixmap(200, 40)
        painter = QPainter(pixmap)
        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 200, 40)
        option.state = QtWidgets.QStyle.State_None

        # Should not crash
        delegate.paint(painter, option, index)
        painter.end()
        tmp_tree.deleteLater()

    def test_paint_parent_item(self) -> None:
        """Paint a parent item with children (expand button drawn)."""
        parent = QTreeWidgetItem(self.tree, ["Parent"])
        QTreeWidgetItem(parent, ["Child"])
        index = self.tree.indexFromItem(parent, 0)

        pixmap = QPixmap(300, 60)
        painter = QPainter(pixmap)
        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 300, 40)
        option.state = QtWidgets.QStyle.State_None

        self.delegate.paint(painter, option, index)
        painter.end()
        self.assertFalse(pixmap.isNull())

    def test_paint_leaf_item(self) -> None:
        """Paint a leaf item (circle dot drawn, no expand button)."""
        leaf = QTreeWidgetItem(self.tree, ["Leaf"])
        index = self.tree.indexFromItem(leaf, 0)

        pixmap = QPixmap(300, 60)
        painter = QPainter(pixmap)
        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 300, 40)
        option.state = QtWidgets.QStyle.State_None

        self.delegate.paint(painter, option, index)
        painter.end()
        self.assertFalse(pixmap.isNull())

    def test_paint_selected_item(self) -> None:
        """Paint a selected item (blue highlight drawn)."""
        item = QTreeWidgetItem(self.tree, ["Selected"])
        item.setSelected(True)
        index = self.tree.indexFromItem(item, 0)

        pixmap = QPixmap(300, 60)
        painter = QPainter(pixmap)
        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 300, 40)
        option.state = QtWidgets.QStyle.State_Selected

        self.delegate.paint(painter, option, index)
        painter.end()
        self.assertFalse(pixmap.isNull())

    def test_paint_nested_child(self) -> None:
        """Paint a child of a child (tree lines at level 2)."""
        grandparent = QTreeWidgetItem(self.tree, ["Grandparent"])
        parent = QTreeWidgetItem(grandparent, ["Parent"])
        child = QTreeWidgetItem(parent, ["Child"])
        index = self.tree.indexFromItem(child, 0)

        pixmap = QPixmap(400, 60)
        painter = QPainter(pixmap)
        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 400, 40)
        option.state = QtWidgets.QStyle.State_None

        self.delegate.paint(painter, option, index)
        painter.end()
        self.assertFalse(pixmap.isNull())

    def test_editor_event_expand_toggle(self) -> None:
        """Clicking the expand button toggles expansion."""
        parent = QTreeWidgetItem(self.tree, ["Parent"])
        QTreeWidgetItem(parent, ["Child"])
        parent.setExpanded(True)

        index = self.tree.indexFromItem(parent, 0)
        model = self.tree.model()

        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 300, 40)

        # Click at decoration_x=5, center_y=20 → box at (5, 13, 14, 14)
        # Click center at (5+7, 13+7) = (12, 20)
        click_pos = QPoint(12, 20)

        # Create a mouse release event
        event = QtCore.QEvent(QtCore.QEvent.Type.MouseButtonRelease)
        # We need to patch the event to have a pos() method
        # Instead, use a QMouseEvent
        from PySide6.QtGui import QMouseEvent

        mouse_event = QMouseEvent(
            QtCore.QEvent.Type.MouseButtonRelease,
            click_pos,
            click_pos,
            click_pos,
            Qt.MouseButton.LeftButton,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )

        # Fire the editorEvent
        result = self.delegate.editorEvent(mouse_event, model, option, index)
        self.assertTrue(result)

    def test_editor_event_outside_button(self) -> None:
        """Clicking outside the expand button delegates to default."""
        parent = QTreeWidgetItem(self.tree, ["Parent"])
        QTreeWidgetItem(parent, ["Child"])
        parent.setExpanded(True)

        index = self.tree.indexFromItem(parent, 0)
        model = self.tree.model()

        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 300, 40)

        # Click far away from the expand button
        from PySide6.QtGui import QMouseEvent

        mouse_event = QMouseEvent(
            QtCore.QEvent.Type.MouseButtonRelease,
            QPoint(200, 20),
            QPoint(200, 20),
            QPoint(200, 20),
            Qt.MouseButton.LeftButton,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )

        result = self.delegate.editorEvent(mouse_event, model, option, index)
        # Should delegate to default (returns False from super)
        self.assertFalse(result)

    def test_editor_event_non_column_zero(self) -> None:
        """Non-column-0 events are delegated to default."""
        parent = QTreeWidgetItem(self.tree, ["Parent"])
        QTreeWidgetItem(parent, ["Child"])

        index = self.tree.indexFromItem(parent, 1)  # column 1
        model = self.tree.model()

        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 300, 40)

        from PySide6.QtGui import QMouseEvent

        mouse_event = QMouseEvent(
            QtCore.QEvent.Type.MouseButtonRelease,
            QPoint(12, 20),
            QPoint(12, 20),
            QPoint(12, 20),
            Qt.MouseButton.LeftButton,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )

        result = self.delegate.editorEvent(mouse_event, model, option, index)
        self.assertFalse(result)

    def test_editor_event_leaf_item(self) -> None:
        """Leaf items (no children) delegate to default."""
        leaf = QTreeWidgetItem(self.tree, ["Leaf"])

        index = self.tree.indexFromItem(leaf, 0)
        model = self.tree.model()

        option = QStyleOptionViewItem()
        option.rect = QRect(0, 0, 300, 40)

        from PySide6.QtGui import QMouseEvent

        mouse_event = QMouseEvent(
            QtCore.QEvent.Type.MouseButtonRelease,
            QPoint(12, 20),
            QPoint(12, 20),
            QPoint(12, 20),
            Qt.MouseButton.LeftButton,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )

        result = self.delegate.editorEvent(mouse_event, model, option, index)
        self.assertFalse(result)


# ═══════════════════════════════════════════════════════════════════════════
#  Widget factories
# ═══════════════════════════════════════════════════════════════════════════


class TestWidgetFactories(unittest.TestCase):
    """Tests for ``_create_*`` factory methods."""

    def test_create_round_toggle_default(self) -> None:
        btn = _create_round_toggle()
        self.assertTrue(btn.isCheckable())
        self.assertTrue(btn.isChecked())
        self.assertEqual(btn.width(), 16)
        self.assertEqual(btn.height(), 16)

    def test_create_round_toggle_unchecked(self) -> None:
        btn = _create_round_toggle(checked=False)
        self.assertFalse(btn.isChecked())

    def test_create_round_toggle_tooltip(self) -> None:
        btn = _create_round_toggle(tooltip="Hello")
        self.assertEqual(btn.toolTip(), "Hello")

    def test_create_round_toggle_signal(self) -> None:
        """Toggling emits the signal."""
        btn = _create_round_toggle(checked=True)
        received: list[bool] = []

        def _on_toggle(checked: bool) -> None:
            received.append(checked)

        btn.toggled.connect(_on_toggle)
        btn.setChecked(False)
        self.assertEqual(received, [False])

    def test_create_keyframe_dot_inactive(self) -> None:
        dot = _create_keyframe_dot(active=False)
        self.assertIsInstance(dot, _CircleIndicator)
        self.assertEqual(dot.color(), QColor(0, 0, 0))  # black when off
        self.assertIsNone(dot._border_color)

    def test_create_keyframe_dot_active(self) -> None:
        dot = _create_keyframe_dot(active=True)
        self.assertIsInstance(dot, _CircleIndicator)
        self.assertEqual(dot.color(), _C["red_accent"])

    def test_create_weight_widget(self) -> None:
        """Weight widget factory creates slider and spinbox with correct ranges."""
        container, slider, spinbox = MorphTreeWidget._create_weight_widget()

        self.assertIsInstance(slider, QtWidgets.QSlider)
        self.assertIsInstance(spinbox, QtWidgets.QDoubleSpinBox)
        self.assertIsInstance(container, QtWidgets.QWidget)

        self.assertEqual(slider.minimum(), 0)
        self.assertEqual(slider.maximum(), 1000)
        self.assertEqual(spinbox.minimum(), 0.0)
        self.assertEqual(spinbox.maximum(), 1.0)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_morph_row_slider_spinbox_sync(self, mock_cmds: MagicMock) -> None:
        """_MorphRow wires slider ↔ spinbox bidirectional sync."""
        mock_cmds.objExists.return_value = False  # no Maya node

        _, slider, spinbox = MorphTreeWidget._create_weight_widget()
        key_dot = _CircleIndicator()
        row = _MorphRow("node.attr", slider, spinbox, None, key_dot)

        # Slider → spinbox
        row.slider.setValue(750)
        self.assertAlmostEqual(row.spinbox.value(), 0.75, places=3)

        # Spinbox → slider
        row.spinbox.setValue(0.33)
        self.assertEqual(row.slider.value(), 330)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_create_envelope_control(self, mock_cmds: MagicMock) -> None:
        """Envelope control creates a round toggle bound to the envelope attr."""
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 1.0

        widget = MorphTreeWidget()
        container = widget._create_envelope_control("node.envelope")
        self.assertIsInstance(container, QtWidgets.QWidget)

        # Find the toggle button inside
        btn = container.findChild(QtWidgets.QPushButton)
        self.assertIsNotNone(btn)
        self.assertTrue(btn.isCheckable())


# ═══════════════════════════════════════════════════════════════════════════
#  _MorphRow keyframe styling
# ═══════════════════════════════════════════════════════════════════════════


class TestMorphRowStyling(unittest.TestCase):
    """Tests for ``_MorphRow.set_keyframe_style``."""

    def setUp(self) -> None:
        self._container, slider, spinbox = MorphTreeWidget._create_weight_widget()
        self.row = _MorphRow("node.attr", slider, spinbox, None, _CircleIndicator())

    def test_set_keyframe_style_at_current(self) -> None:
        self.row.set_keyframe_style(KeyframeState.AT_CURRENT)
        ss = self.row.spinbox.styleSheet()
        self.assertIn(_C["red_accent"].name(), ss)
        self.assertIn("background-color", ss)
        self.assertEqual(self.row.key_dot.color(), _C["red_accent"])

    def test_set_keyframe_style_between(self) -> None:
        self.row.set_keyframe_style(KeyframeState.BETWEEN)
        ss = self.row.spinbox.styleSheet()
        self.assertIn(_C["red_between"].name(), ss)
        self.assertEqual(self.row.key_dot.color(), _C["black"])

    def test_set_keyframe_style_none(self) -> None:
        self.row.set_keyframe_style(KeyframeState.NONE)
        self.assertEqual(self.row.spinbox.styleSheet(), "")
        self.assertEqual(self.row.key_dot.color(), _C["black"])


# ═══════════════════════════════════════════════════════════════════════════
#  MorphTreeWidget construction
# ═══════════════════════════════════════════════════════════════════════════


class TestMorphTreeWidgetConstruction(unittest.TestCase):
    """Tests for constructing ``MorphTreeWidget`` with mocked Maya."""

    def tearDown(self) -> None:
        # Reset the singleton so each test gets a fresh instance
        MorphTreeWidget._active_instance = None

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_constructor_creates_tabs(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """The widget creates two tabs (Vertex, Bone) even with no data."""
        # Stub cmds for discover_model_roots_in_scene
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget = MorphTreeWidget()
        self.assertIsNotNone(widget)

        # Two tabs: "Vertex" and "Bone"
        self.assertEqual(widget._tabs.count(), 2)
        self.assertEqual(widget._tabs.tabText(0), "Vertex")
        self.assertEqual(widget._tabs.tabText(1), "Bone")

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_refresh_clears_and_rebuilds(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Calling refresh() rebuilds the tabs."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget = MorphTreeWidget()
        initial_tab_count = widget._tabs.count()

        widget.refresh(blend_shape_node="", bone_morph_node="")
        self.assertEqual(widget._tabs.count(), initial_tab_count)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_refresh_with_invalid_types(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Passing non-string args to refresh logs an error and returns early."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget = MorphTreeWidget()

        # Should not crash
        widget.refresh(blend_shape_node=42, bone_morph_node=None)  # type: ignore[arg-type]

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_singleton_instance_tracking(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """The most recently created widget is tracked as _active_instance."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget1 = MorphTreeWidget()
        self.assertIs(MorphTreeWidget._active_instance, widget1)

        widget2 = MorphTreeWidget()
        self.assertIs(MorphTreeWidget._active_instance, widget2)

        widget1.deleteLater()
        widget2.deleteLater()


# ═══════════════════════════════════════════════════════════════════════════
#  Weight change callback logic (extracted for testability)
# ═══════════════════════════════════════════════════════════════════════════


class TestOnWeightChangedLogic(unittest.TestCase):
    """Test the Maya-write and keyframe-styling logic of ``_MorphRow``.

    The slider ↔ spinbox ↔ Maya wiring now lives in ``_MorphRow``
    rather than in ``_create_weight_widget`` closures.
    """

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_slider_to_maya_write(self, mock_cmds: MagicMock) -> None:
        """When the slider changes via _MorphRow, cmds.setAttr is called."""
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0

        container, slider, spinbox = MorphTreeWidget._create_weight_widget()
        row = _MorphRow("node.attr", slider, spinbox, None, _CircleIndicator())
        mock_cmds.setAttr.reset_mock()
        slider.setValue(800)
        mock_cmds.setAttr.assert_called_with("node.attr", 0.8)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_spinbox_to_maya_write(self, mock_cmds: MagicMock) -> None:
        """When the spinbox changes via _MorphRow, cmds.setAttr is called."""
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0

        container, slider, spinbox = MorphTreeWidget._create_weight_widget()
        row = _MorphRow("node.attr", slider, spinbox, None, _CircleIndicator())
        mock_cmds.setAttr.reset_mock()
        spinbox.setValue(0.42)
        mock_cmds.setAttr.assert_called_with("node.attr", 0.42)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_slider_preserves_keyframe_between_style(
        self, mock_cmds: MagicMock
    ) -> None:
        """Slider drag between keys retains the ``red_between`` stylesheet."""
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0
        mock_cmds.listConnections.return_value = ["animCurveTL1"]
        mock_cmds.currentTime.return_value = 7.0
        mock_cmds.keyframe.return_value = [1.0, 10.0]

        container, slider, spinbox = MorphTreeWidget._create_weight_widget()
        row = _MorphRow("node.attr", slider, spinbox, None, _CircleIndicator())
        row.set_keyframe_style(KeyframeState.BETWEEN)
        self.assertIn(_C["red_between"].name(), spinbox.styleSheet())

        slider.setValue(500)
        ss = spinbox.styleSheet()
        self.assertIn(
            _C["red_between"].name(),
            ss,
            "red_between style should survive a slider drag between keys",
        )

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_slider_preserves_keyframe_at_current_style(
        self, mock_cmds: MagicMock
    ) -> None:
        """Slider drag at a key retains the ``red_accent`` stylesheet."""
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0
        mock_cmds.listConnections.return_value = ["animCurveTL1"]
        mock_cmds.currentTime.return_value = 5.0
        mock_cmds.keyframe.return_value = [1.0, 5.0, 10.0]

        container, slider, spinbox = MorphTreeWidget._create_weight_widget()
        row = _MorphRow("node.attr", slider, spinbox, None, _CircleIndicator())
        row.set_keyframe_style(KeyframeState.AT_CURRENT)
        self.assertIn(_C["red_accent"].name(), spinbox.styleSheet())

        slider.setValue(750)
        ss = spinbox.styleSheet()
        self.assertIn(
            _C["red_accent"].name(),
            ss,
            "red_accent style should survive a slider drag at a key",
        )

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_slider_clears_style_when_no_keyframes(self, mock_cmds: MagicMock) -> None:
        """Slider drag with no anim curves leaves the stylesheet empty."""
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0
        mock_cmds.listConnections.return_value = None

        container, slider, spinbox = MorphTreeWidget._create_weight_widget()
        row = _MorphRow("node.attr", slider, spinbox, None, _CircleIndicator())
        row.set_keyframe_style(KeyframeState.BETWEEN)
        self.assertIn(_C["red_between"].name(), spinbox.styleSheet())

        slider.setValue(250)
        ss = spinbox.styleSheet()
        self.assertEqual(
            ss, "", "stylesheet should be cleared when no anim curves exist"
        )


# ═══════════════════════════════════════════════════════════════════════════
#  Multi-model source handling
# ═══════════════════════════════════════════════════════════════════════════


class _FakeMorphSource(_mod._AbstractMorphSource):
    """Minimal concrete source for unit tests — avoids Maya calls entirely."""

    def __init__(
        self,
        node_name: str,
        targets: list[str],
        section: str = "Vertex",
        have_envelope: bool = True,
        have_edit: bool = False,
    ):
        super().__init__(node_name)
        self._targets = targets
        self._section = section
        self._have_envelope = have_envelope
        self._have_edit = have_edit
        self.edit_clicked_calls: list[str] = []

    def section_label(self) -> str:
        return self._section

    def get_targets(self) -> list[str]:
        return self._targets

    def get_weight_attr(self, target_name: str) -> str:
        return f"{self._node}.{target_name}"

    def get_envelope_attr(self) -> str | None:
        if self._have_envelope:
            return f"{self._node}.envelope"
        return None

    def get_edit_action_label(self) -> str | None:
        return "Edit" if self._have_edit else None

    def on_edit_clicked(self, target_name: str) -> None:
        self.edit_clicked_calls.append(target_name)


class TestMultiModelSources(unittest.TestCase):
    """Tests for ``_create_tab`` with multiple model sources."""

    def tearDown(self) -> None:
        MorphTreeWidget._active_instance = None

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_two_sources(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Two sources in one tab produce two top-level tree items."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget = MorphTreeWidget()
        source_a = _FakeMorphSource("nodeA", ["t1", "t2"])
        source_b = _FakeMorphSource("nodeB", ["t3", "t4"])

        # _create_tab appends a new tab after the constructor's 2 default tabs.
        widget._create_tab([("Model A", source_a), ("Model B", source_b)], "Vertex")

        # The newly added tab is the last one.
        new_idx = widget._tabs.count() - 1
        tree = widget._tabs.widget(new_idx).findChild(QtWidgets.QTreeWidget)
        self.assertIsNotNone(tree)

        self.assertEqual(tree.topLevelItemCount(), 2)
        self.assertEqual(tree.topLevelItem(0).text(0), "Model A")
        self.assertEqual(tree.topLevelItem(1).text(0), "Model B")
        self.assertEqual(tree.topLevelItem(0).childCount(), 2)
        self.assertEqual(tree.topLevelItem(1).childCount(), 2)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_one_source_empty_targets(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """A source with no targets produces a parent with no children."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget = MorphTreeWidget()
        source = _FakeMorphSource("nodeA", targets=[])

        # _create_tab appends a new tab after the constructor's 2 default tabs.
        widget._create_tab([("Empty Model", source)], "Vertex")

        new_idx = widget._tabs.count() - 1
        tree = widget._tabs.widget(new_idx).findChild(QtWidgets.QTreeWidget)
        self.assertIsNotNone(tree)
        self.assertEqual(tree.topLevelItemCount(), 1)
        self.assertEqual(tree.topLevelItem(0).childCount(), 0)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_empty_sources_list(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """An empty sources list creates an empty tree (not a crash)."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget = MorphTreeWidget()
        widget._create_tab([], "Vertex")

        tree = widget._tabs.widget(0).findChild(QtWidgets.QTreeWidget)
        self.assertIsNotNone(tree)
        self.assertEqual(tree.topLevelItemCount(), 0)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_mixed_morph_types(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Vertex and Bone tabs each get their own sources."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget = MorphTreeWidget()

        # Simulate: Model A has vertex morphs, Model B has bone morphs.
        # _create_tab appends new tabs after the constructor's 2 defaults.
        vertex_sources = [("Model A", _FakeMorphSource("bs1", ["v1", "v2"]))]
        bone_sources = [("Model B", _FakeMorphSource("bm1", ["b1", "b2"]))]

        widget._create_tab(vertex_sources, "Vertex")
        widget._create_tab(bone_sources, "Bone")

        # 2 constructor tabs + 2 _create_tab calls = 4
        self.assertEqual(widget._tabs.count(), 4)

        # First _create_tab tab is at index 2
        vtree = widget._tabs.widget(2).findChild(QtWidgets.QTreeWidget)
        self.assertEqual(vtree.topLevelItemCount(), 1)
        self.assertEqual(vtree.topLevelItem(0).childCount(), 2)

        # Second _create_tab tab is at index 3
        btree = widget._tabs.widget(3).findChild(QtWidgets.QTreeWidget)
        self.assertEqual(btree.topLevelItemCount(), 1)
        self.assertEqual(btree.topLevelItem(0).childCount(), 2)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_weight_widgets_registered(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Weight widgets are registered for attrs from all sources."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0

        widget = MorphTreeWidget()
        source_a = _FakeMorphSource("nodeA", ["t1", "t2"])
        source_b = _FakeMorphSource("nodeB", ["t3"])

        widget._create_tab([("A", source_a), ("B", source_b)], "Vertex")

        # Expected attrs: nodeA.t1, nodeA.t2, nodeB.t3, plus envelope attrs
        expected_attrs = {
            "nodeA.t1",
            "nodeA.t2",
            "nodeB.t3",
            "nodeA.envelope",
            "nodeB.envelope",
        }
        registered = set(widget._rows.keys())
        self.assertEqual(registered, expected_attrs)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_key_indicators_registered(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Key indicators are registered for attrs from all sources."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0

        widget = MorphTreeWidget()
        source_a = _FakeMorphSource("nodeA", ["x"])
        source_b = _FakeMorphSource("nodeB", ["y"])

        widget._create_tab([("A", source_a), ("B", source_b)], "Vertex")

        # Both target attrs and envelope attrs get key indicators (now via _rows)
        expected = {"nodeA.x", "nodeB.y", "nodeA.envelope", "nodeB.envelope"}
        registered = set(widget._rows.keys())
        self.assertEqual(registered, expected)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_with_envelope(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Sources with envelope=True get envelope controls."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 1.0

        widget = MorphTreeWidget()
        source = _FakeMorphSource("nodeA", ["t1"], have_envelope=True)

        widget._create_tab([("A", source)], "Vertex")

        # Envelope attr should be in _rows
        self.assertIn("nodeA.envelope", widget._rows)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_without_envelope(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Sources with envelope=False don't get envelope controls."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0

        widget = MorphTreeWidget()
        source = _FakeMorphSource("nodeA", ["t1"], have_envelope=False)

        widget._create_tab([("A", source)], "Vertex")

        # No envelope attr should be registered
        self.assertNotIn("nodeA.envelope", widget._rows)

        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_create_tab_callback_registration(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Multiple sources register callbacks for each unique node."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.getAttr.return_value = 0.0

        # Mock the callback registration
        mock_om.MNodeMessage.addAttributeChangedCallback.return_value = 42
        mock_om.MSelectionList.return_value.getDependNode.return_value = (
            mock_om.MObject()
        )

        widget = MorphTreeWidget()
        source_a = _FakeMorphSource("nodeA", ["t1", "t2"])
        source_b = _FakeMorphSource("nodeB", ["t3"])

        widget._create_tab([("A", source_a), ("B", source_b)], "Vertex")

        # Should have callbacks for both unique nodes
        self.assertIn("nodeA", widget._callbacks)
        self.assertIn("nodeB", widget._callbacks)

        widget.deleteLater()


# ═══════════════════════════════════════════════════════════════════════════
#  Callback lifecycle (cleanup, idempotency, deleteLater)
# ═══════════════════════════════════════════════════════════════════════════


class TestCallbackLifecycle(unittest.TestCase):
    """Tests for ``_cleanup_callbacks``, ``_register_node_callback``
    idempotency, and ``deleteLater`` safety."""

    def tearDown(self) -> None:
        MorphTreeWidget._active_instance = None

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_cleanup_callbacks_removes_all(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """_cleanup_callbacks stops timer, removes Maya callbacks, clears dicts."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_om.MNodeMessage.addAttributeChangedCallback.return_value = 42
        mock_om.MEventMessage.addEventCallback.return_value = 99

        widget = MorphTreeWidget()
        widget._register_node_callback("nodeA")
        widget._register_node_callback("nodeB")
        self.assertEqual(len(widget._callbacks), 2)
        self.assertIsNotNone(widget._time_callback)

        widget._cleanup_callbacks()

        self.assertEqual(len(widget._callbacks), 0)
        self.assertIsNone(widget._time_callback)
        self.assertFalse(widget._scrub_timer.isActive())
        mock_om.MNodeMessage.removeCallback.assert_called()
        mock_om.MEventMessage.removeCallback.assert_called()
        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_register_callback_idempotent(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Calling _register_node_callback twice for the same node only registers once."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_om.MNodeMessage.addAttributeChangedCallback.return_value = 42
        mock_om.MSelectionList.return_value.getDependNode.return_value = (
            mock_om.MObject()
        )

        widget = MorphTreeWidget()
        widget._register_node_callback("nodeX")
        first_count = len(widget._callbacks)
        widget._register_node_callback("nodeX")  # should be no-op
        self.assertEqual(len(widget._callbacks), first_count)
        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_deleteLater_does_not_raise(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """deleteLater on a populated widget does not raise or crash.

        Note: ``__del__`` is a safety net; normal cleanup happens via
        ``closeEvent`` in ``MMMToolWidget``.  This test just verifies
        that widget teardown doesn't raise exceptions even when the
        Maya callbacks haven't been explicitly cleaned up first.
        """
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_om.MNodeMessage.addAttributeChangedCallback.return_value = 1
        mock_om.MEventMessage.addEventCallback.return_value = 2

        widget = MorphTreeWidget()
        widget._register_node_callback("n")

        # Should not raise
        widget.deleteLater()
        QApplication.processEvents()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_refresh_invalid_type_returns_early_without_maya_calls(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Passing a non-string to refresh logs an error and skips all Maya work."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = False

        widget = MorphTreeWidget()
        widget.refresh(blend_shape_node=42, bone_morph_node=None)  # type: ignore[arg-type]

        # After early return, auto-discovery should NOT have run.
        mock_cmds.ls.assert_not_called()
        widget.deleteLater()


# ═══════════════════════════════════════════════════════════════════════════
#  Scrub-update (debounced keyframe-styling loop)
# ═══════════════════════════════════════════════════════════════════════════


class TestScrubUpdate(unittest.TestCase):
    """Tests for ``_do_scrub_update`` (debounced keyframe-state styling)."""

    def tearDown(self) -> None:
        MorphTreeWidget._active_instance = None

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_scrub_update_iterates_all_tracked_attrs(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """_do_scrub_update styles every tracked attr; currentTime queried once."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.currentTime.return_value = 5.0
        mock_cmds.keyframe.return_value = [5.0]
        mock_cmds.listConnections.return_value = ["animCurveTL1"]

        widget = MorphTreeWidget()
        s1, f1 = QtWidgets.QSlider(), QtWidgets.QDoubleSpinBox()
        s2, f2 = QtWidgets.QSlider(), QtWidgets.QDoubleSpinBox()
        widget._rows["node.t1"] = _MorphRow("node.t1", s1, f1, None, _CircleIndicator())
        widget._rows["node.t2"] = _MorphRow("node.t2", s2, f2, None, _CircleIndicator())

        widget._do_scrub_update()

        self.assertEqual(mock_cmds.currentTime.call_count, 1)
        red = _C["red_accent"].name()
        self.assertIn(red, f1.styleSheet())
        self.assertIn(red, f2.styleSheet())
        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_scrub_update_empty_is_noop(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """_do_scrub_update returns immediately when no weight widgets exist."""
        mock_cmds.ls.return_value = []
        widget = MorphTreeWidget()
        widget._do_scrub_update()
        mock_cmds.currentTime.assert_not_called()
        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_scrub_timer_debounces_on_time_changed(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Each timeChanged event restarts the 50 ms timer; update is deferred."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.currentTime.return_value = 1.0
        mock_cmds.listConnections.return_value = None

        widget = MorphTreeWidget()
        s, f = QtWidgets.QSlider(), QtWidgets.QDoubleSpinBox()
        widget._rows["a"] = _MorphRow("a", s, f, None, _CircleIndicator())

        # Fire timeChanged twice — timer should be active but not yet expired.
        MorphTreeWidget._on_time_changed()
        MorphTreeWidget._on_time_changed()
        self.assertTrue(widget._scrub_timer.isActive())
        mock_cmds.currentTime.assert_not_called()  # update hasn't run yet

        widget.deleteLater()


# ═══════════════════════════════════════════════════════════════════════════
#  Connection-change branch of _on_weight_changed
# ═══════════════════════════════════════════════════════════════════════════


class TestOnWeightChangedConnection(unittest.TestCase):
    """Tests for the connection-change branch of ``_on_weight_changed``.

    When an ``animCurve*`` node is connected to or disconnected from a
    tracked morph-weight attribute, the callback updates keyframe styling
    (weight-field colour + key-indicator dot) without touching weight
    values.
    """

    def tearDown(self) -> None:
        MorphTreeWidget._active_instance = None

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_connection_made_updates_keyframe_styling(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """kConnectionMade → set_keyframe_style styles spinbox + key dot."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.currentTime.return_value = 5.0
        mock_cmds.keyframe.return_value = [5.0]  # key at current time
        mock_cmds.listConnections.return_value = ["animCurveTL1"]

        widget = MorphTreeWidget()
        MorphTreeWidget._active_instance = widget

        _, f1 = QtWidgets.QSlider(), QtWidgets.QDoubleSpinBox()
        key_dot = _CircleIndicator()
        widget._rows["bs.smile"] = _MorphRow(
            "bs.smile", QtWidgets.QSlider(), f1, None, key_dot
        )

        # Simulate Maya callback with kConnectionMade flag
        msg = mock_om.MNodeMessage.kConnectionMade  # 0x0001
        plug = MagicMock()
        plug.name.return_value = "bs.smile"

        MorphTreeWidget._on_weight_changed(msg, plug, None, None)

        # Weight field should be styled as AT_CURRENT (key at current time)
        red = _C["red_accent"].name()
        self.assertIn(red, f1.styleSheet())
        # Indicator dot should be red
        self.assertEqual(key_dot.color(), _C["red_accent"])
        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_connection_broken_resets_keyframe_styling(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """kConnectionBroken → styling reset to NONE (no curves)."""
        mock_cmds.ls.return_value = []
        mock_cmds.objExists.return_value = True
        mock_cmds.listConnections.return_value = None  # no curves connected

        widget = MorphTreeWidget()
        MorphTreeWidget._active_instance = widget

        _, f1 = QtWidgets.QSlider(), QtWidgets.QDoubleSpinBox()
        key_dot = _CircleIndicator(color=_C["red_accent"])
        widget._rows["bs.frown"] = _MorphRow(
            "bs.frown", QtWidgets.QSlider(), f1, None, key_dot
        )

        msg = mock_om.MNodeMessage.kConnectionBroken  # 0x0002
        plug = MagicMock()
        plug.name.return_value = "bs.frown"

        MorphTreeWidget._on_weight_changed(msg, plug, None, None)

        # Weight field stylesheet should be cleared (KeyframeState.NONE)
        self.assertEqual(f1.styleSheet(), "")
        # Indicator dot should become black
        self.assertEqual(key_dot.color(), _C["black"])
        widget.deleteLater()

    @patch("mmd.ui.morph_tree_widget.cmds")
    @patch("mmd.ui.morph_tree_widget.om")
    def test_connection_change_ignores_untracked_attr(
        self, mock_om: MagicMock, mock_cmds: MagicMock
    ) -> None:
        """Connection events for attrs not in _weight_widgets are silently ignored."""
        mock_cmds.ls.return_value = []
        mock_cmds.listConnections.return_value = None

        widget = MorphTreeWidget()
        MorphTreeWidget._active_instance = widget

        msg = mock_om.MNodeMessage.kConnectionMade
        plug = MagicMock()
        plug.name.return_value = "untracked.node.attr"  # not registered

        # Should not raise
        MorphTreeWidget._on_weight_changed(msg, plug, None, None)
        widget.deleteLater()


if __name__ == "__main__":
    unittest.main()
