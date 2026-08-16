"""
Morph Tree Widget for MMD morph editing (vertex + bone morphs).

This module provides a custom QTreeWidget similar to Maya's Shape Editor,
with inline sliders, buttons, and hierarchical pose organization.

Architecture:
    ``_AbstractMorphSource`` defines the interface for enumerating and
    manipulating morph targets on a Maya node.  Two implementations ship
    out of the box:

    * ``_BlendShapeMorphSource`` — drives a standard Maya ``blendShape``
      deformer (used for PMX *vertex* morphs).
    * ``_BoneMorphSource`` — drives the custom ``boneMorphNode``
      (used for PMX *bone* morphs).

    Additional morph types (UV, material, …) can be supported later by
    adding new ``_AbstractMorphSource`` subclasses without touching the
    tree-widget code.
"""

from __future__ import annotations

import abc
import logging
import re
from enum import IntEnum

import maya.api.OpenMaya as om
from maya import cmds, mel
from PySide6 import QtCore, QtWidgets
from PySide6.QtCore import QPoint, QRect, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QHBoxLayout,
    QPushButton,
    QSlider,
    QStyledItemDelegate,
    QStyleOptionViewItem,
    QTreeWidget,
    QTreeWidgetItem,
    QWidget,
)

from mmd.maya.pmx_model_utils import (
    discover_model_roots_in_scene,
    find_blend_shape_node,
    find_bone_morph_node,
)

log = logging.getLogger(__name__)


def _safe_set_attr(attr_path: str, value: float) -> None:
    """Set a Maya attribute, logging warnings on failure."""
    try:
        cmds.setAttr(attr_path, value)
    except Exception as e:
        log.warning("Failed to set %s: %s", attr_path, e)


# ── Keyframe state enum ─────────────────────────────────────────


class KeyframeState(IntEnum):
    """Keyframe state for morph-weight attributes."""

    NONE = 0  # No animation curves connected
    BETWEEN = 1  # Has curves but current time is between keys
    AT_CURRENT = 2  # Has a key at the current time


# ── Color palette (consolidated QColor values for theming) ──────────
_C = {  # module-level constant
    "red_accent": QColor(0xCD, 0x27, 0x29),  # #CD2729
    "red_between": QColor(0xDD, 0x72, 0x7A),  # #DD727A
    "selection_blue": QColor(82, 133, 166),  # #5285A6
    "border_gray": QColor(147, 147, 147),  # #939393
    "fill_gray": QColor(68, 68, 68),  # #444
    "symbol_light": QColor(170, 170, 170),  # #AAA
    "black": QColor(0, 0, 0),
    # (spinbox_color, dot_color, dot_border) per keyframe state
    "keyframe_colors": {
        KeyframeState.AT_CURRENT: (
            QColor(0xCD, 0x27, 0x29),  # red_accent
            QColor(0xCD, 0x27, 0x29),  # red_accent
            QColor(0, 0, 0),  # black
        ),
        KeyframeState.BETWEEN: (
            QColor(0xDD, 0x72, 0x7A),  # red_between
            QColor(0, 0, 0),  # black
            None,
        ),
    },
}

# ── Shared stylesheets ──────────────────────────────────────────────
_STYLESHEETS = {  # module-level constant
    "tree": """
        QTreeView {
            background-color: #363636;
            show-decoration-selected: 1;
            border: none;
            outline: none;
        }
        QTreeView::item {
            padding: 4px 4px;
            border: none;
            height: 36px;
        }
        QTreeView::item:hover {
            background-color: #404040;
        }
        QTreeView::item:selected {
            background-color: #5285A6;
        }
        QTreeView::branch {
            background: transparent;
            border: none;
        }
    """,
    "round_toggle": """
        QPushButton {
            border: 2px solid #000000;
            border-radius: 8px;
            background-color: #000000;
            padding: 0px;
        }
        QPushButton:checked {
            background-color: #F0F0F0;
        }
        QPushButton:!checked {
            background-color: #000000;
        }
        QPushButton:hover {
            border: 1px solid #aaaaaa;
        }
    """,
    "tab": """
        QTabWidget {
            background-color: #444444;
        }
        QTabWidget::pane {
            border: none;
            background-color: #444444;
        }
        QTabWidget > QStackedWidget > QWidget {
            background-color: #444444;
        }
        QTabBar {
            background-color: #373737;
            qproperty-drawBase: 0;
        }
        QTabBar::tab {
            background-color: #373737;
            padding: 2px 10px;
            min-width: 80px;
            margin-right: 2px;
            margin-top: 2px;
            border: 1px solid #2B2B2B;
            border-bottom: none;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }
        QTabBar::tab:first {
            margin-left: 2px;
        }
        QTabBar::tab:selected {
            background-color: #444444;
            border: none;
        }
        QTabBar::tab:hover:!selected {
            background-color: #404040;
        }
    """,
    "weight_error": """
        QDoubleSpinBox {
            border: 2px solid #CD2729;
        }
    """,
    "weight_keyed": """
        QDoubleSpinBox {{
            background-color: {color};
            color: black;
            selection-background-color: darkred;
            border-radius: 3px;
            padding: 1px;
        }}
    """,
}

# ── Module-level widget factories ───────────────────────────────────


def _create_round_toggle(checked: bool = True, tooltip: str = "") -> QPushButton:
    """Create a Maya-style round visibility toggle button (16×16 checkable circle)."""
    btn = QPushButton("")
    btn.setCheckable(True)
    btn.setChecked(checked)
    btn.setFixedSize(16, 16)
    btn.setFlat(True)
    if tooltip:
        btn.setToolTip(tooltip)
    btn.setStyleSheet(_STYLESHEETS["round_toggle"])
    return btn


def _wrap_centered(child: QWidget) -> QWidget:
    """Wrap *child* in a centered container for tree item widgets."""
    w = QWidget()
    ly = QHBoxLayout(w)
    ly.setContentsMargins(0, 0, 0, 0)
    ly.setAlignment(Qt.AlignmentFlag.AlignCenter)
    ly.addWidget(child)
    return w


def _create_keyframe_dot(active: bool = False) -> _CircleIndicator:
    """Create a keyframe dot widget (black when inactive, red when active)."""
    return _CircleIndicator(
        size=17,
        color=_C["red_accent"] if active else _C["black"],
    )


# ═══════════════════════════════════════════════════════════════════════════
#  Morph Source Abstraction
# ═══════════════════════════════════════════════════════════════════════════


class _AbstractMorphSource(abc.ABC):
    """Protocol for a Maya node that exposes morph-target weights.

    Subclasses encapsulate the differences between a native Maya
    ``blendShape`` deformer and the custom ``boneMorphNode`` so that
    ``MorphTreeWidget`` can treat them uniformly.
    """

    def __init__(self, node_name: str) -> None:
        self._node = node_name

    @property
    def node_name(self) -> str:
        """The Maya DG node name (e.g. ``"PMX_Model_BlendShape"``)."""
        return self._node

    @abc.abstractmethod
    def section_label(self) -> str:
        """Human-readable section header shown in the tree tab."""
        ...

    @abc.abstractmethod
    def get_targets(self) -> list[str]:
        """Return the list of morph-target names (aliases)."""
        ...

    def get_weight_attr(self, target_name: str) -> str:
        """Full Maya plug path for a target's weight (e.g.
        ``"blendShape1.eye_smile"``)."""
        return f"{self._node}.{target_name}"

    def get_envelope_attr(self) -> str | None:
        """Full Maya plug path for the node's envelope, or ``None``."""
        if cmds.objExists(self._node) and cmds.attributeQuery(
            "envelope", node=self._node, exists=True
        ):
            return f"{self._node}.envelope"
        return None

    def get_edit_action_label(self) -> str | None:
        """Label for the Edit button, or ``None`` to hide it.

        The default returns ``None`` (no Edit button).  Override in
        subclasses that have a meaningful edit action.
        """
        return None

    def is_edit_enabled(self) -> bool:
        """Whether the Edit button should be enabled.

        Returns ``True`` by default.  Override in subclasses where
        editing is not yet supported (button will appear disabled
        with an explanatory tooltip).
        """
        return True

    def on_edit_clicked(self, _target_name: str) -> None:
        """Called when the user clicks the Edit button for a target.

        The default is a no-op.  Override in subclasses.
        """


# ── BlendShape (vertex morph) source ───────────────────────────────────────


class _BlendShapeMorphSource(_AbstractMorphSource):
    """Morph source wrapping a standard Maya ``blendShape`` deformer."""

    def section_label(self) -> str:
        return "Vertex"

    def get_targets(self) -> list[str]:
        if not cmds.objExists(self._node):
            return []
        pairs: list[str] = cmds.aliasAttr(self._node, query=True) or []
        # aliasAttr returns [alias1, attr1, alias2, attr2, …]
        # Build attr-name → alias mapping
        alias_map: dict[str, str] = {}
        for i in range(0, len(pairs), 2):
            alias_map[pairs[i + 1]] = pairs[i]

        # Get weight indices in their natural index order (matching Shape Editor)
        indices: list[int] = (
            cmds.getAttr(f"{self._node}.weight", multiIndices=True) or []
        )
        return [
            alias_map[f"weight[{idx}]"]
            for idx in indices
            if f"weight[{idx}]" in alias_map
        ]

    def get_edit_action_label(self) -> str | None:
        return "Edit"

    def is_edit_enabled(self) -> bool:
        return False  # Editing morph target data is not yet supported

    def on_edit_clicked(self, _target_name: str) -> None:
        """Open Maya's native Shape Editor so the user can sculpt."""
        try:
            mel.eval("ShapeEditor;")
        except Exception as exc:
            log.warning("Failed to open Shape Editor: %s", exc)


# ── Bone-morph source ──────────────────────────────────────────────────────


class _BoneMorphSource(_AbstractMorphSource):
    """Morph source wrapping the custom ``boneMorphNode`` MPxNode."""

    def section_label(self) -> str:
        return "Bone"

    def get_targets(self) -> list[str]:
        if not cmds.objExists(self._node):
            return []
        try:
            return cmds.boneBlendShape(self._node, query=True, listTargets=True) or []
        except Exception as exc:
            log.error("Failed to query targets from '%s': %s", self._node, exc)
            return []

    def get_edit_action_label(self) -> str | None:
        return "Edit"

    def is_edit_enabled(self) -> bool:
        return False  # Editing morph target data is not yet supported


class _CircleIndicator(QWidget):
    """A fixed-size widget that paints a filled circle via ``paintEvent``.

    Bypasses the stylesheet engine entirely — works reliably inside
    Maya\'s custom Qt style which often ignores ``border-radius``.
    """

    def __init__(
        self,
        size: int = 13,
        color: QColor | None = None,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._color = color if color is not None else _C["black"]
        self._border_color: QColor | None = None
        self.setFixedSize(size, size)

    # ── Public API ─────────────────────────────────────────────────

    def setColor(self, color: QColor) -> None:
        """Change the fill colour and trigger a repaint."""
        if self._color != color:
            self._color = color
            self.update()

    def color(self) -> QColor:
        """Return the current fill colour."""
        return self._color

    # ── Painting ────────────────────────────────────────────────

    def setBorderColor(self, color: QColor | None) -> None:
        """Set the border colour, or ``None`` for no border."""
        self._border_color = color
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(QBrush(self._color))
        if self._border_color is not None:
            painter.setPen(QPen(self._border_color, 1))
        else:
            painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(self.rect())


class MayaStyleTreeDelegate(QStyledItemDelegate):
    """
    Custom delegate to draw tree branches like Maya's Shape Editor.

    Features:
    - Expand/collapse indicators (square boxes with +/-)
    - Vertical tree lines connecting parent to children
    - Horizontal connector lines to each item
    - L-shaped junctions at last child
    - Small circle indicators for leaf items
    """

    def __init__(self, parent: QTreeWidget | None = None):
        super().__init__(parent)
        self.tree_widget: QTreeWidget | None = parent

    def paint(self, painter: QPainter, option, index):
        """Custom paint to draw tree decorations like Maya."""
        if index.column() != 0:
            super().paint(painter, option, index)
            return

        if not self.tree_widget:
            super().paint(painter, option, index)
            return

        item = self.tree_widget.itemFromIndex(index)
        if not item:
            super().paint(painter, option, index)
            return

        has_children = item.childCount() > 0

        # 1. Fill the full row background before drawing text/decorations.
        #    super().paint() with an offset rect only highlights the text
        #    area, leaving the decoration gutter unselected.
        if option.state & QtWidgets.QStyle.State_Selected:
            painter.fillRect(option.rect, _C["selection_blue"])

        # 2. Let Qt draw background + text.  Offset text so it doesn't
        #    overlap the decorations (expand button / tree lines / dot).
        text_option = QStyleOptionViewItem(option)
        offset = 25 if has_children else 12
        text_option.rect = option.rect.adjusted(offset, 0, 0, 0)
        super().paint(painter, text_option, index)

        # 3. Draw decorations on top over the (now highlighted) indent area.
        painter.save()
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        indent = self.tree_widget.indentation()
        decoration_x = option.rect.left() + 5
        row_center_y = option.rect.top() + option.rect.height() // 2

        if has_children:
            self._draw_expand_indicator(painter, item, decoration_x, row_center_y)
        if item.parent():
            self._draw_tree_lines(painter, item, option.rect, row_center_y, indent)

        painter.restore()

    def _draw_expand_indicator(
        self, painter: QPainter, item: QTreeWidgetItem, x: int, center_y: int
    ):
        """Draw expand/collapse square button like Maya."""
        box_size = 14
        box_x = x
        box_y = center_y - box_size // 2

        sel = item.isSelected()
        border_color = _C["border_gray"]
        fill_color = _C["selection_blue"] if sel else _C["fill_gray"]
        symbol_color = _C["symbol_light"]

        # Draw box background
        painter.setPen(QPen(border_color, 1))
        painter.setBrush(QBrush(fill_color))
        painter.drawRect(box_x, box_y, box_size, box_size)

        # Draw +/- symbol
        painter.setPen(QPen(symbol_color, 2))

        center_x = box_x + box_size // 2
        center_box_y = box_y + box_size // 2
        painter.drawLine(box_x + 3, center_box_y, box_x + box_size - 3, center_box_y)

        if item.isExpanded():
            painter.setPen(QPen(border_color, 3))
            painter.drawLine(
                center_x, box_y + box_size, center_x, box_y + box_size + 16
            )
        else:
            painter.drawLine(center_x, box_y + 3, center_x, box_y + box_size - 3)

    def _draw_tree_lines(
        self,
        painter: QPainter,
        item: QTreeWidgetItem,
        rect: QRect,
        center_y: int,
        indent: int,
    ):
        """Draw tree connection lines like Maya.

        option.rect.left() is already indented by Qt for the current level.
        So parent's expand button center is at rect.left() - indent + 5 + 7.
        """
        parent = item.parent()
        if not parent:
            return

        # This child's expand button starts at rect.left() + 5 (Qt already indented rect)
        # Parent's expand button starts at (rect.left() - indent) + 5
        # Parent's button center X = rect.left() - indent + 5 + 7 = rect.left() - indent + 12
        parent_button_center_x = rect.left() - indent + 12

        # Horizontal line from parent's vertical line to just before this item
        horizontal_start_x = parent_button_center_x
        horizontal_end_x = rect.left() + 5  # Start of this item's area

        line_color = _C["border_gray"]
        painter.setPen(QPen(line_color, 3))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawLine(horizontal_start_x, center_y, horizontal_end_x, center_y)

        # Vertical line at parent's button center
        is_last_child = parent.indexOfChild(item) == parent.childCount() - 1

        if is_last_child:
            # L-junction: vertical line only from top of row to center
            painter.drawLine(
                parent_button_center_x, rect.top(), parent_button_center_x, center_y
            )
        else:
            # T-junction: vertical line spans full row height to connect to next sibling
            painter.drawLine(
                parent_button_center_x,
                rect.top(),
                parent_button_center_x,
                rect.bottom(),
            )

        # Draw small circle at the end of the horizontal line for leaf items.
        painter.setBrush(line_color)
        painter.setPen(QPen(line_color, 1))
        painter.drawEllipse(QPoint(horizontal_end_x, center_y), 3, 3)

    def editorEvent(self, event, model, option, index):
        """
        Handle mouse events for custom expand/collapse button.
        """
        if index.column() != 0 or not self.tree_widget:
            return super().editorEvent(event, model, option, index)

        item = self.tree_widget.itemFromIndex(index)
        if not item or item.childCount() == 0:
            return super().editorEvent(event, model, option, index)

        # Calculate button rect (must match _draw_expand_indicator)
        decoration_x = option.rect.left() + 5
        row_center_y = option.rect.top() + option.rect.height() // 2
        box_size = 14
        box_x = decoration_x
        box_y = row_center_y - box_size // 2
        box_rect = QRect(box_x, box_y, box_size, box_size)

        if event.type() == QtCore.QEvent.Type.MouseButtonRelease:
            mouse_event = event
            if box_rect.contains(mouse_event.position().toPoint()):
                # Toggle expand/collapse using QModelIndex
                self.tree_widget.setExpanded(
                    index, not self.tree_widget.isExpanded(index)
                )
                return True

        return super().editorEvent(event, model, option, index)


def _create_tree() -> QTreeWidget:
    """Create and return a QTreeWidget with Maya-style setup."""
    tree = QTreeWidget()

    tree.setHeaderLabels(["Name", "", "Weight", "Edit", "Key"])
    tree.setColumnCount(5)
    tree.setRootIsDecorated(False)
    tree.setItemsExpandable(True)
    tree.setAlternatingRowColors(False)
    tree.setSelectionMode(QTreeWidget.SelectionMode.SingleSelection)
    tree.setIndentation(30)

    header = tree.header()
    header.setDefaultAlignment(Qt.AlignmentFlag.AlignCenter)
    header.setStretchLastSection(False)
    header.setMinimumSectionSize(0)
    header.setSectionResizeMode(0, header.ResizeMode.Interactive)
    header.setSectionResizeMode(1, header.ResizeMode.Fixed)
    header.setSectionResizeMode(2, header.ResizeMode.Stretch)
    header.setSectionResizeMode(3, header.ResizeMode.Fixed)
    header.setSectionResizeMode(4, header.ResizeMode.Fixed)
    tree.setColumnWidth(0, 250)
    tree.setColumnWidth(1, 50)
    tree.setColumnWidth(3, 60)
    tree.setColumnWidth(4, 60)
    header.moveSection(1, 0)

    tree.setUniformRowHeights(True)

    delegate = MayaStyleTreeDelegate(tree)
    tree.setItemDelegateForColumn(0, delegate)

    tree.setStyleSheet(_STYLESHEETS["tree"])

    return tree


def _get_keyframe_state(attr_path: str, current_time: float | None = None) -> int:
    """Determine the keyframe state for an attribute.

    Args:
        attr_path: Full Maya plug path (e.g. ``"blendShape1.smile"``).
        current_time: The current timeline frame.  When ``None``
            (default) it is queried from Maya.

    Returns:
        ``KeyframeState.NONE`` (0): No animation curves connected.
        ``KeyframeState.BETWEEN`` (1): Has curves but current time is between keys.
        ``KeyframeState.AT_CURRENT`` (2): Current time is exactly on a keyframe.
    """
    try:
        if not cmds.listConnections(attr_path, type="animCurve"):
            return KeyframeState.NONE
        if current_time is None:
            current_time = cmds.currentTime(query=True)
        key_times = cmds.keyframe(attr_path, query=True, timeChange=True) or []
        if any(abs(t - current_time) < 0.001 for t in key_times):
            return KeyframeState.AT_CURRENT
        return KeyframeState.BETWEEN
    except Exception:
        return KeyframeState.NONE


class _MorphRow:
    """Bundles the widgets and state for one morph-target row.

    Replaces the four separate tracking dicts (``_weight_widgets``,
    ``_key_indicators``, ``_vis_toggles``, ``_saved_weights``) with a
    single object per target.

    Handles slider—spinbox sync, visibility-toggle logic, and Maya
    attribute writing, keeping ``MorphTreeWidget`` focused on tree
    construction.
    """

    __slots__ = (
        "__weakref__",
        "_edit_button",
        "_edit_supported",
        "key_dot",
        "node_name",
        "saved_weight",
        "slider",
        "spinbox",
        "vis_btn",
        "weight_attr",
    )

    def __init__(
        self,
        weight_attr: str,
        slider: QSlider,
        spinbox: QtWidgets.QDoubleSpinBox,
        vis_btn: QPushButton | None,
        key_dot: _CircleIndicator,
    ) -> None:
        self.weight_attr = weight_attr
        self.node_name = weight_attr.rsplit(".", 1)[0]
        self.slider = slider
        self.spinbox = spinbox
        self.vis_btn = vis_btn
        self.key_dot = key_dot
        self.saved_weight: float = 0.0
        self._edit_button: QPushButton | None = None
        self._edit_supported: bool = False

        # ── Wire slider ↔ spinbox sync ──────────────────────────
        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spinbox.valueChanged.connect(self._on_spinbox_changed)

        # ── Wire visibility toggle (may be absent for envelopes) ──
        if self.vis_btn is not None:
            self.vis_btn.toggled.connect(self._on_vis_toggled)

        # ── Read initial Maya value ───────────────────────────────
        if cmds.objExists(self.node_name):
            try:
                self.sync_value(cmds.getAttr(weight_attr))
            except Exception:
                pass

    # ── Public API ─────────────────────────────────────────────────

    def set_edit_button(
        self, btn: QPushButton | None, edit_supported: bool = False
    ) -> None:
        self._edit_button = btn
        self._edit_supported = edit_supported

    @property
    def is_enabled(self) -> bool:
        return self.slider.isEnabled()

    def sync_value(self, value: float) -> None:
        """Set slider and spinbox to *value* without triggering signals."""
        self.slider.blockSignals(True)
        self.spinbox.blockSignals(True)
        self.slider.setValue(int(value * 1000))
        self.spinbox.setValue(value)
        self.slider.blockSignals(False)
        self.spinbox.blockSignals(False)

    def set_keyframe_style(self, state: int) -> None:
        """Apply keyframe styling to both the spinbox and the key dot."""
        spinbox_color, dot_color, dot_border = _C["keyframe_colors"].get(
            state, (None, _C["black"], None)
        )
        self.spinbox.setStyleSheet(
            _STYLESHEETS["weight_keyed"].format(color=spinbox_color.name())
            if spinbox_color is not None
            else ""
        )
        self.key_dot.setColor(dot_color)
        self.key_dot.setBorderColor(dot_border)

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable the slider, spinbox, and optional edit button."""
        self.slider.setEnabled(enabled)
        self.spinbox.setEnabled(enabled)
        if self._edit_button is not None:
            if enabled:
                self._edit_button.setEnabled(self._edit_supported)
            else:
                self._edit_button.setEnabled(False)

    # ── Private: Maya I/O & signal handlers ────────────────────────

    def _write_maya(self, value: float) -> None:
        """Write *value* to the Maya attribute, with error feedback."""
        if not cmds.objExists(self.node_name):
            return
        try:
            cmds.setAttr(self.weight_attr, value)
            self.spinbox.setToolTip("")
            self.set_keyframe_style(_get_keyframe_state(self.weight_attr))
        except Exception as e:
            log.warning("Failed to set %s: %s", self.weight_attr, e)
            self.spinbox.setStyleSheet(_STYLESHEETS["weight_error"])
            self.spinbox.setToolTip(f"Failed to set {self.weight_attr}: {e}")

    def _on_slider_changed(self, int_value: int) -> None:
        value = int_value / 1000.0
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(value)
        self.spinbox.blockSignals(False)
        self._write_maya(value)

    def _on_spinbox_changed(self, float_value: float) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(int(float_value * 1000))
        self.slider.blockSignals(False)
        self._write_maya(float_value)

    def _on_vis_toggled(self, checked: bool) -> None:
        if checked:
            self.set_enabled(True)
            self._write_maya(self.saved_weight)
        else:
            self.saved_weight = self.slider.value() / 1000.0
            self.set_enabled(False)
            self._write_maya(0.0)


class MorphTreeWidget(QWidget):
    """
    Tabbed morph editor for MMD models (blend shapes + bone morphs).

    Each morph type gets its own tab with an envelope checkbox and a
    tree of targets with inline weight sliders.

    Features:
    - One tab per morph source (e.g. "Vertex", "Bone")
    - Inline weight sliders with numeric input fields
    - Envelope control per tab
    - Edit button opens Maya Shape Editor for vertex morphs
    - Maya-style tree decorations via custom delegate
    - Bidirectional sync with Maya Shape Editor
    - Extensible via :class:`_AbstractMorphSource` subclasses
    """

    _active_instance: MorphTreeWidget | None = None  # for Maya callbacks

    def __init__(self, parent=None):
        super().__init__(parent)

        # Make this widget greedily take all available vertical space.
        expand = QtWidgets.QSizePolicy.Policy.Expanding
        self.setSizePolicy(expand, expand)

        self._tabs = QtWidgets.QTabWidget()
        self._tabs.setDocumentMode(True)
        self._tabs.setStyleSheet(_STYLESHEETS["tab"])
        self._tabs.setSizePolicy(expand, expand)

        # Bidirectional sync with Maya (Shape Editor → widget).
        # MNodeMessage fires on DG evaluation (slider release) — not
        # during interactive drag, but provides clean, predictable updates.
        self._rows: dict[str, _MorphRow] = {}  # attr_path → row bundle
        self._callbacks: dict[str, int] = {}  # node_name → callback_id
        self._time_callback: int | None = None
        self._anim_curve_attrs: set[str] = set()  # attr_paths known to have anim curves
        self._scrub_timer = QtCore.QTimer(self)
        self._scrub_timer.setSingleShot(True)
        self._scrub_timer.setInterval(50)  # ms — fire only after scrubbing pauses
        self._scrub_timer.timeout.connect(self._do_scrub_update)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self._tabs)

        # Ensure the two placeholder tabs are visible immediately, even
        # before a model is imported or selected.
        self.refresh()

        # Wire this instance so Maya callbacks can find it
        MorphTreeWidget._active_instance = self
        self._register_time_callback()

        log.debug("MorphTreeWidget initialized")

    def __del__(self) -> None:
        """Safety net: clean up Maya callbacks if the widget is garbage-
        collected without :meth:`_cleanup_callbacks` having been called.

        In normal operation ``MMMToolWidget.closeEvent`` calls
        ``_cleanup_callbacks()`` before the C++ wrapper is destroyed.
        This ``__del__`` handles edge cases where the Python object
        outlives the C++ side.
        """
        try:
            self._cleanup_callbacks()
        except Exception:
            pass

    # ── Public API ────────────────────────────────────────────────────────

    def refresh(
        self,
        blend_shape_node: str | None = None,
        bone_morph_node: str | None = None,
    ) -> None:
        """Populate tabs from blendShape and/or boneMorphNode nodes.

        Pass ``None`` to auto-discover, ``""`` to show an empty tab.
        """
        while self._tabs.count() > 0:
            self._tabs.removeTab(0)
        self._cleanup_callbacks()
        self._rows.clear()
        self._anim_curve_attrs.clear()
        self._register_time_callback()

        vertex_sources, bone_sources = self._collect_sources(
            blend_shape_node, bone_morph_node
        )
        v_label = vertex_sources[0][1].section_label() if vertex_sources else "Vertex"
        b_label = bone_sources[0][1].section_label() if bone_sources else "Bone"
        self._create_tab(vertex_sources, v_label)
        self._create_tab(bone_sources, b_label)
        log.debug(
            "Tabs populated with %d model source(s)",
            len(vertex_sources) + len(bone_sources),
        )

    def _collect_sources(
        self,
        blend_shape_node: str | None,
        bone_morph_node: str | None,
    ) -> tuple[
        list[tuple[str, _AbstractMorphSource]], list[tuple[str, _AbstractMorphSource]]
    ]:
        """Return (vertex_sources, bone_sources) for the given or auto-discovered nodes."""
        v: list[tuple[str, _AbstractMorphSource]] = []
        b: list[tuple[str, _AbstractMorphSource]] = []

        if blend_shape_node is not None or bone_morph_node is not None:
            if blend_shape_node and cmds.objExists(blend_shape_node):
                v.append(
                    (
                        self._model_display_name(blend_shape_node),
                        _BlendShapeMorphSource(blend_shape_node),
                    )
                )
            if bone_morph_node and cmds.objExists(bone_morph_node):
                b.append(
                    (
                        self._model_display_name(bone_morph_node),
                        _BoneMorphSource(bone_morph_node),
                    )
                )
        else:
            for root in discover_model_roots_in_scene():
                name = self._model_display_name(root)
                if bs := find_blend_shape_node(root):
                    v.append((name, _BlendShapeMorphSource(bs)))
                if bm := find_bone_morph_node(root):
                    b.append((name, _BoneMorphSource(bm)))
        return v, b

    @staticmethod
    def _model_display_name(node_or_root: str) -> str:
        """Strip DAG path and naming-convention suffix (e.g. ``_BlendShape``)."""
        return re.sub(
            r"_(BlendShape|BoneMorph|Root)$", "", node_or_root.rsplit("|", 1)[-1]
        )

    # ── Tab construction ──────────────────────────────────────────────────

    def _create_tab(
        self,
        sources: list[tuple[str, _AbstractMorphSource]],
        tab_label: str,
    ) -> None:
        """Create a tab with a tree; each source becomes a collapsible parent.

        When *sources* is empty the tab shows an empty tree so the widget
        never collapses to nothing.
        """
        tab_widget = QWidget()
        tab_layout = QtWidgets.QVBoxLayout(tab_widget)
        tab_layout.setContentsMargins(4, 25, 4, 4)
        tab_layout.setSpacing(4)

        tree = _create_tree()
        tab_layout.addWidget(tree, stretch=1)

        for model_label, source in sources:
            # ── Parent row: model name, collapsible ──────────────────
            parent_item = QTreeWidgetItem(tree, [model_label, "", "", "", ""])
            parent_item.setExpanded(True)
            parent_item.setSizeHint(0, QtCore.QSize(0, 40))

            # Envelope checkbox on parent row (leftmost visual column)
            envelope_attr = source.get_envelope_attr()
            if envelope_attr:
                checkbox = self._create_envelope_control(envelope_attr)
                tree.setItemWidget(parent_item, 1, checkbox)
                self._install_weight_widget(tree, parent_item, envelope_attr)

            # Child targets
            for target_name in source.get_targets():
                self._create_target_item(tree, parent_item, source, target_name)

        self._tabs.addTab(tab_widget, tab_label)

        # Register Maya callbacks for all unique nodes in this tab
        for _, source in sources:
            self._register_node_callback(source.node_name)

        log.debug("Created tab '%s'", tab_label)

    # ── Target item ───────────────────────────────────────────────────────

    def _make_target_edit_button(
        self,
        source: _AbstractMorphSource,
        target_name: str,
    ) -> QPushButton | None:
        """Create an Edit button for a morph target, or ``None`` if not applicable.

        When editing is not supported (:meth:`_AbstractMorphSource.is_edit_enabled`
        returns ``False``) the button is rendered disabled with an explanatory
        tooltip.
        """
        edit_label = source.get_edit_action_label()
        if not edit_label:
            return None
        edit_enabled = source.is_edit_enabled()
        btn = QPushButton(edit_label)
        btn.setFixedHeight(25)
        btn.setEnabled(edit_enabled)
        if edit_enabled:
            btn.setToolTip(f"Edit '{target_name}' pose")
            btn.clicked.connect(
                lambda _checked=False, s=source, t=target_name: s.on_edit_clicked(t)
            )
        else:
            btn.setToolTip("Editing is not yet supported")
        return btn

    def _install_weight_widget(
        self,
        tree: QTreeWidget,
        item: QTreeWidgetItem,
        weight_attr: str,
        *,
        with_vis_toggle: bool = False,
    ) -> _MorphRow:
        """Create slider, keyframe dot, and (optionally) vis toggle for *item*.

        Returns the ``_MorphRow`` so callers can further configure it.
        """
        weight_container, slider, spinbox = MorphTreeWidget._create_weight_widget()
        tree.setItemWidget(item, 2, weight_container)

        key_state = _get_keyframe_state(weight_attr)
        if key_state != KeyframeState.NONE:
            self._anim_curve_attrs.add(weight_attr)

        key_dot = _create_keyframe_dot(active=(key_state == KeyframeState.AT_CURRENT))
        tree.setItemWidget(item, 4, _wrap_centered(key_dot))

        # Visibility toggle (only for morph targets, not envelopes)
        if with_vis_toggle:
            vis_btn = _create_round_toggle(
                checked=True, tooltip="Toggle morph visibility"
            )
            tree.setItemWidget(item, 1, _wrap_centered(vis_btn))
        else:
            vis_btn = None

        row = _MorphRow(weight_attr, slider, spinbox, vis_btn, key_dot)
        row.set_keyframe_style(key_state)
        self._rows[weight_attr] = row
        return row

    def _create_target_item(
        self,
        tree: QTreeWidget,
        parent_item: QTreeWidgetItem,
        source: _AbstractMorphSource,
        target_name: str,
    ) -> None:
        """Create a tree item for a single morph target."""
        child_item = QTreeWidgetItem(parent_item, [target_name, "", "", "", ""])
        child_item.setSizeHint(0, QtCore.QSize(0, 40))

        weight_attr = source.get_weight_attr(target_name)

        # Column 3: Edit button
        edit_button = self._make_target_edit_button(source, target_name)
        if edit_button is not None:
            tree.setItemWidget(child_item, 3, _wrap_centered(edit_button))

        # Columns 1, 2, 4: vis toggle, weight slider, keyframe dot
        row = self._install_weight_widget(
            tree, child_item, weight_attr, with_vis_toggle=True
        )
        row.set_edit_button(edit_button, source.is_edit_enabled())

    # ── Maya callback helpers (bidirectional sync) ───────────────────

    def _register_time_callback(self) -> None:
        """Register the timeChanged Maya callback if not already active."""
        if self._time_callback is not None:
            return
        try:
            self._time_callback = om.MEventMessage.addEventCallback(
                "timeChanged", MorphTreeWidget._on_time_changed
            )
        except Exception as exc:
            log.warning("Failed to register time callback: %s", exc)

    def _cleanup_callbacks(self) -> None:
        """Remove all registered Maya attribute-change callbacks."""
        self._scrub_timer.stop()
        for _node, cid in list(self._callbacks.items()):
            try:
                om.MNodeMessage.removeCallback(cid)
            except Exception:
                pass
        self._callbacks.clear()
        if self._time_callback is not None:
            try:
                om.MEventMessage.removeCallback(self._time_callback)
            except Exception:
                pass
            self._time_callback = None

    def _register_node_callback(self, node_name: str) -> None:
        """Register a single Maya callback for *node_name* if not already."""
        if node_name in self._callbacks or not cmds.objExists(node_name):
            return
        try:
            sel = om.MSelectionList()
            sel.add(node_name)
            mobj = sel.getDependNode(0)
            cid = om.MNodeMessage.addAttributeChangedCallback(
                mobj, self._on_weight_changed
            )
            self._callbacks[node_name] = cid
        except Exception as exc:
            log.warning("Failed to watch %s: %s", node_name, exc)

    @staticmethod
    def _on_weight_changed(
        msg: om.MNodeMessage,
        plug: om.MPlug,
        other_plug: om.MPlug | None,
        client_data: object,
    ) -> None:
        """Maya callback — sync slider when a tracked weight attr changes.

        * Value change + slider enabled: update slider/spinbox.
        * Value change + slider disabled (widget toggle was OFF): if weight
          became non-zero an external source (e.g. Shape Editor) re-enabled
          the target — re-enable controls and restore the new value.
        * Connection change: update keyframe styling only.

        Note: We intentionally do NOT sync the visibility toggle button
        from weight changes, because weight=0 is ambiguous — it could be
        a manual slider drag or an external toggle-off.  The toggle only
        responds to explicit user clicks.
        """
        widget = MorphTreeWidget._active_instance
        if widget is None:
            return
        try:
            attr_path = plug.name()
        except Exception:
            return

        # ── Quick rejection: ignore untracked attributes ──────────
        row = widget._rows.get(attr_path)
        if row is None:
            return

        # ── Connection change → update keyed styling ──────────────
        if msg & (om.MNodeMessage.kConnectionMade | om.MNodeMessage.kConnectionBroken):
            state = _get_keyframe_state(attr_path)
            # Keep anim-curve cache in sync
            if state == KeyframeState.NONE:
                widget._anim_curve_attrs.discard(attr_path)
            else:
                widget._anim_curve_attrs.add(attr_path)
            row.set_keyframe_style(state)
            return

        # ── Value change + external re-enable handling ────────────
        try:
            value = cmds.getAttr(attr_path)
        except Exception:
            return

        if row.is_enabled:
            row.sync_value(value)
        elif abs(value) >= 0.001:
            row.saved_weight = value
            row.set_enabled(True)
            row.sync_value(value)
            if row.vis_btn is not None:
                row.vis_btn.blockSignals(True)
                row.vis_btn.setChecked(True)
                row.vis_btn.blockSignals(False)

    @staticmethod
    def _on_time_changed(*_: object) -> None:
        """Maya callback — schedule UI refresh + restart the debounce timer.

        Uses ``QTimer.singleShot(0, …)`` to defer the weight read to
        the next Qt event-loop iteration, giving Maya's DG time to
        finish evaluating the current frame.  This is the recommended
        pattern for Maya UI plugins that read DG values during playback.
        """
        widget = MorphTreeWidget._active_instance
        if widget is None:
            return
        QtCore.QTimer.singleShot(0, widget._refresh_animated_values)
        widget._scrub_timer.start()

    def _refresh_animated_values(self) -> None:
        """Read current weight values from Maya and update all sliders.

        Queries every attribute in :attr:`_anim_curve_attrs` via
        ``cmds.getAttr`` — no caching, no manual DG evaluation.
        Works regardless of what drives the weight (animCurve,
        pairBlend, expression, custom node, …).
        """
        if not self._anim_curve_attrs:
            return

        for attr_path in list(self._anim_curve_attrs):
            row = self._rows.get(attr_path)
            if row is None or not row.is_enabled:
                continue
            try:
                value = cmds.getAttr(attr_path)
            except Exception:
                continue
            row.sync_value(value)

    def _do_scrub_update(self) -> None:
        """Run the styling update once per scrub pause (keyframe dots, colours).

        Does NOT touch slider/spinbox values — those are kept in sync
        on every frame by :meth:`_refresh_animated_values`.
        """
        if not self._rows:
            return

        try:
            current_time = cmds.currentTime(query=True)
        except Exception:
            return

        for row in self._rows.values():
            state = _get_keyframe_state(row.weight_attr, current_time)
            row.set_keyframe_style(state)

    # ── Widget factories ──────────────────────────────────────────────────

    def _create_envelope_control(self, envelope_attr: str) -> QWidget:
        """Create a round envelope toggle bound to *envelope_attr*."""
        node_name = envelope_attr.rsplit(".", 1)[0]
        btn = _create_round_toggle(
            checked=True, tooltip="Enable/disable all targets for this model"
        )
        if not cmds.objExists(node_name):
            btn.setEnabled(False)
            btn.setToolTip("Envelope attribute not available")
            return _wrap_centered(btn)

        try:
            btn.setChecked(cmds.getAttr(envelope_attr) > 0.5)
        except Exception:
            pass

        btn.toggled.connect(
            lambda checked, a=envelope_attr: _safe_set_attr(a, 1.0 if checked else 0.0)
        )
        return _wrap_centered(btn)

    @staticmethod
    def _create_weight_widget() -> tuple[QWidget, QSlider, QtWidgets.QDoubleSpinBox]:
        """Create a bare weight slider + numeric spinbox in a container.

        Signal wiring and Maya I/O are handled by :class:`_MorphRow`.
        """
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(5)

        spinbox = QtWidgets.QDoubleSpinBox()
        spinbox.setRange(0.0, 1.0)
        spinbox.setSingleStep(0.01)
        spinbox.setDecimals(3)
        spinbox.setFixedWidth(70)
        spinbox.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 1000)

        layout.addWidget(spinbox)
        layout.addWidget(slider, stretch=1)

        return container, slider, spinbox

    # ── Keyed / animation curve styling ───────────────────────────────────
    # ``_get_keyframe_state`` is defined at module level (above ``_MorphRow``)
    # so it can be called from both ``_MorphRow`` and ``MorphTreeWidget``
    # without a cross-class dependency.
