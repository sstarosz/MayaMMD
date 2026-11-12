"""The Physics section of the MayaMMD panel: rigid bodies and joints.

A scene-wide list of rigid bodies and joints, grouped by model, next to an attribute
inspector, in the shape Maya's own Shape Editor uses (items on the left,
details on the right).  The panel renders the
:class:`~mmd.maya.rigid_body_info.PhysicsInfo` it is handed, reports the row
the user picked, and asks its owner to edit attributes, refresh or delete.  That keeps ``cmds``
out of the widget and lets the whole thing be unit-tested offscreen.

The list can be grouped four ways (:class:`Grouping`): PMX order — the index
MMD users know from PMX Editor — or by collision group, physics mode, or bone,
which answer "what does this group collide with / who drives what" questions
the flat list cannot. Bodies and joints have separate tabs with counts.

The inspector is built like the Attribute Editor: a pinned bold node title,
then collapsible sections of typed attribute controls, labels right-aligned,
values left-aligned in fields. Every section shares one label
column width, so all fields start and end on the same vertical edges.  Every
row is one entry in a declarative table (``_BODY_SECTIONS`` /
``_JOINT_SECTIONS``); ``physics_controls`` supplies the typed editors.

``PhysicsPanel`` is embedded by the tool widget through
:class:`~mmd.ui.collapsible_section.CollapsibleSection`; this module owns no
Maya UI.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Sequence
from dataclasses import replace
from enum import Enum

from PySide6 import QtCore, QtGui, QtWidgets

from mmd.maya.rigid_body_info import (
    AxisRange,
    JointInfo,
    PhysicsInfo,
    RangeKind,
    RigidBodyInfo,
    refresh_joint_bodies,
)
from mmd.ui.collapsible_section import SectionHeader
from mmd.ui.physics_controls import (
    BODY_SCALARS,
    BODY_VECTORS,
    JOINT_VECTORS,
    _group_icon,
    attribute_control,
)

log = logging.getLogger(__name__)

MAYA_SELECTION_BLUE = "#5285A6"  # Maya's selection colour (viewport, outliner)
PANEL_BACKGROUND = "#444444"
FIELD_BACKGROUND = "#292929"
LABEL_COLOR = "#C9C9C9"
VALUE_COLOR = "#D8D8D8"
HEADER_COLOR = "#DDDDDD"
WARNING_COLOR = "#D9A441"
SEPARATOR_COLOR = "#454545"

COLUMNS = ("#", "Name", "Detail")
INDEX_COLUMN = 0
"""The ``#`` column: PMX index, width fixed to the widest index."""
DEFAULT_INDEX_WIDTH = 48
DEFAULT_NAME_WIDTH = 170
DEFAULT_DETAIL_WIDTH = 150
MIN_SECTION_PIXELS = 28
CELL_PADDING = 6
"""Horizontal space a cell adds around its text (item + header padding)."""
INITIAL_SPLIT_SIZES = (300, 250)
MIN_TREE_HEIGHT = 140

_NodeRole = int(QtCore.Qt.ItemDataRole.UserRole)
_GuideRole = _NodeRole + 1

_STYLESHEET = f"""
    QTreeWidget {{
        background-color: #363636;
        border: none;
        outline: none;
        show-decoration-selected: 1;
    }}
    QTreeWidget::item {{
        padding: 4px 4px;
        border: none;
    }}
    QTreeWidget::item:hover {{
        background-color: #404040;
    }}
    QTreeWidget::item:selected {{
        background-color: {MAYA_SELECTION_BLUE};
    }}
    QTreeWidget::branch {{
        background: transparent;
        border: none;
    }}
"""


# ── Grouping ───────────────────────────────────────────────────────────────


class Grouping(Enum):
    """How the list orders and groups its rows."""

    PMX_ORDER = "PMX order"
    """PMX index order — the numbering PMX Editor shows."""

    COLLISION_GROUP = "Collision group"
    """By PMX collision group; a joint follows the group of its body A."""

    PHYSICS_MODE = "Physics mode"
    """Follow bone / physics / physics + bone; a joint follows its body A."""

    BONE = "Bone"
    """By the bone that drives the body; a joint follows its body A."""


_NO_BONE = "(no bone)"  # untranslated, like the inspector's em dash
_NO_BODY = "(no body)"

_GroupRow = tuple[int, str, str, str, str]
"""``(index, name, detail, node, guide)`` for one list row."""
_Group = tuple[str, list[_GroupRow]]
_Section = tuple[str, list[_Group]]
"""``(title, [(bucket title, rows)])`` — an empty bucket title means "flat"."""


def grouped_rows(info: PhysicsInfo, grouping: Grouping) -> list[_Section]:
    """The panel's sections/buckets/rows for ``grouping``.

    Pure: no Qt, no scene, so the layout rules are directly testable.
    """
    bodies = [_body_row(body) for body in info.bodies]
    joints = [_joint_row(joint) for joint in info.joints]
    return [
        (
            f"Rigid Bodies ({len(bodies)})",
            _buckets(info, grouping, info.bodies, bodies),
        ),
        (f"Joints ({len(joints)})", _buckets(info, grouping, info.joints, joints)),
    ]


def _buckets(
    info: PhysicsInfo,
    grouping: Grouping,
    rows: Sequence[RigidBodyInfo | JointInfo],
    rendered: Sequence[_GroupRow],
) -> list[_Group]:
    """Group ``rendered`` rows by ``grouping``'s key, in a stable order."""
    if grouping is Grouping.PMX_ORDER:
        return [("", list(rendered))]
    buckets: dict[tuple[object, ...], tuple[str, list[_GroupRow]]] = {}
    for row, rendered_row in zip(rows, rendered, strict=False):
        key, label = _bucket(info, grouping, row)
        buckets.setdefault(key, (label, []))[1].append(rendered_row)
    return [
        (f"{buckets[key][0]} ({len(buckets[key][1])})", buckets[key][1])
        for key in sorted(buckets)
    ]


def _bucket(
    info: PhysicsInfo, grouping: Grouping, row: RigidBodyInfo | JointInfo
) -> tuple[tuple[object, ...], str]:
    """``((sort key), bucket title)`` for one row."""
    if isinstance(row, JointInfo):
        # A joint has no group / mode / bone of its own: it belongs where its
        # body A belongs, which is the end MMD treats as the anchor.
        body = info.body(row.body_a_index)
        if body is None:
            return ((-1,), _NO_BODY)
        return _bucket(info, grouping, body)
    if grouping is Grouping.COLLISION_GROUP:
        return ((row.group,), f"Group {row.group}")
    if grouping is Grouping.PHYSICS_MODE:
        return ((row.physics_mode,), row.physics_mode_label or str(row.physics_mode))
    if grouping is Grouping.BONE:
        name = row.bone or _NO_BONE
        return ((name.lower(),), name)
    return ((), "")


def _body_row(body: RigidBodyInfo) -> _GroupRow:
    detail = f"{body.collider_label} · G{body.group} · {body.physics_mode_label}"
    return (body.index, body.name, detail, body.node, body.guide)


def _joint_row(joint: JointInfo) -> _GroupRow:
    bodies = f"{joint.body_a_index} → {joint.body_b_index}"
    detail = bodies if joint.status.is_active else f"{bodies} · {joint.status.value}"
    return (joint.index, joint.name, detail, joint.node, joint.guide)


# ── Value formatting ───────────────────────────────────────────────────────


def _number(value: float) -> str:
    """A float the way Maya's Channel Box shows it: short, and never ``-0``."""
    # PMX stores float32 values, so raw printing gives noise like 0.0881798.
    text = f"{0.0 if value == 0 else value:.5g}"
    return "0" if text == "-0" else text


def _vector(values: Sequence[float]) -> str:
    """``x, y, z``."""
    return ", ".join(_number(value) for value in values)


def _node_label(path: str) -> str:
    """The last DAG path component — what the Outliner calls the node."""
    return path.rsplit("|", 1)[-1] if path else ""


def _axis(axis: AxisRange) -> str:
    """How Bullet reads one limit axis, with its spring."""
    text = {
        RangeKind.FIXED: "locked",
        RangeKind.LIMITED: "limited",
        RangeKind.FREE: "free",
    }[axis.kind]
    return f"{text} · spring" if axis.spring else text


def _axes(ranges: Sequence[AxisRange]) -> str:
    """``X limited · Y locked · Z free``."""
    return " · ".join(
        f"{name} {_axis(axis)}" for name, axis in zip("XYZ", ranges, strict=False)
    )


def _groups(values: Sequence[int]) -> str:
    return ", ".join(str(value) for value in values) or "none"


def _plural(count: int, singular: str, plural: str) -> str:
    """``1 joint`` / ``2 joints``."""
    return f"{count} {singular if count == 1 else plural}"


# ── Inspector field tables ─────────────────────────────────────────────────

_BodyRow = tuple[str, Callable[[RigidBodyInfo], str]]
_JointRow = tuple[str, Callable[[JointInfo], str]]
_BodySection = tuple[str, tuple[_BodyRow, ...]]
_JointSection = tuple[str, tuple[_JointRow, ...]]

_BODY_SECTIONS: tuple[_BodySection, ...] = (
    (
        "Body",
        (
            ("Local Name", lambda body: body.name_local or "—"),
            ("Universal Name", lambda body: body.name_universal),
            ("Collision Group", lambda body: str(body.group)),
            ("Collider", lambda body: body.collider_label),
            ("Shape Size", lambda body: _vector(body.shape_size)),
            ("Enabled", lambda body: "yes" if body.enabled else "no"),
        ),
    ),
    (
        "Follows",
        (("Bone", lambda body: body.bone or "—"),),
    ),
    (
        "Rest Pose",
        (
            ("Rest Translate", lambda body: _vector(body.rest_translate)),
            ("Rest Rotate", lambda body: _vector(body.rest_rotate)),
        ),
    ),
    (
        "Physics",
        (
            ("Mode", lambda body: body.physics_mode_label),
            ("Mass", lambda body: _number(body.mass)),
            ("Move Attenuation", lambda body: _number(body.linear_damping)),
            ("Rotation Damping", lambda body: _number(body.angular_damping)),
            ("Repulsion", lambda body: _number(body.restitution)),
            ("Friction", lambda body: _number(body.friction)),
        ),
    ),
    (
        "Collision Mask",
        (("Collides With", lambda body: _groups(body.collides_with)),),
    ),
    (
        "Display",
        (
            ("Draw Mode", lambda body: body.draw_mode_label),
            ("Guide", lambda body: _node_label(body.guide) or "—"),
        ),
    ),
)

_JOINT_SECTIONS: tuple[_JointSection, ...] = (
    (
        "Joint",
        (
            ("Local Name", lambda joint: joint.name_local or "—"),
            ("Universal Name", lambda joint: joint.name_universal),
            ("Status", lambda joint: joint.status.value),
        ),
    ),
    (
        "Bodies",
        (
            ("Body A", lambda joint: f"{joint.body_a_index}: {joint.body_a or '—'}"),
            ("Body B", lambda joint: f"{joint.body_b_index}: {joint.body_b or '—'}"),
        ),
    ),
    (
        "Freedom",
        (
            ("Motion", lambda joint: joint.freedom.summary),
            ("Linear", lambda joint: _axes(joint.freedom.linear)),
            ("Angular", lambda joint: _axes(joint.freedom.angular)),
        ),
    ),
    (
        "Limits",
        (
            ("Linear Min", lambda joint: _vector(joint.linear_min)),
            ("Linear Max", lambda joint: _vector(joint.linear_max)),
            (
                "Angular Min",
                lambda joint: f"{_vector(joint.angular_min)} {joint.angle_unit}",
            ),
            (
                "Angular Max",
                lambda joint: f"{_vector(joint.angular_max)} {joint.angle_unit}",
            ),
        ),
    ),
    (
        "Stiffness",
        (
            ("Linear", lambda joint: _vector(joint.linear_stiffness)),
            ("Angular", lambda joint: _vector(joint.angular_stiffness)),
        ),
    ),
    (
        "Display",
        (
            ("Draw Mode", lambda joint: joint.draw_mode_label),
            ("Frame", lambda joint: _node_label(joint.guide) or "—"),
        ),
    ),
)


# ── Inspector ──────────────────────────────────────────────────────────────

LABEL_COLUMN_PADDING = 4
"""Room after the widest label before the value field starts."""


def _label_column_width(font: QtGui.QFont) -> int:
    """The label column width every section shares.

    Widest label of *both* tables, so switching from a body to a joint never
    shifts the value fields' left edge.
    """
    metrics = QtGui.QFontMetrics(font)
    labels = [
        name
        for sections in (_BODY_SECTIONS, _JOINT_SECTIONS)
        for _, rows in sections
        for name, _ in rows
    ]
    return (
        max(metrics.horizontalAdvance(name) for name in labels) + LABEL_COLUMN_PADDING
    )


class _ValueField(QtWidgets.QLabel):
    """A read-only descriptive field, aligned like names in the editors."""

    def __init__(self, text: str, parent: QtWidgets.QWidget) -> None:
        super().__init__(text, parent)
        self.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.setToolTip(text)
        self.setStyleSheet(
            f"color: {VALUE_COLOR};"
            f" background-color: {FIELD_BACKGROUND};"
            " border: none; border-radius: 2px; padding: 2px 6px;"
        )
        self.setMinimumHeight(self.fontMetrics().height() + 8)
        self.setAlignment(
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter
        )
        self.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
        )
        # A long value must shrink, never widen its column: the labels and
        # fields line up across sections only while those columns stay fixed.
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Ignored,
            QtWidgets.QSizePolicy.Policy.Preferred,
        )


class _InspectorSection(QtWidgets.QWidget):
    """One Attribute-Editor-style block: collapsible header + label/value grid."""

    def __init__(
        self,
        title: str,
        rows: Sequence[tuple[str, str]],
        parent: QtWidgets.QWidget,
        label_width: int,
        source: RigidBodyInfo | JointInfo,
        commit: Callable,
    ) -> None:
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Same custom-painted header as the tool panel sections: a Maya-styled
        # QToolButton would render smaller here than the AE's own titles.
        self.header = SectionHeader(title, parent=self)
        layout.addWidget(self.header)

        self.body = QtWidgets.QWidget(self)
        self.body.setObjectName("physicsAttributeSection")
        self.body.setStyleSheet(
            "QWidget#physicsAttributeSection { background-color: #494949; }"
        )
        content = QtWidgets.QGridLayout(self.body)
        content.setContentsMargins(8, 3, 8, 3)
        content.setHorizontalSpacing(6)
        content.setVerticalSpacing(2)
        content.setColumnStretch(1, 1)
        self._values = [value for _, value in rows]
        for row, (name, value) in enumerate(rows):
            key = QtWidgets.QLabel(name, self.body)
            key.setStyleSheet(f"color: {LABEL_COLOR};")
            key.setAlignment(
                QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter
            )
            key.setFixedWidth(label_width)
            key.setObjectName("attributeLabel")
            content.addWidget(key, row, 0)
            control = attribute_control(source, title, name, self.body, commit)
            if control is None:
                control = _ValueField(value, self.body)
            control.setObjectName(name)
            if title == "Collision Mask":
                content.removeWidget(key)
                content.addWidget(key, 0, 0, 1, 2)
                content.addWidget(control, 1, 0, 1, 2)
                key.setToolTip(
                    "Checked groups collide with this body; unchecked groups are ignored."
                )
            else:
                content.addWidget(control, row, 1)
        layout.addWidget(self.body)
        self.header.toggled.connect(self.body.setVisible)

    @property
    def values(self) -> list[str]:
        """The rendered value texts, in row order."""
        return list(self._values)


class PhysicsInspector(QtWidgets.QWidget):
    """Attribute controls under a fixed node title and above a scrolling body."""

    attributeEdited = QtCore.Signal(str, str, object)

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(3)
        self._header = QtWidgets.QLabel(self)
        self._header.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self._header.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed
        )
        font = self._header.font()
        font.setBold(True)
        self._header.setFont(font)
        self._header.setStyleSheet(f"color: {HEADER_COLOR}; padding: 4px 6px;")
        outer.addWidget(self._header)
        self.scroll_area = QtWidgets.QScrollArea(self)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.scroll_area.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        outer.addWidget(self.scroll_area, 1)
        self._body = QtWidgets.QWidget(self)
        self._body.setObjectName("physicsInspectorBody")
        self._body.setStyleSheet(
            f"QWidget#physicsInspectorBody {{ background-color: {PANEL_BACKGROUND}; }}"
        )
        self._layout = QtWidgets.QVBoxLayout(self._body)
        self._layout.setContentsMargins(6, 2, 6, 4)
        self._layout.setSpacing(2)
        self._layout.setSizeConstraint(
            QtWidgets.QLayout.SizeConstraint.SetMinAndMaxSize
        )
        self.scroll_area.setWidget(self._body)
        self._sections: list[_InspectorSection] = []
        self._title = ""
        self._header.clear()
        self._header.hide()
        self.showMessage("")

    # ── Public API ────────────────────────────────────────────────────────

    @property
    def sections(self) -> dict[str, list[str]]:
        """``section title -> value texts`` for what is currently shown."""
        return {section.header.text(): section.values for section in self._sections}

    @property
    def title(self) -> str:
        """The node name shown above the sections (``""`` when empty)."""
        return self._title

    def showMessage(self, message: str) -> None:
        """Show a single greyed-out line (empty state)."""
        self._clear()
        self._title = ""
        self._header.clear()
        self._header.hide()
        if message:
            label = QtWidgets.QLabel(message, self._body)
            label.setWordWrap(True)
            label.setStyleSheet(f"color: {LABEL_COLOR};")
            self._layout.addWidget(label)
        self._layout.addStretch()

    def showBody(self, body: RigidBodyInfo) -> None:
        """Render every PMX field of ``body``."""
        self._render(
            body.node or body.name,
            [
                (title, [(name, getter(body)) for name, getter in rows])
                for title, rows in _BODY_SECTIONS
            ],
            body,
        )

    def showJoint(self, joint: JointInfo) -> None:
        """Render every PMX field of ``joint``, plus its derived freedom."""
        self._render(
            joint.node or joint.name,
            [
                (title, [(name, getter(joint)) for name, getter in rows])
                for title, rows in _JOINT_SECTIONS
            ],
            joint,
        )

    # ── Internals ─────────────────────────────────────────────────────────

    def _clear(self) -> None:
        """Drop every rendered widget.

        Hide old rows immediately so they cannot paint over replacements.
        Keep their parent until deferred deletion: unparenting promotes them
        to top-level windows, which can briefly flash inside Maya.
        """
        while self._layout.count():
            item = self._layout.takeAt(0)
            if item is None:
                continue
            widget = item.widget()
            if widget is not None:
                widget.hide()
                widget.deleteLater()
        self._sections.clear()

    def _render(
        self,
        title: str,
        sections: Sequence[tuple[str, Sequence[tuple[str, str]]]],
        source: RigidBodyInfo | JointInfo,
    ) -> None:
        same_node = title == self._title
        collapsed = (
            {
                section.header.text(): section.header.isChecked()
                for section in self._sections
            }
            if same_node
            else {}
        )
        scroll = self.scroll_area.verticalScrollBar().value() if same_node else 0
        self._clear()
        self._title = title
        self._header.setText(source.name)
        self._header.setToolTip(title)
        self._header.show()
        label_width = _label_column_width(self._body.font())
        for name, rows in sections:
            section = _InspectorSection(
                name,
                rows,
                self._body,
                label_width,
                source,
                lambda attr, value: self.attributeEdited.emit(source.node, attr, value),
            )
            section.header.setChecked(collapsed.get(name, True))
            self._sections.append(section)
            self._layout.addWidget(section)
        self._layout.addStretch()
        self._layout.activate()
        self.scroll_area.verticalScrollBar().setValue(scroll)


# ── Panel ──────────────────────────────────────────────────────────────────


class _PhysicsTree(QtWidgets.QTreeWidget):
    """Draw disclosure triangles explicitly, independent of Maya's stylesheet."""

    def drawBranches(self, painter, rect, index) -> None:
        if not self.model().hasChildren(index):
            return
        center = QtCore.QPoint(
            rect.right() - self.indentation() // 2, rect.center().y()
        )
        if self.isExpanded(index):
            points = [(-4, -2), (4, -2), (0, 3)]
        else:
            points = [(-2, -4), (-2, 4), (3, 0)]
        painter.save()
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor(LABEL_COLOR))
        painter.drawPolygon(
            QtGui.QPolygon([center + QtCore.QPoint(x, y) for x, y in points])
        )
        painter.restore()


class PhysicsPanel(QtWidgets.QWidget):
    """Scene-wide rigid-body / joint list plus selection-driven inspector.

    Signals:
        selectionChanged: full DAG path of the picked row's data node
            (``""`` = nothing selected).
        deleteRequested: full DAG paths of the rows to delete (their guide
            transforms, or their data node when there is no guide).
    """

    selectionChanged = QtCore.Signal(str)
    deleteRequested = QtCore.Signal(list)

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setStyleSheet(_STYLESHEET)

        self._info = PhysicsInfo()
        self._models: tuple[PhysicsInfo, ...] = ()
        self._items: list[QtWidgets.QTreeWidgetItem] = []
        self._groups: list[QtWidgets.QTreeWidgetItem] = []
        self._syncing = False
        self._details_node = ""
        self._fitting = False
        # Once the user drags a separator their layout wins: auto-fitting must
        # never undo a deliberate resize.
        self._user_sized_columns = False
        self._active_tab = 0
        self._tab_states: dict[int, tuple] = {}

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(2)
        self.tabs = QtWidgets.QTabBar(self)
        self.tabs.setExpanding(False)
        self.tabs.addTab("Rigid Bodies (0)")
        self.tabs.addTab("Joints (0)")
        self.tabs.currentChanged.connect(self._switch_tab)
        layout.addWidget(self.tabs)
        layout.addLayout(self._build_controls())

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal, self)
        splitter.setChildrenCollapsible(False)
        splitter.addWidget(self._build_list_column())
        self._inspector = PhysicsInspector(self)
        splitter.addWidget(self._inspector)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes(list(INITIAL_SPLIT_SIZES))
        layout.addWidget(splitter, 1)

        self._footer = QtWidgets.QLabel(self)
        self._footer.setWordWrap(True)
        layout.addWidget(self._footer)

        self.showModel(PhysicsInfo())

    # ── Construction ──────────────────────────────────────────────────────

    def _build_controls(self) -> QtWidgets.QHBoxLayout:
        controls = QtWidgets.QHBoxLayout()
        controls.setContentsMargins(0, 0, 0, 0)
        controls.setSpacing(4)

        self._filter = QtWidgets.QLineEdit(self)
        self._filter.setPlaceholderText("Filter")
        self._filter.setClearButtonEnabled(True)
        self._filter.setToolTip("Filter rigid bodies and joints by any column")
        self._filter.textChanged.connect(self._apply_filter)
        controls.addWidget(self._filter, 1)

        self._grouping = QtWidgets.QComboBox(self)
        self._grouping.setToolTip("How the rigid bodies and joints are grouped")
        for grouping in Grouping:
            self._grouping.addItem(grouping.value, grouping)
        self._grouping.currentIndexChanged.connect(self._rebuild)
        controls.addWidget(self._grouping)
        reveal = QtWidgets.QPushButton("Reveal Selected", self)
        reveal.setToolTip("Expand and scroll to the selected physics row")
        reveal.clicked.connect(self.revealSelected)
        controls.addWidget(reveal)
        return controls

    def _build_list_column(self) -> QtWidgets.QWidget:
        column = QtWidgets.QWidget(self)
        layout = QtWidgets.QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)
        layout.addWidget(self._build_tree(), 1)

        # Delete sits directly under its list: the button acts on the rows
        # above it, not on the filter.
        row = QtWidgets.QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.addStretch(1)
        self._delete_button = QtWidgets.QPushButton("Delete", column)
        self._delete_button.setToolTip(
            "Delete the selected rigid bodies / joints from the scene"
        )
        self._delete_button.clicked.connect(self._request_delete)
        self._delete_button.setEnabled(False)
        row.addWidget(self._delete_button)
        layout.addLayout(row)
        return column

    def _build_tree(self) -> QtWidgets.QTreeWidget:
        self._tree = _PhysicsTree(self)
        self._tree.setColumnCount(len(COLUMNS))
        self._tree.setHeaderLabels(list(COLUMNS))
        self._tree.setRootIsDecorated(True)
        self._tree.setIndentation(14)
        self._tree.setUniformRowHeights(True)
        self._tree.setMinimumHeight(MIN_TREE_HEIGHT)
        self._tree.setAlternatingRowColors(False)
        self._tree.setSelectionMode(
            QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection
        )
        self._tree.setTextElideMode(QtCore.Qt.TextElideMode.ElideRight)
        self._tree.setEditTriggers(
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers
        )

        header = self._tree.header()
        header.setDefaultAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
        self._tree.headerItem().setTextAlignment(
            INDEX_COLUMN,
            QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter,
        )
        # Every column stays draggable: the names used to be Stretch, which
        # made its separator immovable and pushed the Detail column out of view.
        header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
        header.setStretchLastSection(True)
        header.setMinimumSectionSize(MIN_SECTION_PIXELS)
        header.setSectionsMovable(False)
        header.setHighlightSections(False)
        # Widths are applied in showEvent / on rebuild, once the tree has its
        # real width: setting them while it is still 0px wide makes Qt shrink
        # every section to its minimum, which elided the index to "…".
        self._tree.installEventFilter(self)
        header.installEventFilter(self)

        self._tree.currentItemChanged.connect(self._on_current_changed)
        self._tree.itemSelectionChanged.connect(self._update_delete_button)
        shortcut = QtGui.QShortcut(
            QtGui.QKeySequence(QtCore.Qt.Key.Key_Delete), self._tree
        )
        shortcut.setContext(QtCore.Qt.ShortcutContext.WidgetShortcut)
        shortcut.activated.connect(self._request_delete)
        return self._tree

    # ── Public API ────────────────────────────────────────────────────────

    @property
    def info(self) -> PhysicsInfo:
        """The data currently shown."""
        return self._info

    @property
    def tree(self) -> QtWidgets.QTreeWidget:
        """The list widget (for tests and future row styling)."""
        return self._tree

    @property
    def inspector(self) -> PhysicsInspector:
        """The detail widget."""
        return self._inspector

    @property
    def filter_edit(self) -> QtWidgets.QLineEdit:
        """The filter box."""
        return self._filter

    @property
    def grouping_box(self) -> QtWidgets.QComboBox:
        """The grouping selector."""
        return self._grouping

    @property
    def delete_button(self) -> QtWidgets.QPushButton:
        """The Delete button."""
        return self._delete_button

    @property
    def grouping(self) -> Grouping:
        """The selected grouping."""
        return self._grouping.currentData()

    @property
    def selected_node(self) -> str:
        """Full DAG path of the current row's data node, or ``""``."""
        item = self._tree.currentItem()
        return str(item.data(0, _NodeRole) or "") if item is not None else ""

    def selected_paths(self) -> list[str]:
        """Guide (or data node) paths of every selected row, de-duplicated."""
        paths: list[str] = []
        for item in self._tree.selectedItems():
            path = str(item.data(0, _GuideRole) or item.data(0, _NodeRole) or "")
            if path and path not in paths:
                paths.append(path)
        return paths

    def showModel(self, info: PhysicsInfo) -> None:
        """Convenience wrapper for a scene containing one model."""
        self.showScene([info] if info.root else [])

    def showScene(self, models: Sequence[PhysicsInfo]) -> None:
        """Show every model, preserving browsing state across scene refreshes."""
        self._models = tuple(models)
        self._update_snapshot()
        for index, (title, _) in enumerate(grouped_rows(self._info, Grouping.PMX_ORDER)):
            self.tabs.setTabText(index, title)
        self._details_node = ""
        self._rebuild()
        if not self.selected_node:
            self._inspector.showMessage(self._placeholder())
        self._update_footer()

    def _update_snapshot(self) -> None:
        # Flatten only for path lookup/counts. PMX indices are model-local;
        # grouping must always use the original per-model snapshots.
        self._info = PhysicsInfo(
            bodies=tuple(row for model in self._models for row in model.bodies),
            joints=tuple(row for model in self._models for row in model.joints),
            warnings=tuple(
                f"{model.root.rsplit('|', 1)[-1]}: {warning}"
                for model in self._models for warning in model.warnings
            ),
        )

    def syncSelection(self, nodes: Sequence[str]) -> None:
        """Highlight the row for any of ``nodes`` (the Maya selection).

        Unknown nodes clear the list selection instead of leaving a stale row
        highlighted; a multi-node selection highlights every row it recognises.
        """
        for node in nodes:
            row = self._info.row_at(node) or self._row_under(node)
            if row is not None:
                self.tabs.setCurrentIndex(int(isinstance(row, JointInfo)))
                break
        matches = [index for index in map(self._index_of, nodes) if index is not None]
        if not matches:
            self._clear_current()
            return
        self._syncing = True
        try:
            self._tree.clearSelection()
            for index in matches:
                self._items[index].setSelected(True)
            # NoUpdate: plain setCurrentItem() also *selects*, which would
            # collapse a multi-row match back to a single row.
            self._tree.setCurrentItem(
                self._items[matches[0]],
                0,
                QtCore.QItemSelectionModel.SelectionFlag.NoUpdate,
            )
        finally:
            self._syncing = False
        self._show_details(str(self._items[matches[0]].data(0, _NodeRole) or ""))
        self._update_delete_button()

    def applyAttributeEdit(self, node: str, attribute: str, value: object) -> None:
        """Update our snapshot and row after a successful write, retaining editors."""
        source = self._info.row_at(node)
        if source is None:
            return
        body = isinstance(source, RigidBodyInfo)
        prefix = "body" if body else "joint"
        members = {
            prefix + "NameLocal": "name_local",
            prefix + "NameUniversal": "name_universal",
            "drawMode": "draw_mode",
        }
        vectors = BODY_VECTORS if body else JOINT_VECTORS
        members.update({attr: member for member, attr in vectors.values()})
        if body:
            members.update({attr: member for member, attr, _ in BODY_SCALARS.values()})
            members.update(
                bodyEnabled="enabled",
                bodyGroupId="group",
                bodyColliderType="collider_type",
                bodyPhysicsMode="physics_mode",
            )
        if body and attribute.startswith("bodyMaskGroup"):
            mask = list(source.collision_mask)
            mask.extend([True] * (16 - len(mask)))
            mask[int(attribute.removeprefix("bodyMaskGroup"))] = bool(value)
            changes = {"collision_mask": tuple(mask)}
        elif attribute in members:
            changes = {members[attribute]: value}
        else:
            return
        enum_labels = {
            "bodyColliderType": (
                "collider_label",
                {1: "Box", 2: "Sphere", 3: "Capsule"},
            ),
            "bodyPhysicsMode": (
                "physics_mode_label",
                {0: "FollowBone", 1: "Physics", 2: "PhysicsBone"},
            ),
            "drawMode": (
                "draw_mode_label",
                {0: "Off", 1: "Wire", 2: "Solid", 3: "WireSolid"}
                if body
                else {0: "Off", 1: "Hub", 2: "Ranges"},
            ),
        }
        if attribute in enum_labels:
            member, labels = enum_labels[attribute]
            changes[member] = labels.get(value, str(value))
        updated = replace(source, **changes)
        collection = "bodies" if body else "joints"
        self._models = tuple(replace(
            model,
            **{
                collection: tuple(
                    updated if row.node == node else row
                    for row in getattr(model, collection)
                )
            },
        ) for model in self._models)
        if body:
            self._models = tuple(
                refresh_joint_bodies(model)
                if any(row.node == node for row in model.bodies)
                else model
                for model in self._models
            )
        self._update_snapshot()
        index = self._index_of(node)
        if index is not None:
            row = _body_row(updated) if body else _joint_row(updated)
            self._items[index].setText(1, row[1])
            self._items[index].setText(2, row[2])
        if self._details_node == node:
            self._inspector._header.setText(updated.name)
            tables = _BODY_SECTIONS if body else _JOINT_SECTIONS
            for section, (_, rows) in zip(
                self._inspector._sections, tables, strict=True
            ):
                section._values = [getter(updated) for _, getter in rows]
                for (name, _), text in zip(rows, section._values, strict=True):
                    field = section.body.findChild(_ValueField, name)
                    if field is not None:
                        field.setText(text)
                        field.setToolTip(text)
            if body:
                combo = self._inspector.findChild(
                    QtWidgets.QComboBox, "Collision Group"
                )
                for i in range(combo.count()):
                    combo.setItemIcon(i, _group_icon(combo.itemData(i), updated))
        regroup = (
            attribute == "bodyGroupId" and self.grouping is Grouping.COLLISION_GROUP
        ) or (attribute == "bodyPhysicsMode" and self.grouping is Grouping.PHYSICS_MODE)
        if regroup:
            scroll = self._tree.verticalScrollBar().value()
            self._rebuild()
            self._tree.verticalScrollBar().setValue(scroll)
        if self._filter.text():
            self._apply_filter(self._filter.text())

    def revealSelected(self) -> None:
        """Explicitly reveal selection without moving the view on every click."""
        item = self._tree.currentItem()
        if item is None or not item.data(0, _NodeRole):
            return
        self._filter.clear()
        parent = item.parent()
        while parent is not None:
            parent.setExpanded(True)
            parent = parent.parent()
        self._tree.scrollToItem(item)

    # ── Tree building ─────────────────────────────────────────────────────

    def _switch_tab(self, index: int) -> None:
        self._tab_states[self._active_tab] = (
            self.selected_node,
            [str(item.data(0, _NodeRole)) for item in self._tree.selectedItems()],
            self._tree.verticalScrollBar().value(),
            {self._group_key(group): group.isExpanded() for group in self._groups},
        )
        self._active_tab = index
        self._clear_current()
        self._rebuild()
        current, selected, scroll, expanded = self._tab_states.get(
            index, ("", [], 0, {})
        )
        self._select_node(current, emit=False)
        with QtCore.QSignalBlocker(self._tree):
            for item in self._items:
                item.setSelected(
                    str(item.data(0, _NodeRole)) in selected and not item.isHidden()
                )
            for group in self._groups:
                group.setExpanded(expanded.get(self._group_key(group), True))
        self._tree.verticalScrollBar().setValue(scroll)
        self._update_delete_button()

    def _rebuild(self, *_args: object) -> None:
        previous = self.selected_node
        selected = {item.data(0, _NodeRole) for item in self._tree.selectedItems()}
        scroll = self._tree.verticalScrollBar().value()
        expanded = {self._group_key(group): group.isExpanded() for group in self._groups}
        self._syncing = True
        try:
            self._tree.clear()
            self._items.clear()
            self._groups.clear()
            self._tree.setRootIsDecorated(True)
            for model in self._models:
                _, buckets = grouped_rows(model, self.grouping)[self._active_tab]
                if not any(rows for _, rows in buckets):
                    continue
                root = self._add_heading(None, model.root.rsplit('|', 1)[-1], strong=True)
                root.setToolTip(0, model.root)
                self._groups.append(root)
                for bucket_title, rows in buckets:
                    parent = root
                    if bucket_title:
                        parent = self._add_heading(root, bucket_title, strong=False)
                        self._groups.append(parent)
                    self._add_rows(parent, rows)
            for group in self._groups:
                group.setExpanded(expanded.get(self._group_key(group), True))
        finally:
            self._syncing = False
        self._fit_index_column()
        if previous and not self._select_node(previous, emit=False):
            self._inspector.showMessage(self._placeholder())
        self._apply_filter(self._filter.text())
        with QtCore.QSignalBlocker(self._tree):
            for item in self._items:
                item.setSelected(item.data(0, _NodeRole) in selected and not item.isHidden())
        self._tree.verticalScrollBar().setValue(scroll)
        self._update_delete_button()

    @staticmethod
    def _group_key(group: QtWidgets.QTreeWidgetItem) -> tuple[str, ...]:
        parts = []
        while group is not None:
            parts.append(group.toolTip(0) or group.text(0))
            group = group.parent()
        return tuple(reversed(parts))

    def _add_heading(
        self,
        parent: QtWidgets.QTreeWidgetItem | None,
        title: str,
        *,
        strong: bool,
    ) -> QtWidgets.QTreeWidgetItem:
        """A "Rigid Bodies" section (``strong``) or a grouping bucket."""
        item = (
            QtWidgets.QTreeWidgetItem(self._tree)
            if parent is None
            else QtWidgets.QTreeWidgetItem(parent)
        )
        # Span the narrow index column: a heading could never fit in it.
        item.setText(0, title)
        item.setFirstColumnSpanned(True)
        if strong:
            font = item.font(0)
            font.setBold(True)
            item.setFont(0, font)
        else:
            item.setForeground(0, QtGui.QBrush(QtGui.QColor(LABEL_COLOR)))
        return item

    def _add_rows(
        self, parent: QtWidgets.QTreeWidgetItem | None, rows: Sequence[_GroupRow]
    ) -> None:
        for index, name, detail, node, guide in rows:
            item = QtWidgets.QTreeWidgetItem(
                parent if parent is not None else self._tree
            )
            item.setText(INDEX_COLUMN, str(index))
            item.setTextAlignment(
                INDEX_COLUMN,
                QtCore.Qt.AlignmentFlag.AlignRight
                | QtCore.Qt.AlignmentFlag.AlignVCenter,
            )
            item.setText(1, name)
            item.setTextAlignment(
                1,
                QtCore.Qt.AlignmentFlag.AlignLeft
                | QtCore.Qt.AlignmentFlag.AlignVCenter,
            )
            item.setText(2, detail)
            item.setToolTip(1, node or name)
            item.setData(0, _NodeRole, node)
            item.setData(0, _GuideRole, guide)
            self._items.append(item)

    def _fit_index_column(self) -> None:
        """Widen the index column so multi-digit indices are never elided.

        The width comes from Qt (``resizeColumnToContents``) because only Qt
        knows how much space the indent, the branch indicator and the item
        padding take; a hand-computed width elided ``10`` and beyond to ``…``,
        which is what made the numbers look like they stopped at 9.
        """
        self._tree.resizeColumnToContents(INDEX_COLUMN)
        self._set_column_width(
            INDEX_COLUMN,
            max(
                DEFAULT_INDEX_WIDTH, self._tree.columnWidth(INDEX_COLUMN) + CELL_PADDING
            ),
        )

    def _fit_columns(self) -> None:
        """Give the columns sensible widths for the space available.

        Run whenever the panel is resized, so the Detail column is never
        squeezed out; a layout the user dragged by hand wins over this.
        """
        if self._user_sized_columns:
            return
        available = max(self._tree.viewport().width(), 240)
        index_width = max(DEFAULT_INDEX_WIDTH, self._tree.columnWidth(INDEX_COLUMN))
        remaining = max(available - index_width, 2 * MIN_SECTION_PIXELS)
        name_width = max(int(remaining * 0.40), MIN_SECTION_PIXELS)
        self._set_column_width(INDEX_COLUMN, index_width)
        self._set_column_width(1, name_width)
        # The last column stretches, so it only needs a floor.
        self._set_column_width(
            len(COLUMNS) - 1, max(remaining - name_width, DEFAULT_DETAIL_WIDTH)
        )

    def _set_column_width(self, column: int, width: int) -> None:
        """Set one column's width (programmatic, not a user resize)."""
        self._tree.setColumnWidth(column, width)

    def eventFilter(self, watched: QtCore.QObject, event: QtCore.QEvent) -> bool:
        """Keep the columns fitted, and notice when the user takes over.

        Two things need watching on the tree itself:
        * its **resize**, because it sits in a splitter and also changes width
          when the divider moves or the inspector appears — not only when the
          panel does, which left the Name column sized for a different viewport;
        * a **mouse release on the header**, which is what ends a separator
          drag.  Watching the resize signal instead latched on Qt's own
          deferred notifications and froze the layout.
        """
        header = self._tree.header()
        if watched is self._tree and event.type() == QtCore.QEvent.Type.Resize:
            if not self._fitting:
                self._fitting = True
                try:
                    self._fit_columns()
                finally:
                    self._fitting = False
        elif (
            watched is header and event.type() == QtCore.QEvent.Type.MouseButtonRelease
        ):
            self._user_sized_columns = True
        return super().eventFilter(watched, event)

    # ── Events ────────────────────────────────────────────────────────────

    def showEvent(self, event: QtGui.QShowEvent) -> None:
        super().showEvent(event)
        # First real geometry: size the columns now, not at construction.
        self._fit_columns()
        self._fit_index_column()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        # The dock resizes the panel after it is shown; keep the Detail column
        # visible until the user takes over the layout.
        self._fit_columns()
        self._fit_index_column()

    # ── Filtering ─────────────────────────────────────────────────────────

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        for item in self._items:
            match = not needle or any(
                needle in item.text(column).lower() for column in range(len(COLUMNS))
            )
            item.setHidden(not match)
        # Headings last, deepest first: a heading hides when nothing under it
        # survived the filter.
        for group in reversed(self._groups):
            visible = self._visible_count(group)
            group.setHidden(visible == 0)
            if needle and visible:
                group.setExpanded(True)
        current = self._tree.currentItem()
        if needle and current is not None and current.isHidden():
            self._clear_current()

    @classmethod
    def _visible_count(cls, item: QtWidgets.QTreeWidgetItem) -> int:
        """Rows under ``item`` that the filter left visible."""
        if item.childCount() == 0:
            return int(not item.isHidden())
        return sum(
            cls._visible_count(item.child(index)) for index in range(item.childCount())
        )

    def _clear_current(self) -> None:
        """Drop the highlight without emitting a selection change."""
        selection = self._tree.selectionModel()
        if selection is not None:
            self._syncing = True
            try:
                selection.clear()
            finally:
                self._syncing = False
        self._details_node = ""
        self._inspector.showMessage(self._placeholder())
        self._update_delete_button()

    # ── Selection ─────────────────────────────────────────────────────────

    def _index_of(self, node: str) -> int | None:
        """Index into :attr:`_items` of the row standing for ``node``."""
        if not node:
            return None
        for index, item in enumerate(self._items):
            own = str(item.data(0, _NodeRole) or "")
            guide = str(item.data(0, _GuideRole) or "")
            # A selection can land on the data node, on the guide, or on
            # anything under the guide; all of them mean the same row.
            if node in (own, guide) or (guide and node.startswith(f"{guide}|")):
                return index
        return None

    def _select_node(self, node: str, emit: bool = True) -> bool:
        """Make the row for ``node`` current; ``False`` when there is none."""
        index = self._index_of(node)
        if index is None:
            return False
        item = self._items[index]
        if item.isHidden():
            return False
        self._syncing = not emit
        try:
            self._tree.clearSelection()
            item.setSelected(True)
            self._tree.setCurrentItem(item)
        finally:
            self._syncing = False
        if not emit:
            self._show_details(node)
        self._update_delete_button()
        return True

    def _on_current_changed(
        self,
        current: QtWidgets.QTreeWidgetItem | None,
        _previous: QtWidgets.QTreeWidgetItem | None,
    ) -> None:
        if self._syncing:
            return
        node = str(current.data(0, _NodeRole) or "") if current is not None else ""
        if node:
            self._show_details(node)
            self.selectionChanged.emit(node)
        elif current is not None:
            self._details_node = ""
            self._inspector.showMessage(self._placeholder())

    def _show_details(self, node: str) -> None:
        # Maya reports selection changes far more often than rows change, and
        # rebuilding the inspector costs a few dozen widget allocations.
        if node == self._details_node:
            return
        row = self._info.row_at(node) or self._row_under(node)
        self._details_node = node
        if isinstance(row, RigidBodyInfo):
            self._inspector.showBody(row)
        elif isinstance(row, JointInfo):
            self._inspector.showJoint(row)
        else:
            # The scene changed under us (node deleted, refresh pending).
            self._inspector.showMessage("This node is no longer part of the model.")

    def _row_under(self, node: str) -> RigidBodyInfo | JointInfo | None:
        """The row whose data node or guide contains ``node``."""
        for body in self._info.bodies:
            if any(path and node.startswith(f"{path}|") for path in body.paths):
                return body
        for joint in self._info.joints:
            if any(path and node.startswith(f"{path}|") for path in joint.paths):
                return joint
        return None

    def _request_delete(self) -> None:
        paths = self.selected_paths()
        if paths:
            self.deleteRequested.emit(paths)

    def _update_delete_button(self) -> None:
        if hasattr(self, "_delete_button"):
            self._delete_button.setEnabled(bool(self.selected_paths()))

    # ── Footer ────────────────────────────────────────────────────────────

    def _update_footer(self) -> None:
        if self._info.warnings:
            self._footer.show()
            self._footer.setStyleSheet(f"color: {WARNING_COLOR};")
            self._footer.setText(" · ".join(self._info.warnings))
            return
        self._footer.setStyleSheet(f"color: {LABEL_COLOR};")
        self._footer.setText(self._idle_message())
        self._footer.setVisible(self._info.is_empty)

    def _idle_message(self) -> str:
        bodies, joints = self._info.counts
        if not bodies and not joints:
            return "The scene has no rigid bodies or joints."
        return (
            f"{_plural(bodies, 'rigid body', 'rigid bodies')} · "
            f"{_plural(joints, 'joint', 'joints')}"
        )

    def _placeholder(self) -> str:
        """What the inspector says when no row is selected."""
        if self._info.is_empty:
            return self._idle_message()
        return "Select a rigid body or a joint to inspect it."
