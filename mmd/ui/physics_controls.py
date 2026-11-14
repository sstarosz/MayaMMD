"""Typed physics attribute controls; scene writes belong to the tool owner."""

from __future__ import annotations

from collections.abc import Callable
from typing import cast

from PySide6 import QtCore, QtGui, QtWidgets

from mmd.maya.rigid_body_info import JointInfo, JointStatus, RigidBodyInfo

# Keep in sync with kGroupColors in rigid_body_shape_geometry_override.cpp.
GROUP_COLORS = (
    (1.00, 0.25, 0.25),
    (1.00, 0.55, 0.15),
    (1.00, 0.85, 0.15),
    (0.45, 1.00, 0.25),
    (0.70, 0.25, 0.70),
    (0.20, 0.90, 1.00),
    (0.30, 0.45, 1.00),
    (0.65, 0.30, 1.00),
    (1.00, 0.30, 0.85),
    (0.90, 0.20, 0.50),
    (0.60, 0.40, 0.25),
    (0.55, 0.55, 0.55),
    (0.30, 0.70, 0.50),
    (0.90, 0.75, 0.40),
    (0.40, 0.60, 0.80),
    (0.80, 0.50, 0.60),
)

# Enum labels, kept in the order the Maya nodes declare them.  Kept here (and
# not read from the scene) so the panel's render path needs no ``cmds``.
GROUP_LABELS = {group: f"Group {group}" for group in range(16)}
COLLIDER_LABELS = {1: "Box", 2: "Sphere", 3: "Capsule"}
PHYSICS_MODE_LABELS = {0: "FollowBone", 1: "Physics", 2: "PhysicsBone"}
BODY_DRAW_MODE_LABELS = {0: "Off", 1: "Wire", 2: "Solid", 3: "WireSolid"}
JOINT_DRAW_MODE_LABELS = {0: "Off", 1: "Hub", 2: "Ranges"}

ANGLE_TRIPLES = ("jointAngularMin", "jointAngularMax")
"""Joint limit attributes in the scene's angle unit, shown with that unit."""

BODY_SCALARS = {
    "Mass": ("mass", "bodyMass", 10.0),
    "Move Attenuation": ("linear_damping", "bodyLinearDamping", 1.0),
    "Rotation Damping": ("angular_damping", "bodyAngularDamping", 1.0),
    "Repulsion": ("restitution", "bodyRestitution", 1.0),
    "Friction": ("friction", "bodyFriction", 1.0),
}
BODY_VECTORS = {
    "Shape Size": ("shape_size", "bodyShapeSize"),
    "Rest Translate": ("rest_translate", "bodyRestTranslate"),
    "Rest Rotate": ("rest_rotate", "bodyRestRotate"),
}
JOINT_VECTORS = {
    "Linear Min": ("linear_min", "jointLinearMin"),
    "Linear Max": ("linear_max", "jointLinearMax"),
    "Angular Min": ("angular_min", "jointAngularMin"),
    "Angular Max": ("angular_max", "jointAngularMax"),
    "Linear": ("linear_stiffness", "jointLinearStiffness"),
    "Angular": ("angular_stiffness", "jointAngularStiffness"),
}


def group_color(group: int, body: RigidBodyInfo | None = None) -> QtGui.QColor:
    """Viewport base colour, including disabled/kinematic adjustments."""
    rgb = GROUP_COLORS[group if 0 <= group < 16 else 0]
    if body is not None:
        if not body.enabled:
            rgb = (0.45, 0.45, 0.45)
        elif body.is_kinematic:
            rgb = (rgb[0] * 0.6, rgb[1] * 0.6, rgb[2] * 0.6)
    return QtGui.QColor.fromRgbF(*rgb)


def _group_icon(group: int, body: RigidBodyInfo | None = None) -> QtGui.QIcon:
    pixmap = QtGui.QPixmap(14, 14)
    pixmap.fill(group_color(group, body))
    return QtGui.QIcon(pixmap)


class PrecisionSpinBox(QtWidgets.QDoubleSpinBox):
    """Three display decimals, with round-trip precision while editing."""

    def textFromValue(self, value: float) -> str:
        if self.hasFocus():
            # repr is the shortest decimal that round-trips to this double.
            return repr(value).replace(".", self.locale().decimalPoint())
        return self.locale().toString(0.0 if abs(value) < 0.0005 else value, "f", 3)

    def valueFromText(self, text: str) -> float:
        # Qt may interpret the rounded display when focus/steps change. That
        # display must never replace the underlying, higher-precision value.
        if text == self.textFromValue(self.value()):
            return self.value()
        value, _ok = cast("tuple[float, bool]", self.locale().toDouble(text))
        return value

    def validate(self, text: str, position: int):
        validator = QtGui.QDoubleValidator(self.minimum(), self.maximum(), 323)
        validator.setLocale(self.locale())
        validator.setNotation(QtGui.QDoubleValidator.Notation.ScientificNotation)
        return cast(
            "tuple[QtGui.QValidator.State, str, int]",
            validator.validate(text, position),
        )

    def focusInEvent(self, event: QtGui.QFocusEvent) -> None:
        self.lineEdit().setText(
            repr(self.value()).replace(".", self.locale().decimalPoint())
        )
        super().focusInEvent(event)

    def focusOutEvent(self, event: QtGui.QFocusEvent) -> None:
        # Interpret edited text before replacing it with the rounded display.
        super().focusOutEvent(event)
        self.lineEdit().setText(self.textFromValue(self.value()))


def _number(value: float, parent: QtWidgets.QWidget) -> QtWidgets.QDoubleSpinBox:
    field = PrecisionSpinBox(parent)
    field.setRange(-1e100, 1e100)
    # Keep the full double range, including tiny values. Display precision is
    # independent: reducing setDecimals would round the stored value itself.
    field.setDecimals(323)
    field.setSingleStep(0.01)
    field.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
    field.setKeyboardTracking(False)
    field.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
    field.setValue(value)
    field.setMinimumWidth(0)
    field.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed
    )
    return field


class ScalarControl(QtWidgets.QWidget):
    """Numeric entry and slider; a drag commits once, on release."""

    def __init__(
        self,
        value: float,
        maximum: float,
        commit: Callable,
        parent: QtWidgets.QWidget,
        *,
        minimum: float | None = None,
    ) -> None:
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.field = _number(value, self)
        if minimum is not None:
            # Keep legacy values visible without changing the scene on display.
            self.field.setMinimum(min(minimum, value))
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal, self)
        self.slider.setRange(0, 1000)
        self._low = min(0.0, value) if minimum is None else minimum
        self._high = max(maximum, value) if minimum is None else maximum
        self._sync_slider()
        layout.addWidget(self.field, 1)
        layout.addWidget(self.slider, 2)
        last_committed = self.field.value()

        def changed(value: float) -> None:
            nonlocal last_committed
            if minimum is not None and value < minimum:
                self.field.setValue(last_committed)
                self._sync_slider()
                return
            if value != last_committed:
                last_committed = value
                if minimum is not None:
                    self.field.setMinimum(minimum)
                commit(value)

        def entered() -> None:
            if minimum is None:
                self._low = min(self._low, self.field.value())
                self._high = max(self._high, self.field.value())
            self._sync_slider()
            changed(self.field.value())

        def slid(position: int) -> None:
            self.field.setValue(self._low + position / 1000 * (self._high - self._low))
            if not self.slider.isSliderDown():
                changed(self.field.value())

        self.field.editingFinished.connect(entered)
        self.slider.valueChanged.connect(slid)
        self.slider.sliderReleased.connect(lambda: changed(self.field.value()))

    def _sync_slider(self) -> None:
        with QtCore.QSignalBlocker(self.slider):
            self.slider.setValue(
                round(
                    (self.field.value() - self._low) / (self._high - self._low) * 1000
                )
            )


class CollisionMaskControl(QtWidgets.QWidget):
    """Checked means this body collides with the numbered group."""

    def __init__(
        self, body: RigidBodyInfo, commit: Callable, parent: QtWidgets.QWidget
    ) -> None:
        super().__init__(parent)
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.checkboxes = []
        for group in range(16):
            check = QtWidgets.QCheckBox(str(group), self)
            check.setIcon(_group_icon(group))
            check.setChecked(
                group < len(body.collision_mask) and body.collision_mask[group]
            )
            check.setToolTip(f"Collide with group {group} when checked")
            check.toggled.connect(
                lambda checked, group=group: commit(f"bodyMaskGroup{group}", checked)
            )
            layout.addWidget(check, group // 4, group % 4)
            self.checkboxes.append(check)


def attribute_control(
    source: RigidBodyInfo | JointInfo,
    section: str,
    name: str,
    parent: QtWidgets.QWidget,
    commit: Callable,
) -> QtWidgets.QWidget | None:
    """Build an editor for stored attributes; derived information has no editor."""
    if name in ("Local Name", "Universal Name"):
        return _name_control(source, name, parent, commit)
    if isinstance(source, RigidBodyInfo):
        return _body_control(source, section, name, parent, commit)
    return _joint_control(source, section, name, parent, commit)


def _name_control(
    source: RigidBodyInfo | JointInfo,
    name: str,
    parent: QtWidgets.QWidget,
    commit: Callable,
) -> QtWidgets.QWidget:
    """An editable name field; the plug name depends on the node kind."""
    prefix = "body" if isinstance(source, RigidBodyInfo) else "joint"
    attr = prefix + ("NameLocal" if name == "Local Name" else "NameUniversal")
    text = source.name_local if name == "Local Name" else source.name_universal
    field = QtWidgets.QLineEdit(text, parent)
    field.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
    field.editingFinished.connect(lambda: commit(attr, field.text()))
    return field


def _body_control(
    body: RigidBodyInfo,
    section: str,
    name: str,
    parent: QtWidgets.QWidget,
    commit: Callable,
) -> QtWidgets.QWidget | None:
    if name == "Mode":
        return _readonly_enum(
            body.physics_mode,
            PHYSICS_MODE_LABELS,
            parent,
            "Physics mode determines bone and solver connections at import. "
            "Read-only: change the mode in the PMX file and re-import the model.",
        )
    if name == "Enabled":
        check = QtWidgets.QCheckBox(parent)
        check.setChecked(body.enabled)
        check.toggled.connect(lambda checked: commit("bodyEnabled", checked))
        return check
    if section == "Collision Mask":
        return CollisionMaskControl(body, commit, parent)
    if name in BODY_SCALARS:
        member, attr, maximum = BODY_SCALARS[name]
        control = ScalarControl(
            getattr(body, member),
            maximum,
            lambda value: commit(attr, value),
            parent,
            minimum=0.0 if attr == "bodyRestitution" else None,
        )
        if attr == "bodyRestitution":
            control.setToolTip(
                "Bounciness: 0 means no bounce; 1 means full restitution. "
                "The slider covers 0–1; higher values can be entered numerically. "
                "Negative edits are not allowed; existing values are shown unchanged."
            )
        return control
    if name in BODY_VECTORS:
        return _vector_control(body, BODY_VECTORS[name], name, parent, commit)
    enums = {
        "Collision Group": ("bodyGroupId", body.group, GROUP_LABELS),
        "Collider": ("bodyColliderType", body.collider_type, COLLIDER_LABELS),
        "Draw Mode": ("drawMode", body.draw_mode, BODY_DRAW_MODE_LABELS),
    }
    if name in enums:
        attr, value, choices = enums[name]
        return _enum_control(value, attr, choices, parent, commit, body)
    return None


def _joint_control(
    joint: JointInfo,
    section: str,
    name: str,
    parent: QtWidgets.QWidget,
    commit: Callable,
) -> QtWidgets.QWidget | None:
    if name == "Status":
        return _readonly_enum(
            joint.status.value,
            {status.value: status.value for status in JointStatus},
            parent,
            "Read-only: status is derived from the connected bodies, "
            "their enabled states, and their physics modes.",
        )
    # "Freedom" describes the limits the joint resolves to; "Limits" and
    # "Stiffness" hold the stored triples.  Only the latter are editable.
    if section != "Freedom" and name in JOINT_VECTORS:
        return _vector_control(joint, JOINT_VECTORS[name], name, parent, commit)
    if name == "Draw Mode":
        return _enum_control(
            joint.draw_mode, "drawMode", JOINT_DRAW_MODE_LABELS, parent, commit
        )
    return None


def _vector_control(
    source: RigidBodyInfo | JointInfo,
    mapping: tuple[str, str],
    name: str,
    parent: QtWidgets.QWidget,
    commit: Callable,
) -> QtWidgets.QWidget:
    """Three numeric fields that commit together when editing finishes."""
    member, attr = mapping
    widget = QtWidgets.QWidget(parent)
    layout = QtWidgets.QHBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(4)
    fields = [_number(value, widget) for value in getattr(source, member)]
    last_committed = tuple(field.value() for field in fields)

    def entered() -> None:
        nonlocal last_committed
        values = tuple(field.value() for field in fields)
        if values != last_committed:
            last_committed = values
            commit(attr, values)

    for axis, field in zip("XYZ", fields, strict=True):
        field.setToolTip(f"{name} {axis}")
        layout.addWidget(field)
        field.editingFinished.connect(entered)
    if attr in ANGLE_TRIPLES:
        unit = QtWidgets.QLabel(getattr(source, "angle_unit", ""), widget)
        unit.setToolTip(f"{name} in the scene's angle unit")
        layout.addWidget(unit)
    return widget


def _readonly_enum(
    value: int | str,
    choices: dict,
    parent: QtWidgets.QWidget,
    explanation: str,
) -> QtWidgets.QComboBox:
    combo = QtWidgets.QComboBox(parent)
    for key, label in choices.items():
        combo.addItem(label, key)
    if value not in choices:
        combo.addItem(f"Unknown ({value})", value)
    combo.setCurrentIndex(combo.findData(value))
    combo.setEnabled(False)
    combo.setProperty("readOnlyEnum", True)
    combo.setMinimumWidth(0)
    combo.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed
    )
    combo.setToolTip(explanation + "\nStates: " + ", ".join(choices.values()))
    return combo


def _enum_control(
    value: int,
    attr: str,
    choices: dict[int, str],
    parent: QtWidgets.QWidget,
    commit: Callable,
    body: RigidBodyInfo | None = None,
) -> QtWidgets.QWidget:
    combo = QtWidgets.QComboBox(parent)
    combo.setMinimumWidth(0)
    combo.setSizePolicy(
        QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed
    )
    for index, label in choices.items():
        if body is not None and attr == "bodyGroupId":
            combo.addItem(_group_icon(index, body), label, index)
        else:
            combo.addItem(label, index)
    if value not in choices:
        combo.addItem(f"Unknown ({value})", value)
    combo.setCurrentIndex(combo.findData(value))
    combo.activated.connect(lambda _index: commit(attr, combo.currentData()))
    return combo
