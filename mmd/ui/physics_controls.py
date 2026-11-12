"""Typed physics attribute controls; scene writes belong to the tool owner."""

from __future__ import annotations

from collections.abc import Callable

from PySide6 import QtCore, QtGui, QtWidgets

from mmd.maya.rigid_body_info import JointInfo, RigidBodyInfo

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
            rgb = tuple(channel * 0.6 for channel in rgb)
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
        return self.locale().toDouble(text)[0]

    def validate(self, text: str, position: int):
        validator = QtGui.QDoubleValidator(self.minimum(), self.maximum(), 323)
        validator.setLocale(self.locale())
        validator.setNotation(QtGui.QDoubleValidator.Notation.ScientificNotation)
        state, _, _ = validator.validate(text, position)
        return state, text, position

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
        self, value: float, maximum: float, commit: Callable, parent: QtWidgets.QWidget
    ) -> None:
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.field = _number(value, self)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal, self)
        self.slider.setRange(0, 1000)
        self._low = min(0.0, value)
        self._high = max(maximum, value)
        self._sync_slider()
        layout.addWidget(self.field, 1)
        layout.addWidget(self.slider, 2)
        last_committed = self.field.value()

        def changed(value: float) -> None:
            nonlocal last_committed
            if value != last_committed:
                last_committed = value
                commit(value)

        def entered() -> None:
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
    body = isinstance(source, RigidBodyInfo)
    prefix = "body" if body else "joint"
    if body and name == "Mode":
        field = QtWidgets.QLineEdit(source.physics_mode_label, parent)
        field.setReadOnly(True)
        field.setToolTip(
            "Physics mode determines bone and solver connections at import. "
            "Change the mode in the PMX file and re-import the model."
        )
        return field
    if name in ("Local Name", "Universal Name"):
        attr = prefix + ("NameLocal" if name == "Local Name" else "NameUniversal")
        text = source.name_local if name == "Local Name" else source.name_universal
        field = QtWidgets.QLineEdit(text, parent)
        field.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
        field.editingFinished.connect(lambda: commit(attr, field.text()))
        return field
    if body and name == "Enabled":
        check = QtWidgets.QCheckBox(parent)
        check.setChecked(source.enabled)
        check.toggled.connect(lambda checked: commit("bodyEnabled", checked))
        return check
    if body and section == "Collision Mask":
        return CollisionMaskControl(source, commit, parent)
    if body and name in BODY_SCALARS:
        member, attr, maximum = BODY_SCALARS[name]
        return ScalarControl(
            getattr(source, member), maximum, lambda value: commit(attr, value), parent
        )
    vectors = BODY_VECTORS if body else JOINT_VECTORS
    if name in vectors and section != "Freedom":
        member, attr = vectors[name]
        widget = QtWidgets.QWidget(parent)
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        fields = [_number(value, widget) for value in getattr(source, member)]
        last_committed = tuple(field.value() for field in fields)

        def entered_vector() -> None:
            nonlocal last_committed
            values = tuple(field.value() for field in fields)
            if values != last_committed:
                last_committed = values
                commit(attr, values)

        for axis, field in zip("XYZ", fields, strict=True):
            field.setToolTip(f"{name} {axis}")
            layout.addWidget(field)
            field.editingFinished.connect(entered_vector)
        return widget
    enums = {}
    if body:
        enums = {
            "Collision Group": (
                "bodyGroupId",
                source.group,
                {i: f"Group {i}" for i in range(16)},
            ),
            "Collider": (
                "bodyColliderType",
                source.collider_type,
                {1: "Box", 2: "Sphere", 3: "Capsule"},
            ),
        }
    enums["Draw Mode"] = (
        "drawMode",
        source.draw_mode,
        {0: "Off", 1: "Wire", 2: "Solid", 3: "WireSolid"}
        if body
        else {0: "Off", 1: "Hub", 2: "Ranges"},
    )
    if name in enums:
        attr, value, choices = enums[name]
        combo = QtWidgets.QComboBox(parent)
        combo.setMinimumWidth(0)
        combo.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed
        )
        for index, label in choices.items():
            if name == "Collision Group":
                combo.addItem(_group_icon(index, source), label, index)
            else:
                combo.addItem(label, index)
        if value not in choices:
            combo.addItem(f"Unknown ({value})", value)
        combo.setCurrentIndex(combo.findData(value))
        combo.activated.connect(lambda _index: commit(attr, combo.currentData()))
        return combo
    return None
