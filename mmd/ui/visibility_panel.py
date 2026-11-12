"""The Visibility section of the MayaMMD panel.

A self-contained widget with one toggle per model part (Mesh, Bones, Rigid
Bodies, Joints) plus a master Show All / Hide All button, styled to read like
Maya's own latched toggles.  It owns no scene logic: it emits the requested
state and the caller applies it with :mod:`mmd.maya.model_visibility`, then
calls :meth:`VisibilityPanel.sync` with what the scene actually reports.

The tool panel hosts it inside a
:class:`~mmd.ui.collapsible_section.CollapsibleSection`, like Morphs and
Physics, so all three sections share one header style and scroll together.
"""

from __future__ import annotations

import logging

from PySide6 import QtCore, QtWidgets
from PySide6.QtGui import QColor

from mmd.maya.model_visibility import DEFAULT_VISIBILITY, PART_LABELS, PARTS

log = logging.getLogger(__name__)

MAYA_SELECTION_BLUE = "#5285A6"  # Maya's selection colour (viewport, outliner)

# Off is a flat darker slot (reads as "can be switched on"), on fills it with
# Maya's selection blue, disabled is the native disabled look.  The disabled
# fill keeps the same +7 lightness delta over its background that Maya's own
# disabled buttons have (#4B4B4B on #444444), so it reads the same on the
# #494949 section body (label #808080, as in Maya's disabled buttons).
TOGGLE_OFF_FILL = "#4A4A4A"
TOGGLE_OFF_TEXT = "#C4C4C4"
TOGGLE_ON_FILL = MAYA_SELECTION_BLUE
TOGGLE_ON_TEXT = "#F2F2F2"
TOGGLE_DISABLED_FILL = "#505050"
TOGGLE_DISABLED_TEXT = "#808080"
TOGGLE_SPACING = 4  # px between the toggles
TOGGLE_HEIGHT_PAD = 2  # px the row is shorter than a native button
TOGGLE_MIN_WIDTH = 64

SHOW_ALL_LABEL = "Show All"
HIDE_ALL_LABEL = "Hide All"


def toggle_stylesheet() -> str:
    """Stylesheet for the part toggles (off / on / disabled).

    State is carried by the fill alone - the buttons are borderless.
    ``:checked:disabled`` repeats ``:disabled`` verbatim, so a row with no model
    selected looks identical whatever the stored on/off state is.
    """
    off_fill = QColor(TOGGLE_OFF_FILL)
    on_fill = QColor(TOGGLE_ON_FILL)
    disabled = f"""
            background-color: {TOGGLE_DISABLED_FILL};
            color: {TOGGLE_DISABLED_TEXT};
            border: none;
            border-radius: 2px;
            padding: 1px 6px;"""

    return f"""
        QPushButton {{
            background-color: {off_fill.name()};
            color: {TOGGLE_OFF_TEXT};
            border: none;
            border-radius: 2px;
            padding: 1px 6px;
        }}
        QPushButton:hover {{
            background-color: {off_fill.lighter(112).name()};
        }}
        QPushButton:pressed {{
            background-color: {off_fill.darker(108).name()};
        }}
        QPushButton:checked {{
            background-color: {on_fill.name()};
            color: {TOGGLE_ON_TEXT};
        }}
        QPushButton:checked:hover {{
            background-color: {on_fill.lighter(106).name()};
        }}
        QPushButton:checked:pressed {{
            background-color: {on_fill.darker(108).name()};
        }}
        QPushButton:disabled {{{disabled}
        }}
        QPushButton:checked:disabled {{{disabled}
        }}
    """


class VisibilityPanel(QtWidgets.QWidget):
    """Part toggles + master Show All / Hide All for one model at a time.

    Emits the *requested* state only; the owner applies it to the scene and
    then calls :meth:`sync` so the panel mirrors what really happened.

    Signals:
        partToggled: ``(part, visible)`` for a single part toggle.
        allToggled: ``(visible)`` when the master button is pressed.
    """

    partToggled = QtCore.Signal(str, bool)
    allToggled = QtCore.Signal(bool)

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setStyleSheet(toggle_stylesheet())

        row = QtWidgets.QHBoxLayout(self)
        # Inset so the outer buttons never touch the section border, with tight
        # gaps so the row reads as one control strip.
        row.setContentsMargins(6, 0, 6, 0)
        row.setSpacing(TOGGLE_SPACING)

        self._buttons: dict[str, QtWidgets.QPushButton] = {}
        # The stylesheet replaces Maya's button metrics; sit just under a
        # native button's height so the row stays compact.
        self._toggle_height = 0
        for part in PARTS:
            button = QtWidgets.QPushButton(PART_LABELS[part], self)
            button.setCheckable(True)
            button.setChecked(DEFAULT_VISIBILITY[part])
            button.setToolTip(
                f"Show or hide the {PART_LABELS[part].lower()} of the selected model"
            )
            # Ignored horizontal policy: the hint (label width) would make
            # "Rigid Bodies" wider than the others.  With equal stretch every
            # button gets the same share of the row instead.
            button.setSizePolicy(
                QtWidgets.QSizePolicy.Policy.Ignored,
                QtWidgets.QSizePolicy.Policy.Fixed,
            )
            button.setMinimumWidth(TOGGLE_MIN_WIDTH)
            button.setEnabled(False)  # until a PMX model is targeted
            button.toggled.connect(
                lambda checked, p=part: self.partToggled.emit(p, checked)
            )
            self._buttons[part] = button
            row.addWidget(button, 1)

        self._master_button = QtWidgets.QPushButton(SHOW_ALL_LABEL, self)
        self._master_button.setToolTip("Show or hide every part of the selected model")
        self._master_button.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Ignored,
            QtWidgets.QSizePolicy.Policy.Fixed,
        )
        self._master_button.setMinimumWidth(TOGGLE_MIN_WIDTH)
        self._master_button.setEnabled(False)
        self._master_button.clicked.connect(self._on_master_clicked)
        row.addWidget(self._master_button, 1)

        self._all_visible = False

    # ── Public API ────────────────────────────────────────────────────────

    @property
    def buttons(self) -> dict[str, QtWidgets.QPushButton]:
        """Part key -> its toggle button."""
        return self._buttons

    @property
    def master_button(self) -> QtWidgets.QPushButton:
        """The Show All / Hide All button."""
        return self._master_button

    @property
    def allVisible(self) -> bool:
        """``True`` when the last sync saw every available part shown."""
        return self._all_visible

    def match_row_height(self, native_height: int) -> None:
        """Size the row just under a native button's height (scales with DPI)."""
        self._toggle_height = max(native_height - TOGGLE_HEIGHT_PAD, 16)
        for button in (*self._buttons.values(), self._master_button):
            button.setFixedHeight(self._toggle_height)

    def sync(
        self,
        states: dict[str, bool | None],
        hidden_counts: dict[str, int] | None = None,
    ) -> None:
        """Mirror the scene: check, enable, and relabel the master button.

        Args:
            states: ``part -> visible`` from
                :func:`mmd.maya.model_visibility.get_part_visibility`.  A part
                the model does not have maps to ``None`` and is disabled.
            hidden_counts: ``part -> nodes hidden individually`` from
                :func:`mmd.maya.model_visibility.hidden_descendant_counts`.
                Those stay hidden when a part is shown, so the tooltips say so
                instead of implying the toggle reveals everything.
        """
        hidden_counts = hidden_counts or {}

        for part, button in self._buttons.items():
            visible = states.get(part)
            button.setEnabled(visible is not None)
            button.setToolTip(self._part_tooltip(part, visible, hidden_counts))
            if visible is None:
                continue
            button.blockSignals(True)
            button.setChecked(visible)
            button.blockSignals(False)

        available = [state for state in states.values() if state is not None]
        hidden_total = sum(hidden_counts.values())
        self._all_visible = bool(available) and all(available) and hidden_total == 0
        self._master_button.setEnabled(bool(available))
        self._master_button.setText(
            HIDE_ALL_LABEL if self._all_visible else SHOW_ALL_LABEL
        )
        self._master_button.setToolTip(self._master_tooltip(hidden_total))

    @staticmethod
    def _part_tooltip(
        part: str, visible: bool | None, hidden_counts: dict[str, int]
    ) -> str:
        label = PART_LABELS[part].lower()
        tooltip = f"Show or hide the {label} of the selected model"
        hidden = hidden_counts.get(part, 0)
        if visible and hidden:
            suffix = "node" if hidden == 1 else "nodes"
            tooltip += (
                f"\n{hidden} {suffix} hidden individually stay hidden "
                f"(Show All reveals them)"
            )
        return tooltip

    @staticmethod
    def _master_tooltip(hidden_total: int) -> str:
        if not hidden_total:
            return "Show or hide every part of the selected model"
        suffix = "node" if hidden_total == 1 else "nodes"
        return (
            "Reveal every part of the selected model, including "
            f"{hidden_total} {suffix} hidden individually"
        )

    # â”€â”€ Internals â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    def _on_master_clicked(self) -> None:
        self.allToggled.emit(not self._all_visible)
