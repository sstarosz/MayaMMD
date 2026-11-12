"""A collapsible section that looks like Maya's own ``frameLayout``.

Maya's ``frameLayout`` is the native container, but it cannot live inside a Qt
scroll area: its geometry belongs to Maya's layout engine, so a ``QScrollArea``
cannot size it and the whole panel would not scroll.

Re-styling a ``QToolButton`` was not enough either: Maya installs its own
application-wide stylesheet, and its rules for ``QToolButton``/``QPushButton``
(used all over Maya's UI for small icon buttons) shrank the section headers and
dropped their background, so the Qt sections looked nothing like the frames
they replaced.

The header is therefore **custom-painted**: no stylesheet can reach it, so the
bar keeps Maya's look — flat grey fill, a bold title, a triangle at the
left and the panel's own font — at any DPI.  The section body is painted the
same way, giving the section the Attribute Editor's lighter body background.
"""

from __future__ import annotations

from PySide6 import QtCore, QtGui, QtWidgets

BACKGROUND = QtGui.QColor("#494949")
HEADER_BACKGROUND = QtGui.QColor("#5D5D5D")
HEADER_HOVER_BACKGROUND = QtGui.QColor("#656565")
HEADER_TEXT = QtGui.QColor("#C9C9C9")
TRIANGLE = QtGui.QColor("#C9C9C9")

HEADER_PADDING_Y = 7
"""Extra pixels above/below the font: Maya's frame headers are ~22px at 100%."""
TRIANGLE_SIZE = 12
TEXT_LEFT = 8
SECTION_BORDER = 2
"""The 1px frame border each side, added to a collapsed section's cap."""
NO_MAXIMUM = 16777215
"""Qt's "no maximum", restored when a section is expanded."""


class SectionHeader(QtWidgets.QWidget):
    """The clickable bar: triangle + title, in Maya frameLayout styling.

    Behaves enough like a checkable button (``text`` / ``isChecked`` / ``click``)
    that callers and tests read naturally, without inheriting from a widget
    Maya's stylesheet restyles.
    """

    toggled = QtCore.Signal(bool)
    """``(expanded)`` whenever the header is clicked or :meth:`setChecked`."""

    def __init__(
        self,
        title: str,
        expanded: bool = True,
        flat: bool = False,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._title = title
        self._expanded = expanded
        self._flat = flat
        self._hover = False
        title_font = self.font()
        title_font.setBold(True)
        self.setFont(title_font)
        self.setFixedHeight(self.fontMetrics().height() + HEADER_PADDING_Y)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Fixed,
        )
        self.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        self.setToolTip(f"Show or hide {title}")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_Hover, True)

    # ── Button-like API ───────────────────────────────────────────────────

    def text(self) -> str:
        """The header label."""
        return self._title

    def setText(self, title: str) -> None:
        """Rename the section."""
        self._title = title
        self.setToolTip(f"Show or hide {title}")
        self.update()

    def isChecked(self) -> bool:
        """``True`` when the section is expanded."""
        return self._expanded

    def setChecked(self, expanded: bool) -> None:
        """Expand or collapse; emits :attr:`toggled` when the state changes."""
        if expanded == self._expanded:
            return
        self._expanded = expanded
        self.update()
        self.toggled.emit(expanded)

    def toggle(self) -> None:
        """Flip the state."""
        self.setChecked(not self._expanded)

    def click(self) -> None:
        """What a left click does, for callers that simulate one."""
        self.toggle()

    # ── Painting ──────────────────────────────────────────────────────────

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        rect = self.rect()

        if not self._flat:
            painter.fillRect(
                rect, HEADER_HOVER_BACKGROUND if self._hover else HEADER_BACKGROUND
            )

        centre_x = rect.left() + TEXT_LEFT + TRIANGLE_SIZE // 2
        middle_y = rect.center().y() + 1
        half = TRIANGLE_SIZE // 2
        depth = TRIANGLE_SIZE // 3
        if self._expanded:
            triangle = [
                QtCore.QPoint(centre_x - half, middle_y - depth),
                QtCore.QPoint(centre_x + half, middle_y - depth),
                QtCore.QPoint(centre_x, middle_y + depth),
            ]
        else:
            triangle = [
                QtCore.QPoint(centre_x - depth, middle_y - half),
                QtCore.QPoint(centre_x - depth, middle_y + half),
                QtCore.QPoint(centre_x + depth, middle_y),
            ]
        painter.setBrush(TRIANGLE)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawPolygon(triangle)

        painter.setPen(HEADER_TEXT)
        text_rect = rect.adjusted(TEXT_LEFT + TRIANGLE_SIZE + 12, 0, -4, 0)
        painter.drawText(
            text_rect,
            int(QtCore.Qt.AlignmentFlag.AlignVCenter)
            | int(QtCore.Qt.AlignmentFlag.AlignLeft),
            self._title,
        )

    # ── Hover / click ─────────────────────────────────────────────────────

    def enterEvent(self, event: QtGui.QEnterEvent) -> None:
        self._hover = True
        self.update()
        super().enterEvent(event)

    def leaveEvent(self, event: QtCore.QEvent) -> None:
        self._hover = False
        self.update()
        super().leaveEvent(event)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.toggle()
        super().mousePressEvent(event)



class CollapsibleSection(QtWidgets.QWidget):
    """A titled section that can be collapsed, holding one content widget.

    Used where Maya cannot draw a ``frameLayout`` (tests, headless tools);
    :class:`mmd.ui.maya_section.MayaSection` is the native equivalent and
    behaves the same way, including the height clamp that lets a splitter give
    a collapsed section's space to its neighbour.

    Signals:
        toggled: ``(expanded)`` after the user (or :meth:`setExpanded`) changes
            the state; the owner uses it to refresh lazily-built content.
    """

    toggled = QtCore.Signal(bool)

    def __init__(
        self,
        title: str,
        content: QtWidgets.QWidget | None = None,
        expanded: bool = True,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._content: QtWidgets.QWidget | None = None
        self._content_height = 0
        self._expanded_minimum = 0

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._header = SectionHeader(title, expanded, parent=self)
        self._header.toggled.connect(self._on_header_toggled)
        layout.addWidget(self._header)

        self._body = QtWidgets.QWidget(self)
        self._body_layout = QtWidgets.QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(4, 3, 4, 4)
        self._body_layout.setSpacing(0)
        layout.addWidget(self._body)

        if content is not None:
            self.setContent(content)
        self.setExpanded(expanded, emit=False)

    # ── Public API ────────────────────────────────────────────────────────

    @property
    def title(self) -> str:
        """The header label."""
        return self._header.text()

    @property
    def header(self) -> SectionHeader:
        """The clickable header (for tests and styling)."""
        return self._header

    @property
    def body(self) -> QtWidgets.QWidget:
        """The container the content lives in."""
        return self._body

    @property
    def content(self) -> QtWidgets.QWidget | None:
        """The widget currently shown, if any."""
        return self._content

    def setContent(self, content: QtWidgets.QWidget) -> None:
        """Show ``content``, replacing (and releasing) any previous widget."""
        if self._content is not None:
            self._body_layout.removeWidget(self._content)
            self._content.setParent(None)
            self._content.deleteLater()
        self._content = content
        # Stretch 1: a section taller than its content (a fixed content height)
        # must fill from the top and leave the slack below — without it Qt
        # centres the content in the section.
        self._body_layout.addWidget(content, 1)
        self._apply_content_height()

    def setContentHeight(self, height: int) -> None:
        """Give the content a fixed height (what a scroll area needs).

        Sections that host a scrolling list cannot size themselves from their
        contents, so they declare a height instead; ``0`` restores automatic
        sizing.
        """
        self._content_height = height
        self._apply_content_height()

    @property
    def contentHeight(self) -> int:
        """The height set by :meth:`setContentHeight` (``0`` = automatic)."""
        return self._content_height

    def isExpanded(self) -> bool:
        """``True`` when the content is visible."""
        return self._header.isChecked()

    @property
    def isCollapsed(self) -> bool:
        """``True`` while the section is clamped to its header."""
        return not self._header.isChecked()

    def setExpanded(self, expanded: bool, emit: bool = True) -> None:
        """Show or hide the content; ``emit=False`` keeps the signal quiet."""
        self._set_expanded(expanded, emit=emit)

    # ── Painting ──────────────────────────────────────────────────────────

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:
        """The flat body background, like Maya's Attribute Editor frames."""
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), BACKGROUND)

    # ── Internals ─────────────────────────────────────────────────────────

    def _on_header_toggled(self, expanded: bool) -> None:
        self._set_expanded(expanded, emit=True)

    def _set_expanded(self, expanded: bool, emit: bool) -> None:
        was_expanded = self.isExpanded()
        self._header.blockSignals(True)
        self._header.setChecked(expanded)
        self._header.blockSignals(False)
        self._body.setVisible(expanded)
        self._apply_content_height()
        if expanded:
            self.setMinimumHeight(self._expanded_minimum)
            self.setMaximumHeight(NO_MAXIMUM)
        else:
            if was_expanded:
                self._expanded_minimum = self.minimumHeight()
            # Clamp to the header while collapsed: inside a splitter (or any
            # layout that pins sizes) an uncapped widget keeps the space it was
            # given, so the section would leave an empty gap instead of
            # shrinking.  The minimum matters as much as the maximum — Qt
            # never shrinks a widget below it.
            collapsed_height = self._header.height() + SECTION_BORDER
            self.setMinimumHeight(collapsed_height)
            self.setMaximumHeight(collapsed_height)
        if emit:
            self.toggled.emit(expanded)

    def _apply_content_height(self) -> None:
        if not self._content_height:
            self._body.setMinimumHeight(0)
            self._body.setMaximumHeight(16777215)
            return
        height = self._content_height if self.isExpanded() else 0
        self._body.setFixedHeight(height)
