"""Sections built with Maya's own ``frameLayout``, so they look native.

Qt cannot reproduce Maya's frame: Maya styles ``QToolButton``/``QPushButton``
application-wide, so a hand-styled header ends up with a different size, font
and colour inside Maya than outside it.  The only way to get the native look is
to let Maya draw the frame — header bar, collapse triangle, body background,
padding and fonts included.

Each section is the same three-control stack the panel has always used:

    frameLayout            ← Maya draws the section
      paneLayout(single)   ← its one child fills the frame
        text(placeholder)  ← a Qt widget lives inside this

**Collapsing.**  Maya hides the frame's panes, but the Qt *layout* around the
frame keeps whatever height it was given — a splitter in particular pins its
children to explicit sizes, which left a large empty gap where a collapsed
section used to be.  :class:`MayaSection` therefore clamps the frame widget's
minimum *and* maximum height to the header height while collapsed (and lifts
both clamps on expand), driven by the frame's own
``collapseCommand``/``expandCommand``.  The maximum alone is not enough: Qt
never shrinks a widget below its minimum height, and Maya pins the frame's
minimum to the height its panes ask for, so the whole body stayed as a gap.
The layout then shrinks the section to its title bar and gives the room to its
neighbours.
"""

from __future__ import annotations

import logging
from collections.abc import Callable

from maya import cmds
from PySide6 import QtWidgets
from shiboken6 import isValid

log = logging.getLogger(__name__)

PANE_BACKGROUND = [0.286, 0.286, 0.286]
"""The section body colour the panel has always used."""

PLACEHOLDER_PADDING = 8
"""Pixels added to a content widget's hint so the placeholder never clips it."""

NO_MAXIMUM = 16777215
"""Qt's "no maximum": what an expanded section's cap is reset to."""


class MayaSection:
    """One Maya ``frameLayout`` section and the Qt widget that wraps it.

    Owns the two things a Qt layout needs to know: that the section can shrink
    to its header, and how to make it do so when Maya collapses it.
    """

    def __init__(
        self,
        title: str,
        frame: str,
        wrapper: QtWidgets.QWidget,
        content: QtWidgets.QWidget,
        header_height: int,
    ) -> None:
        self.title = title
        self.frame = frame
        self._wrapper = wrapper
        self._target = None
        self._target_index = -1
        self._wrapper_name = wrapper.objectName()
        self.content = content
        self.header_height = header_height
        self.on_toggle: Callable[[], None] | None = None
        """Run after a collapse/expand, so the owner can react (e.g. refresh)."""
        self._expanded_height = 0
        self._content_height = 0
        self._collapsed = False
        self._expanded_minimum = 0
        """The frame's own minimum height, restored when the clamp is lifted."""
        _SECTIONS[frame] = self

    @property
    def wrapper(self) -> QtWidgets.QWidget:
        """Resolve the live frame after Maya invalidates a Python wrapper.

        Reparenting a native Maya control can invalidate the Shiboken wrapper
        while the Qt layout still owns a live widget. Never keep using the
        stale wrapper or remove its collapse callback from the registry.
        """
        if not isValid(self._wrapper) and self._target is not None:
            target = self._target
            if isValid(target):
                if isinstance(target, QtWidgets.QSplitter):
                    live = target.widget(self._target_index)
                else:
                    item = target.itemAt(self._target_index)
                    live = item.widget() if item is not None else None
                if (
                    live is not None
                    and isValid(live)
                    and live.objectName() == self._wrapper_name
                ):
                    self._wrapper = live
        return self._wrapper

    def bind_target(self, target, index: int) -> None:
        """Remember the Qt owner that can return a fresh wrapper."""
        self._target = target
        self._target_index = index

    def setContentHeight(self, height: int) -> None:
        """Reserve a fixed body height, leaving its contents to scroll."""
        self._content_height = height
        if not self.isCollapsed:
            self.wrapper.setFixedHeight(self.header_height + height)

    # ── Collapse handling ─────────────────────────────────────────────────

    def setCollapsed(self, collapsed: bool) -> None:
        """Shrink the section to its header, or restore it."""
        if collapsed:
            if not self.isCollapsed:
                # Remember what the layout had given us: a splitter cannot
                # recover it after the clamp.
                self._expanded_height = max(self.wrapper.height(), self.header_height)
                self._expanded_minimum = self.wrapper.minimumHeight()
            self.wrapper.setMinimumHeight(self.header_height)
            self.wrapper.setMaximumHeight(self.header_height)
        else:
            self.wrapper.setMaximumHeight(NO_MAXIMUM)
            self.wrapper.setMinimumHeight(self._expanded_minimum)
            if self._expanded_height > self.header_height:
                self.wrapper.resize(self.wrapper.width(), self._expanded_height)
            self._expanded_height = 0
            if self._content_height:
                self.wrapper.setFixedHeight(self.header_height + self._content_height)
        self._collapsed = collapsed
        if self.on_toggle is not None:
            self.on_toggle()

    @property
    def isCollapsed(self) -> bool:
        """``True`` while the section is clamped to its header."""
        return self._collapsed

    def dispose(self) -> None:
        """Forget this section (the panel is closing)."""
        _SECTIONS.pop(self.frame, None)


_SECTIONS: dict[str, MayaSection] = {}


def section_for(frame: str) -> MayaSection | None:
    """The live section for a Maya frame name, if it still exists."""
    return _SECTIONS.get(frame)


def clear_sections() -> None:
    """Drop every registered section (panel close, tests)."""
    _SECTIONS.clear()


def on_frame_toggled(frame: str, collapsed: bool) -> None:
    """MEL entry point: clamp or restore the section ``frame``.

    Called from the frame's ``collapseCommand`` / ``expandCommand``.
    """
    section = _SECTIONS.get(frame)
    if section is None:
        return
    try:
        section.setCollapsed(bool(collapsed))
    except RuntimeError:  # pragma: no cover - widget already destroyed
        _SECTIONS.pop(frame, None)


def _toggle_command(
    frame: str, collapsed: bool, extra: str | None = None
) -> Callable[..., None]:
    """Return a Python callback for cmds.frameLayout's script flags.

    A MEL-style ``python(...)`` string is not Python callback source. Use a
    callable so Maya invokes the clamp without a second language interpreter.
    """

    def callback(*_args: object) -> None:
        on_frame_toggled(frame, collapsed)
        if extra:
            from maya import mel

            mel.eval(extra)

    return callback


def _default_header_height(widget: QtWidgets.QWidget) -> int:
    """Height of a Maya frame title bar at the current DPI.

    Maya does not expose the frame's header height, so it is derived from the
    font the panel actually uses; collapsing to a fixed constant would clip the
    title on a scaled display.
    """
    return widget.fontMetrics().height() + HEADER_ALLOWANCE


HEADER_ALLOWANCE = 10
"""Padding above and below the title text inside a frame header."""


def maya_section(
    title: str,
    content: QtWidgets.QWidget,
    to_qt=None,
    height: int | None = None,
    collapsed: bool = False,
    expand_command: str | None = None,
    header_height: int | None = None,
) -> MayaSection | None:
    """Build one Maya ``frameLayout`` section around ``content``.

    Args:
        title: The frame's label (what the header shows).
        content: The Qt widget to host.
        to_qt: Converts a Maya control name to a Qt widget (the caller's
            ``MQtUtil`` wrapper).  Without it the Maya side cannot be built.
        height: Placeholder height.  ``None`` uses the content's size hint.
        collapsed: Start collapsed.
        expand_command: Extra MEL run when the frame is expanded, after the
            clamp is lifted (used to refresh stale data).
        header_height: Height of the title bar — what a collapsed section is
            clamped to.

    Returns:
        The live :class:`MayaSection`, or ``None`` when the Maya side is
        unavailable (no GUI, stale control, a failed call).
    """
    if to_qt is None:
        return None

    frame: str = ""
    depth = 0
    try:
        # The collapse callbacks need the frame's name, so they are attached
        # after creation (below).
        frame = cmds.frameLayout(label=title, collapsable=True, collapse=collapsed)
        depth = 1
        cmds.paneLayout(configuration="single", backgroundColor=PANE_BACKGROUND)
        depth = 2
        placeholder = to_qt(
            cmds.text(
                label="",
                height=(
                    height
                    if height is not None
                    else content.sizeHint().height() + PLACEHOLDER_PADDING
                ),
            )
        )
        if placeholder is not None:
            body = QtWidgets.QVBoxLayout(placeholder)
            body.setContentsMargins(0, 0, 0, 0)
            body.addWidget(content)
    except Exception:  # noqa: BLE001 - Maya UI is best-effort
        log.warning("Could not build the %s section", title, exc_info=True)
        return None
    finally:
        # Restore the parent stack even on failure: an unbalanced stack would
        # nest every later section inside this one.
        while depth:
            depth -= 1
            try:
                cmds.setParent("..")
            except Exception:  # noqa: BLE001 - already unwinding a failure
                break

    wrap = None
    if frame:
        try:
            wrap = to_qt(frame)
        except Exception:  # noqa: BLE001 - a failed bridge means "no Maya UI"
            log.debug("Could not wrap the %s frame", title, exc_info=True)
    if wrap is None:
        return None

    section = MayaSection(
        title,
        frame,
        wrap,
        content,
        header_height if header_height is not None else _default_header_height(content),
    )
    if height is not None:
        section.setContentHeight(height)
    try:
        cmds.frameLayout(
            frame,
            edit=True,
            collapseCommand=_toggle_command(frame, True),
            expandCommand=_toggle_command(frame, False, expand_command),
        )
    except Exception:  # noqa: BLE001 - non-fatal: the frame still works
        log.warning("Could not attach collapse callbacks to %s", title, exc_info=True)

    if collapsed:
        # No callback fires for the state a frame was created in, so mirror it.
        section.setCollapsed(True)
    return section


def embed_maya_section(
    target,
    title: str,
    content: QtWidgets.QWidget,
    to_qt=None,
    fallback_factory=None,
    height: int | None = None,
    expand_command: str | None = None,
):
    """Add a Maya section to ``target`` (a Qt layout or a splitter).

    Args:
        target: Anything with ``addWidget`` — ``QVBoxLayout``, ``QSplitter``…
        fallback_factory: ``factory(title, content) -> widget`` used when the
            Maya frame cannot be built (no GUI, headless tests).  Its result is
            returned so the caller can keep a reference.

    Returns:
        The :class:`MayaSection` when the native frame was created (already
        added to ``target``), otherwise the Qt fallback widget.
    """
    section = maya_section(
        title, content, to_qt, height=height, expand_command=expand_command
    )
    if section is not None:
        index = target.count()
        target.addWidget(section.wrapper)
        section.bind_target(target, index)
        return section

    log.debug("Falling back to a Qt section for %s", title)
    if fallback_factory is None:
        target.addWidget(content)
        return content
    fallback = fallback_factory(title, content)
    target.addWidget(fallback)
    return fallback
