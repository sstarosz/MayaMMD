"""Unit tests for :mod:`mmd.ui.maya_section`.

The sections are Maya ``frameLayout``s hosting Qt widgets; these tests pin the
command sequence, the parent-stack unwinding that keeps later sections from
nesting, and the Qt fallback used when there is no Maya GUI.
"""

from __future__ import annotations

import os
import unittest
from unittest.mock import MagicMock, call, patch

os.environ["QT_QPA_PLATFORM"] = "offscreen"

from tests.unit_tests.maya.maya_stub import install_maya_stub

install_maya_stub(profile="headless")

from PySide6 import QtCore, QtWidgets
from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QApplication
from shiboken6 import isValid

from mmd.ui import maya_section as ms

_app: QCoreApplication | None = None


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


def _dispose(widget: QtWidgets.QWidget) -> None:
    if isValid(widget):
        widget.deleteLater()


class _SectionTestBase(unittest.TestCase):
    def setUp(self) -> None:
        patcher = patch("mmd.ui.maya_section.cmds")
        self.cmds = patcher.start()
        self.addCleanup(patcher.stop)
        self.cmds.frameLayout.return_value = "MayaMMDVisibility"
        self.content = QtWidgets.QWidget()
        self.addCleanup(_dispose, self.content)
        self.placeholders: list[QtWidgets.QWidget] = []

        def to_qt(_name: str) -> QtWidgets.QWidget:
            widget = QtWidgets.QWidget()
            self.placeholders.append(widget)
            self.addCleanup(_dispose, widget)
            return widget

        self.to_qt = to_qt


class TestMayaSection(_SectionTestBase):
    def test_without_a_qt_bridge_nothing_is_built(self) -> None:
        self.assertIsNone(ms.maya_section("Visibility", self.content, None))
        self.cmds.frameLayout.assert_not_called()

    def test_builds_frame_pane_and_placeholder_in_order(self) -> None:
        ms.maya_section("Visibility", self.content, self.to_qt)

        frame_kwargs = self.cmds.frameLayout.call_args_list[0].kwargs
        self.assertEqual(frame_kwargs["label"], "Visibility")
        self.assertTrue(frame_kwargs["collapsable"])
        self.assertFalse(frame_kwargs["collapse"])

        pane_kwargs = self.cmds.paneLayout.call_args.kwargs
        self.assertEqual(pane_kwargs["configuration"], "single")
        self.assertEqual(pane_kwargs["backgroundColor"], ms.PANE_BACKGROUND)

        text_kwargs = self.cmds.text.call_args.kwargs
        self.assertGreater(text_kwargs["height"], self.content.sizeHint().height())

    def test_returns_the_frame_wrapper(self) -> None:
        wrapper = ms.maya_section("Morphs", self.content, self.to_qt)
        self.assertIsNotNone(wrapper)

    def test_parent_stack_is_restored(self) -> None:
        ms.maya_section("Morphs", self.content, self.to_qt)
        self.assertEqual(self.cmds.setParent.call_args_list, [call(".."), call("..")])

    def test_parent_stack_is_restored_when_the_pane_fails(self) -> None:
        """A half-built section must not swallow the next one."""
        self.cmds.paneLayout.side_effect = RuntimeError("boom")

        self.assertIsNone(ms.maya_section("Morphs", self.content, self.to_qt))
        self.assertEqual(self.cmds.setParent.call_args_list, [call("..")])

    def test_content_lands_in_the_placeholder(self) -> None:
        ms.maya_section("Morphs", self.content, self.to_qt)
        self.assertIs(self.content.parentWidget(), self.placeholders[0])

    def test_height_can_be_forced(self) -> None:
        ms.maya_section("Physics", self.content, self.to_qt, height=400)
        self.assertEqual(self.cmds.text.call_args.kwargs["height"], 400)

    def test_collapse_and_expand_commands_execute_the_clamp(self) -> None:
        section = ms.maya_section("Physics", self.content, self.to_qt)
        assert section is not None
        section.wrapper.setMinimumHeight(400)
        on_toggle = MagicMock()
        section.on_toggle = on_toggle
        edit_kwargs = self.cmds.frameLayout.call_args.kwargs

        # Execute what Maya receives, rather than merely inspecting its text.
        edit_kwargs["collapseCommand"]()
        self.assertTrue(section.isCollapsed)
        self.assertEqual(section.wrapper.minimumHeight(), section.header_height)
        self.assertEqual(section.wrapper.maximumHeight(), section.header_height)
        on_toggle.assert_called_once_with()

        edit_kwargs["expandCommand"]()
        self.assertFalse(section.isCollapsed)
        self.assertEqual(section.wrapper.minimumHeight(), 400)
        self.assertEqual(on_toggle.call_count, 2)

    def test_optional_mel_runs_after_the_python_expand_callback(self) -> None:
        section = ms.maya_section(
            "Physics", self.content, self.to_qt, expand_command="customRefresh()"
        )
        assert section is not None
        section.setCollapsed(True)
        edit_kwargs = self.cmds.frameLayout.call_args.kwargs
        mel = MagicMock()
        mel.eval.side_effect = lambda _command: self.assertFalse(section.isCollapsed)
        with (
            patch.dict("sys.modules", {"maya.mel": mel}),
            patch("maya.mel", mel, create=True),
        ):
            edit_kwargs["expandCommand"]("ignored Maya argument")
        mel.eval.assert_called_once_with("customRefresh()")


class TestCollapseClamp(_SectionTestBase):
    """Maya hides the panes but the Qt layout keeps the height: clamp it."""

    def test_collapsing_caps_the_wrapper_at_the_header(self) -> None:
        section = ms.maya_section("Morphs", self.content, self.to_qt)
        assert section is not None
        section.setCollapsed(True)
        self.assertEqual(section.wrapper.maximumHeight(), section.header_height)
        self.assertTrue(section.isCollapsed)

    def test_collapsing_also_lowers_the_minimum_height(self) -> None:
        """Qt never shrinks a widget below its minimum: clamp that too."""
        section = ms.maya_section("Morphs", self.content, self.to_qt)
        assert section is not None
        section.wrapper.setMinimumHeight(400)
        section.setCollapsed(True)
        self.assertEqual(section.wrapper.minimumHeight(), section.header_height)

    def test_expanding_restores_the_minimum_height(self) -> None:
        section = ms.maya_section("Morphs", self.content, self.to_qt)
        assert section is not None
        section.wrapper.setMinimumHeight(400)
        section.setCollapsed(True)
        section.setCollapsed(False)
        self.assertEqual(section.wrapper.minimumHeight(), 400)

    def test_a_pinned_minimum_does_not_trap_the_splitter_space(self) -> None:
        """The reported gap: Maya pins the frame minimum to its panes.

        A collapsed section must hand its space to its neighbour instead of
        keeping the whole body as an empty gap.
        """
        section = ms.maya_section("Morphs", self.content, self.to_qt)
        assert section is not None
        section.wrapper.setMinimumHeight(400)

        host = QtWidgets.QWidget()
        self.addCleanup(_dispose, host)
        neighbour = QtWidgets.QWidget()
        self.addCleanup(_dispose, neighbour)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        splitter.addWidget(section.wrapper)
        splitter.addWidget(neighbour)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        layout = QtWidgets.QVBoxLayout(host)
        layout.addWidget(splitter)
        host.resize(300, 600)
        host.show()
        QApplication.processEvents()
        before = splitter.sizes()

        section.setCollapsed(True)
        QApplication.processEvents()

        sizes = splitter.sizes()
        self.assertLessEqual(sizes[0], section.header_height + 4)
        self.assertGreater(sizes[1], 500)

        section.setCollapsed(False)
        QApplication.processEvents()
        self.assertEqual(splitter.sizes(), before)

    def test_expanding_lifts_the_cap(self) -> None:
        section = ms.maya_section("Morphs", self.content, self.to_qt)
        assert section is not None
        section.setCollapsed(True)
        section.setCollapsed(False)
        self.assertEqual(section.wrapper.maximumHeight(), ms.NO_MAXIMUM)
        self.assertFalse(section.isCollapsed)

    def test_fixed_body_height_is_restored_after_native_collapse(self) -> None:
        section = ms.maya_section("Morphs", self.content, self.to_qt, height=280)
        callbacks = self.cmds.frameLayout.call_args.kwargs
        self.assertFalse(section.isCollapsed)
        self.assertEqual(section.wrapper.maximumHeight(), section.header_height + 280)
        callbacks["collapseCommand"]()
        self.assertTrue(section.isCollapsed)
        self.assertEqual(section.wrapper.maximumHeight(), section.header_height)
        callbacks["expandCommand"]()
        self.assertFalse(section.isCollapsed)
        self.assertEqual(section.wrapper.minimumHeight(), section.header_height + 280)
        self.assertEqual(section.wrapper.maximumHeight(), section.header_height + 280)

    def test_on_toggle_runs_after_each_change(self) -> None:
        section = ms.maya_section("Morphs", self.content, self.to_qt)
        assert section is not None
        calls: list[str] = []
        section.on_toggle = lambda: calls.append("x")
        section.setCollapsed(True)
        section.setCollapsed(False)
        self.assertEqual(calls, ["x", "x"])

    def test_created_collapsed_is_clamped_immediately(self) -> None:
        """No callback fires for the state a frame is built in."""
        section = ms.maya_section("Morphs", self.content, self.to_qt, collapsed=True)
        assert section is not None
        self.assertTrue(section.isCollapsed)

    def test_mel_toggle_finds_the_section(self) -> None:
        section = ms.maya_section("Morphs", self.content, self.to_qt)
        assert section is not None
        ms.on_frame_toggled(section.frame, True)
        self.assertTrue(section.isCollapsed)
        ms.on_frame_toggled(section.frame, False)
        self.assertFalse(section.isCollapsed)

    def test_unknown_frame_is_ignored(self) -> None:
        ms.on_frame_toggled("|not_a_section", True)

    def test_frame_toggle_discards_a_dead_section(self) -> None:
        """A destroyed widget must not break the Maya callback."""
        section = ms.maya_section("Morphs", self.content, self.to_qt)
        assert section is not None

        def dead(_collapsed: bool) -> None:
            raise RuntimeError("wrapped C/C++ object has been deleted")

        with patch.object(ms.MayaSection, "setCollapsed", side_effect=dead):
            ms.on_frame_toggled(section.frame, True)

        self.assertIsNone(ms.section_for(section.frame))


class TestEmbedSection(_SectionTestBase):
    def setUp(self) -> None:
        super().setUp()
        self.target = QtWidgets.QWidget()
        self.addCleanup(_dispose, self.target)
        self.layout = QtWidgets.QVBoxLayout(self.target)

    def test_native_frame_is_added_to_the_layout(self) -> None:
        section = ms.embed_maya_section(
            self.layout,
            "Morphs",
            self.content,
            self.to_qt,
            fallback_factory=MagicMock(),
        )
        self.assertIsInstance(section, ms.MayaSection)
        self.assertEqual(self.layout.count(), 1)
        item = self.layout.itemAt(0)
        assert item is not None
        self.assertIs(item.widget(), section.wrapper)

    def test_callback_recovers_a_stale_wrapper_from_its_owner(self) -> None:
        from shiboken6 import delete

        for use_splitter in (False, True):
            with self.subTest(splitter=use_splitter):
                target = QtWidgets.QSplitter() if use_splitter else self.layout
                if use_splitter:
                    self.addCleanup(_dispose, target)
                section = ms.embed_maya_section(
                    target, "Morphs", self.content, self.to_qt
                )
                live = section.wrapper
                callback = self.cmds.frameLayout.call_args.kwargs
                # Reproduce the report: saved wrapper is dead, owner has a
                # live frame with the same name and no height constraint.
                stale = QtWidgets.QWidget()
                delete(stale)
                section._wrapper = stale
                callback["collapseCommand"]()
                self.assertIs(section.wrapper, live)
                self.assertEqual(live.maximumHeight(), section.header_height)
                self.assertIs(ms.section_for(section.frame), section)
                callback["expandCommand"]()
                self.assertEqual(live.maximumHeight(), ms.NO_MAXIMUM)

    def test_falls_back_to_a_qt_section(self) -> None:
        factory = MagicMock(side_effect=lambda title, content: QtWidgets.QWidget())
        fallback = ms.embed_maya_section(
            self.layout, "Morphs", self.content, None, fallback_factory=factory
        )

        factory.assert_called_once_with("Morphs", self.content)
        self.assertIsNotNone(fallback)
        self.assertEqual(self.layout.count(), 1)

    def test_falls_back_to_the_bare_widget_without_a_factory(self) -> None:
        ms.embed_maya_section(self.layout, "Morphs", self.content, None)
        item = self.layout.itemAt(0)
        assert item is not None
        self.assertIs(item.widget(), self.content)


if __name__ == "__main__":
    unittest.main()
