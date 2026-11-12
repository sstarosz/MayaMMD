"""Qt headless tests for :mod:`mmd.ui.collapsible_section`.

Maya's frameLayout cannot live in a Qt scroll area, so the panel uses this
widget for every section; these tests pin the behaviour the tool panel relies
on: header text, expand/collapse, the emitted signal and content heights.
"""

from __future__ import annotations

import os
import unittest

os.environ["QT_QPA_PLATFORM"] = "offscreen"

from PySide6 import QtCore, QtWidgets
from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QApplication
from shiboken6 import isValid

from mmd.ui import collapsible_section as cs
from mmd.ui.collapsible_section import CollapsibleSection

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


class _SectionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.content = QtWidgets.QWidget()
        self.addCleanup(_dispose, self.content)
        self.section = CollapsibleSection("Physics", self.content)
        self.addCleanup(_dispose, self.section)
        self.states: list[bool] = []
        self.section.toggled.connect(self.states.append)


class TestDefaults(_SectionTest):
    def test_header_shows_the_title(self) -> None:
        self.assertEqual(self.section.title, "Physics")
        self.assertEqual(self.section.header.text(), "Physics")

    def test_starts_expanded(self) -> None:
        self.assertTrue(self.section.isExpanded())
        self.assertFalse(self.section.body.isHidden())

    def test_holds_the_content(self) -> None:
        self.assertIs(self.section.content, self.content)
        self.assertIs(self.content.parentWidget(), self.section.body)

    def test_header_is_the_full_width_bar(self) -> None:
        policy = self.section.header.sizePolicy()
        self.assertEqual(
            policy.horizontalPolicy(), QtWidgets.QSizePolicy.Policy.Expanding
        )


class TestCollapsing(_SectionTest):
    def test_clicking_the_header_collapses_and_emits(self) -> None:
        self.section.header.click()
        self.assertFalse(self.section.isExpanded())
        self.assertTrue(self.section.body.isHidden())
        self.assertEqual(self.states, [False])

    def test_clicking_again_expands(self) -> None:
        self.section.setExpanded(False)
        self.section.header.click()
        self.assertTrue(self.section.isExpanded())
        self.assertEqual(self.states, [False, True])

    def test_header_state_follows_the_section(self) -> None:
        """The triangle is drawn from the header's own state."""
        self.assertTrue(self.section.header.isChecked())
        self.section.setExpanded(False)
        self.assertFalse(self.section.header.isChecked())

    def test_set_expanded_can_stay_silent(self) -> None:
        """A programmatic change must not look like a user gesture."""
        self.section.setExpanded(False, emit=False)
        self.assertFalse(self.section.isExpanded())
        self.assertEqual(self.states, [])


class TestLayoutBehaviour(_SectionTest):
    """A collapsed section must stop occupying space in its parent layout."""

    def test_collapsing_clamps_the_section_to_its_header(self) -> None:
        self.section.setExpanded(False)
        self.assertEqual(
            self.section.maximumHeight(),
            self.section.header.height() + cs.SECTION_BORDER,
        )

    def test_expanding_lifts_the_clamp(self) -> None:
        self.section.setExpanded(False)
        self.section.setExpanded(True)
        self.assertEqual(self.section.maximumHeight(), cs.NO_MAXIMUM)

    def test_isCollapsed_tracks_the_state(self) -> None:
        self.assertFalse(self.section.isCollapsed)
        self.section.setExpanded(False)
        self.assertTrue(self.section.isCollapsed)

    def test_a_splitter_gives_the_space_to_the_neighbour(self) -> None:
        """The reported bug: a collapsed section left a large empty gap."""
        splitter, _other = self._split_with_neighbour()

        self.section.setExpanded(False)
        QApplication.processEvents()

        sizes = splitter.sizes()
        self.assertLessEqual(sizes[0], self.section.header.height() + 4)
        self.assertGreater(sizes[1], 500)

    def test_a_pinned_minimum_still_frees_the_space(self) -> None:
        """Qt never shrinks a widget below its minimum height either."""
        self.section.setMinimumHeight(300)
        splitter, _other = self._split_with_neighbour()

        self.section.setExpanded(False)
        QApplication.processEvents()

        sizes = splitter.sizes()
        self.assertLessEqual(sizes[0], self.section.header.height() + 4)
        self.assertGreater(sizes[1], 500)

    def test_expanding_restores_the_minimum_height(self) -> None:
        self.section.setMinimumHeight(300)
        self.section.setExpanded(False)
        self.section.setExpanded(True)
        self.assertEqual(self.section.minimumHeight(), 300)

    def _split_with_neighbour(self) -> tuple[QtWidgets.QSplitter, QtWidgets.QWidget]:
        """Put the section above a plain widget, the way the panel does."""
        host = QtWidgets.QWidget()
        self.addCleanup(_dispose, host)
        other = QtWidgets.QWidget()
        self.addCleanup(_dispose, other)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.addWidget(self.section)
        splitter.addWidget(other)
        layout = QtWidgets.QVBoxLayout(host)
        layout.addWidget(splitter)
        host.resize(300, 600)
        host.show()
        QApplication.processEvents()
        return splitter, other


class TestContentHeight(_SectionTest):
    def test_automatic_by_default(self) -> None:
        self.assertEqual(self.section.contentHeight, 0)

    def test_fixed_height_is_applied_to_the_body(self) -> None:
        self.section.setContentHeight(300)
        self.assertEqual(self.section.body.height(), 300)

    def test_fixed_height_collapses_to_zero(self) -> None:
        """A collapsed section must not keep reserving its height."""
        self.section.setContentHeight(300)
        self.section.setExpanded(False)
        self.assertEqual(self.section.body.height(), 0)
        self.section.setExpanded(True)
        self.assertEqual(self.section.body.height(), 300)

    def test_clearing_the_height_restores_automatic_sizing(self) -> None:
        self.section.setContentHeight(300)
        self.section.setContentHeight(0)
        self.assertGreater(self.section.body.maximumHeight(), 300)

    def test_replacing_the_content_releases_the_old_widget(self) -> None:
        replacement = QtWidgets.QWidget()
        self.addCleanup(_dispose, replacement)
        self.section.setContent(replacement)
        self.assertIs(self.section.content, replacement)
        self.assertIsNone(self.content.parentWidget())


if __name__ == "__main__":
    unittest.main()
