"""Qt headless tests for the Visibility panel and its Maya section.

Three layers are covered:

* :class:`VisibilityPanel` - the standalone widget (toggles, master button,
  styling, syncing).
* ``CollapsibleSection`` hosting - the shared collapsible header the panel is
  embedded in.
* ``MMMToolWidget`` wiring - dispatching a toggle to the targeted models and
  mirroring the scene back into the panel.

Maya is stubbed and Qt runs on the ``offscreen`` platform.
"""

from __future__ import annotations

import os
import unittest
from unittest.mock import call, patch

# ── Force offscreen platform BEFORE Qt imports ─────────────────────────
os.environ["QT_QPA_PLATFORM"] = "offscreen"

# ── Install Maya stub (for import passthrough) ─────────────────────────
from tests.unit_tests.maya.maya_stub import install_maya_stub

install_maya_stub(profile="headless")

from PySide6.QtCore import QCoreApplication
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QApplication

# ``tool_main_widget`` hot-reloads its dependencies in dev mode, which
# re-executes module globals and breaks class identity for modules other test
# files have already imported - turn it off for this process.
os.environ["MAYAMMD_DEV"] = "0"

from mmd.maya import model_visibility as mv
from mmd.ui import tool_main_widget as tw
from mmd.ui import visibility_panel as vp
from mmd.ui.tool_main_widget import MMMToolWidget
from mmd.ui.visibility_panel import VisibilityPanel

del os.environ["MAYAMMD_DEV"]

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


def _block(stylesheet: str, selector: str) -> str:
    """Body of one ``selector { ... }`` rule from the toggle stylesheet."""
    return stylesheet.split(f"{selector} {{")[1].split("}")[0]


def _declarations(block: str) -> dict[str, str]:
    return {
        key.strip(): value.strip()
        for key, value in (
            chunk.split(":", 1) for chunk in block.split(";") if ":" in chunk
        )
    }


# ═══════════════════════════════════════════════════════════════════════════
#  VisibilityPanel - the widget itself
# ═══════════════════════════════════════════════════════════════════════════


class TestVisibilityPanelWidget(unittest.TestCase):
    def _panel(self) -> VisibilityPanel:
        panel = VisibilityPanel()
        self.addCleanup(panel.deleteLater)
        return panel

    def _show(self, panel: VisibilityPanel, width: int = 600) -> None:
        panel.resize(width, 40)
        panel.show()
        QApplication.processEvents()
        self.addCleanup(panel.hide)

    # ── Structure ────────────────────────────────────────────────────────

    def test_one_toggle_per_part_plus_a_master(self) -> None:
        panel = self._panel()
        self.assertEqual(
            [button.text() for button in panel.buttons.values()],
            ["Mesh", "Bones", "Rigid Bodies", "Joints"],
        )
        for button in panel.buttons.values():
            self.assertTrue(button.isCheckable())
        self.assertEqual(panel.master_button.text(), vp.SHOW_ALL_LABEL)
        self.assertFalse(panel.master_button.isCheckable())

    def test_defaults_match_default_visibility(self) -> None:
        panel = self._panel()
        for part, button in panel.buttons.items():
            self.assertEqual(button.isChecked(), mv.DEFAULT_VISIBILITY[part])

    def test_everything_starts_disabled(self) -> None:
        panel = self._panel()
        for part, button in panel.buttons.items():
            self.assertFalse(button.isEnabled(), f"{part} should start disabled")
        self.assertFalse(panel.master_button.isEnabled())

    # ── Styling ──────────────────────────────────────────────────────────

    def test_off_is_a_flat_empty_slot(self) -> None:
        panel = self._panel()
        rest = _block(panel.styleSheet(), "QPushButton")
        self.assertIn(f"background-color: {QColor(vp.TOGGLE_OFF_FILL).name()};", rest)
        self.assertIn("border: none;", rest)

    def test_on_fills_the_slot_with_maya_blue(self) -> None:
        panel = self._panel()
        checked = _block(panel.styleSheet(), "QPushButton:checked")
        self.assertIn(
            f"background-color: {QColor(vp.MAYA_SELECTION_BLUE).name()};", checked
        )
        self.assertNotIn("border", checked)

    def test_on_and_off_fills_are_distinct(self) -> None:
        panel = self._panel()
        style = panel.styleSheet()
        off = _declarations(_block(style, "QPushButton"))["background-color"]
        on = _declarations(_block(style, "QPushButton:checked"))["background-color"]
        self.assertNotEqual(off, on)

    def test_toggles_are_borderless_with_2px_corners(self) -> None:
        style = self._panel().styleSheet()
        self.assertEqual(style.count("border: none;"), 3)
        self.assertNotIn("border-top", style)
        self.assertNotIn("border-bottom", style)
        self.assertIn("border-radius: 2px;", style)

    def test_label_colour_is_the_same_in_off_and_on(self) -> None:
        style = self._panel().styleSheet()
        self.assertIn(f"color: {vp.TOGGLE_OFF_TEXT};", style)
        self.assertIn(f"color: {vp.TOGGLE_ON_TEXT};", style)

    def test_disabled_is_identical_for_checked_and_unchecked(self) -> None:
        style = self._panel().styleSheet()
        disabled = _block(style, "QPushButton:disabled")
        checked_disabled = _block(style, "QPushButton:checked:disabled")

        self.assertEqual(_declarations(disabled), _declarations(checked_disabled))
        self.assertNotIn(QColor(vp.MAYA_SELECTION_BLUE).name(), checked_disabled)

    def test_disabled_matches_the_native_disabled_buttons(self) -> None:
        """Same +7 lightness delta over its background as Maya's own disabled
        buttons (#4B4B4B on #444444), plus the native #808080 label."""
        native_delta = QColor("#4B4B4B").lightness() - QColor("#444444").lightness()
        disabled_fill = QColor(vp.TOGGLE_DISABLED_FILL)
        self.assertEqual(
            disabled_fill.lightness() - QColor("#494949").lightness(), native_delta
        )
        self.assertEqual(QColor(vp.TOGGLE_DISABLED_TEXT).name(), "#808080")
        self.assertIn(
            f"background-color: {vp.TOGGLE_DISABLED_FILL};", self._panel().styleSheet()
        )

    # ── Layout ───────────────────────────────────────────────────────────

    def test_buttons_split_the_row_evenly(self) -> None:
        panel = self._panel()
        self._show(panel)

        widths = [button.width() for button in panel.buttons.values()]
        widths.append(panel.master_button.width())
        self.assertLessEqual(max(widths) - min(widths), 1, f"uneven widths: {widths}")

        layout = panel.layout()
        margins = layout.contentsMargins()
        usable = panel.width() - margins.left() - margins.right()
        self.assertEqual(sum(widths) + layout.spacing() * (len(widths) - 1), usable)

    def test_buttons_are_inset_from_the_edges(self) -> None:
        panel = self._panel()
        self._show(panel)

        margins = panel.layout().contentsMargins()
        self.assertGreater(margins.left(), 0)
        self.assertGreater(margins.right(), 0)
        self.assertGreater(panel.buttons[mv.MESH].x(), 0)
        last = panel.master_button
        self.assertLessEqual(last.x() + last.width(), panel.width())

    def test_row_has_tight_gaps(self) -> None:
        panel = self._panel()
        self.assertEqual(panel.layout().spacing(), vp.TOGGLE_SPACING)
        self.assertLess(panel.layout().spacing(), 6)

    def test_row_height_follows_the_native_button(self) -> None:
        panel = self._panel()
        self._show(panel)
        expected = max(38 - vp.TOGGLE_HEIGHT_PAD, 16)

        panel.match_row_height(38)
        QApplication.processEvents()

        self.assertEqual(panel.buttons[mv.MESH].height(), expected)
        self.assertEqual(panel.master_button.height(), expected)

    # ── Syncing with the scene ───────────────────────────────────────────

    def test_sync_sets_checks_and_enables_present_parts(self) -> None:
        panel = self._panel()
        panel.sync(
            {mv.MESH: True, mv.BONES: False, mv.RIGID_BODIES: None, mv.JOINTS: True}
        )

        self.assertTrue(panel.buttons[mv.MESH].isChecked())
        self.assertFalse(panel.buttons[mv.BONES].isChecked())
        self.assertTrue(panel.buttons[mv.JOINTS].isChecked())
        self.assertTrue(panel.buttons[mv.MESH].isEnabled())
        self.assertFalse(panel.buttons[mv.RIGID_BODIES].isEnabled())

    def test_sync_does_not_emit_part_toggles(self) -> None:
        panel = self._panel()
        seen: list[tuple[str, bool]] = []
        panel.partToggled.connect(lambda part, visible: seen.append((part, visible)))
        panel.sync(dict.fromkeys(mv.PARTS, True))
        self.assertEqual(seen, [])

    def test_sync_without_a_model_disables_everything(self) -> None:
        panel = self._panel()
        panel.sync(dict.fromkeys(mv.PARTS, True))
        panel.sync({})
        for part, button in panel.buttons.items():
            self.assertFalse(button.isEnabled(), f"{part} should be disabled")
        self.assertFalse(panel.master_button.isEnabled())
        self.assertFalse(panel.allVisible)

    def test_master_label_switches_between_show_and_hide(self) -> None:
        panel = self._panel()
        panel.sync(dict.fromkeys(mv.PARTS, True))
        self.assertTrue(panel.allVisible)
        self.assertEqual(panel.master_button.text(), vp.HIDE_ALL_LABEL)

        panel.sync({**dict.fromkeys(mv.PARTS, True), mv.JOINTS: False})
        self.assertFalse(panel.allVisible)
        self.assertEqual(panel.master_button.text(), vp.SHOW_ALL_LABEL)

    # ── Individually hidden nodes (tooltip-only reporting) ───────────────

    def test_tooltip_explains_nodes_hidden_individually(self) -> None:
        panel = self._panel()
        panel.sync(dict.fromkeys(mv.PARTS, True), {mv.BONES: 3})

        tooltip = panel.buttons[mv.BONES].toolTip()
        self.assertIn("3 nodes hidden individually", tooltip)
        self.assertIn("Show All", tooltip)
        # The button state must not change: the container is still visible.
        self.assertTrue(panel.buttons[mv.BONES].isChecked())

    def test_tooltip_is_plain_when_nothing_is_hidden(self) -> None:
        panel = self._panel()
        panel.sync(dict.fromkeys(mv.PARTS, True), dict.fromkeys(mv.PARTS, 0))
        self.assertNotIn("hidden individually", panel.buttons[mv.BONES].toolTip())

    def test_singular_wording_for_one_node(self) -> None:
        panel = self._panel()
        panel.sync(dict.fromkeys(mv.PARTS, True), {mv.MESH: 1})
        self.assertIn("1 node hidden individually", panel.buttons[mv.MESH].toolTip())

    def test_master_says_show_all_when_nodes_are_hidden(self) -> None:
        """Everything visible at container level, but one bone is hidden."""
        panel = self._panel()
        panel.sync(dict.fromkeys(mv.PARTS, True), {mv.BONES: 1})

        self.assertFalse(panel.allVisible)
        self.assertEqual(panel.master_button.text(), vp.SHOW_ALL_LABEL)
        self.assertIn("1 node hidden individually", panel.master_button.toolTip())

    def test_style_is_kept_while_disabled(self) -> None:
        """Disabled toggles keep the toggle look, not Maya's native buttons."""
        panel = self._panel()
        panel.sync(dict.fromkeys(mv.PARTS, True))
        enabled_style = panel.styleSheet()
        panel.sync({})
        self.assertTrue(panel.styleSheet())
        self.assertEqual(panel.styleSheet(), enabled_style)

    # ── Signals ──────────────────────────────────────────────────────────

    def test_part_toggle_emits_part_and_new_state(self) -> None:
        panel = self._panel()
        panel.sync(dict.fromkeys(mv.PARTS, True))
        seen: list[tuple[str, bool]] = []
        panel.partToggled.connect(lambda part, visible: seen.append((part, visible)))

        panel.buttons[mv.BONES].click()

        self.assertEqual(seen, [(mv.BONES, False)])

    def test_master_click_requests_show_all_then_hide_all(self) -> None:
        panel = self._panel()
        seen: list[bool] = []
        panel.allToggled.connect(seen.append)

        panel.sync({**dict.fromkeys(mv.PARTS, True), mv.JOINTS: False})
        panel.master_button.click()  # not everything visible -> show all
        panel.sync(dict.fromkeys(mv.PARTS, True))
        panel.master_button.click()  # everything visible -> hide all

        self.assertEqual(seen, [True, False])


# ═══════════════════════════════════════════════════════════════════════════
#  MMMToolWidget wiring
# ═══════════════════════════════════════════════════════════════════════════


class TestToolWidgetWiring(unittest.TestCase):
    def setUp(self) -> None:
        super().setUp()

    def _make_widget(self) -> MMMToolWidget:
        widget = MMMToolWidget()
        self.addCleanup(widget.deleteLater)
        return widget

    def _widget_with_enabled_toggles(
        self, widget: MMMToolWidget | None = None
    ) -> MMMToolWidget:
        widget = widget or self._make_widget()
        all_visible = dict.fromkeys(mv.PARTS, True)
        with patch.object(tw, "get_part_visibility", return_value=all_visible):
            widget._refresh_visibility_buttons()
        for button in widget.visibility_panel.buttons.values():
            self.assertTrue(button.isEnabled(), "toggles should be enabled")
        return widget

    def test_hosts_a_visibility_panel(self) -> None:
        widget = self._make_widget()
        self.assertIsInstance(widget.visibility_panel, VisibilityPanel)
        self.assertFalse(widget.visibility_panel.buttons[mv.MESH].isEnabled())

    def test_refresh_syncs_from_the_scene(self) -> None:
        widget = self._make_widget()
        states = {
            mv.MESH: True,
            mv.BONES: False,
            mv.RIGID_BODIES: None,
            mv.JOINTS: True,
        }
        with (
            patch.object(tw, "get_part_visibility", return_value=states),
            patch.object(tw, "set_part_visibility") as set_visibility,
        ):
            widget._refresh_visibility_buttons()

        buttons = widget.visibility_panel.buttons
        self.assertTrue(buttons[mv.MESH].isChecked())
        self.assertFalse(buttons[mv.BONES].isChecked())
        self.assertTrue(buttons[mv.JOINTS].isChecked())
        self.assertTrue(buttons[mv.MESH].isEnabled())
        self.assertFalse(buttons[mv.RIGID_BODIES].isEnabled())
        # Syncing the panel must never write to the scene.
        set_visibility.assert_not_called()

    def test_toggle_applies_to_every_targeted_model(self) -> None:
        widget = self._widget_with_enabled_toggles()
        joints = widget.visibility_panel.buttons[mv.JOINTS]
        after = {**dict.fromkeys(mv.PARTS, True), mv.JOINTS: False}

        with (
            patch.object(
                tw,
                "find_all_model_roots_from_selection",
                return_value=["|A_Root", "|B_Root"],
            ),
            patch.object(
                tw, "set_part_visibility", return_value=True
            ) as set_visibility,
            patch.object(tw, "get_part_visibility", return_value=after),
        ):
            joints.click()  # shown -> hide

        set_visibility.assert_has_calls(
            [
                call("|A_Root", mv.JOINTS, False),
                call("|B_Root", mv.JOINTS, False),
            ]
        )
        self.assertFalse(joints.isChecked(), "panel must follow the scene")

    def test_toggle_shows_a_hidden_part(self) -> None:
        widget = self._make_widget()
        before = {**dict.fromkeys(mv.PARTS, True), mv.MESH: False}
        with patch.object(tw, "get_part_visibility", return_value=before):
            widget._refresh_visibility_buttons()
        mesh = widget.visibility_panel.buttons[mv.MESH]
        self.assertFalse(mesh.isChecked())

        with (
            patch.object(
                tw, "find_all_model_roots_from_selection", return_value=["|A_Root"]
            ),
            patch.object(
                tw, "set_part_visibility", return_value=True
            ) as set_visibility,
            patch.object(
                tw, "get_part_visibility", return_value=dict.fromkeys(mv.PARTS, True)
            ),
        ):
            mesh.click()

        set_visibility.assert_called_once_with("|A_Root", mv.MESH, True)
        self.assertTrue(mesh.isChecked())

    def test_toggle_without_a_model_is_a_noop(self) -> None:
        widget = self._widget_with_enabled_toggles()
        with (
            patch.object(tw, "find_all_model_roots_from_selection", return_value=[]),
            patch.object(
                tw, "set_part_visibility", return_value=True
            ) as set_visibility,
            patch.object(tw, "get_part_visibility", return_value={}),
        ):
            widget.visibility_panel.buttons[mv.MESH].click()

        set_visibility.assert_not_called()

    def test_toggle_without_a_model_disables_the_panel(self) -> None:
        """The targeted model can be deleted while the panel still shows it."""
        widget = self._widget_with_enabled_toggles()
        with (
            patch.object(tw, "find_all_model_roots_from_selection", return_value=[]),
            patch.object(tw, "get_part_visibility", return_value={}),
        ):
            widget.visibility_panel.buttons[mv.MESH].click()

        for part, button in widget.visibility_panel.buttons.items():
            self.assertFalse(button.isEnabled(), f"{part} should be disabled")

    def test_failed_toggle_restores_the_button(self) -> None:
        widget = self._widget_with_enabled_toggles()
        joints = widget.visibility_panel.buttons[mv.JOINTS]

        with (
            patch.object(
                tw, "find_all_model_roots_from_selection", return_value=["|A_Root"]
            ),
            patch.object(tw, "set_part_visibility", return_value=False),
            patch.object(
                tw, "get_part_visibility", return_value=dict.fromkeys(mv.PARTS, True)
            ),
        ):
            joints.click()  # hide attempt the scene rejects

        self.assertTrue(joints.isChecked())

    def test_partial_toggle_resyncs_to_the_active_model(self) -> None:
        """One of two models lacks the part: the button must show the truth."""
        widget = self._widget_with_enabled_toggles()
        with (
            patch.object(
                tw,
                "find_all_model_roots_from_selection",
                return_value=["|A_Root", "|B_Root"],
            ),
            patch.object(tw, "set_part_visibility", side_effect=[False, True]),
            patch.object(
                tw, "get_part_visibility", return_value=dict.fromkeys(mv.PARTS, True)
            ),
        ):
            widget.visibility_panel.buttons[mv.MESH].click()  # shown -> hide

        self.assertTrue(
            widget.visibility_panel.buttons[mv.MESH].isChecked(),
            "the active model still shows the mesh, so the toggle must stay on",
        )

    def test_master_click_hides_every_part(self) -> None:
        widget = self._widget_with_enabled_toggles()
        with (
            patch.object(
                tw, "find_all_model_roots_from_selection", return_value=["|A_Root"]
            ),
            patch.object(tw, "set_all_parts_visibility", return_value=4) as set_all,
            patch.object(
                tw, "get_part_visibility", return_value=dict.fromkeys(mv.PARTS, False)
            ),
        ):
            widget.visibility_panel.master_button.click()  # all visible -> hide all

        set_all.assert_called_once_with("|A_Root", False)
        self.assertEqual(
            widget.visibility_panel.master_button.text(), vp.SHOW_ALL_LABEL
        )

    def test_master_click_shows_every_part(self) -> None:
        widget = self._make_widget()
        with patch.object(
            tw, "get_part_visibility", return_value=dict.fromkeys(mv.PARTS, False)
        ):
            widget._refresh_visibility_buttons()
        self.assertEqual(
            widget.visibility_panel.master_button.text(), vp.SHOW_ALL_LABEL
        )

        with (
            patch.object(
                tw, "find_all_model_roots_from_selection", return_value=["|A_Root"]
            ),
            patch.object(tw, "reveal_all", return_value=2) as reveal,
            patch.object(
                tw, "get_part_visibility", return_value=dict.fromkeys(mv.PARTS, True)
            ),
        ):
            widget.visibility_panel.master_button.click()

        reveal.assert_called_once_with("|A_Root")
        self.assertEqual(
            widget.visibility_panel.master_button.text(), vp.HIDE_ALL_LABEL
        )

    def test_refresh_passes_individually_hidden_counts_to_the_panel(self) -> None:
        widget = self._make_widget()
        counts = {**dict.fromkeys(mv.PARTS, 0), mv.BONES: 2}
        with (
            patch.object(
                tw, "get_part_visibility", return_value=dict.fromkeys(mv.PARTS, True)
            ),
            patch.object(tw, "hidden_descendant_counts", return_value=counts),
        ):
            widget._refresh_visibility_buttons()

        self.assertIn(
            "2 nodes hidden individually",
            widget.visibility_panel.buttons[mv.BONES].toolTip(),
        )
