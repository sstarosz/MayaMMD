"""
Unit tests for ``MorphTreeWidget`` static methods that call ``maya.cmds``.

These tests mock ``cmds`` to verify the logic of ``_get_keyframe_state``
without requiring a Maya runtime.
"""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

# ── Install Maya stub (for import passthrough) ─────────────────────────
from tests.unit_tests.maya.maya_stub import install_maya_stub

install_maya_stub(profile="headless")

from mmd.ui.morph_tree_widget import KeyframeState, MorphTreeWidget, _get_keyframe_state  # noqa: E402


class TestGetKeyframeState(unittest.TestCase):
    """Tests for ``_get_keyframe_state``."""

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_no_curve_returns_key_none(self, mock_cmds: MagicMock) -> None:
        """When no animCurve is connected, return KeyframeState.NONE."""
        mock_cmds.listConnections.return_value = None
        result = _get_keyframe_state("attr.path")
        self.assertEqual(result, KeyframeState.NONE)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_has_curve_no_keys(self, mock_cmds: MagicMock) -> None:
        """When a curve exists but has no keyframes, return KeyframeState.BETWEEN.

        The attribute is 'keyed' (a curve is connected), just not at the
        current time.  This is consistent with Maya's convention: a
        connected animCurve means the attribute is driven by animation.
        """
        mock_cmds.listConnections.return_value = ["animCurveTL1"]
        mock_cmds.currentTime.return_value = 10.0
        mock_cmds.keyframe.return_value = None  # no keys

        result = _get_keyframe_state("attr.path")
        self.assertEqual(result, KeyframeState.BETWEEN)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_key_at_current_time(self, mock_cmds: MagicMock) -> None:
        """When the current time matches a key, return KeyframeState.AT_CURRENT."""
        mock_cmds.listConnections.return_value = ["animCurveTL1"]
        mock_cmds.currentTime.return_value = 5.0
        mock_cmds.keyframe.return_value = [1.0, 5.0, 10.0]

        result = _get_keyframe_state("attr.path")
        self.assertEqual(result, KeyframeState.AT_CURRENT)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_key_between_keys(self, mock_cmds: MagicMock) -> None:
        """When the current time is between two keys, return KeyframeState.BETWEEN."""
        mock_cmds.listConnections.return_value = ["animCurveTL1"]
        mock_cmds.currentTime.return_value = 7.0
        mock_cmds.keyframe.return_value = [1.0, 10.0]

        result = _get_keyframe_state("attr.path")
        self.assertEqual(result, KeyframeState.BETWEEN)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_key_at_current_time_with_epsilon(self, mock_cmds: MagicMock) -> None:
        """Near-exact match within 0.001 counts as KeyframeState.AT_CURRENT."""
        mock_cmds.listConnections.return_value = ["animCurveTL1"]
        mock_cmds.currentTime.return_value = 5.0005
        mock_cmds.keyframe.return_value = [5.0]

        result = _get_keyframe_state("attr.path")
        self.assertEqual(result, KeyframeState.AT_CURRENT)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_key_just_outside_epsilon(self, mock_cmds: MagicMock) -> None:
        """A key 0.002 away is KeyframeState.BETWEEN, not AT_CURRENT."""
        mock_cmds.listConnections.return_value = ["animCurveTL1"]
        mock_cmds.currentTime.return_value = 5.002
        mock_cmds.keyframe.return_value = [5.0]

        result = _get_keyframe_state("attr.path")
        self.assertEqual(result, KeyframeState.BETWEEN)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_current_time_query_raises(self, mock_cmds: MagicMock) -> None:
        """If currentTime query fails, return KeyframeState.NONE."""
        mock_cmds.listConnections.return_value = ["animCurveTL1"]
        mock_cmds.currentTime.side_effect = RuntimeError("oops")

        result = _get_keyframe_state("attr.path")
        self.assertEqual(result, KeyframeState.NONE)


if __name__ == "__main__":
    unittest.main()
