"""
Unit tests for ``_AbstractMorphSource`` subclasses
(``_BlendShapeMorphSource``, ``_BoneMorphSource``).

These tests verify that the morph-source abstraction correctly queries
and formats Maya data — without requiring an actual Maya runtime.
All Maya calls are mocked via ``unittest.mock``.
"""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

# ── Install Maya stub (for import passthrough) ─────────────────────────
from tests.unit_tests.maya.maya_stub import install_maya_stub

install_maya_stub(profile="headless")

# We import the module under test, then reference classes via it.
from mmd.ui import morph_tree_widget as _mod


class TestBlendShapeMorphSource(unittest.TestCase):
    """Tests for ``_BlendShapeMorphSource``."""

    def setUp(self) -> None:
        self.source = _mod._BlendShapeMorphSource("testBS")

    def test_section_label(self) -> None:
        self.assertEqual(self.source.section_label(), "Vertex")

    def test_node_name(self) -> None:
        self.assertEqual(self.source.node_name, "testBS")

    def test_get_weight_attr(self) -> None:
        self.assertEqual(self.source.get_weight_attr("smile"), "testBS.smile")

    def test_get_edit_action_label(self) -> None:
        self.assertEqual(self.source.get_edit_action_label(), "Edit")

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_targets_empty(self, mock_cmds: MagicMock) -> None:
        """When the node doesn't exist, return empty list."""
        mock_cmds.objExists.return_value = False
        result = self.source.get_targets()
        self.assertEqual(result, [])

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_targets_with_data(self, mock_cmds: MagicMock) -> None:
        """Targets are extracted from aliasAttr + multiIndices."""
        mock_cmds.objExists.return_value = True
        # aliasAttr returns [alias1, attr1, alias2, attr2, …]
        mock_cmds.aliasAttr.return_value = [
            "smile",
            "weight[0]",
            "frown",
            "weight[1]",
            "neutral",
            "weight[2]",
        ]
        # getAttr multiIndices returns weight indices in natural order
        mock_cmds.getAttr.return_value = [0, 1, 2]

        result = self.source.get_targets()
        self.assertEqual(result, ["smile", "frown", "neutral"])

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_targets_indices_out_of_order(self, mock_cmds: MagicMock) -> None:
        """Targets are returned in the order given by multiIndices."""
        mock_cmds.objExists.return_value = True
        mock_cmds.aliasAttr.return_value = [
            "A",
            "weight[0]",
            "B",
            "weight[1]",
            "C",
            "weight[2]",
        ]
        # multiIndices returns whatever order Maya stores them in
        mock_cmds.getAttr.return_value = [2, 0, 1]

        result = self.source.get_targets()
        self.assertEqual(result, ["C", "A", "B"])

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_envelope_exists(self, mock_cmds: MagicMock) -> None:
        """When envelope attribute exists, return its plug path."""
        mock_cmds.objExists.return_value = True
        mock_cmds.attributeQuery.return_value = True

        result = self.source.get_envelope_attr()
        self.assertEqual(result, "testBS.envelope")

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_envelope_missing(self, mock_cmds: MagicMock) -> None:
        """When envelope attribute doesn't exist, return None."""
        mock_cmds.objExists.return_value = True
        mock_cmds.attributeQuery.return_value = False

        result = self.source.get_envelope_attr()
        self.assertIsNone(result)

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_envelope_node_missing(self, mock_cmds: MagicMock) -> None:
        """When the node itself doesn't exist, return None."""
        mock_cmds.objExists.return_value = False

        result = self.source.get_envelope_attr()
        self.assertIsNone(result)

    @patch("mmd.ui.morph_tree_widget.mel")
    def test_on_edit_clicked_opens_shape_editor(self, mock_mel: MagicMock) -> None:
        """Clicking Edit calls mel.eval('ShapeEditor;')."""
        self.source.on_edit_clicked("smile")
        mock_mel.eval.assert_called_once_with("ShapeEditor;")


class TestBoneMorphSource(unittest.TestCase):
    """Tests for ``_BoneMorphSource``."""

    def setUp(self) -> None:
        self.source = _mod._BoneMorphSource("testBoneMorph")

    def test_section_label(self) -> None:
        self.assertEqual(self.source.section_label(), "Bone")

    def test_node_name(self) -> None:
        self.assertEqual(self.source.node_name, "testBoneMorph")

    def test_get_weight_attr(self) -> None:
        self.assertEqual(self.source.get_weight_attr("pose1"), "testBoneMorph.pose1")

    def test_get_edit_action_label(self) -> None:
        """Bone morphs show a disabled Edit button."""
        self.assertEqual(self.source.get_edit_action_label(), "Edit")

    def test_on_edit_clicked_noop(self) -> None:
        """Default on_edit_clicked should be a no-op."""
        # Should not raise
        self.source.on_edit_clicked("pose1")

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_targets_empty(self, mock_cmds: MagicMock) -> None:
        """When the node doesn't exist, return empty list."""
        mock_cmds.objExists.return_value = False
        result = self.source.get_targets()
        self.assertEqual(result, [])

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_targets_with_data(self, mock_cmds: MagicMock) -> None:
        """Targets are extracted from the boneBlendShape command."""
        mock_cmds.objExists.return_value = True
        mock_cmds.boneBlendShape.return_value = ["poseA", "poseB", "poseC"]

        result = self.source.get_targets()
        self.assertEqual(result, ["poseA", "poseB", "poseC"])

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_targets_bone_blend_shape_error(self, mock_cmds: MagicMock) -> None:
        """If boneBlendShape raises, return empty list."""
        mock_cmds.objExists.return_value = True
        mock_cmds.boneBlendShape.side_effect = RuntimeError("boom")

        result = self.source.get_targets()
        self.assertEqual(result, [])

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_envelope_exists(self, mock_cmds: MagicMock) -> None:
        """When envelope attribute exists, return its plug path."""
        mock_cmds.objExists.return_value = True
        mock_cmds.attributeQuery.return_value = True

        result = self.source.get_envelope_attr()
        self.assertEqual(result, "testBoneMorph.envelope")

    @patch("mmd.ui.morph_tree_widget.cmds")
    def test_get_envelope_missing(self, mock_cmds: MagicMock) -> None:
        """When envelope attribute doesn't exist, return None."""
        mock_cmds.objExists.return_value = True
        mock_cmds.attributeQuery.return_value = False

        result = self.source.get_envelope_attr()
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
