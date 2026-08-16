"""
Pure-function unit tests for ``MorphTreeWidget`` (no Maya, no Qt).

Tests methods that have zero external dependencies — just string
manipulation and data transformation.
"""

from __future__ import annotations

import unittest

# ── Install Maya stub (for import passthrough) ─────────────────────────
from tests.unit_tests.maya.maya_stub import install_maya_stub

install_maya_stub(profile="headless")

from mmd.ui.morph_tree_widget import KeyframeState, MorphTreeWidget


class TestModelDisplayName(unittest.TestCase):
    """Tests for ``MorphTreeWidget._model_display_name``."""

    def test_simple_node_name(self) -> None:
        """A plain node name is returned as-is."""
        result = MorphTreeWidget._model_display_name("MyModel")
        self.assertEqual(result, "MyModel")

    def test_strips_blendshape_suffix(self) -> None:
        """The ``_BlendShape`` suffix is removed."""
        result = MorphTreeWidget._model_display_name("Model_BlendShape")
        self.assertEqual(result, "Model")

    def test_strips_bone_morph_suffix(self) -> None:
        """The ``_BoneMorph`` suffix is removed."""
        result = MorphTreeWidget._model_display_name("Model_BoneMorph")
        self.assertEqual(result, "Model")

    def test_strips_root_suffix(self) -> None:
        """The ``_Root`` suffix is removed."""
        result = MorphTreeWidget._model_display_name("Model_Root")
        self.assertEqual(result, "Model")

    def test_dag_path_handling(self) -> None:
        """Full DAG paths are shortened to the leaf name."""
        result = MorphTreeWidget._model_display_name("|root|group|Character_BlendShape")
        self.assertEqual(result, "Character")

    def test_dag_root_path(self) -> None:
        """Root transform DAG path strips path and ``_Root`` suffix."""
        result = MorphTreeWidget._model_display_name("|group|HatsuneMiku_Root")
        self.assertEqual(result, "HatsuneMiku")

    def test_empty_string(self) -> None:
        """Empty string is returned as empty."""
        result = MorphTreeWidget._model_display_name("")
        self.assertEqual(result, "")

    def test_only_suffix(self) -> None:
        """A string that is only a known suffix is returned empty."""
        result = MorphTreeWidget._model_display_name("_BlendShape")
        self.assertEqual(result, "")

    def test_double_suffix_only_last_is_stripped(self) -> None:
        """removesuffix only strips from the end; first suffix is left intact."""
        result = MorphTreeWidget._model_display_name("Model_BlendShape_BoneMorph")
        # "Model_BlendShape_BoneMorph" → strip _BoneMorph → "Model_BlendShape"
        self.assertEqual(result, "Model_BlendShape")

    def test_triple_suffix_only_last_stripped(self) -> None:
        """Only the rightmost suffix is removed; earlier ones remain."""
        result = MorphTreeWidget._model_display_name("Model_Root_BlendShape_BoneMorph")
        # → strip _BoneMorph → "Model_Root_BlendShape"
        self.assertEqual(result, "Model_Root_BlendShape")

    def test_suffix_embedded_in_middle_not_stripped(self) -> None:
        """Suffixes are only stripped from the end, not mid-name."""
        result = MorphTreeWidget._model_display_name("Model_BlendShape_v2")
        self.assertEqual(result, "Model_BlendShape_v2")


class TestKeyframeStateConstants(unittest.TestCase):
    """Validate the keyframe state enum values."""

    def test_constants_are_distinct(self) -> None:
        """KeyframeState enum values are distinct."""
        self.assertEqual(KeyframeState.NONE, 0)
        self.assertEqual(KeyframeState.BETWEEN, 1)
        self.assertEqual(KeyframeState.AT_CURRENT, 2)
        self.assertNotEqual(KeyframeState.NONE, KeyframeState.BETWEEN)
        self.assertNotEqual(KeyframeState.NONE, KeyframeState.AT_CURRENT)
        self.assertNotEqual(
            KeyframeState.BETWEEN,
            KeyframeState.AT_CURRENT,
        )


if __name__ == "__main__":
    unittest.main()
