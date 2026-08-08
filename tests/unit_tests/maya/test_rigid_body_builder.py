"""
test_rigid_body_builder.py

Unit tests for the PURE logic in mmd.maya.pmx.rigid_body_builder.

This file tests only functions that don't depend on Maya's runtime (the Maya
stub installed by the conftest makes the module importable outside mayapy).
Maya-dependent build functions (guide creation, constraints, node wiring) are
tested in the integration suite that runs inside Maya.
"""

import pytest

from mmd.maya.pmx.rigid_body_builder import (
    _DEFAULT_FPS,
    _scene_fps,
    _time_unit_to_fps,
)


class TestTimeUnitToFps:
    def test_named_units(self):
        assert _time_unit_to_fps("film") == 24.0
        assert _time_unit_to_fps("game") == 30.0
        assert _time_unit_to_fps("ntsc") == 30.0
        assert _time_unit_to_fps("pal") == 25.0
        assert _time_unit_to_fps("show") == 48.0
        assert _time_unit_to_fps("palf") == 50.0
        assert _time_unit_to_fps("ntscf") == 60.0

    def test_custom_fps_units(self):
        assert _time_unit_to_fps("30fps") == 30.0
        assert _time_unit_to_fps("23.976fps") == pytest.approx(23.976)
        assert _time_unit_to_fps("60fps") == 60.0
        assert _time_unit_to_fps("240fps") == 240.0

    def test_unresolvable_units(self):
        assert _time_unit_to_fps(None) is None
        assert _time_unit_to_fps("") is None
        assert _time_unit_to_fps("bogus") is None
        assert _time_unit_to_fps("12fpsx") is None
        assert _time_unit_to_fps("abc") is None


class TestSceneFps:
    def test_named_scene_unit(self, monkeypatch):
        import mmd.maya.pmx.rigid_body_builder as rb

        monkeypatch.setattr(rb.cmds, "currentUnit", lambda **kwargs: "film")
        assert _scene_fps() == 24.0

    def test_custom_scene_unit(self, monkeypatch):
        import mmd.maya.pmx.rigid_body_builder as rb

        monkeypatch.setattr(rb.cmds, "currentUnit", lambda **kwargs: "60fps")
        assert _scene_fps() == 60.0

    def test_falls_back_when_unit_unresolvable(self, monkeypatch):
        import mmd.maya.pmx.rigid_body_builder as rb

        monkeypatch.setattr(rb.cmds, "currentUnit", lambda **kwargs: "bogus")
        assert _scene_fps() == _DEFAULT_FPS

    def test_falls_back_when_query_fails(self, monkeypatch):
        import mmd.maya.pmx.rigid_body_builder as rb

        def _boom(**kwargs):
            raise RuntimeError("no time unit")

        monkeypatch.setattr(rb.cmds, "currentUnit", _boom)
        assert _scene_fps() == _DEFAULT_FPS
