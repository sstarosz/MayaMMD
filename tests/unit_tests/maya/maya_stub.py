"""
Stub ``maya`` modules to enable Maya-independent unit tests.

Some Maya-independent logic (e.g. morph source queries, callback state
machines) does not itself call ``maya.cmds`` at runtime, but importing
the module under test triggers ``from maya import cmds`` — which fails
with ``ModuleNotFoundError`` outside mayapy.

``install_maya_stub()`` registers ``maya``, ``maya.cmds``, ``maya.mel``,
``maya.api.OpenMaya`` etc. as ``MagicMock``-based dummy modules in
``sys.modules``.  This allows *importing* Maya-touching modules in pure
Python (no Maya runtime), while unit tests inject their own return values
via ``configure_mock`` helpers.

Key assumptions:
- This stub is for **import passthrough** only — it does not emulate
  Maya API behaviour.  Tests must configure mock return values.
- If real ``maya`` is already in ``sys.modules`` (mayapy), this is a
  no-op, so it does not pollute integration tests.

Usage (at the top of the test module, BEFORE the import under test)::

    from tests.unit_tests.maya.maya_stub import install_maya_stub
    install_maya_stub(profile="headless")

    from mmd.ui.morph_tree_widget import MorphTreeWidget
"""

import sys
from types import ModuleType
from typing import Optional
from unittest.mock import MagicMock


# ── Module names registered by install_maya_stub ───────────────────────────
_STUBBED_NAMES = (
    "maya",
    "maya.cmds",
    "maya.mel",
    "maya.OpenMaya",
    "maya.OpenMayaMPx",
    "maya.api",
    "maya.api.OpenMaya",
    "maya.api.OpenMayaAnim",
    "maya.api.OpenMayaRender",
    "maya.api.OpenMayaUI",
)

# ── cmds methods managed by named profiles ─────────────────────────────────
_CMDS_PROFILE_METHODS = (
    "loadPlugin",
    "namespace",
    "namespaceInfo",
    "ls",
    "listRelatives",
    "listConnections",
    "objExists",
    "attributeQuery",
    "aliasAttr",
    "getAttr",
    "setAttr",
    "listHistory",
    "nodeType",
    "currentTime",
    "keyframe",
    "boneBlendShape",
    "file",
    "pluginInfo",
)


def _reset_cmds_profile_methods(cmds: MagicMock) -> None:
    """Reset methods managed by named profiles to plain MagicMock children."""
    for name in _CMDS_PROFILE_METHODS:
        setattr(cmds, name, MagicMock(name=f"maya.cmds.{name}"))


def _configure_cmds_headless_profile(cmds: MagicMock) -> None:
    """Apply headless-safe defaults for common Maya query commands.

    Bare ``MagicMock`` results are truthy and record every chained call.
    Code that probes Maya state in a loop can otherwise grow memory
    abruptly in pure Python tests.
    """
    _reset_cmds_profile_methods(cmds)

    def _namespace(*_args, **kwargs):
        if "exists" in kwargs:
            return False
        if "set" in kwargs or "add" in kwargs or "removeNamespace" in kwargs:
            return None
        return None

    def _namespace_info(*_args, **kwargs):
        if kwargs.get("currentNamespace"):
            return ":"
        if kwargs.get("listOnlyNamespaces"):
            return []
        return None

    cmds.namespace.side_effect = _namespace
    cmds.namespaceInfo.side_effect = _namespace_info
    cmds.ls.return_value = []
    cmds.listRelatives.return_value = []
    cmds.listConnections.return_value = []
    cmds.objExists.return_value = False
    cmds.attributeQuery.return_value = False
    cmds.aliasAttr.return_value = []
    cmds.getAttr.return_value = 0.0
    cmds.setAttr.return_value = None
    cmds.listHistory.return_value = []
    cmds.currentTime.return_value = 1.0
    cmds.keyframe.return_value = []
    cmds.boneBlendShape.return_value = []
    cmds.pluginInfo.return_value = False


def _configure_cmds_minimal_profile(_cmds: MagicMock) -> None:
    """Leave ``maya.cmds`` as a plain MagicMock for import-only tests."""
    _reset_cmds_profile_methods(_cmds)


_CMDS_PROFILE_CONFIGURERS = {
    "headless": _configure_cmds_headless_profile,
    "minimal": _configure_cmds_minimal_profile,
}


def _configure_cmds_profile(cmds: MagicMock, profile: str) -> None:
    """Apply a named ``maya.cmds`` stub profile."""
    try:
        configure = _CMDS_PROFILE_CONFIGURERS[profile]
    except KeyError as exc:
        valid = ", ".join(sorted(_CMDS_PROFILE_CONFIGURERS))
        raise ValueError(
            f"Unknown Maya cmds stub profile '{profile}'. Expected one of: {valid}"
        ) from exc
    configure(cmds)


def _is_real_maya_present() -> bool:
    """Detect whether we are running inside a real Maya environment (mayapy).

    Real ``maya`` is a ``ModuleType`` whose ``cmds`` attribute is NOT a
    ``MagicMock``.  If ``maya`` is missing, or its ``cmds`` is a
    ``MagicMock``, we are outside Maya.
    """
    maya_mod = sys.modules.get("maya")
    if maya_mod is None:
        return False
    cmds = getattr(maya_mod, "cmds", None)
    if cmds is None or isinstance(cmds, MagicMock):
        return False
    return True


def install_maya_stub(profile: Optional[str] = None) -> bool:
    """Register stub ``maya`` modules in ``sys.modules``.

    Args:
        profile: Default behaviour for ``maya.cmds``.  If not specified
            and the stub is being created fresh, ``"minimal"`` is used
            (import passthrough only).  ``"headless"`` applies query-safe
            defaults.  If the stub already exists and *profile* is
            ``None``, this is a no-op.

    Returns:
        ``True`` if the stub was newly registered, ``False`` if real
        Maya was already present and nothing was done.
    """
    if _is_real_maya_present():
        return False

    # Already stubbed → idempotent (reconfigure profile if requested)
    if isinstance(sys.modules.get("maya"), ModuleType) and isinstance(
        getattr(sys.modules.get("maya"), "cmds", None), MagicMock
    ):
        if profile is not None:
            _configure_cmds_profile(sys.modules["maya"].cmds, profile)
        return True

    maya = ModuleType("maya")
    maya.cmds = MagicMock(name="maya.cmds")
    _configure_cmds_profile(maya.cmds, profile or "minimal")
    maya.mel = MagicMock(name="maya.mel")

    # ── OpenMaya (API 1.0) ─────────────────────────────────────────────
    maya.OpenMaya = MagicMock(name="maya.OpenMaya")

    # ── OpenMayaMPx ────────────────────────────────────────────────────
    class _StubMPxFileTranslator:
        kImportAccessMode = 0
        kOpenAccessMode = 1
        kReferenceAccessMode = 2
        kIsMyFileType = 0
        kCouldBeMyFileType = 1
        kNotMyFileType = 2

        def __init__(self, *args, **kwargs):
            pass

    open_maya_mpx = ModuleType("maya.OpenMayaMPx")
    open_maya_mpx.MPxFileTranslator = _StubMPxFileTranslator
    open_maya_mpx.MFnPlugin = MagicMock(name="maya.OpenMayaMPx.MFnPlugin")
    open_maya_mpx.asMPxPtr = MagicMock(
        name="maya.OpenMayaMPx.asMPxPtr", side_effect=lambda value: value
    )
    maya.OpenMayaMPx = open_maya_mpx

    # ── OpenMaya API 2.0 namespace ─────────────────────────────────────
    api = ModuleType("maya.api")

    # OpenMaya (API 2.0) — needs MagicMock so attribute access works,
    # but also needs specific enum-like classes for type-checking code.
    _api_om = MagicMock(name="maya.api.OpenMaya")

    # Enums / constants commonly referenced in Maya API 2.0 code
    _api_om.MFn = MagicMock(name="maya.api.OpenMaya.MFn")
    _api_om.MFn.kMesh = 0
    _api_om.MFn.kAnimCurve = 0
    _api_om.MFn.kPlusMinusAverage = 0

    # Core types (MagicMock instances for simple attribute access)
    _api_om.MObject = MagicMock(name="maya.api.OpenMaya.MObject")
    _api_om.MPlug = MagicMock(name="maya.api.OpenMaya.MPlug")
    _api_om.MSelectionList = MagicMock(name="maya.api.OpenMaya.MSelectionList")
    _api_om.MQuaternion = MagicMock(name="maya.api.OpenMaya.MQuaternion")
    _api_om.MEulerRotation = MagicMock(name="maya.api.OpenMaya.MEulerRotation")
    _api_om.MMatrix = MagicMock(name="maya.api.OpenMaya.MMatrix")
    _api_om.MTransformationMatrix = MagicMock(
        name="maya.api.OpenMaya.MTransformationMatrix"
    )
    _api_om.MVector = MagicMock(name="maya.api.OpenMaya.MVector")

    # Message types (for callback registration tests)
    class _StubMNodeMessage:
        kConnectionMade = 0x0001
        kConnectionBroken = 0x0002
        kAttributeEval = 0x0004

        @staticmethod
        def addAttributeChangedCallback(*_args, **_kwargs):
            return 1  # dummy callback id

        @staticmethod
        def removeCallback(*_args, **_kwargs):
            return None

    class _StubMEventMessage:
        @staticmethod
        def addEventCallback(*_args, **_kwargs):
            return 1  # dummy callback id

        @staticmethod
        def removeCallback(*_args, **_kwargs):
            return None

    _api_om.MNodeMessage = _StubMNodeMessage
    _api_om.MEventMessage = _StubMEventMessage

    api.OpenMaya = _api_om

    # OpenMayaAnim
    _api_oma = MagicMock(name="maya.api.OpenMayaAnim")
    _api_oma.MFnAnimCurve = MagicMock(name="maya.api.OpenMayaAnim.MFnAnimCurve")
    _api_oma.MFnIkJoint = MagicMock(name="maya.api.OpenMayaAnim.MFnIkJoint")
    api.OpenMayaAnim = _api_oma

    # OpenMayaRender
    api.OpenMayaRender = MagicMock(name="maya.api.OpenMayaRender")

    # OpenMayaUI
    api.OpenMayaUI = MagicMock(name="maya.api.OpenMayaUI")

    maya.api = api

    # ── Register in sys.modules ────────────────────────────────────────
    sys.modules["maya"] = maya
    sys.modules["maya.cmds"] = maya.cmds
    sys.modules["maya.mel"] = maya.mel
    sys.modules["maya.OpenMaya"] = maya.OpenMaya
    sys.modules["maya.OpenMayaMPx"] = maya.OpenMayaMPx
    sys.modules["maya.api"] = api
    sys.modules["maya.api.OpenMaya"] = api.OpenMaya
    sys.modules["maya.api.OpenMayaAnim"] = api.OpenMayaAnim
    sys.modules["maya.api.OpenMayaRender"] = api.OpenMayaRender
    sys.modules["maya.api.OpenMayaUI"] = api.OpenMayaUI

    return True


def remove_maya_stub() -> None:
    """Remove all stubbed Maya modules from ``sys.modules``.

    Useful for test teardown to restore a clean state.
    """
    for name in _STUBBED_NAMES:
        sys.modules.pop(name, None)
