# SPDX-License-Identifier: MIT
"""
MayaMMD — Maya plugin implementation.

This module contains the actual plugin logic (initialization, commands, UI).
It is loaded via the C++ native entry point ``MayaMMD.mll``, which calls
:func:`initializePlugin` after registering its C++ commands/nodes.

The plugin directory is computed from this module's own path::

    mmd/plugin.py   →   ../   (the plug-ins / module root)
"""

from __future__ import annotations

import gc
import importlib
import logging
import os
import sys
import traceback

import maya.api.OpenMaya as om
from maya import cmds, mel

# ── UI imports are lazy — only imported when GUI is available ────────────
# shiboken6, maya.OpenMayaUI, and PySide6.QtWidgets are only needed for
# the dockable widget.  Deferring imports speeds up plugin loading in
# headless / batch / CI mode where the UI is never created.
from mmd.maya.cmds.bone_blend_shape_cmd import BoneBlendShapeCmd
from mmd.maya.nodes.bone_morph_node import BoneMorphNode
from mmd.ui.tool_main_widget import MMMToolWidget

# Dev mode detection — enabled when running from source (``.py`` files).
# Disable with the MAYAMMD_DEV=0 environment variable.
_DEV_MODE = os.environ.get("MAYAMMD_DEV", "1") == "1"

# ---------------------------------------------------------------------------
# Plugin metadata
# ---------------------------------------------------------------------------
try:
    from mmd._version import __version__ as _mmd_version

    PLUGIN_VERSION = _mmd_version
except ImportError:
    PLUGIN_VERSION = "0.0.0+dev"

PLUGIN_NAME = "MayaMMD"
PLUGIN_AUTHOR = "Sebastian Starosz"

log = logging.getLogger(PLUGIN_NAME)


# Tracked so uninitializePlugin can break references before deleting UI.
_mmd_widget: MMMToolWidget | None = None

_MMD_MODULES = [
    "mmd.core.binary_reader",
    "mmd.core.data_types",
    "mmd.core.pmx_importer",
    "mmd.core.pmx_reader",
    "mmd.core.pmx_validate",
    "mmd.core.vmd_importer",
    "mmd.core.vpd_importer",
    "mmd.maya.maya_data_types",
    "mmd.maya.pmx.rigid_body_builder",
    "mmd.maya.pmx_scene_builder",
    "mmd.maya.pmx_model_utils",
    "mmd.maya.vmd_scene_builder",
    "mmd.maya.vpd_scene_builder",
    "mmd.maya.pmx_naming_manager",
    "mmd.maya.pmx.bone_builder",
    "mmd.maya.pmx.morph_builder",
    "mmd.maya.nodes.bone_morph_node",
    "mmd.maya.cmds.bone_blend_shape_cmd",
    "mmd.ui.morph_tree_widget",
    "mmd.ui.tool_main_widget",
]


# ---------------------------------------------------------------------------
# Module reloading
# ---------------------------------------------------------------------------
def _reload_modules() -> int:
    """Import (or reload) all ``mmd.*`` modules.

    In dev mode, prints per-module status.
    In release mode, runs silently — failures are logged.

    UI modules (``mmd.ui.*``) are skipped when running in headless / offscreen
    mode (``QT_QPA_PLATFORM=offscreen``), since no GUI will ever be created.
    """
    _headless = os.environ.get("QT_QPA_PLATFORM") == "offscreen"
    success = 0
    for module_name in _MMD_MODULES:
        if _headless and module_name.startswith("mmd.ui."):
            success += 1  # intentionally skipped in headless mode — count as OK
            if _DEV_MODE:
                log.debug("  %-45s SKIPPED (headless)", module_name)
            continue
        try:
            if module_name in sys.modules:
                importlib.reload(sys.modules[module_name])
            else:
                importlib.import_module(module_name)
            success += 1
            if _DEV_MODE:
                log.debug("  %-45s OK", module_name)
        except Exception as e:
            log.error("  %-45s FAILED — %s", module_name, e)
    return success


# ---------------------------------------------------------------------------
# Dev utilities (always available — call from Script Editor if needed)
# ---------------------------------------------------------------------------
def verify_plugin_modules() -> None:
    """Print loaded module status for debugging."""
    log.info("%s", "=" * 60)
    log.info("PLUGIN MODULE VERIFICATION")
    log.info("%s", "=" * 60)
    loaded = 0
    unloaded = 0
    for name in _MMD_MODULES:
        if name in sys.modules:
            m = sys.modules[name]
            f = getattr(m, "__file__", "?")
            log.info("  [OK] %-45s %s", name, f)
            loaded += 1
        else:
            log.info("  [--] %-45s (not loaded)", name)
            unloaded += 1
    log.info("Loaded: %d, Not loaded: %d", loaded, unloaded)
    log.info("%s", "=" * 60)


# ---------------------------------------------------------------------------
# Shared widget cleanup helper
# ---------------------------------------------------------------------------
def _cleanup_widget() -> None:
    """Safely tear down the current :class:`MMMToolWidget` instance.

    Called when the user closes the UI tab, when opening a new instance
    (replacing the old one), and when the plugin unloads.  Safe to call
    multiple times.
    """
    global _mmd_widget
    if _mmd_widget is None:
        return
    # 1. Remove Maya API 2.0 selection callback
    _mmd_widget._stop_selection_listener()
    # 2. Disconnect the model-changed signal (pure-Python QObject — no
    #    C++ wrapper to check, just guard against double-disconnect)
    try:
        if _mmd_widget.ctx is not None:
            _mmd_widget.ctx.modelChanged.disconnect(_mmd_widget._on_model_changed)
    except (RuntimeError, TypeError):
        pass
    # 3. Release the reference so the Python wrapper + its ModelContext
    #    can be garbage-collected promptly
    _mmd_widget = None


# ---------------------------------------------------------------------------
# Dockable UI
# ---------------------------------------------------------------------------
def show_dockable_widget() -> None:
    """Create (or re-create) the MayaMMD dockable workspace control."""
    # Lazy-import UI modules — they are only needed here.
    import shiboken6
    from maya import OpenMayaUI as omui
    from PySide6 import QtWidgets

    # Tear down any previous instance cleanly before building a new one
    _cleanup_widget()

    workspace_control_name = "MayaMMDMenuWorkspaceControl"
    if cmds.workspaceControl(workspace_control_name, exists=True, query=True):
        try:
            cmds.deleteUI(workspace_control_name)
        except Exception:
            pass

    cmds.workspaceControl(
        workspace_control_name,
        label="MayaMMD",
        dockToMainWindow=("right", True),
        retain=False,
        visible=True,
        initialWidth=400,
        initialHeight=600,
    )

    control_ptr = omui.MQtUtil.findControl(workspace_control_name)
    if control_ptr is None:
        msg = (
            "Could not create MayaMMD panel. "
            "Try restarting Maya or run 'MayaMMD()' in the Script Editor."
        )
        log.warning(msg)
        print(f"[WARN] {msg}")
        return
    control_widget = shiboken6.wrapInstance(int(control_ptr), QtWidgets.QWidget)

    global _mmd_widget
    _mmd_widget = widget = MMMToolWidget(parent=control_widget)
    control_layout = control_widget.layout()
    if control_layout is not None:
        control_layout.addWidget(widget)
    else:
        layout = QtWidgets.QVBoxLayout(control_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(widget)

    cmds.workspaceControl(workspace_control_name, edit=True, visible=True)
    cmds.workspaceControl(workspace_control_name, edit=True, restore=True)
    try:
        cmds.workspaceControl(workspace_control_name, edit=True, r=True)
    except Exception:
        pass

    print(f"[OK] MayaMMD v{PLUGIN_VERSION} - panel opened on the RIGHT side of Maya")


# ---------------------------------------------------------------------------
# Shelf
# ---------------------------------------------------------------------------
_MMD_SHELF_LAYOUT = "MayaMMDShelf"
# Shelf button icon — ships inside the mmd package (mmd/icons/), so it works
# both from the source tree and from the installed module (plug-ins/mmd/icons/).
_MMD_SHELF_ICON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "icons", "MayaMMD.png"
)
_MMD_SHELF_ICON_FALLBACK = "pythonFamily.png"


def _add_shelf_button() -> None:
    """Create (or switch to) the MayaMMD shelf and add a launch button.

    Silently skipped in batch/standalone mode (no UI available).
    """
    # In batch/standalone mode the shelf MEL global does not exist.
    # Pre-check with cmds.about to avoid MEL eval error noise on stderr.
    try:
        if cmds.about(batch=True):
            return
        shelf = mel.eval("$gShelfTopLevel = $gShelfTopLevel")
    except Exception:
        return

    is_new = not cmds.shelfLayout(_MMD_SHELF_LAYOUT, exists=True)
    if is_new:
        cmds.shelfLayout(_MMD_SHELF_LAYOUT, parent=shelf)
    cmds.tabLayout(shelf, edit=True, tabLabel=[(_MMD_SHELF_LAYOUT, "MayaMMD")])
    # Only switch to the new tab when creating it for the first time.
    # On subsequent plugin loads the user's current shelf tab is preserved.
    if is_new:
        cmds.tabLayout(shelf, edit=True, selectTab=_MMD_SHELF_LAYOUT)

    # Remove any existing MMD buttons from the shelf to prevent duplicates.
    # (Buttons can survive in persisted shelf files across restarts.)
    children = cmds.shelfLayout(_MMD_SHELF_LAYOUT, query=True, childArray=True) or []
    for child in children:
        if cmds.shelfButton(child, query=True, label=True) == "MMD":
            cmds.deleteUI(child)

    # When installed as a Maya Module, the plug-ins/ directory is on
    # MAYA_PLUG_IN_PATH (and thus sys.path) automatically, so no
    # sys.path manipulation is needed here.
    command = "MayaMMD()"

    try:
        cmds.shelfButton(
            command=command,
            annotation="MayaMMD — Import and animate MMD models (PMX/VMD/VPD)",
            label="MMD",
            sourceType="mel",
            image=_MMD_SHELF_ICON
            if os.path.exists(_MMD_SHELF_ICON)
            else _MMD_SHELF_ICON_FALLBACK,
            parent=_MMD_SHELF_LAYOUT,
        )
        log.debug("MayaMMD shelf created")
    except Exception as e:
        log.warning("Could not create shelf button: %s", e)


def _remove_shelf_button() -> None:
    """Remove the entire MayaMMD shelf tab (button + tab)."""
    if cmds.shelfLayout(_MMD_SHELF_LAYOUT, exists=True):
        try:
            cmds.deleteUI(_MMD_SHELF_LAYOUT)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
class MayaMMDCmd(om.MPxCommand):
    """Maya command that opens the MayaMMD dockable UI."""

    def __init__(self) -> None:
        om.MPxCommand.__init__(self)

    def doIt(self, args: om.MArgList) -> None:
        show_dockable_widget()

    @staticmethod
    def cmdCreator() -> om.MPxCommand:
        return MayaMMDCmd()


# ---------------------------------------------------------------------------
# C++ .mll entry point helpers
#
# These are called from the C++ entry point via MGlobal::executePythonCommand.
# They use API 2.0 findPlugin to get the plugin MObject (registered by
# the C++ initializePlugin), so everything registers under the same
# "MayaMMD" identity — one entry in the Plugin Manager.
# ---------------------------------------------------------------------------


def _register_ae_template_path() -> None:
    """Make the Attribute Editor find the shipped AE XML templates.

    The AE reads ``MAYA_CUSTOM_TEMPLATE_PATH`` (showEditor.mel: templateDirs)
    every time it builds a node's editor, so appending the templates folder
    at plugin load works even when ``MayaMMD.mod`` is not on
    ``MAYA_MODULE_PATH`` (e.g. the plugin was loaded via
    ``MAYA_PLUG_IN_PATH`` or the shelf).  Candidate folders, in order:

      1. relative to the loaded .mll  → ``<module>/scripts/AETemplates``
      2. dev-from-source fallback     → ``<repo>/scripts/ae``
    """
    candidates: list[str] = []
    try:
        mll = cmds.pluginInfo(PLUGIN_NAME, query=True, path=True)
        if mll:
            candidates.append(
                os.path.normpath(
                    os.path.join(os.path.dirname(mll), "..", "scripts", "AETemplates")
                )
            )
    except Exception:  # noqa: BLE001, S110 - pluginInfo may be unavailable headless
        pass
    candidates.append(
        os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "scripts", "ae"))
    )

    for folder in candidates:
        if not os.path.isdir(folder):
            continue
        # mel putenv / getenv use the process env; forward slashes avoid MEL
        # backslash escaping in the putenv value.
        folder_fwd = folder.replace("\\", "/")
        current = mel.eval("getenv MAYA_CUSTOM_TEMPLATE_PATH") or ""
        existing = [p.strip() for p in current.split(";") if p.strip()]
        if folder_fwd in existing:
            return
        merged = ";".join([folder_fwd] + existing)
        mel.eval(f'putenv "MAYA_CUSTOM_TEMPLATE_PATH" "{merged}"')
        log.debug("AE templates: %s", folder_fwd)
        # Make the grouped views the DEFAULT for both rigid-body node types.
        # The AE only builds a custom (XML) view when the per-type optionVar
        # names one — otherwise it builds the plain editor, which dumps the
        # attributes under "Extra Attributes".  Same optionVar the AE's Views
        # menu writes when the user picks a view.
        try:
            # Shape node: the grouped "Body" XML view stays the default.
            mel.eval('optionVar -sv AEpmxRigidBodyShapeCustomView "Body"')
            # Solver node: deliberately NO XML-view optionVar.  A named view
            # routes the AE to the (gravity-only) XML template, bypassing
            # createEditor — but the joints editor is the MEL template, which
            # the AE only invokes through createEditor when no custom view is
            # active.  Clear the legacy "Solver" value so the MEL template is
            # used.
            if mel.eval("optionVar -q AEpmxRigidBodyNodeCustomView") == "Solver":
                mel.eval("optionVar -rm AEpmxRigidBodyNodeCustomView")
        except Exception:
            log.debug("Could not set default AE views", exc_info=True)
        return
    log.debug("No AE template folder found")


# Classic MEL Attribute-Editor template for the solver node.  Maya's XML AE
# templates cannot render the ``joints`` compound-array (an empty control
# with no add button), so this node uses the standard MEL template instead:
# ``editorTemplate -addControl "joints"`` builds the DEFAULT control for the
# attribute, which for an array of compounds is a proper Multi (add/remove,
# expandable children) — no custom widget, and no reliance on Maya's
# internal Flux AE framework (whose custom-control layout overlapped the
# Extra Attributes section).  ``gravity`` gets its standard vector control.
# The proc is only invoked when the plugin clears the
# ``AEpmxRigidBodyNodeCustomView`` optionVar, so the AE takes the
# ``createEditor`` path (a named view would route it to the XML template
# instead).
_AE_SOLVER_TEMPLATE_MEL = """\
global proc AEpmxRigidBodyNodeTemplate( string $nodeName ) {
    editorTemplate -beginScrollLayout;
    editorTemplate -suppress "bodyShapes";
    editorTemplate -suppress "outTranslate";
    editorTemplate -suppress "outRotate";
    editorTemplate -suppress "outGuideTranslate";
    editorTemplate -suppress "outGuideRotate";
    editorTemplate -beginLayout "Solver" -collapse 0;
        editorTemplate -addControl "gravity";
    editorTemplate -endLayout;
    editorTemplate -beginLayout "Joints" -collapse 0;
        editorTemplate -addControl "joints";
    editorTemplate -endLayout;
    editorTemplate -addExtraControls;
    editorTemplate -endScrollLayout;
}
"""


def _register_ae_mel_templates() -> None:
    """Define the solver's MEL AE template (the ``joints`` Multi editor)."""
    mel.eval(_AE_SOLVER_TEMPLATE_MEL)


def initializePlugin() -> None:
    """Called by C++ .mll's initializePlugin after native C++ registration.

    Uses API 2.0 ``MFnPlugin.findPlugin`` to get the plugin MObject
    (registered by the C++ ``initializePlugin``), so everything registers
    under the same "MayaMMD" identity — one entry in the Plugin Manager.
    """
    # logging.basicConfig(
    #    level=logging.DEBUG if _DEV_MODE else logging.WARNING, force=True
    # )
    # logging.getLogger().setLevel(logging.DEBUG if _DEV_MODE else logging.WARNING)
    logging.getLogger().setLevel(logging.INFO)

    if _DEV_MODE:
        log.info("%s", "─" * 50)
        log.info("MayaMMD v%s (DEV MODE)", PLUGIN_VERSION)
        log.info("%s", "─" * 50)

    try:
        loaded = _reload_modules()
        if loaded < len(_MMD_MODULES):
            log.warning(
                "%d/%d modules loaded — some features may be unavailable",
                loaded,
                len(_MMD_MODULES),
            )

        # Attribute-Editor XML templates: make Maya find them no matter how
        # the plugin was loaded (module, MAYA_PLUG_IN_PATH or shelf), and
        # make the grouped "Body" view the default (see the function).
        try:
            _register_ae_template_path()
        except Exception:
            log.debug("Could not set MAYA_CUSTOM_TEMPLATE_PATH", exc_info=True)

        # Solver node: XML AE templates can't render the joints compound-array,
        # so define the classic MEL AE template (default Multi control).
        try:
            _register_ae_mel_templates()
        except Exception:
            log.debug("Could not register MEL AE template", exc_info=True)

        # Cleanup stale UI elements
        if cmds.menu("MayaMMDMenu", exists=True):
            cmds.deleteUI("MayaMMDMenu")
        workspace_control_name = "MayaMMDMenuWorkspaceControl"
        if cmds.workspaceControl(workspace_control_name, exists=True):
            cmds.deleteUI(workspace_control_name)

        # Get plugin MObject via API 2.0 findPlugin (same identity as C++ .mll).
        mobject = om.MFnPlugin.findPlugin(PLUGIN_NAME)
        if mobject.isNull():
            raise RuntimeError(f"Could not find plugin '{PLUGIN_NAME}' via findPlugin")

        mplugin = om.MFnPlugin(mobject, PLUGIN_AUTHOR, PLUGIN_VERSION, "Any")

        # Register custom nodes (API 2.0)
        mplugin.registerNode(
            "boneMorphNode",
            om.MTypeId(0x87000),
            BoneMorphNode.nodeCreator,
            BoneMorphNode.nodeInitializer,
            om.MPxNode.kDependNode,
            "utility/general",
        )

        # Register commands
        mplugin.registerCommand(PLUGIN_NAME, MayaMMDCmd.cmdCreator)
        mplugin.registerCommand(
            BoneBlendShapeCmd.kName,
            BoneBlendShapeCmd.cmdCreator,
            BoneBlendShapeCmd.syntaxCreator,
        )

        # Shelf (UI opens on demand via shelf button, not at startup)
        # Wrap in try/except — shelf creation requires interactive Maya GUI
        # and will fail in standalone/batch mode.
        try:
            _add_shelf_button()
        except Exception as shelf_err:
            log.debug("Shelf creation skipped (expected in batch mode): %s", shelf_err)

        # User-facing summary (always printed, regardless of log level)
        print(f"[OK] MayaMMD v{PLUGIN_VERSION} ready")
        print("  Click the 'MMD' shelf button or run 'MayaMMD()' to open the UI.")

    except Exception:
        log.error("Failed to initialize %s", PLUGIN_NAME)
        traceback.print_exc()
        print("[FAIL] MayaMMD failed to load — see Script Editor for details.")


def uninitializePlugin() -> None:
    """Called by C++ .mll's uninitializePlugin before native C++ deregistration."""
    log.debug("Unloading %s ...", PLUGIN_NAME)

    try:
        _remove_shelf_button()

        # Tear down the widget cleanly, then delete Maya UI wrappers.
        _cleanup_widget()

        if cmds.workspaceControl("MayaMMDMenuWorkspaceControl", exists=True):
            cmds.deleteUI("MayaMMDMenuWorkspaceControl")
        if cmds.menu("MayaMMDMenu", exists=True):
            cmds.deleteUI("MayaMMDMenu")

        gc.collect()

        mobject = om.MFnPlugin.findPlugin(PLUGIN_NAME)
        if not mobject.isNull():
            mplugin = om.MFnPlugin(mobject)
            mplugin.deregisterCommand(PLUGIN_NAME)
            mplugin.deregisterCommand(BoneBlendShapeCmd.kName)
            mplugin.deregisterNode(om.MTypeId(0x87000))

        log.debug("✓ %s unloaded", PLUGIN_NAME)
        print(f"  MayaMMD v{PLUGIN_VERSION} unloaded.")
    except Exception:
        log.error("Failed to unload %s", PLUGIN_NAME)
        traceback.print_exc()
