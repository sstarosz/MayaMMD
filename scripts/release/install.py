"""
MayaMMD Installer

Drag-and-drop installer for MayaMMD Maya plugin (Maya Module).

Usage:
    1. Drag install.py onto the Maya viewport (Maya 2017+), or
    2. Run ``import install`` in Maya's Script Editor.

The installer will:
    - Detect the OS platform and Maya version.
    - Select the correct mmd/ source package from the zip.
    - Install as a Maya Module into the user's Maya ``modules/``
      directory, creating the proper module structure with
      ``plug-ins/``, ``scripts/``, and ``icons/`` subdirectories.
    - Write a ``MayaMMD.mod`` module description file.
    - Load the plugin.

Supported Maya versions: 2024, 2025, 2026, 2027
Supported platforms: Windows, Linux, macOS
"""

# ---------------------------------------------------------------------------
# MUST be before any other imports — prevents stale .pyc from shadowing
# updates to this file when it's dragged repeatedly into Maya.
# ---------------------------------------------------------------------------
import sys as _sys

_sys.dont_write_bytecode = True

# Nuke any previously-cached bytecode for this file so Python always
# reads the .py source fresh.
if __file__:
    import os as _os

    _pycache = _os.path.join(
        _os.path.dirname(_os.path.abspath(__file__)), "__pycache__"
    )
    if _os.path.isdir(_pycache):
        _basename = _os.path.splitext(_os.path.basename(__file__))[0]
        for _entry in _os.listdir(_pycache):
            if _entry.startswith(_basename + ".cpython-") and _entry.endswith(".pyc"):
                _pyc = _os.path.join(_pycache, _entry)
                try:
                    _os.remove(_pyc)
                except OSError:
                    pass

import gc
import os
import sys
import shutil
import time as _time

# ---------------------------------------------------------------------------
# Maya version → Python version mapping
# ---------------------------------------------------------------------------
# Maya 2024 = Python 3.10, 2025-2026 = Python 3.11, 2027 = Python 3.13
PLUGIN_NAME = "MayaMMD"

# Platform display names for user-friendly directory names
_PLATFORM_NAMES: dict[str, str] = {
    "win32": "Windows",
    "linux": "Linux",
    "darwin": "macOS",
}

# Maya version → display label for directory names
_MAYA_VERSION_LABEL: dict[int, str] = {
    2024: "2024",
    2025: "2025",
    2026: "2026",
    2027: "2027",
}


# ---------------------------------------------------------------------------
# Platform detection
# ---------------------------------------------------------------------------
def _get_platform_tag() -> str:
    """
    Return the sys.platform value used as subdirectory tag in the release zip.
    """
    return sys.platform  # 'win32', 'linux', 'darwin'


# ---------------------------------------------------------------------------
# Maya detection
# ---------------------------------------------------------------------------
def _is_maya() -> bool:
    """Check if running inside Maya."""
    try:
        import maya.cmds  # noqa: F401

        return True
    except ImportError:
        return False


def _get_maya_version() -> int:
    """
    Return the integer Maya version (e.g. 2024, 2025, 2026).
    Raises RuntimeError if Maya is not available.
    """
    if not _is_maya():
        raise RuntimeError("Not running inside Autodesk Maya")
    import maya.cmds as cmds

    version_str = cmds.about(version=True)
    # about() returns something like "2024.0" — extract the major year
    return int(float(version_str))


def _get_maya_base_dir(maya_version: int) -> str:
    """Return the user Maya base directory for the given Maya version.

    Platform paths (standard Maya install locations):
        Windows: ~/Documents/maya/<version>/
        Linux:   ~/maya/<version>/
        macOS:   ~/Library/Preferences/Autodesk/maya/<version>/
    """
    home = os.path.expanduser("~")
    if sys.platform == "win32":
        return os.path.join(home, "Documents", "maya", str(maya_version))
    elif sys.platform == "darwin":
        return os.path.join(
            home,
            "Library",
            "Preferences",
            "Autodesk",
            "maya",
            str(maya_version),
        )
    else:  # linux
        return os.path.join(home, "maya", str(maya_version))


def _get_modules_dir(maya_version: int) -> str:
    """Return the user modules directory for the given Maya version.

    This is the standard location for Maya module description files (.mod).
    Maya's default ``MAYA_MODULE_PATH`` includes this directory.

    Platform paths:
        Windows: ~/Documents/maya/<version>/modules/
        Linux:   ~/maya/<version>/modules/
        macOS:   ~/Library/Preferences/Autodesk/maya/<version>/modules/
    """
    return os.path.join(_get_maya_base_dir(maya_version), "modules")


# ---------------------------------------------------------------------------
# Cross-platform helper: unblock plugin files (Windows)
# ---------------------------------------------------------------------------
def _unblock_files(directory: str) -> None:
    """
    Remove Windows Mark-of-the-Web (Zone.Identifier) from plugin files.

    When files are copied from a zip or downloaded, Windows adds an alternate
    data stream ``Zone.Identifier`` that marks them as untrusted.  Maya may
    refuse to load such files unless the stream is removed.

    On non-Windows platforms this is a no-op (Zone.Identifier is a Windows
    NTFS alternate data stream concept).
    """
    if sys.platform != "win32":
        return
    removed = 0
    for root, _dirs, files in os.walk(directory):
        for name in files:
            if name.endswith((".pyd", ".dll", ".so", ".py")):
                full = os.path.join(root, name)
                zone_id = full + ":Zone.Identifier"
                try:
                    if os.path.exists(zone_id):
                        os.remove(zone_id)
                        removed += 1
                except OSError:
                    pass
    if removed:
        print(f"  Unblocked {removed} file(s)")


# ---------------------------------------------------------------------------
# Helpers for replacing locked plugin files on Windows
# ---------------------------------------------------------------------------
def _remove_with_retry(path: str, max_attempts: int = 5) -> None:
    """Remove *path*, retrying with a delay on PermissionError.

    On Windows, plugin files may remain locked in the Maya process even after
    unloading.  We try deletion with a back-off delay; if that
    fails, we rename the directory (Windows allows renaming in-use files),
    then schedule deletion for the next install.
    """
    for attempt in range(max_attempts):
        try:
            if os.path.isfile(path):
                os.unlink(path)
            else:
                shutil.rmtree(path)
            return  # success
        except PermissionError:
            if attempt < max_attempts - 1:
                _time.sleep(0.5 * (attempt + 1))  # progressive back-off
            else:
                # Last resort — rename the directory out of the way.
                # Windows typically allows renaming directories that contain
                # loaded DLLs, even though it won't let us delete them.
                parent = os.path.dirname(path)
                base = os.path.basename(path)
                renamed = os.path.join(parent, f"{base}_old_{int(_time.time())}")
                try:
                    os.rename(path, renamed)
                    print(f"  Renamed locked directory to: {renamed}")
                    print("  (will be cleaned up on next install)")
                except OSError:
                    # Truly stuck — nothing more we can do
                    print(
                        f"  ⚠ Could not remove {base} — files are locked by Maya.\n"
                        f"     Restart Maya and run the installer again."
                    )


# ---------------------------------------------------------------------------
# Sentinel raised when the user cancels because the plugin is still in use.
# ---------------------------------------------------------------------------
class _InstallCancelled(RuntimeError):
    """User chose to abort installation (plugin still in use)."""


# ---------------------------------------------------------------------------
# Plugin unload with user-friendly dialog on failure
# ---------------------------------------------------------------------------
def _native_plugin_name() -> str:
    """Return the MayaMMD plugin filename for the current platform.

    Windows → MayaMMD.mll, Linux → MayaMMD.so, macOS → MayaMMD.bundle.
    """
    ext = {
        "win32": ".mll",
        "linux": ".so",
        "darwin": ".bundle",
    }.get(_sys.platform, ".mll")
    return f"MayaMMD{ext}"


def _unload_current_plugin(cmds) -> None:
    """Try to unload any loaded variant of MayaMMD.

    If the plugin is still in use (UI open, custom nodes in scene, etc.),
    shows a dialog explaining how to resolve it and lets the user retry or
    defer to a Maya restart.

    Raises:
        _InstallCancelled: if the user chooses to abort.
    """
    variants = (_native_plugin_name(),)
    loaded = [v for v in variants if cmds.pluginInfo(v, query=True, loaded=True)]
    if not loaded:
        return

    while True:
        variant = loaded[0]
        print(f"  Unloading existing {variant}...")
        try:
            cmds.unloadPlugin(variant)
            # Re-check remaining variants after a successful unload
            loaded = [
                v for v in variants if cmds.pluginInfo(v, query=True, loaded=True)
            ]
            if not loaded:
                return
        except RuntimeError as e:
            msg = str(e)
            result = cmds.confirmDialog(
                title="MayaMMD — Cannot Unload",
                message=(
                    f"MayaMMD is currently in use and cannot be unloaded.\n\n"
                    f"  Maya says: {msg}\n\n"
                    f"Common cause:\n"
                    f"  • PMX models with custom nodes (bones, morphs, IK)\n"
                    f"    are still in the scene\n\n"
                    f"The installer will copy the updated files now, but you\n"
                    f"must restart Maya for the new plugin to take effect."
                ),
                button=["Continue (Restart Later)", "Cancel"],
                defaultButton="Continue (Restart Later)",
                cancelButton="Cancel",
                dismissString="Cancel",
            )
            if result == "Cancel":
                raise _InstallCancelled(
                    "Installation cancelled — restart Maya and run the installer again."
                )
            # "Continue (Restart Later)"
            print("  Skipping unload — restart Maya for changes to take effect.")
            return


# ---------------------------------------------------------------------------
# Installation logic
# ---------------------------------------------------------------------------
def _install(maya_version: int, source_dir: str) -> list[str]:
    """
    Perform the actual installation (Maya Module).

    Args:
        maya_version: e.g. 2024, 2025, 2026
        source_dir: Path to the unzipped MayaMMD release directory

    Returns:
        List of log messages
    """
    import maya.cmds as cmds

    messages = []

    # --- 1. Validate Maya version ---
    supported = list(_MAYA_VERSION_LABEL.keys())
    if maya_version not in supported:
        msg = (
            f"Unsupported Maya version: {maya_version}. "
            f"Supported versions: {supported}"
        )
        raise RuntimeError(msg)

    # --- 2. Locate the correct release package for this platform + Maya version ---
    platform_tag = _get_platform_tag()
    maya_label = _MAYA_VERSION_LABEL.get(maya_version, str(maya_version))
    plat_name = _PLATFORM_NAMES.get(platform_tag, platform_tag)
    subdir_name = f"MayaMMD-Maya{maya_label}-{plat_name}"

    package_src = os.path.join(source_dir, subdir_name)
    if not os.path.isdir(package_src):
        raise RuntimeError(
            f"Could not find release package for Maya {maya_version} "
            f"on {platform_tag}. Expected at: {subdir_name}"
        )

    # The source has the module structure: MayaMMD.mod + MayaMMD/ folder
    has_module_structure = os.path.isfile(
        os.path.join(package_src, "MayaMMD.mod")
    ) and os.path.isdir(os.path.join(package_src, "MayaMMD"))

    # --- 3. Unload plugin if already loaded ---
    # Clear undo history first — it can hold references to plugin-owned
    # DG nodes and Python objects, keeping modules locked on Windows.
    cmds.undoInfo(state=False)
    cmds.undoInfo(state=True)

    _unload_current_plugin(cmds)

    # Release module locks by removing them from sys.modules.
    for mod_name in list(sys.modules.keys()):
        if mod_name == "mmd" or mod_name.startswith("mmd."):
            del sys.modules[mod_name]

    # Destroy any strong references that may have survived (e.g. callbacks,
    # workspace controls). Then force GC + a brief pause so Windows can
    # release DLL handles.
    gc.collect()
    _time.sleep(0.5)

    # --- 4. Install as a Maya Module ---
    # Maya modules are the recommended distribution mechanism.
    # The source package contains:
    #   MayaMMD.mod          → ~/Documents/maya/<ver>/modules/MayaMMD.mod
    #   MayaMMD/             → ~/Documents/maya/<ver>/modules/MayaMMD/
    modules_dir = _get_modules_dir(maya_version)
    module_dir = os.path.join(modules_dir, "MayaMMD")
    os.makedirs(modules_dir, exist_ok=True)

    # Clean up renamed leftovers from previous installs where files were
    # locked and couldn't be deleted directly.
    for _entry in os.listdir(modules_dir):
        if _entry.startswith("MAYAMMD_old_"):
            _old = os.path.join(modules_dir, _entry)
            try:
                shutil.rmtree(_old)
            except (OSError, PermissionError):
                pass

    if has_module_structure:
        # New-style: copy MayaMMD.mod and MayaMMD/ directly into modules/
        entries = os.listdir(package_src)
        entry_list = ", ".join(
            e + "/" if os.path.isdir(os.path.join(package_src, e)) else e
            for e in entries
        )
        print(
            f"  Copying {len(entries)} item(s) to {modules_dir} ...",
            flush=True,
        )
        for entry_name in entries:
            src_entry = os.path.join(package_src, entry_name)
            dst_entry = os.path.join(modules_dir, entry_name)
            if os.path.isdir(src_entry):
                if os.path.exists(dst_entry):
                    _remove_with_retry(dst_entry)
                shutil.copytree(src_entry, dst_entry)
            else:
                shutil.copy2(src_entry, dst_entry)
        print(f"    ✓ {entry_list}")
        print(f"    → {modules_dir}")
    else:
        # Fallback: flat layout (pre-module releases) — mmd/ package only
        if os.path.exists(module_dir):
            _remove_with_retry(module_dir)
        os.makedirs(module_dir, exist_ok=True)
        for entry_name in os.listdir(package_src):
            src_entry = os.path.join(package_src, entry_name)
            dst_entry = os.path.join(module_dir, entry_name)
            if os.path.isdir(src_entry):
                shutil.copytree(src_entry, dst_entry)
            else:
                shutil.copy2(src_entry, dst_entry)

        # Generate .mod file for flat-layout fallback
        module_path = module_dir.replace("\\", "/")
        version = _get_version(os.path.join(module_dir, "mmd"))
        mod_content = f"+ MayaMMD {version} {module_path}\n"
        with open(
            os.path.join(modules_dir, "MayaMMD.mod"), "w", encoding="utf-8"
        ) as f:
            f.write(mod_content)
        print(f"  ✓ Wrote MayaMMD.mod (flat fallback)")

    # Read version from installed mmd/_version.py
    version = _get_version(os.path.join(module_dir, "plug-ins", "mmd")) or _get_version(
        os.path.join(module_dir, "mmd")
    )

    # Unblock plugin files (Windows SmartScreen / Mark-of-the-Web)
    _unblock_files(os.path.join(module_dir, "plug-ins"))

    # --- 5. Load the plugin ---
    loader_name = None
    for candidate in (_native_plugin_name(),):
        candidate_path = os.path.join(module_dir, "plug-ins", candidate)
        if os.path.exists(candidate_path):
            loader_dst = candidate_path
            loader_name = candidate
            break

    if not loader_name:
        raise RuntimeError(f"No {_native_plugin_name()} found in {package_src}")

    # Ensure the plug-ins dir is on MAYA_PLUG_IN_PATH so cmds.loadPlugin works.
    # (Maya adds module plug-ins/ paths at startup, but at runtime we may
    # need to add it for the current session.)
    plugins_dir = os.path.join(module_dir, "plug-ins")
    plugin_paths = os.environ.get("MAYA_PLUG_IN_PATH", "")
    if plugins_dir not in plugin_paths:
        os.environ["MAYA_PLUG_IN_PATH"] = plugins_dir + os.pathsep + plugin_paths
    if plugins_dir not in sys.path:
        # Use append(), NOT insert(0) — Maya's executeDroppedPythonFile
        # inserts the dropped file's directory at sys.path[0] and expects
        # to pop it back off when done.  Inserting at 0 here would shift
        # Maya's entry, causing the wrong directory to be removed and
        # polluting sys.path with stale paths.
        sys.path.append(plugins_dir)

    # Unload any previously loaded variant
    _unload_current_plugin(cmds)

    print("  ── Loading plugin ──")
    cmds.loadPlugin(loader_dst, quiet=True)

    # Enable auto-load so the plugin loads on next startup too.
    try:
        cmds.pluginInfo(loader_name, edit=True, autoload=True)
    except Exception:
        pass

    messages.append(f"Installed MayaMMD v{version}")
    messages.append(f"  Module path: {module_dir}")
    messages.append(f"  Plugin will auto-load on next Maya restart")

    return messages


def _get_version(mmd_dir: str) -> str:
    """Read the plugin version from an mmd/_version.py file (regex, not exec)."""
    import re

    try:
        vf = os.path.join(mmd_dir, "_version.py")
        with open(vf, encoding="utf-8") as f:
            content = f.read()
        match = re.search(
            r"__version__\s*=\s*(?:version\s*=\s*)?['\"]([^'\"]+)['\"]", content
        )
        if match:
            return match.group(1)
    except Exception:
        pass
    return "?"


# ---------------------------------------------------------------------------
# Maya drag-and-drop entry points
# ---------------------------------------------------------------------------
def onMayaDroppedPythonFile(*args, **kwargs) -> None:
    """
    Maya 2017+ drag-and-drop entry point.

    Runs the installer, then removes this module from ``sys.modules`` so
    the next drag always imports a fresh copy from disk.
    """
    try:
        source_dir = os.path.dirname(os.path.abspath(__file__))
        _run_install(source_dir)
    finally:
        # __name__ is the module name Maya used to import us (e.g. "install").
        # Removing it from sys.modules ensures the next drag picks up the
        # latest file instead of reusing this cached module object.
        sys.modules.pop(__name__, None)

        # Also bust Python's internal finder caches (sys.path_importer_cache
        # and importlib's own caches).  Without this, switching between
        # different directories that each contain an install.py can return
        # a stale finder that points to the wrong version.
        import importlib

        importlib.invalidate_caches()
        gc.collect()


def _run_install(source_dir: str) -> None:
    """Shared install logic for all entry points."""
    import maya.cmds as cmds

    print("\n" + "=" * 50)
    print("MayaMMD Installer")
    print("=" * 50)

    try:
        maya_version = _get_maya_version()
        modules_dir = _get_modules_dir(maya_version)
        module_dir = os.path.join(modules_dir, "MayaMMD")

        # Determine version before the dialog so the user can see what
        # they're about to install (read from the source package, not the
        # destination — the destination may not exist yet).
        platform_tag = _get_platform_tag()
        maya_label = _MAYA_VERSION_LABEL.get(maya_version, str(maya_version))
        plat_name = _PLATFORM_NAMES.get(platform_tag, platform_tag)
        pkg_dir = os.path.join(
            source_dir,
            f"MayaMMD-Maya{maya_label}-{plat_name}",
            "MayaMMD",
            "plug-ins",
            "mmd",
        )
        version = _get_version(pkg_dir) if os.path.isdir(pkg_dir) else "?"

        print(f"Detected Maya {maya_version}")
        print(f"Version to install: v{version}")
        print(f"Module location: {module_dir}")
        print()

        # Ask user for confirmation
        install_msg = (
            f"Install MayaMMD v{version} for Maya {maya_version}?\n\n"
            f"Module path: {module_dir}"
        )

        result = cmds.confirmDialog(
            title="MayaMMD Installer",
            message=install_msg,
            button=["Install", "Cancel"],
            defaultButton="Install",
            cancelButton="Cancel",
            dismissString="Cancel",
        )
        if result != "Install":
            print("  Installation cancelled by user.")
            return

        messages = _install(maya_version, source_dir)

        # Clean up __pycache__ directories in the extraction folder
        # (Python locks .pyc files on Windows, preventing deletion)
        for root, dirs, _files in os.walk(source_dir):
            for d in dirs:
                if d == "__pycache__":
                    full = os.path.join(root, d)
                    try:
                        shutil.rmtree(full)
                    except PermissionError:
                        pass

        print("\n" + "─" * 50)
        for msg in messages:
            print(f"  • {msg}")
        print("─" * 50)

        print()
        print("  Click the 'MMD' shelf button or run 'MayaMMD()' to open the UI.")

    except _InstallCancelled as e:
        print(f"\n  ⚠ {e}")
    except Exception as e:
        print(f"\n✗ Installation failed: {e}")
        import traceback

        traceback.print_exc()


# ---------------------------------------------------------------------------
# Standalone / Script Editor entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    if _is_maya():
        source_dir = os.path.dirname(os.path.abspath(__file__))
        _run_install(source_dir)
    else:
        print(
            "This installer must run inside Autodesk Maya. "
            "Drag install.py onto the Maya viewport."
        )
