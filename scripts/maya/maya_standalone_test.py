"""
MayaStandaloneTest.py

Simple standalone test harness for debugging PMX/VMD imports in Maya.

Edit MODEL_PATH and MOTION_PATH below to change what gets loaded.
Add your debug code inside the ``debug_tests()`` function at the bottom.

Usage:  mayapy MayaStandaloneTest.py
"""

import logging
import os
import sys
import time
import traceback

# This script is inside scripts/maya folder, so we need to go two levels up to the project root
_PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

# ── Set MAYA_MODULE_PATH so Maya discovers MayaMMD.mod ─────────────────
# This lets Maya handle MAYA_PLUG_IN_PATH and PYTHONPATH automatically.
# Falls back to manual sys.path if the .mod file doesn't exist yet.
_mod_dir = os.path.join(_PROJECT_ROOT, "out", "install", "maya2026-release")
_mod_file = os.path.join(_mod_dir, "MayaMMD.mod")
if os.path.isfile(_mod_file):
    existing = os.environ.get("MAYA_MODULE_PATH", "")
    os.environ["MAYA_MODULE_PATH"] = (
        f"{_mod_dir}{os.pathsep}{existing}" if existing else _mod_dir
    )
    print(f"[mod]  MAYA_MODULE_PATH += {_mod_dir}")
else:
    # Fallback: add project root to sys.path so mmd/ is importable
    if _PROJECT_ROOT not in sys.path:
        sys.path.insert(0, _PROJECT_ROOT)
    print("[mod]  not found — using sys.path fallback")

# ==============================================================================
# Edit these paths to change which model / motion is loaded.
# ==============================================================================
MODEL_PATH = os.path.join(
    _PROJECT_ROOT,
    "assets",
    "models",
    "GirlsFrontline",
    "TololoDefault",
    "GirlsFrontline TololoDefault.pmx",
)
MOTION_PATH = os.path.join(_PROJECT_ROOT, "assets", "motions", "1.vmd")

# ==============================================================================
# Maya standalone - must be initialised BEFORE any maya.* imports
# ==============================================================================
import maya.standalone  # noqa: E402

maya.standalone.initialize()

import maya.cmds as cmds  # noqa: E402
import maya.mel as mel  # noqa: E402

from mmd.core.pmx_importer import parse_pmx  # noqa: E402
from mmd.core.vmd_importer import parse_vmd_file  # noqa: E402
from mmd.maya.pmx_scene_builder import build_pmx_scene  # noqa: E402
from mmd.maya.vmd_scene_builder import apply_vmd_to_scene  # noqa: E402

logging.basicConfig(level=logging.INFO, force=True)
log = logging.getLogger(__name__)


# ==============================================================================
# Plugin loader
# ==============================================================================


def load_plugin():
    """Load the MayaMMD plugin via Maya's standard plugin discovery.

    Maya finds the .mll via ``MAYA_PLUG_IN_PATH`` (set by the .mod file
    or Maya.env).  No explicit path needed.
    """
    plugin_name = "MayaMMD"
    if not cmds.pluginInfo(plugin_name, query=True, loaded=True):
        cmds.loadPlugin(plugin_name)
    print(f"[OK] {plugin_name} loaded")


# ==============================================================================
# ==============================================================================
# Add your debug code here.  ``maya_data`` is the result of
# build_pmx_scene() - it has .root_name, .mesh_name, .bone_name_map, etc.
# ==============================================================================
# ==============================================================================


def debug_tests(maya_data):
    """Add your ad-hoc assertions / experiments here."""

    print(f"\n-- Debug --")
    print(f"Root: {maya_data.root_name}")
    print(f"Mesh: {maya_data.mesh_name}")
    print(f"Bones mapped: {len(maya_data.bone_name_map)}")
    print(f"Morphs mapped: {len(maya_data.morph_name_map)}")
    print(f"IK handles: {len(maya_data.ik_handles)}")

    # -- Your tests below ------------------------------------------------

    # Example: list all joints
    # joints = cmds.listRelatives(maya_data.root_name, allDescendents=True, type="joint") or []
    # for j in joints:
    #     print(f"  {j}")

    # Example: check a specific bone's animation
    # bone = maya_data.bone_name_map.get("センター")
    # if bone:
    #     keys = cmds.keyframe(bone, attribute="rotateX", q=True) or []
    #     print(f"{bone} rotateX keys: {len(keys)}")

    # --------------------------------------------------------------------


# ==============================================================================
# Main
# ==============================================================================


def main():
    print(
        f"Maya {mel.eval('getApplicationVersionAsFloat()')}  |  "
        f"Project: {_PROJECT_ROOT}"
    )

    load_plugin()

    cmds.file(new=True, force=True)

    # Import model
    t0 = time.perf_counter()
    print(f"\n-- Importing model: {MODEL_PATH}")
    pmx_data = parse_pmx(MODEL_PATH)
    maya_data = build_pmx_scene(pmx_data)
    print(f"[OK] Scene built in {time.perf_counter() - t0:.2f}s")

    # Apply motion
    t0 = time.perf_counter()
    print(f"\n-- Applying motion: {MOTION_PATH}")
    vmd_data = parse_vmd_file(MOTION_PATH)
    apply_vmd_to_scene(
        vmd_data,
        model=maya_data.to_resolved(),
    )
    print(
        f"[OK] Motion applied in {time.perf_counter() - t0:.2f}s"
    )

    # Run debug tests
    debug_tests(maya_data)

    maya.standalone.uninitialize()
    print("\nDone.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        maya.standalone.uninitialize()
        sys.exit(1)
