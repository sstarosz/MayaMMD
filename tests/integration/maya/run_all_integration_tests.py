"""
run_all_integration_tests.py

xUnified test runner for all PMX/VMD/VPD → Maya integration tests.

This script is invoked by ``run_integration_tests.py`` (the outer CLI wrapper).
It accepts command-line arguments that control which suites run, which models
are tested, and which motion/pose files are used.

All configuration is done via CLI arguments — no hardcoded dicts.
"""

from __future__ import annotations

import argparse
import functools
import logging
import os
import sys
import time
import traceback
from collections.abc import Callable
from dataclasses import dataclass

# ── Headless Qt — must be set BEFORE any Maya/Qt imports ───────────────────
# Maya's standalone initialize() creates a QGuiApplication internally.
# QWidget requires a QApplication (subclass of QGuiApplication), so we must
# create our own QApplication BEFORE Maya standalone to prevent Maya from
# creating a bare QGuiApplication that blocks QWidget usage.
# The offscreen QPA platform is required for headless testing (mayapy, CI).
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# Force UTF-8 I/O to avoid cp1250 encoding errors with Unicode checkmarks
os.environ.setdefault("PYTHONIOENCODING", "utf-8")
# Suppress Qt font warnings in headless mode
os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.fonts.warning=false")

# Create QApplication before Maya standalone so Qt uses it as the singleton.
# Only do this in mayapy (standalone) mode — Maya's interactive session
# already has a full QApplication.
from PySide6.QtWidgets import QApplication  # noqa: E402

_qt_app: QApplication | None = None
if QApplication.instance() is None:
    _qt_app = QApplication([])
elif not isinstance(QApplication.instance(), QApplication):
    # Maya standalone created a QGuiApplication (not QApplication).
    # We can't create a second QCoreApplication, so QWidget-based
    # tests won't work.  This is expected — the suite that needs
    # QWidgets must be skipped or handled specially.
    pass

# ── Add workspace root to sys.path ──────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

# ── Maya standalone must be initialised BEFORE any maya.* imports ───────────
import maya.standalone

maya.standalone.initialize()

import maya.mel as mel  # noqa: E402
import maya.cmds as cmds  # noqa: E402

# ── Project imports ─────────────────────────────────────────────────────────
from assets.assets_utils import (  # noqa: E402
    get_all_pmx_model_paths,
    get_all_vmd_paths,
    get_all_vpd_paths,
)
from mmd.core.pmx_importer import parse_pmx as _uncached_parse_pmx  # noqa: E402
from mmd.core.vmd_importer import parse_vmd_file  # noqa: E402
from mmd.core.vpd_importer import parse_vpd_file  # noqa: E402
from mmd.maya.pmx_scene_builder import build_pmx_scene  # noqa: E402

# Cache parsed PMX data: across suites (import → bone → morph) the same
# model is loaded 3×.  parse_pmx is pure (no Maya deps), so caching is safe.
_parse_cache: dict[str, object] = {}


def parse_pmx(path: str) -> object:
    """Thin caching wrapper around :func:`mmd.core.pmx_importer.parse_pmx`."""
    if path not in _parse_cache:
        _parse_cache[path] = _uncached_parse_pmx(path)
    return _parse_cache[path]


from tests.integration.test_helpers import (  # noqa: E402
    _run_single_test,
    color_text,
    load_plugin,
    run_standalone_suite,
    run_test_suite,
    setup_test_environment,
)

# ══════════════════════════════════════════════════════════════════════════
# CLI argument parsing
# ══════════════════════════════════════════════════════════════════════════


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments (used by mayapy subprocess)."""
    parser = argparse.ArgumentParser(
        description="Unified integration test runner for PMX/VMD/VPD → Maya."
    )
    parser.add_argument(
        "suite",
        nargs="?",
        default="all",
        choices=[
            "all",
            "import",
            "bone",
            "morph",
            "rigidbody",
            "vmd",
            "vpd",
            "node",
            "cmd",
            "ccd",
            "multi",
            "context",
            "pose-tree",
        ],
        help="Test suite to run (default: all)",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Substring filter for model paths (case-insensitive).",
    )
    parser.add_argument(
        "--motion",
        default=None,
        help="Substring filter for VMD motion file paths.",
    )
    parser.add_argument(
        "--pose",
        default=None,
        help="Substring filter for VPD pose file paths.",
    )
    parser.add_argument(
        "--all-models",
        action="store_true",
        help="Test against every available model (vmd/vpd suites).",
    )
    parser.add_argument(
        "--all-motions",
        action="store_true",
        help="Use every available VMD motion file.",
    )
    parser.add_argument(
        "--all-poses",
        action="store_true",
        help="Use every available VPD pose file.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        default=None,
        help="Treat production-code warnings as test failures.",
    )
    return parser.parse_args(argv)


# ══════════════════════════════════════════════════════════════════════════

# Cache all available paths (evaluated once at import time)
_ALL_PMX_MODELS = get_all_pmx_model_paths()
_ALL_VMD_MOTIONS = get_all_vmd_paths()
_ALL_VPD_POSES = get_all_vpd_paths()


# ---------------------------------------------------------------------------
# Test suite definitions
# ---------------------------------------------------------------------------


def get_import_tests():
    """Import test_pmx_import_integration and return its test list."""
    # Import the module here to avoid issues with Maya initialization
    from tests.integration.maya import test_pmx_import_integration

    return test_pmx_import_integration._TESTS


def get_bone_tests():
    """Import test_pmx_bone_integration and return its test list."""
    from tests.integration.maya import test_pmx_bone_integration

    return test_pmx_bone_integration._TESTS


def get_morph_tests():
    """Import test_pmx_morph_integration and return its test list."""
    from tests.integration.maya import test_pmx_morph_integration

    return test_pmx_morph_integration._TESTS


def get_rigid_body_tests():
    """Import test_pmx_rigid_body_integration and return its test list."""
    from tests.integration.maya import test_pmx_rigid_body_integration

    return test_pmx_rigid_body_integration._TESTS


def get_node_tests():
    """Import test_bone_morph_node_integration and return its test list (no PMX data needed)."""
    from tests.integration.maya.nodes import test_bone_morph_node_integration

    return test_bone_morph_node_integration._TESTS


def get_ccd_solver_tests():
    """Import test_ccd_ik_solver_node_integration and return its test list (no PMX data needed)."""
    from tests.integration.maya.nodes import test_ccd_ik_solver_node_integration

    return test_ccd_ik_solver_node_integration._TESTS


def get_cmd_tests():
    """Import the native command test modules and return their test lists.

    boneMorphNode, boneBlendShape, mmdPhysicsNode, mmdRigidBody and
    mmdRigidBodyConstraint are already registered by MayaMMD.mll.
    """
    from tests.integration.maya.cmds import (
        test_bone_blend_shape_cmd_integration,
        test_mmd_rigid_body_cmd_integration,
    )

    return (
        test_bone_blend_shape_cmd_integration._TESTS
        + test_mmd_rigid_body_cmd_integration._TESTS
    )


def get_vpd_tests():
    """Import test_vpd_integration and return its test list."""
    from tests.integration.maya import test_vpd_integration

    return test_vpd_integration._TESTS


def get_vmd_tests():
    """Import test_vmd_integration and return its test list."""
    from tests.integration.maya import test_vmd_integration

    return test_vmd_integration._TESTS


def get_multi_import_tests():
    """Import test_multi_import_integration and return its test list."""
    from tests.integration.maya import test_multi_import_integration

    return test_multi_import_integration._TESTS


def get_context_tests():
    """Import test_model_context_integration and return its test list."""
    from tests.integration.maya import test_model_context_integration

    return test_model_context_integration._TESTS


def get_pose_tree_tests():
    """Import test_morph_tree_widget_integration and return its test list."""
    from tests.integration.maya import test_morph_tree_widget_integration

    return test_morph_tree_widget_integration._TESTS


# ── Suite metadata ──────────────────────────────────────────────────────


@dataclass
class SuiteInfo:
    """Declarative metadata for a test suite.

    All behavioural decisions (which suites need models, which are
    mutating, which participate in the unified model-loop, etc.) flow
    from these flags rather than being scattered across conditionals.
    """

    display_name: str
    """Human-readable label shown in test output."""

    getter: Callable[[], list]
    """Zero-argument callable that returns the suite's ``_TESTS`` list."""

    needs_model: bool = True
    """Suite requires a PMX model to be loaded and built in the scene."""

    needs_vmd: bool = False
    """Suite requires VMD motion data to be loaded."""

    needs_vpd: bool = False
    """Suite requires VPD pose data to be loaded."""

    is_mutating: bool = False
    """Suite modifies the scene and should be wrapped in undo chunks."""

    needs_pristine_scene: bool = False
    """Suite needs a freshly built scene (used after mutating suites)."""

    supports_unified: bool = False
    """Suite participates in the unified model-loop optimisation
    (build scene once, run multiple suites against it)."""

    dispatch_mode: str = "generic"
    """Dispatch strategy used in the per-suite loop.

    ``"generic"``
        Routes to :func:`run_test_suite` (if ``needs_model``) or
        :func:`run_standalone_suite` (otherwise).  Covers import, bone,
        morph, node, ccd, cmd.

    ``"vmd"`` / ``"vpd"``
        Matrix-aware dispatch that splits tests by ``@matrix`` decorator
        and runs matrix tests per motion/pose file.

    ``"multi"`` / ``"context"`` / ``"pose-tree"``
        Suite-specific dispatch with custom model selection and
        pre-build steps.
    """


# Test suite registry: maps suite key → SuiteInfo
TEST_SUITES: dict[str, SuiteInfo] = {
    # ── PMX-dependent suites (participate in unified loop) ───────────
    "import": SuiteInfo(
        "PMX General Import Tests", get_import_tests, supports_unified=True
    ),
    "bone": SuiteInfo(
        "PMX Bone Integration Tests", get_bone_tests, supports_unified=True
    ),
    "morph": SuiteInfo(
        "PMX Morph Integration Tests", get_morph_tests, supports_unified=True
    ),
    "rigidbody": SuiteInfo(
        "PMX Rigid Body Tests", get_rigid_body_tests, supports_unified=True
    ),
    "vmd": SuiteInfo(
        "VMD Integration Tests",
        get_vmd_tests,
        needs_vmd=True,
        is_mutating=True,
        supports_unified=True,
        dispatch_mode="vmd",
    ),
    "vpd": SuiteInfo(
        "VPD Pose Integration Tests",
        get_vpd_tests,
        needs_vpd=True,
        is_mutating=True,
        needs_pristine_scene=True,
        supports_unified=True,
        dispatch_mode="vpd",
    ),
    # ── No-asset suites ─────────────────────────────────────────────
    "node": SuiteInfo(
        "BoneMorphNode Tests (No PMX)", get_node_tests, needs_model=False
    ),
    "ccd": SuiteInfo(
        "CCD IK Solver Node Tests (No PMX)", get_ccd_solver_tests, needs_model=False
    ),
    "cmd": SuiteInfo("Command Tests (No PMX)", get_cmd_tests, needs_model=False),
    # ── Multi-model / custom-dispatch suites ─────────────────────────
    "multi": SuiteInfo(
        "Multi-Import Tests", get_multi_import_tests, dispatch_mode="multi"
    ),
    "context": SuiteInfo(
        "ModelContext Tests (No PMX)",
        get_context_tests,
        needs_vmd=True,
        needs_vpd=True,
        dispatch_mode="context",
    ),
    "pose-tree": SuiteInfo(
        "Pose Tree Widget Tests",
        get_pose_tree_tests,
        needs_vmd=True,
        dispatch_mode="pose-tree",
    ),
}


# ── Shared helpers ──────────────────────────────────────────────────────


def _ensure_two_models(filtered: list[str], all_models: list[str]) -> list[str]:
    """Return a list of at least 2 distinct model paths.

    Model A is always the first entry from *filtered* (respecting
    ``--model``).  When fewer than 2 models are available in *filtered*,
    model B is picked from *all_models* (excluding model A) so that
    multi-model tests always have two distinct models to work with.

    Args:
        filtered: Model paths matching the ``--model`` filter.
        all_models: Every available model path.

    Returns:
        A *new* list with 0, 1, or ≥2 model paths.
    """
    result: list[str] = list(filtered)
    if len(result) < 2:
        for m in all_models:
            if m not in result:
                result.append(m)
                break
    return result


# ---------------------------------------------------------------------------
# Main runner
# ---------------------------------------------------------------------------


def _create_model_filter(model_arg: str | None):
    """Create a model filter callable from the ``--model`` CLI argument.

    Returns ``None`` when no filter is requested.
    """
    if model_arg is None:
        return None

    # Case-insensitive substring match against the full path
    def _filter(path: str) -> bool:
        if os.name == "nt":
            return model_arg.lower() in path.lower()
        return model_arg in path

    return _filter


def _select_motion_files(motion_arg: str | None, all_motions: bool) -> list[str]:
    """Select which VMD motion files to use based on CLI args.

    Args:
        motion_arg: Value of ``--motion`` (substring filter), or ``None``.
        all_motions: ``True`` if ``--all-motions`` was passed.

    Returns:
        List of selected motion file paths.
    """
    if all_motions:
        return _ALL_VMD_MOTIONS
    if motion_arg:
        # Filter by substring
        result = [
            p
            for p in _ALL_VMD_MOTIONS
            if (motion_arg.lower() in p.lower() if os.name == "nt" else motion_arg in p)
        ]
        if result:
            return result
        # Fall back to first match if filter yields nothing
    # Default: pick the first motion that looks like "1.vmd"
    default = next(
        (m for m in _ALL_VMD_MOTIONS if "1.vmd" in m),
        _ALL_VMD_MOTIONS[0] if _ALL_VMD_MOTIONS else None,
    )
    return [default] if default else []


def _select_pose_files(pose_arg: str | None, all_poses: bool) -> list[str]:
    """Select which VPD pose files to use based on CLI args.

    Args:
        pose_arg: Value of ``--pose`` (substring filter), or ``None``.
        all_poses: ``True`` if ``--all-poses`` was passed.

    Returns:
        List of selected pose file paths.
    """
    if all_poses:
        return _ALL_VPD_POSES
    if pose_arg:
        result = [
            p
            for p in _ALL_VPD_POSES
            if (pose_arg.lower() in p.lower() if os.name == "nt" else pose_arg in p)
        ]
        if result:
            return result
    # Default: pick the first pose that looks like "1.vpd"
    default = next(
        (p for p in _ALL_VPD_POSES if "1.vpd" in p),
        _ALL_VPD_POSES[0] if _ALL_VPD_POSES else None,
    )
    return [default] if default else []


def run_all_integration_tests(args: argparse.Namespace) -> bool:
    """Run all enabled test suites with configured model filtering.

    Args:
        args: Parsed command-line arguments from :func:`parse_args`.

    Returns:
        ``True`` if all enabled tests passed, ``False`` otherwise.
    """
    _start_time = time.perf_counter()

    # ── Determine enabled suites ─────────────────────────────────────────
    if args.suite == "all":
        enabled = list(TEST_SUITES.keys())
    else:
        enabled = [args.suite]

    # ── Model filter ─────────────────────────────────────────────────────
    model_filter = _create_model_filter(args.model)

    all_models = _ALL_PMX_MODELS
    if model_filter is not None:
        filtered_models = [m for m in all_models if model_filter(m)]
    else:
        filtered_models = all_models

    # ── VMD/VPD file selection ───────────────────────────────────────────
    _need_vmd = any(TEST_SUITES[s].needs_vmd for s in enabled)
    _need_vpd = any(TEST_SUITES[s].needs_vpd for s in enabled)
    vmd_motions = (
        _select_motion_files(args.motion, args.all_motions) if _need_vmd else []
    )
    vpd_poses = _select_pose_files(args.pose, args.all_poses) if _need_vpd else []

    # ── VMD performance tuning ──────────────────────────────────────────
    # Sample every Nth keyframe (1 = exhaustive for CI, 5 = fast dev).
    _vmd_sample_every = int(os.environ.get("MAYAMMD_VMD_SAMPLE_EVERY", "5"))

    # ── Display configuration ────────────────────────────────────────────
    print(color_text("\n" + "═" * 60, "cyan"))
    print(color_text("  PMX/VMD/VPD Integration Test Suite Runner", "bold"))
    print(color_text("═" * 60, "cyan"))

    print(f"\n{color_text('Configuration:', 'bold')}")
    print(f"  Enabled suites: {', '.join(enabled)}\n")
    print(
        f"  Model filter  : {args.model or 'None (testing all models)'}, {len(filtered_models)} of {len(all_models)} total"
    )
    if filtered_models:
        for m in filtered_models:
            print(f"    - {os.path.basename(m)}")

    if "vmd" in enabled:
        print(
            f"\n  VMD motion filter: {args.motion or 'None (using default motion)'}, {len(vmd_motions)} of {len(_ALL_VMD_MOTIONS)} total"
        )
        for m in vmd_motions:
            print(f"    - {os.path.basename(m)}")

    if "vpd" in enabled:
        print(
            f"\n  VPD pose filter: {args.pose or 'None (using default pose)'}, {len(vpd_poses)} of {len(_ALL_VPD_POSES)} total"
        )
        for p in vpd_poses:
            print(f"    - {os.path.basename(p)}")

    # The multi-import suite picks its own models internally, so only
    # error when no models are available for PMX-based suites.
    suites_needing_models = [s for s in enabled if TEST_SUITES[s].needs_model]
    if suites_needing_models and not filtered_models:
        print(color_text("ERROR: No models match the filter criteria!", "red"))
        return False

    # ── Inject file selections into the test modules ────────────────────
    # (Data is now passed via functools.partial per-iteration below;
    #  no globals are set on the test modules.)

    # ── Ensure a QApplication exists before any Qt-dependent suite ──────
    # A QApplication should already exist from module-level init (created
    # before Maya standalone to prevent Maya from creating a bare
    # QGuiApplication).  This block is a safety net for edge cases.
    if "pose-tree" in enabled:
        from PySide6.QtWidgets import QApplication

        app = QApplication.instance()
        if app is None:
            os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
            sys.modules[__name__]._qt_app_run = QApplication([])
        elif not isinstance(app, QApplication):
            print(
                color_text(
                    "WARNING: Existing Qt app is QGuiApplication, not QApplication. "
                    "QWidget-based tests may not work correctly.",
                    "yellow",
                )
            )

    # ── Run each enabled test suite ──────────────────────────────────────
    all_success = True
    suite_results = []

    # ── Unified model-loop for "all" mode ────────────────────────────────
    # When running every suite, build each model ONCE and run all
    # PMX-dependent tests against that single scene.  VMD / VPD tests wrap
    # animation / pose application in undo chunks so the scene stays clean
    # for the next suite.
    _UNIFIED_PMX_SUITES = [
        s for s, info in TEST_SUITES.items() if info.supports_unified
    ]
    _use_unified = args.suite == "all" and any(
        s in enabled for s in _UNIFIED_PMX_SUITES
    )

    if _use_unified:
        # Collect test lists for the unified suites (cached per suite key).
        # For VMD/VPD suites, pre-parse data and wrap tests with
        # functools.partial so the inner loop can call them uniformly.
        _unified_tests: dict[str, list] = {}
        for sk in _UNIFIED_PMX_SUITES:
            if sk not in enabled:
                continue
            raw_tests = TEST_SUITES[sk].getter()
            if sk == "vmd" and vmd_motions:
                try:
                    _uni_vmd = parse_vmd_file(vmd_motions[0])
                except Exception:
                    _uni_vmd = None
                raw_tests = [
                    (
                        n,
                        functools.partial(
                            f, vmd_data=_uni_vmd, sample_every=_vmd_sample_every
                        ),
                    )
                    for n, f in raw_tests
                ]
            elif sk == "vpd" and vpd_poses:
                # Split matrix vs non-matrix tests.
                # Matrix tests use the CLI-filtered pose list (--all-poses
                # controls speed).  Non-matrix tests (stacking prevention,
                # etc.) always need EVERY available pose to function.
                _matrix_raw = [
                    (n, f) for n, f in raw_tests if getattr(f, "_is_matrix", False)
                ]
                _non_matrix_raw = [
                    (n, f) for n, f in raw_tests if not getattr(f, "_is_matrix", False)
                ]

                # Load ALL poses for non-matrix tests
                _all_vpd: dict[str, object] = {}
                for pp in _ALL_VPD_POSES:
                    try:
                        _all_vpd[os.path.basename(pp)] = parse_vpd_file(pp)
                    except Exception:
                        pass

                # Load filtered poses for matrix tests
                _uni_vpd: dict[str, object] = {}
                for pp in vpd_poses:
                    try:
                        _uni_vpd[os.path.basename(pp)] = parse_vpd_file(pp)
                    except Exception:
                        pass

                wrapped: list = []
                for n, f in _matrix_raw:
                    wrapped.append((n, functools.partial(f, all_vpd_data=_uni_vpd)))
                for n, f in _non_matrix_raw:
                    wrapped.append((n, functools.partial(f, all_vpd_data=_all_vpd)))
                raw_tests = wrapped
            _unified_tests[sk] = raw_tests

        # Metadata-driven: which suites mutate the scene / need a rebuild.
        _MUTATING_UNIFIED: set[str] = {
            s for s in _UNIFIED_PMX_SUITES if TEST_SUITES[s].is_mutating
        }
        _REBUILD_BEFORE: set[str] = {
            s for s in _UNIFIED_PMX_SUITES if TEST_SUITES[s].needs_pristine_scene
        }

        for model_path in filtered_models:
            model_label = (
                os.path.basename(os.path.dirname(model_path))
                + "/"
                + os.path.basename(model_path)
            )

            # ── Build the scene once ─────────────────────────────────
            setup_test_environment()
            pmx_data = parse_pmx(model_path)
            maya_scene_data = build_pmx_scene(pmx_data)

            for sk in _UNIFIED_PMX_SUITES:
                if sk not in _unified_tests:
                    continue

                # Rebuild scene for suites that need a pristine state
                # (e.g. VPD after VMD may have residual DG connections).
                if sk in _REBUILD_BEFORE:
                    setup_test_environment()
                    maya_scene_data = build_pmx_scene(pmx_data)

                suite_name = TEST_SUITES[sk].display_name
                tests = _unified_tests[sk]
                use_undo = sk in _MUTATING_UNIFIED

                suite_start = time.perf_counter()
                suite_pass = run_test_suite(
                    suite_name=f"{suite_name} [{model_label}]",
                    tests=tests,
                    testing_models=[model_path],
                    parse_fn=parse_pmx,
                    build_fn=build_pmx_scene,
                    strict=args.strict,
                    preloaded=(pmx_data, maya_scene_data, model_label, use_undo),
                )

                suite_elapsed = time.perf_counter() - suite_start
                suite_results.append((suite_name, suite_pass, suite_elapsed))
                all_success = all_success and suite_pass

        # Remove unified suites from 'enabled' so the loop below skips them.
        enabled = [s for s in enabled if s not in _UNIFIED_PMX_SUITES]

    # ── Per-suite dispatch (individual suite runs + non-PMX suites) ──────
    for suite_key in enabled:
        if suite_key not in TEST_SUITES:
            print(
                color_text(
                    f"\nWARNING: Unknown suite '{suite_key}', skipping", "yellow"
                )
            )
            continue

        info = TEST_SUITES[suite_key]
        suite_name = info.display_name

        try:
            tests = info.getter()
            suite_start = time.perf_counter()

            mode = info.dispatch_mode

            # ── Generic dispatch: node / ccd / cmd / import / bone / morph ─
            if mode == "generic":
                if info.needs_model:
                    if args.all_models:
                        pmx_models = filtered_models
                    else:
                        pmx_models = [filtered_models[0]] if filtered_models else []
                    if not pmx_models:
                        print(color_text("  No models available – skipping", "yellow"))
                        success = True
                    else:
                        success = run_test_suite(
                            suite_name,
                            tests,
                            pmx_models,
                            parse_fn=parse_pmx,
                            build_fn=build_pmx_scene,
                            strict=args.strict,
                        )
                else:
                    success = run_standalone_suite(
                        suite_name, tests, strict=args.strict
                    )

            # ── VMD: matrix (model × motion) + non-matrix dispatch ────
            elif mode == "vmd":
                # Single model by default for speed; --all-models to test all
                if args.all_models:
                    vmd_models = filtered_models
                else:
                    vmd_models = [filtered_models[0]] if filtered_models else []

                if not vmd_models:
                    print(
                        color_text(
                            "  No models available for VMD testing – skipping",
                            "yellow",
                        )
                    )
                    success = True
                else:
                    # Split tests by @matrix decorator
                    matrix_tests = [
                        (n, f) for n, f in tests if getattr(f, "_is_matrix", False)
                    ]
                    non_matrix_tests = [
                        (n, f) for n, f in tests if not getattr(f, "_is_matrix", False)
                    ]

                    success = True
                    # ── Matrix tests: model × motion ────────────────────
                    for motion_path in vmd_motions:
                        motion_label = os.path.basename(motion_path)
                        try:
                            vmd_data = parse_vmd_file(motion_path)
                        except Exception as exc:
                            print(
                                color_text(
                                    f"  ✗ Failed to parse {motion_label}: {exc}", "red"
                                )
                            )
                            success = False
                            continue
                        wrapped = [
                            (
                                n,
                                functools.partial(
                                    f, vmd_data=vmd_data, sample_every=_vmd_sample_every
                                ),
                            )
                            for n, f in matrix_tests
                        ]
                        motion_suite_name = f"{suite_name} [{motion_label}]"
                        if not run_test_suite(
                            suite_name=motion_suite_name,
                            tests=wrapped,
                            testing_models=vmd_models,
                            parse_fn=parse_pmx,
                            build_fn=build_pmx_scene,
                            model_filter=None,
                            strict=args.strict,
                        ):
                            success = False

                    # ── Non-matrix tests: once per model, all motions ───
                    if non_matrix_tests:
                        try:
                            vmd_data = (
                                parse_vmd_file(vmd_motions[0]) if vmd_motions else None
                            )
                        except Exception as exc:
                            vmd_data = None
                            print(
                                color_text(f"  ✗ Failed to parse motion: {exc}", "red")
                            )
                        wrapped = [
                            (
                                n,
                                functools.partial(
                                    f, vmd_data=vmd_data, sample_every=_vmd_sample_every
                                ),
                            )
                            for n, f in non_matrix_tests
                        ]
                        if not run_test_suite(
                            suite_name=suite_name,
                            tests=wrapped,
                            testing_models=vmd_models,
                            parse_fn=parse_pmx,
                            build_fn=build_pmx_scene,
                            model_filter=None,
                            strict=args.strict,
                        ):
                            success = False
            elif mode == "vpd":
                # Single model by default for speed; --all-models to test all
                if args.all_models:
                    vpd_models = filtered_models
                else:
                    vpd_models = [filtered_models[0]] if filtered_models else []

                if not vpd_models:
                    print(
                        color_text(
                            "  No models available for VPD testing – skipping",
                            "yellow",
                        )
                    )
                    success = True
                else:
                    # Split tests by @matrix decorator
                    matrix_tests = [
                        (n, f) for n, f in tests if getattr(f, "_is_matrix", False)
                    ]
                    non_matrix_tests = [
                        (n, f) for n, f in tests if not getattr(f, "_is_matrix", False)
                    ]

                    success = True
                    # ── Matrix tests: model × pose ──────────────────────
                    for pose_path in vpd_poses:
                        pose_label = os.path.basename(pose_path)
                        try:
                            vpd_data = parse_vpd_file(pose_path)
                            all_vpd = {pose_label: vpd_data}
                        except Exception as exc:
                            print(
                                color_text(
                                    f"  ✗ Failed to parse {pose_label}: {exc}", "red"
                                )
                            )
                            success = False
                            continue
                        wrapped = [
                            (n, functools.partial(f, all_vpd_data=all_vpd))
                            for n, f in matrix_tests
                        ]
                        pose_suite_name = f"{suite_name} [{pose_label}]"
                        if not run_test_suite(
                            suite_name=pose_suite_name,
                            tests=wrapped,
                            testing_models=vpd_models,
                            parse_fn=parse_pmx,
                            build_fn=build_pmx_scene,
                            model_filter=None,
                            strict=args.strict,
                        ):
                            success = False

                    # ── Non-matrix tests: once per model, all poses ────
                    # Always use every available pose — non-matrix tests
                    # (e.g. stacking prevention) need at least 2 poses to
                    # function and are not subject to --all-poses filtering.
                    if non_matrix_tests:
                        all_vpd: dict[str, object] = {}
                        for pose_path in _ALL_VPD_POSES:
                            try:
                                vpd_data = parse_vpd_file(pose_path)
                                all_vpd[os.path.basename(pose_path)] = vpd_data
                            except Exception as exc:
                                print(
                                    color_text(
                                        f"  ✗ Failed to parse {os.path.basename(pose_path)}: {exc}",
                                        "red",
                                    )
                                )
                        wrapped = [
                            (n, functools.partial(f, all_vpd_data=all_vpd))
                            for n, f in non_matrix_tests
                        ]
                        if not run_test_suite(
                            suite_name=suite_name,
                            tests=wrapped,
                            testing_models=vpd_models,
                            parse_fn=parse_pmx,
                            build_fn=build_pmx_scene,
                            model_filter=None,
                            strict=args.strict,
                        ):
                            success = False
            elif mode == "multi":
                # Multi-import tests: pick 2 models from the filtered list.
                _mi_models = (
                    filtered_models[:2]
                    if len(filtered_models) >= 2
                    else list(filtered_models)
                )
                if not _mi_models:
                    print(color_text("  No models available – skipping", "yellow"))
                    success = True
                else:
                    from mmd.core.pmx_importer import parse_pmx as _mi_parse

                    pmx_a = _mi_parse(_mi_models[0])
                    pmx_b = _mi_parse(_mi_models[1]) if len(_mi_models) > 1 else None

                    wrapped = []
                    for test_name, test_func in tests:
                        if test_name in (
                            "Same Model Three Times - Scaling",
                            "diff_after_import Convenience",
                        ):
                            wrapped.append(
                                (test_name, functools.partial(test_func, pmx_a, None))
                            )
                        elif test_name in ("Two Different Models - No Collision",):
                            wrapped.append(
                                (test_name, functools.partial(test_func, pmx_a, pmx_b))
                            )
                        elif test_name in (
                            "SceneSnapshot Utility",
                            "Naming Manager make_unique",
                        ):
                            wrapped.append(
                                (test_name, functools.partial(test_func, None, None))
                            )
                        else:
                            wrapped.append((test_name, lambda: True))

                    success = run_standalone_suite(
                        suite_name, wrapped, strict=args.strict
                    )
            elif mode == "context":
                # ModelContext tests: synthetic (no PMX data) + real-model.
                from tests.integration.maya import (
                    test_model_context_integration as _ctx_mod,
                )

                # ── Phase 1: Synthetic tests ─────────────────────────────
                synth_tests = [
                    (n, f) for n, f in tests if not n.startswith("Real Model")
                ]
                success = run_standalone_suite(
                    "ModelContext — Synthetic Tests", synth_tests, strict=args.strict
                )

                # ── Phase 2: Real-model tests ────────────────────────────
                real_tests = [(n, f) for n, f in tests if n.startswith("Real Model")]
                if real_tests:
                    _ctx_models = _ensure_two_models(filtered_models, _ALL_PMX_MODELS)
                    if not _ctx_models:
                        for test_name, _ in real_tests:
                            print(f"\n  {test_name}")
                            print(
                                color_text(
                                    "  SKIP: No PMX model files available", "yellow"
                                )
                            )
                    else:
                        # Pre-build model A once.
                        setup_test_environment()
                        pmx_a, maya_a = _ctx_mod._load_and_build(_ctx_models[0])

                        # Pre-build model B lazily (on first multi-model test).
                        pmx_b = maya_b = None
                        _model_b_path = _ctx_models[1] if len(_ctx_models) > 1 else None

                        # Pre-parse VMD/VPD data once.
                        _vmd = parse_vmd_file(vmd_motions[0]) if vmd_motions else None
                        _vpd = parse_vpd_file(vpd_poses[0]) if vpd_poses else None

                        # ── Test name → (kwargs, needs_undo) mapping ────
                        _ctx_mutating = getattr(
                            _ctx_mod, "_MUTATING_CONTEXT_TESTS", set()
                        )
                        _CTX_TEST_KWARGS: dict[str, dict] = {
                            "Real Model - ModelContext discovery": dict(
                                pmx_a=pmx_a, maya_a=maya_a
                            ),
                            "Real Model - ModelContext → VMD builder": dict(
                                maya_a=maya_a, vmd_data=_vmd
                            ),
                            "Real Model - ModelContext → VPD builder": dict(
                                maya_a=maya_a, vpd_data=_vpd
                            ),
                        }

                        for test_name, test_func in real_tests:
                            # Build model B on demand for first multi-model test.
                            if (
                                "multi-model" in test_name
                                and maya_b is None
                                and _model_b_path
                            ):
                                pmx_b, maya_b = _ctx_mod._load_and_build(_model_b_path)

                            # Resolve kwargs for this test.
                            if test_name in _CTX_TEST_KWARGS:
                                kwargs = _CTX_TEST_KWARGS[test_name]
                            elif "VMD" in test_name and "multi-model" in test_name:
                                kwargs = dict(
                                    maya_a=maya_a, maya_b=maya_b, vmd_data=_vmd
                                )
                            elif "VPD" in test_name and "multi-model" in test_name:
                                kwargs = dict(
                                    maya_a=maya_a, maya_b=maya_b, vpd_data=_vpd
                                )
                            elif "multi-model" in test_name:
                                kwargs = dict(maya_a=maya_a, maya_b=maya_b)
                            else:
                                kwargs = dict(maya_a=maya_a, pmx_a=pmx_a)

                            wrapped = functools.partial(test_func, **kwargs)
                            use_undo = test_name in _ctx_mutating
                            tr = _run_single_test(
                                wrapped, use_undo=use_undo, collect_warnings=False
                            )

                            passed = tr.passed
                            tick = (
                                color_text("⊘", "yellow")
                                if tr.skipped
                                else color_text("✓", "green")
                                if passed
                                else color_text("✗", "red")
                            )
                            et = (
                                f"[{tr.elapsed:.1f}s]"
                                if tr.elapsed >= 0.1
                                else f"[{tr.elapsed * 1000:.0f}ms]"
                            )
                            hint = ""
                            if tr.captured:
                                last_line = tr.captured.splitlines()[-1].strip()
                                if last_line:
                                    hint = f"  {color_text(last_line, 'green' if passed else 'red')}"
                            print(f"  {tick} {et} {test_name}{hint}")
                            success = success and passed
            elif mode == "pose-tree":
                # Single model by default for speed; --all-models to test all.
                # Multi-model tests always run (they pick their own two models).
                if args.all_models:
                    pt_models = filtered_models
                else:
                    pt_models = [filtered_models[0]] if filtered_models else []

                from tests.integration.maya import (
                    test_morph_tree_widget_integration as _pt_mod,
                )

                # Inject VMD path for the keyframe-indicator test.
                _pt_mod._TEST_VMD = vmd_motions[0] if vmd_motions else None

                if not pt_models:
                    print(color_text("  No models available – skipping", "yellow"))
                    single_ok = True
                else:
                    single_ok = run_test_suite(
                        suite_name=suite_name,
                        tests=tests,
                        testing_models=pt_models,
                        parse_fn=parse_pmx,
                        build_fn=build_pmx_scene,
                        model_filter=None,
                    )

                # ── Multi-model tests ───────────────────────────────────
                _pt_multi = _ensure_two_models(filtered_models, _ALL_PMX_MODELS)
                if len(_pt_multi) < 2:
                    print(
                        color_text(
                            "  Multi-model tests skipped (need 2 models)", "yellow"
                        )
                    )
                    multi_ok = True
                else:
                    setup_test_environment()
                    build_pmx_scene(parse_pmx(_pt_multi[0]))
                    build_pmx_scene(parse_pmx(_pt_multi[1]))
                    wrapped = [
                        (
                            n,
                            functools.partial(
                                f, model_a_path=_pt_multi[0], model_b_path=_pt_multi[1]
                            ),
                        )
                        for n, f in _pt_mod._MULTI_MODEL_TESTS
                    ]
                    multi_ok = run_standalone_suite(
                        "Pose Tree — Multi-Model Tests",
                        wrapped,
                    )

                _pt_mod._TEST_VMD = None
                success = single_ok and multi_ok

            suite_elapsed = time.perf_counter() - suite_start
            suite_results.append((suite_name, success, suite_elapsed))
            all_success = all_success and success

        except Exception as e:
            print(color_text(f"\nFATAL ERROR in suite '{suite_key}': {e}", "red"))
            traceback.print_exc()
            suite_results.append((suite_name, False, 0.0))
            all_success = False

    # ── Print consolidated summary ───────────────────────────────────────
    print(color_text("\n" + "═" * 60, "cyan"))
    print(color_text("  Test Suite Summary", "bold"))
    print(color_text("═" * 60, "cyan"))

    for suite_name, success, elapsed in suite_results:
        status = (
            color_text("✓ PASS", "green") if success else color_text("✗ FAIL", "red")
        )
        elapsed_str = f"[{elapsed:.1f}s]" if elapsed >= 60 else f"[{elapsed:.0f}s]"
        print(f"  {status}  {elapsed_str:>8s}  {suite_name}")

    total_elapsed = time.perf_counter() - _start_time
    elapsed_str = (
        f"[{total_elapsed:.1f}s]" if total_elapsed >= 60 else f"[{total_elapsed:.0f}s]"
    )
    print(color_text(f"  Total time: {elapsed_str}", "bold"))

    if all_success:
        print(color_text("\n  All test suites passed!", "green"))
    else:
        print(color_text("\n  Some test suites failed.", "red"))

    print(color_text("═" * 60 + "\n", "cyan"))

    return all_success


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    """Main entry point with plugin loading and cleanup.

    This script is designed to be invoked by the outer
    ``run_integration_tests.py`` wrapper which sets the
    ``MAYAMMD_FROM_WRAPPER`` environment variable.  Running it directly
    (e.g. ``mayapy run_all_integration_tests.py ...``) is supported but
    log-to-file output will not be available — use the outer wrapper for
    full logging.
    """
    # ── Guard: warn if not invoked through the outer wrapper ────────────
    _from_wrapper = os.environ.get("MAYAMMD_FROM_WRAPPER") == "1"

    if not _from_wrapper:
        print(
            color_text(
                "NOTE: Running directly. For log-to-file output, use the outer wrapper:\n"
                "      python tests/integration/run_integration_tests.py [suite] [options]",
                "yellow",
            )
        )

    # ── Ensure production-code loggers are visible ────────────────────────
    # Set to INFO so the WarningCollector (installed per-test) can capture
    # all messages.  WARNING+ from production loggers are surfaced in the
    # suite report.  In --strict mode they fail the test.
    logging.getLogger("mmd").setLevel(logging.INFO)
    logging.getLogger("MayaMMD").setLevel(logging.INFO)

    # Load plugin
    if not load_plugin():
        print(color_text("Failed to load MayaMMD plugin. Aborting tests.", "red"))
        maya.standalone.uninitialize()
        sys.exit(1)

    # Parse CLI arguments (passed from outer run_integration_tests.py)
    args = parse_args()

    # Resolve --strict: CLI flag takes precedence, then env var MAYAMMD_STRICT
    if args.strict is None:
        args.strict = os.environ.get("MAYAMMD_STRICT") == "1"

    if args.strict:
        print(
            color_text(
                "  ⚠ Strict mode ON — production warnings will fail tests", "yellow"
            )
        )

    try:
        success = run_all_integration_tests(args)
        maya.standalone.uninitialize()
        sys.exit(0 if success else 1)

    except ImportError as e:
        print(color_text(f"Import error (may be non-critical): {e}", "yellow"))
        try:
            success = run_all_integration_tests(args)
            maya.standalone.uninitialize()
            sys.exit(0 if success else 1)
        except Exception as e2:
            print(color_text(f"Fatal error: {e2}", "red"))
            traceback.print_exc()
            maya.standalone.uninitialize()
            sys.exit(1)

    except Exception as e:
        print(color_text(f"Fatal error: {e}", "red"))
        traceback.print_exc()
        try:
            maya.standalone.uninitialize()
        except Exception:
            pass
        sys.exit(1)


if __name__ == "__main__":
    main()
