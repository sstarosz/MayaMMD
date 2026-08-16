"""
vpd_loading_benchmark.py

Benchmark for VPD pose file parsing and scene application (requires Maya).

Measures per-pose:
  1. Time to parse a VPD file (:func:`mmd.core.vpd_importer.parse_vpd_file`).
  2. Time to build the PMX scene (:func:`mmd.maya.pmx_scene_builder.build_pmx_scene`).
  3. Time to apply the VPD pose to the scene
     (:func:`mmd.maya.vpd_scene_builder.apply_vpd_pose_to_scene`).

Uses a single model (TololoDefault) and a limited set of pose files (default
7 poses from Pose Pack #15) so the benchmark completes in reasonable time.
Maya standalone must be initialised before importing this module.
"""

from __future__ import annotations

import os
import sys
import time

# ── Ensure workspace root is on sys.path ─────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from maya import cmds

from assets.assets_utils import get_all_pmx_model_paths, get_all_vpd_paths
from mmd.core.pmx_importer import parse_pmx
from mmd.core.vpd_importer import parse_vpd_file
from mmd.maya.pmx_scene_builder import build_pmx_scene
from mmd.maya.vpd_scene_builder import apply_vpd_pose_to_scene
from tests.benchmarks.core.benchmark_helpers import (
    BenchmarkReport,
    BenchmarkResult,
    print_benchmark_table,
    save_benchmark_report,
)

# ── Globals ───────────────────────────────────────────────────────────────

_ALL_MODELS = get_all_pmx_model_paths()
_ALL_POSES = get_all_vpd_paths()

# Select TololoDefault as the single test model
_TEST_MODEL = next(
    (m for m in _ALL_MODELS if "GirlsFrontline TololoDefault.pmx" in m),
    _ALL_MODELS[0] if _ALL_MODELS else None,
)

# Use same default pose selection as the integration tests: poses 1-7
_TEST_POSES = [v for v in _ALL_POSES if any(f"{i}.vpd" in v for i in range(1, 8))]

_BENCHMARK_SUITE = "VPD Loading Benchmark"


# ── Warmup ────────────────────────────────────────────────────────────────


def _warmup() -> None:
    """Parse + build one model + apply one pose to warm caches."""
    if _TEST_MODEL and _TEST_POSES:
        _setup_scene()
        pmx = parse_pmx(_TEST_MODEL)
        _ = build_pmx_scene(pmx)
        vpd_data = parse_vpd_file(_TEST_POSES[0])
        _setup_scene()
        maya_data = build_pmx_scene(pmx)
        apply_vpd_pose_to_scene(
            vpd_data=vpd_data,
            model=maya_data.to_resolved(),
            create_keyframe=False,
        )


# ── Helpers ───────────────────────────────────────────────────────────────


def _setup_scene() -> None:
    cmds.file(new=True, force=True)


def _file_size_mb(path: str) -> str:
    try:
        size = os.path.getsize(path)
        return f"{size / (1024 * 1024):.1f} MB"
    except OSError:
        return "?"


# ---------------------------------------------------------------------------
# Benchmark runner
# ---------------------------------------------------------------------------


def run_vpd_loading_benchmarks(
    model_filter: str | None = None,
) -> BenchmarkReport:
    """Run VPD loading benchmarks.

    For the selected model, each pose file is benchmarked independently:
      1. Parse PMX file (once, cached).
      2. Build Maya scene (once per pose — each pose needs a fresh scene).
      3. Parse the VPD file.
      4. Apply the VPD pose to the scene.

    Args:
        model_filter: Optional substring to filter model paths (case-insensitive).

    Returns:
        Aggregated ``BenchmarkReport``.
    """
    # Warm-up
    _warmup()

    # Select model (single only for speed)
    if model_filter:
        models = [m for m in _ALL_MODELS if model_filter.lower() in m.lower()]
    else:
        models = [m for m in _ALL_MODELS if _TEST_MODEL and m == _TEST_MODEL]

    if not models:
        print("ERROR: No models matched the filter.")
        return BenchmarkReport(
            suite_name=_BENCHMARK_SUITE,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
            results=[],
        )

    # Use only the first model (single-model benchmark for speed)
    model_path = models[0]
    model_label = os.path.basename(model_path)
    model_size = _file_size_mb(model_path)

    poses = _TEST_POSES
    if not poses:
        print("ERROR: No VPD pose files found.")
        return BenchmarkReport(
            suite_name=_BENCHMARK_SUITE,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
            results=[],
        )

    output_dir = os.path.join(_PROJECT_ROOT, "test-logs", "benchmarks")

    # ── Parse PMX once (cached, no Maya needed) ─────────────────────────
    pmx_data = parse_pmx(model_path)
    pmx_parse_time = 0.0  # not included per-pose; reported separately below

    # Warm-up build
    _setup_scene()
    _ = build_pmx_scene(pmx_data)

    results: list[BenchmarkResult] = []

    # ── Benchmark each pose ──────────────────────────────────────────────
    for pose_path in poses:
        pose_label = os.path.basename(pose_path)
        pose_size = _file_size_mb(pose_path)
        full_label = (
            f"{model_label} + {pose_label}  (model:{model_size}, pose:{pose_size})"
        )

        # Parse VPD file (no Maya needed)
        t0 = time.perf_counter()
        vpd_data = parse_vpd_file(pose_path)
        t1 = time.perf_counter()
        vpd_parse_time = t1 - t0

        # Build fresh scene + apply pose
        _setup_scene()
        maya_data = build_pmx_scene(pmx_data)
        t2 = time.perf_counter()
        scene_build_time = t2 - t1

        # Apply VPD pose
        apply_vpd_pose_to_scene(
            vpd_data=vpd_data,
            model=maya_data.to_resolved(),
            create_keyframe=False,
        )
        t3 = time.perf_counter()
        vpd_apply_time = t3 - t2

        total = t3 - t0

        results.append(
            BenchmarkResult(
                label=full_label,
                parse_time=pmx_parse_time,
                build_time=scene_build_time,
                total_time=total,
                details={
                    "vpd_parse": vpd_parse_time,
                    "vpd_apply": vpd_apply_time,
                },
                note=(f"scene_build={scene_build_time:.3f}s, total={total:.3f}s"),
            )
        )

    report = BenchmarkReport(
        suite_name=_BENCHMARK_SUITE,
        timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
        results=results,
    )

    # Display
    print_benchmark_table(report)

    # Save JSON to test-logs/benchmarks/
    path = save_benchmark_report(report, output_dir)
    print(f"  JSON report saved to: {path}\n")

    return report


# ---------------------------------------------------------------------------
# Standalone entry point
# ---------------------------------------------------------------------------


def main() -> int:
    """Run VPD loading benchmarks standalone (initialises Maya)."""
    import maya.standalone

    maya.standalone.initialize()

    from tests.integration.test_helpers import load_plugin

    if not load_plugin():
        print("ERROR: Failed to load MayaMMD plugin.")
        return 1

    from maya import mel

    print(f"Maya version: {mel.eval('getApplicationVersionAsFloat()')}")

    run_vpd_loading_benchmarks()
    return 0


if __name__ == "__main__":
    sys.exit(main())
