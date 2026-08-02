"""
vmd_loading_benchmark.py

Benchmark for VMD motion file parsing and scene application (requires Maya).

Measures:
  1. Time to parse a VMD file (:func:`mmd.core.vmd_importer.parse_vmd_file`).
  2. Time to build the PMX scene (:func:`mmd.maya.pmx_scene_builder.build_pmx_scene`).
  3. Time to apply the VMD animation to the scene
     (:func:`mmd.maya.vmd_scene_builder.apply_vmd_to_scene`).

Uses a single model (TololoDefault) and a single motion file (1.vmd) for a
fast, representative baseline.  Maya standalone must be initialised before
importing this module.
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

from assets.assets_utils import get_all_pmx_model_paths, get_all_vmd_paths  # noqa: E402
from mmd.core.pmx_importer import parse_pmx  # noqa: E402
from mmd.core.vmd_importer import parse_vmd_file  # noqa: E402
from mmd.maya.pmx_scene_builder import build_pmx_scene  # noqa: E402
from mmd.maya.vmd_scene_builder import apply_vmd_to_scene  # noqa: E402

from tests.benchmarks.core.benchmark_helpers import (  # noqa: E402
    BenchmarkReport,
    BenchmarkResult,
    print_benchmark_table,
    save_benchmark_report,
)

import maya.cmds as cmds  # noqa: E402

# ── Globals ───────────────────────────────────────────────────────────────

_ALL_MODELS = get_all_pmx_model_paths()
_ALL_MOTIONS = get_all_vmd_paths()

# Select TololoDefault as the single test model
_TEST_MODEL = next(
    (m for m in _ALL_MODELS if "GirlsFrontline TololoDefault.pmx" in m),
    _ALL_MODELS[0] if _ALL_MODELS else None,
)

# Select 1.vmd as the single test motion
_TEST_MOTION = next(
    (m for m in _ALL_MOTIONS if "1.vmd" in m),
    _ALL_MOTIONS[0] if _ALL_MOTIONS else None,
)

_BENCHMARK_SUITE = "VMD Loading Benchmark"


# ── Warmup ────────────────────────────────────────────────────────────────


def _warmup() -> None:
    """Parse + build one model + apply one motion to warm caches."""
    if _TEST_MODEL and _TEST_MOTION:
        _setup_scene()
        pmx = parse_pmx(_TEST_MODEL)
        maya_data = build_pmx_scene(pmx)
        vmd_data = parse_vmd_file(_TEST_MOTION)
        apply_vmd_to_scene(
            vmd_data=vmd_data,
            model=maya_data.to_resolved(),
            start_frame=1,
            apply_bone_anim=True,
            apply_morph_anim=True,
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


def run_vmd_loading_benchmarks(
    model_filter: str | None = None,
) -> BenchmarkReport:
    """Run VMD loading benchmarks.

    Benchmarks are split into three phases for each (model, motion) pair:
      1. Parse PMX file
      2. Build Maya scene
      3. Parse VMD file
      4. Apply VMD animation to scene

    Args:
        model_filter: Optional substring to filter model paths (case-insensitive).

    Returns:
        Aggregated ``BenchmarkReport``.
    """
    # Warm-up
    _warmup()

    # Select model
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

    motion_path = _TEST_MOTION
    if not motion_path:
        print("ERROR: No VMD motion file found.")
        return BenchmarkReport(
            suite_name=_BENCHMARK_SUITE,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
            results=[],
        )

    motion_size = _file_size_mb(motion_path)
    motion_label = os.path.basename(motion_path)

    output_dir = os.path.join(_PROJECT_ROOT, "test-logs", "benchmarks")

    results: list[BenchmarkResult] = []

    for model_path in models:
        model_label = os.path.basename(model_path)
        model_size = _file_size_mb(model_path)
        full_label = f"{model_label} + {motion_label}  (model:{model_size}, motion:{motion_size})"

        # ── Phase 1: Parse PMX ──────────────────────────────────────────
        _setup_scene()
        t0 = time.perf_counter()
        pmx = parse_pmx(model_path)
        t1 = time.perf_counter()
        pmx_parse_time = t1 - t0

        # ── Phase 2: Build scene ────────────────────────────────────────
        maya_data = build_pmx_scene(pmx)
        t2 = time.perf_counter()
        scene_build_time = t2 - t1

        # ── Phase 3: Parse VMD ──────────────────────────────────────────
        t3 = time.perf_counter()
        vmd_data = parse_vmd_file(motion_path)
        t4 = time.perf_counter()
        vmd_parse_time = t4 - t3

        # ── Phase 4: Apply VMD animation ────────────────────────────────
        apply_vmd_to_scene(
            vmd_data=vmd_data,
            model=maya_data.to_resolved(),
            start_frame=1,
            apply_bone_anim=True,
            apply_morph_anim=True,
        )
        t5 = time.perf_counter()
        vmd_apply_time = t5 - t4

        parse_time = pmx_parse_time
        build_time = scene_build_time
        total = t5 - t0

        results.append(
            BenchmarkResult(
                label=full_label,
                parse_time=parse_time,
                build_time=build_time,
                total_time=total,
                details={
                    "vmd_parse": vmd_parse_time,
                    "vmd_apply": vmd_apply_time,
                },
                note=(
                    f"pmx_parse={pmx_parse_time:.3f}s, "
                    f"scene_build={scene_build_time:.3f}s, "
                    f"total={total:.3f}s"
                ),
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
    """Run VMD loading benchmarks standalone (initialises Maya)."""
    import maya.standalone  # noqa: E402

    maya.standalone.initialize()

    from tests.integration.test_helpers import load_plugin  # noqa: E402

    if not load_plugin():
        print("ERROR: Failed to load MayaMMD plugin.")
        return 1

    import maya.mel as mel  # noqa: E402

    print(f"Maya version: {mel.eval('getApplicationVersionAsFloat()')}")

    run_vmd_loading_benchmarks()
    return 0


if __name__ == "__main__":
    sys.exit(main())
