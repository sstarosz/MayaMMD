"""
pmx_loading_benchmark.py

Benchmark for PMX model loading performance in Maya (requires Maya standalone).

Measures how long :func:`mmd.core.pmx_importer.parse_pmx` and
:func:`mmd.maya.pmx_scene_builder.build_pmx_scene` take for each model
in the asset library.

This script is meant to be invoked via ``run_maya_benchmarks.py``
(which initialises Maya standalone), but can also be imported and used
directly as long as Maya is already initialised.

Output:
    - Timed summary table printed to stdout.
    - Optional JSON report saved to ``test-logs/benchmarks/``.
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

from assets.assets_utils import get_all_pmx_model_paths  # noqa: E402
from mmd.core.pmx_importer import parse_pmx  # noqa: E402
from mmd.maya.pmx_scene_builder import build_pmx_scene  # noqa: E402

from tests.benchmarks.core.benchmark_helpers import (  # noqa: E402
    BenchmarkReport,
    BenchmarkResult,
    print_benchmark_table,
    save_benchmark_report,
)

# Maya imports (caller must initialise standalone before these)
import maya.cmds as cmds  # noqa: E402

# ── Globals ───────────────────────────────────────────────────────────────

_ALL_MODELS = get_all_pmx_model_paths()
_BENCHMARK_SUITE = "PMX Loading Benchmark"

# ── Helpers ───────────────────────────────────────────────────────────────


def _setup_scene() -> None:
    """Reset Maya to a clean empty scene."""
    cmds.file(new=True, force=True)


def _file_size_mb(path: str) -> str:
    try:
        size = os.path.getsize(path)
        return f"{size / (1024 * 1024):.1f} MB"
    except OSError:
        return "?"


def _warmup(model_path: str | None = None) -> None:
    """Parse + build one small model to warm caches."""
    if model_path is None:
        model_path = next(
            (m for m in _ALL_MODELS if "TololoDefault" in m),
            _ALL_MODELS[0] if _ALL_MODELS else None,
        )
    if model_path:
        _setup_scene()
        pmx = parse_pmx(model_path)
        _ = build_pmx_scene(pmx)


# ---------------------------------------------------------------------------
# Benchmark runner
# ---------------------------------------------------------------------------


def run_pmx_loading_benchmarks(
    model_filter: str | None = None,
) -> BenchmarkReport:
    """Run PMX loading benchmarks against selected (or all) models.

    Args:
        model_filter: Optional substring to filter model paths (case-insensitive).
        save_json:    If ``True``, write a JSON report to ``test-logs/benchmarks/``.

    Returns:
        Aggregated ``BenchmarkReport``.
    """
    # Select models
    models = list(_ALL_MODELS)
    if model_filter:
        models = [
            m
            for m in models
            if (
                model_filter.lower() in m.lower()
                if os.name == "nt"
                else model_filter in m
            )
        ]

    if not models:
        print("ERROR: No models matched the filter.")
        return BenchmarkReport(
            suite_name=_BENCHMARK_SUITE,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
            results=[],
        )

    # Warm-up
    _warmup()

    output_dir = os.path.join(_PROJECT_ROOT, "test-logs", "benchmarks")

    # Build results list directly (no generator complexity)
    results: list[BenchmarkResult] = []

    for model_path in models:
        label = os.path.basename(model_path)
        size_note = _file_size_mb(model_path)
        full_label = f"{label}  ({size_note})"

        # Parse + build in a fresh scene
        _setup_scene()

        t0 = time.perf_counter()
        pmx = parse_pmx(model_path)
        t1 = time.perf_counter()

        _ = build_pmx_scene(pmx)
        t2 = time.perf_counter()

        parse_time = t1 - t0
        build_time = t2 - t1
        total_time = t2 - t0

        results.append(
            BenchmarkResult(
                label=full_label,
                parse_time=parse_time,
                build_time=build_time,
                total_time=total_time,
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
    """Run PMX loading benchmarks standalone (initialises Maya)."""
    import maya.standalone  # noqa: E402

    maya.standalone.initialize()

    from tests.integration.test_helpers import load_plugin  # noqa: E402

    if not load_plugin():
        print("ERROR: Failed to load MayaMMD plugin.")
        return 1

    import maya.mel as mel  # noqa: E402

    print(f"Maya version: {mel.eval('getApplicationVersionAsFloat()')}")

    run_pmx_loading_benchmarks()
    return 0


if __name__ == "__main__":
    sys.exit(main())
