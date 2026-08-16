"""
test_vmd_parsing_benchmark.py

Benchmark for VMD motion file parsing performance (no Maya dependency).

Measures how long :func:`mmd.core.vmd_importer.parse_vmd_file` takes for each
motion file in the asset library.  Can be run with **pytest** or directly as a
standalone script.

Usage:
    # Run with pytest
    pytest tests/benchmarks/core/test_vmd_parsing_benchmark.py -v

    # Run standalone for a summary table
    python tests/benchmarks/core/test_vmd_parsing_benchmark.py

    # Run standalone, saving JSON report
    python tests/benchmarks/core/test_vmd_parsing_benchmark.py --save-json
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

import pytest

from assets.assets_utils import get_all_vmd_paths
from mmd.core.vmd_importer import parse_vmd_file
from tests.benchmarks.core.benchmark_helpers import (
    BenchmarkReport,
    run_benchmarks,
)

# ── Globals ───────────────────────────────────────────────────────────────

_ALL_MOTIONS = get_all_vmd_paths()
_BENCHMARK_SUITE = "VMD Parsing Benchmark"

# Pick a warm-up motion from the collection
_WARMUP_MOTION = next(
    (m for m in _ALL_MOTIONS if "1.vmd" in m),
    _ALL_MOTIONS[0] if _ALL_MOTIONS else None,
)


def _warmup() -> None:
    """Parse one motion to warm filesystem / Python caches."""
    if _WARMUP_MOTION:
        _ = parse_vmd_file(_WARMUP_MOTION)


def _file_size_mb(path: str) -> str:
    try:
        size = os.path.getsize(path)
        return f"{size / (1024 * 1024):.1f} MB"
    except OSError:
        return "?"


# ---------------------------------------------------------------------------
# Pytest parametrised tests (one per motion file)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "motion_path",
    _ALL_MOTIONS,
    ids=[os.path.basename(p) for p in _ALL_MOTIONS],
)
def test_parse_vmd_speed(motion_path: str) -> None:
    """Measure :func:`parse_vmd_file` time for a single motion file."""
    motion_name = os.path.basename(motion_path)
    size = _file_size_mb(motion_path)

    t0 = time.perf_counter()
    result = parse_vmd_file(motion_path)
    elapsed = time.perf_counter() - t0

    print(f"  {motion_name}  ({size})  parsed in {elapsed:.3f}s")

    # Sanity check
    assert result is not None
    assert hasattr(result, "bone_keyframes")


# ---------------------------------------------------------------------------
# Standalone runner (summary table + optional JSON export)
# ---------------------------------------------------------------------------


def _run_standalone(save_json: bool = False) -> BenchmarkReport:
    """Parse all motion files, print a summary table, optionally save JSON."""
    _warmup()

    output_dir = ""
    if save_json:
        output_dir = os.path.join(_PROJECT_ROOT, "test-logs", "benchmarks")

    def _make_items():
        for motion_path in _ALL_MOTIONS:
            label = os.path.basename(motion_path)
            size_note = _file_size_mb(motion_path)

            def _parse(p: str = motion_path) -> float:
                t0 = time.perf_counter()
                _ = parse_vmd_file(p)
                return time.perf_counter() - t0

            yield (f"{label}  ({size_note})", _parse, lambda: 0.0)

    return run_benchmarks(
        suite_name=_BENCHMARK_SUITE,
        items=list(_make_items()),
        output_dir=output_dir,
    )


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> int:
    _run_standalone(save_json=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
