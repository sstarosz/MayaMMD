"""
test_vpd_parsing_benchmark.py

Benchmark for VPD pose file parsing performance (no Maya dependency).

Measures how long :func:`mmd.core.vpd_importer.parse_vpd_file` takes for each
pose file in the asset library.  Can be run with **pytest** or directly as a
standalone script.

Usage:
    # Run with pytest
    pytest tests/benchmarks/core/test_vpd_parsing_benchmark.py -v

    # Run standalone for a summary table
    python tests/benchmarks/core/test_vpd_parsing_benchmark.py

    # Run standalone, saving JSON report
    python tests/benchmarks/core/test_vpd_parsing_benchmark.py --save-json
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

import pytest  # noqa: E402

from assets.assets_utils import get_all_vpd_paths  # noqa: E402
from mmd.core.vpd_importer import parse_vpd_file  # noqa: E402

from tests.benchmarks.core.benchmark_helpers import (  # noqa: E402
    BenchmarkReport,
    run_benchmarks,
)

# ── Globals ───────────────────────────────────────────────────────────────

_ALL_POSES = get_all_vpd_paths()
_BENCHMARK_SUITE = "VPD Parsing Benchmark"

# Pick a warm-up pose from the collection
_WARMUP_POSE = next(
    (p for p in _ALL_POSES if "1.vpd" in p),
    _ALL_POSES[0] if _ALL_POSES else None,
)


def _warmup() -> None:
    """Parse one pose to warm filesystem / Python caches."""
    if _WARMUP_POSE:
        _ = parse_vpd_file(_WARMUP_POSE)


def _file_size_mb(path: str) -> str:
    try:
        size = os.path.getsize(path)
        return f"{size / (1024 * 1024):.1f} MB"
    except OSError:
        return "?"


# ---------------------------------------------------------------------------
# Pytest parametrised tests (one per pose file)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "pose_path",
    _ALL_POSES,
    ids=[os.path.basename(p) for p in _ALL_POSES],
)
def test_parse_vpd_speed(pose_path: str) -> None:
    """Measure :func:`parse_vpd_file` time for a single pose file."""
    pose_name = os.path.basename(pose_path)
    size = _file_size_mb(pose_path)

    t0 = time.perf_counter()
    result = parse_vpd_file(pose_path)
    elapsed = time.perf_counter() - t0

    print(f"  {pose_name}  ({size})  parsed in {elapsed:.3f}s")

    # Sanity check
    assert result is not None
    assert hasattr(result, "bone_count")


# ---------------------------------------------------------------------------
# Standalone runner (summary table + optional JSON export)
# ---------------------------------------------------------------------------


def _run_standalone(save_json: bool = False) -> BenchmarkReport:
    """Parse all pose files, print a summary table, optionally save JSON."""
    _warmup()

    output_dir = ""
    if save_json:
        output_dir = os.path.join(_PROJECT_ROOT, "test-logs", "benchmarks")

    def _make_items():
        for pose_path in _ALL_POSES:
            label = os.path.basename(pose_path)
            size_note = _file_size_mb(pose_path)

            def _parse(p: str = pose_path) -> float:
                t0 = time.perf_counter()
                _ = parse_vpd_file(p)
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
