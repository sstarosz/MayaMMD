"""
test_pmx_parsing_benchmark.py

Benchmark for PMX file parsing performance (no Maya dependency).

Measures how long :func:`mmd.core.pmx_importer.parse_pmx` takes for each
model in the asset library.  Can be run with **pytest** or directly as a
standalone script.

Usage:
    # Run with pytest (shows per-model timing)
    pytest tests/benchmarks/core/test_pmx_parsing_benchmark.py -v

    # Run standalone for a summary table
    python tests/benchmarks/core/test_pmx_parsing_benchmark.py

    # Run standalone, saving a JSON report for trend analysis
    python tests/benchmarks/core/test_pmx_parsing_benchmark.py --save-json
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

from assets.assets_utils import get_all_pmx_model_paths  # noqa: E402
from mmd.core.pmx_importer import parse_pmx  # noqa: E402

from tests.benchmarks.core.benchmark_helpers import (  # noqa: E402
    BenchmarkReport,
    run_benchmarks,
)

# ── Globals ───────────────────────────────────────────────────────────────

_ALL_MODELS = get_all_pmx_model_paths()
_BENCHMARK_SUITE = "PMX Parsing Benchmark"

# Pick a warm-up model from the collection
_WARMUP_MODEL = next(
    (m for m in _ALL_MODELS if "TololoDefault" in m),
    _ALL_MODELS[0] if _ALL_MODELS else None,
)


def _warmup() -> None:
    """Parse one model to warm filesystem / Python caches."""
    if _WARMUP_MODEL:
        _ = parse_pmx(_WARMUP_MODEL)


def _file_size_mb(path: str) -> str:
    try:
        size = os.path.getsize(path)
        return f"{size / (1024 * 1024):.1f} MB"
    except OSError:
        return "?"


# ---------------------------------------------------------------------------
# Pytest parametrised tests (one per model)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "model_path",
    _ALL_MODELS,
    ids=[os.path.basename(p) for p in _ALL_MODELS],
)
def test_parse_pmx_speed(model_path: str) -> None:
    """Measure :func:`parse_pmx` time for a single model."""
    model_name = os.path.basename(model_path)
    size = _file_size_mb(model_path)

    t0 = time.perf_counter()
    result = parse_pmx(model_path)
    elapsed = time.perf_counter() - t0

    print(f"  {model_name}  ({size})  parsed in {elapsed:.3f}s")

    # Sanity check: result must be valid
    assert result is not None
    assert hasattr(result, "header")


# ---------------------------------------------------------------------------
# Standalone runner (summary table + optional JSON export)
# ---------------------------------------------------------------------------


def _run_standalone(save_json: bool = False) -> BenchmarkReport:
    """Parse all models, print a summary table, optionally save JSON."""
    _warmup()

    output_dir = ""
    if save_json:
        output_dir = os.path.join(_PROJECT_ROOT, "test-logs", "benchmarks")

    def _make_items():
        for model_path in _ALL_MODELS:
            label = os.path.basename(model_path)
            size_note = _file_size_mb(model_path)

            def _parse(p: str = model_path) -> float:
                t0 = time.perf_counter()
                _ = parse_pmx(p)
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
