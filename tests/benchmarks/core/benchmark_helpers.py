"""
benchmark_helpers.py

Shared utilities for benchmarking PMX parsing and PMX loading.

Provides a ``BenchmarkResult`` data class, a ``BenchmarkRunner`` context manager,
and a pretty-printing table formatter that outputs structured timing reports
to both the terminal and an optional JSON file for historical comparison.
"""

from __future__ import annotations

import dataclasses
import json
import os
import time
from collections.abc import Callable
from datetime import datetime, timezone

# ──────────────────────────────────────────────────────────────────────────────
# Data types
# ──────────────────────────────────────────────────────────────────────────────


@dataclasses.dataclass
class BenchmarkResult:
    """Timing result for a single benchmark iteration.

    Attributes:
        label:         Human-readable name (e.g. model filename).
        parse_time:    Seconds spent parsing the file.
        build_time:    Seconds spent building the Maya scene (0 for core benchmarks).
        total_time:    Wall-clock seconds for the entire operation.
        details:       Optional dict of extra timing columns (e.g. ``vmd_parse``,
                       ``vmd_apply``).  Keys become column headers in the table.
        note:          Optional extra info (file size, vertex count, etc.).
    """

    label: str
    parse_time: float
    build_time: float
    total_time: float
    details: dict[str, float] = dataclasses.field(default_factory=dict)
    note: str = ""


@dataclasses.dataclass
class BenchmarkReport:
    """Aggregated report from a full benchmark run."""

    suite_name: str
    timestamp: str
    results: list[BenchmarkResult]
    total_parse_time: float = 0.0
    total_build_time: float = 0.0
    total_time: float = 0.0

    def __post_init__(self) -> None:
        self.total_parse_time = sum(r.parse_time for r in self.results)
        self.total_build_time = sum(r.build_time for r in self.results)
        self.total_time = sum(r.total_time for r in self.results)


# ──────────────────────────────────────────────────────────────────────────────
# Timing utilities
# ──────────────────────────────────────────────────────────────────────────────


class Timer:
    """Simple wall-clock stopwatch."""

    def __init__(self) -> None:
        self._start: float | None = None
        self.elapsed: float = 0.0

    def start(self) -> None:
        """Start (or restart) the timer."""
        self._start = time.perf_counter()

    def stop(self) -> float:
        """Stop the timer and return elapsed seconds."""
        if self._start is None:
            raise RuntimeError("Timer was never started")
        self.elapsed = time.perf_counter() - self._start
        return self.elapsed


# ──────────────────────────────────────────────────────────────────────────────
# Table formatting
# ──────────────────────────────────────────────────────────────────────────────

_COL_SEP = "  │  "


def _fmt_seconds(seconds: float) -> str:
    """Format a duration nicely (e.g. ``1.234 s`` or ``0.012 s``)."""
    return f"{seconds:.3f}s"


def _fmt_mb(bytes_size: int) -> str:
    """Format a byte size in MB."""
    return f"{bytes_size / (1024 * 1024):.1f} MB"


def _collect_detail_keys(results: list[BenchmarkResult]) -> list[str]:
    """Collect all detail column names that appear across results, preserving
    the order they first appear."""
    seen: dict[str, int] = {}
    for r in results:
        for key in r.details:
            if key not in seen:
                seen[key] = len(seen)
    return sorted(seen, key=seen.__getitem__)


def _detail_width(detail_keys: list[str]) -> int:
    """Compute column width for detail columns (header or value)."""
    if not detail_keys:
        return 0
    # Each detail column shows values like "0.123s" (8 chars) plus separators
    return max(len(k) for k in detail_keys) + 2  # +2 for padding


def _detail_header(detail_keys: list[str], w: int) -> str:
    """Build the header fragment for detail columns."""
    parts = []
    for k in detail_keys:
        label = k.replace("_", " ").title()
        parts.append(f"{label:>{w}}")
    return _COL_SEP.join(parts)


def _detail_value(value: float, w: int) -> str:
    return f"{_fmt_seconds(value):>{w}}"


def print_benchmark_table(report: BenchmarkReport) -> None:
    """Print a human-readable benchmark results table to stdout.

    If any result carries a ``details`` dict, extra columns are automatically
    added between the ``Total Time`` and ``Note`` columns.

    Args:
        report: Aggregated benchmark report to display.
    """
    print(f"═══ {report.suite_name} ═══")
    print(f"  Timestamp: {report.timestamp}")
    print()

    # Column widths
    col_label = max(len(r.label) for r in report.results) if report.results else 10
    col_label = max(col_label, len("Model / Label"))
    col_label = min(col_label, 60)  # cap width

    # Detail columns (auto-discovered)
    detail_keys = _collect_detail_keys(report.results)
    dw = _detail_width(detail_keys) if detail_keys else 0

    # Build header dynamically
    header_parts = [
        f"{'Model / Label':<{col_label}}",
        f"{'Parse Time':>10}",
        f"{'Build Time':>10}",
        f"{'Total Time':>10}",
    ]
    if detail_keys:
        header_parts.append(_detail_header(detail_keys, dw))
    header_parts.append(f"{'Note'}")
    header = _COL_SEP.join(header_parts)
    sep = "─" * len(header)

    print(sep)
    print(header)
    print(sep)

    # Rows
    for r in report.results:
        label = r.label[:col_label]  # truncate if too long
        parts = [
            f"{label:<{col_label}}",
            f"{_fmt_seconds(r.parse_time):>10}",
            f"{_fmt_seconds(r.build_time):>10}",
            f"{_fmt_seconds(r.total_time):>10}",
        ]
        if detail_keys:
            vals = [_detail_value(r.details.get(k, 0.0), dw) for k in detail_keys]
            parts.append(_COL_SEP.join(vals))
        parts.append(f"{r.note}")
        print(_COL_SEP.join(parts))

    # Totals row
    if len(report.results) > 1:
        print(sep)
        total_parts = [
            f"{'TOTAL':<{col_label}}",
            f"{_fmt_seconds(report.total_parse_time):>10}",
            f"{_fmt_seconds(report.total_build_time):>10}",
            f"{_fmt_seconds(report.total_time):>10}",
        ]
        if detail_keys:
            total_detail = {}
            for k in detail_keys:
                total_detail[k] = sum(r.details.get(k, 0.0) for r in report.results)
            vals = [_detail_value(total_detail[k], dw) for k in detail_keys]
            total_parts.append(_COL_SEP.join(vals))
        total_parts.append(f"{len(report.results)} item(s)")
        print(_COL_SEP.join(total_parts))

    print(sep)
    print()


# ──────────────────────────────────────────────────────────────────────────────
# JSON export (for historical comparison)
# ──────────────────────────────────────────────────────────────────────────────


def save_benchmark_report(
    report: BenchmarkReport,
    output_dir: str,
) -> str:
    """Save *report* as a timestamped JSON file under *output_dir*.

    Args:
        report:     The benchmark report to persist.
        output_dir: Directory where the JSON file will be written.

    Returns:
        Absolute path to the saved JSON file.
    """
    os.makedirs(output_dir, exist_ok=True)

    # Build a filename-friendly timestamp
    ts = datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S")
    safe_name = report.suite_name.lower().replace(" ", "_").replace(":", "")
    filename = f"benchmark_{safe_name}_{ts}.json"
    filepath = os.path.join(output_dir, filename)

    data = {
        "suite": report.suite_name,
        "timestamp": report.timestamp,
        "results": [
            {
                "label": r.label,
                "parse_time_s": round(r.parse_time, 4),
                "build_time_s": round(r.build_time, 4),
                "total_time_s": round(r.total_time, 4),
                "details": {k: round(v, 4) for k, v in r.details.items()},
                "note": r.note,
            }
            for r in report.results
        ],
        "totals": {
            "parse_time_s": round(report.total_parse_time, 4),
            "build_time_s": round(report.total_build_time, 4),
            "total_time_s": round(report.total_time, 4),
            "model_count": len(report.results),
        },
    }

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    return filepath


# ──────────────────────────────────────────────────────────────────────────────
# Benchmark runner decorator / context
# ──────────────────────────────────────────────────────────────────────────────


def run_benchmarks(
    suite_name: str,
    items: list[tuple[str, Callable[[], float], Callable[[], float]]],
    output_dir: str = "",
) -> BenchmarkReport:
    """Run a collection of benchmark items and produce a report.

    Each item is a 3-tuple: ``(label, parse_fn, build_fn)``.

    - ``label`` appears in the output table.
    - ``parse_fn`` is called first; its return value (float seconds) is the
      parse time.  If there is no parse step, pass ``lambda: 0.0``.
    - ``build_fn`` is called after parse; its return value is the build time.
      If there is no build step, pass ``lambda: 0.0``.

    Args:
        suite_name:  Name of this benchmark suite (shown in the report header).
        items:       Iterable of ``(label, parse_fn, build_fn)`` tuples.
        output_dir:  If non-empty, save a JSON report to this directory.

    Returns:
        A ``BenchmarkReport`` with all results.
    """
    results: list[BenchmarkResult] = []

    for label, parse_fn, build_fn in items:
        # Parse phase
        t0 = time.perf_counter()
        parse_time = parse_fn()
        _t1 = time.perf_counter()

        # Build phase
        build_time = build_fn()
        t2 = time.perf_counter()

        # The fn calls already timed their own work; we also capture wall time
        total = t2 - t0

        results.append(
            BenchmarkResult(
                label=label,
                parse_time=parse_time,
                build_time=build_time,
                total_time=total,
            )
        )

    report = BenchmarkReport(
        suite_name=suite_name,
        timestamp=datetime.now(tz=timezone.utc).isoformat(timespec="seconds"),
        results=results,
    )

    # Display
    print_benchmark_table(report)

    # Save JSON if requested
    if output_dir:
        path = save_benchmark_report(report, output_dir)
        print(f"  JSON report saved to: {path}\n")

    return report
