#!/usr/bin/env python
"""
plot_benchmarks.py

Parse benchmark JSON results from ``test-logs/benchmarks/`` and display
them as plots using matplotlib.

Usage:
    # Plot the latest run of every suite
    python tests/benchmarks/plot_benchmarks.py

    # Plot a specific JSON file
    python tests/benchmarks/plot_benchmarks.py test-logs/benchmarks/pmx_loading.json

    # Show only PMX-related suites
    python tests/benchmarks/plot_benchmarks.py --suite pmx

    # Save plot to a file instead of displaying
    python tests/benchmarks/plot_benchmarks.py --output benchmark_report.png

    # Show all historical runs (not just latest)
    python tests/benchmarks/plot_benchmarks.py --all-runs
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

# ── Workspace root ─────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, ".."))
_BENCHMARK_DIR = os.path.join(_PROJECT_ROOT, "test-logs", "benchmarks")
_DEFAULT_OUTPUT = os.path.join(_BENCHMARK_DIR, "benchmark_report.png")


def discover_json_files(
    benchmark_dir: str,
    suite_filter: str | None = None,
    latest_only: bool = True,
) -> list[str]:
    """Find benchmark JSON files, optionally filtered and deduped to latest.

    Args:
        benchmark_dir: Path to the directory containing JSON reports.
        suite_filter:  Optional substring to match against suite names.
        latest_only:   If True, keep only the most recent file per suite.

    Returns:
        Sorted list of absolute paths.
    """
    if not os.path.isdir(benchmark_dir):
        print(f"ERROR: Benchmark directory not found: {benchmark_dir}")
        return []

    all_json = sorted(
        Path(benchmark_dir).glob("benchmark_*.json"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )

    if suite_filter:
        all_json = [p for p in all_json if suite_filter.lower() in p.name.lower()]

    if not latest_only:
        return [str(p) for p in sorted(all_json)]

    # Keep only the latest file per suite
    seen_suites: dict[str, str] = {}
    for p in all_json:
        # Extract suite key from filename: benchmark_<suite>_<timestamp>.json
        stem = p.stem  # e.g. benchmark_pmx_loading_benchmark_20260727_173800
        # Remove leading "benchmark_" and trailing timestamp
        parts = stem.split("_")
        if len(parts) >= 3:
            # Timestamp is the last 2 parts: _YYYYMMDD_HHMMSS
            suite_key = "_".join(parts[1:-2])
        else:
            suite_key = stem
        if suite_key not in seen_suites:
            seen_suites[suite_key] = str(p)

    return sorted(seen_suites.values())


def load_report(filepath: str) -> dict | None:
    """Load a single benchmark JSON report.

    Returns:
        Parsed dict, or None on failure.
    """
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            return json.load(f)  # type: ignore[no-any-return]
    except (json.JSONDecodeError, OSError) as e:
        print(f"  WARNING: Could not parse {filepath}: {e}")
        return None


def _setup_matplotlib_font() -> None:
    """Configure matplotlib to use a CJK-capable font on Windows."""
    import matplotlib.pyplot as plt

    # Try Windows CJK fonts in order of preference
    cjk_fonts = [
        "Microsoft YaHei",
        "SimHei",
        "MS Gothic",
        "Meiryo",
    ]
    for font_name in cjk_fonts:
        try:
            plt.rcParams["font.family"] = font_name
            # Test that the font actually renders by checking a test figure
            fig = plt.figure()
            fig.text(0.5, 0.5, "测试", fontsize=10)
            plt.close(fig)
            return
        except Exception:
            continue

    # Fallback: use sans-serif and hope for the best
    plt.rcParams["font.family"] = "sans-serif"


def plot_benchmarks(
    json_files: list[str],
    output_path: str,
    show: bool = False,
) -> None:
    """Load all reports and render comparison plots.

    Each benchmark suite gets its own subplot with a horizontal stacked bar
    chart showing parse time (blue) and build time (orange).  Total time is
    annotated at the end of each bar.

    Args:
        json_files:  List of JSON file paths to plot.
        output_path: Path to save the figure to.
        show:        If True, also display the plot interactively.
    """
    # Group reports by suite name
    reports: list[dict] = []
    for fp in json_files:
        r = load_report(fp)
        if r and r.get("results"):
            reports.append(r)

    if not reports:
        print("ERROR: No valid benchmark data found.")
        return

    # ── Configure CJK font before any plotting ─────────────────────────
    _setup_matplotlib_font()

    # ── Determine grid layout ──────────────────────────────────────────
    import matplotlib.pyplot as plt
    import matplotlib.ticker as mticker

    n_suites = len(reports)
    cols = min(2, n_suites)
    rows = (n_suites + cols - 1) // cols

    fig, axes = plt.subplots(
        rows,
        cols,
        figsize=(max(10, cols * 7), max(5, rows * 5)),
        squeeze=False,
    )
    fig.suptitle(
        "Benchmark Results",
        fontsize=14,
        fontweight="bold",
        y=0.98,
    )

    for idx, report in enumerate(reports):
        ax = axes[idx // cols][idx % cols]
        suite = report.get("suite", "Unknown")
        timestamp = report.get("timestamp", "")
        results = report.get("results", [])

        # ── Prepare data ────────────────────────────────────────────────
        labels: list[str] = []
        parse_times: list[float] = []
        build_times: list[float] = []
        total_times: list[float] = []

        for r in results:
            raw_label = r.get("label", "?")
            # Truncate long labels
            if len(raw_label) > 50:
                raw_label = raw_label[:47] + "..."
            labels.append(raw_label)
            parse_times.append(r.get("parse_time_s", 0))
            build_times.append(r.get("build_time_s", 0))
            total_times.append(r.get("total_time_s", 0))

        # Reverse so longest bars are at the top
        labels.reverse()
        parse_times.reverse()
        build_times.reverse()
        total_times.reverse()

        y_pos = range(len(labels))

        # ── Horizontal stacked bars ─────────────────────────────────────
        bar_height = 0.6
        bars_parse = ax.barh(
            y_pos,
            parse_times,
            bar_height,
            label="Parse",
            color="#4C72B0",
            zorder=2,
        )
        bars_build = ax.barh(
            y_pos,
            build_times,
            bar_height,
            left=parse_times,
            label="Build",
            color="#DD8452",
            zorder=2,
        )

        # Annotate total time at end of each bar
        for i, total in enumerate(total_times):
            ax.text(
                total + max(total_times) * 0.01,
                i,
                f"{total:.2f}s",
                va="center",
                fontsize=7,
                color="#333333",
            )

        # ── Styling ─────────────────────────────────────────────────────
        ax.set_yticks(list(y_pos))
        ax.set_yticklabels(labels, fontsize=7)
        ax.set_xlabel("Time (seconds)", fontsize=8)
        ax.set_title(f"{suite}\n{timestamp}", fontsize=9, fontweight="bold")
        ax.xaxis.set_major_formatter(mticker.FormatStrFormatter("%.1fs"))
        ax.tick_params(labelsize=7)
        ax.invert_yaxis()
        ax.set_xlim(0, max(total_times) * 1.15 if total_times else 10)

        # Legend (only on first subplot)
        if idx == 0:
            ax.legend(loc="lower right", fontsize=7)

        # Grid
        ax.xaxis.grid(True, alpha=0.3, zorder=0)
        ax.set_axisbelow(True)

    # ── Hide unused subplots ────────────────────────────────────────────
    for idx in range(n_suites, rows * cols):
        axes[idx // cols][idx % cols].set_visible(False)

    fig.tight_layout(rect=[0, 0, 1, 0.95])

    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved to: {output_path}")

    if show:
        plt.show()

    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot benchmark results from JSON reports.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "files",
        nargs="*",
        help="Specific JSON file(s) to plot (default: auto-discover latest).",
    )
    parser.add_argument(
        "--suite",
        "-s",
        default=None,
        help="Substring filter for suite names (e.g. 'pmx', 'vmd').",
    )
    parser.add_argument(
        "--all-runs",
        action="store_true",
        help="Plot all historical runs, not just the latest per suite.",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=_DEFAULT_OUTPUT,
        help="Save plot to file (default: test-logs/benchmarks/benchmark_report.png).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Also display the plot interactively after saving.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    # Discover files
    if args.files:
        json_files = args.files
    else:
        json_files = discover_json_files(
            _BENCHMARK_DIR,
            suite_filter=args.suite,
            latest_only=not args.all_runs,
        )

    if not json_files:
        print("No benchmark JSON files found.")
        print(f"  Expected location: {_BENCHMARK_DIR}")
        print("  Run benchmarks first to generate data.")
        return 1

    print(f"Plotting {len(json_files)} benchmark report(s):")
    for f in json_files:
        print(f"  - {os.path.basename(f)}")

    plot_benchmarks(json_files, output_path=args.output, show=args.show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
