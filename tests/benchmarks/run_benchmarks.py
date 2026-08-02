#!/usr/bin/env python
"""
run_benchmarks.py

Unified entry point for all benchmark suites (core + Maya).

Usage:
    # Run all benchmark suites (core + Maya)
    python tests/benchmarks/run_benchmarks.py

    # Run only the PMX parsing benchmark (no Maya needed)
    python tests/benchmarks/run_benchmarks.py parse

    # Run only the PMX loading benchmark
    python tests/benchmarks/run_benchmarks.py pmx-load

    # Run only the VMD loading benchmark
    python tests/benchmarks/run_benchmarks.py vmd-load

    # Run only the VPD loading benchmark
    python tests/benchmarks/run_benchmarks.py vpd-load

    # … with model filtering
    python tests/benchmarks/run_benchmarks.py pmx-load --model Tololo

Output:
    - Timed summary table printed to stdout.
    - JSON report saved to ``test-logs/benchmarks/`` for trend analysis.

Exit code:
    0 if all benchmarks ran successfully, 1 if any suite failed.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

# ── Workspace root ─────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))

# Maya 2026 Python interpreter
_MAYAPY = r"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe"

# ANSI colour helper (reuse same scheme as integration tests)
_COLORS = {
    "green": "\033[92m",
    "red": "\033[91m",
    "cyan": "\033[96m",
    "bold": "\033[1m",
    "reset": "\033[0m",
}


def _color(text: str, color: str) -> str:
    return f"{_COLORS.get(color, '')}{text}{_COLORS['reset']}"


# ══════════════════════════════════════════════════════════════════════════
# Suite definitions
# ══════════════════════════════════════════════════════════════════════════

_SUITE_CHOICES = [
    "all",
    "parse",
    "vmd-parse",
    "vpd-parse",
    "pmx-load",
    "vmd-load",
    "vpd-load",
]


def _run_core_script(script_name: str) -> bool:
    """Run a core benchmark script (pure Python, no Maya)."""
    cmd = [sys.executable, os.path.join(_HERE, "core", script_name)]
    result = subprocess.run(cmd, cwd=_PROJECT_ROOT)
    return result.returncode == 0


def run_parse_benchmark() -> bool:
    """Run the PMX parsing benchmark."""
    return _run_core_script("test_pmx_parsing_benchmark.py")


def run_vmd_parse_benchmark() -> bool:
    """Run the VMD parsing benchmark (no Maya)."""
    return _run_core_script("test_vmd_parsing_benchmark.py")


def run_vpd_parse_benchmark() -> bool:
    """Run the VPD parsing benchmark (no Maya)."""
    return _run_core_script("test_vpd_parsing_benchmark.py")


def _run_maya_suite(
    suite: str,
    model_filter: str | None,
) -> bool:
    """Run a Maya-based benchmark suite via mayapy.

    Args:
        suite:        Sub-suite name to pass to run_maya_benchmarks.py.
        model_filter: Optional model name filter.

    Returns:
        True on success.
    """
    maya_script = os.path.join(_HERE, "maya", "run_maya_benchmarks.py")
    cmd = [_MAYAPY, maya_script, suite]

    if model_filter:
        cmd.extend(["--model", model_filter])

    print(f"  Invoking mayapy: {' '.join(cmd)}")
    print()

    try:
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        env["PYTHONUNBUFFERED"] = "1"
        result = subprocess.run(
            cmd,
            cwd=_PROJECT_ROOT,
            capture_output=False,  # let output flow through
        )
        return result.returncode == 0
    except FileNotFoundError:
        print(
            _color(
                f"  ERROR: Could not find mayapy at: {_MAYAPY}\n"
                f"  Update _MAYAPY path in run_benchmarks.py if needed.",
                "red",
            )
        )
        return False
    except Exception as e:
        print(_color(f"  ERROR running mayapy: {e}", "red"))
        return False


def run_pmx_load_benchmark(
    model_filter: str | None,
) -> bool:
    """Run the PMX loading benchmark via mayapy."""
    return _run_maya_suite("pmx-load", model_filter)


def run_vmd_load_benchmark(
    model_filter: str | None,
) -> bool:
    """Run the VMD loading benchmark via mayapy."""
    return _run_maya_suite("vmd-load", model_filter)


def run_vpd_load_benchmark(
    model_filter: str | None,
) -> bool:
    """Run the VPD loading benchmark via mayapy."""
    return _run_maya_suite("vpd-load", model_filter)


# ══════════════════════════════════════════════════════════════════════════


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Benchmark suite for PMX parsing and PMX loading in Maya.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "suite",
        nargs="?",
        default="all",
        choices=_SUITE_CHOICES,
        help="Benchmark suite to run (default: all)",
    )
    parser.add_argument(
        "--model",
        "-m",
        default=None,
        help="Substring filter for model paths (case-insensitive).",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    suites_to_run: list[tuple[str, str]] = []
    if args.suite == "all":
        suites_to_run = [
            ("parse", "PMX Parsing"),
            ("vmd-parse", "VMD Parsing"),
            ("vpd-parse", "VPD Parsing"),
            ("pmx-load", "PMX Loading"),
            ("vmd-load", "VMD Loading"),
            ("vpd-load", "VPD Loading"),
        ]
    else:
        suite_names = {
            "parse": "PMX Parsing",
            "vmd-parse": "VMD Parsing",
            "vpd-parse": "VPD Parsing",
            "pmx-load": "PMX Loading",
            "vmd-load": "VMD Loading",
            "vpd-load": "VPD Loading",
        }
        suites_to_run = [(args.suite, suite_names.get(args.suite, args.suite))]

    all_passed = True
    results_log: list[tuple[str, bool]] = []

    for suite_key, suite_label in suites_to_run:
        success = False
        if suite_key == "parse":
            success = run_parse_benchmark()
        elif suite_key == "vmd-parse":
            success = run_vmd_parse_benchmark()
        elif suite_key == "vpd-parse":
            success = run_vpd_parse_benchmark()
        elif suite_key == "pmx-load":
            success = run_pmx_load_benchmark(model_filter=args.model)
        elif suite_key == "vmd-load":
            success = run_vmd_load_benchmark(model_filter=args.model)
        elif suite_key == "vpd-load":
            success = run_vpd_load_benchmark(model_filter=args.model)

        results_log.append((suite_label, success))
        all_passed = all_passed and success

    # ── Summary ────────────────────────────────────────────────────────
    print(_color("\n" + "═" * 60, "cyan"))
    print(_color("  Benchmark Summary", "bold"))
    print(_color("═" * 60, "cyan"))

    for label, ok in results_log:
        status = _color("✓ PASS", "green") if ok else _color("✗ FAIL", "red")
        print(f"  {status}  {label}")

    print()
    if all_passed:
        print(_color("  All benchmarks completed successfully.", "green"))
    else:
        print(_color("  Some benchmarks failed.", "red"))

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
