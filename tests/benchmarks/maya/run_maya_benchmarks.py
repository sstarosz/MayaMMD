"""
run_maya_benchmarks.py

Benchmark runner for all Maya-based benchmarks — invoked by mayapy.

This script is the Maya-side entry point analogous to
``run_all_integration_tests.py`` for integration tests.  It is meant to be
called from ``tests/benchmarks/run_benchmarks.py`` (the outer CLI wrapper).

Usage (via outer wrapper):
    python tests/benchmarks/run_benchmarks.py pmx-load [--model FILTER] [--save-json]
    python tests/benchmarks/run_benchmarks.py vmd-load [--model FILTER] [--save-json]
    python tests/benchmarks/run_benchmarks.py vpd-load [--model FILTER] [--save-json]
"""

from __future__ import annotations

import argparse
import os
import sys

# ── Add workspace root to sys.path ──────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

# ── Maya standalone MUST be initialised BEFORE any maya.* imports ───────────
import maya.standalone

maya.standalone.initialize()

# ── Project / benchmark imports ─────────────────────────────────────────────
from tests.integration.test_helpers import load_plugin

_SUITE_CHOICES = ["pmx-load", "vmd-load", "vpd-load"]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Maya benchmark runner (pmx-load | vmd-load | vpd-load)."
    )
    parser.add_argument(
        "suite",
        nargs="?",
        default="pmx-load",
        choices=_SUITE_CHOICES,
        help="Which Maya benchmark suite to run (default: pmx-load).",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Substring filter for model paths (case-insensitive).",
    )
    return parser.parse_args(argv)


def main() -> int:
    # Import mel after standalone initialisation
    from maya import mel

    args = parse_args()

    print(f"\nStarting Maya Benchmarks — suite: {args.suite}")
    print(f"Maya version: {mel.eval('getApplicationVersionAsFloat()')}")

    # Load the MayaMMD plugin (required for boneMorphNode / boneBlendShape)
    if not load_plugin():
        print("ERROR: Failed to load MayaMMD plugin. Aborting benchmarks.")
        return 1

    # ── Route to the correct benchmark suite ─────────────────────────────
    if args.suite == "pmx-load":
        from tests.benchmarks.maya.pmx_loading_benchmark import (
            run_pmx_loading_benchmarks,
        )

        report = run_pmx_loading_benchmarks(
            model_filter=args.model,
        )
    elif args.suite == "vmd-load":
        from tests.benchmarks.maya.vmd_loading_benchmark import (
            run_vmd_loading_benchmarks,
        )

        report = run_vmd_loading_benchmarks(
            model_filter=args.model,
        )
    elif args.suite == "vpd-load":
        from tests.benchmarks.maya.vpd_loading_benchmark import (
            run_vpd_loading_benchmarks,
        )

        report = run_vpd_loading_benchmarks(
            model_filter=args.model,
        )
    else:
        print(f"ERROR: Unknown suite '{args.suite}'")
        return 1

    maya.standalone.uninitialize()

    if not report.results:
        return 1

    print(f"Benchmark complete. Tested {len(report.results)} iteration(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
