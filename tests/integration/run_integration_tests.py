#!/usr/bin/env python
"""
run_integration_tests.py

Single entry point for all PMX/VMD/VPD integration tests with Maya Python.
All output is simultaneously written to a timestamped log file under
``test-logs/`` in the workspace root.

Usage:
    python tests/integration/run_integration_tests.py [suite] [options]

Examples:
    # Run all test suites with defaults
    python tests/integration/run_integration_tests.py

    # Run only VPD tests on a specific model
    python tests/integration/run_integration_tests.py vpd --model Tololo

    # Run VMD tests against all models with all motion files
    python tests/integration/run_integration_tests.py vmd --all-models --all-motions

    # Run VPD tests with a specific pose file
    python tests/integration/run_integration_tests.py vpd --pose 1.vpd
"""

import argparse
import io
import os
import re
import subprocess
import sys
from datetime import datetime, timezone

# Regex to strip ANSI escape sequences (color codes used by color_text())
_ANSI_RE = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


# Maya 2026 Python interpreter path — discovered via $MAYA_LOCATION, with
# a hardcoded fallback for environments where the env var is not set.
def _find_mayapy() -> str:
    maya_loc = os.environ.get("MAYA_LOCATION")
    if maya_loc:
        candidate = os.path.join(maya_loc, "bin", "mayapy.exe")
        if os.path.exists(candidate):
            return candidate
    fallback = r"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe"
    if os.path.exists(fallback):
        return fallback
    return fallback  # let the subprocess call fail with a clear error


MAYAPY = _find_mayapy()

# The only test script we ever invoke (run_all handles suite selection internally)
_INNER_SCRIPT = r"tests\integration\maya\run_all_integration_tests.py"

_SUITE_CHOICES = [
    "all",
    "import",
    "bone",
    "morph",
    "vmd",
    "vpd",
    "node",
    "ccd",
    "cmd",
    "multi",
    "bench",
    "context",
    "pose-tree",
]


# ---------------------------------------------------------------------------
# Tee: duplicate all print / subprocess output to both terminal and a log file
# ---------------------------------------------------------------------------


class _LogFile:
    """Manages a timestamped log file and provides a ``write`` + ``flush``
    interface that mirrors a file-like object so mayapy subprocess output
    can be duplicated easily."""

    def __init__(self) -> None:
        self._dir: str = ""
        self._path: str = ""
        self._file: io.TextIOWrapper | None = None

    # -- context manager interface -------------------------------------------

    def open(self, log_dir: str) -> None:
        """Open a timestamped log file in *log_dir* (created if needed)."""
        os.makedirs(log_dir, exist_ok=True)
        self._dir = log_dir
        timestamp = datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S")
        basename = f"maya-integration_{timestamp}.log"
        self._path = os.path.join(log_dir, basename)
        # Use utf-8-sig so Windows tools (Notepad) display Unicode correctly.
        # The handle is kept as object state and written across the suite, so a
        # `with` block (one-shot open/close) does not fit here.
        self._file = open(self._path, "w", encoding="utf-8-sig", buffering=1)  # noqa: SIM115

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None

    @property
    def path(self) -> str:
        return self._path

    # -- file-like write/flush for subprocess pipes -------------------------

    def write(self, text: str) -> None:
        if self._file is not None:
            self._file.write(text)

    def flush(self) -> None:
        if self._file is not None:
            self._file.flush()


# Single shared instance
_LOG_FILE = _LogFile()
_ORIGINAL_PRINT = print  # keep a reference to the real built-in


def _print_and_log(*args, **kwargs) -> None:
    """Like built-in ``print`` but also writes to the log file."""
    _ORIGINAL_PRINT(*args, **kwargs)
    # Capture what was printed into a string
    buf = io.StringIO()
    _ORIGINAL_PRINT(*args, file=buf, **kwargs)
    _LOG_FILE.write(_ANSI_RE.sub("", buf.getvalue()))
    _LOG_FILE.flush()


print = _print_and_log


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run PMX/VMD/VPD integration tests with Maya Python.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "suite",
        nargs="?",
        default="all",
        choices=_SUITE_CHOICES,
        help="Test suite to run (default: all)",
    )
    parser.add_argument(
        "--model",
        "-m",
        default=None,
        help="Only test models whose path contains this substring (case-insensitive).",
    )
    parser.add_argument(
        "--motion",
        default=None,
        help="Only use VMD motion files whose path contains this substring.",
    )
    parser.add_argument(
        "--pose",
        default=None,
        help="Only use VPD pose files whose path contains this substring.",
    )
    parser.add_argument(
        "--all-models",
        action="store_true",
        help="Test against every available model.",
    )
    parser.add_argument(
        "--all-motions",
        action="store_true",
        help="Use every available VMD motion file.",
    )
    parser.add_argument(
        "--all-poses",
        action="store_true",
        help="Use every available VPD pose file.",
    )
    parser.add_argument(
        "--vmd-sample-every",
        type=int,
        default=5,
        metavar="N",
        help="Sample every Nth VMD keyframe (default: 5).",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Treat production-code warnings as test failures.",
    )
    return parser


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def setup_logging(workspace_root: str) -> str:
    """Initialise the log file directory and return its path."""
    log_dir = os.path.join(workspace_root, "test-logs")
    _LOG_FILE.open(log_dir)
    return _LOG_FILE.path


def main():
    parser = build_parser()
    args = parser.parse_args()

    # Get workspace root (parent of tests/integration/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.abspath(os.path.join(script_dir, "..", ".."))

    # Initialise log file
    log_path = setup_logging(workspace_root)

    # ── Build mayapy command ──────────────────────────────────────────
    # CLI defaults are set by argparse — no separate TEST_CONFIG dict.
    # Suites that need all models get --all-models by default.
    use_all_models = args.all_models or args.suite not in ("vmd", "vpd")

    if args.suite == "bench":
        cmd = [
            MAYAPY,
            os.path.join(
                workspace_root, "tests", "benchmarks", "maya", "run_maya_benchmarks.py"
            ),
            "pmx-load",
        ]
        if args.model:
            cmd.extend(["--model", args.model])
        print("Running PMX loading benchmark...\n")
    else:
        inner_path = os.path.join(workspace_root, _INNER_SCRIPT)
        cmd = [MAYAPY, inner_path, args.suite]
        if args.model:
            cmd.extend(["--model", args.model])
        if args.motion and not args.all_motions:
            cmd.extend(["--motion", args.motion])
        if args.pose and not args.all_poses:
            cmd.extend(["--pose", args.pose])
        if use_all_models:
            cmd.append("--all-models")
        if args.all_motions:
            cmd.append("--all-motions")
        if args.all_poses:
            cmd.append("--all-poses")

    # Run the test with mayapy, piping its output through both terminal and log
    try:
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"  # avoid cp1252/cp1250 UnicodeErrors
        env["PYTHONUNBUFFERED"] = "1"  # prevent out-of-order log interleaving
        env["MAYAMMD_VMD_SAMPLE_EVERY"] = str(args.vmd_sample_every)
        env["MAYAMMD_FROM_WRAPPER"] = "1"  # signals inner runner it's invoked properly
        if args.strict:
            env["MAYAMMD_STRICT"] = "1"
        with subprocess.Popen(
            cmd,
            cwd=workspace_root,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
        ) as proc:
            # Stream output line-by-line
            if proc.stdout is not None:
                for line in proc.stdout:
                    try:
                        sys.__stdout__.write(line)
                    except UnicodeEncodeError:
                        # Windows console may be cp1250 — write ascii-safe fallback
                        sys.__stdout__.write(
                            line.encode("ascii", errors="replace").decode("ascii")
                        )
                    sys.__stdout__.flush()
                    # Strip ANSI color codes before writing to log file
                    _LOG_FILE.write(_ANSI_RE.sub("", line))
                    _LOG_FILE.flush()
            exit_code = proc.wait()

        if exit_code == 0:
            print("\n[OK] All tests passed!")
        else:
            print(f"\n[FAIL] Some tests failed (exit code: {exit_code})")

        return exit_code

    except FileNotFoundError:
        print(f"\n[FAIL] Error: Could not find mayapy at: {MAYAPY}")
        print("Please update MAYAPY path in this script.")
        return 1
    except Exception as e:
        print(f"\n[FAIL] Error running tests: {e}")
        return 1
    finally:
        _LOG_FILE.close()
        print(f"\nFull log saved to: {log_path}")


if __name__ == "__main__":
    sys.exit(main())
