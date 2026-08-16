"""
test_helpers.py

Shared infrastructure for PMX/VMD integration tests.

Maya standalone is initialised by ``run_all_integration_tests.py`` before
any test module is imported, so individual test files no longer need to
call ``maya.standalone.initialize()`` themselves.
"""

from __future__ import annotations

import io
import logging
import math
import os
import sys
import time
import traceback
from collections.abc import Callable, Iterator
from contextlib import contextmanager, redirect_stdout
from dataclasses import dataclass, field
from typing import Self

# ---------------------------------------------------------------------------
# Maya imports – safe because every caller initialises standalone first
# ---------------------------------------------------------------------------
import maya.api.OpenMaya as om
from maya import cmds

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------
TestFunc = Callable[..., bool]
TestEntry = tuple[str, TestFunc]


class TestResult:
    """Outcome of a single test against a single model."""

    __slots__ = (
        "errors",
        "model_name",
        "output",
        "passed",
        "skipped",
        "strict_fail",
        "test_name",
        "warnings",
    )

    def __init__(
        self,
        model_name: str,
        test_name: str,
        passed: bool,
        output: str = "",
        skipped: bool = False,
        warnings: list[str] | None = None,
        errors: list[str] | None = None,
        strict_fail: bool = False,
    ) -> None:
        self.model_name = model_name
        self.test_name = test_name
        self.passed = passed
        self.skipped = skipped
        self.output = output  # captured stdout from the test function
        self.warnings: list[str] = warnings or []
        self.errors: list[str] = errors or []
        self.strict_fail = strict_fail  # True when --strict mode catches warnings


# ---------------------------------------------------------------------------
# Test assertions — lightweight pytest-style helpers for integration tests
# ---------------------------------------------------------------------------


class TestSkipped(Exception):
    """Raised by :func:`skip_test` to signal a test was intentionally skipped."""


class TestFailed(AssertionError):
    """Raised by ``assert_*`` helpers on failure.  Carries a user-facing message."""


def assert_true(condition: bool, msg: str = "") -> None:
    """Assert *condition* is truthy, raising :class:`TestFailed` if not.

    Usage inside a test function::

        assert_true(len(joints) == expected, "Joint count mismatch")
    """
    if not condition:
        raise TestFailed(msg or "assert_true failed")


def assert_eq(actual: object, expected: object, msg: str = "") -> None:
    """Assert ``actual == expected``, raising :class:`TestFailed` on mismatch.

    The failure message includes both values for diagnostics.
    """
    if actual != expected:
        detail = f"expected {expected!r}, got {actual!r}"
        raise TestFailed(f"{msg}: {detail}" if msg else detail)


def assert_approx(
    actual: float, expected: float, tolerance: float = 0.001, msg: str = ""
) -> None:
    """Assert ``abs(actual - expected) <= tolerance``."""
    if abs(actual - expected) > tolerance:
        detail = f"expected ≈{expected}, got {actual} (tol={tolerance})"
        raise TestFailed(f"{msg}: {detail}" if msg else detail)


def skip_test(reason: str = "") -> None:
    """Signal that a test should be skipped (e.g. missing data, not applicable).

    The test runner catches :class:`TestSkipped` and prints a ``⊘ SKIP`` line.
    """
    raise TestSkipped(reason)


# ---------------------------------------------------------------------------
# Test markers — decorators for categorising test functions
# ---------------------------------------------------------------------------


def matrix(func):
    """Mark a test function as part of the model × file matrix.

    Matrix tests are dispatched once per model × per pose/motion file.
    Non-matrix (unmarked) tests run once per model with all files available.

    Usage::

        @matrix
        def test_vpd_bone_rotation(pmx_data, maya_pmx_data):
            ...
    """
    func._is_matrix = True
    return func


# ---------------------------------------------------------------------------
# Math helpers — lightweight floating-point comparisons
# ---------------------------------------------------------------------------


def approx_equal(a: float, b: float, tolerance: float = 0.001) -> bool:
    """Return ``True`` if *a* and *b* differ by less than *tolerance*."""
    return abs(a - b) < tolerance


def approx_equal_tuple(
    a: tuple[float, ...], b: tuple[float, ...], tolerance: float = 0.001
) -> bool:
    """Return ``True`` if two tuples are element-wise within *tolerance*."""
    if len(a) != len(b):
        return False
    return all(abs(x - y) < tolerance for x, y in zip(a, b))


# ---------------------------------------------------------------------------
# Quaternion helpers — shared between VMD / VPD / bone-morph tests
# ---------------------------------------------------------------------------

# Maya MEulerRotation order constants, indexed by rotateOrder attribute value.
_ROTATE_ORDERS = [
    om.MEulerRotation.kXYZ,  # 0
    om.MEulerRotation.kYZX,  # 1
    om.MEulerRotation.kZXY,  # 2
    om.MEulerRotation.kXZY,  # 3
    om.MEulerRotation.kYXZ,  # 4
    om.MEulerRotation.kZYX,  # 5
]


def euler_degrees_to_quat(
    rx: float, ry: float, rz: float, rotate_order: int = 0
) -> om.MQuaternion:
    """Convert Euler angles in **degrees** to an :class:`~maya.api.OpenMaya.MQuaternion`.

    Args:
        rx, ry, rz: Rotation angles in degrees.
        rotate_order: Maya ``rotateOrder`` attribute value (0-5).
    """
    ro = (
        _ROTATE_ORDERS[rotate_order]
        if 0 <= rotate_order <= 5
        else om.MEulerRotation.kXYZ
    )
    return om.MEulerRotation(
        math.radians(rx), math.radians(ry), math.radians(rz), ro
    ).asQuaternion()


def quat_dot(a: om.MQuaternion, b: om.MQuaternion) -> float:
    """Absolute quaternion dot product — 1.0 = identical, 0.0 = orthogonal."""
    return abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z)


# ---------------------------------------------------------------------------
# Console utilities
# ---------------------------------------------------------------------------

_COLORS = {
    "green": "\033[92m",
    "red": "\033[91m",
    "yellow": "\033[93m",
    "blue": "\033[94m",
    "magenta": "\033[95m",
    "cyan": "\033[96m",
    "bold": "\033[1m",
    "reset": "\033[0m",
}


def color_text(text: str, color: str) -> str:
    """Wrap *text* in ANSI colour codes. Unknown colours are ignored."""
    return f"{_COLORS.get(color, '')}{text}{_COLORS['reset']}"


# ---------------------------------------------------------------------------
# Maya scene helpers
# ---------------------------------------------------------------------------


def setup_test_environment() -> None:
    """Open a brand-new empty Maya scene, discarding any previous state."""
    cmds.file(new=True, force=True)


def load_plugin(plugin_name: str = "MayaMMD") -> bool:
    """Load the MayaMMD Maya plugin.

    Tries Maya's standard plugin discovery via ``MAYA_PLUG_IN_PATH`` first.
    If that fails (e.g. standalone without env vars), falls back to locating
    the .mll relative to the project root.

    After loading the C++ .mll entry point, also loads the standalone Python
    plugin files that register commands (e.g. ``boneBlendShape``).
    ``registerCommand`` via ``findPlugin()`` does not work reliably in Maya
    API 2.0 standalone when called from within the C++ plugin's
    ``initializePlugin``, so the .py plugins must be loaded from outside
    that call chain as a fallback.

    Args:
        plugin_name: Maya plugin name (default ``"MayaMMD"``).

    Returns:
        ``True`` on success, ``False`` if loading fails.
    """
    # Resolve the project root — prefer env var, fall back to relative path
    project_root = os.environ.get(
        "MAYAMMD_PROJECT_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")),
    )

    try:
        # Load the C++ .mll — try by name first (MAYA_PLUG_IN_PATH), then by path
        if not cmds.pluginInfo(plugin_name, query=True, loaded=True):
            try:
                cmds.loadPlugin(plugin_name)
            except Exception:
                # Fallback: explicit path (useful when MAYA_PLUG_IN_PATH not set)
                mll_path = os.path.join(
                    project_root,
                    "out",
                    "install",
                    "maya2026-release",
                    "plug-ins",
                    "MayaMMD.mll",
                )
                if not os.path.exists(mll_path):
                    # Legacy fallback: old location in mmd/
                    mll_path = os.path.join(project_root, "mmd", "MayaMMD.mll")
                if os.path.exists(mll_path):
                    cmds.loadPlugin(mll_path)
                else:
                    raise FileNotFoundError(f"MayaMMD.mll not found at {mll_path}")

        # The C++ plugin (MayaMMD.mll) automatically calls mmd.plugin.initializePlugin(),
        # which registers boneMorphNode and boneBlendShape via findPlugin().
        # No additional Python plugin loading is needed.

        return True
    except Exception as exc:
        print(f"Failed to load plugin: {exc}")
        return False


def step_physics(node: str | None) -> None:
    """Force a fresh pmxRigidBodyNode solver evaluation at the current time.

    Only needed for headless/batch use (or to manually advance the sim) — in
    interactive Maya the node is time-driven and steps on its own.

    The node is an ``MPxLocatorNode``; a bare ``dgeval(node)`` does NOT
    reliably pull its custom solver outputs (it evaluates the DAG shape, not
    the ``outTranslate``/``outRotate`` plugs).  Demanding an output plug
    explicitly forces ``compute()`` to run.
    """
    if not node:
        return
    try:
        cmds.dgdirty(node)
        cmds.dgeval(f"{node}.outTranslate")
    except Exception:
        try:
            cmds.dgeval(node)
        except Exception as e:
            print(f"physics step dgeval failed: {e}")


def suppressed_undo() -> None:
    """Call ``cmds.undo()`` with script-editor output suppressed.

    Maya's undo command prints "Undo:" to C stdout which cannot be
    intercepted via ``redirect_stdout`` or ``commandEcho``.  Wrapping
    it in ``scriptEditorInfo(suppressResults=True, suppressInfo=True)``
    silences most (but not all) of the noise.
    """
    cmds.scriptEditorInfo(suppressResults=True, suppressInfo=True)
    try:
        cmds.undo()
    finally:
        cmds.scriptEditorInfo(suppressResults=False, suppressInfo=False)


def suppressed_redo() -> None:
    """Like :func:`suppressed_undo` but for ``cmds.redo()``."""
    cmds.scriptEditorInfo(suppressResults=True, suppressInfo=True)
    try:
        cmds.redo()
    finally:
        cmds.scriptEditorInfo(suppressResults=False, suppressInfo=False)


# ---------------------------------------------------------------------------
# Warning collection — captures production-code log messages per test
# ---------------------------------------------------------------------------

_PROD_LOGGER_PREFIXES = ("mmd.", "MayaMMD")


def _is_production_logger(name: str) -> bool:
    """Return ``True`` if *name* is a production-code logger."""
    return name.startswith(_PROD_LOGGER_PREFIXES)


class WarningCollector(logging.Handler):
    """A :class:`logging.Handler` that captures log records during a test.

    Install via :meth:`install`, run the test, then call :meth:`uninstall`
    to retrieve collected records.  Only WARNING and above from production
    loggers (``mmd.*``, ``MayaMMD``) are considered "warnings" for the
    purposes of the test report.

    Usage as context manager::

        with WarningCollector() as collector:
            run_test()
        for w in collector.get_warnings():
            print(w)
    """

    def __init__(self) -> None:
        super().__init__(level=logging.WARNING)
        self._records: list[logging.LogRecord] = []
        self._installed: bool = False

    # -- context manager ----------------------------------------------------

    def __enter__(self) -> Self:
        self.install()
        return self

    def __exit__(self, *args: object) -> None:
        self.uninstall()

    # -- Handler interface --------------------------------------------------

    def emit(self, record: logging.LogRecord) -> None:
        self._records.append(record)

    # -- lifecycle ----------------------------------------------------------

    def install(self) -> None:
        """Attach this handler to the root logger."""
        if not self._installed:
            logging.getLogger().addHandler(self)
            self._installed = True

    def uninstall(self) -> list[logging.LogRecord]:
        """Detach and return all collected log records."""
        if self._installed:
            logging.getLogger().removeHandler(self)
            self._installed = False
        return self._records

    def clear(self) -> None:
        """Reset collected records without uninstalling."""
        self._records.clear()

    # -- query --------------------------------------------------------------

    def get_warnings(self) -> list[str]:
        """Return formatted WARNING messages from production loggers."""
        return [
            f"{r.name}: {r.getMessage()}"
            for r in self._records
            if r.levelno >= logging.WARNING
            and r.levelno < logging.ERROR
            and _is_production_logger(r.name)
        ]

    def get_errors(self) -> list[str]:
        """Return formatted ERROR+ messages from production loggers."""
        return [
            f"{r.name}: {r.getMessage()}"
            for r in self._records
            if r.levelno >= logging.ERROR and _is_production_logger(r.name)
        ]

    @property
    def has_production_warnings(self) -> bool:
        """``True`` if any production WARNING+ were captured."""
        return any(
            r.levelno >= logging.WARNING and _is_production_logger(r.name)
            for r in self._records
        )


@contextmanager
def capture_warnings() -> Iterator[WarningCollector]:
    """Context manager that installs a :class:`WarningCollector` for the
    duration of the ``with`` block.

    Example::

        with capture_warnings() as collector:
            result = run_test_suite(...)
        print(collector.get_warnings())
    """
    collector = WarningCollector()
    collector.install()
    try:
        yield collector
    finally:
        collector.uninstall()


# ---------------------------------------------------------------------------
# Suite report — printed after each test suite
# ---------------------------------------------------------------------------


def _deduplicate_warnings(
    warnings: list[str],
) -> list[tuple[str, int, str]]:
    """Collapse repeated warnings into ``(label, count, first_occurrence)``.

    Warnings that differ only in numbers (e.g. "Could not match 103 bone names"
    vs "Could not match 93 bone names") are grouped under a common label.
    """
    if not warnings:
        return []

    # Group identical messages and count occurrences
    counter: dict[str, tuple[int, str]] = {}
    for w in warnings:
        if w in counter:
            count, first = counter[w]
            counter[w] = (count + 1, first)
        else:
            counter[w] = (1, w)

    # Sort by count descending
    return [
        (msg, count, first)
        for msg, (count, first) in sorted(counter.items(), key=lambda x: -x[1][0])
    ]


def print_suite_report(
    results: list[TestResult],
    suite_name: str = "",
    strict: bool = False,
) -> None:
    """Print a detailed summary of test results including warnings.

    Args:
        results:    Collected :class:`TestResult` objects from a suite run.
        suite_name: Optional suite label for the report header.
        strict:     If ``True``, production warnings are treated as failures.
    """
    if not results:
        return

    passed = [r for r in results if r.passed and not r.skipped]
    failed = [r for r in results if not r.passed]
    skipped = [r for r in results if r.skipped]

    # Collect all warnings across results
    all_warnings: list[str] = []
    for r in results:
        all_warnings.extend(r.warnings)
        all_warnings.extend(r.errors)

    # Print report header
    header = " Suite Report" if not suite_name else f" Suite Report: {suite_name}"
    print(color_text(f"\n{'─' * 60}", "cyan"))
    print(color_text(header, "bold"))
    print(color_text(f"{'─' * 60}", "cyan"))

    # Per-model summary
    models = sorted({r.model_name for r in results})
    print(f"\n  Models tested: {len(models)}")
    print(
        f"  {color_text(f'{len(passed)} passed', 'green')}, "
        f"{color_text(f'{len(failed)} failed', 'red')}, "
        f"{color_text(f'{len(skipped)} skipped', 'yellow')}"
    )

    # ── Failed tests ────────────────────────────────────────────────────
    if failed:
        print(color_text(f"\n  ── Failed Tests ({len(failed)}) ──", "red"))
        for r in failed:
            label = "⚠ WARN-FAIL" if r.strict_fail else "✗ FAIL"
            print(color_text(f"\n  {label}  [{r.model_name}] {r.test_name}", "red"))
            if r.output:
                for line in r.output.splitlines():
                    print(f"         {line}")

    # ── Skipped tests ────────────────────────────────────────────────────
    if skipped:
        print(color_text(f"\n  ── Skipped Tests ({len(skipped)}) ──", "yellow"))
        for r in skipped:
            reason = r.output.strip() if r.output else "(no reason)"
            print(f"  ⊘  [{r.model_name}] {r.test_name}  —  {reason}")

    # ── Warnings ─────────────────────────────────────────────────────────
    if all_warnings:
        deduped = _deduplicate_warnings(all_warnings)
        severity_label = "WARN-FAIL" if strict else "WARNING"
        warn_color = "red" if strict else "yellow"
        print(
            color_text(
                f"\n  ── Production {severity_label}s ({len(all_warnings)} total, "
                f"{len(deduped)} unique) ──",
                warn_color,
            )
        )
        for _msg, count, first in deduped:
            prefix = f"  ⚠ ×{count}" if count > 1 else "  ⚠"
            print(color_text(f"{prefix}  {first}", warn_color))
    else:
        print(color_text("\n  ✓ No production warnings", "green"))

    print(color_text(f"{'─' * 60}\n", "cyan"))


# ---------------------------------------------------------------------------
# Generic test runners
# ---------------------------------------------------------------------------


@dataclass
class SingleTestResult:
    """Result of executing a single test function.

    Encapsulates timing, captured output, warnings, and pass/skip/fail
    so that the inline test-execution blocks across all runners
    (unified loop, ``run_test_suite``, ``run_standalone_suite``) share
    the same logic.
    """

    passed: bool
    skipped: bool
    elapsed: float
    captured: str
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)


def _run_single_test(
    test_func: Callable[..., bool],
    *args: object,
    use_undo: bool = False,
    collect_warnings: bool = True,
) -> SingleTestResult:
    """Execute *test_func(**args)* with timing, stdout capture, and
    optional undo wrapping.

    Args:
        test_func: The test callable (may be ``functools.partial``).
        *args: Positional arguments forwarded to *test_func*.
        use_undo: If ``True``, wrap the call in ``cmds.undoInfo`` chunks.
        collect_warnings: If ``True``, install a :class:`WarningCollector`
            to capture production-code warnings emitted during the test.

    Returns:
        A :class:`SingleTestResult` with elapsed time, captured stdout,
        pass/skip status, and any warnings collected.
    """
    captured = ""
    skipped = False
    collector = WarningCollector() if collect_warnings else None
    t_start = time.perf_counter()

    try:
        if collector is not None:
            collector.install()
        with redirect_stdout(io.StringIO()) as buf:
            passed = test_func(*args)
        elapsed = time.perf_counter() - t_start
        captured = buf.getvalue().rstrip()
    except TestSkipped as skip_exc:
        elapsed = time.perf_counter() - t_start
        skipped = True
        passed = True
        reason = str(skip_exc).strip()
        captured = ("SKIP: " + reason) if reason else "SKIP"
    except (TestFailed, AssertionError) as exc:
        elapsed = time.perf_counter() - t_start
        passed = False
        captured = f"FAIL: {exc}"
    except Exception as exc:
        elapsed = time.perf_counter() - t_start
        passed = False
        captured = f"EXCEPTION: {exc}"
        traceback.print_exc()
    finally:
        if collector is not None:
            collector.uninstall()
        if use_undo:
            cmds.undoInfo(closeChunk=True)
            try:
                suppressed_undo()
            except RuntimeError:
                pass

    return SingleTestResult(
        passed=passed,
        skipped=skipped,
        elapsed=elapsed,
        captured=captured,
        warnings=collector.get_warnings() if collector else [],
        errors=collector.get_errors() if collector else [],
    )


# ── run_test_suite ──────────────────────────────────────────────────────


def run_test_suite(
    suite_name: str,
    tests: list[TestEntry],
    testing_models: list[str],
    parse_fn: Callable,
    build_fn: Callable,
    model_filter: Callable[[str], bool] | None = None,
    strict: bool = False,
    *,
    preloaded: tuple[object, object, str, bool] | None = None,
) -> bool:
    """Run *tests* against every model in *testing_models*.

    Output is kept compact: one line per test during the run, with full
    error details collected and printed together in the failure report at
    the end so nothing is lost in scroll-back.

    Args:
        suite_name:      Human-readable label shown in the header banner.
        tests:           Ordered ``[(display_name, test_func), ...]`` list.
        testing_models:  Absolute paths to ``.pmx`` files.
        parse_fn:        ``parse_fn(path) -> pmx_data`` – parses the PMX file.
        build_fn:        ``build_fn(pmx_data) -> maya_scene_data`` – builds the
                         Maya scene and returns whatever object the tests expect.
        model_filter:    Optional callable that takes a model path and returns
                         ``True`` if the model should be tested. If ``None``, all
                         models are tested.
        strict:          If ``True``, production warnings fail the test.
        preloaded:       Optional ``(pmx_data, maya_scene_data, model_label,
                         use_undo)`` tuple.  When provided the load/build step
                         is skipped — the caller has already built the scene.
                         Used by the unified model-loop.

    Returns:
        ``True`` if every test passed, ``False`` otherwise.
    """
    # ── Preloaded path (unified model-loop) ──────────────────────────────
    if preloaded is not None:
        _pre_pmx, _pre_maya, _pre_label, _pre_undo = preloaded
        print(color_text(f"\n{'─' * 60}", "cyan"))
        print(color_text(f"  {suite_name}", "bold"))
        print(color_text(f"{'─' * 60}", "cyan"))
        print(f"\n{color_text(_pre_label, 'yellow')}")
        results: list[TestResult] = []
        for test_name, test_func in tests:
            tr = _run_single_test(
                test_func,
                _pre_pmx,
                _pre_maya,
                use_undo=_pre_undo,
            )
            result = TestResult(
                _pre_label,
                test_name,
                tr.passed,
                tr.captured,
                skipped=tr.skipped,
                warnings=tr.warnings,
                errors=tr.errors,
            )
            if strict and (tr.warnings or tr.errors):
                result.passed = False
                result.strict_fail = True
                if result.output:
                    result.output += "\n"
                result.output += "\n".join(tr.warnings + tr.errors)
            results.append(result)
            tick = (
                color_text("⊘", "yellow")
                if result.skipped
                else color_text("⚠", "yellow")
                if result.strict_fail
                else color_text("✓", "green")
                if result.passed
                else color_text("✗", "red")
            )
            et = (
                f"[{tr.elapsed:.1f}s]"
                if tr.elapsed >= 0.1
                else f"[{tr.elapsed * 1000:.0f}ms]"
            )
            hint = ""
            if result.output:
                last_line = result.output.splitlines()[-1].strip()
                if last_line:
                    c = (
                        "yellow"
                        if result.skipped
                        else ("green" if result.passed else "red")
                    )
                    hint = f"  {color_text(last_line, c)}"
            print(f"  {tick} {et} {test_name}{hint}")
        print_suite_report(results, suite_name, strict=strict)
        return all(r.passed for r in results)

    # ── Normal path: load model per path ─────────────────────────────────
    # Apply model filter if provided
    if model_filter is not None:
        filtered_models = [m for m in testing_models if model_filter(m)]
    else:
        filtered_models = testing_models

    print(color_text(f"\n{'─' * 60}", "cyan"))
    print(color_text(f"  {suite_name}", "bold"))
    print(color_text(f"{'─' * 60}", "cyan"))

    results: list[TestResult] = []

    for model_path in filtered_models:
        model_name = (
            os.path.basename(os.path.dirname(model_path))
            + "/"
            + os.path.basename(model_path)
        )
        print(f"\n{color_text(model_name, 'yellow')}")

        # ── Load model ────────────────────────────────────────────────────
        load_buf = io.StringIO()
        try:
            setup_test_environment()
            with redirect_stdout(load_buf):
                pmx_data = parse_fn(model_path)
                maya_scene_data = build_fn(pmx_data)
        except Exception as exc:
            err_text = f"LOAD ERROR: {exc}\n{traceback.format_exc()}"
            print(f"  {color_text('✗ LOAD FAILED', 'red')}  {exc}")
            for test_name, _ in tests:
                results.append(TestResult(model_name, test_name, False, err_text))
            continue

        # ── Execute tests ─────────────────────────────────────────────────
        for test_name, test_func in tests:
            tr = _run_single_test(test_func, pmx_data, maya_scene_data)

            result = TestResult(
                model_name,
                test_name,
                tr.passed,
                tr.captured,
                skipped=tr.skipped,
                warnings=tr.warnings,
                errors=tr.errors,
            )

            # Strict mode: production warnings → test failure
            if strict and (tr.warnings or tr.errors):
                result.passed = False
                result.strict_fail = True
                if result.output:
                    result.output += "\n"
                result.output += "\n".join(tr.warnings + tr.errors)

            results.append(result)

            # Compact one-liner per test with timing
            if result.skipped:
                tick = color_text("⊘", "yellow")
            elif result.passed:
                tick = color_text("✓", "green")
            elif result.strict_fail:
                tick = color_text("⚠", "yellow")
            else:
                tick = color_text("✗", "red")
            elapsed_str = (
                f"[{tr.elapsed:.1f}s]"
                if tr.elapsed >= 0.1
                else f"[{tr.elapsed * 1000:.0f}ms]"
            )
            # Grab last meaningful line from captured output as the inline hint
            hint = ""
            captured = result.output
            if captured:
                last_line = captured.splitlines()[-1].strip()
                if last_line:
                    color = (
                        "yellow"
                        if result.skipped
                        else ("green" if result.passed else "red")
                    )
                    hint = f"  {color_text(last_line, color)}"
            print(f"  {tick} {elapsed_str} {test_name}{hint}")

    # ── Print failure / warning report ─────────────────────────────────
    print_suite_report(results, suite_name, strict=strict)

    return all(r.passed for r in results)


# ---------------------------------------------------------------------------
# Standalone test runner — for node / cmd suites that don't need PMX models
# ---------------------------------------------------------------------------


def run_standalone_suite(
    suite_name: str,
    tests: list[TestEntry],
    strict: bool = False,
) -> bool:
    """Run a list of zero-argument test functions with standardized output.

    Unlike :func:`run_test_suite`, this does NOT load PMX models — each
    test is a plain ``() -> bool`` callable.  Output matches the standard
    ``✓ [Xms] TestName  hint`` format.

    Args:
        suite_name: Human-readable label for the header.
        tests: Ordered ``[(display_name, test_func), ...]`` list.
        strict: If ``True``, production warnings fail the test.

    Returns:
        ``True`` if every test passed.
    """
    print(color_text(f"\n{'─' * 60}", "cyan"))
    print(color_text(f"  {suite_name}", "bold"))
    print(color_text(f"{'─' * 60}", "cyan"))

    results: list[TestResult] = []

    for test_name, test_func in tests:
        tr = _run_single_test(test_func)

        captured = tr.captured
        ok = tr.passed
        strict_fail = False

        # Strict mode: production warnings → test failure
        if strict and (tr.warnings or tr.errors):
            ok = False
            strict_fail = True
            if captured:
                captured += "\n"
            captured += "\n".join(tr.warnings + tr.errors)

        result = TestResult(
            model_name="(standalone)",
            test_name=test_name,
            passed=ok,
            output=captured,
            warnings=tr.warnings,
            errors=tr.errors,
            strict_fail=strict_fail,
        )
        results.append(result)

        tick = (
            color_text("⚠", "yellow")
            if strict_fail
            else color_text("✓", "green")
            if ok
            else color_text("✗", "red")
        )
        et = (
            f"[{tr.elapsed:.1f}s]"
            if tr.elapsed >= 0.1
            else f"[{tr.elapsed * 1000:.0f}ms]"
        )
        hint = ""
        if captured:
            last_line = captured.splitlines()[-1].strip()
            # Strip common test-output prefixes so we don't get double ticks
            for prefix in ("✓ ", "✗ ", "PASS: ", "FAIL: ", "SKIP: "):
                if last_line.startswith(prefix):
                    last_line = last_line[len(prefix) :]
                    break
            if last_line:
                hint = f"  {color_text(last_line, 'green' if ok else 'red')}"
        print(f"  {tick} {et} {test_name}{hint}")

    # ── Print failure / warning report ─────────────────────────────────
    print_suite_report(results, suite_name, strict=strict)

    return all(r.passed for r in results)


# ---------------------------------------------------------------------------
# Standard __main__ entrypoint helper
# ---------------------------------------------------------------------------


def run_main(suite_fn: Callable[[], bool], *, plugin_name: str = "MayaMMD") -> None:
    """Canonical ``__main__`` entry-point used by every integration test script.

    Loads the plugin, calls *suite_fn()*, uninitialises Maya, and exits with
    code ``0`` on full success or ``1`` on any failure.

    Args:
        suite_fn:    Zero-argument callable that runs the test suite and
                     returns ``True`` if all tests passed.
        plugin_name: Maya plugin name (forwarded to :func:`load_plugin`).
    """
    import maya.standalone as _standalone  # local import – already initialised

    try:
        if not load_plugin(plugin_name):
            print("Failed to load MayaMMD plugin. Aborting tests.")
            _standalone.uninitialize()
            sys.exit(1)

        success = suite_fn()
        _standalone.uninitialize()
        sys.exit(0 if success else 1)

    except ImportError as exc:
        print(f"Import error (may be non-critical): {exc}")
        try:
            success = suite_fn()
            _standalone.uninitialize()
            sys.exit(0 if success else 1)
        except Exception as exc2:
            print(f"Fatal error: {exc2}")
            traceback.print_exc()
            _standalone.uninitialize()
            sys.exit(1)

    except Exception as exc:
        print(f"Fatal error: {exc}")
        traceback.print_exc()
        try:
            _standalone.uninitialize()
        except Exception:
            pass
        sys.exit(1)


# ---------------------------------------------------------------------------
# Model filtering utilities
# ---------------------------------------------------------------------------


def create_model_name_filter(*model_names: str) -> Callable[[str], bool]:
    """Create a filter that matches models by filename.

    Args:
        *model_names: One or more model filenames (e.g., ``"model.pmx"``).

    Returns:
        A filter function that returns ``True`` if the model path's
        basename matches any of the provided names.

    Example:
        >>> filter_fn = create_model_name_filter("Acacia.pmx", "Fanny.pmx")
        >>> filter_fn("/path/to/Acacia.pmx")  # True
        >>> filter_fn("/path/to/Other.pmx")   # False
    """
    name_set = set(model_names)

    def filter_fn(model_path: str) -> bool:
        return os.path.basename(model_path) in name_set

    return filter_fn


def create_model_folder_filter(*folder_names: str) -> Callable[[str], bool]:
    """Create a filter that matches models by their parent folder name.

    Args:
        *folder_names: One or more folder names (e.g., ``"Acacia"``, ``"Fanny"``).

    Returns:
        A filter function that returns ``True`` if the model's parent
        directory name matches any of the provided folder names.

    Example:
        >>> filter_fn = create_model_folder_filter("Acacia", "Fanny")
        >>> filter_fn("/models/Acacia/model.pmx")  # True
        >>> filter_fn("/models/Other/model.pmx")   # False
    """
    folder_set = set(folder_names)

    def filter_fn(model_path: str) -> bool:
        parent_dir = os.path.basename(os.path.dirname(model_path))
        return parent_dir in folder_set

    return filter_fn


def create_model_path_pattern_filter(pattern: str) -> Callable[[str], bool]:
    """Create a filter that matches models by a substring in their full path.

    Args:
        pattern: A substring to search for in the full model path.

    Returns:
        A filter function that returns ``True`` if *pattern* appears
        anywhere in the model's full path (case-insensitive on Windows).

    Example:
        >>> filter_fn = create_model_path_pattern_filter("SnowbreakContainmentZone")
        >>> filter_fn("c:/models/SnowbreakContainmentZone/Acacia/model.pmx")  # True
        >>> filter_fn("c:/models/Other/model.pmx")  # False
    """

    def filter_fn(model_path: str) -> bool:
        # Case-insensitive on Windows, case-sensitive on Unix
        if os.name == "nt":
            return pattern.lower() in model_path.lower()
        return pattern in model_path

    return filter_fn


def combine_filters(*filters: Callable[[str], bool]) -> Callable[[str], bool]:
    """Combine multiple filters with AND logic.

    Args:
        *filters: One or more filter functions.

    Returns:
        A filter function that returns ``True`` only if all provided
        filters return ``True`` for a given model path.

    Example:
        >>> folder_filter = create_model_folder_filter("Acacia")
        >>> name_filter = create_model_name_filter("model.pmx")
        >>> combined = combine_filters(folder_filter, name_filter)
        >>> combined("/models/Acacia/model.pmx")  # True
        >>> combined("/models/Acacia/other.pmx")  # False
    """

    def filter_fn(model_path: str) -> bool:
        return all(f(model_path) for f in filters)

    return filter_fn
