"""
rigid_body_simulation_benchmark.py

Benchmark for rigid-body playback (requires Maya standalone).

Measures the per-frame cost of the ``pmxRigidBodyNode`` solver over a real
animated model — the cost that decides whether playback runs at frame rate.

Two measurements per frame:

  * ``sim_frame`` — ``currentTime`` (the timeline move, whose dirt path also
    re-evaluates the solver) plus pulling ``outStep``: what one frame of Maya
    playback costs the physics model.
  * ``solver``    — pulling ``outStep`` alone: the solver's ``compute()``
    (PMX input reads + Bullet step + output write-back).

Maya standalone has no viewport, so the viewport-side cost (geometry
overrides, ``MPxLocatorNode::boundingBox``) is not part of these numbers —
this suite is the simulation half of the frame budget.

The JSON report keeps the shared benchmark schema: ``build_time_s`` is the
scene setup, ``total_time_s`` the per-frame ``sim_frame``, and ``details``
carry both per-frame numbers (seconds).

Maya standalone must be initialised before importing this module.
"""

from __future__ import annotations

import os
import statistics
import sys
import time

# ── Ensure workspace root is on sys.path ─────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from maya import cmds

from assets.assets_utils import get_all_pmx_model_paths, get_all_vmd_paths
from mmd.core.pmx_importer import parse_pmx
from mmd.core.vmd_importer import parse_vmd_file
from mmd.maya.pmx_scene_builder import build_pmx_scene
from mmd.maya.vmd_scene_builder import apply_vmd_to_scene
from tests.benchmarks.core.benchmark_helpers import (
    BenchmarkReport,
    BenchmarkResult,
    save_benchmark_report,
)

# ── Globals ───────────────────────────────────────────────────────────────

_ALL_MODELS = get_all_pmx_model_paths()
_ALL_MOTIONS = get_all_vmd_paths()

# TololoDefault is the representative model: a typical game-model rig with
# ~300 bodies and ~400 joints (the assets range from 234 to 477 bodies).  It
# is the default so a plain run stays fast; --model selects others.
_DEFAULT_MODEL = next(
    (m for m in _ALL_MODELS if "GirlsFrontline TololoDefault.pmx" in m),
    _ALL_MODELS[0] if _ALL_MODELS else None,
)
_TEST_MOTION = next(
    (m for m in _ALL_MOTIONS if "1.vmd" in m),
    _ALL_MOTIONS[0] if _ALL_MOTIONS else None,
)

_BENCHMARK_SUITE = "Rigid Body Playback Benchmark"

# Frames measured per model, plus warm-up frames that are discarded (the first
# steps wake every island and fill the caches).
_DEFAULT_FRAMES = 60
_WARMUP_FRAMES = 5
# Settling scenario: play this many frames up to the motion's end, then measure
# the held-pose frames that follow (hair and cloth come to rest there).
_SETTLE_PREROLL = 200
_SETTLE_FRAMES = 60


# ── Scene setup ───────────────────────────────────────────────────────────


def _setup_scene() -> None:
    """Reset Maya to a clean empty scene."""
    cmds.file(new=True, force=True)


def _build_animated_scene(model_path: str) -> str:
    """Import ``model_path``, apply the test motion and return the solver name."""
    pmx = parse_pmx(model_path)
    maya_data = build_pmx_scene(pmx)
    vmd = parse_vmd_file(_TEST_MOTION)
    apply_vmd_to_scene(
        vmd_data=vmd,
        model=maya_data.to_resolved(),
        start_frame=1,
        apply_bone_anim=True,
        apply_morph_anim=False,  # morphs are not part of the physics frame
    )
    solvers = cmds.ls(type="pmxRigidBodyNode")
    if not solvers:
        raise RuntimeError("No pmxRigidBodyNode was created by the import")
    return solvers[0]


def _count(node_type: str) -> int:
    return len(cmds.ls(type=node_type))


# ── Measurement ───────────────────────────────────────────────────────────


def _play(
    solver: str, start_frame: int, frames: int
) -> tuple[list[float], list[float]]:
    """Step ``frames`` frames and return (frame, solver-only) per-frame seconds."""
    per_frame: list[float] = []
    solver_only: list[float] = []
    cmds.currentTime(start_frame)
    for step in range(frames):
        frame = start_frame + step

        frame_start = time.perf_counter()
        cmds.currentTime(frame + 1)
        solve_start = time.perf_counter()
        # The pull the viewport performs every frame; it runs compute() on the
        # solver if the timeline move left it dirty.
        cmds.getAttr(f"{solver}.outStep")
        solve_end = time.perf_counter()

        per_frame.append(solve_end - frame_start)
        solver_only.append(solve_end - solve_start)
    return per_frame, solver_only


def _median_ms(values: list[float]) -> float:
    return statistics.median(values) * 1000.0


def _settle_step(solver: str, end_frame: int, frames: int = _SETTLE_FRAMES) -> float:
    """Per-frame ms with the pose held past the motion's end.

    The last seconds of the motion are played, then time keeps advancing while
    the skeleton holds its final pose: hair and cloth settle.  A regression in
    how fast a settled body stops costing the solver (e.g. every body staying
    permanently awake) shows up here, and it is where a "never settles"
    behaviour would appear.
    """
    start = max(1, end_frame - _SETTLE_PREROLL)
    cmds.currentTime(start)
    cmds.getAttr(f"{solver}.outStep")
    for frame in range(start + 1, end_frame + 1):
        cmds.currentTime(frame)
        cmds.getAttr(f"{solver}.outStep")

    samples: list[float] = []
    for step in range(frames):
        t0 = time.perf_counter()
        cmds.currentTime(end_frame + 1 + step)
        cmds.getAttr(f"{solver}.outStep")
        samples.append(time.perf_counter() - t0)
    return _median_ms(samples)


def _fps(values: list[float]) -> float:
    total = sum(values)
    return (len(values) / total) if total > 0.0 else 0.0


def _print_table(report: BenchmarkReport) -> None:
    """Print the per-frame physics table (ms per frame, median over frames)."""
    results = report.results
    label_w = min(
        max(max((len(r.label) for r in results), default=10), len("Model")), 64
    )
    header = (
        f"{'Model':<{label_w}}  {'sim_frame':>9}  {'solver':>8}  {'settle':>8}  "
        f"{'fps':>7}  {'setup':>8}"
    )
    print(f"═══ {_BENCHMARK_SUITE} ═══")
    print(f"  Timestamp: {report.timestamp}")
    print()
    print("─" * len(header))
    print(header)
    print("─" * len(header))
    for r in results:
        sim_frame = r.details["sim_frame"] * 1000.0
        solver = r.details["solver"] * 1000.0
        settle = r.details.get("settle", 0.0) * 1000.0
        fps = 1000.0 / sim_frame if sim_frame > 0.0 else 0.0
        print(
            f"{r.label[:label_w]:<{label_w}}  {sim_frame:>7.2f}ms  {solver:>6.2f}ms  "
            f"{settle:>6.2f}ms  {fps:>7.1f}  {r.build_time:>7.2f}s"
        )
    print("─" * len(header))
    print(
        "  sim_frame = timeline move + solver pull (one playback frame)\n"
        "  solver    = solver compute() only\n"
        "  settle    = same, with the pose held past the motion's end (cloth at rest)\n"
        "  setup     = model import + motion application"
    )
    print()


# ── Benchmark runner ──────────────────────────────────────────────────────


def run_rigid_body_benchmarks(
    model_filter: str | None = None,
    frames: int = _DEFAULT_FRAMES,
) -> BenchmarkReport:
    """Run the rigid-body playback benchmark on the selected models.

    Args:
        model_filter: Optional substring to filter model paths (case-insensitive).
        frames:       Measured frames per model.

    Returns:
        Aggregated ``BenchmarkReport`` (details carry per-frame seconds).
    """
    models = list(_ALL_MODELS)
    if model_filter:
        models = [m for m in models if model_filter.lower() in m.lower()]
    else:
        models = [m for m in models if _DEFAULT_MODEL and m == _DEFAULT_MODEL]

    if not models:
        print("ERROR: No models matched the filter.")
        return BenchmarkReport(
            suite_name=_BENCHMARK_SUITE,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
            results=[],
        )
    if not _TEST_MOTION:
        print("ERROR: No VMD motion file found.")
        return BenchmarkReport(
            suite_name=_BENCHMARK_SUITE,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
            results=[],
        )

    results: list[BenchmarkResult] = []

    for model_path in models:
        label = os.path.basename(model_path)
        _setup_scene()

        build_start = time.perf_counter()
        solver = _build_animated_scene(model_path)
        build_time = time.perf_counter() - build_start

        bodies = _count("pmxRigidBodyShape")
        joints = _count("pmxRigidBodyJoint")

        # Warm-up: the first steps wake sleeping islands and fill the caches.
        _play(solver, 1, _WARMUP_FRAMES)
        per_frame, solver_only = _play(solver, 1 + _WARMUP_FRAMES, frames)

        sim_frame_ms = _median_ms(per_frame)
        solver_ms = _median_ms(solver_only)
        end_frame = int(cmds.playbackOptions(query=True, maxTime=True))
        settle_ms = _settle_step(solver, end_frame)

        results.append(
            BenchmarkResult(
                label=(
                    f"{label}  ({bodies} bodies, {joints} joints, "
                    f"{_fps(per_frame):.1f} fps)"
                ),
                parse_time=0.0,  # no parse phase in this suite
                build_time=build_time,
                total_time=sim_frame_ms / 1000.0,
                details={
                    "sim_frame": sim_frame_ms / 1000.0,
                    "solver": solver_ms / 1000.0,
                    "settle": settle_ms / 1000.0,
                },
                note=(
                    f"sim_frame={sim_frame_ms:.2f}ms, solver={solver_ms:.2f}ms, "
                    f"settle={settle_ms:.2f}ms, frames={frames}"
                ),
            )
        )

    report = BenchmarkReport(
        suite_name=_BENCHMARK_SUITE,
        timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
        results=results,
    )

    _print_table(report)
    output_dir = os.path.join(_PROJECT_ROOT, "test-logs", "benchmarks")
    path = save_benchmark_report(report, output_dir)
    print(f"  JSON report saved to: {path}\n")

    return report


# ── Standalone entry point ────────────────────────────────────────────────


def main() -> int:
    """Run the rigid-body benchmark standalone (initialises Maya)."""
    import maya.standalone

    maya.standalone.initialize()

    from tests.integration.test_helpers import load_plugin

    if not load_plugin():
        print("ERROR: Failed to load MayaMMD plugin.")
        return 1

    from maya import mel

    print(f"Maya version: {mel.eval('getApplicationVersionAsFloat()')}")

    report = run_rigid_body_benchmarks()
    maya.standalone.uninitialize()
    return 0 if report.results else 1


if __name__ == "__main__":
    sys.exit(main())
