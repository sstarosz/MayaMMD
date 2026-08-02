"""
diagnose_pmx_import_hang.py

Step-by-step PMX import diagnostic for the "Maya hangs / gets stuck on import"
problem (no crash, no CPU/memory growth => something BLOCKS instead of computing).

It runs the exact import pipeline that `build_pmx_scene()` runs, one phase at a
time, and:

  * prints a progress line *before* and *after* every step (flushed), so the
    last printed step is the one that blocks or is slow;
  * runs a watchdog thread that, every WATCHDOG_INTERVAL seconds, reports the
    current step AND dumps the main thread's Python stack trace.  Even if the
    main thread is stuck inside a Maya C++ call (GIL released), the watchdog
    can still walk the main thread's Python frames and tell us the exact call.

Usage (standalone, via mayapy):
    mayapy scripts/maya/diagnose_pmx_import_hang.py "<path/to/model.pmx>"

Usage (interactive Maya):
    run this file from the Script Editor, passing the model path:
        exec(open(r"scripts/maya/diagnose_pmx_import_hang.py").read())
        main(r"C:/path/to/model.pmx")
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

_PROJECT_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

WATCHDOG_INTERVAL = 5.0  # seconds between watchdog status reports


# ---------------------------------------------------------------------------
# Watchdog — reports which step the main thread is in, plus its Python stack
# ---------------------------------------------------------------------------
class _StepTracker:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._step = "startup"
        self._since = time.time()

    def set(self, step: str) -> None:
        with self._lock:
            self._step = step
            self._since = time.time()

    def get(self) -> tuple[str, float]:
        with self._lock:
            return self._step, time.time() - self._since


def _format_main_stack(main_thread_id: int) -> str:
    try:
        frame = sys._current_frames().get(main_thread_id)
    except Exception:  # noqa: BLE001 - diagnostic best-effort
        return "    (could not read main thread stack)"
    if frame is None:
        return "    (main thread frame not found)"

    lines = []
    while frame is not None:
        fname = frame.f_code.co_filename
        # Skip watchdog/threading internals.
        if "threading.py" in fname and frame.f_code.co_name in ("run", "_bootstrap"):
            break
        lines.append(
            f"    {os.path.basename(fname)}:{frame.f_lineno}  in  "
            f"{frame.f_code.co_name}"
        )
        frame = frame.f_back
    return "\n".join(lines)


def _start_watchdog(tracker: _StepTracker, stop_event: threading.Event) -> None:
    main_id = threading.main_thread().ident

    def _loop() -> None:
        while not stop_event.is_set():
            step, since = tracker.get()
            if since >= WATCHDOG_INTERVAL:
                print(
                    f"\n[WATCHDOG] still in step '{step}' for {since:.1f}s — "
                    "main thread stack:",
                    flush=True,
                )
                print(_format_main_stack(main_id), flush=True)
                print(flush=True)
            time.sleep(1.0)

    t = threading.Thread(target=_loop, daemon=True, name="import-watchdog")
    t.start()


# ---------------------------------------------------------------------------
# Maya bootstrap (works standalone via mayapy, or already-interactive)
# ---------------------------------------------------------------------------
def _ensure_maya() -> None:
    global cmds
    try:
        import maya.cmds as _cmds

        _cmds.about(version=True)
        cmds = _cmds
        print("[maya] detected interactive session", flush=True)
    except Exception:  # noqa: BLE001 - probe interactive vs standalone
        import maya.standalone

        maya.standalone.initialize()
        import maya.cmds as _cmds

        cmds = _cmds
        print("[maya] initialized standalone", flush=True)


# ---------------------------------------------------------------------------
# Step runner
# ---------------------------------------------------------------------------
def _run_steps(steps: list[tuple[str, object]], tracker: _StepTracker) -> None:
    for name, fn in steps:
        tracker.set(name)
        print(f"\n>>> STEP: {name}", flush=True)
        t0 = time.perf_counter()
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 - diagnostic must keep going
            print(f"    !! STEP FAILED ({type(exc).__name__}): {exc}", flush=True)
        dt = time.perf_counter() - t0
        print(f"    <<< DONE in {dt:.2f}s", flush=True)
        tracker.set("(idle)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(model_path: str, load_plugin: bool = True) -> int:
    tracker = _StepTracker()
    stop_event = threading.Event()
    _start_watchdog(tracker, stop_event)

    _ensure_maya()

    if load_plugin:
        from tests.integration.test_helpers import load_plugin as _load_plugin

        tracker.set("load_plugin")
        print(">>> STEP: load_plugin", flush=True)
        t0 = time.perf_counter()
        ok = _load_plugin()
        print(
            f"    <<< plugin loaded={ok} in {time.perf_counter() - t0:.2f}s", flush=True
        )
        tracker.set("(idle)")

    if not os.path.isfile(model_path):
        print(f"ERROR: model not found: {model_path}", flush=True)
        stop_event.set()
        return 2

    # ── Parse (pure Python) ───────────────────────────────────────────────
    from mmd.core.pmx_importer import parse_pmx
    from mmd.core.pmx_validate import validate_pmx_model
    from mmd.maya.pmx_naming_manager import PMXNamingManager

    steps: list[tuple[str, object]] = []
    pmx_ref: dict[str, object] = {}

    def s(name: str, fn: object) -> None:
        steps.append((name, fn))

    def _do_parse() -> object:
        pmx_ref["data"] = parse_pmx(model_path)
        return pmx_ref["data"]

    s("parse_pmx", _do_parse)
    s("validate_pmx_model", lambda: validate_pmx_model(pmx_ref["data"]))

    # ── Naming manager (pure Python) ──────────────────────────────────────
    def _do_naming() -> object:
        pmx_ref["nr"] = PMXNamingManager(pmx_ref["data"])
        return pmx_ref["nr"]

    s("PMXNamingManager", _do_naming)

    # ── Scene build phases ────────────────────────────────────────────────
    from mmd.maya.pmx.bone_builder import create_bones_from_pmx_bones
    from mmd.maya.pmx.morph_builder import create_blendshapes_from_pmx_data
    from mmd.maya.pmx_scene_builder import (
        apply_skin_weights,
        assign_materials_to_mesh_faces,
        create_materials_from_pmx_materials,
        create_mesh_nodes_from_pmx_data,
        create_root_node_for_pmx_model,
        create_skin_cluster_for_mesh,
    )

    def _do_root() -> object:
        pmx_ref["root"] = create_root_node_for_pmx_model(pmx_ref["data"], pmx_ref["nr"])
        return pmx_ref["root"]

    def _do_mesh() -> object:
        pmx_ref["maya_data"] = create_mesh_nodes_from_pmx_data(
            pmx_ref["data"],
            root_transform_obj=pmx_ref["root"],
            name_registry=pmx_ref["nr"],
        )
        return pmx_ref["maya_data"]

    def _do_materials() -> object:
        pmx_ref["mats"] = create_materials_from_pmx_materials(
            pmx_ref["data"], pmx_ref["nr"]
        )
        return pmx_ref["mats"]

    def _do_assign_faces() -> object:
        return assign_materials_to_mesh_faces(
            pmx_ref["maya_data"]["mesh_dag_path"],
            pmx_ref["data"].materials,
            pmx_ref["mats"],
        )

    def _do_bones() -> object:
        pmx_ref["bones"] = create_bones_from_pmx_bones(
            pmx_ref["data"],
            root_transform_obj=pmx_ref["root"],
            name_registry=pmx_ref["nr"],
        )
        return pmx_ref["bones"]

    def _do_blendshapes() -> object:
        joints = pmx_ref["bones"][0]
        pmx_ref["bs"] = create_blendshapes_from_pmx_data(
            pmx_ref["data"],
            mesh_name=pmx_ref["nr"].get_mesh_name(),
            name_registry=pmx_ref["nr"],
            joints=joints,
        )
        return pmx_ref["bs"]

    def _do_skin_cluster() -> object:
        joints = pmx_ref["bones"][0]
        pmx_ref["sc"] = create_skin_cluster_for_mesh(
            pmx_ref["data"],
            mesh_dag_path=pmx_ref["maya_data"]["mesh_dag_path"],
            joints=joints,
            name_registry=pmx_ref["nr"],
        )
        return pmx_ref["sc"]

    def _do_skin_weights() -> object:
        joints = pmx_ref["bones"][0]
        return apply_skin_weights(
            pmx_ref["data"],
            mesh_dag_path=pmx_ref["maya_data"]["mesh_dag_path"],
            skin_cluster_obj=pmx_ref["sc"],
            joints=joints,
        )

    s("create_root_node", _do_root)
    s("create_mesh_nodes", _do_mesh)
    s("create_materials", _do_materials)
    s("assign_materials_to_faces", _do_assign_faces)
    s("create_bones", _do_bones)
    s("create_blendshapes", _do_blendshapes)
    s("create_skin_cluster", _do_skin_cluster)
    s("apply_skin_weights", _do_skin_weights)

    print("\n========== STARTING IMPORT PIPELINE ==========", flush=True)
    _run_steps(steps, tracker)

    print("\n========== ALL STEPS COMPLETED ==========", flush=True)
    stop_event.set()
    return 0


def _cli() -> int:
    parser = argparse.ArgumentParser(description="Diagnose PMX import hang")
    parser.add_argument("model", help="Path to the .pmx model file")
    parser.add_argument(
        "--no-plugin", action="store_true", help="Skip loading the MayaMMD plugin"
    )
    args = parser.parse_args()
    return main(args.model, load_plugin=not args.no_plugin)


if __name__ == "__main__":
    raise SystemExit(_cli())
