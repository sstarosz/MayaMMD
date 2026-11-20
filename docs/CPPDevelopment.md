# C++ Development Guide

This document covers the C++ development workflow for MayaMMD — building
the native Maya plugin (`.mll` / `.so` / `.bundle`), running tests, formatting,
and adding new C++ commands or nodes.

## Prerequisites

| Tool                   | Version             | Notes                                     |
| ---------------------- | ------------------- | ----------------------------------------- |
| **CMake**              | ≥ 3.21              | Build system                              |
| **Visual Studio 2022** | 17.x                | Windows only; includes MSVC C++ toolchain |
| **GCC / Clang**        | Any C++17           | Linux / macOS                             |
| **Python**             | 3.10, 3.11, or 3.13 | Must match the Maya version you target    |
| **Maya SDK**           | 2024–2027           | Headers + libs for your Maya version      |
| **vcpkg**              | ≥ 2023-03-29        | Dependency manager                        |
| **Python dev tools**   | —                   | Install via `pip install ".[dev]"`        |
| **clang-format**       | 23.1.4 (CI pin)     | C++ code formatting (optional)            |
| **clang-tidy**         | ≥ 15                | C++ static analysis (optional)            |

### Maya SDK Setup

The easiest way to get the Maya SDK is via the auto-download script:

```bash
# Auto-detect Maya version from your Python version
python scripts/ci/strip_maya_sdk.py ensure

# Or specify a version explicitly
python scripts/ci/strip_maya_sdk.py ensure --version 2026
```

This downloads the DevKit from Autodesk, strips it to headers + libs only
(~44 MB), and caches it in `out/.sdk/sdk-maya{version}/`.

For CI builds, the SDK is downloaded automatically by CMake (see `CMakeLists.txt`).

### vcpkg Setup (dependencies)

MayaMMD resolves its C/C++ dependencies via **vcpkg** (`vcpkg.json`).
Currently that is **Bullet 3.25** (float precision) for the native
`pmxRigidBodyNode`; future C/C++ dependencies will be added to `vcpkg.json`
the same way. A fresh clone just needs a bootstrapped vcpkg with
`VCPKG_ROOT` set:

```bash
# Linux/macOS
git clone https://github.com/microsoft/vcpkg "$HOME/vcpkg"
"$HOME/vcpkg/bootstrap-vcpkg.sh"
export VCPKG_ROOT="$HOME/vcpkg"

# Windows
git clone https://github.com/microsoft/vcpkg "%USERPROFILE%\vcpkg"
"%USERPROFILE%\vcpkg\bootstrap-vcpkg.bat"
set VCPKG_ROOT=%USERPROFILE%\vcpkg
```

The vcpkg toolchain is wired through `CMakePresets.json`: the `maya*` presets
set `CMAKE_TOOLCHAIN_FILE` from `$VCPKG_ROOT` (hidden `with-vcpkg` preset),
and CMake installs the project's dependencies on the first configure (into
the build directory) — no manual `vcpkg install` is needed.

- **No vcpkg:** if `VCPKG_ROOT` is unset, the `maya*` presets fail with a
  clear toolchain error. Bootstrap vcpkg, set `VCPKG_ROOT`, and re-run.
- Keep Bullet at **float** precision: do **not** enable vcpkg's
  `double-precision` feature — `btScalar` must stay float to match
  `rigid_body_node.cpp`.

## Quick Start

### Local C++ Development

```bash
# 1. Ensure the Maya SDK + vcpkg are available (one-time — both auto-detected)
cmake --preset maya2026-release

# 2. Build
cmake --build out/build/maya2026-release --config Release

# 3. The .mll is built — use VS Code task: Build: MayaMMD.mll
#    Load it in Maya: loadPlugin "mmd/MayaMMD.mll";
```

### Python Changes (No Build Needed)

When you edit Python files in `mmd/`, no compilation is needed. Just reload the
plugin in Maya:

```python
# In Maya Script Editor:
cmds.unloadPlugin("MayaMMD")
cmds.loadPlugin("path/to/MayaMMD.mll")
```

## Build System

### CMake Presets

| Preset             | Purpose                   | Maya SDK        |
| ------------------ | ------------------------- | --------------- |
| `maya2024-debug`   | Local C++ dev (Maya 2024) | Auto-downloaded |
| `maya2024-release` | Local C++ dev (Maya 2024) | Auto-downloaded |
| `maya2025-debug`   | Local C++ dev (Maya 2025) | Auto-downloaded |
| `maya2025-release` | Local C++ dev (Maya 2025) | Auto-downloaded |
| `maya2026-debug`   | Local C++ dev (Maya 2026) | Auto-downloaded |
| `maya2026-release` | Local C++ dev (Maya 2026) | Auto-downloaded |
| `maya2027-debug`   | Local C++ dev (Maya 2027) | Auto-downloaded |
| `maya2027-release` | Local C++ dev (Maya 2027) | Auto-downloaded |

### Build Variants

**Local dev** (fast, no Python compilation):
```bash
cmake --preset maya2026-release
cmake --build out/build/maya2026-release --config Release
cmake --install out/build/maya2026-release --config Release
```
Builds `MayaMMD.mll` into `out/install/maya2026-release/plug-ins/`. Python files are used as-is.

**Release build** (assembles Python source + .mll):
```bash
cmake --preset maya2026-release -DBUILD_PACKAGE=ON
cmake --build out/build/maya2026-release --config Release
cmake --install out/build/maya2026-release --config Release
```
Produces `out/install/maya2026-release/MayaMMD-Maya2026-{OS}/` with `.mll` + Python source files.

**Unit tests** (Catch2; enable with `-DBUILD_TESTS=ON`):
```bash
cmake --preset maya2026-release -DBUILD_TESTS=ON
cmake --build out/build/maya2026-release --config Release
ctest --preset default --output-on-failure
```

### Development Scripts

| Command                                                 | Description                   |
| ------------------------------------------------------- | ----------------------------- |
| `cmake --preset maya2026-release && cmake --build ...`  | Build .mll for Maya 2026      |
| `python scripts/ci/strip_maya_sdk.py ensure`            | Download + cache Maya SDK     |
| `clang-format -i $(git ls-files '*.cpp' '*.h' '*.hpp')` | Format all C++ code           |
| `clang-tidy mmd/...`                                    | Lint C++ code with clang-tidy |

See `.vscode/tasks.json` for pre-configured VS Code tasks for all of the above.

### Performance: the rigid-body playback benchmark

Playback cost is dominated by the rigid-body solver, and the solver's cost is
dominated by how many plugs it reads through the DG per frame. Before changing
the solver, the joint display or the body/joint nodes, measure:

#### What the work so far actually bought

Measured on `GirlsFrontline TololoDefault` (298 bodies, 408 joints), same
machine, `rigid-body` benchmark, and a scripted timeline walk with forced
redraws in an interactive Maya for the viewport numbers:

| Measure                                   | Before | After |
| ----------------------------------------- | -----: | ----: |
| `sim_frame` (timeline move + solver pull)  | 23–29 ms | **13–16 ms** |
| `solver` (`compute()` alone)               | 17–19 ms | **7–8 ms** |
| in-viewport frame (`eval med`, scripted)   | 41.0 ms  | **17 ms** |
| joint display per drawn frame             | —        | **0.0 ms** (VP2-cached) |
| rig with the viewport suspended           | —        | 13 ms (bones/skinning, not this plugin) |

This machine's run-to-run variance is large (±40% on a cold run), so compare
interleaved runs and medians, never single samples.

The three root causes, in the order they were found and worth re-checking if
this regresses:

1. `readInputs()` re-read every body's and joint's ~30 attributes through
   `findPlug` on every frame (9.3 ms of a 16.3 ms compute).  The solver now
   caches its inputs behind `rigidBodyEditSerial()`.
2. The joint display re-read every joint's attributes and frame per redraw.
   `RigidBodyJoint::resolveView()` now caches the configuration, refreshed by
   the config serial, and the frame by its own serial.
3. **The expensive one:** the solver was evaluated ~5× per frame because reading
   the kinematic anchors (`bodyAnchorWorld`) re-dirtied it mid-frame.  Each of
   those evaluations re-read all 25 anchors and rewrote all 298 outputs.  The
   anchors are now read once per `rigidBodyAnchorSerial()` and shared
   (`World::anchorRaw`), and `compute()` skips the world transition entirely
   when nothing can have changed.  This alone took the in-viewport frame from
   41 ms to 17 ms.

What is left, in priority order: `compute()` still runs ~5× per frame, but only
one of those runs the physics (measured over 20 frames: 1 world rebuild, 19
steps, and 40 evaluations that found nothing to do). The other four are output
re-writes of all 298 bodies, ~1.7 ms each — writing only the outputs Maya
actually asked for is the next real win. The display has nothing left
(`updateDG`, `populateGeometry` and `boundingBox` all measured 0.0 ms per
redraw once VP2's caching is respected).

Approaches already tried and **rejected with evidence** — do not repeat them:

- Bullet deactivation / letting cloth sleep: MMD cloth is constraint-chained to
  always-active kinematic bodies, so Bullet merges them into one island and
  nothing ever sleeps.  Identical poses and identical residual motion with and
  without it.
- `MRenderItem::setMatrix()` to carry the hub pose so the vertex data could stay
  static: the item matrix is composed with the node's DAG transform (hubs
  double-transform and fly off the model) and nothing re-prepares the item
  without a per-frame dirty (hubs freeze).
- `setGeometryDrawDirty(obj, /*topologyChanged=*/false)`: VP2 then stops calling
  `populateGeometry` altogether and the hubs freeze.
- Disabling cached playback: `pmxRigidBodyNode` is a stateful solver, so Maya's
  Cached Playback drops to Safe Mode by design.

To re-measure:

```bash
# One representative model (~300 bodies, ~400 joints), 60 measured frames
python tests/benchmarks/run_benchmarks.py rigid-body

# Other models / longer runs
python tests/benchmarks/run_benchmarks.py rigid-body --model Miku --frames 120
```

The suite reports per-frame milliseconds (median over frames):

| Column      | Meaning                                                          |
| ----------- | ---------------------------------------------------------------- |
| `sim_frame` | one playback frame: timeline move + solver pull                   |
| `solver`    | the solver's `compute()` alone (input reads + Bullet + write-back) |
| `settle`    | the same with the skeleton held past the motion's end (cloth at rest) |
| `fps`       | frames per second implied by `sim_frame`                          |
| `setup`     | model import + motion apply (cache/warm-up sanity)                |

Maya standalone has no viewport, so the geometry overrides and
`boundingBox()` are not covered — this measures the simulation half of the
frame budget. Each run also writes a JSON report to
`test-logs/benchmarks/` for trend comparison.

Two invariants keep the per-frame reads small, and a change that breaks them
shows up as a large `solver` regression:

- `pmxRigidBodyNode` caches its PMX-verbatim inputs and re-reads them only when
  `mmd::maya::rigidBodyEditSerial()` moves (a reported config edit) or a body /
  joint connection changed. `bodyAnchorWorld` is per-frame data, not config.
- The kinematic anchors (`bodyAnchorWorld`) are read once per
  `mmd::maya::rigidBodyAnchorSerial()` and shared by every evaluation of that
  frame (`World::anchorRaw`). Reading one is a cross-node plug evaluation, the
  test rig has 25 of them, and the solver is evaluated several times per frame
  — once per output batch Maya pulls. Use `noteRigidBodyAnchorEdit()` from
  anything that can move an anchor.
- `compute()` returns to the output write without touching the world when
  nothing can have changed: no input edit (`inputsStale`), the same time, and an
  unchanged anchor serial. Skipping this fast path is what made playback pay for
  the world transition several times per frame.
- `pmxRigidBodyJoint::resolveSolverView()` caches the joint's static display
  data (limits, body links, status, hub radius) and refreshes the live
  attachment frames only; its frame is re-read when
  `mmd::maya::rigidBodyFrameSerial()` moves (a moved joint or model root), so
  no DG read is needed per joint per frame just to notice a move.

Inside the solver, the remaining per-frame cost is the Bullet step itself plus
one `bodyAnchorWorld` read per kinematic body — both real simulation work.
Adding per-frame attribute reads (through `MPlug`) anywhere in `compute()` is
what to avoid: a single cross-node plug read costs more than the whole
input-cache bookkeeping.

Both geometry overrides take part in VP2 caching: `requiresGeometryUpdate()`
returns false for both (the node marks its drawing dirty on every input edit)
and `requiresUpdateRenderItems()` returns false as well for both. Do not turn
either back on to "make sure" a change is picked up: mark the drawing dirty
where the change happens instead.

One thing that looks like a shortcut but is not: handing VP2 the pose through
`MRenderItem::setMatrix()` so that the vertex data can stay static. For an item
in a geometry override that matrix is **composed** with the node's DAG
transform instead of replacing it (the hubs double-transform and fly off the
model), and returning `true` from `requiresUpdateRenderItems()` does not by
itself make VP2 re-prepare the object, so the item matrix is never refreshed and
the hubs freeze. The pose therefore stays baked into the vertices, and the
per-evaluation `setGeometryDrawDirty()` in `RigidBodyJoint` is load-bearing.

### The viewport half

A joint's hub is re-baked every frame: its pose is part of the vertex data
(see the note above on why the render-item matrix cannot carry it), so
`RigidBodyJoint` marks the drawing dirty on every evaluation. What is left in
that path is kept free of DG reads and of allocations:

- `updateDG()` reads the joint once through `RigidBodyJoint::resolveSolverView()`
  and re-classifies the hub only when its config serial moved (see
  `docs/JointVisualization.md`).
- `populateGeometry()` bakes the shared unit hub into reused scratch streams,
  and `fillMeshBuffers()` `memset`s a zeroed UV stream instead of building a
  temporary `std::vector`.
- `updateRenderItems()` finds its four render items through the remembered
  indices and only falls back to a name search when VP2 reordered the list.
- `addUIDrawables()` caches Maya's display status (a viewport query) until the
  selection or isolate state changes, instead of asking once per joint per
  redraw (`rigid_body_display::displayStatusSerial`).

The dirty calls keep `setGeometryDrawDirty()`'s default `topologyChanged`
argument: passing `false` ("only the vertex data changed") reads like exactly
what a moving hub needs, but VP2 then stops calling `populateGeometry` and the
hubs freeze at their last pose. A collider is no different.

`pmxRigidBodyShape`'s geometry is pose-independent, so its override is not
repopulated during playback at all; only its `addUIDrawables()` wire runs.

Maya standalone has no viewport, so none of the display cost above is measured
by the benchmark. To measure it, run an interactive Maya with a model and a
motion loaded, and walk the timeline with a forced redraw:

```python
from time import perf_counter
from maya import cmds

start, frames = 1, 60
cmds.currentTime(start, update=True)
cmds.refresh(force=True)  # warm-up: shader compile, first buffer upload
samples = []
for i in range(frames):
    began = perf_counter()
    cmds.currentTime(start + 1 + i, update=True)
    cmds.refresh(force=True)
    samples.append(perf_counter() - began)
samples.sort()
print(f"median {samples[len(samples) // 2] * 1000:.2f} ms/frame")
```

Repeat the walk with the display switched off (`pmxRigidBodyJoint.drawMode =
0`, `pmxRigidBodyShape.drawMode = 0`) and with the solver disabled
(`pmxRigidBodyNode.nodeState = 1`) to attribute the cost. Two traps when
comparing numbers:

- Hiding the joints also stops the physics: the joint override is what pulls
  `pmxRigidBodyNode.frameMatrix`. Subtract the benchmark's `solver` column to
  split draw from simulation.
- `cmds.playbackOptions` is real-time synced by default, so any "achieved fps"
  measured through `cmds.play()` saturates at the scene frame rate (24) however
  fast the frame really is. Measure a scripted walk instead (as above), or set
  `playbackOptions(playbackSpeed=1)` first.

## Project Structure

```
mmd/                          # Python package + C++ plugin entry point
├── MayaMMD.cpp              # C++ plugin entry point (initializePlugin)
├── MayaMMD.mll              # Built C++ plugin (copied by post-build)
├── plugin.py                 # Python-side initialization
├── core/                     # File format readers
├── maya/                     # Maya-specific logic
│   ├── nodes/                # C++ and Python MPxNode implementations
│   │   ├── bone_morph_node.py         # Python bone morph node
│   │   ├── ccd_ik_solver_node.h       # C++ CCD IK solver header
│   │   ├── ccd_ik_solver_node.cpp     # C++ CCD IK solver implementation
│   │   ├── rigid_body_node.hpp        # C++ rigid-body physics node header
│   │   └── rigid_body_node.cpp        # C++ rigid-body physics node (embedded Bullet)
│   ├── pmx/                  # PMX scene builders
│   └── ...
└── ui/                       # Qt UI widgets

tests/
├── CMakeLists.txt            # CTest definitions (Catch2 + Maya integration)
├── unit_tests/               # Unit tests
│   ├── core/                 # Core tests (pytest .py + Catch2 .cpp)
│   ├── maya/                 # Python Maya tests (pytest)
│   └── ui/                   # Python UI tests (pytest)
├── integration/              # Maya integration tests
└── benchmarks/               # Performance benchmarks
```

## Architecture: C++ / Python Bridge

`MayaMMD.mll` is the **single entry point** for the plugin. When Maya loads it:

1. **C++ `initializePlugin`** runs:
   - Registers native C++ nodes (`ccdIKSolverNode`) via `MFnPlugin::registerNode`
   - Calls Python via `MGlobal::executePythonCommand`

2. **Python `initializePlugin()`** is called:
   - Uses `om.MFnPlugin.findPlugin("MayaMMD")` (API 2.0) to get the plugin handle
   - Registers Python-based nodes/commands on the same handle
   - Sets up the shelf button and UI

Everything registers under the **same "MayaMMD" identity** — one entry in Maya's
Plugin Manager.

## Adding a New C++ Command

1. Create header and implementation in `mmd/maya/nodes/`:
```cpp
// my_command.h
#pragma once
#include <maya/MPxCommand.h>

class MyCommand : public MPxCommand {
public:
    static constexpr const char *kCommandName = "myCommand";
    static void *creator();
    static MSyntax createSyntax();
    MStatus doIt(const MArgList &args) override;
};
```

2. Implement `creator()`, `createSyntax()`, and `doIt()` in `my_command.cpp`.

3. Register in `MayaMMD.cpp` `initializePlugin`:
```cpp
stat = plugin.registerCommand(
    MyCommand::kCommandName,
    MyCommand::creator,
    MyCommand::createSyntax);
```

4. Add the source file to `CMakeLists.txt` in the `MayaMMD` target.

5. Build and test.

## Adding a New C++ Node

1. Subclass `MPxNode` (or `MPxIkSolverNode` for IK solvers).
2. Define a unique `MTypeId` (use `0x00080xxx` range to avoid conflicts).
3. Implement `creator()`, `initialize()`, and node behaviors.
4. Register in `MayaMMD.cpp` via `plugin.registerNode(...)`.
5. Add to `CMakeLists.txt`.

## Debugging the .mll

### Attaching a Debugger (Windows / Visual Studio)

1. Build the Debug configuration: `cmake --preset default && cmake --build build/cmake --config Debug`
2. In Visual Studio, open the solution: `build/cmake/mayammd-native.sln`
3. Set Maya as the startup project: Right-click MayaMMD → Properties → Debugging → Command = `C:\Program Files\Autodesk\Maya2026\bin\maya.exe`
4. Set breakpoints in your C++ code
5. Press F5 to launch Maya with the debugger attached

### Logging

Use `MGlobal::displayInfo()` / `MGlobal::displayWarning()` / `MGlobal::displayError()` for diagnostic output. These appear in Maya's Script Editor.

## Testing

### C++ Unit Tests (Catch2)

```bash
# Configure + build + run (enable tests with -DBUILD_TESTS=ON)
cmake --preset maya2026-release -DBUILD_TESTS=ON
cmake --build out/build/maya2026-release --config Release
ctest --preset default --output-on-failure
```

Add new test files in `tests/unit_tests/core/` and add them to the
`mmd_core_tests` target in `tests/unit_tests/core/CMakeLists.txt`.

### Sanitizers (ASan + UBSan)

The core tests are Maya-free, so they also run under AddressSanitizer and
UndefinedBehaviorSanitizer — that suite covers the Bullet math, manual indexing
and pose round-trips where undefined behaviour hides. CI runs this as the
`C++ Sanitizers (ASan+UBSan)` job; locally (Linux/macOS):

```bash
cmake --preset maya2026-release -DBUILD_TESTS=ON \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -g" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build out/build/maya2026-release --target mmd_core_tests
ctest --preset default -E "maya-integration" --output-on-failure
```

Memory errors abort the test binary (so ctest fails); UBSan diagnostics are
reported with a stack trace and counted into a warning until the existing ones
are fixed. Use a separate build directory if you also need a normal build — the
sanitizer flags stick to the CMake cache. Bullet itself comes from vcpkg
prebuilt, so only project code is instrumented.

### Maya Integration Tests

C++ plugin tests run in Maya standalone (requires `mayapy`):
```bash
ctest --preset default -L maya
```

## Formatting & Linting

```bash
# Format C++ code (whole repository)
clang-format -i $(git ls-files '*.cpp' '*.h' '*.hpp')

# Check formatting (this is exactly what CI runs)
clang-format --dry-run --Werror $(git ls-files '*.cpp' '*.h' '*.hpp')

# Static analysis
clang-tidy mmd/MayaMMD.cpp mmd/maya/nodes/*.cpp -- -std=c++17
```

`clang-tidy` uses the profile in `.clang-tidy` (which includes `modernize-*`) and
runs in CI as the `C++ Tidy` job over every tracked `.cpp`. The tree is clean, so
that whole-tree run is a **hard gate** — any finding fails the job. The same job
also pipes the PR diff into `clang-tidy-diff.py`, which reports the findings this
PR introduced on the lines it touched (handy when reviewing a change).

```bash
# Same checks locally (needs the pip package: pip install clang-tidy==23.1.4)
clang-tidy -p out/build/maya2026-release -warnings-as-errors='*,-clang-diagnostic-*' \
  $(git ls-files '*.cpp')

# Or just the changed lines
git diff -U0 origin/main...HEAD -- '*.cpp' |
  clang-tidy-diff.py -p1 -path out/build/maya2026-release -quiet \
    -warnings-as-errors='*,-clang-diagnostic-*'
```

`-warnings-as-errors` is what makes clang-tidy's exit code non-zero (plain
clang-tidy exits 0 even with findings); `clang-diagnostic-*` is excluded because
compiler diagnostics belong to the build job.

Every configure preset sets `CMAKE_EXPORT_COMPILE_COMMANDS`, so the
Makefile/Ninja generators (Linux/macOS, and therefore CI) write
`out/build/<preset>/compile_commands.json` right after configure — clangd and the
IDE pick it up too. The Visual Studio generator does not emit that file, so on
Windows run clang-tidy from a Ninja/Makefile configure. Running it locally:

```bash
cmake --preset maya2026-release -DBUILD_TESTS=ON
clang-tidy -p out/build/maya2026-release $(git ls-files '*.cpp')
```

Python linting is configured in `pyproject.toml` (`[tool.ruff]`, `[tool.mypy]`);
CI runs `ruff format --check` as a gate and `ruff check` / `mypy` as warnings
until their existing findings are triaged.
The project uses LLVM-style formatting with 4-space indentation (see `.clang-format`).

**Braces are mandatory on control statements.** `InsertBraces: true` adds the
missing `{ }` around `if` / `else` / `for` / `while` / `do` bodies, and
`AllowShortIfStatementsOnASingleLine: Never`, `AllowShortLoopsOnASingleLine: false`
and `AllowShortBlocksOnASingleLine: Never` stop a one-line body from being
collapsed back onto the statement line. `clang-tidy` enforces the same rule via
`readability-braces-around-statements`.

## CI checks (pull requests)

| Job | Command | Blocking |
| --- | --- | --- |
| `Python` | `ruff format --check`, `ruff check`, `mypy`, pytest | format + tests |
| `C++ Format` | `clang-format --dry-run --Werror` (all tracked C++ files) | yes |
| `C++ Tidy` | `clang-tidy -p out/build/maya2026-release` (all tracked `.cpp`) | yes |
| `C++ Tidy` (changed lines) | `clang-tidy-diff.py` over the PR diff | yes |
| `C++ Build` | builds the `.mll`, runs the Catch2 suite | yes |
| `C++ Sanitizers (ASan+UBSan)` | core tests under `-fsanitize=address,undefined` | ASan yes, UBSan warns |

clang-format and clang-tidy are pinned to 23.1.4 in CI. Superseded pushes on a
PR cancel their in-flight run, and workflows run with `contents: read`.

Formatting is checked in CI (`.github/workflows/pr-checks.yml`, job `C++ Format`)
with clang-format pinned to **23.1.4**; use the same version locally
(`pip install clang-format==23.1.4`) since line wrapping differs between
clang-format releases.

## Troubleshooting

### "Maya SDK not found"

Run `python scripts/ci/strip_maya_sdk.py ensure` to download the SDK. Or set `MAYA_LOCATION` env var to your Maya install directory.

### "vcpkg is not configured" / toolchain not found

The native plugin resolves its dependencies via vcpkg. Bootstrap vcpkg and set
`VCPKG_ROOT` (see [vcpkg Setup](#vcpkg-setup-dependencies)), then re-run
`cmake --preset maya2026-release`.

### "Cannot open include file: 'maya/MFnPlugin.h'"

The Maya SDK include path is not set. Verify `out/.sdk/sdk-maya{ver}/include/maya/` exists. Re-run `cmake --preset default` after ensuring the SDK.

### "The specified module could not be found" (loading .mll)

This usually means a dependent DLL is missing. The .mll links against `Foundation`, `OpenMaya`, and `OpenMayaAnim` — these are in Maya's `bin/` directory and should be on `PATH` when Maya is running. If loading via `mayapy`, ensure Maya's `bin/` is on `PATH`.

### "Entry point not found" (loading .mll)

The .mll was built with mismatched Maya SDK headers. Ensure the SDK version matches the Maya version you're loading into. Rebuild with the correct preset.
