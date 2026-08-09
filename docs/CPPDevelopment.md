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
| **clang-format**       | ≥ 15                | C++ code formatting (optional)            |
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
`pmxPhysicsNode`; future C/C++ dependencies will be added to `vcpkg.json`
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
  `physics_node.cpp`.

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

| Command                                                | Description                       |
| ------------------------------------------------------ | --------------------------------- |
| `cmake --preset maya2026-release && cmake --build ...` | Build .mll for Maya 2026          |
| `python scripts/ci/strip_maya_sdk.py ensure`           | Download + cache Maya SDK         |
| `clang-format -i mmd/...`                              | Format C++ code with clang-format |
| `clang-tidy mmd/...`                                   | Lint C++ code with clang-tidy     |

See `.vscode/tasks.json` for pre-configured VS Code tasks for all of the above.

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
│   │   └── ccd_ik_solver_node.cpp     # C++ CCD IK solver implementation
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

### Maya Integration Tests

C++ plugin tests run in Maya standalone (requires `mayapy`):
```bash
ctest --preset default -L maya
```

## Formatting & Linting

```bash
# Format C++ code
clang-format -i mmd/MayaMMD.cpp mmd/maya/nodes/*.cpp mmd/maya/nodes/*.h

# Check formatting (CI)
clang-format --dry-run --Werror mmd/MayaMMD.cpp mmd/maya/nodes/*.cpp mmd/maya/nodes/*.h

# Static analysis
clang-tidy mmd/MayaMMD.cpp mmd/maya/nodes/*.cpp -- -std=c++17
```

The project uses LLVM-style formatting with 4-space indentation (see `.clang-format`).

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
