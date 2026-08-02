# Releasing MayaMMD

This document describes the process for creating a new release of MayaMMD.

## Release Process

### 1. Prepare the Release

- [ ] Ensure all changes for the release are merged to the `main` branch.
- [ ] Update `CHANGELOG.md` with the new version and release date.
- [ ] Decide on the version number for this release (e.g. `v1.0.0`).
- [ ] Run the full test suite locally:
  ```bash
  pip install ".[dev]"
  ruff format --check mmd tests
  ruff check mmd tests
  mypy mmd tests
  pytest tests/unit_tests tests/integration/core/ --ignore=scripts --ignore=tests/integration/maya
  ```

### 2. Create a Release Tag

Push a version tag to GitHub. The CI will automatically build and publish the release.

```bash
# For a full release:
git tag -a v1.0.0 -m "v1.0.0"
git push origin v1.0.0

# For a release candidate (pre-release):
git tag -a v1.0.0-rc1 -m "v1.0.0-rc1"
git push origin v1.0.0-rc1
```

### 3. CI Automation

Pushing a tag matching `v*` triggers the **Release** workflow (`.github/workflows/release.yml`):

1. **Python Tests** — Format check, linting, type checking, unit tests, version validation.
2. **Build (12-job matrix)** — Each job targets one Maya version × OS combination:
   - Maya 2024, 2025, 2026, 2027
   - Windows, Linux, macOS
   
   Each job:
   - Downloads the Maya SDK from GitHub Releases cache
   - Builds `MayaMMD.mll` (or `.so` / `.bundle`) via CMake
   - Assembles Python source modules (matching Python version)
   - Assembles `MayaMMD-Maya{ver}-{OS}/` via CMake install
3. **Package** — Downloads all 12 variants and assembles a single cross-platform release zip.
4. **Dry-run** (workflow_dispatch only) — Validates the zip without publishing.

### 4. Manual Release Build (Local)

To build a release variant locally for testing:

```bash
# One-time: publish stripped SDKs to GitHub Releases
python scripts/ci/strip_maya_sdk.py publish --version 2026

# Build for a specific Maya version (auto-downloads SDK)
cmake --preset maya2026-release -DBUILD_PACKAGE=ON
cmake --build out/build/maya2026-release --config Release
cmake --install out/build/maya2026-release --config Release

# Output: out/install/maya2026-release/MayaMMD-Maya2026-{OS}/

# Package all available variants into a zip
python scripts/release/package_release.py --build-root out/install
# Output: dist/mayammd-v{version}.zip
```

### 5. Release Artifact Structure

```
mayammd-v{version}.zip
├── install.py                       # Drag onto Maya viewport to install
├── INSTALL.txt                      # Manual install instructions
├── MayaMMD-Maya2024-Windows/
│   ├── MayaMMD.mod
│   └── MayaMMD/
│       ├── plug-ins/
│       │   ├── MayaMMD.mll         # C++ entry point
│       │   └── mmd/                 # Python source modules
│       ├── scripts/
│       ├── icons/
│       ├── README.md
│       ├── LICENSE.txt
│       └── CHANGELOG.md
├── MayaMMD-Maya2024-Linux/
├── MayaMMD-Maya2024-macOS/
├── MayaMMD-Maya2025-Windows/
├── ... (12 variants total)
└── MayaMMD-Maya2027-macOS/
```

### 6. SDK Cache Maintenance

The release builds download Maya SDKs from a pre-configured source.
See `scripts/ci/strip_maya_sdk.py` (in the SDK repository) for details.
If Autodesk publishes new DevKit updates, regenerate the cached SDKs:

```bash
# Republish SDKs for all Maya versions
python scripts/ci/strip_maya_sdk.py publish --version 2024
python scripts/ci/strip_maya_sdk.py publish --version 2025
python scripts/ci/strip_maya_sdk.py publish --version 2026
python scripts/ci/strip_maya_sdk.py publish --version 2027

# SDK updates are handled in a separate (private) repository.
# See the SDK repository for publish and upload instructions.
```

### 7. Verify the Release

- [ ] Visit the [Releases page](https://github.com/sstarosz/MayaMMD/releases) and confirm the new release is listed.
- [ ] Download the zip and test installation on:
  - **Windows**: Drag `install.py` into Maya 2026 → shelf button appears → load plugin → import a PMX
  - **Linux**: Same test on Maya 2025
  - **macOS**: Same test on Maya 2024
- [ ] Verify version display: The plugin should report `v1.0.0` (matching the release tag).

## Version Scheme

This project follows [Semantic Versioning](https://semver.org/):

- **MAJOR** — Breaking changes (incompatible Maya version requirements, architecture changes)
- **MINOR** — New features (new import formats, new UI capabilities)
- **PATCH** — Bug fixes, performance improvements, documentation

**Git tags are the single source of truth.** At CMake configure time,
`cmake/GitVersion.cmake` runs ``git describe --tags --match "v[0-9]*"``
to produce the project version.  This version is injected into:

- ``mmd/_version.py`` — read by Python at import time
- ``build/cmake/generated/version.hpp`` — included by the C++ plugin
- ``MayaMMD.mod`` — Maya module descriptor

The Python ``mmd.plugin`` reads ``__version__`` from ``mmd._version``
(with a ``"0.0.0+dev"`` fallback if the file is missing).

### During Development

``mmd/_version.py`` is generated by CMake configure and should not be
committed (it is in ``.gitignore``).  After ``git tag vX.Y.Z`` and a
CMake reconfigure, the version updates automatically.

## Pre-Release Checklist

Before tagging a release, verify:

- [ ] Git tag is pushed (e.g. `v1.0.0`)
- [ ] `CHANGELOG.md` is up to date
- [ ] `README.md` accurately describes current features
- [ ] `scripts/release/install.py` is the single installer file
- [ ] All CI checks pass on `main`
- [ ] Smoke test on at least one Maya version per supported OS

## Manual Build (Without CI)

To build a release zip locally (single platform):

```bash
# 1. Assemble the mmd/ source package
python scripts/release/build_release.py

# 2. Package the release zip
python scripts/release/package_release.py
```

For a full cross-platform build, you need to run on each OS:
- `python scripts/release/build_release.py` on Windows, Linux, and macOS
  (run once per Python version: 3.10, 3.11, 3.13)

Then combine the outputs with `python scripts/release/package_release.py`.
