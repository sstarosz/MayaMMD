"""
MayaMMD — Release Packaging Script

Assembles the final release zip from CMake-produced release directories.

Usage:
    python scripts/release/package_release.py [--build-root out/install]

The version is read from the CMake-generated ``mmd/_version.py`` (via ``GitVersion.cmake``).
Release directories are read from the build root and assembled into dist/.

Expected build directory layout (produced by CMake install):

    out/install/
    ├── MayaMMD-Maya2024-Windows/
    │   ├── MayaMMD.mod
    │   └── MayaMMD/
    │       ├── plug-ins/
    │       │   ├── MayaMMD.mll       # C++ entry point
    │       │   └── mmd/               # Python source modules
    │       ├── scripts/
    │       ├── icons/
    │       ├── README.md
    │       ├── LICENSE.txt
    │       └── CHANGELOG.md
    ├── MayaMMD-Maya2024-Linux/
    ├── MayaMMD-Maya2024-macOS/
    ├── MayaMMD-Maya2025-Windows/
    ├── MayaMMD-Maya2025-Linux/
    ├── MayaMMD-Maya2025-macOS/
    ├── MayaMMD-Maya2026-Windows/
    ├── MayaMMD-Maya2026-Linux/
    ├── MayaMMD-Maya2026-macOS/
    ├── MayaMMD-Maya2027-Windows/
    ├── MayaMMD-Maya2027-Linux/
    └── MayaMMD-Maya2027-macOS/

Output:

    dist/
    └── mayammd-v{version}.zip
        ├── install.py
        ├── INSTALL.txt
        ├── MayaMMD-Maya2024-Windows/
        ├── MayaMMD-Maya2024-Linux/
        ├── MayaMMD-Maya2024-macOS/
        ├── MayaMMD-Maya2025-Windows/
        ├── MayaMMD-Maya2025-Linux/
        ├── MayaMMD-Maya2025-macOS/
        ├── MayaMMD-Maya2026-Windows/
        ├── MayaMMD-Maya2026-Linux/
        ├── MayaMMD-Maya2026-macOS/
        ├── MayaMMD-Maya2027-Windows/
        ├── MayaMMD-Maya2027-Linux/
        └── MayaMMD-Maya2027-macOS/
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


# ---------------------------------------------------------------------------
# Script location — resolve all paths relative to the repo root
# ---------------------------------------------------------------------------
def _find_project_root() -> Path:
    """Walk up from this script to find the repo root (where pyproject.toml lives)."""
    current = Path(__file__).resolve().parent
    while current.parent != current:
        if (current / "pyproject.toml").exists():
            return current
        current = current.parent
    return current


PROJECT_ROOT = _find_project_root()

# Files from the repo root to include at the zip root.
# install.py lives in scripts/release/ but goes at the zip root so users
# can drag it onto the Maya viewport.
RELEASE_FILES: dict[str, str] = {
    "scripts/release/install.py": "install.py",
}


# ---------------------------------------------------------------------------
# Version
# ---------------------------------------------------------------------------
def _get_version() -> str:
    """Read the project version from the CMake-generated _version.py.

    CMake's GitVersion.cmake produces mmd/_version.py from git tags.
    This is the single source of truth — no hatch-vcs dependency.
    """
    version_py = PROJECT_ROOT / "mmd" / "_version.py"
    if version_py.exists():
        text = version_py.read_text(encoding="utf-8")
        match = re.search(
            r"__version__\s*=\s*(?:version\s*=\s*)?['\"]([^'\"]+)['\"]", text
        )
        if match:
            return match.group(1)

    # Fallback: try git directly (same logic as GitVersion.cmake)
    try:
        result = subprocess.run(
            ["git", "describe", "--tags", "--match", "v[0-9]*", "--abbrev=0"],
            capture_output=True,
            text=True,
            cwd=PROJECT_ROOT,
        )
        if result.returncode == 0:
            return result.stdout.strip().lstrip("v")
    except OSError:
        pass

    return "0.0.0+dev"


# ---------------------------------------------------------------------------
# Expected variants (4 Maya × 3 OS)
# ---------------------------------------------------------------------------
MAYA_VERSIONS = [2024, 2025, 2026, 2027]
OS_NAMES = ["Windows", "Linux", "macOS"]


def _find_release_dirs(build_root: Path) -> dict[tuple[int, str], Path]:
    """Scan the build root for CMake-produced release directories.

    Returns:
        dict mapping (maya_version, os_name) → Path to the release directory
    """
    found: dict[tuple[int, str], Path] = {}
    if not build_root.exists():
        return found

    for entry in sorted(build_root.iterdir()):
        if not entry.is_dir():
            continue
        # Match pattern: MayaMMD-Maya{ver}-{OS}
        name = entry.name
        if not name.startswith("MayaMMD-Maya"):
            continue
        rest = name[len("MayaMMD-Maya") :]
        for os_name in OS_NAMES:
            if rest.endswith(f"-{os_name}"):
                maya_str = rest[: -len(f"-{os_name}")]
                try:
                    maya_ver = int(maya_str)
                    found[(maya_ver, os_name)] = entry
                except ValueError:
                    pass
                break

    return found


# ---------------------------------------------------------------------------
# Packaging
# ---------------------------------------------------------------------------
def _assemble_zip(
    version: str,
    release_dirs: dict[tuple[int, str], Path],
    output_dir: Path,
    tmp_dir: Path,
) -> Path:
    """Assemble the release zip from pre-built release directories."""
    stage = tmp_dir / f"mayammd-v{version}"
    stage.mkdir(parents=True, exist_ok=True)

    # --- 1. Copy release files to zip root ---
    print("Copying release files...")
    for src_rel, dst_name in RELEASE_FILES.items():
        src = PROJECT_ROOT / src_rel
        if src.exists():
            shutil.copy2(src, stage / dst_name)
            print(f"  ✓ {dst_name}")
        else:
            print(f"  ⚠ {src_rel} not found — skipping")

    # Copy standalone install instructions
    install_txt_src = PROJECT_ROOT / "scripts" / "release" / "INSTALL.txt"
    if install_txt_src.exists():
        shutil.copy2(install_txt_src, stage / "INSTALL.txt")
        print(f"  ✓ INSTALL.txt")
    else:
        print(f"  ⚠ INSTALL.txt not found — skipping")

    # --- 2. Copy each release directory into the zip ---
    print("\nCopying release directories...")
    for (maya_ver, os_name), src_dir in sorted(release_dirs.items()):
        subdir_name = f"MayaMMD-Maya{maya_ver}-{os_name}"
        dst_dir = stage / subdir_name
        shutil.copytree(src_dir, dst_dir)

        # Count files for summary
        n_files = sum(1 for _ in dst_dir.rglob("*") if _.is_file())
        print(f"  ✓ {subdir_name}/ ({n_files} files)")

    # --- 3. Create the zip ---
    output_dir.mkdir(parents=True, exist_ok=True)
    zip_name = f"mayammd-v{version}.zip"
    zip_path = output_dir / zip_name

    print(f"\nCreating zip: {zip_path} ...")
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, _dirs, files in os.walk(stage):
            for file in files:
                file_path = Path(root) / file
                arcname = file_path.relative_to(stage)
                zf.write(file_path, arcname)

    print(f"✓ Created: {zip_path}")
    return zip_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="MayaMMD — Release Packaging")
    parser.add_argument(
        "--build-root",
        type=str,
        default=None,
        help="Path to build root containing release directories (default: out/install)",
    )
    args = parser.parse_args()

    version = _get_version()
    build_root = (
        Path(args.build_root) if args.build_root else (PROJECT_ROOT / "out" / "install")
    )
    output_dir = PROJECT_ROOT / "dist"

    print("=" * 60)
    print("MayaMMD — Release Packaging")
    print("=" * 60)
    print(f"Version:     v{version}")
    print(f"Build root:  {build_root}")
    print(f"Output:      {output_dir}")
    print()

    release_dirs = _find_release_dirs(build_root)
    if not release_dirs:
        print("✗ No release directories found!")
        print(f"  Expected directories in: {build_root}")
        print("  Run CMake release builds first:")
        print(
            "    cmake --preset maya2026-release -DBUILD_PACKAGE=ON && cmake --build out/build/maya2026-release && cmake --install out/build/maya2026-release"
        )
        return 1

    print(f"Found {len(release_dirs)} release directories:")
    for (maya_ver, os_name), path in sorted(release_dirs.items()):
        print(f"  Maya {maya_ver} / {os_name}: {path.name}")

    # Check completeness
    expected = len(MAYA_VERSIONS) * len(OS_NAMES)
    missing = expected - len(release_dirs)
    if missing > 0:
        print(f"\n  ⚠ {missing} of {expected} expected variants are missing")
        for mv in MAYA_VERSIONS:
            for osn in OS_NAMES:
                if (mv, osn) not in release_dirs:
                    print(f"      - MayaMMD-Maya{mv}-{osn}")
    else:
        print(f"\n  ✓ All {expected} expected variants present")

    # Assemble
    print()
    with tempfile.TemporaryDirectory(prefix="mmd_package_") as tmp:
        tmp_path = Path(tmp)
        zip_path = _assemble_zip(version, release_dirs, output_dir, tmp_path)

    # Summary
    print("\n" + "-" * 60)
    print("Package Summary")
    print("-" * 60)
    size_mb = zip_path.stat().st_size / (1024 * 1024)
    print(f"Zip size:     {size_mb:.1f} MB")
    print(f"Variants:     {len(release_dirs)}")
    print(f"Location:     {zip_path}")
    print("-" * 60)
    print("✓ Release package ready!")

    # List top-level contents
    print(f"\nZip contents ({zip_path.name}):")
    with zipfile.ZipFile(zip_path, "r") as zf:
        top_dirs = set()
        for info in zf.infolist():
            parts = info.filename.split("/")
            if len(parts) >= 1:
                top_dirs.add(
                    parts[0] if not info.is_dir() else info.filename.rstrip("/")
                )
        for d in sorted(top_dirs):
            print(f"  {d}/")

    return 0


if __name__ == "__main__":
    exit(main() or 0)
