"""MayaMMD — Build Script. Assembles the mmd/ source package for release."""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def _find_project_root() -> Path:
    """Walk up from this script to find the repo root (where pyproject.toml lives)."""
    current = Path(__file__).resolve().parent
    while current.parent != current:
        if (current / "pyproject.toml").exists():
            return current
        current = current.parent
    return current


PROJECT_ROOT = _find_project_root()
MMD_PACKAGE = PROJECT_ROOT / "mmd"


def _copy_python_sources(src: Path, dst: Path) -> None:
    """Copy all .py files from *src* into *dst*, preserving the directory tree."""
    for py_file in src.rglob("*.py"):
        rel = py_file.relative_to(src)
        (dst / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(py_file, dst / rel)


def _copy_icons(src: Path, dst: Path) -> None:
    """Copy the icons directory (non-Python assets) into the staging output."""
    icons_src = src / "icons"
    if not icons_src.is_dir():
        print("  No icons directory found — skipping")
        return
    icons_dst = dst / "icons"
    icons_dst.mkdir(parents=True, exist_ok=True)
    for icon_file in icons_src.iterdir():
        if icon_file.is_file():
            shutil.copy2(icon_file, icons_dst / icon_file.name)
    print(f"  Copied icon(s) -> {icons_dst}")


def _copy_version_file(output_dir: Path) -> None:
    """Copy CMake-generated _version.py into the staging output.

    CMake owns _version.py generation (from git tags via GitVersion.cmake).
    We copy it into the staging directory with the release version baked in.
    """
    src = PROJECT_ROOT / "mmd" / "_version.py"
    if src.exists():
        shutil.copy2(src, output_dir / "_version.py")
        print(f"  Copied {src} -> {output_dir / '_version.py'}")
    else:
        print("  WARNING: mmd/_version.py not found — version will be missing")


def main() -> int:
    import argparse

    _parser = argparse.ArgumentParser()
    _parser.add_argument("--output-dir", type=str, required=True)
    _parser.add_argument("--version", type=str, default="0.0.0")
    _args = _parser.parse_args()

    output_dir = Path(_args.output_dir) / "mmd"

    print("=" * 60)
    print("MayaMMD — Build Release")
    print("=" * 60)
    print(f"Version: {_args.version} | Output: {output_dir}")
    print()

    # Clean and recreate output directory
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    # Copy all Python source files
    py_count = sum(1 for _ in MMD_PACKAGE.rglob("*.py"))
    print(f"Copying {py_count} Python source file(s)...")
    _copy_python_sources(MMD_PACKAGE, output_dir)

    # Copy non-Python assets (e.g., the Maya shelf icon)
    print("Copying icons...")
    _copy_icons(MMD_PACKAGE, output_dir)

    # Inject version file
    print("Injecting version...")
    _copy_version_file(output_dir)

    print()
    print("-" * 60)
    print(f"Output: {output_dir}")
    print("-" * 60)
    print("[OK] Build complete!")
    return 0


if __name__ == "__main__":
    exit(main())
