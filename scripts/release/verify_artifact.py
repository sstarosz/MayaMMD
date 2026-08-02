"""
Verify MayaMMD release artifacts.

Usage:
    python scripts/release/verify_artifact.py <path/to/release.zip>
    python scripts/release/verify_artifact.py --scripts
    python scripts/release/verify_artifact.py <path/to/release.zip> --scripts
"""

from __future__ import annotations

import argparse
import sys
import zipfile
from pathlib import Path

# ---------------------------------------------------------------------------
# Expected structure
# ---------------------------------------------------------------------------
# Maya versions that should be present in a complete release
MAYA_VERSIONS = ["2024", "2025", "2026", "2027"]
PLATFORM_NAMES: dict[str, str] = {
    "win32": "Windows",
    "linux": "Linux",
    "darwin": "macOS",
}


# Files that must exist at zip root
REQUIRED_ROOT_FILES = {
    "install.py",
    "INSTALL.txt",
}

# Paths that must exist inside each platform subdirectory (Maya Module structure).
# The platform dir contains MayaMMD.mod + an MayaMMD/ module folder.
REQUIRED_INNER_FILES = {
    "MayaMMD/plug-ins/mmd/__init__.py",
}

# Native Maya plugin extension per platform
_NATIVE_PLUGIN_EXT: dict[str, str] = {
    "Windows": ".mll",
    "Linux": ".so",
    "macOS": ".bundle",
}


def _get_expected_loader(platform_dir_name: str) -> str:
    """Return the expected loader path for a given platform directory name.

    Example:
        "MayaMMD-Maya2024-Windows" → "MayaMMD/plug-ins/MayaMMD.mll"
        "MayaMMD-Maya2025-Linux"   → "MayaMMD/plug-ins/MayaMMD.so"
        "MayaMMD-Maya2026-macOS"   → "MayaMMD/plug-ins/MayaMMD.bundle"
    """
    for plat_name, ext in _NATIVE_PLUGIN_EXT.items():
        if platform_dir_name.endswith(f"-{plat_name}"):
            return f"MayaMMD/plug-ins/MayaMMD{ext}"
    # Fallback: try .mll (Windows convention)
    return "MayaMMD/plug-ins/MayaMMD.mll"


# File extensions for native plugin binaries (C++ .mll/.so/.bundle)
NATIVE_EXTENSIONS = {".mll", ".so", ".bundle"}


# ---------------------------------------------------------------------------
# Verifier
# ---------------------------------------------------------------------------
class ArtifactError(Exception):
    """Raised when a verification check fails."""


def verify_zip(zip_path: Path) -> list[str]:
    """
    Open *zip_path* and verify its contents.

    Returns a list of human-readable log messages.  Raises ``ArtifactError``
    if a critical check fails.
    """
    messages: list[str] = []

    if not zip_path.exists():
        raise ArtifactError(f"Zip not found: {zip_path}")

    size_kb = zip_path.stat().st_size / 1024
    messages.append(f"Zip size: {size_kb:.1f} KB")

    with zipfile.ZipFile(zip_path, "r") as zf:
        names = zf.namelist()
        all_names_set = set(names)

        # --- Root-level files ---
        missing_root = REQUIRED_ROOT_FILES - all_names_set
        if missing_root:
            raise ArtifactError(f"Missing root files: {sorted(missing_root)}")
        messages.append(f"✓ All {len(REQUIRED_ROOT_FILES)} root files present")

        # --- Platform subdirectories ---
        platform_dirs = set()
        native_count = 0
        py_count = 0
        for name in names:
            parts = name.split("/")
            if len(parts) >= 2 and parts[0].startswith("MayaMMD-"):
                platform_dirs.add(parts[0])
            if any(name.endswith(ext) for ext in NATIVE_EXTENSIONS):
                native_count += 1
            if name.endswith(".py"):
                py_count += 1

        messages.append(f"  Native plugins: {native_count}  Python modules: {py_count}")

        expected_dirs = set()
        for maya in MAYA_VERSIONS:
            for sys_plat in PLATFORM_NAMES:
                plat_name = PLATFORM_NAMES[sys_plat]
                expected_dirs.add(f"MayaMMD-Maya{maya}-{plat_name}")
        missing_dirs = expected_dirs - platform_dirs
        if missing_dirs:
            messages.append(f"⚠ Missing platform dirs: {sorted(missing_dirs)}")
        else:
            messages.append(f"✓ All {len(expected_dirs)} platform dirs present")

        # --- Check each platform subdirectory has minimal expected files ---
        errors: list[str] = []
        for platform_dir in sorted(platform_dirs):
            prefix = f"{platform_dir}/"
            inner_files = {n[len(prefix) :] for n in names if n.startswith(prefix)}

            missing_inner = REQUIRED_INNER_FILES - inner_files
            if missing_inner:
                msg = f"  {platform_dir}: missing {sorted(missing_inner)}"
                messages.append(msg)
                errors.append(msg)
                continue

            expected_loader = _get_expected_loader(platform_dir)
            has_loader = expected_loader in inner_files
            if not has_loader:
                msg = f"  {platform_dir}: missing {expected_loader}"
                messages.append(msg)
                errors.append(msg)
                continue

            # Check for Python modules inside MayaMMD/plug-ins/mmd/
            mmd_prefix = f"{prefix}MayaMMD/plug-ins/mmd/"
            has_py = any(
                n.endswith(".py") and n.startswith(mmd_prefix)
                for n in names
            )
            py_note = " ✓ python" if has_py else " ⚠ no python modules"
            messages.append(f"  {platform_dir}: ✓{py_note}")

        if errors:
            raise ArtifactError(
                f"{len(errors)} platform director{'y' if len(errors) == 1 else 'ies'} "
                f"missing required files:\n" + "\n".join(errors)
            )

        # --- Version sanity check from mmd/_version.py ---
        version_found = False
        for name in names:
            if name.endswith("mmd/_version.py"):
                try:
                    content = zf.read(name).decode("utf-8")
                    if "__version__" in content:
                        version_found = True
                        break
                except (UnicodeDecodeError, RuntimeError):
                    pass
        if version_found:
            messages.append("✓ mmd/_version.py contains __version__")
        else:
            messages.append("⚠ Version check skipped (no mmd/_version.py found)")

        # --- Check install.py is valid Python ---
        if "install.py" in all_names_set:
            try:
                compile(zf.read("install.py"), "install.py", "exec")
                messages.append("✓ install.py syntax valid")
            except SyntaxError as e:
                raise ArtifactError(f"install.py syntax error: {e}")

    return messages


# ---------------------------------------------------------------------------
# Release scripts validation (local repo, not zip)
# ---------------------------------------------------------------------------
def verify_release_scripts(scripts_dir: Path) -> list[str]:
    """
    Validate the syntax of all release Python scripts in *scripts_dir*.

    Returns a list of log messages.  Raises ``ArtifactError`` if any script
    has a syntax error.
    """
    messages: list[str] = []
    py_files = sorted(scripts_dir.glob("*.py"))
    if not py_files:
        raise ArtifactError(f"No Python scripts found in {scripts_dir}")

    for py_file in py_files:
        try:
            compile(py_file.read_text(encoding="utf-8"), str(py_file), "exec")
            messages.append(f"  ✓ {py_file.name}")
        except SyntaxError as e:
            raise ArtifactError(f"{py_file.name} syntax error: {e}") from e

    messages.insert(0, f"✓ All {len(py_files)} release script(s) syntax-valid")
    return messages


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify MayaMMD release zip integrity"
    )
    parser.add_argument(
        "zip_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the release zip file",
    )
    parser.add_argument(
        "--scripts",
        action="store_true",
        help="Also validate release scripts in scripts/release/",
    )
    args = parser.parse_args()

    exit_code = 0

    # --- Validate release zip ---
    if args.zip_path:
        try:
            messages = verify_zip(Path(args.zip_path))
            print("\n" + "─" * 50)
            for msg in messages:
                print(f"  {msg}")
            print("─" * 50)
            print("✓ Zip verification passed!")
        except ArtifactError as e:
            print(f"\n✗ Zip verification failed: {e}", file=sys.stderr)
            exit_code = 1
    elif not args.scripts:
        parser.error("Either zip_path or --scripts is required")

    # --- Validate release scripts ---
    if args.scripts:
        # Find scripts/release/ relative to this file
        script_dir = Path(__file__).resolve().parent
        try:
            messages = verify_release_scripts(script_dir)
            print("\n" + "─" * 50)
            for msg in messages:
                print(f"  {msg}")
            print("─" * 50)
            print("✓ Release scripts verification passed!")
        except ArtifactError as e:
            print(f"\n✗ Release scripts verification failed: {e}", file=sys.stderr)
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
