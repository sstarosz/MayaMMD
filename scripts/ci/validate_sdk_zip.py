"""
Validate a Maya SDK combined package (sdk-maya{version}.zip).

Usage:
    python scripts/ci/validate_sdk_zip.py out/sdk-maya2026.zip

Exit code:
    0 = valid
    1 = validation errors found
"""

from __future__ import annotations

import sys
import zipfile
from pathlib import Path

# Expected structure per platform
EXPECTED_LIBS: dict[str, list[str]] = {
    "windows": [
        "Foundation.lib",
        "OpenMaya.lib",
        "OpenMayaAnim.lib",
        "OpenMayaFX.lib",
        "OpenMayaRender.lib",
        "OpenMayaUI.lib",
    ],
    "linux": [
        "libFoundation.so",
        "libOpenMaya.so",
        "libOpenMayaAnim.so",
        "libOpenMayaFX.so",
        "libOpenMayaRender.so",
        "libOpenMayaUI.so",
    ],
    "macos": [
        "libFoundation.dylib",
        "libOpenMaya.dylib",
        "libOpenMayaAnim.dylib",
        "libOpenMayaFX.dylib",
        "libOpenMayaRender.dylib",
        "libOpenMayaUI.dylib",
    ],
}

REQUIRED_HEADERS = [
    "include/maya/MFnPlugin.h",
    "include/maya/MFnDependencyNode.h",
    "include/maya/MPxIkSolverNode.h",
    "include/maya/MPxNode.h",
    "include/maya/MGlobal.h",
    "include/maya/MObject.h",
    "include/maya/MStatus.h",
    "include/maya/MFnPlugin.h",
]


def validate(zip_path: Path) -> int:
    errors: list[str] = []
    warnings: list[str] = []
    info: list[str] = []

    if not zip_path.exists():
        print(f"✗ File not found: {zip_path}")
        return 1

    size_mb = zip_path.stat().st_size / (1024 * 1024)
    info.append(f"File size: {size_mb:.1f} MB")

    try:
        with zipfile.ZipFile(zip_path, "r") as zf:
            names = zf.namelist()
            files = [n for n in names if not n.endswith("/")]
            dirs = [n for n in names if n.endswith("/")]
    except zipfile.BadZipFile as e:
        print(f"✗ Bad ZIP file: {e}")
        return 1

    info.append(f"Total entries: {len(files)} files, {len(dirs)} directories")

    # ── 1. Top-level structure ──
    top_dirs = set(n.split("/")[0] for n in files)
    info.append(f"Top-level directories: {', '.join(sorted(top_dirs))}")

    if "include" not in top_dirs:
        errors.append("Missing top-level include/ directory")
    if "lib" not in top_dirs:
        errors.append("Missing top-level lib/ directory")

    # ── 2. Headers ──
    header_count = len([n for n in files if n.startswith("include/")])
    info.append(f"Header files: {header_count}")
    if header_count < 500:
        warnings.append(f"Low header count ({header_count}) — expected 500+")

    for hdr in REQUIRED_HEADERS:
        if not any(hdr == n for n in files):
            # Try partial match (some paths may differ)
            if not any(hdr.replace("include/", "") in n for n in files):
                errors.append(f"Missing required header: {hdr}")

    # ── 3. Platform libraries ──
    platforms_found: list[str] = []
    for plat, expected in EXPECTED_LIBS.items():
        plat_libs = [n for n in files if n.startswith(f"lib/{plat}/")]
        if plat_libs:
            platforms_found.append(plat)
            info.append(f"  lib/{plat}/: {len(plat_libs)} files")

            # Check essential libraries
            for lib in expected:
                if not any(lib in n for n in plat_libs):
                    # Try without extension (some platforms strip it)
                    lib_stem = Path(lib).stem
                    if not any(lib_stem in n for n in plat_libs):
                        warnings.append(f"  Possibly missing {plat}/{lib}")
        else:
            warnings.append(
                f"No libraries for platform '{plat}' (only needed if targeting this platform)"
            )

    if not platforms_found:
        errors.append("No platform library directories found in lib/")

    # ── 4. Size sanity ──
    if size_mb < 20:
        warnings.append(f"Package is very small ({size_mb:.1f} MB) — may be incomplete")
    elif size_mb < 35:
        warnings.append(
            f"Package has only {len(platforms_found)} platform(s). "
            f"If all platforms are expected, something may be missing."
        )

    # ── 5. Check for common unwanted content ──
    unwanted = ["devkit/", "samples/", "Qt.zip", "Autodesk_EULA", "boost.zip"]
    for pattern in unwanted:
        if any(pattern in n for n in names):
            warnings.append(f"Contains potentially unnecessary content: {pattern}")

    # ── Output ──
    print(f"\n=== Validation Report: {zip_path.name} ===")
    for line in info:
        print(f"  {line}")

    if warnings:
        print(f"\n  ⚠ Warnings ({len(warnings)}):")
        for w in warnings:
            print(f"    • {w}")

    if errors:
        print(f"\n  ✗ ERRORS ({len(errors)}):")
        for e in errors:
            print(f"    • {e}")
        print(f"\n  ✗ VALIDATION FAILED")
        return 1
    else:
        platform_str = "+".join(platforms_found) if platforms_found else "none"
        print(f"\n  ✓ VALID — {len(platforms_found)} platform(s): {platform_str}")
        if warnings:
            print(f"    ({len(warnings)} warning(s) — review above)")
        return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(
            "Usage: python scripts/ci/validate_sdk_zip.py <path/to/sdk-maya{version}.zip>"
        )
        print()
        print("Examples:")
        print("  python scripts/ci/validate_sdk_zip.py out/sdk-maya2026.zip")
        print("  python scripts/ci/validate_sdk_zip.py out/.sdk/sdk-maya2026.zip")
        return 1

    zip_path = Path(sys.argv[1])
    return validate(zip_path)


if __name__ == "__main__":
    exit(main())
