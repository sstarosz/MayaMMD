"""
MayaMMD — Maya SDK Manager

Downloads Maya DevKits from Autodesk's S3 bucket, strips them to the
minimum needed for plugin compilation (headers + libs), and archives
the result for CI caching.

Usage:
    # Auto-download + strip the SDK needed for the current Python version
    python scripts/ci/strip_maya_sdk.py ensure

    # Auto-download + strip a specific version
    python scripts/ci/strip_maya_sdk.py ensure --version 2026

    # Strip a local Maya installation
    python scripts/ci/strip_maya_sdk.py strip --source "source_dir" --output build/.sdk/sdk-maya2026

    # Strip a downloaded DevKit ZIP
    python scripts/ci/strip_maya_sdk.py strip --source Maya_DevKit_2026.zip --output build/.sdk/sdk-maya2026

    # Re-zip the stripped SDK for upload
    python scripts/ci/strip_maya_sdk.py archive --input build/.sdk/sdk-maya2026 --output build/.sdk/sdk-maya2026.zip

    # List available DevKits with known URLs
    python scripts/ci/strip_maya_sdk.py list-urls

    # Download a DevKit
    python scripts/ci/strip_maya_sdk.py download --version 2026 --platform Windows --output devkit.zip
"""

from __future__ import annotations

import argparse
import concurrent.futures
import os
import shutil
import subprocess
import sys
import tarfile
import threading
import zipfile
from pathlib import Path

# Ensure UTF-8-safe stdout — some Windows consoles (cp1250/cp1252) can't encode
# the ✓/✗/⚠ glyphs used in this script's progress output.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

# Print lock for clean concurrent output
_print_lock = threading.Lock()

# ---------------------------------------------------------------------------
# Known DevKit URLs (latest update per Maya version per platform)
# ---------------------------------------------------------------------------
DEVKIT_URLS: dict[int, dict[str, str]] = {
    2024: {
        "Windows": (
            "https://autodesk-adn-transfer.s3-us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2024/"
            "Autodesk_Maya_2024_2_Update_DEVKIT_Windows.zip"
        ),
        "Linux": (
            "https://autodesk-adn-transfer.s3-us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2024/"
            "Autodesk_Maya_2024_2_Update_DEVKIT_Linux.tgz"
        ),
        "macOS": (
            "https://autodesk-adn-transfer.s3-us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2024/"
            "Autodesk_Maya_2024_2_Update_DEVKIT_Mac.dmg"
        ),
    },
    2025: {
        "Windows": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2025/"
            "Autodesk_Maya_2025_3_Update_DEVKIT_Windows.zip"
        ),
        "Linux": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2025/"
            "Autodesk_Maya_2025_3_Update_DEVKIT_Linux.tgz"
        ),
        "macOS": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2025/"
            "Autodesk_Maya_2025_3_Update_DEVKIT_Mac.dmg"
        ),
    },
    2026: {
        "Windows": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2026/"
            "Autodesk_Maya_2026_3_Update_DEVKIT_Windows.zip"
        ),
        "Linux": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2026/"
            "Autodesk_Maya_2026_3_Update_DEVKIT_Linux.tgz"
        ),
        "macOS": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2026/"
            "Autodesk_Maya_2026_3_Update_DEVKIT_Mac.dmg"
        ),
    },
    2027: {
        "Windows": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2027/"
            "Autodesk_Maya_2027_1_Update_DEVKIT_Windows.zip"
        ),
        "Linux": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2027/"
            "Autodesk_Maya_2027_1_Update_DEVKIT_Linux.tgz"
        ),
        "macOS": (
            "https://autodesk-adn-transfer.s3.us-west-2.amazonaws.com/"
            "ADN+Extranet/M%26E/Maya/devkit+2027/"
            "Autodesk_Maya_2027_1_Update_DEVKIT_Mac.dmg"
        ),
    },
}

# ---------------------------------------------------------------------------
# Optional GitHub Release cache — pre-stripped SDKs for fast download.
# Set MAYAMMD_SDK_CACHE_URL to use a pre-stripped cache instead of
# downloading the full DevKit from Autodesk (~44 MB vs ~527 MB).
#
#   $env:MAYAMMD_SDK_CACHE_URL = "https://github.com/owner/repo/releases/download/tag"
#
# If not set, 'ensure' downloads directly from Autodesk (public, no auth needed).
# ---------------------------------------------------------------------------
_SDK_CACHE_URL = os.environ.get("MAYAMMD_SDK_CACHE_URL", "")


def _github_asset_url(version: int) -> str:
    """URL of the pre-stripped SDK on GitHub Releases (all platforms combined)."""
    base = _SDK_CACHE_URL.rstrip("/")
    return f"{base}/sdk-maya{version}.zip"


# ---------------------------------------------------------------------------
# SDK cache directory
# ---------------------------------------------------------------------------
# Where `ensure` keeps extracted SDKs for daily development use.
# Override with MMD_SDK_DIR env var.
# ---------------------------------------------------------------------------
_SDK_CACHE = Path(
    os.environ.get(
        "MMD_SDK_DIR", Path(__file__).resolve().parent.parent.parent / "out" / ".sdk"
    )
)

# Python version → Maya version mapping
PYTHON_TO_MAYA: dict[str, int] = {
    "3.10": 2024,
    "3.11": 2025,  # Covers 2025 and 2026
    "3.12": 2026,
    "3.13": 2027,
}

# Maya libraries required for plugin compilation
REQUIRED_LIBS: list[str] = [
    "Foundation",
    "OpenMaya",
    "OpenMayaAnim",
    "OpenMayaFX",
    "OpenMayaRender",
    "OpenMayaUI",
    "Image",
]


def _sdk_dir(version: int) -> Path:
    return _SDK_CACHE / f"sdk-maya{version}"


def _ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def _should_keep_all_libs(name: str) -> bool:
    """All .lib files in the DevKit are needed (~14 libs, 14 MB total)."""
    return name.endswith(".lib")


def _detect_maya_version() -> int:
    """Map the running Python version to the matching Maya version."""
    py_ver = f"{sys.version_info.major}.{sys.version_info.minor}"
    return PYTHON_TO_MAYA.get(py_ver, 2026)  # 2026 fallback


def _current_platform() -> str:
    """Return 'windows', 'linux', or 'macos' for the current system."""
    if sys.platform == "win32":
        return "windows"
    elif sys.platform == "darwin":
        return "macos"
    else:
        return "linux"


def _setup_platform_libs(sdk_path: Path) -> None:
    """After extracting a combined SDK, set up the current platform's libs.

    Combined package has: lib/windows/, lib/linux/, lib/macos/
    We copy the current platform's libs to lib/ so FindMaya.cmake finds them.
    """
    plat = _current_platform()
    src = sdk_path / "lib" / plat
    dst = sdk_path / "lib"

    if not src.exists():
        print(f"  ⚠ No libs for platform '{plat}' in combined SDK")
        return

    moved = 0
    for f in src.iterdir():
        if f.is_file():
            shutil.copy2(f, dst / f.name)
            moved += 1

    # Remove platform subdirectories
    for p in ["windows", "linux", "macos"]:
        sub = sdk_path / "lib" / p
        if sub.exists():
            shutil.rmtree(sub)

    print(f"  Copied {moved} libs for platform '{plat}'")


def _is_complete_archive(path: Path) -> bool:
    """Return True if *path* looks like a complete (non-truncated) archive."""
    if not path.exists() or path.stat().st_size == 0:
        return False
    name = path.name.lower()
    if name.endswith(".zip"):
        try:
            with zipfile.ZipFile(path, "r") as zf:
                zf.namelist()  # reads the central directory — truncated zips fail here
            return True
        except (zipfile.BadZipFile, OSError):
            return False
    if name.endswith((".tgz", ".tar.gz")):
        try:
            import tarfile

            with tarfile.open(path, "r:gz") as tf:
                tf.getmembers()
            return True
        except (tarfile.TarError, OSError, EOFError):
            return False
    # Other formats (.dmg, ...) — require a plausible non-empty size
    return path.stat().st_size > 1_000_000


def _download_file(url: str, dest: Path, label: str = "") -> bool:
    """Download *url* to *dest* with progress. Reuses *dest* only if it is a
    complete archive; removes and re-downloads truncated/corrupt files.
    Returns True on success."""
    import urllib.request

    if dest.exists():
        if _is_complete_archive(dest):
            size_mb = dest.stat().st_size / (1024 * 1024)
            print(f"    ✓ already cached: {dest.name} ({size_mb:.0f} MB)")
            return True
        size_mb = dest.stat().st_size / (1024 * 1024)
        print(f"    ⚠ cached file is incomplete ({size_mb:.1f} MB) — re-downloading")
        try:
            dest.unlink()
        except OSError:
            pass

    if label:
        print(f"  [{threading.current_thread().name}] Downloading {label}...")

    def _report(c, bs, ts):
        if ts > 0:
            with _print_lock:
                pct = c * bs * 100 / ts
                dl = c * bs / (1024 * 1024)
                tot = ts / (1024 * 1024)
                print(f"\r    {dl:.0f}/{tot:.0f} MB ({pct:.0f}%)", end="", flush=True)

    try:
        urllib.request.urlretrieve(url, dest, reporthook=_report)
        print(f"\n    ✓ {dest.name} ({dest.stat().st_size / (1024*1024):.0f} MB)")
        return True
    except Exception as e:
        print(f"\n    ✗ Failed: {e}")
        return False


def _download_github_stripped(version: int, dest: Path) -> bool:
    """Try to download a pre-stripped SDK from the configured cache (fast, ~44 MB).
    Requires MAYAMMD_SDK_CACHE_URL to be set.
    Uses gh CLI for auth (private repos) or plain HTTP (public repos).
    Returns False if not configured or fails.
    """
    if not _SDK_CACHE_URL:
        return False

    # Parse owner/repo/tag from the cache URL
    # URL format: https://github.com/{owner}/{repo}/releases/download/{tag}
    import re
    m = re.match(r"https://github\.com/([^/]+)/([^/]+)/releases/download/([^/]+)/?", _SDK_CACHE_URL)
    if not m:
        print(f"  ⚠ Invalid MAYAMMD_SDK_CACHE_URL format: {_SDK_CACHE_URL}")
        return False
    owner, repo, tag = m.group(1), m.group(2), m.group(3)
    asset = f"sdk-maya{version}.zip"

    # Try gh CLI first (works with GH_TOKEN for private repos)
    gh = shutil.which("gh")
    if gh:
        print(f"  Downloading {asset} via gh CLI ({owner}/{repo})...")
        result = subprocess.run(
            [gh, "release", "download", tag, "--pattern", asset,
             "--dir", str(dest.parent), "--repo", f"{owner}/{repo}", "--clobber"],
            capture_output=True, text=True, timeout=120,
        )
        if result.returncode == 0 and dest.exists():
            print(f"    ✓ {asset} ({dest.stat().st_size / (1024*1024):.0f} MB)")
            return True
        # If gh fails (no auth, wrong repo, etc.), fall through to HTTP

    # Fallback: direct HTTP download (works for public repos)
    url = _github_asset_url(version)
    print(f"  Attempting direct download...")
    if _download_file(url, dest, f"stripped Maya {version} SDK"):
        return True
    print(f"  (cache not available — falling back to Autodesk)")
    return False


def cmd_strip(args: argparse.Namespace) -> int:
    """Strip a Maya installation or DevKit ZIP to minimum (headers + libs only)."""
    source = Path(args.source)
    output = Path(args.output)

    if not source.exists():
        print(f"✗ Source not found: {source}")
        return 1

    _ensure_dir(output)

    # ── Determine if source is a ZIP or a directory ──
    is_zip = source.suffix.lower() in (".zip",)

    if is_zip:
        print(f"Extracting from ZIP: {source.name}")
        _strip_from_zip(source, output, args)
    else:
        print(f"Copying from directory: {source}")
        _strip_from_dir(source, output, args)

    print(f"\n✓ Stripped SDK written to: {output}")
    _print_summary(output)
    return 0


def _strip_from_zip(zip_path: Path, output: Path, args: argparse.Namespace) -> None:
    """Extract only the relevant parts from a DevKit ZIP."""
    # Patterns for essential Maya libraries
    _ESSENTIAL_LIB_PREFIXES = [
        "Foundation",
        "OpenMaya",
        "Image",
        "tbb",
        "cg",
        "clew",
        "adskIMF",
        "awxml2",
        "MetaData",
    ]

    with zipfile.ZipFile(zip_path, "r") as zf:
        for entry in zf.namelist():
            if not entry.endswith("/"):
                parts = Path(entry).parts
                # Skip non-essential top-level directories
                if "samples" in parts or "doc" in parts or "docs" in parts:
                    continue
                if "python" in parts:
                    continue
                # For lib/, only keep essential Maya libs
                if "lib" in parts:
                    fname = Path(entry).name
                    lower = fname.lower()
                    if not any(
                        lower.startswith(p.lower()) for p in _ESSENTIAL_LIB_PREFIXES
                    ):
                        continue
                # Keep include/ and lib/
                if "include" in parts or "lib" in parts:
                    zf.extract(entry, output)
                    print(f"  {entry}")


def _strip_from_dir(source: Path, output: Path, args: argparse.Namespace) -> None:
    """Copy relevant files from a Maya installation or DevKit directory.

    DevKit structure (source = devkitBase/):
        include/   →  KEEP (maya/, Python311/, tbb/, Cg/, etc.)
        lib/       →  KEEP only essential Maya libraries
        cmake/     →  KEEP (CMake modules)
        devkit/    →  SKIP (tools/binaries/samples)
        Qt.zip     →  SKIP (444 MB, not needed with PySide6)
        *.zip      →  SKIP

    For lib/, only essential Maya libraries are kept:
        Foundation*, OpenMaya*, Image*, tbb*, cg*, clew*, adskIMF*, awxml2*
    Other bundled libs (Qt, Python, Boost, etc.) are skipped — they come
    from Maya's own runtime or from PySide6.
    """
    # Patterns for essential Maya libraries
    _ESSENTIAL_LIB_PREFIXES = [
        "Foundation",
        "OpenMaya",
        "Image",
        "tbb",
        "cg",
        "clew",
        "adskIMF",
        "awxml2",
        "MetaData",
        "libFoundation",
        "libOpenMaya",
        "libImage",
        "libtbb",
        "libcg",
        "libclew",
    ]

    def _is_essential_lib(name: str) -> bool:
        lower = name.lower()
        for prefix in _ESSENTIAL_LIB_PREFIXES:
            if lower.startswith(prefix.lower()):
                return True
        return False

    entries = list(source.iterdir())

    for item in entries:
        name = item.name.lower()

        # ── Skip clearly unneeded items ──
        if name == "devkit" and item.is_dir():
            devkit_size = sum(
                f.stat().st_size for f in item.rglob("*") if f.is_file()
            ) / (1024 * 1024)
            print(f"  SKIP devkit/  ({devkit_size:.0f} MB — samples, tools, binaries)")
            continue
        if name == "qt.zip" or name.endswith(".zip"):
            print(f"  SKIP {item.name}")
            continue
        if name in ("autodesk_eula.pdf", "license.pdf", "readme.md"):
            continue

        # ── Copy everything else (include/, lib/, cmake/, etc.) ──
        if item.is_dir():
            dst = output / item.name
            dst.mkdir(parents=True, exist_ok=True)
            # For lib/, only copy essential Maya libraries
            if item.name.lower() == "lib":
                copied = 0
                skipped = 0
                for f in item.iterdir():
                    if f.is_file():
                        if _is_essential_lib(f.name):
                            shutil.copy2(f, dst / f.name)
                            copied += 1
                        else:
                            skipped += 1
                total_size = sum(
                    f.stat().st_size for f in dst.iterdir() if f.is_file()
                ) / (1024 * 1024)
                print(
                    f"  lib/  ({copied} files, skipped {skipped}, {total_size:.1f} MB)"
                )
            else:
                shutil.copytree(item, dst, dirs_exist_ok=True)
                n_files = len(list(dst.rglob("*")))
                size_mb = sum(
                    f.stat().st_size for f in dst.rglob("*") if f.is_file()
                ) / (1024 * 1024)
                print(f"  {item.name}/  ({n_files} items, {size_mb:.1f} MB)")
        elif item.is_file():
            shutil.copy2(item, output / item.name)
            print(f"  {item.name}  ({item.stat().st_size / 1024:.0f} KB)")


def _print_summary(output: Path) -> None:
    """Show what ended up in the stripped output."""
    total_size = 0
    for f in output.rglob("*"):
        if f.is_file():
            total_size += f.stat().st_size
    print(f"  Total size: {total_size / 1024 / 1024:.1f} MB")


# ---------------------------------------------------------------------------
# Helpers: extract DevKit archives of various formats
# ---------------------------------------------------------------------------
def _extract_archive(archive_path: Path, dest: Path) -> Path | None:
    """Extract a DevKit archive to *dest* and return the path to devkitBase/.

    Supports .zip, .tgz/.tar.gz, and .dmg (via 7-Zip on Windows).
    Returns None if extraction fails.
    """
    name = archive_path.name.lower()

    if name.endswith(".zip"):
        print(f"  Extracting ZIP: {archive_path.name}")
        with zipfile.ZipFile(archive_path, "r") as zf:
            zf.extractall(dest)

    elif name.endswith((".tgz", ".tar.gz")):
        print(f"  Extracting TGZ: {archive_path.name}")
        import tarfile

        with tarfile.open(archive_path, "r:gz") as tf:
            tf.extractall(dest)

    elif name.endswith(".dmg"):
        print(f"  Extracting DMG: {archive_path.name}")
        # Try 7-Zip first (available on Windows)
        _7z = shutil.which("7z") or shutil.which("7za")
        if _7z is None:
            _7z = "C:/Program Files/7-Zip/7z.exe"
        if os.path.exists(_7z):
            import subprocess

            # -snl disables symlink restoration (needs admin on Windows — we don't need them)
            result = subprocess.run(
                [_7z, "x", str(archive_path), f"-o{dest}", "-y", "-snl"],
                capture_output=True,
                text=True,
                timeout=180,
            )
            if result.returncode != 0:
                # If -snl didn't help, try without (some 7z versions lack this flag)
                if "-snl" in result.stderr or "unknown option" in result.stderr.lower():
                    result = subprocess.run(
                        [_7z, "x", str(archive_path), f"-o{dest}", "-y"],
                        capture_output=True,
                        text=True,
                        timeout=180,
                    )
            if result.returncode != 0:
                print(f"  ⚠ 7-Zip extraction failed (rc={result.returncode})")
                print(f"    Try extracting manually, then provide --sdk-macos path.")
                # Strip back to macOS anyway — maybe enough files got through
                if (
                    dest.exists()
                    and (dest / "devkitBase" / "include" / "maya").exists()
                ):
                    print(f"    (partial extraction found — continuing)")
                else:
                    return None
            print(f"  7-Zip extraction complete")
        else:
            print(f"  ⚠ No tool available to extract .dmg on this platform.")
            print(
                f"    Install 7-Zip or extract on macOS and provide --sdk-macos path."
            )
            return None

    else:
        print(f"  ⚠ Unknown archive format: {archive_path.suffix}")
        return None

    # Find devkitBase/ in the extracted directory
    candidates = [
        dest / "devkitBase",
    ]
    # Also search one level down for the extracted folder name
    for item in dest.iterdir():
        if item.is_dir():
            inner = item / "devkitBase"
            if inner.exists():
                candidates.append(inner)
            else:
                candidates.append(item)  # Try the extracted folder itself

    for c in candidates:
        if c.exists() and (c / "include" / "maya").exists():
            return c

    return dest  # Return dest as fallback — maybe it IS devkitBase


# ---------------------------------------------------------------------------
# Command: ensure — auto-download + strip SDK for the current Python version
# ---------------------------------------------------------------------------
def cmd_ensure(args: argparse.Namespace) -> int:
    """Ensure a Maya SDK is available locally.

    Strategy (fastest first):
      1. Already extracted at thirdy-party/sdk-maya{version}/  →  done
      2. Download stripped SDK from GitHub Releases            →  fast (~44 MB)
      3. Download full DevKit from Autodesk                    →  slow (~527 MB), strips it
    """
    version = args.version or _detect_maya_version()
    if _SDK_CACHE_URL:
        print(f"SDK cache: {_SDK_CACHE_URL}")
    else:
        print(f"SDK cache: not set (will download from Autodesk)")
    sdk_path = _sdk_dir(version)

    # ── 1. Already available? ──
    if sdk_path.exists() and (sdk_path / "include" / "maya" / "MFnPlugin.h").exists():
        print(f"✓ Maya {version} SDK already at: {sdk_path}")
        return 0

    platform = "Windows"
    _SDK_CACHE.mkdir(parents=True, exist_ok=True)

    # ── 2. Try GitHub Releases (fast, stripped, cross-platform) ──
    stripped_zip = _SDK_CACHE / f"sdk-maya{version}.zip"
    if _download_github_stripped(version, stripped_zip):
        print(f"  Extracting...")
        with zipfile.ZipFile(stripped_zip, "r") as zf:
            zf.extractall(sdk_path)
        _setup_platform_libs(sdk_path)
        print(f"✓ Maya {version} SDK ready from GitHub cache at: {sdk_path}")
        return 0

    # ── 3. Fall back to full DevKit from Autodesk (slow) ──
    devkit_name = f"Autodesk_Maya_{version}_DevKit_{platform}"
    zip_path = _SDK_CACHE / f"{devkit_name}.zip"

    if not zip_path.exists():
        url = DEVKIT_URLS.get(version, {}).get(platform)
        if not url:
            print(f"✗ No download URL known for Maya {version} / {platform}")
            print("  Download manually and place the ZIP in thirdy-party/")
            return 1

        print(f"\n! Autodesk download (slow, ~527 MB). This is a one-time cost.")
        print(f"  After it finishes, run 'python scripts/ci/strip_maya_sdk.py publish --version {version}' to")
        print(f"  create a stripped artifact for fast future downloads.\n")
        if not _download_file(url, zip_path, f"Maya {version} DevKit"):
            return 1

    # Strip to sdk-maya{version}/
    print(f"Stripping to: {sdk_path}")
    extracted_dir = _SDK_CACHE / devkit_name / "devkitBase"
    if extracted_dir.exists():
        _strip_from_dir(extracted_dir, sdk_path, args)
    else:
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            print("  Extracting DevKit ZIP...")
            with zipfile.ZipFile(zip_path, "r") as zf:
                zf.extractall(tmp)
            base = Path(tmp)
            devkit_base = base / devkit_name / "devkitBase"
            if not devkit_base.exists():
                devkit_base = base / "devkitBase"
            _strip_from_dir(
                devkit_base if devkit_base.exists() else base, sdk_path, args
            )

    print(f"\n✓ Maya {version} SDK ready at: {sdk_path}")
    print(f"  Tip: run 'python scripts/ci/strip_maya_sdk.py publish --version {version}' to create a stripped ZIP")
    print(f"  for fast future downloads.")
    return 0


def cmd_archive(args: argparse.Namespace) -> int:
    """Re-zip a stripped SDK directory for upload/storage."""
    input_dir = Path(args.input)
    output_path = Path(args.output)

    if not input_dir.exists():
        print(f"✗ Input not found: {input_dir}")
        return 1

    _ensure_dir(output_path.parent)

    with zipfile.ZipFile(output_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, _dirs, files in os.walk(input_dir):
            for file in files:
                file_path = Path(root) / file
                arcname = file_path.relative_to(input_dir)
                zf.write(file_path, arcname)

    size_mb = output_path.stat().st_size / 1024 / 1024
    print(f"✓ Archived: {output_path} ({size_mb:.1f} MB)")
    return 0


def cmd_list_urls(args: argparse.Namespace) -> int:
    """Print known DevKit download URLs."""
    print("Known Maya DevKit download URLs:\n")
    for version in sorted(DEVKIT_URLS):
        for plat, url in DEVKIT_URLS[version].items():
            if url:
                print(f"  Maya {version} ({plat}): {url}")
            else:
                print(f"  Maya {version} ({plat}): (URL not yet known)")
    return 0


def cmd_download(args: argparse.Namespace) -> int:
    """Download a Maya DevKit ZIP from the known URLs."""
    version = args.version
    platform = args.platform
    output = Path(args.output)

    url = DEVKIT_URLS.get(version, {}).get(platform)
    if not url:
        print(f"✗ No URL known for Maya {version} / {platform}")
        print("  Use 'list-urls' to see available URLs.")
        return 1

    _download_file(url, output, f"Maya {version} DevKit ({platform})")
    return 0


# ---------------------------------------------------------------------------
# Command: publish — create combined cross-platform SDK ZIP
# ---------------------------------------------------------------------------
def cmd_publish(args: argparse.Namespace) -> int:
    """Create a cross-platform stripped SDK ZIP for GitHub Release upload.

    Combines all three platform SDKs into one package:
        sdk-maya{version}.zip
        ├── include/maya/          ← shared headers
        ├── cmake/                 ← shared CMake modules
        ├── lib/windows/           ← Windows .lib files
        ├── lib/linux/             ← Linux .so files
        └── lib/macos/             ← macOS .dylib/.bundle files

    Usage:
        # Publish from pre-stripped SDKs (recommended)
        python scripts/ci/strip_maya_sdk.py publish --version 2026
            --sdk-windows thirdy-party/sdk-maya2026-windows
            --sdk-linux   thirdy-party/sdk-maya2026-linux
            --sdk-macos   thirdy-party/sdk-maya2026-macos

        # Publish from just one platform (others will be omitted)
        python scripts/ci/strip_maya_sdk.py publish --version 2026
    """
    version = args.version or _detect_maya_version()
    output_dir = Path("build")
    output_dir.mkdir(parents=True, exist_ok=True)
    zip_name = f"sdk-maya{version}.zip"
    zip_path = output_dir / zip_name

    # Collect SDK source directories per platform
    platforms = {
        "windows": args.sdk_windows,
        "linux": args.sdk_linux,
        "macos": args.sdk_macos,
    }
    available = {}
    for plat, path_str in platforms.items():
        if path_str and Path(path_str).exists():
            available[plat] = Path(path_str)
            print(f"  Using provided {plat} SDK: {path_str}")

    # For platforms not provided, try auto-download+strip (concurrent)
    devkit_dir = _SDK_CACHE / "devkit-zips"
    devkit_dir.mkdir(parents=True, exist_ok=True)

    import concurrent.futures

    def _ensure_platform(plat: str) -> tuple[str, Path] | None:
        """Download, extract, and strip a single platform. Returns (name, path) or None."""
        if plat in available:
            return None

        # Check if already stripped
        default_sdk = _SDK_CACHE / f"sdk-maya{version}-{plat}"
        with _print_lock:
            if (
                default_sdk.exists()
                and (default_sdk / "include" / "maya" / "MFnPlugin.h").exists()
            ):
                print(f"  Using cached {plat} SDK: {default_sdk}")
                return plat, default_sdk

        # Download
        plat_cap = (
            "Windows"
            if plat == "windows"
            else ("Linux" if plat == "linux" else "macOS")
        )
        url = DEVKIT_URLS.get(version, {}).get(plat_cap)
        if not url:
            with _print_lock:
                print(f"  ⚠ No URL known for {plat_cap}. Provide --sdk-{plat} path.")
            return None

        ext = ".zip" if plat == "windows" else (".tgz" if plat == "linux" else ".dmg")
        archive_path = devkit_dir / f"maya{version}-{plat}{ext}"

        with _print_lock:
            print(f"  Downloading {plat} DevKit...")
        if not _download_file(url, archive_path, f"Maya {version} {plat_cap} DevKit"):
            return None

        # Extract to temp dir
        import tempfile

        with tempfile.TemporaryDirectory(prefix=f"mmd-sdk-{plat}-") as tmp:
            tmp_path = Path(tmp)
            with _print_lock:
                print(f"  Extracting {plat}...")
            devkit_base = _extract_archive(archive_path, tmp_path)
            if devkit_base is None:
                return None

            # Strip to cache
            with _print_lock:
                print(f"  Stripping {plat} SDK...")
            _strip_from_dir(devkit_base, default_sdk, args)
            return plat, default_sdk

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        for result in executor.map(_ensure_platform, ["windows", "linux", "macos"]):
            if result is not None:
                plat, path = result
                available[plat] = path

    if not available:
        # Try default: current Python's stripped SDK
        default = _sdk_dir(version)
        if default.exists():
            available[_current_platform()] = default
            print(f"  Using default SDK: {default}")
        else:
            print(f"✗ No SDK sources provided and no stripped SDK at {default}")
            print("  Run 'ensure' first or provide --sdk-* paths.")
            return 1

    # Build the combined package
    def _copy_to(sub_path: str, src: Path):
        dst = output_dir / "._staging" / sub_path
        if src.is_dir():
            shutil.copytree(src, dst, dirs_exist_ok=True)
        elif src.is_file():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)

    staging = output_dir / "._staging"
    if staging.exists():
        shutil.rmtree(staging)

    for plat, sdk_path in available.items():
        # Copy include/ (from first platform only — they're the same)
        if plat == list(available.keys())[0]:
            for item in ["include", "cmake"]:
                src = sdk_path / item
                if src.exists():
                    _copy_to(item, src)
                    n = len(list(src.rglob("*"))) if src.is_dir() else 1
                    print(f"  {item}/  ({n} items, shared)")

        # Copy platform libs
        src_lib = sdk_path / "lib"
        if src_lib.exists():
            dst_lib = staging / "lib" / plat
            dst_lib.mkdir(parents=True, exist_ok=True)
            for f in src_lib.iterdir():
                if f.is_file():
                    shutil.copy2(f, dst_lib / f.name)
            count = len(list(dst_lib.iterdir()))
            print(f"  lib/{plat}/  ({count} files)")

    # Create zip
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, _dirs, files in os.walk(staging):
            for file in files:
                file_path = Path(root) / file
                arcname = file_path.relative_to(staging)
                zf.write(file_path, arcname)

    shutil.rmtree(staging)
    size_mb = zip_path.stat().st_size / (1024 * 1024)
    platforms_str = "+".join(available.keys())

    # Also copy to SDK cache so 'ensure' can find it immediately
    cache_path = _SDK_CACHE / zip_name
    _SDK_CACHE.mkdir(parents=True, exist_ok=True)
    shutil.copy2(zip_path, cache_path)

    # ── Validate the package ──
    print(f"\n  Validating {zip_name}...")
    errors = []
    with zipfile.ZipFile(zip_path, "r") as zf:
        names = zf.namelist()
        # Check required paths exist
        has_include = any(n.startswith("include/maya/") for n in names)
        has_lib = any(n.startswith("lib/") for n in names)
        if not has_include:
            errors.append("Missing include/maya/")
        if not has_lib:
            errors.append("Missing lib/")
        for plat in ["windows", "linux", "macos"]:
            plat_libs = [
                n for n in names if n.startswith(f"lib/{plat}/") and not n.endswith("/")
            ]
            if plat in platforms_str:
                if not plat_libs:
                    errors.append(f"Missing lib/{plat}/ content")
                else:
                    print(f"    lib/{plat}/: {len(plat_libs)} files")
        missing_mfn = any(n.endswith("MFnPlugin.h") for n in names)
        if not missing_mfn:
            errors.append("Missing MFnPlugin.h")
        total_entries = len([n for n in names if not n.endswith("/")])
        print(f"    Total: {total_entries} files, {size_mb:.1f} MB")

    if errors:
        print(f"  ⚠ Validation errors: {errors}")
    else:
        print(f"  ✓ Package structure valid")

    print(f"\n✓ Published: {zip_path} ({size_mb:.1f} MB, {platforms_str})")
    print(f"  SDK cache: {cache_path}")
    if _SDK_CACHE_URL:
        print(f"\nUpload to your SDK cache:")
        print(f"  Asset: {zip_name}")
        print(f"  URL:  {_SDK_CACHE_URL}/{zip_name}")
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Maya SDK Manager — download, strip, archive DevKits"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # strip
    p_strip = sub.add_parser(
        "strip", help="Strip a Maya installation or DevKit ZIP to minimum"
    )
    p_strip.add_argument(
        "--source", "-s", required=True, help="Path to Maya install dir or DevKit ZIP"
    )
    p_strip.add_argument(
        "--output", "-o", required=True, help="Output directory for stripped SDK"
    )

    # ensure — auto-download + strip
    p_ensure = sub.add_parser(
        "ensure", help="Auto-download + strip SDK for the current Python version"
    )
    p_ensure.add_argument(
        "--version",
        "-v",
        type=int,
        default=None,
        help="Maya version (default: auto-detect from Python version)",
    )

    # archive
    p_arc = sub.add_parser("archive", help="Re-zip a stripped SDK directory")
    p_arc.add_argument("--input", "-i", required=True)
    p_arc.add_argument("--output", "-o", required=True)

    # list-urls
    sub.add_parser("list-urls", help="Print known DevKit download URLs")

    # download
    p_dl = sub.add_parser("download", help="Download a Maya DevKit ZIP")
    p_dl.add_argument(
        "--version", "-v", type=int, required=True, help="Maya version (e.g. 2026)"
    )
    p_dl.add_argument(
        "--platform",
        "-p",
        required=True,
        choices=["Windows", "Linux", "macOS"],
        help="Target platform",
    )
    p_dl.add_argument("--output", "-o", required=True, help="Output ZIP path")

    # publish — create cross-platform stripped ZIP for GitHub upload
    p_pub = sub.add_parser(
        "publish", help="Create cross-platform stripped SDK ZIP for GitHub Release"
    )
    p_pub.add_argument(
        "--version",
        "-v",
        type=int,
        default=None,
        help="Maya version (default: auto-detect from Python version)",
    )
    p_pub.add_argument(
        "--sdk-windows", type=str, default=None, help="Path to stripped Windows SDK dir"
    )
    p_pub.add_argument(
        "--sdk-linux", type=str, default=None, help="Path to stripped Linux SDK dir"
    )
    p_pub.add_argument(
        "--sdk-macos", type=str, default=None, help="Path to stripped macOS SDK dir"
    )

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    commands = {
        "strip": cmd_strip,
        "ensure": cmd_ensure,
        "archive": cmd_archive,
        "list-urls": cmd_list_urls,
        "download": cmd_download,
        "publish": cmd_publish,
    }
    return commands[args.command](args)


if __name__ == "__main__":
    exit(main())
