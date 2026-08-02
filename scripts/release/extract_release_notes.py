"""
Extract release notes for a given version from CHANGELOG.md.

Usage:
    python scripts/extract_release_notes.py <version>

The script reads CHANGELOG.md (expected in the current working directory)
and prints the notes for the requested version. The version is matched
against Markdown headers in the format:

    ## [1.0.0] - 2026-01-01
    ## [Unreleased]

For pre-release versions (e.g. 1.0.0-rc1), it looks for the main release
header [1.0.0]. Falls back to [Unreleased] for development versions (0.x).
"""

import re
import sys


def extract_notes(version: str, changelog_path: str = "CHANGELOG.md") -> str:
    """
    Extract release notes for *version* from the changelog at *changelog_path*.

    Returns the notes text, or a fallback message if the version is not found.
    """
    with open(changelog_path, encoding="utf-8") as f:
        content = f.read()

    # Determine which header to look for
    if version.startswith("0.") or version == "unreleased":
        header = "[Unreleased]"
    else:
        # Strip pre-release suffix (e.g. "1.0.0-rc1" -> "1.0.0")
        base_version = version.split("-")[0]
        header = f"[{base_version}]"

    pattern = rf"## {re.escape(header)}(.*?)(?=\n## \[|\Z)"
    match = re.search(pattern, content, re.DOTALL)
    if match:
        return match.group(1).strip()
    return f"See CHANGELOG.md for details on version {version}."


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: python extract_release_notes.py <version>", file=sys.stderr)
        return 1

    version = sys.argv[1]
    notes = extract_notes(version)
    print(notes)
    return 0


if __name__ == "__main__":
    sys.exit(main())
