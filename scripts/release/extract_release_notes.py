"""
Extract release notes for a given version from CHANGELOG.md.

Usage:
    python scripts/extract_release_notes.py <version>

The script reads CHANGELOG.md (expected in the current working directory)
and prints the notes for the requested version. The version is matched
against Markdown headers in the format:

    ## [1.0.0] - 2026-01-01
    ## [Unreleased]

The exact version header (e.g. [1.0.0], [0.1.0]) is used when present;
pre-release versions (e.g. 1.0.0-rc1) match the base release header.
Falls back to [Unreleased] for versions without a dedicated section.
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

    # Strip pre-release suffix (e.g. "1.0.0-rc1" -> "1.0.0")
    base_version = version.split("-")[0]

    # Try the exact version header first — this also covers 0.x releases
    # that have their own changelog section (e.g. 0.1.0).
    pattern = rf"## \[{re.escape(base_version)}\].*?(?=\n## \[|\Z)"
    match = re.search(pattern, content, re.DOTALL)
    if match:
        return match.group(0).strip()

    # Fall back to [Unreleased] for development versions without a section.
    match = re.search(r"## \[Unreleased\](.*?)(?=\n## \[|\Z)", content, re.DOTALL)
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
