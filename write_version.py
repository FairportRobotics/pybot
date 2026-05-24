#!/usr/bin/env python3
"""Write the current git commit hash to version.txt.

Run this script before ``robotpy deploy`` so the hash is baked into the
deploy package and readable on the roboRIO (where git is not available).

Usage
-----
::

    python write_version.py          # writes version.txt
    python write_version.py --check  # prints the hash and exits (no write)

The generated ``version.txt`` is intentionally NOT git-ignored so that
``robotpy deploy`` includes it in the file sync to the robot.
"""

import argparse
import subprocess
from pathlib import Path

VERSION_FILE = Path(__file__).parent / "version.txt"


def get_git_info() -> str:
    """Return a version string containing the short hash and dirty flag.

    Format: ``<short-hash>[-dirty]``  e.g. ``3ca6270`` or ``3ca6270-dirty``

    Falls back to ``"unknown"`` if git is not available.
    """
    try:
        short_hash = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()

        # Append "-dirty" when there are uncommitted changes so the driver
        # station display makes it obvious the robot is running modified code.
        dirty = subprocess.call(
            ["git", "diff", "--quiet", "HEAD"],
            stderr=subprocess.DEVNULL,
        )
        return f"{short_hash}-dirty" if dirty else short_hash

    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Print the version string and exit without writing the file.",
    )
    args = parser.parse_args()

    version = get_git_info()

    if args.check:
        print(version)
        return

    VERSION_FILE.write_text(version + "\n", encoding="utf-8")
    print(f"Wrote {VERSION_FILE.name}: {version}")


if __name__ == "__main__":
    main()
