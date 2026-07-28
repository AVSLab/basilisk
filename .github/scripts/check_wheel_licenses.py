#!/usr/bin/env python3
#
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Verify that built Basilisk wheels contain their declared legal files."""

from __future__ import annotations

import argparse
import sys
import zipfile
from email.parser import BytesParser
from pathlib import Path


EXPECTED_LICENSE_FILES = {
    "LICENSE",
    "LICENSES/RUST-THIRD-PARTY.txt",
}


def wheel_paths(paths: list[Path]) -> list[Path]:
    """Expand input wheel files and directories into a unique sorted list."""
    wheels = set()
    for path in paths:
        if path.is_dir():
            wheels.update(path.glob("*.whl"))
        elif path.suffix == ".whl":
            wheels.add(path)
    return sorted(wheels)


def check_wheel(path: Path) -> list[str]:
    """Return packaging errors found in one wheel."""
    errors = []
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        metadata_names = [name for name in names if name.endswith(".dist-info/METADATA")]
        if len(metadata_names) != 1:
            return [f"expected one .dist-info/METADATA file, found {len(metadata_names)}"]

        metadata_name = metadata_names[0]
        distribution_root = metadata_name.removesuffix("METADATA")
        metadata = BytesParser().parsebytes(archive.read(metadata_name))
        declared = set(metadata.get_all("License-File", []))

        for relative_path in sorted(EXPECTED_LICENSE_FILES):
            if relative_path not in declared:
                errors.append(f"METADATA does not declare License-File: {relative_path}")

            archive_path = f"{distribution_root}licenses/{relative_path}"
            if archive_path not in names:
                errors.append(f"wheel does not contain {archive_path}")
            elif not archive.read(archive_path).strip():
                errors.append(f"wheel contains an empty {archive_path}")
    return errors


def main() -> int:
    """Check all wheel files supplied on the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path, help="wheel files or directories")
    arguments = parser.parse_args()

    wheels = wheel_paths(arguments.paths)
    if not wheels:
        print("No wheel files were found to inspect.", file=sys.stderr)
        return 1

    failed = False
    for wheel in wheels:
        errors = check_wheel(wheel)
        if errors:
            failed = True
            for error in errors:
                print(f"{wheel}: {error}", file=sys.stderr)
        else:
            print(f"Wheel license files verified: {wheel}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
