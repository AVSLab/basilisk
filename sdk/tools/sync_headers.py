#!/usr/bin/env python3

#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


"""
Synchronize a curated subset of Basilisk source headers into the SDK package.

This script copies selected directories from the main Basilisk `src/` tree into:

    sdk/src/bsk_sdk/include/Basilisk/

This script is intended to be run by Basilisk maintainers when updating the SDK.
"""

from __future__ import annotations

import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"

SDK_ROOT = REPO_ROOT / "sdk"
SDK_INCLUDE_ROOT = SDK_ROOT / "src" / "bsk_sdk" / "include" / "Basilisk"

# Directories to vendor into the SDK relative to src/
DIRECTORIES = [
    "architecture/_GeneralModuleFiles",
    "architecture/messaging",
    "architecture/utilities",
    "architecture/msgPayloadDefC",
    "architecture/msgPayloadDefCpp",
    "fswAlgorithms/fswUtilities",
    "simulation/environment/_GeneralModuleFiles",
    "simulation/dynamics/reactionWheels",
]

# Things that must be excluded from the SDK
IGNORE_PATTERNS = [
    "_UnitTest",
    "_Documentation",
    "__pycache__",
    "*.swg",
    "*.i",
    "*.py",
    "*.cpp",
    "*.c",
]


def copy_tree(src: Path, dest: Path) -> None:
    """Replace dest with a filtered copy of src."""
    if dest.exists():
        shutil.rmtree(dest)

    dest.parent.mkdir(parents=True, exist_ok=True)

    shutil.copytree(
        src,
        dest,
        ignore=shutil.ignore_patterns(*IGNORE_PATTERNS),
    )


def main() -> None:
    if not SRC_ROOT.exists():
        raise RuntimeError(f"Expected Basilisk src directory not found: {SRC_ROOT}")

    SDK_INCLUDE_ROOT.mkdir(parents=True, exist_ok=True)

    for relative in DIRECTORIES:
        src_dir = SRC_ROOT / relative
        dest_dir = SDK_INCLUDE_ROOT / relative

        if not src_dir.exists():
            raise FileNotFoundError(f"Missing source directory: {src_dir}")

        print(f"[bsk-sdk] Copying {src_dir} -> {dest_dir}")
        copy_tree(src_dir, dest_dir)

    print("[bsk-sdk] Header synchronization complete.")


if __name__ == "__main__":
    main()
