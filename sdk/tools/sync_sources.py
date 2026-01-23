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
Synchronize a small curated set of Basilisk source files into the SDK package.

This complements sync_headers.py:

- sync_headers.py vendors public headers into:
    sdk/src/bsk_sdk/include/Basilisk/...

- sync_sources.py vendors the minimal compiled implementation sources ("arch_min")
  into:
    sdk/src/bsk_sdk/arch_min/...

These files are compiled by the bsk-sdk CMake project to produce bsk::arch_min,
so plugin authors do not have to compile them in every plugin.
"""

from __future__ import annotations

import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"

SDK_ROOT = REPO_ROOT / "sdk"
SDK_ARCH_MIN_ROOT = SDK_ROOT / "src" / "bsk_sdk" / "arch_min"

ARCH_MIN_FILES: list[tuple[str, str]] = [
    ("architecture/_GeneralModuleFiles/sys_model.cpp", "sys_model.cpp"),
    ("architecture/utilities/bskLogging.cpp", "bskLogging.cpp"),
    (
        "architecture/utilities/moduleIdGenerator/moduleIdGenerator.cpp",
        "moduleIdGenerator.cpp",
    ),
    ("architecture/utilities/linearAlgebra.c", "linearAlgebra.c"),
]


def copy_file(src: Path, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)


def main() -> None:
    if not SRC_ROOT.exists():
        raise RuntimeError(f"Expected Basilisk src directory not found: {SRC_ROOT}")

    SDK_ARCH_MIN_ROOT.mkdir(parents=True, exist_ok=True)

    copied: list[Path] = []
    for rel_src, out_name in ARCH_MIN_FILES:
        src_file = SRC_ROOT / rel_src
        if not src_file.exists():
            raise FileNotFoundError(f"Missing source file: {src_file}")

        dest_file = SDK_ARCH_MIN_ROOT / out_name
        print(f"[bsk-sdk] Copying {src_file} -> {dest_file}")
        copy_file(src_file, dest_file)
        copied.append(dest_file)

    print(f"[bsk-sdk] Source synchronization complete ({len(copied)} files).")


if __name__ == "__main__":
    main()
