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
Synchronize a curated subset of Basilisk *runtime* sources into the SDK package.

This copies selected .cpp files from the main Basilisk `src/` tree into:

    sdk/src/bsk_sdk/runtime_min/

and also auto-generates "flat include" compatibility shims into:

    sdk/src/bsk_sdk/include_compat/

so curated runtime_min translation units that do things like:

    #include "atmosphereBase.h"

will still compile without patching upstream Basilisk sources.

Notes:
- Shims are only generated for *quoted* includes:  #include "Foo.h"
- System includes like <Eigen/Dense> are not shimmed.
- If a flat header name is ambiguous (multiple matches under include/Basilisk),
  this script errors and you must add an explicit override in FLAT_HEADER_OVERRIDES.
"""

from __future__ import annotations

import re
import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"

SDK_ROOT = REPO_ROOT / "sdk"
SDK_RUNTIME_ROOT = SDK_ROOT / "src" / "bsk_sdk" / "runtime_min"
SDK_INCLUDE_ROOT = SDK_ROOT / "src" / "bsk_sdk" / "include" / "Basilisk"
SDK_COMPAT_INCLUDE_ROOT = SDK_ROOT / "src" / "bsk_sdk" / "include_compat"

CPP_FILES: list[str] = ["simulation/environment/_GeneralModuleFiles/atmosphereBase.cpp"]

# Flat-header resolution overrides
FLAT_HEADER_OVERRIDES: dict[str, str] = {
    "atmosphereBase.h": "simulation/environment/_GeneralModuleFiles/atmosphereBase.h",
    "linearAlgebra.h": "architecture/utilities/linearAlgebra.h",
}

# Matches #include "Header.h"
INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"\s*$')


def copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _is_flat_header(h: str) -> bool:
    return "/" not in h and "\\" not in h and h.endswith((".h", ".hpp"))


def _find_header_under_sdk(include_root: Path, header_name: str) -> Path | None:
    """
    Find a header by basename under include_root.
    """
    matches = list(include_root.rglob(header_name))
    if not matches:
        return None
    if len(matches) > 1:
        raise RuntimeError(
            f"Ambiguous header '{header_name}' found in multiple locations:\n"
            + "\n".join(f"  - {m}" for m in matches)
        )
    return matches[0]


def _header_rel_to_basilisk(include_root: Path, header_path: Path) -> str:
    return header_path.relative_to(include_root).as_posix()


def generate_compat_shims_for_runtime() -> None:
    if not SDK_INCLUDE_ROOT.exists():
        raise RuntimeError(
            f"SDK headers root does not exist: {SDK_INCLUDE_ROOT}\n"
            "Did you run sync_headers.py first (or otherwise populate sdk/src/bsk_sdk/include/Basilisk)?"
        )

    SDK_COMPAT_INCLUDE_ROOT.mkdir(parents=True, exist_ok=True)

    runtime_cpp_files = sorted(SDK_RUNTIME_ROOT.glob("*.cpp"))
    needed_flat_headers: set[str] = set()

    for cpp in runtime_cpp_files:
        text = cpp.read_text(encoding="utf-8", errors="replace")
        for line in text.splitlines():
            m = INCLUDE_RE.match(line)
            if not m:
                continue
            hdr = m.group(1)
            if _is_flat_header(hdr):
                needed_flat_headers.add(hdr)

    for hdr in sorted(needed_flat_headers):
        if hdr in FLAT_HEADER_OVERRIDES:
            rel = FLAT_HEADER_OVERRIDES[hdr]
            real = SDK_INCLUDE_ROOT / rel
            if not real.exists():
                raise FileNotFoundError(
                    f"Override for '{hdr}' points to missing SDK header:\n"
                    f"  {real}\n"
                    "Fix sync_headers.py DIRECTORIES or update FLAT_HEADER_OVERRIDES."
                )
        else:
            real = _find_header_under_sdk(SDK_INCLUDE_ROOT, hdr)
            if real is None:
                raise FileNotFoundError(
                    f"runtime_min needs '{hdr}' but it was not found under SDK headers:\n"
                    f"  {SDK_INCLUDE_ROOT}\n"
                    "Fix sync_headers.py DIRECTORIES (or add an explicit override)."
                )
            rel = _header_rel_to_basilisk(SDK_INCLUDE_ROOT, real)

        shim = SDK_COMPAT_INCLUDE_ROOT / hdr
        shim.write_text(f'#pragma once\n#include "{rel}"\n', encoding="utf-8")
        print(f"[bsk-sdk] compat shim: {shim} -> {rel}")


def main() -> None:
    if not SRC_ROOT.exists():
        raise RuntimeError(f"Expected Basilisk src directory not found: {SRC_ROOT}")

    SDK_RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)

    for rel in CPP_FILES:
        src = SRC_ROOT / rel
        if not src.exists():
            raise FileNotFoundError(f"Missing source file: {src}")

        dst = SDK_RUNTIME_ROOT / src.name
        print(f"[bsk-sdk] Copying {src} -> {dst}")
        copy_file(src, dst)

    generate_compat_shims_for_runtime()
    print("[bsk-sdk] Runtime synchronization complete.")


if __name__ == "__main__":
    main()
