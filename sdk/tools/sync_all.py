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


from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def run(cmd: list[str], cwd: Path) -> None:
    print(f"\n==> {' '.join(cmd)}")
    subprocess.run(cmd, cwd=str(cwd), check=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="Run all bsk-sdk sync scripts in order.")
    ap.add_argument(
        "--sdk-tools-dir",
        default=None,
        help="Path to sdk/tools (defaults to this script's directory).",
    )
    ap.add_argument(
        "--python",
        default=sys.executable,
        help="Python executable to use (default: current interpreter).",
    )
    args = ap.parse_args()

    tools_dir = (
        Path(args.sdk_tools_dir).resolve()
        if args.sdk_tools_dir
        else Path(__file__).resolve().parent
    )
    py = args.python

    scripts = [
        "sync_headers.py",
        "sync_runtime.py",
        "sync_sources.py",
        "sync_swig.py",
    ]

    for s in scripts:
        p = tools_dir / s
        if not p.exists():
            raise FileNotFoundError(f"Missing {p}")
        run([py, str(p)], cwd=tools_dir)

    print("\nAll sync steps completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
