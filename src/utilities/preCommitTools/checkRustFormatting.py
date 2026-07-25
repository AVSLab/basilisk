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

"""Check Rust workspace formatting when the Rust tools are available.

Rust remains optional for developers who are not working on Rust modules.
Continuous integration installs the formatter explicitly and therefore always
enforces this check.
"""

import os
import shutil
import subprocess
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
RUST_MANIFEST = Path("src") / "Cargo.toml"


def is_continuous_integration() -> bool:
    """Return ``True`` when the hook is running in continuous integration."""
    return os.environ.get("CI", "").lower() in {"1", "true", "yes"}


def unavailable_tool_result(tool_name: str) -> int:
    """Report an unavailable Rust tool and return the appropriate status."""
    if is_continuous_integration():
        print(f"Error: {tool_name} is required for the CI Rust formatting check.")
        return 1

    print(
        f"Rust formatting skipped: {tool_name} is not installed. "
        "CI will still enforce Rust formatting."
    )
    return 0


def main() -> int:
    """Run ``cargo fmt --check`` or skip it when Rust is unavailable locally."""
    cargo = shutil.which("cargo")
    if cargo is None:
        return unavailable_tool_result("Cargo")

    formatter = subprocess.run(
        [cargo, "fmt", "--version"],
        cwd=REPOSITORY_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    if formatter.returncode != 0:
        return unavailable_tool_result("rustfmt")

    formatting = subprocess.run(
        [
            cargo,
            "fmt",
            "--all",
            "--check",
            "--manifest-path",
            str(RUST_MANIFEST),
        ],
        cwd=REPOSITORY_ROOT,
        check=False,
    )
    return formatting.returncode


if __name__ == "__main__":
    raise SystemExit(main())
