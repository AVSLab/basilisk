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

"""Verify that Rust modules reject a panic-aborting build profile."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import tempfile


EXPECTED_DIAGNOSTIC = 'Basilisk Rust modules require panic="unwind"'


def check_unwind_policy(repository_root: Path) -> None:
    """Require ``rustModuleTemplate`` to reject ``panic="abort"``."""
    manifest = repository_root / "src" / "Cargo.toml"
    with tempfile.TemporaryDirectory(prefix="bsk-panic-abort-") as target_directory:
        environment = os.environ.copy()
        environment["CARGO_TARGET_DIR"] = target_directory
        environment["CARGO_TERM_COLOR"] = "never"
        result = subprocess.run(
            [
                "cargo",
                "rustc",
                "--quiet",
                "-p",
                "rustModuleTemplate",
                "--locked",
                "--manifest-path",
                str(manifest),
                "--",
                "-C",
                "panic=abort",
            ],
            cwd=repository_root,
            env=environment,
            capture_output=True,
            text=True,
            check=False,
            timeout=300,  # [s]
        )

    output = result.stdout + result.stderr
    if result.returncode == 0:
        raise RuntimeError(
            "rustModuleTemplate accepted panic=abort; caught panics could cross "
            "the FFI boundary"
        )
    if EXPECTED_DIAGNOSTIC not in output:
        raise RuntimeError(
            "panic=abort failed without the bsk-build policy diagnostic:\n" + output
        )

    print("Verified that Rust module builds reject panic=abort.")


def main() -> int:
    """Run the Rust unwind-policy check from the repository checkout."""
    repository_root = Path(__file__).resolve().parents[2]
    try:
        check_unwind_policy(repository_root)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
