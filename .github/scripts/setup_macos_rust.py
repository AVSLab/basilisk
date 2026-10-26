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

"""Install and verify the selected macOS Rust toolchain's LLVM component."""

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


def run(command, environment=None):
    """Capture diagnostics, including failures reported with a zero exit status."""
    return subprocess.run(
        command, env=environment, capture_output=True, text=True,
        check=True, timeout=600,  # [s] Allow time for component downloads.
    )


def verify_stripping(compiler):
    """Exercise direct toolchain invocations without rustup's library-path help.

    Corrosion invokes the selected compiler by absolute path. Testing through
    the rustup proxy alone can hide a missing LLVM library in that toolchain.
    """
    sysroot = Path(run([str(compiler), "--print", "sysroot"]).stdout.strip())
    version = run([str(compiler), "--version", "--verbose"]).stdout
    host = re.search(r"^host: (\S+)$", version, re.MULTILINE)
    if host is None or not host.group(1).endswith("-apple-darwin"):
        raise RuntimeError(f"Expected a macOS Rust host in compiler metadata:\n{version}")

    environment = os.environ.copy()
    # Require the toolchain to work even when a parent process has not supplied
    # Rust's top-level lib directory. Do not alter the caller's environment.
    environment.pop("DYLD_LIBRARY_PATH", None)
    environment.pop("DYLD_FALLBACK_LIBRARY_PATH", None)
    objcopy = sysroot / "lib" / "rustlib" / host.group(1) / "bin" / "rust-objcopy"
    run([str(objcopy), "--version"], environment)

    with tempfile.TemporaryDirectory(prefix="bsk-rust-strip-") as directory:
        source = Path(directory) / "probe.rs"
        executable = Path(directory) / "probe"
        source.write_text("fn main() {}\n", encoding="utf-8")
        result = run([
            str(compiler), str(source), "--crate-name", "bsk_strip_probe",
            "--edition=2021", "--color=never", "-Copt-level=3",
            "-Cstrip=debuginfo", "-o", str(executable),
        ], environment)
        diagnostics = result.stdout + result.stderr
        # rustc can emit the stripping failure as a warning and still exit 0.
        if re.search(r"\bwarning(?:\[[^\]]+\])?:", diagnostics):
            raise RuntimeError(f"Rust debug-stripping probe emitted warnings:\n{diagnostics}")
        run([str(executable)], environment)


def setup_macos_rust(install=True):
    """Prepare the active toolchain without changing the user's default selection."""
    if sys.platform != "darwin":
        return
    # Respect RUSTUP_TOOLCHAIN, directory overrides, and the active default.
    # In wheel jobs RUSTUP_TOOLCHAIN contains the workflow's exact Rust version.
    active = run(["rustup", "show", "active-toolchain"]).stdout.split()
    if not active:
        raise RuntimeError("rustup did not report an active toolchain")
    toolchain = active[0]
    compiler = Path(run(["rustup", "which", "--toolchain", toolchain, "rustc"]).stdout.strip())
    if install:
        result = run([
            "rustup", "component", "add", "llvm-tools-preview", "--toolchain", toolchain,
        ])
        print(result.stdout + result.stderr, end="", flush=True)
    verify_stripping(compiler)
    print(f"Verified Rust debug stripping with {toolchain}", flush=True)


def main(argv=None):
    """Install the component, or only check an already prepared toolchain."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check-only", action="store_true", help="verify without installing components",
    )
    arguments = parser.parse_args(argv)
    try:
        setup_macos_rust(install=not arguments.check_only)
    except subprocess.CalledProcessError as error:
        print(
            f"Rust toolchain check failed: {error}\n{error.stdout or ''}{error.stderr or ''}",
            file=sys.stderr,
        )
        return 1
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"Rust toolchain check failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
