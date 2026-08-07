#!/usr/bin/env python3
"""Exercise representative Basilisk incremental-build dependencies.

The test uses an existing Rust-enabled build and temporarily advances source
file modification times. Original timestamps are restored even when a build
or assertion fails.
"""

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

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MTIME_MARGIN_SECONDS = 1.1  # [s]
COMPILED_ARTIFACT_SUFFIXES = {".o", ".obj", ".rlib", ".rmeta"}
COMPILE_OUTPUT_MARKERS = (
    "Building C object",
    "Building CXX object",
    "Swig compile",
    "Compiling ",
)


@dataclass(frozen=True)
class WrapperTarget:
    """Describe one representative SWIG wrapper target."""

    name: str
    wrapper: Path


class IncrementalBuildError(RuntimeError):
    """Report an incremental-build behavior regression."""


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--source-dir",
        type=Path,
        default=REPOSITORY_ROOT / "src",
        help="Basilisk CMake source directory",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=REPOSITORY_ROOT / "dist3",
        help="Existing Rust-enabled CMake build directory",
    )
    parser.add_argument(
        "--config",
        help="Optional CMake multi-configuration build configuration",
    )
    parser.add_argument(
        "--parallel",
        type=int,
        default=min(os.cpu_count() or 1, 12),
        help="Maximum number of parallel build jobs",
    )
    return parser.parse_args()


def wrapper_targets(build_dir: Path) -> tuple[WrapperTarget, ...]:
    """Return standard, generated-message, and Rust wrapper representatives."""

    return (
        WrapperTarget(
            "mrpFeedback",
            build_dir / "Basilisk/fswAlgorithms/mrpFeedbackPYTHON_wrap.cxx",
        ),
        WrapperTarget(
            "AttRefMsgPayload",
            build_dir / "Basilisk/architecture/messaging/AttRefMsgPayloadPYTHON_wrap.cxx",
        ),
        WrapperTarget(
            "_bsk_python_mrpPDRust",
            build_dir / "Basilisk/fswAlgorithms/mrpPDRust_rust_wrapPYTHON_wrap.cxx",
        ),
    )


def run_build(
    build_dir: Path,
    targets: tuple[WrapperTarget, ...],
    parallel: int,
    configuration: Optional[str],
) -> str:
    """Build the representative targets and return their combined output."""

    cmake = shutil.which("cmake")
    if cmake is None:
        environment_cmake = Path(sys.executable).with_name("cmake")
        if environment_cmake.is_file():
            cmake = str(environment_cmake)
        else:
            raise IncrementalBuildError("Could not locate the CMake executable.")

    command = [
        cmake,
        "--build",
        str(build_dir),
        "--parallel",
        str(parallel),
        "--target",
        *(target.name for target in targets),
    ]
    if configuration:
        command.extend(("--config", configuration))

    print("+", " ".join(command), flush=True)
    result = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    print(result.stdout, end="")
    if result.returncode:
        raise IncrementalBuildError(
            f"CMake build failed with exit status {result.returncode}."
        )
    return result.stdout


def require_files(paths: tuple[Path, ...]) -> None:
    """Fail with a useful message when required inputs or outputs are absent."""

    missing = [path for path in paths if not path.is_file()]
    if missing:
        formatted = "\n".join(f"  - {path}" for path in missing)
        raise IncrementalBuildError(f"Required files are missing:\n{formatted}")


def modification_times(targets: tuple[WrapperTarget, ...]) -> dict[str, int]:
    """Return generated-wrapper modification times in nanoseconds."""

    return {target.name: target.wrapper.stat().st_mtime_ns for target in targets}


def wait_for_newer_clock(paths: tuple[Path, ...]) -> None:
    """Wait until a new timestamp is distinguishable from all given paths."""

    newest_seconds = max(path.stat().st_mtime_ns for path in paths) / 1_000_000_000  # [s]
    delay_seconds = max(0.0, newest_seconds - time.time()) + MTIME_MARGIN_SECONDS  # [s]
    time.sleep(delay_seconds)


@contextmanager
def temporarily_touch(path: Path, compared_paths: tuple[Path, ...]) -> Iterator[None]:
    """Advance one file's timestamp and restore its original timestamps afterward."""

    original_stat = path.stat()
    wait_for_newer_clock(compared_paths)
    path.touch()
    try:
        yield
    finally:
        os.utime(
            path,
            ns=(original_stat.st_atime_ns, original_stat.st_mtime_ns),
        )


def assert_wrappers_regenerated(
    before: dict[str, int],
    after: dict[str, int],
) -> None:
    """Require every representative wrapper to have a newer timestamp."""

    stale = [name for name in before if after[name] <= before[name]]
    if stale:
        raise IncrementalBuildError(
            "A real SWIG include did not regenerate these wrappers: "
            + ", ".join(stale)
        )


def assert_wrappers_unchanged(
    before: dict[str, int],
    after: dict[str, int],
) -> None:
    """Require every representative wrapper timestamp to remain unchanged."""

    changed = [name for name in before if after[name] != before[name]]
    if changed:
        raise IncrementalBuildError(
            "An unrelated C++ source regenerated these wrappers: "
            + ", ".join(changed)
        )


def compiled_artifacts(build_dir: Path) -> dict[Path, tuple[int, int]]:
    """Return timestamps and sizes for compiled objects in the build tree."""

    artifacts: dict[Path, tuple[int, int]] = {}
    for path in build_dir.rglob("*"):
        if path.is_file() and path.suffix.lower() in COMPILED_ARTIFACT_SUFFIXES:
            stat = path.stat()
            artifacts[path.relative_to(build_dir)] = (stat.st_mtime_ns, stat.st_size)
    return artifacts


def assert_no_compilation(
    before: dict[Path, tuple[int, int]],
    after: dict[Path, tuple[int, int]],
    output: str,
) -> None:
    """Require an unchanged build to leave compiled artifacts untouched."""

    changed = sorted(path for path in before.keys() | after.keys() if before.get(path) != after.get(path))
    compile_lines = [
        line
        for line in output.splitlines()
        if any(marker in line for marker in COMPILE_OUTPUT_MARKERS)
    ]
    if changed or compile_lines:
        details = []
        if changed:
            details.append(
                "Compiled artifacts changed:\n"
                + "\n".join(f"  - {path}" for path in changed)
            )
        if compile_lines:
            details.append(
                "Compilation commands were reported:\n"
                + "\n".join(f"  {line}" for line in compile_lines)
            )
        raise IncrementalBuildError(
            "The second unchanged build performed compilation.\n" + "\n".join(details)
        )


def run_regression_test(args: argparse.Namespace) -> None:
    """Run positive, negative, and no-change incremental-build checks."""

    source_dir = args.source_dir.resolve()
    build_dir = args.build_dir.resolve()
    targets = wrapper_targets(build_dir)
    wrappers = tuple(target.wrapper for target in targets)
    shared_swig_include = source_dir / "architecture/utilities/bskException.swg"
    unrelated_cpp = source_dir / "architecture/_GeneralModuleFiles/sys_model.cpp"

    require_files((shared_swig_include, unrelated_cpp))
    run_build(build_dir, targets, args.parallel, args.config)
    require_files(wrappers)

    before_include = modification_times(targets)
    with temporarily_touch(shared_swig_include, wrappers):
        run_build(build_dir, targets, args.parallel, args.config)
    after_include = modification_times(targets)
    assert_wrappers_regenerated(before_include, after_include)
    print("PASS: a real %include regenerated all three wrapper types.")

    before_cpp = modification_times(targets)
    with temporarily_touch(unrelated_cpp, wrappers):
        run_build(build_dir, targets, args.parallel, args.config)
    after_cpp = modification_times(targets)
    assert_wrappers_unchanged(before_cpp, after_cpp)
    print("PASS: an unrelated .cpp did not regenerate any wrapper.")

    before_unchanged_wrappers = modification_times(targets)
    before_unchanged = compiled_artifacts(build_dir)
    unchanged_output = run_build(build_dir, targets, args.parallel, args.config)
    after_unchanged = compiled_artifacts(build_dir)
    after_unchanged_wrappers = modification_times(targets)
    if after_unchanged_wrappers != before_unchanged_wrappers:
        raise IncrementalBuildError(
            "The second unchanged build regenerated one or more wrappers."
        )
    assert_no_compilation(before_unchanged, after_unchanged, unchanged_output)
    print("PASS: a second unchanged build compiled nothing.")


def main() -> int:
    """Run the command-line integration test."""

    try:
        run_regression_test(parse_arguments())
    except (IncrementalBuildError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
