#!/usr/bin/env python3
"""Generate and test optional Basilisk component wheels.

 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

from build_optional_bsk_wheel import COMPONENTS, DEFAULT_VERSION_FILE, build_optional_wheel
from test_optional_bsk_wheels import find_one_wheel, test_optional_wheels


def reset_output_directory(directory: Path, protected_directories: tuple[Path, ...]) -> None:
    resolved_directory = directory.resolve()
    protected = {path.resolve() for path in protected_directories}
    protected.update((Path.cwd().resolve(), Path.home().resolve()))
    if resolved_directory in protected or resolved_directory == resolved_directory.parent:
        raise ValueError(f"Refusing to clean protected output directory: {directory}")

    if directory.exists() and not directory.is_dir():
        raise ValueError(f"Output path is not a directory: {directory}")

    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True, exist_ok=True)


def copy_wheel_to_wheelhouse(wheel: Path, wheelhouse: Path) -> Path:
    wheelhouse.mkdir(parents=True, exist_ok=True)
    destination = wheelhouse / wheel.name
    if wheel.resolve() != destination.resolve():
        shutil.copy2(wheel, destination)
    return destination


def assemble_test_wheelhouse(base_wheel: Path, optional_wheel: Path, wheelhouse: Path) -> None:
    copy_wheel_to_wheelhouse(base_wheel, wheelhouse)
    copy_wheel_to_wheelhouse(optional_wheel, wheelhouse)


def run_optional_wheel_test(
    component_name: str,
    base_wheel_dir: Path,
    component_wheel_dir: Path,
    optional_wheel_dir: Path,
    test_wheelhouse: Path,
    version_file: Path,
) -> None:
    base_wheel = find_one_wheel(base_wheel_dir, "bsk-*.whl")
    component_wheel = find_one_wheel(component_wheel_dir, "bsk-*.whl")
    print(f"Base wheel input: {base_wheel}", flush=True)
    print(f"Component wheel input: {component_wheel}", flush=True)

    protected_directories = (base_wheel_dir, component_wheel_dir)
    reset_output_directory(optional_wheel_dir, protected_directories)
    reset_output_directory(test_wheelhouse, protected_directories)

    optional_wheel = build_optional_wheel(
        component_name,
        base_wheel,
        component_wheel,
        optional_wheel_dir,
        version_file,
    )
    print(f"Generated optional wheel: {optional_wheel}", flush=True)

    assemble_test_wheelhouse(base_wheel, optional_wheel, test_wheelhouse)
    test_optional_wheels(test_wheelhouse)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an optional Basilisk wheel and test local installs.",
    )
    parser.add_argument("--component", choices=sorted(COMPONENTS), required=True)
    parser.add_argument("--base-wheel-dir", type=Path, required=True)
    parser.add_argument("--component-wheel-dir", type=Path, required=True)
    parser.add_argument("--optional-wheel-dir", type=Path, required=True)
    parser.add_argument("--test-wheelhouse", type=Path, required=True)
    parser.add_argument("--version-file", type=Path, default=DEFAULT_VERSION_FILE)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        run_optional_wheel_test(
            args.component,
            args.base_wheel_dir,
            args.component_wheel_dir,
            args.optional_wheel_dir,
            args.test_wheelhouse,
            args.version_file,
        )
    except Exception as err:
        print(f"error: {err}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
