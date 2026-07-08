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
#

import argparse
import re
from pathlib import Path


def _normalize_project_name(project_name: str) -> str:
    return re.sub(r"[-_.]+", "-", project_name).lower()


def _wheel_project_name(wheel: Path) -> str:
    parts = wheel.name.removesuffix(".whl").split("-")
    if len(parts) < 5:
        raise ValueError(f"Unexpected wheel filename: {wheel.name}")
    return _normalize_project_name(parts[0])


def _package_index(project_name: str, wheels: list[Path]) -> str:
    links = "\n".join(f'  <a href="{w.name}">{w.name}</a><br/>' for w in wheels)
    return (
        "<!DOCTYPE html>\n"
        f"<html><head><title>Links for {project_name}</title></head><body>\n"
        f"<h1>Links for {project_name}</h1>\n"
        f"{links}\n"
        "</body></html>\n"
    )


def _root_index(project_names: list[str]) -> str:
    links = "\n".join(f'  <a href="{name}/">{name}</a><br/>' for name in project_names)
    return (
        "<!DOCTYPE html>\n"
        "<html><head><title>Simple Index</title></head><body>\n"
        "<h1>Simple Index</h1>\n"
        f"{links}\n"
        "</body></html>\n"
    )


def _group_wheels_by_project(wheels: list[Path]) -> dict[str, list[Path]]:
    grouped_wheels: dict[str, list[Path]] = {}
    for wheel in wheels:
        project_name = _wheel_project_name(wheel)
        grouped_wheels.setdefault(project_name, []).append(wheel)
    return grouped_wheels


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheels-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    wheels = sorted(args.wheels_dir.glob("*.whl"))
    if not wheels:
        raise SystemExit(f"No wheels found in {args.wheels_dir}")

    grouped_wheels = _group_wheels_by_project(wheels)
    project_names = sorted(grouped_wheels)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "index.html").write_text(_root_index(project_names))

    for project_name in project_names:
        pkg_dir = args.output_dir / project_name
        pkg_dir.mkdir(parents=True, exist_ok=True)
        project_wheels = sorted(grouped_wheels[project_name])
        (pkg_dir / "index.html").write_text(_package_index(project_name, project_wheels))
        for wheel in project_wheels:
            (pkg_dir / wheel.name).write_bytes(wheel.read_bytes())

    print(
        f"Index generated with {len(wheels)} wheel(s) across "
        f"{len(project_names)} package(s) in {args.output_dir}"
    )


if __name__ == "__main__":
    main()
