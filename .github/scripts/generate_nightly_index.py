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
from pathlib import Path


def _package_index(wheels: list[Path]) -> str:
    links = "\n".join(f'  <a href="{w.name}">{w.name}</a><br/>' for w in wheels)
    return (
        "<!DOCTYPE html>\n"
        "<html><head><title>Links for bsk</title></head><body>\n"
        "<h1>Links for bsk</h1>\n"
        f"{links}\n"
        "</body></html>\n"
    )


def _root_index() -> str:
    return (
        "<!DOCTYPE html>\n"
        "<html><head><title>Simple Index</title></head><body>\n"
        "<h1>Simple Index</h1>\n"
        '  <a href="bsk/">bsk</a><br/>\n'
        "</body></html>\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheels-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    wheels = sorted(args.wheels_dir.glob("*.whl"))
    if not wheels:
        raise SystemExit(f"No wheels found in {args.wheels_dir}")

    pkg_dir = args.output_dir / "bsk"
    pkg_dir.mkdir(parents=True, exist_ok=True)

    (pkg_dir / "index.html").write_text(_package_index(wheels))
    (args.output_dir / "index.html").write_text(_root_index())

    for whl in wheels:
        (pkg_dir / whl.name).write_bytes(whl.read_bytes())

    print(f"Index generated with {len(wheels)} wheel(s) in {args.output_dir}")


if __name__ == "__main__":
    main()
