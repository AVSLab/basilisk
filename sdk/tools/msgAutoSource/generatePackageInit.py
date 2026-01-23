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
import sys
from pathlib import Path


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(
            "Usage:\n"
            "  generatePackageInit.py <moduleOutputPath> <headerDir1> [headerDir2 ...]\n",
            file=sys.stderr,
        )
        return 2

    module_output_dir = Path(argv[1]).resolve()
    module_output_dir.mkdir(parents=True, exist_ok=True)

    # WORKING_DIRECTORY is set to msgAutoSource, and the header dirs are passed as relative paths
    cwd = Path.cwd()

    header_dirs: list[Path] = []
    for a in argv[2:]:
        p = Path(a)
        if not p.is_absolute():
            p = (cwd / p).resolve()
        header_dirs.append(p)

    init_py = module_output_dir / "__init__.py"

    lines: list[str] = []
    lines.append("# Auto-generated. Do not edit.\n")

    for header_dir in header_dirs:
        if not header_dir.exists():
            raise FileNotFoundError(f"Header input path not found: {header_dir}")

        for file_path in sorted(header_dir.iterdir()):
            if file_path.suffix.lower() not in (".h", ".hpp"):
                continue

            class_name = file_path.stem
            lines.append(
                f"from Basilisk.architecture.messaging.{class_name} import *\n"
            )

    # Deduplicate imports
    seen = set()
    deduped: list[str] = []
    for ln in lines:
        if ln.startswith("from "):
            if ln in seen:
                continue
            seen.add(ln)
        deduped.append(ln)

    init_py.write_text("".join(deduped), encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
