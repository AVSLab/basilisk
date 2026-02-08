#!/usr/bin/env python3

#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from pathlib import Path
import subprocess
import sys
import tempfile
import filecmp


ROOT = Path(__file__).resolve().parents[3]
make_script = ROOT.joinpath("src", "utilities", "supportDataTools", "makeRegistry.py")
registry_path = ROOT.joinpath(
    "src", "utilities", "supportDataTools", "registrySnippet.py"
)


def main():
    # Regenerate registry into a temporary file
    with tempfile.NamedTemporaryFile(delete=False) as tmp:
        tmp_path = Path(tmp.name)

    result = subprocess.run(
        [sys.executable, str(make_script)],
        stdout=tmp_path.open("w"),
        stderr=subprocess.PIPE,
        text=True,
    )

    if result.returncode != 0:
        print("Error running makeRegistry.py")
        print(result.stderr)
        return 1

    # Compare with committed registry
    if not filecmp.cmp(tmp_path, registry_path, shallow=False):
        print("❌ supportData/ changed, but registrySnippet.py is outdated.")
        print(
            "   Run: python src/utilities/supportDataTools/makeRegistry.py > src/utilities/supportDataTools/registrySnippet.py"
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
