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

import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]

SETUP_METADATA_SCRIPT = """
import sys
from pathlib import Path

from setuptools.build_meta import prepare_metadata_for_build_editable


metadataRoot = Path(sys.argv[1])
distInfoName = prepare_metadata_for_build_editable(str(metadataRoot))
metadataPath = metadataRoot / distInfoName / "METADATA"
if not metadataPath.is_file():
    raise RuntimeError(f"Editable metadata was not generated at {metadataPath}")
"""


def test_editable_metadata_generation(tmp_path):
    r"""Verify editable metadata generation with the installed Setuptools release.

    This exercises ``setup.py`` in an isolated process so both the legacy
    ``Extension`` implementation and the Setuptools 84 dataclass-backed
    implementation use the same Basilisk compatibility path.
    """
    result = subprocess.run(
        [sys.executable, "-c", SETUP_METADATA_SCRIPT, str(tmp_path)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, (
        "Editable metadata generation failed:\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )
