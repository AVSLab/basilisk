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

import subprocess
import sys

import pytest


@pytest.mark.parametrize(
    ("module_name", "class_name"),
    (("dvAttEffect", "dvAttEffect"), ("sunSafeACS", "sunSafeACS")),
)
def test_error_conversion_standalone_import(module_name: str, class_name: str) -> None:
    """Verify each error-conversion wrapper loads in a fresh Python process.

    **Validation Test Description**

    This test imports and constructs each wrapper without first loading the
    other wrapper. This ensures shared error-conversion functions resolve from
    their owning package library instead of relying on process-wide symbols
    supplied by a prior module import.

    **Description of Variables Being Tested**

    The subprocess return code verifies that the wrapper and all of its native
    symbols load successfully.

    :param module_name: Python wrapper module to import.
    :param class_name: Wrapped module class to construct.

    """
    script = (
        f"from Basilisk.fswAlgorithms import {module_name}\n"
        f"{module_name}.{class_name}()\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr
