#
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

import subprocess
import sys

import pytest

from Basilisk import hasBuildFeature


_PROTOBUF_CONSUMERS = (
    ("Basilisk.fswAlgorithms.centerRadiusCNN", "CenterRadiusCNN"),
    ("Basilisk.simulation.vizInterface", "VizInterface"),
)
_PROTOBUF_CONSUMERS_ENABLED = (
    hasBuildFeature("opNav") and hasBuildFeature("vizInterface")
)
pytestmark = pytest.mark.skipif(
    not _PROTOBUF_CONSUMERS_ENABLED,
    reason="Requires Basilisk built with --opNav True and --vizInterface True",
)

_WORKER = """
import importlib
import sys

instances = []
for moduleName, className in zip(sys.argv[1::2], sys.argv[2::2]):
    module = importlib.import_module(moduleName)
    instances.append(getattr(module, className)())
"""


@pytest.mark.parametrize(
    "import_order",
    [
        pytest.param(_PROTOBUF_CONSUMERS, id="centerRadiusCNN-first"),
        pytest.param(tuple(reversed(_PROTOBUF_CONSUMERS)), id="vizInterface-first"),
    ],
)
def test_protobuf_consumers_exit_cleanly(import_order):
    """Verify that both protobuf consumers can share and cleanly exit a process.

    Each import order runs in a fresh interpreter because the regression occurs
    during interpreter shutdown, after ordinary in-process assertions pass.

    :param import_order: Module and class pairs in the order to load them.
    """
    arguments = [value for module_info in import_order for value in module_info]
    timeout = 60  # [s]
    result = subprocess.run(
        [sys.executable, "-X", "faulthandler", "-c", _WORKER, *arguments],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, (
        f"Protobuf consumer process exited abnormally with return code "
        f"{result.returncode}:\n{output}"
    )
