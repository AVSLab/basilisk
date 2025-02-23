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

# This test file is used to test all the mujoco scenarios in the examples/mujoco folder
# The test will run each scenario and check if it runs without any errors

import sys
import os
import importlib

import pytest

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except:
    couldImportMujoco = False

SCENARIO_FOLDER = os.path.join(
    os.path.dirname(__file__), "..", "..", "examples", "mujoco"
)
SCENARIO_FILES = [
    filename[:-3]
    for filename in os.listdir(SCENARIO_FOLDER)
    if filename.startswith("scenario") and filename.endswith(".py")
]

sys.path.append(SCENARIO_FOLDER)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("scenario", SCENARIO_FILES)
@pytest.mark.scenarioTest
def test_scenarios(scenario: str):
    module = importlib.import_module(scenario)
    module.run()  # Every mujoco scenario should have a run function


if __name__ == "__main__":
    pytest.main([__file__])
