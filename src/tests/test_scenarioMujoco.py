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
import inspect
import importlib

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

import pytest
from Basilisk.utilities import simHelpers

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except:
    couldImportMujoco = False

THIS_FOLDER = os.path.dirname(__file__)
SCENARIO_FOLDER = os.path.join(
    THIS_FOLDER, "..", "..", "examples", "mujoco"
)
SCENARIO_FILES = [
    filename[:-3]
    for filename in os.listdir(SCENARIO_FOLDER)
    if filename.startswith("scenario") and filename.endswith(".py")
]

SCENARIO_RUN_KWARGS = {
    # Run both planet-state sources used by the Earth-Moon gravity tutorial.
    "scenarioMJEarthMoonGravity": [
        {"useSpice": True},
        {"useSpice": False},
    ],
    "scenarioThrArmControl": {"showPlots": False, "timeStep": 0.08, "runTime": 240.0},
    # Run the stochastic-drag scenario in both the default (Ornstein-Uhlenbeck) and the
    # IGBM density-noise modes, so both sets of documentation figures are generated.
    "scenarioStochasticDrag": [{}, {"useIgbm": True}],
}
OPTIONAL_EXAMPLE_DEPENDENCIES = {"scipy"}

sys.path.append(SCENARIO_FOLDER)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("scenario", SCENARIO_FILES)
@pytest.mark.scenarioTest
def test_scenarios(scenario: str):
    try:
        module = importlib.import_module(scenario)
    except ModuleNotFoundError as exc:
        if exc.name in OPTIONAL_EXAMPLE_DEPENDENCIES:
            pytest.skip(
                f"Scenario '{scenario}' requires optional example dependency "
                f"'{exc.name}'. Install Basilisk with `pip install -e .[examples]`."
            )
        raise
    # An entry may be a single kwargs dict or a list of dicts (the scenario is run once
    # per variant, e.g. to generate documentation figures for each mode).
    kwargsVariants = SCENARIO_RUN_KWARGS.get(scenario, {})
    if not isinstance(kwargsVariants, list):
        kwargsVariants = [kwargsVariants]

    for kwargs in kwargsVariants:
        figureList = module.run(**kwargs)  # Every mujoco scenario should have a run function
        if figureList is None:
            continue

        for pltName, plt in figureList.items():
            print(f"Saving MuJoCo scenario figure: {pltName} (from '{scenario}')")
            simHelpers.saveScenarioFigure(pltName, plt, path)

if __name__ == "__main__":
    pytest.main([__file__])
