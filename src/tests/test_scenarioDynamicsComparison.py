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

# Runs every dynamics-engine comparison scenario in examples/dynamicsComparison,
# confirming each runs without error and saving its result figures.

import os
import sys
import inspect
import importlib

import pytest
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

THIS_FOLDER = os.path.dirname(__file__)
SCENARIO_FOLDER = os.path.join(
    THIS_FOLDER, "..", "..", "examples", "dynamicsComparison"
)
SCENARIO_FILES = [
    scenarioFile[:-3]
    for scenarioFile in os.listdir(SCENARIO_FOLDER)
    if scenarioFile.startswith("scenario") and scenarioFile.endswith(".py")
]

# Per-scenario test overrides: reduced sweeps (and, for the Pareto studies, a cheaper
# reference and shorter horizon) to keep each test under the scenario runtime budget.
SCENARIO_RUN_KWARGS = {
    "scenarioCompareFlexPanels": {"nSegmentsList": (1, 2, 4, 8)},
    "scenarioCompareParetoRwPanels": {
        "sweepConfigs": [
            ("svIntegratorRK4", 0.1, None),
            ("svIntegratorEuler", 0.02, None),
            ("svIntegratorRKF45", 0.5, 1.0e-7),
        ],
        "reference": {"integrator": "svIntegratorRKF45", "dt": 0.05, "tol": 1.0e-9},
        "referenceCheck": {"integrator": "svIntegratorRKF45", "dt": 0.025, "tol": 1.0e-9},
    },
    "scenarioCompareParetoFlexPanels": {
        "sweepConfigs": [
            ("svIntegratorRKF45", 0.1, 1.0e-6),
        ],
        "simDuration": 1.0,
        "reference": {"integrator": "svIntegratorRKF45", "dt": 0.2, "tol": 1.0e-6},
    },
}

# Accuracy scenarios expose a ``saveTiming`` flag: when set, ``run`` measures the
# BSM-vs-MJScene wall-clock cost and writes the runtime table its docstring embeds via a
# ``.. csv-table:: :file:`` directive. The reported speedup ratio is machine-independent;
# absolute times are best-of-N minima.
TIMING_SCENARIOS = {
    "scenarioCompareOrbit",
    "scenarioCompareTorque",
    "scenarioCompareRwPanels",
    "scenarioCompareOrbitMultibody",
}

sys.path.append(SCENARIO_FOLDER)


@pytest.mark.parametrize("scenario", SCENARIO_FILES)
@pytest.mark.scenarioTest
def test_scenarios(scenario: str):
    """Run a dynamics-comparison scenario and save its figures (and runtime table).

    Args:
        scenario (str): module name of the scenario file to import and run.
    """
    module = importlib.import_module(scenario)
    kwargs = dict(SCENARIO_RUN_KWARGS.get(scenario, {}))
    if scenario in TIMING_SCENARIOS:
        kwargs["saveTiming"] = True
    figureList = module.run(showPlots=False, saveJson=False, **kwargs)
    if figureList is None:
        return

    for pltName, plt in figureList.items():
        print(f"Saving dynamics-comparison figure: {pltName} (from '{scenario}')")
        unitTestSupport.saveScenarioFigure(pltName, plt, path)


if __name__ == "__main__":
    pytest.main([__file__])
