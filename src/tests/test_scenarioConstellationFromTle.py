#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the scenario_constellationFromTle
# Author:   Codex
# Creation Date:  Feb. 22, 2026
#


import inspect
import os
import pathlib
import sys

import pytest
from Basilisk.utilities import tleHandling, unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

ROOT_DIR = pathlib.Path(__file__).parent.parent.parent
DATA_DIR = ROOT_DIR / "examples" / "MultiSatBskSim" / "tleData"

sys.path.append(path + "/../../examples/MultiSatBskSim/scenariosMultiSat")
import scenario_constellationFromTle


@pytest.mark.parametrize("tleFile, numThreads", [("spacestations.2le", 2)])
@pytest.mark.scenarioTest
def test_scenarioConstellationFromTle(show_plots, tleFile, numThreads):
    """This function is called by the py.test environment."""
    testFailCount = 0
    testMessages = []

    try:
        tleData = tleHandling.satTle2elem(DATA_DIR / tleFile)
        figureList = scenario_constellationFromTle.run(show_plots, tleData, numThreads)
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)
    except OSError:
        testFailCount += 1
        testMessages.append("scenario_constellationFromTle test failed.")

    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    assert testFailCount < 1, testMessages


if __name__ == "__main__":
    test_scenarioConstellationFromTle(
        False,              # show_plots
        "spacestations.2le",  # tleFile
        2                   # numThreads
    )
