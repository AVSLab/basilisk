#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the scenario_AttGuidMultiSat
# Author:   João Vaz Carneiro
# Creation Date:  Jul. 2, 2021
#


import inspect
import os
import sys

import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/MultiSatBskSim/scenariosMultiSat')
import scenario_AttGuidMultiSat


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("numberSpacecraft", [3])
@pytest.mark.scenarioTest
# provide a unique test method name, starting with test_
def test_scenarioAttGuidMultiSat(show_plots, numberSpacecraft):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        figureList = scenario_AttGuidMultiSat.run(show_plots, numberSpacecraft)
        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append("scenario_AttGuidMultiSat  test are failed.")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages


if __name__ == "__main__":
    test_scenarioAttGuidMultiSat(
        False,  # show_plots
        3  # numberSpacecraft
    )
