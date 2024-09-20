import inspect
import os
import sys

import pytest
import numpy as np
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../../examples')

import scenarioSweepingSpacecraft

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.

@pytest.mark.parametrize("useAltBodyFrame, angle_rate_command, time_command", [
    (False, np.array([[0.0,0.0,0.0],[0.0,0.002,0.0],[0.0,-0.002,0.0],[0.0,0.0,0.0]]), np.array([10,10,10,10])),
    (True, np.array([[0.0,0,0.0],[0.0,0.002,0.0],[0.0,-0.002,0.0],[0.0,0.0,0.0]]), np.array([10,10,10,10]))
])

def test_scenarioSweepingSpacecraft(show_plots, useAltBodyFrame, angle_rate_command, time_command):
    """This function is called by the py.test environment."""

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    try:
        figureList = scenarioSweepingSpacecraft.run(show_plots, useAltBodyFrame, angle_rate_command, time_command)

        # save the figures to the Doxygen scenario images folder
        for pltName, plt in list(figureList.items()):
            unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount += 1
        testMessages.append('scenarioSweepingSpacecraft test failed.')

    # print out success message if no error is found
    if testFailCount == 0:
        print('PASSED')
    else:
        print('Failed: testFailCount is '+ str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages
