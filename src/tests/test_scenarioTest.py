#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
# Integrated tests
#
# Purpose:  This script calls a series of basic scenario tutorials to ensure
# that they complete properly.  No simulation values are returned and tested.
# Author:   Hanspeter Schaub
# Creation Date:  May 20, 2021
#


import importlib
import inspect
import os
import sys

import pytest
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/')

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("scenarioCase", [
                                        'scenarioGroundLocationImaging'
                                        , 'scenarioGroundDownlink'
                                        , 'scenarioMtbMomentumManagement'
                                        , 'scenarioMtbMomentumManagementSimple'
                                        , 'scenarioSmallBodyNav'
                                        , 'scenarioSmallBodyNavUKF'
                                        , 'scenarioSmallBodyFeedbackControl'
                                        , 'scenarioJupiterArrival'
                                        , 'scenarioSpiceSpacecraft'
                                        , 'scenarioInertialSpiral'
                                        , 'scenarioDeployingPanel'
                                        , 'scenarioAsteroidArrival'
                                        , 'scenarioTwoChargedSC'
                                        , 'scenarioRendezVous'
                                        , 'scenarioSensorThermal'
                                        , 'scenarioHaloOrbit'
                                        , 'scenarioDeployingSolarArrays'
                                        ])
@pytest.mark.scenarioTest
def test_scenarioBskScenarios(show_plots, scenarioCase):

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    # import the bskSim script to be tested
    scene_plt = importlib.import_module(scenarioCase)

    try:
        figureList = scene_plt.run(False)

        # save the figures to the Doxygen scenario images folder

        if figureList != {}:
            for pltName, plt in list(figureList.items()):
                unitTestSupport.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount = testFailCount + 1
        testMessages.append("OS error: {0}".format(err))

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("Failed: testFailCount is " + str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

