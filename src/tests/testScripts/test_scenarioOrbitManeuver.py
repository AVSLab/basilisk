''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus() and gravity modules illustrating
#           how impulsive Delta_v maneuver can be simulated with stoping and starting the
#           simulation.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 26, 2016
#

import sys, os, inspect
import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioOrbitManeuver


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
# The following 'parametrize' function decorator provides the parameters for each
#   of the multiple test runs for this test.


@pytest.mark.parametrize("maneuverCase", [0, 1])
# provide a unique test method name, starting with test_
def test_scenarioOrbitManeuver(show_plots, maneuverCase):
    '''This function is called by the py.test environment.'''

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # each test method requires a single assert method to be called
    dataPos, figureList = scenarioOrbitManeuver.run(show_plots, maneuverCase)



    # setup truth data for unit test
    if maneuverCase == 0:
        truePos = [
            [10298352.587758573, 40947481.244493686, 0.0]
        ]
    if maneuverCase == 1:
        truePos = [
            [5937590.072546725, 3675220.9560903916, 477503.77340122446]
        ]

    # compare the results to the truth values
    accuracy = 1e-6
    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPos, accuracy, "r_BN_N Vector",
        testFailCount, testMessages)

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in figureList.items():
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    #   print out success message if no error were found
    if testFailCount == 0:
        print "PASSED "
    else:
        print "testFailCount: " + str(testFailCount)
        print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages