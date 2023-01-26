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
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstration of how to setup and use different integrators in
#           Basilisk.  The simulation performs a 3-DOF orbit scenario.
# Author:   João Vaz Carneiro
# Creation Date:  Oct. 4, 2021
#

import inspect
import os
import sys

import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioVariableTimeStepIntegrators


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Scott's brain no-worky\n")
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
# @pytest.mark.parametrize("integratorCase", ["rk4", "rkf45", "euler", "rk2"])
@pytest.mark.scenarioTest
def test_scenarioIntegrators(show_plots):
    """This function is called by the py.test environment."""

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # for integratorCase in ["rk4", "rkf45", "rkf78"]:
    for integratorCase in ["rk4", "rkf45", "rkf78"]:

        # each test method requires a single assert method to be called
        posData, figureList = scenarioVariableTimeStepIntegrators.run(show_plots, integratorCase, 1e-4, 1e-8)

        numTruthPoints = 5
        skipValue = int(len(posData) / (numTruthPoints - 1))
        dataPosRed = posData[::skipValue]

        # setup truth data for unit test
        if integratorCase == "rk4":
            truePos = [
                [2.6965319797723856e+07, -4.0438014777803928e+07, -3.0909521009497888e+07]
                , [-1.8150865907367039e+08, -6.3070994959505990e+07,  6.1267912683989421e+07]
                , [-1.9303770991326889e+08, -1.5676982514195076e+08,  2.5889371222740099e+07]
                , [-9.5984840659665108e+07, -1.6150850904769760e+08, -2.3710817876644962e+07]
            ]
        if integratorCase == "rkf45":
            truePos = [
                [2.6965319797723856e+07, -4.0438014777803928e+07, -3.0909521009497888e+07]
                , [-1.9213449717316824e+08, -5.6224612680104271e+07,  6.9468790545288116e+07]
                , [-2.3287921820332012e+08, -1.5541450266266349e+08,  4.5992609018449008e+07]
                , [-1.8429335189943227e+08, -1.9670820137819085e+08,  4.1211605646267980e+06]
            ]
        if integratorCase == "rkf78":
            truePos = [
                [2.6965319797723856e+07, -4.0438014777803928e+07, -3.0909521009497888e+07]
                , [-1.9213620487859124e+08, -5.6231399937485360e+07, 6.9466655120191082e+07]
                , [-2.3288519890219730e+08, -1.5542401305950305e+08, 4.5991373747153923e+07]
                , [-1.8431058338898057e+08, -1.9672328628053436e+08, 4.1229939645756241e+06]
            ]

    # compare the results to the truth values
    accuracy = 1.0  # meters

    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPosRed, accuracy, "r_BN_N Vector",
        testFailCount, testMessages)

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages
