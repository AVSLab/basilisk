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
                , [-1.8150865887545696e+08, -6.3070993898878656e+07,  6.1267913051301539e+07]
                , [-1.9303771205266798e+08, -1.5676982447684008e+08,  2.5889372561579809e+07]
                , [-9.5984847333963335e+07, -1.6150851236724496e+08, -2.3710816061730530e+07]
            ]
        if integratorCase == "rkf45":
            truePos = [
                [2.6965319797723856e+07, -4.0438014777803928e+07, -3.0909521009497888e+07]
                , [-1.9216506804530445e+08, -5.6289160679974444e+07,  6.9455499628984332e+07]
                , [-2.3299581149964386e+08, -1.5552472452278799e+08,  4.6001444710378133e+07]
                , [-1.8459881436340547e+08, -1.9692546690809220e+08,  4.1756159526298083e+06]
            ]
        if integratorCase == "rkf78":
            truePos = [
                [2.6965319797723856e+07, -4.0438014777803928e+07, -3.0909521009497888e+07]
                , [-1.9213620413957629e+08, -5.6231399043057553e+07,  6.9466655149912968e+07]
                , [-2.3288519903034091e+08, -1.5542401202654269e+08,  4.5991374262165107e+07]
                , [-1.8431058525168183e+08, -1.9672328614927649e+08,  4.1229949341833070e+06]
            ]

        # compare the results to the truth values
        accuracy = 1.0  # meters
        testFailCount, testMessages = unitTestSupport.compareArray(
            truePos, dataPosRed, accuracy, "r_BN_N Vector, case: " + integratorCase,
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
