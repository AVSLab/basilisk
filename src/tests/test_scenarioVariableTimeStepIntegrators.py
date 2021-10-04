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
    for integratorCase in ["rk4", "rkf45"]:

        # each test method requires a single assert method to be called
        posData, figureList = scenarioVariableTimeStepIntegrators.run(show_plots, integratorCase, 1e-4, 1e-8)

        numTruthPoints = 5
        skipValue = int(len(posData) / (numTruthPoints - 1))
        dataPosRed = posData[::skipValue]

        # setup truth data for unit test
        if integratorCase == "rk4":
            truePos = [
                [2.6965319797723856e+07, -4.0438014777803928e+07, -3.0909521009497888e+07]
                , [-1.7952552182753333e+08, -4.8624117998177312e+07, 6.6622073840559043e+07]
                , [-2.2025270694539085e+08, -1.4489345309456939e+08, 4.4416011758481033e+07]
                , [-1.7605646301700157e+08, -1.8712249702868047e+08, 4.2845786279798364e+06]
                , [-5.6908923259079009e+07, -1.4444116692739022e+08, -3.5373161176619194e+07]
            ]
        if integratorCase == "rkf45":
            truePos = [
                [2.6965319797723856e+07, -4.0438014777803928e+07, -3.0909521009497888e+07]
                , [-1.8292479511653122e+08, -4.5628416840100564e+07, 6.9598258905080795e+07]
                , [-2.3320052481526387e+08, -1.4259969077232993e+08, 5.1760657494025618e+07]
                , [-2.0364594584579268e+08, -1.9272717532164046e+08, 1.5340871461692769e+07]
                , [-1.0806127770149110e+08, -1.7853691626215625e+08, -2.5252700754825063e+07]
            ]
        if integratorCase == "rkf78":
            truePos = [
                [6343122.681395919, 396501.78632660967, -2932539.2914087307]
                , [6385759.757220186, 1232061.6219036756, -2587584.918333401]
                , [6321657.127943617, 2047027.1643309772, -2199378.3835694017]
                , [6151884.468695515, 2827775.46506309, -1774408.027800635]
                , [5879278.326923609, 3561255.294074021, -1319777.089191588]
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
