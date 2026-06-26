#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import inspect
import os
import sys

import pytest
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simHelpers

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioFormationMeanOEFeedback


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useClassicElem", [(True), (False)])
@pytest.mark.scenarioTest

# provide a unique test method name, starting with test_
def test_bskFormationMeanOEFeedback(show_plots, useClassicElem):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataVel, dataPos2, dataVel2, numDataPoints, figureList = \
        scenarioFormationMeanOEFeedback.run(show_plots, useClassicElem, 1.)

    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataPos = dataPos[::skipValue]
    dataVel = dataVel[::skipValue]
    dataPos2 = dataPos2[::skipValue]
    dataVel2 = dataVel2[::skipValue]

    # setup truth data for unit test
    truePos = [
        [2.25733295e+06, 6.10774942e+06, 1.07696101e+06]
        , [ -1.16512929e+07, -5.29348123e+06, -9.38586463e+05]
        , [ -4.18797841e+06, -1.45287489e+07, -2.56449944e+06]
        , [  7.22483339e+06, -9.08494945e+06, -1.59809085e+06]
    ]

    trueVel = [
        [-8.64065671e+03, 3.09716311e+03, 5.46113421e+02]
        , [ 2.89326360e+02, -4.99814168e+03, -8.81761101e+02]
        , [ 3.85696908e+03, -8.90509382e+02, -1.55219414e+02]
        , [ 2.70894786e+03, 4.86593271e+03,  8.60079018e+02]
    ]
    truePos2 = trueVel2 = []
    if(useClassicElem):
        truePos2 = [
            [2.24178592e+06, 6.11203995e+06, 1.07566911e+06]
            , [ -1.16485108e+07, -5.31123406e+06, -9.39973856e+05]
            , [ -4.17872812e+06, -1.45368314e+07, -2.56333927e+06]
            , [  7.22884304e+06, -9.08762888e+06, -1.59750574e+06]
        ]
        trueVel2 = [
            [-8.64850674e+03, 3.08090460e+03, 5.42902521e+02]
            , [ 2.94836877e+02, -4.99616729e+03, -8.80700867e+02]
            , [ 3.85655555e+03, -8.87290829e+02, -1.54783294e+02]
            , [ 2.70687143e+03,  4.86568415e+03,  8.59163503e+02]
        ]
    else:
        truePos2 = [
            [2.24178592e+06, 6.11203995e+06, 1.07566911e+06]
            , [ -1.16531048e+07, -5.30437497e+06, -9.38902968e+05]
            , [ -4.18989234e+06, -1.45340982e+07, -2.56591300e+06]
            , [ 7.22077411e+06, -9.09444949e+06, -1.60489676e+06]
        ]
        trueVel2 = [
            [-8.64850674e+03, 3.08090460e+03, 5.42902521e+02]
            , [ 2.91744842e+02, -4.99568854e+03, -8.80758263e+02]
            , [ 3.85565686e+03, -8.89929967e+02, -1.56724355e+02]
            , [ 2.71047752e+03,  4.86243659e+03,  8.58719415e+02]
        ]

    # compare the results to the truth values
    accuracy = 1e-6

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        truePos, dataPos, accuracy, "chief r_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        trueVel, dataVel, accuracy, "chief v_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        truePos2, dataPos2, accuracy, "deputy r_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        trueVel2, dataVel2, accuracy, "deputy v_BN_N Vector",
        testFailCount, testMessages)

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        simHelpers.saveScenarioFigure(pltName, plt, path)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("# Errors:", testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages
