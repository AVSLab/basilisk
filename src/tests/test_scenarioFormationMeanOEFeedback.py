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

    # setup truth data for unit test. The planet orientation is advanced as a rotation by the gravity effector. The
    # chief orbit, which is uncontrolled, matches an
    # independent integration of the degree-2 GGM03S field with the SPICE orientation to 0.4 m.
    truePos = [
        [2.25733295e+06, 6.10774942e+06, 1.07696101e+06]
        , [-1.16512871e+07, -5.29348500e+06, -9.38587959e+05]
        , [-4.18795722e+06, -1.45287438e+07, -2.56449870e+06]
        , [7.22485250e+06, -9.08490705e+06, -1.59808179e+06]
    ]

    trueVel = [
        [-8.64065671e+03, 3.09716311e+03, 5.46113421e+02]
        , [2.89330869e+02, -4.99814200e+03, -8.81761232e+02]
        , [3.85697370e+03, -8.90502532e+02, -1.55217728e+02]
        , [2.70893560e+03, 4.86595115e+03, 8.60082783e+02]
    ]
    truePos2 = trueVel2 = []
    if(useClassicElem):
        truePos2 = [
            [2.24178592e+06, 6.11203995e+06, 1.07566911e+06]
            , [-1.16485050e+07, -5.31123782e+06, -9.39975349e+05]
            , [-4.17870694e+06, -1.45368263e+07, -2.56333852e+06]
            , [7.22886213e+06, -9.08758649e+06, -1.59749668e+06]
        ]
        trueVel2 = [
            [-8.64850674e+03, 3.08090460e+03, 5.42902521e+02]
            , [2.94841385e+02, -4.99616760e+03, -8.80700996e+02]
            , [3.85656016e+03, -8.87283980e+02, -1.54781609e+02]
            , [2.70685918e+03, 4.86570257e+03, 8.59167262e+02]
        ]
    else:
        truePos2 = [
            [2.24178592e+06, 6.11203995e+06, 1.07566911e+06]
            , [-1.16530991e+07, -5.30437873e+06, -9.38904459e+05]
            , [-4.18987116e+06, -1.45340931e+07, -2.56591226e+06]
            , [7.22079323e+06, -9.09440713e+06, -1.60488770e+06]
        ]
        trueVel2 = [
            [-8.64850674e+03, 3.08090460e+03, 5.42902521e+02]
            , [2.91749347e+02, -4.99568886e+03, -8.80758393e+02]
            , [3.85566147e+03, -8.89923124e+02, -1.56722671e+02]
            , [2.71046529e+03, 4.86245502e+03, 8.58723185e+02]
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
