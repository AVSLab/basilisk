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

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + "/../../examples")
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

    dataPos, dataVel, dataPos2, dataVel2, numDataPoints, figureList = (
        scenarioFormationMeanOEFeedback.run(show_plots, useClassicElem, 1.0)
    )

    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataPos = dataPos[::skipValue]
    dataVel = dataVel[::skipValue]
    dataPos2 = dataPos2[::skipValue]
    dataVel2 = dataVel2[::skipValue]

    # setup truth data for unit test
    truePos = [
        [2.25733295e06, 6.10774942e06, 1.07696101e06],
        [-1.16514866e07, -5.29342483e06, -9.38604632e05],
        [-4.18867411e06, -1.45289784e07, -2.56460544e06],
        [7.22420145e06, -9.08651262e06, -1.59841285e06],
    ]

    trueVel = [
        [-8.64065677e03, 3.09716314e03, 5.46113424e02],
        [2.89200675e02, -4.99813535e03, -8.81777665e02],
        [3.85680359e03, -8.90742994e02, -1.55265663e02],
        [2.70939151e03, 4.86526632e03, 8.59982579e02],
    ]
    truePos2 = trueVel2 = []
    if useClassicElem:
        truePos2 = [
            [2.24178592e06, 6.11203995e06, 1.07566911e06],
            [-1.16487043e07, -5.31117839e06, -9.39992120e05],
            [-4.17942261e06, -1.45370616e07, -2.56344522e06],
            [7.22821253e06, -9.08919123e06, -1.59782730e06],
        ]
        trueVel2 = [
            [-8.64850680e03, 3.08090462e03, 5.42902525e02],
            [2.94711417e02, -4.99616112e03, -8.80717414e02],
            [3.85639038e03, -8.87524222e02, -1.54829462e02],
            [2.70731452e03, 4.86501852e03, 8.59067219e02],
        ]
    else:
        truePos2 = [
            [2.24178592e06, 6.11203995e06, 1.07566911e06],
            [-1.16532985e07, -5.30431921e06, -9.38921358e05],
            [-4.19058734e06, -1.45343280e07, -2.56601902e06],
            [7.22014205e06, -9.09601164e06, -1.60521870e06],
        ]
        trueVel2 = [
            [-8.64850680e03, 3.08090462e03, 5.42902525e02],
            [2.91619332e02, -4.99568227e03, -8.80774817e02],
            [3.85549147e03, -8.90163289e02, -1.56770498e02],
            [2.71092010e03, 4.86177055e03, 8.58622744e02],
        ]

    # compare the results to the truth values
    accuracy = 1e-6

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        truePos, dataPos, accuracy, "chief r_BN_N Vector", testFailCount, testMessages
    )

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        trueVel, dataVel, accuracy, "chief v_BN_N Vector", testFailCount, testMessages
    )

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        truePos2,
        dataPos2,
        accuracy,
        "deputy r_BN_N Vector",
        testFailCount,
        testMessages,
    )

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        trueVel2,
        dataVel2,
        accuracy,
        "deputy v_BN_N Vector",
        testFailCount,
        testMessages,
    )

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("# Errors:", testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages
