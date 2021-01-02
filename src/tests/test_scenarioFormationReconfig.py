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

import pytest
import os
import sys
import inspect
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioFormationReconfig


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useRefAttitude", [(True), (False)])
@pytest.mark.scenarioTest

# provide a unique test method name, starting with test_
def test_scenarioFormationReconfig(show_plots, useRefAttitude):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataVel, dataPos2, dataVel2, dataAttErr, numDataPoints, figureList = \
        scenarioFormationReconfig.run(show_plots, useRefAttitude)

    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataPos = dataPos[::skipValue]
    dataVel = dataVel[::skipValue]
    dataPos2 = dataPos2[::skipValue]
    dataVel2 = dataVel2[::skipValue]
    dataAttErr = dataAttErr[::skipValue]

    # setup truth data for unit test
    truePos = [
        [-2.66855044e+06, -6.98089811e+06, 4.62206494e+06]
        , [ 4.10060709e+06, -1.19138745e+07, -7.10245983e+06]
        , [ 7.29007364e+06, -3.37655699e+06, -1.26267779e+07]
        , [ 3.48198922e+06,  7.41168323e+06, -6.03098225e+06]
        , [ -3.14587454e+06, -5.05638958e+06,  5.44881453e+06]
    ]
    trueVel = [
        [1.95209236e+03, -6.26436922e+03, -3.38112316e+03]
        , [ 2.04824699e+03,  1.44895265e+03, -3.54766786e+03]
        , [ 8.41191317e+01,  4.12343024e+03, -1.45698610e+02]
        , [-3.05009556e+03,  2.22223576e+03,  5.28292047e+03]
        , [ 1.40039612e+03, -7.39482598e+03, -2.42555722e+03]
    ]
    truePos2 = trueVel2 = []
    if(useRefAttitude):
        truePos2 = [
            [-2.66750797e+06, -6.98345483e+06,  4.62616555e+06]
            , [ 4.10190373e+06, -1.19176629e+07, -7.09808744e+06]
            , [ 7.29028788e+06, -3.38327054e+06, -1.26277120e+07]
            , [ 3.48508705e+06,  7.40732516e+06, -6.03779734e+06]
            , [-3.14599627e+06, -5.05151074e+06,  5.45063092e+06]
        ]
        trueVel2 = [
            [ 1.95340294e+03, -6.26233384e+03, -3.37983470e+03]
            , [ 2.04800676e+03,  1.44783320e+03, -3.54907414e+03]
            , [ 8.36446804e+01,  4.12260778e+03, -1.47760433e+02]
            , [-3.04767120e+03,  2.22595358e+03,  5.28157214e+03]
            , [ 1.39739658e+03, -7.39779282e+03, -2.42282925e+03]
        ]
    else:
        truePos2 = [
            [-2.66750797e+06, -6.98345483e+06, 4.62616555e+06]
            , [ 4.10190373e+06, -1.19176629e+07, -7.09808744e+06]
            , [ 7.29028565e+06, -3.38327733e+06, -1.26277156e+07]
            , [ 3.48508338e+06,  7.40729862e+06, -6.03780781e+06]
            , [-3.14593170e+06, -5.05164020e+06,  5.45053219e+06]
        ]
        trueVel2 = [
            [1.95340294e+03, -6.26233384e+03, -3.37983470e+03]
            , [ 2.04800676e+03,  1.44783320e+03, -3.54907414e+03]
            , [ 8.36430570e+01,  4.12260193e+03, -1.47763855e+02]
            , [-3.04767240e+03,  2.22594581e+03,  5.28157616e+03]
            , [ 1.39746091e+03, -7.39774973e+03, -2.42293199e+03]
        ]
    trueAttErr = []
    if(useRefAttitude):
        trueAttErr = [
            [-1,0,0]
            , [1.11022302e-16, 0.00000000e+00, 0.00000000e+00]
            , [ 7.35112600e-08,  6.38971464e-08, -4.49913751e-08]
            , [ 1.02484475e-08,  1.77691147e-09,  1.12311917e-09]
            , [ 7.20644592e-08,  1.83206367e-07,  2.97356826e-07]
        ]
    else:
        trueAttErr = [
            [-1.68285300e-01, 3.48843815e-01, 4.23586373e-01]
            , [ 3.92603788e-17, -1.09929061e-16, -3.14083031e-17]
            , [-2.93932037e-08,  7.50468882e-08, 2.42035010e-08]
            , [ 6.99759143e-09,  4.14940045e-09, -2.94105715e-09]
            , [-4.67398483e-17, -6.54357876e-17,  6.07618028e-17]
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
    
    testFailCount, testMessages = unitTestSupport.compareArray(
        trueAttErr, dataAttErr, accuracy, "deputy attitude Error",
        testFailCount, testMessages)

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
