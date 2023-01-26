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
    """This function is called by the py.test environment."""
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
        [-2668550.43581724,  -6980898.10827464,   4622064.93739554]
        ,[  3239719.11900864, -12361691.80735476,  -5611358.11637523]
        ,[  6956886.21097725,  -6512028.87881183, -12049680.37988781]
        ,[  6203357.9456249,    3386315.3204033,  -10744531.13935835]
        ,[  -904077.16857348,   6986198.37448257,   1565907.58993232]
        ,[ -1497223.74594733,  -9688096.70685282,   2593267.5982793]
    ]
    trueVel = [
        [1952.09236395, -6264.36921517, -3381.12315544]
        , [ 2251.85678459,   773.95089899, -3900.33036227]
        , [  735.27848237,  3673.48086541, -1273.53968917]
        , [-1521.51022886,  4060.99888361,  2635.33302062]
        , [-3836.04015487, -3920.91042533,  6644.21644811]
        , [ 2480.74592749, -4214.7585866,  -4296.77798709]
    ]
    truePos2 = trueVel2 = []
    if useRefAttitude:
        truePos2 = [
            [-2667507.97243685,  -6983454.82971912,   4626165.55303272]
            , [  3241108.94387093, -12365029.53882338,  -5606492.86033854]
            , [  6957594.00740821,  -6517924.48451533, -12049169.67128781]
            , [  6203604.90565029,   3379408.49215067, -10749322.78076205]
            , [  -899274.80454994,   6988633.22340264,   1559582.60590812]
            , [ -1499288.94036976,  -9685672.95831528,   2596285.54475707]
        ]
        trueVel2 = [
            [ 1953.40294294, -6262.33384039, -3379.83469644]
            , [ 2251.63426593,   772.82266533, -3901.38099724]
            , [  734.53359595,  3672.29757633, -1275.08087935]
            , [-1519.3712049,   4061.85725527,  2633.58633737]
            , [-3836.64595382, -3914.06736883,  6647.63278995]
            , [ 2478.79370247, -4217.73758483, -4296.15323074]
        ]
    else:
        truePos2 = [
            [-2667507.97243685,  -6983454.82971912,   4626165.55303272]
            , [  3241108.94387093, -12365029.53882338,  -5606492.86033854]
            , [  6957593.3006838,   -6517926.49084233, -12049170.68096033]
            , [  6203601.49094786,   3379393.02430127, -10749332.43260227]
            , [  -899283.12085206,   6988580.86462193,   1559588.28436277]
            , [ -1499187.38519734,  -9685744.8647062,    2596126.83315495]
        ]
        trueVel2 = [
            [1953.40294294, -6262.33384039, -3379.83469644]
            , [ 2251.63426593,   772.82266533, -3901.38099724]
            , [  734.53150821,  3672.2915335,  -1275.08394125]
            , [-1519.37151914,  4061.84582195,  2633.58351375]
            , [-3836.64758968, -3914.12259276,  6647.64791968]
            , [ 2478.82196216, -4217.65022651, -4296.20016979]
        ]
    trueAttErr = []
    if useRefAttitude:
        trueAttErr = [
            [0.00000000e+00,  0.00000000e+00,  0.00000000e+00]
            , [ 5.21804822e-15,  0.00000000e+00,  0.00000000e+00]
            , [ 2.73188701e-03,  2.42888055e-03, -1.85264053e-03]
            , [ 1.50892109e-01,  1.31129690e-02,  1.99974845e-02]
            , [ 1.66533454e-15,  2.49085251e-16,  1.43583255e-16]
            , [ 2.01960670e-12,  4.98741819e-12,  7.61317659e-12]
        ]
    else:
        trueAttErr = [
            [0.00000000e+00,  0.00000000e+00,  0.00000000e+00]
            , [ 1.01291777e-15, -2.81104312e-15, -2.10435630e-15]
            , [-1.09830506e-03,  2.86489461e-03,  9.96740835e-04]
            , [ 9.08383514e-02,  6.71564082e-02, -3.21870531e-02]
            , [ 1.35545560e-15,  1.02827666e-15, -6.35661937e-16]
            , [-5.65881607e-05,  3.19108458e-04, -1.34177223e-04]
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
        truePos2, dataPos2, accuracy, "deputy r2_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArrayRelative(
        trueVel2, dataVel2, accuracy, "deputy v2_BN_N Vector",
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
