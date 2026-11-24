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

    # Initial deputy-minus-chief offsets relative to the commanded formation.
    initial_errors = (300., -300., -100., 500., 100., 500.)  # [ppm, ppm, µrad, µrad, µrad, µrad]
    error_axes = figureList[f"scenarioFormationReconfig3{int(useRefAttitude)}"].axes
    assert len(error_axes) == len(initial_errors)
    for axis, initial in zip(error_axes, initial_errors):
        samples = axis.lines[0].get_ydata()
        assert len(samples) == len(dataPos)
        assert samples[0] == pytest.approx(initial)
        assert abs(samples[-1]) < abs(initial)

    # Both start at +500 µrad. Their distinct signed final errors identify each trace.
    final_errors = {axis.get_title(): axis.lines[0].get_ydata()[-1] for axis in error_axes}
    expected = (-5.22, -47.78) if useRefAttitude else (-4.34, -39.81)  # [µrad] RAAN, mean anomaly
    actual = (final_errors["Right ascension of the ascending node"], final_errors["Mean anomaly"])
    assert actual == pytest.approx(expected, abs=0.1)  # [µrad]

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
    # Deputy position [m], velocity [m/s], and MRP error [-] for burns at 4458, 7080, and 10206 s.
    truePos2 = trueVel2 = []
    if useRefAttitude:
        truePos2 = [
            [-2667507.97243685,  -6983454.82971912,   4626165.55303272]
            , [  3241108.94387093, -12365029.53882338,  -5606492.86033854]
            , [  6957592.47443948,  -6517925.35190657, -12049167.00045167]
            , [  6203605.90128806,   3379408.64114478, -10749316.17389493]
            , [  -899280.99902852,   6988622.91379951,   1559594.99370778]
            , [ -1499283.32617736,  -9685683.13460855,   2596270.40502664]
        ]
        trueVel2 = [
            [ 1953.40294294, -6262.33384039, -3379.83469644]
            , [ 2251.63426593,   772.82266533, -3901.38099724]
            , [  734.53355850,  3672.29767189, -1275.08081506]
            , [-1519.37239927,  4061.85758232,  2633.58823852]
            , [-3836.64774131, -3914.08199415,  6647.63003330]
            , [ 2478.79610972, -4217.72681391, -4296.15620198]
        ]
    else:
        truePos2 = [
            [-2667507.97243685,  -6983454.82971912,   4626165.55303272]
            , [  3241108.94387093, -12365029.53882338,  -5606492.86033854]
            , [  6957591.76354100,  -6517927.37032531, -12049168.01625015]
            , [  6203602.48704407,   3379393.14730503, -10749325.83435110]
            , [  -899289.31137017,   6988570.54408341,   1559600.66973449]
            , [ -1499181.76609050,  -9685755.05386250,   2596111.68264744]
        ]
        trueVel2 = [
            [1953.40294294, -6262.33384039, -3379.83469644]
            , [ 2251.63426593,   772.82266533, -3901.38099724]
            , [  734.53147131,  3672.29162908, -1275.08387744]
            , [-1519.37270921,  4061.84615236,  2633.58541153]
            , [-3836.64938055, -3914.13722047,  6647.64517290]
            , [ 2478.82436534, -4217.63944939, -4296.20314042]
        ]
    trueAttErr = []
    if useRefAttitude:
        trueAttErr = [
            [0.00000000e+00,  0.00000000e+00,  0.00000000e+00]
            , [ 5.21804822e-15,  0.00000000e+00,  0.00000000e+00]
            , [ 2.65911396e-03,  2.36681599e-03, -1.80444161e-03]
            , [ 1.44285759e-01,  1.31628282e-02,  1.83033495e-02]
            , [ 1.44328993e-15,  2.42608910e-16,  1.39828515e-16]
            , [ 1.96720418e-12,  4.85774354e-12,  7.41408928e-12]
        ]
    else:
        trueAttErr = [
            [0.00000000e+00,  0.00000000e+00,  0.00000000e+00]
            , [ 1.01291777e-15, -2.81104312e-15, -2.10435630e-15]
            , [-1.06877222e-03,  2.79148299e-03,  9.70789605e-04]
            , [ 8.77861065e-02,  6.33908188e-02, -3.07211170e-02]
            , [ 1.06899914e-15,  6.47426427e-16, -4.83988072e-16]
            , [-5.51346908e-05,  3.10799986e-04, -1.30667867e-04]
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
