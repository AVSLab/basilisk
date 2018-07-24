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
# Purpose:  Integrated test of the spacecraftPlus(), extForceTorque, simpleNav() and
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit.
#           This scenario is the same as test_scenarioAttitudeControl, but with the
#           difference that here the control and dynamics are executed at different
#           frequencies or time steps.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 25, 2016
#


import pytest
import os, sys
import inspect
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioAttitudeFeedback2T



# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useUnmodeledTorque, useIntGain", [(False, False), (True, False), (True, True)])
# provide a unique test method name, starting with test_
def test_bskAttitudeFeedback2T(show_plots, useUnmodeledTorque, useIntGain):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataSigmaBR, dataLr, numDataPoints, figureList = \
        scenarioAttitudeFeedback2T.run(show_plots, useUnmodeledTorque, useIntGain)


    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataLrRed = dataLr[::skipValue]
    dataSigmaBRRed = dataSigmaBR[::skipValue]
    dataPosRed = dataPos[::skipValue]

    # setup truth data for unit test
    truePos = [
        [-4.0203386903966456e+06, 7.4905667418525163e+06, 5.2482992115893615e+06]
        , [-4.6421397265661405e+06, 7.0494536040548589e+06, 5.3596540365520352e+06]
        , [-5.2364026851194846e+06, 6.5665185661712112e+06, 5.4392129624019405e+06]
        , [-5.7996735881523984e+06, 6.0447162866713591e+06, 5.4865782619213760e+06]
        , [-6.3286970190056376e+06, 5.4872170491069853e+06, 5.5015438477240102e+06]
    ]
    trueLr = trueSigmaBR = []
    if useUnmodeledTorque == True and useIntGain == True:
        trueLr = [
            [-3.8540000000000002e-01, -3.5200000000000009e-01, 4.2000000000000121e-02]
            , [-2.4832413594005637e-01, 2.7423420741984977e-01, -1.2547995906140999e-01]
            , [-2.4963855704325660e-01, 2.4180503459076927e-01, -9.0824655654560021e-02]
            , [-2.4705814926163225e-01, 2.4984933912594240e-01, -9.9789581766606752e-02]
            , [-2.4992663484004582e-01, 2.4915540593910049e-01, -9.9730854856601880e-02]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
            , [2.4124788077353267e-02, -8.8078468077718686e-02, 6.7556236560029792e-02]
            , [2.0013848145133590e-02, -1.5036479216989354e-02, 1.6743292993630865e-02]
            , [4.3556855886602566e-03, -8.2916392106937194e-03, 4.8022149157636237e-03]
            , [1.7102077355609178e-03, -2.5229471654219780e-03, 2.0963057897404771e-03]
        ]
    if useUnmodeledTorque == True and useIntGain == False:
        trueLr = [
            [-3.8000000000000000e-01, -4.0000000000000008e-01, 1.5000000000000013e-01]
            , [-2.6967258574434960e-01, 2.3852492578210521e-01, -9.9303066167128723e-02]
            , [-2.4553483719840241e-01, 2.5582895110635612e-01, -9.9783874073020584e-02]
            , [-2.5082575869743895e-01, 2.4917049711833658e-01, -9.9921820727609134e-02]
            , [-2.4986476881781602e-01, 2.5008633794967206e-01, -1.0003104112824485e-01]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
            , [6.3945338706459742e-02, -8.7562909724589022e-02, 4.4198807712487694e-02]
            , [7.1960157856111040e-02, -7.0992542477623266e-02, 2.7112368217686023e-02]
            , [7.1431247623012131e-02, -7.1324233641929718e-02, 2.8746725391756406e-02]
            , [7.1414708584927961e-02, -7.1455518150866384e-02, 2.8552586824521019e-02]
        ]
    if useUnmodeledTorque == False and useIntGain == False:
        trueLr = [
            [-3.8000000000000000e-01, -4.0000000000000008e-01, 1.5000000000000013e-01]
            , [2.9018622651951317e-02, -2.3731740129077500e-03, 1.8500888075394767e-02]
            , [-1.4212083106812448e-03, 1.7754834987403689e-03, -1.3760887721230200e-03]
            , [-6.8638983386268455e-05, -2.6617199062440423e-04, 5.5312276312328556e-05]
            , [3.3478249139870173e-05, 2.8598845181088252e-05, -1.3792582437169445e-06]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
            , [-1.6310559825609434e-02, -1.1632615581332386e-02, 9.0311147891659286e-03]
            , [1.8165879114118769e-03, 7.4688330133023404e-04, -1.2070602115782872e-04]
            , [-1.8335140530225598e-04, -2.9214999036672645e-05, -5.7216976124152215e-06]
            , [1.6181183222292735e-05, -1.0129144274203115e-06, 5.1639058023290004e-07]
        ]
    # compare the results to the truth values
    accuracy = 1e-6

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueLr, dataLrRed, accuracy, "Lr Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPosRed, accuracy, "r_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueSigmaBR, dataSigmaBRRed, accuracy, "sigma_BR Set",
        testFailCount, testMessages)

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in figureList.items():
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    #   print out success message if no error were found
    if testFailCount == 0:
        print "PASSED "
    else:
        print "# Errors:", testFailCount
        print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(False,  # do unit tests
        True,  # show_plots
        False,  # useUnmodeledTorque
        False  # useIntGain
        )
