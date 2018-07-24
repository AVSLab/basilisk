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
# Purpose:  Integrated test of the spacecraftPlus(), RWs, simpleNav() and
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 7, 2017
#


import pytest
import os, inspect, sys

from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioAttitudeFeedbackRW


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useJitterSimple, useRWVoltageIO", [(False, False), (True, False), (False, True)])
# provide a unique test method name, starting with test_
def test_bskAttitudeFeedbackRW(show_plots, useJitterSimple, useRWVoltageIO):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataSigmaBR, dataUsReq, numDataPoints, figureList = \
        scenarioAttitudeFeedbackRW.run(show_plots, useJitterSimple, useRWVoltageIO)



    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataUsRed = dataUsReq[::skipValue]
    dataSigmaBRRed = dataSigmaBR[::skipValue]
    dataPosRed = dataPos[::skipValue]

    # setup truth data for unit test
    truePos = [
        [-4.0203386903966456e+06, 7.4905667418525163e+06, 5.2482992115893615e+06],
        [-4.6421397265661405e+06, 7.0494536040548589e+06, 5.3596540365520352e+06],
        [-5.2364026851194846e+06, 6.5665185661712112e+06, 5.4392129624019405e+06],
        [-5.7996735881523984e+06, 6.0447162866713591e+06, 5.4865782619213760e+06],
        [-6.3286970190056376e+06, 5.4872170491069853e+06, 5.5015438477240102e+06]
    ]
    trueUs = trueSigmaBR = []

    if useJitterSimple is False and useRWVoltageIO is False:
        trueUs = [
            [3.8000000000000000e-01, 4.0000000000000008e-01, -1.5000000000000013e-01],
            [1.0886536396638849e-02, -5.1081088867427316e-01, -4.9465001721576522e-02],
            [-5.3356020546124400e-02, 7.3280121862582176e-02, 2.3622678489553288e-02],
            [2.4053273555142078e-02, -2.7877284619006338e-03, 1.0490688667807481e-04],
            [-4.4666896866933491e-03, -3.0806563642653785e-03, -3.2335993502972866e-03]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01],
            [1.3881610052729310e-02, -1.5485878435765174e-01, -1.7589430807049264e-02],
            [-2.7923740563112063e-02, 1.1269976169106372e-02, 4.7871422910631181e-04],
            [6.1959342447429396e-03, 2.4918559180853771e-03, 3.7300409079442311e-03],
            [1.5260133637049377e-05, -1.2491549414001010e-03, -1.4158582039329860e-03]
        ]

    if useJitterSimple is True and useRWVoltageIO is False:
        trueUs = [
            [3.8000000000000000e-01, 4.0000000000000008e-01, -1.5000000000000013e-01],
            [1.1065138334427127e-02, -5.1268877119457312e-01, -5.0197674196675285e-02],
            [-5.0848148107366119e-02, 7.4099100493587991e-02, 2.2771409384433863e-02],
            [2.4176151141330489e-02, -2.8562626784347737e-03, -1.1764370510636973e-04],
            [-4.4366215100514186e-03, -3.0640074660972742e-03, -3.2900068347372418e-03]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01],
            [1.4000649100987304e-02, -1.5524376685777014e-01, -1.7672779218442999e-02],
            [-2.7813457642929151e-02, 1.0946112552094834e-02, 4.4875764271143668e-04],
            [6.2095289346616083e-03, 2.4867062677496696e-03, 3.6922501700617210e-03],
            [1.6904290117009812e-05, -1.2461998354027675e-03, -1.4336939003900724e-03]
        ]
        truePos = [
            [-4.0203386903966456e+06, 7.4905667418525163e+06, 5.2482992115893615e+06],
            [-4.6421397299586143e+06, 7.0494535906981705e+06, 5.3596540487686256e+06],
            [-5.2364027267925823e+06, 6.5665184975009989e+06, 5.4392130114279613e+06],
            [-5.7996736190037578e+06, 6.0447161564746955e+06, 5.4865783260474391e+06],
            [-6.3286970385898352e+06, 5.4872168578844322e+06, 5.5015439263280211e+06]
        ]

    if useJitterSimple is False and useRWVoltageIO is True:
        trueUs = [
            [3.8000000000000000e-01, 4.0000000000000008e-01, -1.5000000000000013e-01],
            [1.1231402312140600e-02, -5.1291709034434607e-01, -4.9996296037748973e-02],
            [-5.3576899204811061e-02, 7.3722479933297697e-02, 2.3880144351365474e-02],
            [2.4193559082756406e-02, -2.8516319358299399e-03, 2.6158801499764212e-06],
            [-4.5358804715397794e-03, -3.0828353818758043e-03, -3.2251584952585279e-03]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01],
            [1.4061613716759683e-02, -1.5537401133724818e-01, -1.7736020110557197e-02],
            [-2.8072554033139227e-02, 1.1328152717859538e-02, 4.8023651815938773e-04],
            [6.2505180487499833e-03, 2.4908595924511283e-03, 3.7332111196198281e-03],
            [-1.2999627747526236e-06, -1.2575327981617813e-03, -1.4238011880860959e-03]
        ]

    # compare the results to the truth values
    accuracy = 1e-6

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueUs, dataUsRed, accuracy, "Lr Vector",
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
        print testFailCount
        print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages

