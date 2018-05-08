''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#           MRP_Steering() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 7, 2017
#


import sys, os, inspect
import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioAttitudeSteering



# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("simCase", [0, 1, 2, 3])
def test_bskAttitudeFeedbackRW(show_plots, simCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataUsReq, dataSigmaBR, numDataPoints = scenarioAttitudeSteering.run(True, show_plots, simCase)


    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataUsRed = dataUsReq[::skipValue]
    dataSigmaBRRed = dataSigmaBR[::skipValue]
    dataPosRed = dataPos[::skipValue]

    # setup truth data for unit test
    truePos = [
        [-4.0203386903966456e+06, 7.4905667418525163e+06, 5.2482992115893615e+06]
        , [-4.6521134526484497e+06, 7.0418660564864147e+06, 5.3612159766829805e+06]
        , [-5.2553803148656003e+06, 6.5500098234596634e+06, 5.4412780230905702e+06]
        , [-5.8265176097709052e+06, 6.0181061406054925e+06, 5.4880726887781229e+06]
        , [-6.3621161952049593e+06, 5.4494925133885061e+06, 5.5013918238723921e+06]
    ]
    trueUs = trueSigmaBR = []

    if simCase == 0:
        trueUs = [
            [-9.3735248485571654e-01, 3.1657897125315637e-01, 9.4810829234824123e-01]
            , [4.7191187864907794e-02, -3.0070771545280692e-02, -2.9518238485616782e-02]
            , [1.7016920405413215e-02, -1.5817057493123454e-02, 1.7041006711987533e-03]
            , [1.0462152119029601e-02, -1.3835672310362663e-02, 4.7069307938441447e-03]
            , [7.9429100667987116e-03, -1.4225803320727568e-02, 5.0252369827272388e-03]
        ]
        trueSigmaBR = [
            [-7.2833461132468502e-01, 2.2697297495054194e-01, 5.0137002864322056e-01]
            , [-1.3375995332049051e-01, 2.5646319783854472e-02, 9.9894348609658193e-02]
            , [-2.6596473923292778e-02, 5.4185997488293940e-03, 2.0187273917517602e-02]
            , [-5.6260975047995720e-03, 1.1494179543533006e-03, 4.2750978169215689e-03]
            , [-1.1939254320624162e-03, 2.4393826018610445e-04, 9.0726554710592988e-04]
        ]
    if simCase == 1:
        trueUs = [
            [-9.3735248485571654e-01, 3.1657897125315637e-01, 9.4810829234824123e-01]
            , [4.7579565156181863e-02, -3.0349425236291302e-02, -3.0758207114084640e-02]
            , [1.7238526778602402e-02, -1.5954311821205262e-02, 1.6062184747321928e-03]
            , [1.0527976809083944e-02, -1.3845191318712704e-02, 4.7101442446694820e-03]
            , [7.9728496722686545e-03, -1.4219676900387224e-02, 5.0553065828725912e-03]
        ]
        trueSigmaBR = [
            [-7.2833461132468502e-01, 2.2697297495054194e-01, 5.0137002864322056e-01]
            , [-1.3680399695426440e-01, 2.4494146008974305e-02, 1.0332411551330754e-01]
            , [-2.6056558162020488e-02, 4.2029826932260149e-03, 2.1432334589966056e-02]
            , [-4.4366133145544101e-03, -2.0513208877515532e-04, 5.0642220585757807e-03]
            , [1.1719472835041103e-04, -1.1640729624930607e-03, 1.5886821146121159e-03]
        ]
    if simCase == 2:
        trueUs = [
            [5.3427982784745387e-01, 2.4950294126525017e+00, 2.5650481888590777e+00]
            , [5.4358272666288343e-04, -8.5844271924247484e-03, 1.1402708588315012e-03]
            , [2.5450020484230063e-03, 9.9162427490413693e-03, -1.4057457475478573e-03]
            , [-4.5294788639083094e-04, 2.0546056360929266e-02, 1.9900162652881723e-03]
            , [3.1307504052369151e-03, -5.2522043216498004e-02, -1.6091392647118244e-03]
        ]
        trueSigmaBR = [
            [5.9238063851049559e-03, 1.5059956700233917e-01, 1.6414145977723277e-01]
            , [-3.6813728373729475e-03, -1.2652171375839346e-03, -4.4560965458308630e-05]
            , [-3.7277213568358554e-03, 1.1091516017274978e-04, -4.6671624098081926e-06]
            , [-3.7353840579002540e-03, 1.1378091633261421e-03, 4.2064324048174655e-05]
            , [-3.6762687214375281e-03, -1.8796889549439252e-03, -6.9433492742490023e-05]
        ]
    if simCase == 3:
        trueUs = [
            [1.5463342442514050e+00, 2.4989188277446774e+00, 2.5666604392831682e+00]
            , [8.7400016892695467e-03, -2.2389585500966587e+00, -2.0172110694939094e-03]
            , [1.2942263063120260e-03, -9.0623022045545419e-04, 1.4725090860529959e-06]
            , [1.2175561336695026e-03, -9.9918509941947773e-04, 1.5678601387200759e-06]
            , [1.1342525280809703e-03, -1.0892870415896393e-03, 1.5413853915482904e-06]
        ]
        trueSigmaBR = [
            [5.9238063851049559e-03, 1.5059956700233917e-01, 1.6414145977723277e-01]
            , [1.0802870404159959e-04, -9.9707506726354513e-03, -9.5795774144360256e-05]
            , [6.3385780276990460e-08, 2.7851495444539080e-06, -4.9151262511072661e-07]
            , [3.9812854928119984e-09, 4.5312268214171323e-08, -9.7417287710713351e-09]
            , [1.0932992615194772e-10, 7.3941898989080499e-10, -1.8758847429304412e-10]
        ]

    # compare the results to the truth values
    accuracy = 1e-7

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueUs, dataUsRed, accuracy, "RW us Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPosRed, accuracy, "r_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueSigmaBR, dataSigmaBRRed, accuracy, "sigma_BR Set",
        testFailCount, testMessages)

    #   print out success message if no error were found
    if testFailCount == 0:
        print "PASSED "
    else:
        print testFailCount
        print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages

