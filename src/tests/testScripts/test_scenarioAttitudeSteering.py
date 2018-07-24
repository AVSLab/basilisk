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

    dataPos, dataUsReq, dataSigmaBR, numDataPoints, figureList = \
        scenarioAttitudeSteering.run(show_plots, simCase)


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
            , [4.7189430944356903e-02, -3.0073466223710051e-02, -2.9519112173765433e-02]
            , [1.7017393221924075e-02, -1.5816980619665775e-02, 1.7044198770450832e-03]
            , [1.0462226562586911e-02, -1.3835610900036455e-02, 4.7069960207056401e-03]
            , [7.9429231563465721e-03, -1.4225784753257699e-02, 5.0252484301120453e-03]
        ]
        trueSigmaBR = [
            [-7.2833461132468502e-01, 2.2697297495054194e-01, 5.0137002864322056e-01]
            , [-1.3376220078206866e-01, 2.5642982824822001e-02, 9.9893437505902988e-02]
            , [-2.6596722500433580e-02, 5.4175180934137234e-03, 2.0187430996142657e-02]
            , [-5.6261093572915885e-03, 1.1492244793915528e-03, 4.2751758351042791e-03]
            , [-1.1939193784911433e-03, 2.4390270333934458e-04, 9.0728974314283311e-04]
        ]
    if simCase == 1:
        trueUs = [
            [-9.3735248485571654e-01, 3.1657897125315637e-01, 9.4810829234824123e-01]
            , [4.7579069035722919e-02, -3.0350173015691886e-02, -3.0759687838562672e-02]
            , [1.7238539200207487e-02, -1.5955025767513092e-02, 1.6062225704305648e-03]
            , [1.0527974506305259e-02, -1.3845379734132237e-02, 4.7102045984043937e-03]
            , [7.9728224611463980e-03, -1.4219725192432723e-02, 5.0553331993023450e-03]
        ]
        trueSigmaBR = [
            [-7.2833461132468502e-01, 2.2697297495054194e-01, 5.0137002864322056e-01]
            , [-1.3680796469746681e-01, 2.4498057497325483e-02, 1.0331789069873951e-01]
            , [-2.6057624568875816e-02, 4.2034815962751302e-03, 2.1430719862692515e-02]
            , [-4.4368206252905053e-03, -2.0503038156657912e-04, 5.0638152689513266e-03]
            , [1.1723050228483424e-04, -1.1640087921737483e-03, 1.5885872975580151e-03]
        ]
    if simCase == 2:
        trueUs = [
            [5.3427982784745387e-01, 2.4950294126525017e+00, 2.5650481888590777e+00]
            , [5.4535038442571349e-04, -8.5847664663658008e-03, 1.1506995960148571e-03]
            , [2.5451268231140300e-03, 9.9209273438078185e-03, -1.4089786225525037e-03]
            , [-4.5147004490431186e-04, 2.0507889350771857e-02, 1.9867017009794555e-03]
            , [3.1308591060124069e-03, -5.2529942101931404e-02, -1.6166923093728809e-03]
        ]
        trueSigmaBR = [
            [5.9238063851049559e-03,1.5059956700233917e-01, 1.6414145977723277e-01]
            , [-3.6813496549612653e-03, -1.2657386279747327e-03, -4.4566147389672021e-05]
            , [-3.7277383354709500e-03, 1.1163341331501285e-04, -4.6747777009811916e-06]
            , [-3.7353609790765695e-03, 1.1369833573167945e-03, 4.2187051788826694e-05]
            , [-3.6762767044320912e-03, -1.8795061282052812e-03, -6.9734000719422047e-05]
        ]
    if simCase == 3:
        trueUs = [[1.5463342442514050e+00, 2.4989188277446774e+00, 2.5666604392831682e+00]
            , [8.7951706496990634e-03, -2.2404760703769417e+00, -2.0368540337080115e-03]
            , [1.2942351409611938e-03, -9.0627675695472082e-04, 1.4686988294372436e-06]
            , [1.2175562291945774e-03, -9.9918587342460487e-04, 1.5678126695029573e-06]
            , [1.1342525284651131e-03, -1.0892870544848576e-03, 1.5413848543380973e-06]
        ]
        trueSigmaBR = [
            [5.9238063851049559e-03, 1.5059956700233917e-01, 1.6414145977723277e-01]
            , [1.0823033276500025e-04, -9.9720976722507976e-03, -9.6100659296812272e-05]
            , [6.3761494671625873e-08, 2.8199868851713313e-06, -4.9508867776145111e-07]
            , [4.0258863129135774e-09, 4.5877371818153005e-08, -9.8222037103927326e-09]
            , [1.1457920382880396e-10, 7.4465632405103127e-10, -1.8927251639829800e-10]
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

