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
#           MRP_Feedback() modules.  Illustrates the spacecraft attitude dynamics while
#           in deep space.
# Author:   Hanspeter Schaub
# Creation Date:  Sept. 13, 2018
#

import pytest
import os, inspect, sys
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioAttitudeFeedbackNoEarth


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useUnmodeledTorque, useIntGain, useKnownTorque", [
    (False, False, False)
    , (True, False, False)
    , (True, True, False)
    , (True, False, True)
])
def test_bskAttitudeFeedbackNoEarth(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque):
    '''This function is called by the py.test environment.'''
    # provide a unique test method name, starting with test_

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # each test method requires a single assert method to be called
    dataPos, dataSigmaBR, dataLr, numDataPoints, figureList  \
        = scenarioAttitudeFeedbackNoEarth.run(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque)


    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataLrRed = dataLr[::skipValue]
    dataSigmaBRRed = dataSigmaBR[::skipValue]
    dataPosRed = dataPos[::skipValue]

    # setup truth data for unit test
    truePos = [
        [0.0, 0.0,0.0]
        , [0.0, 0.0,0.0]
        , [0.0, 0.0,0.0]
        , [0.0, 0.0,0.0]
        , [0.0, 0.0,0.0]
    ]
    trueLr = trueSigmaBR = []

    if useUnmodeledTorque == True and useIntGain == True and useKnownTorque == False:
        trueLr = [
            [-3.8540000000000002e-01, -3.5200000000000009e-01, 4.2000000000000121e-02]
            , [-2.3849697806730846e-01, 2.9471283787682012e-01, -1.3566545702259455e-01]
            , [-2.5271637424714444e-01, 2.3615511889142107e-01, -9.0488478286136861e-02]
            , [-2.4614222882341519e-01, 2.5067425482476591e-01, -9.8977162057449455e-02]
            , [-2.4977928591111503e-01, 2.4887615666172175e-01, -9.9881092081412159e-02]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
            , [2.2480494577949272e-02, -9.5531096658816039e-02, 7.0195707303957244e-02]
            , [2.2497104328479428e-02, -1.6693879988589459e-02, 2.1096813515555320e-02]
            , [5.5130423153784084e-03, -9.6647966447711703e-03, 5.2740482749995665e-03]
            , [1.9666952518230217e-03, -3.2953351361057178e-03, 2.7072233285654586e-03]
        ]
    if useUnmodeledTorque == True and useIntGain == False and useKnownTorque == False:
        trueLr = [
            [-3.8000000000000000e-01, -4.0000000000000008e-01, 1.5000000000000013e-01]
            , [-2.6249726949900559e-01, 2.5589984841560653e-01, -1.0917765643851718e-01]
            , [-2.4690643822771280e-01, 2.5434333217456701e-01, -9.8004180550054276e-02]
            , [-2.5063142579907921e-01, 2.4907026475177449e-01, -1.0026690856174182e-01]
            , [-2.4987503028354416e-01, 2.5015462259257626e-01, -9.9974532278102879e-02]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
            , [6.0923084226923496e-02, -9.2434292921154987e-02, 4.6511034647862895e-02]
            , [7.2857056494818079e-02, -6.9771873369940063e-02, 2.6850412008424970e-02]
            , [7.1220889524830688e-02, -7.1510953126517118e-02, 2.8814970926949179e-02]
            , [7.1456109973308091e-02, -7.1435046045892472e-02, 2.8534341637639557e-02]
        ]
    if useUnmodeledTorque == False and useIntGain == False and useKnownTorque == False:
        trueLr = [
            [-3.8000000000000000e-01, -4.0000000000000008e-01, 1.5000000000000013e-01]
            , [4.3304295406265583e-02, 7.7970819853086931e-03, 1.2148680350980004e-02]
            , [-4.8427756513968068e-03, 2.6583725254198179e-04, -1.4133980386514184e-03]
            , [5.2386812124888282e-04, -1.3752464748227947e-04, 8.9786880165438401e-05]
            , [-5.3815258259032201e-05, 2.3975789622333814e-05, -4.5666337024320216e-06]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
            , [-1.7700318439403492e-02, -1.4154347776578310e-02, 1.2434108941675513e-02]
            , [2.3210853655701645e-03, 1.3316275028241674e-03, -4.1569615433473430e-04]
            , [-3.0275893560215703e-04, -1.1614876733451711e-04, 8.6068784583440090e-06]
            , [3.9002194932293482e-05, 9.3813814117398300e-06, 1.5011853130355206e-07]
        ]
    if useUnmodeledTorque == True and useIntGain == False and useKnownTorque == True:
        trueLr = [
            [-6.3000000000000000e-01, -1.5000000000000008e-01, 5.0000000000000128e-02]
            , [-2.0669233648787125e-01, 2.5765561543404975e-01, -8.7857327347252573e-02]
            , [-2.5484394926182946e-01, 2.5027978077275026e-01, -1.0141318726877116e-01]
            , [-2.4947595336005560e-01, 2.4986116764798483e-01, -9.9910231437738084e-02]
            , [-2.5005384173665213e-01, 2.5002409166692252e-01, -1.0000456570945075e-01]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
            , [-1.7696313533933930e-02, -1.4143120263281624e-02, 1.2430697844911655e-02]
            , [2.3206734210692009e-03, 1.3296808140089579e-03, -4.1561415351098834e-04]
            , [-3.0275893560215703e-04, -1.1614876733451711e-04, 8.6068784583440090e-06]
            , [3.9002194932293482e-05, 9.3813814117398300e-06, 1.5011853130355206e-07]
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
        print testFailCount
        print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages

