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
#           MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in deep space.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 19, 2016
#

import pytest
import os, sys
import inspect


from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioAttitudePointing



# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useLargeTumble", [False, True])
def test_bskAttitudePointing(show_plots, useLargeTumble):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataLr, dataSigmaBR, numDataPoints, figureList = \
        scenarioAttitudePointing.run(show_plots, useLargeTumble)


    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataLrRed = dataLr[::skipValue]
    dataSigmaBRRed = dataSigmaBR[::skipValue]

    trueLr = trueSigmaBR = []

    if useLargeTumble is True:
        trueLr = [
            [-2.4350000000000001e+01, 1.7300000000000001e+01, -1.3949999999999999e+01],
            [-1.8261025223247096e-01, -1.9802131477666673e-01, -2.2905552303763882e-01],
            [-2.2703347936179175e-02, 2.8322384043503845e-02, -7.5383083954013580e-03],
            [3.9685083651031109e-03, -4.6726997381575461e-03, 9.1702648415809018e-04],
            [-6.5254418265915193e-04, 6.1478222187531318e-04, -6.2014699070663979e-05]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01],
            [1.5260971061679154e-01, -2.6346123607709682e-01, 1.9116787137307839e-01],
            [-1.8707224538059040e-02, 1.9073543274306739e-02, -9.2763187341566734e-03],
            [2.1576458281319776e-03, -1.5090989414394025e-03, 3.2041623116321299e-04],
            [-2.4360871626616175e-04, 1.0266828769375566e-04, -7.5369979791638928e-06]
        ]
    if useLargeTumble is False:
        trueLr = [
            [-3.8000000000000000e-01, -4.0000000000000008e-01, 1.5000000000000013e-01],
            [4.3304295406265583e-02, 7.7970819853086931e-03, 1.2148680350980004e-02],
            [-4.8427756513968068e-03, 2.6583725254198179e-04, -1.4133980386514184e-03],
            [5.2386812124888282e-04, -1.3752464748227947e-04, 8.9786880165438401e-05],
            [-5.3815258259032201e-05, 2.3975789622333814e-05, -4.5666337024320216e-06]
        ]
        trueSigmaBR = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01]
            , [-1.7700318439403492e-02, -1.4154347776578310e-02, 1.2434108941675513e-02]
            , [2.3210853655701645e-03, 1.3316275028241674e-03, -4.1569615433473430e-04]
            , [-3.0275893560215703e-04, -1.1614876733451711e-04, 8.6068784583440090e-06]
            , [3.9002194932293482e-05, 9.3813814117398300e-06, 1.5011853130355206e-07]
        ]

    # compare the results to the truth values
    accuracy = 1e-6

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueLr, dataLrRed, accuracy, "Lr Vector",
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

