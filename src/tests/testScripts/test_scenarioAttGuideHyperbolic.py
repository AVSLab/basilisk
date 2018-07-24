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
# Purpose:  Integrated test of the spacecraftPlus(), extForceTorque, simpleNav(),
#           MRP_Feedback() with attitude navigation modules.  This script is a
#           spinoff from the attitude guidance tutorial, it implements a hyperbolic
#           trajectory and uses the velocityPoint module.
# Author:   Anne Bennett
# Creation Date:  Aug. 28th, 2017
#

import inspect
import os, sys
import pytest

from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioAttGuideHyperbolic




# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useAltBodyFrame", [False, True])

# provide a unique test method name, starting with test_
def test_bskAttGuide_Hyperbolic(show_plots, useAltBodyFrame):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataSigmaBN, numDataPoints, figureList = \
        scenarioAttGuideHyperbolic.run(show_plots, useAltBodyFrame)



    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataSigmaBNRed = dataSigmaBN[::skipValue]
    dataPosRed = dataPos[::skipValue]

    # setup truth data for unit test
    truePos = [
        [3.6223376821150966e+07, 7.1776505575846523e+07, 1.3687819378018096e+07],
        [3.5873290076594226e+07, 7.2092075260881290e+07, 1.3997417901516432e+07],
        [3.5522532862572916e+07, 7.2406297570323750e+07, 1.4306754823209373e+07],
        [3.5171116051793166e+07, 7.2719175431216419e+07, 1.4615826100429773e+07],
        [3.4819050453380756e+07, 7.3030711891436249e+07, 1.4924627774549646e+07]
    ]

    trueLr = trueSigmaBR = []
    if useAltBodyFrame is True:
        trueSigmaBN = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01],
            [-9.2494162977495867e-02, 1.9471395865807911e-01, -6.3717384535805643e-01],
            [-8.4160284482831221e-02, 1.8751522022305400e-01, -6.2862018070118753e-01],
            [-8.3717220192117484e-02, 1.8793830908990347e-01, -6.2761281563466287e-01],
            [-8.3427503754355453e-02, 1.8790862092331320e-01, -6.2675005457853550e-01]
        ]
    if useAltBodyFrame is False:
        trueSigmaBN = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01],
            [1.3870159058177514e-01, 6.5242458655457275e-02, 2.1071408452248369e-01],
            [1.3927887967605357e-01, 6.2240967986042707e-02, 2.0898043796751192e-01],
            [1.3967975559519039e-01, 6.2219318146119917e-02, 2.0946440039329009e-01],
            [1.3978125300497049e-01, 6.2060435748053963e-02, 2.1011602986235331e-01]
        ]
    # compare the results to the truth values
    accuracy = 1e-6

    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPosRed, accuracy, "r_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueSigmaBN, dataSigmaBNRed, accuracy, "sigma_BN Set",
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


