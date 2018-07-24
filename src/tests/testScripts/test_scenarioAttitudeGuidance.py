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
#           MRP_Feedback() with attitude navigation modules.  Illustrates how
#           attitude guidance behavior can be changed in a very modular manner.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 2, 2016
#


import pytest
import os, sys
import inspect
import numpy as np

from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioAttitudeGuidance


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useAltBodyFrame", [False, True])
def test_bskAttitudeGuidance(show_plots, useAltBodyFrame):
    '''This function is called by the py.test environment.'''

    # provide a unique test method name, starting with test_
    # each test method requires a single assert method to be called
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataSigmaBN, numDataPoints, figureList = \
        scenarioAttitudeGuidance.run(show_plots, useAltBodyFrame)


    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataSigmaBNRed = dataSigmaBN[::skipValue]
    dataPosRed = dataPos[::skipValue]

    # setup truth data for unit test
    truePos = [
        [-3.9514176198221971e+06, 7.3621552027224889e+06, 5.1583270902798297e+06],
        [-4.6086027653051550e+06, 6.9672123694011206e+06, 5.3072237697180342e+06],
        [-5.2376104604200358e+06, 6.5296380777946832e+06, 5.4236570091631645e+06],
        [-5.8353304991197446e+06, 6.0530297355945092e+06, 5.5076788366235951e+06],
        [-6.3989830900466563e+06, 5.5410585900721485e+06, 5.5595354088057131e+06]
    ]
    trueSigmaBN = []
    if useAltBodyFrame is True:
        trueSigmaBN = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01],
            [-6.4143845119742271e-01, 3.7549202067008880e-01, 1.6228422035818663e-01],
            [-8.2514275559858030e-01, 3.7431052486815464e-01, 2.6641953651279665e-01],
            [-8.0514621677426934e-01, 3.3744944030160068e-01, 2.4586406789433021e-01],
            [-8.1316266101544810e-01, 3.0421565940809858e-01, 2.4203891375413897e-01]
        ]
    if useAltBodyFrame is False:
        trueSigmaBN = [
            [1.0000000000000001e-01, 2.0000000000000001e-01, -2.9999999999999999e-01],
            [1.9757381842655744e-01, -1.8325113332909412e-02, 5.3116118128700796e-01],
            [1.9401404616468543e-01, -6.2047093744322206e-02, 6.2244720069697612e-01],
            [1.9788907419526672e-01, -6.8298668119320893e-02, 6.4548524709461186e-01],
            [1.9984147378665409e-01, -7.7874650384175126e-02, 6.7169950976963932e-01]
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



