
# ISC License
#
# Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.



#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test for the TAM sensor on a rigid spacecraft
# Author:   Demet Cilden-Guler
# Creation Date:  September 23, 2019
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
import scenarioTAM

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitCase, planetCase, useBias, useBounds", [
    ('circular', 'Earth', False, False),
    ('elliptical', 'Earth', True, False),
    ('circular', 'Jupiter', False, True),
    ('elliptical', 'Jupiter', False, False)
])
@pytest.mark.scenarioTest

def test_scenarioTAM(show_plots, orbitCase, planetCase, useBias, useBounds):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    magData, tamData, figureList, simulationTime = scenarioTAM.run(show_plots, orbitCase, planetCase, useBias, useBounds)

    numTruthPoints = 5
    skipValue = int(len(tamData) / (numTruthPoints - 1))
    dataMagRed = tamData[::skipValue]

    # setup truth data for unit test
    trueMag = []
    if orbitCase == 'circular' and planetCase == 'Earth' and useBias == False and useBounds == False:
        trueMag = [
            [-9.8909711711167910e-06, -1.5750678254527107e-05, -3.9770940590502404e-05],
            [1.8137775834150782e-05, 1.3753594278249663e-05, 1.4627065911256415e-05],
            [-1.2438328043293678e-06, -2.3225636857449733e-05, -3.1154667860968480e-05],
            [2.0220127543110147e-05, 1.4951978250984546e-05, 7.0917680337198594e-06],
            [-1.3773943421777615e-05, -1.9421019773673322e-05, -3.6313467519953167e-05]
        ]
    if orbitCase == 'elliptical' and planetCase == 'Earth' and useBias == True and useBounds == False:
        trueMag = [
            [-2.9321026358524342e-06, -5.8651112860140206e-06, -1.3646243717429426e-05],
            [0.0000000000000000e+00, 0.0000000000000000e+00, -9.9999999999999995e-07],
            [0.0000000000000000e+00, 0.0000000000000000e+00, -9.9999999999999995e-07],
            [-1.3642616233976047e-06, -2.5553818742773536e-06, -1.3674037556015926e-06],
            [0.0000000000000000e+00, 0.0000000000000000e+00, -9.9999999999999995e-07]
        ]
    if orbitCase == 'elliptical' and planetCase == 'Jupiter' and useBias == False and useBounds == False:
        trueMag = [
            [-5.4118082036659928e-05, -7.3309064488523122e-05, -1.7481547130195689e-04],
            [1.2443851358473241e-05, 1.4960617668135152e-05, -1.8639105517436789e-06],
            [4.9056456907319987e-07, 2.9201251508677668e-06, 1.0222789452821487e-05],
            [-3.0771050824857636e-05, -3.2476159775524664e-05, -8.3194247838253931e-06],
            [-1.2553066020901707e-04, -1.4753925529969626e-04, -1.6322348568320908e-04]
        ]
    if orbitCase == 'circular' and planetCase == 'Jupiter' and useBias == False and useBounds == True:
        trueMag = [
            [-1.6819836450589402e-04, -2.2784371297700405e-04, -3.5000000000000000e-04],
            [1.9468153473841567e-04, 2.7464566890817013e-04, 1.9285361028600469e-04],
            [-2.1437405607225573e-04, -2.7189272324396394e-04, -3.5000000000000000e-04],
            [2.3757943503534954e-04, 3.1431522854130238e-04, 1.2955048838291110e-04],
            [-2.5365042123908573e-04, -3.0686495730002382e-04, -3.5000000000000000e-04]
        ]

    # compare the results to the truth values
    accuracy = 1e-3  # Tesla

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueMag, dataMagRed, accuracy, "TAM Measurements",
        testFailCount, testMessages)

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("Failed: testFailCount is " + str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

