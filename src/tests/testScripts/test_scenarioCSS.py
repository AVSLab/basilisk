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
# Purpose:  Demonstrates how to setup CSS sensors on a rigid spacecraft
# Author:   Hanspeter Schaub
# Creation Date:  July 21, 2017
#

import sys, os, inspect
import pytest
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioCSS


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useCSSConstellation, usePlatform, useEclipse, useKelly", [
      (False, False, False, False),
      (False, True, False, False),
      (False, False, True, False),
      (False, False, False, True),
      (True, False, False, False)
])
def test_bskAttitudeFeedback(show_plots, useCSSConstellation, usePlatform, useEclipse, useKelly):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    dataCSSArray, dataCSS1, dataCSS2, simulationTime, figureList \
        = scenarioCSS.run(show_plots, useCSSConstellation, usePlatform, useEclipse, useKelly)



    numTruthPoints = 5
    skipValue = int(simulationTime*macros.NANO2SEC/numTruthPoints)
    if useCSSConstellation:
        dataCSSArrayRed = dataCSSArray[::skipValue]
        trueCSS = [
              [ 2.0000000000000000e+00, 0.0000000000000000e+00]
            , [ 1.8270909152861943e+00, 8.1347328614937120e-01]
            , [ 1.3382612127209323e+00, 1.4862896509518926e+00]
            , [ 6.1803398875474258e-01, 1.9021130325887317e+00]
            , [ 0.0000000000000000e+00, 1.9890437907369840e+00]
            , [ 0.0000000000000000e+00, 1.7320508075693963e+00]
        ]

    else:
        dataCSS1red = dataCSS1[::skipValue]
        dataCSS2red = dataCSS2[::skipValue]

        # setup truth data for unit test
        if useEclipse == False and useKelly==False:
            trueCSS1 = [
                  [ 2.0000000000000000e+00]
                , [ 1.8270909152861943e+00]
                , [ 1.3382612127209323e+00]
                , [ 6.1803398875474258e-01]
                , [ 0.0000000000000000e+00]
                , [ 0.0000000000000000e+00]
            ]
            trueCSS2 = [
                  [ 0.0000000000000000e+00]
                , [ 8.1347328614937120e-01]
                , [ 1.4862896509518926e+00]
                , [ 1.9021130325887317e+00]
                , [ 1.9890437907369840e+00]
                , [ 1.7320508075693963e+00]
            ]
        if usePlatform == False and useEclipse == True and useKelly==False:
            trueCSS1 = [
                  [ 1.0000000000000000e+00]
                , [ 9.1354545764309714e-01]
                , [ 6.6913060636046617e-01]
                , [ 3.0901699437737129e-01]
                , [ 0.0000000000000000e+00]
                , [ 0.0000000000000000e+00]
            ]
            trueCSS2 = [
                  [ 0.0000000000000000e+00]
                , [ 4.0673664307468560e-01]
                , [ 7.4314482547594629e-01]
                , [ 9.5105651629436583e-01]
                , [ 9.9452189536849200e-01]
                , [ 8.6602540378469817e-01]
            ]
        if usePlatform == False and useEclipse == False and useKelly==True:
            trueCSS1 = [
                  [ 1.9865241060018290e+00]
                , [ 1.7989379186517085e+00]
                , [ 1.1956035766929121e+00]
                , [ 2.3463126305909773e-01]
                , [ 0.0000000000000000e+00]
                , [ 0.0000000000000000e+00]
            ]
            trueCSS2 = [
                  [ 0.0000000000000000e+00]
                , [ 4.5775481598362727e-01]
                , [ 1.3923439498033288e+00]
                , [ 1.8814534722288898e+00]
                , [ 1.9748891817681278e+00]
                , [ 1.6913168768673754e+00]
            ]
    # compare the results to the truth values
    accuracy = 1e-6

    if useCSSConstellation:
        testFailCount, testMessages = unitTestSupport.compareArrayND(
            trueCSS, dataCSSArrayRed, accuracy, "CSSarray", 2,
            testFailCount, testMessages)
    else:
        testFailCount, testMessages = unitTestSupport.compareDoubleArray(
            trueCSS1, dataCSS1red, accuracy, "CSS1",
            testFailCount, testMessages)
        testFailCount, testMessages = unitTestSupport.compareDoubleArray(
            trueCSS2, dataCSS2red, accuracy, "CSS2",
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

