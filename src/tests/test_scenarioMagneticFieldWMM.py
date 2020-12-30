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
# Purpose:  Integrated test of the for the magnetic field tutorial script.
# Author:   Hanspeter Schaub
# Creation Date:  March 17, 2019
#

import sys, os, inspect
import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioMagneticFieldWMM


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitCase", ['circular', 'elliptical'])
@pytest.mark.scenarioTest

def test_scenarioMagneticField(show_plots, orbitCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    magData, figureList = scenarioMagneticFieldWMM.run(show_plots, orbitCase)

    numTruthPoints = 5
    skipValue = int(len(magData) / (numTruthPoints - 1))
    dataMagRed = magData[::skipValue]
    print(dataMagRed)

    # setup truth data for unit test
    trueMag = []
    if orbitCase == 'circular':
        trueMag = [
            [-9.8909713313380078e-06, -1.5750679233723167e-05, -3.9770938505865843e-05],
            [ 1.8137791136402835e-05,  1.3753603596811574e-05,  1.4627084437715579e-05],
            [-1.2437785307274045e-06, -2.3225641671126602e-05, -3.1154775849546575e-05],
            [ 2.0220096405005604e-05,  1.4951989103712117e-05,  7.0919234777338577e-06],
            [-1.3773821010930255e-05, -1.9420906806230346e-05, -3.6313624937603541e-05]
        ]
    if orbitCase == 'elliptical':
        trueMag = [
            [-2.9321026012579349e-06, -5.8651116571908833e-06, -1.2646241736266489e-05],
            [ 0.0000000000000000e+00,  0.0000000000000000e+00,  0.0000000000000000e+00],
            [ 0.0000000000000000e+00,  0.0000000000000000e+00,  0.0000000000000000e+00],
            [-1.3642620207349210e-06, -2.5553830162719547e-06, -3.6740465177531499e-07],
            [ 0.0000000000000000e+00,  0.0000000000000000e+00,  0.0000000000000000e+00]
        ]


    # compare the results to the truth values
    accuracy = 1e-8  # Tesla

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueMag, dataMagRed, accuracy, "magField_N Vector",
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
