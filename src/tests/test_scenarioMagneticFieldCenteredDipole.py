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

sys.path.append(path + '/../examples')
import scenarioMagneticFieldCenteredDipole


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitCase, planetCase", [
    ('circular', 'Earth'),
    ('elliptical', 'Earth'),
    ('elliptical', 'Jupiter')
])
@pytest.mark.scenarioTest

def test_scenarioMagneticField(show_plots, orbitCase, planetCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    magData, figureList = scenarioMagneticFieldCenteredDipole.run(show_plots, orbitCase, planetCase)

    numTruthPoints = 5
    skipValue = int(len(magData) / (numTruthPoints - 1))
    dataMagRed = magData[::skipValue]

    # setup truth data for unit test
    trueMag = []
    if orbitCase == 'circular' and planetCase == 'Earth':
        trueMag = [
            [-6.6009426825634923e-06, -2.1606480803755902e-05, -3.7225617022490532e-05],
            [ 1.9571283124234556e-05,  1.4473114863968404e-05,  1.4637740465762316e-05],
            [-8.3563153727853268e-06, -2.3286702947772914e-05, -3.5069159314356784e-05],
            [ 2.1263624019782942e-05,  1.6068114169971413e-05,  1.2369095766805633e-05],
            [-9.9818049625740595e-06, -2.4792879600959000e-05, -3.2693447813550890e-05]
        ]
    if orbitCase == 'elliptical' and planetCase == 'Earth':
        trueMag = [
            [-2.1238634434031201e-06, -6.9519183738567534e-06, -1.1977399434331797e-05],
            [ 1.1391253004872550e-06,  8.7261808677234548e-07, -1.5194103813516944e-07],
            [ 2.5647444618706193e-07,  8.0307959125810588e-08,  7.4056345246373448e-07],
            [-1.5745250158167339e-06, -2.6860047342812571e-06, -3.4974650298810283e-07],
            [-7.0918520482901710e-06, -1.3705347575700211e-05, -1.0177610509651923e-05]
        ]
    if orbitCase == 'elliptical' and planetCase == 'Jupiter':
        trueMag = [
            [-5.4118082036659928e-05, -7.3309064488523109e-05, -1.7481547130195684e-04],
            [ 1.2443851358473244e-05,  1.4960617668135156e-05, -1.8639105517436840e-06],
            [ 4.9056456907319786e-07,  2.9201251508677655e-06,  1.0222789452821492e-05],
            [-3.0771050824857697e-05, -3.2476159775524712e-05, -8.3194247838254659e-06],
            [-1.2553066020901807e-04, -1.4753925529969713e-04, -1.6322348568320887e-04]
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

