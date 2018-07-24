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
# Purpose:  Integrated test of the spacecraftPlus() and gravity modules.  Illustrates
#           a 3-DOV spacecraft on a range of orbit types.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 26, 2016
#

import sys, os, inspect
import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioBasicOrbit


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitCase, useSphericalHarmonics, planetCase", [
    ('LEO', False, 'Earth'),
    ('GTO', False, 'Earth'),
    ('GEO', False, 'Earth'),
    ('LEO', True, 'Earth'),
    ('LEO', False, 'Mars')
])
def test_scenarioBasicOrbit(show_plots, orbitCase, useSphericalHarmonics, planetCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    posData, figureList = scenarioBasicOrbit.run(show_plots, orbitCase, useSphericalHarmonics, planetCase)

    numTruthPoints = 5
    skipValue = int(len(posData) / (numTruthPoints - 1))
    dataPosRed = posData[::skipValue]

    # setup truth data for unit test
    truePos = []
    if orbitCase is 'LEO' and useSphericalHarmonics is False and planetCase is 'Earth':
        truePos = [
            [-2.8168016010234966e6, 5.248174846916143e6, 3.6771572646772987e6],
            [-6.3710310400031125e6, -1.6053384413404597e6, 2.4169406797143915e6],
            [-1.970125344005881e6, -6.454584898598424e6, -1.8612676901068345e6],
            [4.890526131271289e6, -3.2440700705588777e6, -3.815174368497354e6]
        ]
    if orbitCase is 'GTO' and useSphericalHarmonics is False and planetCase is 'Earth':
        truePos = [
            [-5.889529848066479e6, 9.686574890007671e6, 0.],
            [-3.2026565710377645e7, -4.305001879844011e6, 0.],
            [-3.624269187139845e7, -1.8990291195663467e7, 0.],
            [-2.9802077401931673e7, -2.831957848900475e7, 0.],
            [-1.4932981196798025e7, -2.939523308237971e7, 0.]
        ]
    if orbitCase is 'GEO' and useSphericalHarmonics is False and planetCase is 'Earth':
        truePos = [
            [-2.1819784817951165e7, 3.588724145651873e7, 0.],
            [-4.16996933506621e7, -5.016611324503355e6, 0.],
            [-1.2686252555573342e7, -4.0038573722578734e7, 0.],
            [3.1201815137542922e7, -2.8114754297243357e7, 0.],
            [3.850428014786283e7, 1.677456292503084e7, 0.]
        ]
    if orbitCase is 'LEO' and useSphericalHarmonics is True and planetCase is 'Earth':
        truePos = [
            [-2.8168016010234915e6, 5.248174846916147e6, 3.677157264677297e6],
            [5.787240887314784e6, 3.7547029876434486e6, -1.1653623184693705e6],
            [2.5908823579481775e6, -5.38042751586389e6, -3.6401355110844015e6],
            [-5.905673984221732e6, -3.5332208726054016e6, 1.2748483822117285e6],
            [-2.3741237403798397e6, 5.508156976353034e6, 3.6085612280591857e6]
        ]
    if orbitCase is 'LEO' and useSphericalHarmonics is False and planetCase is 'Mars':
        truePos = [
            [-2.8168016010234966e6, 5.248174846916143e6, 3.6771572646772987e6],
            [-6.370345938284969e6, -1.6147054668864955e6, 2.412504030081398e6],
            [-1.9520854768447054e6, -6.457181115789631e6, -1.8712382659451987e6],
            [4.90876381054031e6, -3.2188851633259663e6, -3.8130784005532693e6]
        ]

    # compare the results to the truth values
    accuracy = 1.0  # meters

    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPosRed, accuracy, "r_BN_N Vector",
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

