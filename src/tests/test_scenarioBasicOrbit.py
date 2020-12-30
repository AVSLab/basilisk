
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

sys.path.append(path + '/../../examples')
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
@pytest.mark.scenarioTest

def test_scenarioBasicOrbit(show_plots, orbitCase, useSphericalHarmonics, planetCase):
    """This function is called by the py.test environment."""
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
    if orbitCase == 'LEO' and useSphericalHarmonics == False and planetCase == 'Earth':
        truePos = [
            [-2.8168016010234966e6, 5.248174846916143e6, 3.6771572646772987e6],
            [-6.3710310400031125e6, -1.6053384413404597e6, 2.4169406797143915e6],
            [-1.970125344005881e6, -6.454584898598424e6, -1.8612676901068345e6],
            [4934518.68895958,   -3182978.5164827467, -3809969.2020971766]
        ]
    if orbitCase == 'GTO' and useSphericalHarmonics == False and planetCase == 'Earth':
        truePos = [
                      [-5889529.848066478,   9686574.89000767,          0.]
                    , [-32443422.719031237,  -4922427.414354751,         0.]
                    , [-36078086.41160842,  -19915150.791915383,         0.]
                    , [-28507259.223436695, -28944549.738026343,         0.]
        ]
    if orbitCase == 'GEO' and useSphericalHarmonics == False and planetCase == 'Earth':
        truePos = [
            [-21819784.817951124, 35887241.456518754, 0.],
            [-41428536.29954456, -6907093.740425735, 0.],
            [-8997262.092482397, -41025304.931358635, 0.],
            [34752956.740168475, -23584317.348184828, 0.]
        ]
    if orbitCase == 'LEO' and useSphericalHarmonics == True and planetCase == 'Earth':
        truePos = [
            [-2816801.6010234905, 5248174.8469161475, 3677157.2646772973],
            [5696634.592350598, 3924740.596447138, -1046156.0460872669],
            [2963860.9635370146, -5133282.1341001475, -3711547.9300516895],
            [-5637594.03464369, -4044595.6453490634, 917125.4175524816]
        ]
    if orbitCase == 'LEO' and useSphericalHarmonics == False and planetCase == 'Mars':
        truePos = [
            [-2816801.6010234905, 5248174.8469161475, 3677157.2646772973],
            [-6370345.93827535, -1614705.466882285, 2412504.0300785312],
            [-1930585.6802137129, -6460197.886362413, -1883087.252397967],
            [4937514.365486056, -3178775.7558852, -3809596.048901214]
        ]
    print(dataPosRed)
    # compare the results to the truth values
    accuracy = 1.0  # meters

    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPosRed, accuracy, "r_BN_N Vector",
        testFailCount, testMessages)

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

if __name__ == "__main__":
    test_scenarioBasicOrbit(
        False,        # show_plots
        'LEO',       # orbit Case (LEO, GTO, GEO)
        False,       # useSphericalHarmonics
        'Mars'      # planetCase (Earth, Mars)
    )