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
# Purpose:  Demonstration of how to setup and use different integrators in
#           Basilisk.  The simulation performs a 3-DOF orbit scenario.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 14, 2016
#

import sys, os, inspect
import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioIntegrators



# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Scott's brain no-worky\n")
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("integratorCase", ["rk4", "euler", "rk2"])
def test_scenarioIntegrators(show_plots, integratorCase):
    '''This function is called by the py.test environment.'''

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    # each test method requires a single assert method to be called
    posData, figureList = scenarioIntegrators.run(show_plots, integratorCase)


    numTruthPoints = 5
    skipValue = int(len(posData)/(numTruthPoints-1))
    dataPosRed = posData[::skipValue]

    # setup truth data for unit test
    if integratorCase is "rk4":
        truePos = [
              [-2.8168016010234915e6, 5.248174846916147e6, 3.677157264677297e6]
            , [-6.379381726549218e6, -1.4688565370540658e6, 2.4807857675497606e6]
            , [-2.230094305694789e6, -6.410420020364709e6, -1.7146277675541767e6]
            , [4.614900659014343e6, -3.60224207689023e6, -3.837022825958977e6]
            , [5.879095186201691e6, 3.561495655367985e6, -1.3195821703218794e6]
        ]
    if integratorCase is "euler":
        truePos = [
              [-2.8168016010234915e6, 5.248174846916147e6, 3.677157264677297e6]
            , [-7.061548530211288e6, -1.4488790844105487e6, 2.823580168201031e6]
            , [-4.831279689590867e6, -8.015202650472983e6, -1.1434851461593418e6]
            , [719606.5825106134, -1.0537603309084207e7, -4.966060248346598e6]
            , [6.431097055190775e6, -9.795566286964862e6, -7.438012269629238e6]
        ]
    if integratorCase is "rk2":
        truePos = [
              [-2.8168016010234915e6, 5.248174846916147e6, 3.677157264677297e6]
            , [-6.425636528569288e6, -1.466693214251768e6, 2.50438327358707e6]
            , [-2.466642497083674e6, -6.509473992136429e6, -1.6421621818735446e6]
            , [4.342561337924192e6, -4.1593822658140697e6, -3.947594705237753e6]
            , [6.279757158711852e6, 2.8527385905952943e6, -1.8260959147806289e6]
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

