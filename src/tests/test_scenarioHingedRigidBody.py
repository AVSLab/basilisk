#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated tutorial of the spacecraft(), gravity, and hinged rigid body modules illustrating
#           how Delta_v maneuver from test_scenarioOrbitManeuver.py affects the motion of the hinged rigid bodies.
#           Rotational motion is allowed on the spacecraft to simulate the full interaction of the hinged rigid
#           bodies and the spacecraft.
# Author:   Scott Carnahan
# Creation Date:  Jul. 17, 2017
#


import inspect
import os
import sys

import numpy as np
import pytest
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples')
import scenarioHingedRigidBody
@pytest.mark.scenarioTest

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
# The following 'parametrize' function decorator provides the parameters for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("doUnitTests, show_plots,", [(1, 0)])
def test_scenarioOrbitManeuver(doUnitTests, show_plots):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    velData, figureList = scenarioHingedRigidBody.run(show_plots)

    spaceCraftMomentum = np.sqrt(velData[-1, 0] ** 2 + velData[-1, 1] ** 2 + velData[-1, 2] ** 2)

    # setup truth data for unit test
    InstMomentum = 8470.84340921
    accuracy = 345.1819
    # compare the results to the truth values
    if abs(spaceCraftMomentum - InstMomentum) > accuracy:
        testFailCount += 1
        testMessages.append("Failed HingedRigidBody Tutorial test. Post-maneuver momentum incorrect.")

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("testFailCount: " + str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

