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
import scenarioFuelSlosh


@pytest.mark.parametrize("damping_parameter, timeStep", [
    (0.0, 0.75),
    (0.0, 0.3),
    (15.0, 0.75),
])
@pytest.mark.scenarioTest
def test_scenarioFuelSlosh(show_plots, damping_parameter, timeStep):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    time, rhojOuts, figureList = scenarioFuelSlosh.run(
        show_plots, damping_parameter, timeStep)

    if damping_parameter != 0:
        accuracy = 0.05
        mass = (1500, 1400, 1300)

        for i in range(3):
            zita = damping_parameter / (2 * np.sqrt(mass[i] * 1.0))
            omegan = np.sqrt(1.0 / mass[i])
            settling_time = -1.0 / (zita * omegan) * np.log(0.05 * np.sqrt(1 - zita**2))
            index_settling_time = np.argmax(time > settling_time)

            if abs(rhojOuts[i][index_settling_time] - rhojOuts[i][-1]) > accuracy:
                testFailCount = testFailCount + 1
                testMessages = [testMessages, f"Particle {i+1} settling time does "
                                "not match second order systems theories"]

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    assert testFailCount < 1, testMessages
