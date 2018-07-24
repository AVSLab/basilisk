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


import sys, os, inspect
import pytest
import numpy as np
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../scenarios')
import scenarioFuelSlosh


@pytest.mark.parametrize("damping_parameter, timeStep", [
    (0.0, 0.75),
    (0.0, 0.3),
    (15.0, 0.75),
])
def test_scenarioFuelSlosh(show_plots, damping_parameter, timeStep):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    # provide a unique test method name, starting with test_
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    rhoj1Out, rhoj2Out, rhoj3Out, figureList = \
        scenarioFuelSlosh.run(show_plots, damping_parameter, timeStep)

    if damping_parameter != 0:

        zita1 = damping_parameter / (2 * np.sqrt(1500.0 * 1.0))
        omegan1 = np.sqrt(1.0 / 1500.0)
        settling_time1 = -1.0 / (zita1 * omegan1) * np.log(0.05 * np.sqrt(1 - zita1**2))
        index_settling_time1 = np.argmax(rhoj1Out[:, 0] * 1e-9 > settling_time1)

        zita2 = damping_parameter / (2 * np.sqrt(1400.0 * 1.0))
        omegan2 = np.sqrt(1.0 / 1400.0)
        settling_time2 = -1.0 / (zita2 * omegan2) * np.log(0.05 * np.sqrt(1 - zita2**2))
        index_settling_time2 = np.argmax(rhoj2Out[:, 0] * 1e-9 > settling_time2)

        zita3 = damping_parameter / (2 * np.sqrt(1300.0 * 1.0))
        omegan3 = np.sqrt(1.0 / 1300.0)
        settling_time3 = -1.0 / (zita3 * omegan3) * np.log(0.05 * np.sqrt(1 - zita3**2))
        index_settling_time3 = np.argmax(rhoj3Out[:, 0] * 1e-9 > settling_time3)

        accuracy = 0.05
        if abs(rhoj1Out[index_settling_time1, 1] - rhoj1Out[-1, 1]) > accuracy:
            testFailCount = testFailCount + 1
            testMessages = [testMessages, "Particle 1 settling time does not match second order systems theories"]
        if abs(rhoj2Out[index_settling_time2, 1] - rhoj2Out[-1, 1]) > accuracy:
            testFailCount = testFailCount + 1
            testMessages = [testMessages, "Particle 1 settling time does not match second order systems theories"]
        if abs(rhoj3Out[index_settling_time3, 1] - rhoj3Out[-1, 1]) > accuracy:
            testFailCount = testFailCount + 1
            testMessages = [testMessages, "Particle 1 settling time does not match second order systems theories"]

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in figureList.items():
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    if testFailCount == 0:
        print "PASSED "
    else:
        print testFailCount
        print testMessages

    return [testFailCount, ''.join(testMessages)]
