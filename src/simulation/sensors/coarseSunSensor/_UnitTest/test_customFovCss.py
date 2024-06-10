#
#  ISC License
#
#  Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging
from Basilisk.simulation import coarseSunSensor
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass


@pytest.mark.parametrize("xi", [np.pi/6, np.pi/3, np.pi/2])
@pytest.mark.parametrize("eta", [np.pi/6, np.pi/3, np.pi/2])
@pytest.mark.parametrize("zeta", [np.pi/6, np.pi/3, np.pi/2])
@pytest.mark.parametrize("sunLocation", [0, 1, 2, 3])
@pytest.mark.parametrize("accuracy", [1e-10])
def test_customFovCss(show_plots, xi, eta, zeta, sunLocation, accuracy):

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    lHat_B = [1, 0, 0]
    mHat_B = [0, 1, 0]
    nHat_B = [0, 0, 1]

    # Construct algorithm and associated C++ container
    CSS = coarseSunSensor.CoarseSunSensor()
    CSS.ModelTag = "coarseSunSensor"
    CSS.scaleFactor = 1
    CSS.lHat_B = lHat_B
    CSS.mHat_B = mHat_B
    CSS.nHat_B = nHat_B
    CSS.customFov = True
    CSS.fovXi = xi
    CSS.fovEta = eta
    CSS.fovZeta = zeta

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, CSS)

    # Initialize the test module configuration data
    # These will eventually become input messages

    omega_BN_B = np.array([0.1, -0.2, 0.3])

    # Create input navigation message
    SCStatesMsgData = messaging.SCStatesMsgPayload()
    SCStatesMsgData.r_BN_N = [0, 0, 0]
    SCStatesMsgData.sigma_BN = [0, 0, 0]
    SCStatesMsg = messaging.SCStatesMsg().write(SCStatesMsgData)
    CSS.stateInMsg.subscribeTo(SCStatesMsg)

    x = 0
    y = 0
    match sunLocation:
        case 0:
            y = np.sin(xi - accuracy)
        case 1:
            x = np.sin(zeta - accuracy)
        case 2:
            y = -np.sin(eta - accuracy)
        case 3:
            x = -np.sin(zeta - accuracy)
    z = (1 - x*x - y*y)**0.5

    # Create input Sun spice msg
    SpicePlanetStateMsgData = messaging.SpicePlanetStateMsgPayload()
    SpicePlanetStateMsgData.PositionVector = astroFunctions.AU * 1000 * np.array([x, y, z])
    SpicePlanetStateMsg = messaging.SpicePlanetStateMsg().write(SpicePlanetStateMsgData)
    CSS.sunInMsg.subscribeTo(SpicePlanetStateMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = CSS.cssDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    moduleOutput = dataLog.OutputData[0]

    truth = np.dot(nHat_B, [x, y, z])

    # set the filtered output truth states
    np.testing.assert_allclose(moduleOutput, truth, rtol=0, atol=accuracy, verbose=True)

    return


if __name__ == "__main__":
    test_customFovCss(False, np.pi/6, np.pi/3, np.pi/2, 0, 1e-10)
