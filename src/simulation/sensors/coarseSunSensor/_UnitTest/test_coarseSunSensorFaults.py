
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
# Coarse Sun Sensor Unit Test
#
# Purpose:  Test the proper function of the coarse sun sensor (css) module.
#           For basic functionality, results are compared to simple truth values calculated using np.cos().
#           For noise testing, noiseless truth values are subtracted from the output and the standard deviation is compared
#           to the input standard deviation.
#           For css constellation set up, two identical constellations are set up with different methods and compared to
#           each other
# Creation Date:  May. 31, 2017
#

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import coarseSunSensor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion as om

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize(
    "cssFault",
    [
        "CSSFAULT_OFF",
        "CSSFAULT_STUCK_CURRENT",
        "CSSFAULT_STUCK_MAX",
        "CSSFAULT_STUCK_RAND",
        "CSSFAULT_RAND",
    ],
)
# provide a unique test method name, starting with test_
def test_coarseSunSensor(cssFault):
    """Verify that each coarse sun sensor fault has the documented behavior."""
    outputs = run(cssFault)
    nominalOutput = outputs[0]
    faultOutputs = outputs[1:]

    if cssFault == "CSSFAULT_OFF":
        np.testing.assert_array_equal(faultOutputs, np.zeros(3))
    elif cssFault == "CSSFAULT_STUCK_CURRENT":
        np.testing.assert_allclose(faultOutputs, nominalOutput)
    elif cssFault == "CSSFAULT_STUCK_MAX":
        expectedOutput = 2.0  # [-] unit signal multiplied by the configured scale factor
        np.testing.assert_allclose(faultOutputs, expectedOutput)
    elif cssFault == "CSSFAULT_STUCK_RAND":
        np.testing.assert_allclose(faultOutputs, faultOutputs[0])
        assert not np.isclose(faultOutputs[0], nominalOutput)
    elif cssFault == "CSSFAULT_RAND":
        assert np.ptp(faultOutputs) > np.finfo(float).eps

    faultBound = 4.0  # [-] Gauss-Markov fault bound multiplied by the configured scale factor
    assert np.all(np.abs(faultOutputs) <= faultBound)


def run(cssFault):
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskPeriod = 0.1  # [s]
    testTaskRate = macros.sec2nano(testTaskPeriod)

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # unitTestSim.RNGSeed = 10

    # Ensure simulation is empty
    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    # Input Message Setup
    # Creates inputs from sun, spacecraft, and eclipse so that those modules don't have to be included
    # Create dummy sun message
    sunPositionMsg = messaging.SpicePlanetStateMsgPayload()
    sunPositionMsg.PositionVector = [om.AU * 1000.0, 0.0, 0.0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunPositionMsg)

    # Create dummy spacecraft message
    satelliteStateMsg = messaging.SCStatesMsgPayload()
    satelliteStateMsg.r_BN_N = [0.0, 0.0, 0.0]
    angle = np.pi / 16  # [rad]
    satelliteStateMsg.sigma_BN = [0., 0., angle]
    scMsg = messaging.SCStatesMsg().write(satelliteStateMsg)

    # Calculate sun distance factor
    CSS = coarseSunSensor.CoarseSunSensor()

    CSS.fov = 80.0 * macros.D2R  # [rad] half-angle field of view value
    CSS.scaleFactor = 2.0  # [-]
    CSS.nHat_B = np.array([1., 0., 0.])
    CSS.sunInMsg.subscribeTo(sunMsg)
    CSS.stateInMsg.subscribeTo(scMsg)
    CSS.ModelTag = "CSS"
    CSS.RNGSeed = 123
    unitTestSim.AddModelToTask(testTaskName, CSS)

    # log single CSS
    cssRecoder = CSS.cssDataOutMsg.recorder()
    unitTestSim.AddModelToTask(testTaskName, cssRecoder)

    cssFaultValue = getattr(coarseSunSensor, cssFault)

    unitTestSim.InitializeSimulation()

    # Execute the simulation for one time step
    unitTestSim.TotalSim.SingleStepProcesses()
    CSS.faultState = cssFaultValue
    for i in range(3):
        unitTestSim.TotalSim.SingleStepProcesses()

    return np.asarray(cssRecoder.OutputData)


if __name__ == "__main__":
    run("CSSFAULT_STUCK_MAX")
