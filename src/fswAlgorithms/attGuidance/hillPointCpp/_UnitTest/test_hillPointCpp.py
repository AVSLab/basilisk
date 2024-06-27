#
#  ISC License
#
#  Copyright (c) 2024 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import hillPointCpp  # import the module that is to be tested
from Basilisk.utilities import astroFunctions as af
from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass


@pytest.mark.parametrize("celMsgSet", [True, False])
def test_hillPointCpp(show_plots, celMsgSet):
    taskName = "unitTask"  # arbitrary name (don't change)
    processName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = sim.CreateNewProcess(processName)
    testProc.addTask(sim.CreateNewTask(taskName, testProcessRate))

    module = hillPointCpp.HillPointCpp()
    module.ModelTag = "hillPointCpp"

    # Add test module to runtime call list
    sim.AddModelToTask(taskName, module)

    # Initialize the test module configuration data
    a = af.E_radius * 2.8
    e = 0.0
    i = 0.0
    Omega = 0.0
    omega = 0.0
    f = 60 * af.D2R
    (r, v) = af.OE2RV(af.mu_E, a, e, i, Omega, omega, f)
    r_BN_N = r
    v_BN_N = v
    planetPos = np.array([0.0, 0.0, 0.0])
    planetVel = np.array([0.0, 0.0, 0.0])

    # Navigation Input Message
    navStateOutData = messaging.NavTransMsgPayload()  # Create a structure for the input message
    navStateOutData.r_BN_N = r_BN_N
    navStateOutData.v_BN_N = v_BN_N
    navMsg = messaging.NavTransMsg().write(navStateOutData)
    module.transNavInMsg.subscribeTo(navMsg)

    # Spice Input Message
    if celMsgSet:
        celBodyData = messaging.EphemerisMsgPayload()
        celBodyData.r_BdyZero_N = planetPos
        celBodyData.v_BdyZero_N = planetVel
        celBodyMsg = messaging.EphemerisMsg().write(celBodyData)
        module.celBodyInMsg.subscribeTo(celBodyMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.attRefOutMsg.recorder()
    sim.AddModelToTask(taskName, dataLog)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    sim.ExecuteSimulation()

    moduleOutput = dataLog.sigma_RN
    trueVector = [
        [0.0, 0.0, 0.267949192431],
        [0.0, 0.0, 0.267949192431],
        [0.0, 0.0, 0.267949192431]
    ]
    accuracy = 1e-12
    for i in range(0, len(trueVector)):
        np.testing.assert_allclose(trueVector[i],
                                   moduleOutput[i],
                                   atol=accuracy,
                                   verbose=True)

    moduleOutput = dataLog.omega_RN_N
    trueVector = [
        [0.0, 0.0, 0.000264539877],
        [0.0, 0.0, 0.000264539877],
        [0.0, 0.0, 0.000264539877]
    ]
    accuracy = 1e-12
    for i in range(0, len(trueVector)):
        np.testing.assert_allclose(trueVector[i],
                                   moduleOutput[i],
                                   atol=accuracy,
                                   verbose=True)

    moduleOutput = dataLog.domega_RN_N
    trueVector = [
        [0.0, 0.0, 1.315647475046e-23],
        [0.0, 0.0, 1.315647475046e-23],
        [0.0, 0.0, 1.315647475046e-23]
    ]
    accuracy = 1e-12
    for i in range(0, len(trueVector)):
        np.testing.assert_allclose(trueVector[i],
                                   moduleOutput[i],
                                   atol=accuracy,
                                   verbose=True)

    sim.ConfigureStopTime(macros.sec2nano(0.6))
    sim.ExecuteSimulation()


if __name__ == "__main__":
    test_hillPointCpp(False, True)
