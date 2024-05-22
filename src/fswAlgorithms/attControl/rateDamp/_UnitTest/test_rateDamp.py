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


from Basilisk.utilities import SimulationBaseClass
from Basilisk.fswAlgorithms import rateDamp  # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging


@pytest.mark.parametrize("P", [0.0, 2.0])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_rateDamp(show_plots, P, accuracy):

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    attControl = rateDamp.RateDamp()
    attControl.setRateGain(P)
    attControl.ModelTag = "rateDamp"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, attControl)

    # Initialize the test module configuration data
    # These will eventually become input messages

    omega_BN_B = np.array([0.1, -0.2, 0.3])

    # Create input navigation message
    NavAttMessageData = messaging.NavAttMsgPayload()
    NavAttMessageData.omega_BN_B = omega_BN_B
    NavAttMsg = messaging.NavAttMsg().write(NavAttMessageData)
    attControl.attNavInMsg.subscribeTo(NavAttMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = attControl.cmdTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    dataLogC = attControl.cmdTorqueOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogC)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    moduleOutput = dataLog.torqueRequestBody[0]
    moduleOutputC = dataLogC.torqueRequestBody[0]

    truth = -attControl.getRateGain() * omega_BN_B

    # set the filtered output truth states
    np.testing.assert_allclose(moduleOutput, truth, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(moduleOutputC, truth, rtol=0, atol=accuracy, verbose=True)

    return


if __name__ == "__main__":
    test_rateDamp(False, 1, 1e-12)
