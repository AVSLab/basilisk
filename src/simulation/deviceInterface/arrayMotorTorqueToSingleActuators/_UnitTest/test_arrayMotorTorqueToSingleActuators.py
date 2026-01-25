#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
from Basilisk.simulation import arrayMotorTorqueToSingleActuators
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


@pytest.mark.parametrize("numActuators", [1, 3, 8])
def test_arrayMotorTorqueToSingleActuators(numActuators: int):
    r"""
    **Verification Test Description**

    This unit test verifies that the ``ArrayMotorTorqueToSingleActuators`` adapter
    module correctly maps an ``ArrayMotorTorqueMsgPayload`` input message into a
    set of scalar ``SingleActuatorMsgPayload`` output messages using an
    order-preserving mapping.

    **Test Parameters**

    Args:
        numActuators (int): Number of scalar actuator output messages.

    **Description of Variables Being Tested**

    Each scalar actuator output message is checked to ensure that it matches the
    corresponding element of the input motor torque array.
    """

    taskName = "unitTask"
    processName = "testProcess"
    testSim = SimulationBaseClass.SimBaseClass()

    timeStepSec = 0.1
    processRate = macros.sec2nano(timeStepSec)
    testProcess = testSim.CreateNewProcess(processName)
    testProcess.addTask(testSim.CreateNewTask(taskName, processRate))

    adapter = arrayMotorTorqueToSingleActuators.ArrayMotorTorqueToSingleActuators()
    adapter.ModelTag = "arrayMotorTorqueToSingleActuators"
    adapter.setNumActuators(numActuators)
    testSim.AddModelToTask(taskName, adapter)

    torques = [(actIdx + 1) * 0.05 for actIdx in range(numActuators)]

    inputPayload = messaging.ArrayMotorTorqueMsgPayload(motorTorque=torques)
    inputMsg = messaging.ArrayMotorTorqueMsg().write(inputPayload)

    adapter.torqueInMsg.subscribeTo(inputMsg)

    testSim.InitializeSimulation()
    testSim.ConfigureStopTime(macros.sec2nano(timeStepSec))
    testSim.ExecuteSimulation()

    for actIdx in range(numActuators):
        cmdLast = adapter.actuatorOutMsgs[actIdx].read().input
        truth = torques[actIdx]
        np.testing.assert_allclose(cmdLast, truth, atol=1e-14, rtol=0.0)

if __name__ == "__main__":
    test_arrayMotorTorqueToSingleActuators(3)
