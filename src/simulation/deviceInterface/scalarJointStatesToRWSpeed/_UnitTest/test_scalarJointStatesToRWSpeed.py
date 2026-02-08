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
from Basilisk.simulation import scalarJointStatesToRWSpeed
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


@pytest.mark.parametrize("numJoints", [1, 3, 8])
def test_scalarJointStatesToRWSpeed(numJoints: int):
    r"""
    **Verification Test Description**

    This unit test verifies that the ``ScalarJointStatesToRWSpeed`` adapter
    module correctly maps a set of ``ScalarJointStateMsgPayload`` input messages
    into a single ``RWSpeedMsgPayload`` output message using an order-preserving
    mapping.

    **Test Parameters**

    Args:
        numJoints (int): Number of scalar joint input messages.

    **Description of Variables Being Tested**

    Each element of the output reaction wheel speed vector is checked to ensure
    that it matches the corresponding joint angular rate provided in the input
    messages.
    """

    taskName = "unitTask"
    processName = "testProcess"
    testSim = SimulationBaseClass.SimBaseClass()

    timeStepSec = 0.1
    processRate = macros.sec2nano(timeStepSec)
    testProcess = testSim.CreateNewProcess(processName)
    testProcess.addTask(testSim.CreateNewTask(taskName, processRate))

    adapter = scalarJointStatesToRWSpeed.ScalarJointStatesToRWSpeed()
    adapter.ModelTag = "scalarJointStatesToRWSpeed"
    adapter.setNumJoints(numJoints)
    testSim.AddModelToTask(taskName, adapter)

    jointRates = [(jointIdx + 1) * 0.2 for jointIdx in range(numJoints)]

    msgs = []
    for jointIdx in range(numJoints):
        payload = messaging.ScalarJointStateMsgPayload(
            state=jointRates[jointIdx]
        )
        msg = messaging.ScalarJointStateMsg().write(payload)
        adapter.jointStateInMsgs[jointIdx].subscribeTo(msg)
        msgs.append(msg)

    testSim.InitializeSimulation()
    testSim.ConfigureStopTime(macros.sec2nano(timeStepSec))
    testSim.ExecuteSimulation()

    rwSpeedPayload = adapter.rwSpeedOutMsg.read()

    for jointIdx in range(numJoints):
        np.testing.assert_allclose(
            rwSpeedPayload.wheelSpeeds[jointIdx],
            jointRates[jointIdx],
            atol=1e-14,
            rtol=0.0
        )


if __name__ == "__main__":
    test_scalarJointStatesToRWSpeed(3)
