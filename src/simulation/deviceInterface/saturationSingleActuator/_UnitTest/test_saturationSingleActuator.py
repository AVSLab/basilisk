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
from Basilisk.simulation import saturationSingleActuator
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


@pytest.mark.parametrize("saturationEnabled", [False, True])
def test_saturationSingleActuator(saturationEnabled: bool):
    r"""
    **Verification Test Description**

    This unit test verifies that the ``SaturationSingleActuator`` module
    independently applies minimum and maximum limits when saturation is enabled,
    and passes commands through unchanged when saturation is disabled.
    """

    taskName = "unitTask"
    processName = "testProcess"
    testSim = SimulationBaseClass.SimBaseClass()

    timeStepSec = 0.1
    processRate = macros.sec2nano(timeStepSec)
    testProcess = testSim.CreateNewProcess(processName)
    testProcess.addTask(testSim.CreateNewTask(taskName, processRate))

    limiter = saturationSingleActuator.SaturationSingleActuator()
    limiter.ModelTag = "saturationSingleActuator"
    limiter.setSaturationEnabled(saturationEnabled)
    limiter.setMinInput(-0.2)
    limiter.setMaxInput(0.15)
    testSim.AddModelToTask(taskName, limiter)

    testValues = [-0.5, -0.2, -0.1, 0.0, 0.1, 0.15, 0.3]

    for value in testValues:
        payload = messaging.SingleActuatorMsgPayload()
        payload.input = value
        inMsg = messaging.SingleActuatorMsg().write(payload)

        limiter.actuatorInMsg.subscribeTo(inMsg)

        testSim.InitializeSimulation()
        testSim.ConfigureStopTime(macros.sec2nano(timeStepSec))
        testSim.ExecuteSimulation()

        outValue = limiter.actuatorOutMsg.read().input

        if saturationEnabled:
            truth = max(-0.2, min(value, 0.15))
        else:
            truth = value

        np.testing.assert_allclose(outValue, truth, atol=1e-14, rtol=0.0)


if __name__ == "__main__":
    test_saturationSingleActuator(True)
