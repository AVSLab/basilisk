#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import numpy as np
import pytest

from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import thrFiringRound

@pytest.mark.parametrize("label, desired, expectedFirings", [
     ("roundUp", 0.005, 1),
     ("roundDown", 0.004999, 0),
])

def test_thrFiringRound(label, desired, expectedFirings):
    r"""
    **Validation Test Description**

    Test the thruster firing rounding algorithm.

    **Test Parameters**

    The requested firing time is varied to test the rounding behavior.

    Args:
        label (str): label for the specific test
        request (float): requested firing time in seconds
        expectedFirings (int): expected number of firings after rounding

    **Description of Variables Being Tested**

    Checking the output thruster firings against the expected number of firings.
    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    timeStep = 0.01
    testProcessRate = macros.sec2nano(timeStep)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = thrFiringRound.ThrFiringRound()
    module.ModelTag = "thrFiringRoundTag"
    thrForces = [2.0,2.0]
    module.setTimeStep(timeStep)
    module.setTHRForce(thrForces)
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure blank module input messages
    onTimeInMsgData = messaging.THRArrayOnTimeCmdMsgPayload()
    onTimeInMsgData.OnTimeRequest = [desired, desired]
    onTimeInMsg = messaging.THRArrayOnTimeCmdMsg().write(onTimeInMsgData)

    # subscribe input messages to module
    module.onTimeInMsg.subscribeTo(onTimeInMsg)

    # setup output message recorder objects
    thrForceOutMsgRec = module.thrForceOutMsg.recorder()
    thrForceOutMsgCRec = module.thrForceOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, thrForceOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, thrForceOutMsgCRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.01))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    thrForceOutMsgData = thrForceOutMsgRec.thrForce[0,:2]
    thrForceOutMsgCData = thrForceOutMsgCRec.thrForce[0,:2]

    # build the truth data
    thrForceTruth = np.array(thrForces) * expectedFirings

    # Assert thruster forces are correct
    assert thrForceOutMsgData == pytest.approx(thrForceTruth, rel=1e-8), \
        f"{label} thruster forces {thrForceOutMsgData} not close to expected {thrForceTruth}"

    # # Assert the C++ and C messages are identical
    assert thrForceOutMsgData == pytest.approx(thrForceOutMsgCData, rel=1e-8), \
        f"{label} C++ and C thruster forces {thrForceOutMsgData} and {thrForceOutMsgCData} not identical"




if __name__ == "__main__":
    test_thrFiringRound( "roundUp", 0.005, 1)
