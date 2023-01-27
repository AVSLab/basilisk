# 
#  ISC License
# 
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import hingedRigidBodyMotor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("K", [5,10])
@pytest.mark.parametrize("P", [1,2])
@pytest.mark.parametrize("sensedTheta, sensedThetaDot, refTheta, refThetaDot", [
    (1,.1,1.2,.2),
    (1,.1,.8,-.1)
])
def test_hingedRigidBodyMotor(show_plots, K, P, sensedTheta, sensedThetaDot, refTheta, refThetaDot, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the output motor torque matches what is expected from the PD control law for the given reference and sensed hinged rigid body state.

    **Test Parameters**

    Args:
        K (double): Proportional gain value.
        P (double): Derivative gain value.
        sensedTheta (double): sensed theta value.
        sensedThetaDot (double): sensed thetaDot value.
        refTheta (double): reference theta value.
        refThetaDot (double): reference thetaDot value.
        accuracy (double): unit text accuracy

    **Description of Variables Being Tested**
    
    K and P are varied (note both must be set to positive values). The sensed hinged rigid body state is held constant while the reference is also varied to check positive and negative deltas.

    """
    [testResults, testMessage] = hingedRigidBodyMotorTestFunction(show_plots, K, P, sensedTheta, sensedThetaDot, refTheta, refThetaDot, accuracy)
    assert testResults < 1, testMessage


def hingedRigidBodyMotorTestFunction(show_plots, K, P, sensedTheta, sensedThetaDot, refTheta, refThetaDot, accuracy):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = hingedRigidBodyMotor.HingedRigidBodyMotor()
    module.ModelTag = "hingedRigidBodyMotorTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure blank module input messages
    hingedBodyStateSensedInMsgData = messaging.HingedRigidBodyMsgPayload()
    hingedBodyStateSensedInMsgData.theta = sensedTheta
    hingedBodyStateSensedInMsgData.thetaDot = sensedThetaDot
    hingedBodyStateSensedInMsg = messaging.HingedRigidBodyMsg().write(hingedBodyStateSensedInMsgData)

    hingedBodyStateReferenceInMsgData = messaging.HingedRigidBodyMsgPayload()
    hingedBodyStateReferenceInMsgData.theta = refTheta
    hingedBodyStateReferenceInMsgData.thetaDot = refThetaDot
    hingedBodyStateReferenceInMsg = messaging.HingedRigidBodyMsg().write(hingedBodyStateReferenceInMsgData)

    # subscribe input messages to module
    module.hingedBodyStateSensedInMsg.subscribeTo(hingedBodyStateSensedInMsg)
    module.hingedBodyStateReferenceInMsg.subscribeTo(hingedBodyStateReferenceInMsg)
    
    module.K = K
    module.P = P

    # setup output message recorder objects
    dataLog = module.motorTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    trueTorque = -K*(sensedTheta-refTheta)-P*(sensedThetaDot-refThetaDot)
    torqueEqualTest = unitTestSupport.isDoubleEqualRelative(trueTorque, dataLog.motorTorque[-1][0], accuracy)
    if not torqueEqualTest:
        testFailCount += 1;
        testMessages.append("Failed motor torque.")
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_hingedRigidBodyMotor(False, 5, 1, 1, .1, 1.2, .2, 1e-12) # first test case above


