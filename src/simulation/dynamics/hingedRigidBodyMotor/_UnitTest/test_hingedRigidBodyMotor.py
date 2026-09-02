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


# K [N m/rad], P [N m s/rad], angles [rad], rates [rad/s], uMax [N m], and expected torque [N m].
@pytest.mark.parametrize(
    "K, P, sensedTheta, sensedThetaDot, refTheta, refThetaDot, uMax, expectedTorque",
    [
        pytest.param(5.0, 1.0, 1.0, 0.1, 1.2, 0.2, None, 1.1, id="unlimited-positive"),
        pytest.param(5.0, 1.0, 1.0, 0.1, 0.8, -0.1, None, -1.2, id="unlimited-negative"),
        pytest.param(5.0, 1.0, 1.0, 0.1, 1.2, 0.2, 2.0, 1.1, id="limited-in-range"),
        pytest.param(5.0, 1.0, 1.0, 0.1, 1.2, 0.2, 0.5, 0.5, id="limited-positive"),
        pytest.param(5.0, 1.0, 1.0, 0.1, 0.8, -0.1, 0.5, -0.5, id="limited-negative"),
        pytest.param(5.0, 1.0, 1.0, 0.1, 1.2, 0.2, 0.0, 0.0, id="zero-limit"),
    ],
)
def test_hingedRigidBodyMotor(
    K, P, sensedTheta, sensedThetaDot, refTheta, refThetaDot, uMax, expectedTorque
):
    r"""
    **Validation Test Description**

    Verify that the module output matches the PD control law with torque saturation disabled and with a configured
    symmetric torque limit.  The cases exercise positive and negative saturation, a command inside the limit, and a
    zero limit.

    **Test Parameter Discussion**

    ``K`` and ``P`` are positive controller gains.  The sensed and reference hinge states produce both signs of raw
    torque command.  ``uMax`` is either left at its negative default, set above or below the raw command magnitude, or
    set to zero.

    **Description of Variables Being Tested**

    The first element of ``motorTorqueOutMsg.motorTorque`` is compared with the independently evaluated and clamped
    controller torque.
    """
    actualTorque = run_motor_case(
        K, P, sensedTheta, sensedThetaDot, refTheta, refThetaDot, uMax
    )
    assert actualTorque == pytest.approx(expectedTorque, rel=1e-12)


def run_motor_case(K, P, sensedTheta, sensedThetaDot, refTheta, refThetaDot, uMax):
    """Run one controller configuration and return its torque output."""
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)  # [ns]
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
    if uMax is not None:
        module.uMax = uMax

    # setup output message recorder objects
    dataLog = module.motorTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # [ns]
    unitTestSim.ExecuteSimulation()

    return dataLog.motorTorque[-1][0]


if __name__ == "__main__":
    torque = run_motor_case(5.0, 1.0, 1.0, 0.1, 1.2, 0.2, 0.5)
    assert torque == pytest.approx(0.5, rel=1e-12)
