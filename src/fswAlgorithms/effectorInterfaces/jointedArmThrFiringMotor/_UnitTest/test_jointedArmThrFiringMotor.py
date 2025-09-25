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
from numpy.testing import assert_allclose
import pytest

from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import jointedArmThrFiringMotor

def jointTorques(M, nbase, nj, bias, thrFT, maxTorque=None):
    M = np.asarray(M, dtype=float)
    bias = np.asarray(bias, dtype=float).reshape(-1)
    thrFT = np.asarray(thrFT, dtype=float).reshape(-1)

    if nbase==0:
        u_H = -bias - thrFT

    else:
        Mbase = M[0:nbase,0:nbase]
        Mtht = M[nbase:nbase+nj,0:3]
        Mthr = M[nbase:nbase+nj,3:6]

        baseBias = bias[0:nbase]
        baseThr = thrFT[0:nbase]
        jointBias = bias[nbase:nbase+nj]
        jointThr = thrFT[nbase:nbase+nj]

        baseAccel = np.linalg.solve(Mbase, (baseBias+baseThr))

        u_H = -(Mtht @ baseAccel[0:3] + Mthr @ baseAccel[3:6]) - jointBias - jointThr

    if maxTorque is not None:
        lim = np.asarray(maxTorque, dtype=float).reshape(-1)
        u_H = np.clip(u_H, -lim, lim)

    return u_H

@pytest.mark.parametrize("nonActForces", [False, True])
@pytest.mark.parametrize("maxTorque", [False, True])
@pytest.mark.parametrize("base", [False, True])
@pytest.mark.parametrize("numJoints", [1,4])

def test_jointedArmThrFiringMotor(nonActForces, maxTorque, base, numJoints):
    """Module Unit Test"""
    [testResults, testMessage] = jointedArmThrFiringMotorTestFunction(nonActForces, maxTorque, base, numJoints)
    assert testResults < 1, testMessage

def jointedArmThrFiringMotorTestFunction(nonActForces, maxTorque, base, numJoints):
    r"""
    **Validation Test Description**

    This unit test validates the motor torques found for a jointed arm when a thruster is firing.

    **Test Parameters**

    The number of joints for the arm, presence of non actuator based forces, and presence of max torque limits, and movability of the base
     can be varied.

    Args:
        nonActForces (bool): Flag to include non-actuator forces in the test
        maxTorque (bool): Flag to include maximum torque limits in the test
        base (bool): Flag to include base dynamics in the test
        numJoints (int): Number of joints to include in the test

    **Description of Variables Being Tested**

    In this test the motor torques output by the module are compared to the expected torques calculated in python.
    """
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = jointedArmThrFiringMotor.JointedArmThrFiringMotor()
    module.ModelTag = "jointedArmThrFiringMotorTag"
    if maxTorque:
        u_max = [0.0] * numJoints
        module.setU_max(u_max)
    unitTestSim.AddModelToTask(unitTaskName, module)
    # create the MassMatrix input message
    if base:
        nbase = 6
    else:
        nbase = 0
    nj = numJoints
    M = np.eye(nbase + nj)
    massMatrixInMsgData = messaging.MJSysMassMatrixMsgPayload()
    massMatrixInMsgData.nbase = nbase
    massMatrixInMsgData.nj = nj
    massMatrixInMsgData.MassMatrix = M

    # create the NonActuatorForces input message
    nonActForceInMsgData = messaging.MJNonActuatorForcesMsgPayload()
    if base:
        bias = np.zeros(nbase + nj)
        bias[0:3] = 2.0 if nonActForces else 0.0
        bias[nbase:] = 2.0 if nonActForces else 0.0
        nonActForceInMsgData.baseTransForces = bias[0:3].tolist()
        nonActForceInMsgData.jointForces = bias[nbase:].tolist()
    else:
        bias = np.full(nj, 2.0 if nonActForces else 0.0, dtype=float)
        nonActForceInMsgData.jointForces = bias.tolist()

    # create the CmdForceBody input message
    bodyForceInMsgData = messaging.CmdForceBodyMsgPayload()
    force_B = np.array([1,2,3])
    bodyForceInMsgData.forceRequestBody = force_B.tolist()

    # create the CmdTorqueBody input message
    bodyTorqueInMsgData = messaging.CmdTorqueBodyMsgPayload()
    torque_B = np.array([3,2,1])
    bodyTorqueInMsgData.torqueRequestBody = torque_B.tolist()

    # create the JointTorque input message
    jointTorqueInMsgData = messaging.ArrayMotorTorqueMsgPayload()
    torque_j = np.full(nj, 1.0, dtype=float)
    jointTorqueInMsgData.motorTorque = torque_j.tolist()

    # Configure blank module input messages
    massMatrixInMsg = messaging.MJSysMassMatrixMsg().write(massMatrixInMsgData)
    nonActForceInMsg = messaging.MJNonActuatorForcesMsg().write(nonActForceInMsgData)
    bodyForceInMsg = messaging.CmdForceBodyMsg().write(bodyForceInMsgData)
    bodyTorqueInMsg = messaging.CmdTorqueBodyMsg().write(bodyTorqueInMsgData)
    jointTorqueInMsg = messaging.ArrayMotorTorqueMsg().write(jointTorqueInMsgData)

    # subscribe input messages to module
    module.massMatrixInMsg.subscribeTo(massMatrixInMsg)
    module.nonActForceInMsg.subscribeTo(nonActForceInMsg)
    module.bodyForceInMsg.subscribeTo(bodyForceInMsg)
    module.bodyTorqueInMsg.subscribeTo(bodyTorqueInMsg)
    module.jointTorqueInMsg.subscribeTo(jointTorqueInMsg)

    # setup output message recorder objects
    jointTorqueOutMsgRec = module.jointTorqueOutMsg.recorder()
    jointTorqueOutMsgRecC = module.jointTorqueOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, jointTorqueOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, jointTorqueOutMsgRecC)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.1))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    modTorque = jointTorqueOutMsgRec.motorTorque[-1, 0:numJoints]
    modTorqueC = jointTorqueOutMsgRecC.motorTorque[-1, 0:numJoints]

    # calculate truth data
    thrFT = np.concatenate((force_B, torque_B, torque_j)) if base else torque_j
    truthTorque = jointTorques(M, nbase, nj, bias, thrFT, u_max if maxTorque else None)

    # compare the module results to the truth values
    try:
        assert_allclose(modTorque, truthTorque, rtol=1e-12, atol=1e-12)
    except AssertionError as err:
        testFailCount += 1
        testMessages.append(f"Joint Torques fail {err}\n")

    try:
        assert_allclose(modTorque, modTorqueC, rtol=1e-12, atol=1e-12)
    except AssertionError as err:
        testFailCount += 1
        testMessages.append(f"Joint Torque C/C++ msg not equal {err}\n")

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_jointedArmThrFiringMotor(False, False, False, 1)
