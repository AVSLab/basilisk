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
from Basilisk.fswAlgorithms import jointedArmPDMotor

def jointTorques(M, K, P, theta, thetaDot, desTheta, desThetaDot, bias, maxTorque=None):
    M = np.asarray(M, dtype=float)
    K = np.asarray(K, dtype=float)
    P = np.asarray(P, dtype=float)
    theta      = np.asarray(theta, dtype=float).reshape(-1)
    thetaDot   = np.asarray(thetaDot, dtype=float).reshape(-1)
    desTheta   = np.asarray(desTheta, dtype=float).reshape(-1)
    desThetaDot= np.asarray(desThetaDot, dtype=float).reshape(-1)
    bias       = np.asarray(bias, dtype=float).reshape(-1)

    nj = theta.size
    e    = theta - desTheta
    edot = thetaDot - desThetaDot
    theta_ddot_des = -(K @ e) - (P @ edot)

    if M.shape == (nj,nj):
        u_H = M @ (theta_ddot_des) - bias

    else:
        Mtt = M[0:3,0:3]
        Mtth = M[0:3,6:6+nj]
        Mtht = M[6:6+nj,0:3]
        Mthth = M[6:6+nj,6:6+nj]

        transBias = bias[0:3]
        jointBias = bias[6:6+nj]

        rhs = Mtth @ theta_ddot_des + transBias

        x = np.linalg.solve(Mtt, rhs)

        u_H = Mthth @ theta_ddot_des - Mtht @ x + jointBias


    if maxTorque is not None:
        lim = np.asarray(maxTorque, dtype=float).reshape(-1)
        u_H = np.clip(u_H, -lim, lim)

    return u_H

@pytest.mark.parametrize("nonActForces", [False, True])
@pytest.mark.parametrize("maxTorque", [False, True])
@pytest.mark.parametrize("numJoints", [1,4])
@pytest.mark.parametrize("base", [False, True])

def test_jointedArmPDMotor(nonActForces, maxTorque, numJoints, base):
    """Module Unit Test"""
    [testResults, testMessage] = jointedArmPDMotorTestFunction(nonActForces, maxTorque, numJoints, base)
    assert testResults < 1, testMessage

def jointedArmPDMotorTestFunction(nonActForces, maxTorque, numJoints, base):
    r"""
    **Validation Test Description**

    This unit test validates the motor torques found by the PD controller for a jointed arm.

    **Test Parameters**

    The number of joints for the arm, presence of non actuator based forces, and presence of max torque limits can be varied.

    Args:
        nonActForces (bool): Flag to indicate presence of non-actuator forces
        maxTorque (bool): Flag to indicate presence of max torque limits
        numJoints (int): Number of joints in the arm

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
    module = jointedArmPDMotor.JointedArmPDMotor()
    module.ModelTag = "jointedArmPDMotorTag"
    Ktheta = 10.0 * np.eye(numJoints)
    Ptheta = 2.0 * np.sqrt(10.0) * np.eye(numJoints)
    module.setKtheta(Ktheta.flatten().tolist())
    module.setPtheta(Ptheta.flatten().tolist())
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

    # create the JointState input message
    jointStateInMsgData = messaging.JointArrayStateMsgPayload()
    if numJoints == 1:
        theta = [0.1]
        thetaDot = [0.5]
    elif numJoints == 4:
        theta = [0.1, -0.5, 0.3, 0.7]
        thetaDot = [0.5, -0.2, 0.1, -0.3]
    jointStateInMsgData.thetas = theta
    jointStateInMsgData.thetaDots = thetaDot

    # create the DesiredJointState input message
    desJointStateInMsgData = messaging.JointArrayStateMsgPayload()
    desTheta = [0.12] * numJoints
    desThetaDot = [0.05] * numJoints
    desJointStateInMsgData.thetas = desTheta
    desJointStateInMsgData.thetaDots = desThetaDot

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

    # write input messages
    massMatrixInMsg = messaging.MJSysMassMatrixMsg().write(massMatrixInMsgData)
    jointStateInMsg = messaging.JointArrayStateMsg().write(jointStateInMsgData)
    desJointStateInMsg = messaging.JointArrayStateMsg().write(desJointStateInMsgData)
    nonActForceInMsg = messaging.MJNonActuatorForcesMsg().write(nonActForceInMsgData)

    # subscribe input messages to module
    module.massMatrixInMsg.subscribeTo(massMatrixInMsg)
    module.jointStateInMsg.subscribeTo(jointStateInMsg)
    module.desJointStateInMsg.subscribeTo(desJointStateInMsg)
    module.nonActForceInMsg.subscribeTo(nonActForceInMsg)

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

    # calulate truth data
    truthTorque = jointTorques(M, Ktheta, Ptheta, theta, thetaDot, desTheta, desThetaDot, bias, u_max if maxTorque else None)

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
    test_jointedArmPDMotor(False, False, 1, False)
