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
from Basilisk.fswAlgorithms import hingedJointArrayMotor

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

    Mtt = M[0:3,0:3]
    Mtth = M[0:3,6:6+nj]
    Mtht = M[6:6+nj,0:3]
    Mthth = M[6:6+nj,6:6+nj]

    transBias = bias[0:3]
    jointBias = bias[6:6+nj]

    rhs = -Mtth @ theta_ddot_des + transBias

    x = np.linalg.solve(Mtt, rhs)

    u_H = Mthth @ theta_ddot_des + Mtht @ x - jointBias

    if maxTorque is not None:
        for i in range(nj):
            if u_H[i] > maxTorque[i]:
                u_H[i] = maxTorque[i]
            elif u_H[i] < -maxTorque[i]:
                u_H[i] = -maxTorque[i]

    return u_H

@pytest.mark.parametrize("nonActForces", [False, True])
@pytest.mark.parametrize("maxTorque", [False, True])
@pytest.mark.parametrize("numJoints", [1,4])
@pytest.mark.parametrize("numSpacecraft", [1, 2])

def test_hingedJointArrayMotor(nonActForces, maxTorque, numJoints, numSpacecraft):
    r"""
    **Validation Test Description**

    This unit test validates the motor torques found by the joint array controller for spacecraft with attached jointed arms.

    **Test Parameters**

    The presence of non actuator based forces, presence of max torque limits, number of joints, and number of spacecraft can be varied.

    Args:
        nonActForces (bool): Flag to indicate presence of non-actuator forces
        maxTorque (bool): Flag to indicate presence of max torque limits
        numJoints (int): Number of joints in the arm
        numSpacecraft (int): Number of spacecraft in the simulation

    **Description of Variables Being Tested**

    In this test the motor torques output by the module are compared to the expected torques calculated in python.
    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = hingedJointArrayMotor.HingedJointArrayMotor()
    module.ModelTag = "hingedJointArrayMotorTag"
    Ktheta = 10.0 * np.eye(numJoints * numSpacecraft)
    Ptheta = 2.0 * np.sqrt(10.0) * np.eye(numJoints * numSpacecraft)
    module.setKtheta(Ktheta.flatten().tolist())
    module.setPtheta(Ptheta.flatten().tolist())
    if maxTorque:
        uMax = [0.03] * (numJoints * numSpacecraft)
        module.setUMax(uMax)
    for sc in range(numSpacecraft):
        for j in range(numJoints):
            module.addHingedJoint()
    unitTestSim.AddModelToTask(unitTaskName, module)

    # create mass matrix input message
    M = np.eye((6 + numJoints) * numSpacecraft)
    massMatrixInMsgData = messaging.MJSysMassMatrixMsgPayload()
    massMatrixInMsgData.massMatrix.clear()
    for v in M.flatten():
        massMatrixInMsgData.massMatrix.push_back(v)

    # create the joint states input messages vectors
    if numJoints == 1:
        theta = [0.1]
        thetaDot = [0.5]
    elif numJoints == 4:
        theta = [0.1, -0.5, 0.3, 0.7]
        thetaDot = [0.5, -0.2, 0.1, -0.3]
    jointStatesInMsgDataList = []
    jointStateDotsInMsgDataList = []
    for sc in range(numSpacecraft):
        for j in range(numJoints):
            jointStatesInMsgData = messaging.ScalarJointStateMsgPayload()
            jointStatesInMsgData.state = theta[j]
            jointStatesInMsgDataList.append(jointStatesInMsgData)

            jointStateDotsInMsgData = messaging.ScalarJointStateMsgPayload()
            jointStateDotsInMsgData.state = thetaDot[j]
            jointStateDotsInMsgDataList.append(jointStateDotsInMsgData)

    # create the desired joint state input message
    desJointStateInMsgData = messaging.JointArrayStateMsgPayload()
    desTheta = [0.12] * (numJoints * numSpacecraft)
    desThetaDot = [0.05] * (numJoints * numSpacecraft)
    desJointStateInMsgData.states.clear()
    desJointStateInMsgData.stateDots.clear()
    for v in np.asarray(desTheta).flatten():
        desJointStateInMsgData.states.push_back(float(v))
    for v in np.asarray(desThetaDot).flatten():
        desJointStateInMsgData.stateDots.push_back(float(v))

    # create the joint reactions input message
    JointReactionsInMsgData = messaging.MJJointReactionsMsgPayload()
    jointTreeIdx = []
    jointTypes = []
    jointDOFStart = []
    currentDOF = 0
    for sc in range(numSpacecraft):
        idx = [sc] * (numJoints+1)
        jointTreeIdx.extend(idx)
        types = [0] + [3] * numJoints
        jointTypes.extend(types)
        jointDOFStart.append(currentDOF)
        currentDOF += 6
        for j in range(numJoints):
            jointDOFStart.append(currentDOF)
            currentDOF += 1
    if nonActForces:
        biasForces = [-2.0] * ((6 + numJoints) * numSpacecraft)
    else:
        biasForces = [0.0] * ((6 + numJoints) * numSpacecraft)
    passiveForces = [0.0] * ((6 + numJoints) * numSpacecraft)
    constraintForces = [0.0] * ((6 + numJoints) * numSpacecraft)
    appliedForces = [0.0] * ((6 + numJoints) * numSpacecraft)
    JointReactionsInMsgData.jointTreeIdx.clear()
    JointReactionsInMsgData.jointTypes.clear()
    JointReactionsInMsgData.jointDOFStart.clear()
    JointReactionsInMsgData.biasForces.clear()
    JointReactionsInMsgData.passiveForces.clear()
    JointReactionsInMsgData.constraintForces.clear()
    JointReactionsInMsgData.appliedForces.clear()
    for v in jointTreeIdx:
        JointReactionsInMsgData.jointTreeIdx.push_back(v)
    for v in jointTypes:
        JointReactionsInMsgData.jointTypes.push_back(v)
    for v in jointDOFStart:
        JointReactionsInMsgData.jointDOFStart.push_back(v)
    for v in biasForces:
        JointReactionsInMsgData.biasForces.push_back(v)
    for v in passiveForces:
        JointReactionsInMsgData.passiveForces.push_back(v)
    for v in constraintForces:
        JointReactionsInMsgData.constraintForces.push_back(v)
    for v in appliedForces:
        JointReactionsInMsgData.appliedForces.push_back(v)

    # write input messages
    massMatrixInMsg = messaging.MJSysMassMatrixMsg().write(massMatrixInMsgData)
    reactionForcesInMsg = messaging.MJJointReactionsMsg().write(JointReactionsInMsgData)
    desJointStateInMsg = messaging.JointArrayStateMsg().write(desJointStateInMsgData)
    jointStatesInMsgList = []
    jointStateDotsInMsgList = []
    for i in range(numJoints * numSpacecraft):
        jointStatesInMsg = messaging.ScalarJointStateMsg().write(jointStatesInMsgDataList[i])
        jointStatesInMsgList.append(jointStatesInMsg)
        jointStateDotsInMsg = messaging.ScalarJointStateMsg().write(jointStateDotsInMsgDataList[i])
        jointStateDotsInMsgList.append(jointStateDotsInMsg)

    # subscribe input messages to module
    module.massMatrixInMsg.subscribeTo(massMatrixInMsg)
    module.reactionForcesInMsg.subscribeTo(reactionForcesInMsg)
    module.desJointStatesInMsg.subscribeTo(desJointStateInMsg)
    for i in range(numJoints * numSpacecraft):
        module.jointStatesInMsgs[i].subscribeTo(jointStatesInMsgList[i])
        module.jointStateDotsInMsgs[i].subscribeTo(jointStateDotsInMsgList[i])

    # setup output message recorder objects
    motorTorqueRecorders = []
    for i in range(numJoints * numSpacecraft):
        rec = module.motorTorquesOutMsgs[i].recorder()
        motorTorqueRecorders.append(rec)
        unitTestSim.AddModelToTask(unitTaskName, rec)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.1))
    unitTestSim.ExecuteSimulation()

    # pull module data
    moduleTorques = []
    for rec in motorTorqueRecorders:
        moduleTorques.append(rec.input[-1])
    moduleTorques = np.asarray(moduleTorques, dtype=float).reshape(-1)

    # compute expected results
    reactionForces = np.asarray(biasForces, dtype=float).reshape(-1)
    expectedTorques = np.zeros(numJoints * numSpacecraft, dtype=float)
    for sc in range(numSpacecraft):
        startIdx = sc * (6 + numJoints)
        M_sc = M[startIdx:startIdx+6+numJoints, startIdx:startIdx+6+numJoints]
        K_sc = Ktheta[sc*numJoints:(sc+1)*numJoints, sc*numJoints:(sc+1)*numJoints]
        P_sc = Ptheta[sc*numJoints:(sc+1)*numJoints, sc*numJoints:(sc+1)*numJoints]
        theta_sc = theta[0:numJoints]
        thetaDot_sc = thetaDot[0:numJoints]
        desTheta_sc = desTheta[sc*numJoints:(sc+1)*numJoints]
        desThetaDot_sc = desThetaDot[sc*numJoints:(sc+1)*numJoints]
        bias_sc = -reactionForces[startIdx:startIdx+6+numJoints]

        if maxTorque:
            uMax_sc = uMax
        else:
            uMax_sc = None

        torques_sc = jointTorques(M_sc, K_sc, P_sc, theta_sc, thetaDot_sc,
                                  desTheta_sc, desThetaDot_sc, bias_sc, maxTorque=uMax_sc)
        expectedTorques[sc*numJoints:(sc+1)*numJoints] = torques_sc

    # Assert the motor torques are correct
    np.testing.assert_allclose(moduleTorques, expectedTorques, rtol=1e-8,
                               err_msg=f"Motor torques {moduleTorques} do not match expected values {expectedTorques}")

if __name__ == "__main__":
    test_hingedJointArrayMotor(True, False, 1, 1)
