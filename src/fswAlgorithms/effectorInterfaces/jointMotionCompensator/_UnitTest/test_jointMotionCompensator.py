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
from Basilisk.fswAlgorithms import jointMotionCompensator

def compensationTorque(M, nonActForce, jointTorque, maxTorque=None):
    M = np.asarray(M, dtype=float)
    nonActForce = np.asarray(nonActForce, dtype=float).reshape(-1)
    jointTorque = np.asarray(jointTorque, dtype=float).reshape(-1)
    if maxTorque is not None:
        maxTorque = np.asarray(maxTorque, dtype=float).reshape(-1)
    nj = jointTorque.size

    baseTransBias = nonActForce[0:3]
    baseRotBias   = nonActForce[3:6]
    jointBias     = nonActForce[6:6+nj]

    u_H = jointTorque

    Mtt = M[0:3,0:3]
    Mrt = M[3:6,0:3]
    Mrth= M[3:6,6:6+nj]
    Mtth= M[0:3,6:6+nj]
    Mtht= M[6:6+nj,0:3]
    Mthth= M[6:6+nj,6:6+nj]

    # solve for theta_ddot and r_ddot using block matrix inversion
    temp1 = np.linalg.solve(Mtt, baseTransBias)
    temp2 = np.linalg.solve(Mtt, Mtth)
    temp3 = Mthth - Mtht @ temp2
    theta_ddot = np.linalg.solve(temp3, jointBias + u_H - (Mtht @ temp1))
    r_ddot = temp1 - (temp2 @ theta_ddot)

    tau_compensation = -baseRotBias + Mrt @ r_ddot + Mrth @ theta_ddot

    if maxTorque is not None:
        for i in range(3):
            if tau_compensation[i] > maxTorque[i]:
                tau_compensation[i] = maxTorque[i]
            elif tau_compensation[i] < -maxTorque[i]:
                tau_compensation[i] = -maxTorque[i]

    return tau_compensation

@pytest.mark.parametrize("nonActForces", [False, True])
@pytest.mark.parametrize("maxTorque", [False, True])
@pytest.mark.parametrize("numJoints", [1,4])
@pytest.mark.parametrize("numSpacecraft", [1, 2])

def test_jointMotionCompensator(nonActForces, maxTorque, numJoints, numSpacecraft):
    r"""
    **Validation Test Description**

    This unit test validates the hub torques found to counteract reaction torques induced by moving joints on spacecraft with attached jointed arms.

    **Test Parameters**

    The presence of non actuator based forces, presence of max torque limits, number of joints, and number of spacecraft can be varied.

    Args:
        nonActForces (bool): Flag to indicate presence of non-actuator forces
        maxTorque (bool): Flag to indicate presence of max torque limits
        numJoints (int): Number of joints in the arm
        numSpacecraft (int): Number of spacecraft in the simulation

    **Description of Variables Being Tested**

    In this test the hub torques output by the module are compared to the expected torques calculated in python.
    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = jointMotionCompensator.JointMotionCompensator()
    module.ModelTag = "jointMotionCompensatorTag"
    if maxTorque:
        uMax = [0.03] * numSpacecraft * 3
        module.setUMax(uMax)
    for _ in range(numSpacecraft):
        module.addSpacecraft()
        for _ in range(numJoints):
            module.addHingedJoint()
    unitTestSim.AddModelToTask(unitTaskName, module)

    # create mass matrix input message
    rng = np.random.default_rng(12345)
    M = np.zeros(((6 + numJoints) * numSpacecraft, (6 + numJoints) * numSpacecraft), dtype=float)
    for sc in range(numSpacecraft):
        startIdx = sc * (6 + numJoints)
        endIdx = startIdx + 6 + numJoints
        m = 10.0
        I = np.diag([8.0, 6.0, 4.0])
        A = rng.standard_normal((6 + numJoints, 6 + numJoints))
        SPD = A @ A.T + 1e-3 * np.eye(6 + numJoints)
        SPD[0:3,0:3] = m * np.eye(3)
        SPD[3:6,3:6] = I
        M[startIdx:endIdx, startIdx:endIdx] = SPD
    massMatrixInMsgData = messaging.MJSysMassMatrixMsgPayload()
    massMatrixInMsgData.massMatrix.clear()
    for v in M.flatten():
        massMatrixInMsgData.massMatrix.push_back(v)

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

    # create the joint torque input messages
    if numJoints == 1:
        motorTorque = [1.0]
    elif numJoints == 4:
        motorTorque = [1.0, 0.5, -0.5, -1.0]
    motorTorqueInMsgDataList = []
    for sc in range(numSpacecraft):
        for j in range(numJoints):
            jointTorqueInMsgData = messaging.SingleActuatorMsgPayload()
            jointTorqueInMsgData.input = motorTorque[j]
            motorTorqueInMsgDataList.append(jointTorqueInMsgData)

    # write input messages
    massMatrixInMsg = messaging.MJSysMassMatrixMsg().write(massMatrixInMsgData)
    reactionForcesInMsg = messaging.MJJointReactionsMsg().write(JointReactionsInMsgData)
    jointTorqueInMsgList = []
    for i in range(numSpacecraft * numJoints):
        jointTorqueInMsg = messaging.SingleActuatorMsg().write(motorTorqueInMsgDataList[i])
        jointTorqueInMsgList.append(jointTorqueInMsg)

    # subscribe input messages to module
    module.massMatrixInMsg.subscribeTo(massMatrixInMsg)
    module.reactionForcesInMsg.subscribeTo(reactionForcesInMsg)
    for i in range(numSpacecraft * numJoints):
        module.jointTorqueInMsgs[i].subscribeTo(jointTorqueInMsgList[i])

    # setup output message recorder objects
    hubTorqueRecorders = []
    for i in range(numSpacecraft * 3):
        rec = module.hubTorqueOutMsgs[i].recorder()
        hubTorqueRecorders.append(rec)
        unitTestSim.AddModelToTask(unitTaskName, rec)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.1))
    unitTestSim.ExecuteSimulation()

    # pull module data
    moduleTorques = []
    for rec in hubTorqueRecorders:
        moduleTorques.append(rec.input[-1])
    moduleTorques = np.asarray(moduleTorques, dtype=float).reshape(-1)

    # compute expected results
    reactionForces = np.asarray(biasForces, dtype=float).reshape(-1)
    expectedTorques = np.zeros(numSpacecraft * 3, dtype=float)
    for sc in range(numSpacecraft):
        startIdx = sc * (6 + numJoints)
        endIdx = startIdx + 6 + numJoints
        M_sc = M[startIdx:endIdx, startIdx:endIdx]
        reactionForces_sc = -reactionForces[startIdx:endIdx]
        jointTorque_sc = []
        for j in range(numJoints):
            jointTorque_sc.append(motorTorque[j])
        expectedTorque_sc = compensationTorque(M_sc, reactionForces_sc, jointTorque_sc,
                                              maxTorque=[0.03, 0.03, 0.03] if maxTorque else None)
        expectedTorques[sc*3:(sc+1)*3] = expectedTorque_sc

    # assert the results are as expected
    np.testing.assert_allclose(moduleTorques, expectedTorques, rtol=1e-8,
                               err_msg=f"Hub torques {moduleTorques} do not match expected {expectedTorques}")


if __name__ == "__main__":
    test_jointMotionCompensator(True, False, 1, 1)
