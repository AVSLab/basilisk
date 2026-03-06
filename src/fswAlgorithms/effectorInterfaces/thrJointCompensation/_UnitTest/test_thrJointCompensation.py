#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
from Basilisk.fswAlgorithms import thrJointCompensation

def jointTorques(nSc, nArms, nJoints, M, theta, bias, thrust, armCfg, maxTorque=None):
    M = np.asarray(M, dtype=float)
    theta = np.asarray(theta, dtype=float).reshape(-1)
    bias = np.asarray(bias, dtype=float).reshape(-1)
    bias *= -1.0
    thrust = np.asarray(thrust, dtype=float).reshape(-1)

    nArmsTot = nSc * nArms
    nH = nArmsTot * nJoints
    nBase = 6
    r_CP_P, shat_P, dcm_C0P, r_TP_P, fhat_P = unpackCfgArrays(nSc, nArms, nJoints, armCfg)
    dcm_CB = np.zeros((nH, 3, 3), dtype=float)
    r_CB_B = np.zeros((nH,3), dtype=float)

    for sc in range(nSc):
        for arm in range(nArms):
            dcm_PB = np.eye(3)
            r_PB_B = np.zeros(3)
            for j in range(nJoints):
                h = sc*(nArms*nJoints) + arm*nJoints + j
                kFlat = h
                r = r_CP_P[kFlat]
                shat  = shat_P[kFlat]
                C_C0P = dcm_C0P[kFlat]
                phi = theta[j]
                C_CC0 = PRV2C(shat, phi)
                C_CP = C_CC0 @ C_C0P
                r_CB_B[h] = r_PB_B + dcm_PB.T @ r
                dcm_CB[h] = C_CP @ dcm_PB
                r_PB_B = r_CB_B[h]
                dcm_PB = dcm_CB[h]

    FBase = np.zeros((nSc,3))
    LBase = np.zeros((nSc,3))
    tauJoint = np.zeros(nH)
    for sc in range(nSc):
        for arm in range(nArms):
            thrIdx = sc*nArms + arm
            jLocal = nJoints - 1
            hAttach = sc*(nArms*nJoints) + arm*nJoints + jLocal

            C_CB = dcm_CB[hAttach]
            r = r_CB_B[hAttach]
            fhat = fhat_P[thrIdx]
            f_B = C_CB.T @ (fhat * thrust[thrIdx])
            r_TB_B = r + C_CB.T @ r_TP_P[thrIdx]
            FBase[sc] += f_B
            LBase[sc] += np.cross(r_TB_B, f_B)

            for j in range(jLocal + 1):
                hJ = sc*(nArms*nJoints) + arm*nJoints + j
                C_JB = dcm_CB[hJ]
                shat_B = C_JB.T @ shat_P[hJ]
                r_TJ_B = r_TB_B - r_CB_B[hJ]
                tauJoint[hJ] += np.dot(shat_B, np.cross(r_TJ_B, f_B))

    uH = np.zeros(nH)
    for sc in range(nSc):
        sc0 = sc*(nBase + nArms*nJoints)
        sc1 = sc0 + (nBase + nArms*nJoints)
        M_sc = M[sc0:sc1, sc0:sc1]
        bias_sc = bias[sc0:sc1]
        Mbase = M_sc[0:nBase, 0:nBase]
        Mthb  = M_sc[nBase:, 0:nBase]
        baseThr = np.zeros(nBase)
        baseThr[0:3] = FBase[sc]
        baseThr[3:6] = LBase[sc]
        baseBias = bias_sc[:nBase]
        baseAccel = np.linalg.solve(Mbase, baseThr + baseBias)
        jointBias = bias_sc[nBase:]
        tauThr_sc = tauJoint[sc*(nArms*nJoints):(sc+1)*(nArms*nJoints)]
        uH_sc = (Mthb @ baseAccel) - jointBias - tauThr_sc
        uH[sc*(nArms*nJoints):(sc+1)*(nArms*nJoints)] = uH_sc

    if maxTorque is not None:
        uMax = np.asarray(maxTorque, dtype=float).reshape(-1)
        uH = np.clip(uH, -uMax, uMax)

    return uH

def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])

def unpackCfgArrays(nSc, nArms, nJoints, armCfg):
    nArmsTot = nSc * nArms
    nFlat = nArmsTot * nJoints
    r_CP_P = np.asarray(armCfg["r_CP_P"], dtype=float).reshape(nFlat, 3)
    shat_P = np.asarray(armCfg["shat_P"], dtype=float).reshape(nFlat, 3)
    d = np.asarray(armCfg["dcm_C0P"], dtype=float).reshape(nFlat, 9)
    dcm_C0P = np.zeros((nFlat, 3, 3), dtype=float)
    for k in range(nFlat):
        c0 = d[k, 0:3]
        c1 = d[k, 3:6]
        c2 = d[k, 6:9]
        dcm_C0P[k, :, 0] = c0
        dcm_C0P[k, :, 1] = c1
        dcm_C0P[k, :, 2] = c2
    r_TP_P = np.asarray(armCfg["r_TP_P"], dtype=float).reshape(nArmsTot, 3)
    fhat_P = np.asarray(armCfg["fhat_P"], dtype=float).reshape(nArmsTot, 3)

    return r_CP_P, shat_P, dcm_C0P, r_TP_P, fhat_P

def PRV2C (ehat, phi):
    return np.eye(3)*np.cos(phi) - np.sin(phi)*skew(ehat) + (1-np.cos(phi))*np.outer(ehat, ehat)

def packVec(lst, v):
    lst.extend([float(v[0]), float(v[1]), float(v[2])])

def packDCM(lst, C):
    lst.extend([
        float(C[0][0]), float(C[1][0]), float(C[2][0]),
        float(C[0][1]), float(C[1][1]), float(C[2][1]),
        float(C[0][2]), float(C[1][2]), float(C[2][2])
    ])

@pytest.mark.parametrize("nonActForces", [False, True])
@pytest.mark.parametrize("maxTorque", [False, True])
@pytest.mark.parametrize("numJoints", [1, 4])
@pytest.mark.parametrize("numArms", [1, 2])
@pytest.mark.parametrize("numSpacecraft", [1, 2])

def test_thrJointCompensation(nonActForces, maxTorque, numJoints, numArms, numSpacecraft):
    r"""
    **Validation Test Description**

    This unit test validates the motor torques found by the thrJointCompensation controller for spacecraft with attached jointed arms.

    **Test Parameters**

    The presence of non actuator based forces, presence of max torque limits, number of joints, number of arms, and number of spacecraft can be varied.

    Args:
        nonActForces (bool): Flag to indicate presence of non-actuator forces
        maxTorque (bool): Flag to indicate presence of max torque limits
        numJoints (int): Number of joints in the arm
        numArms (int): Number of arms attached to the spacecraft
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
    module = thrJointCompensation.ThrJointCompensation()
    module.ModelTag = "thrJointCompensationTag"
    if maxTorque:
        uMax = [1e-6] * (numJoints * numArms * numSpacecraft)
        module.setUMax(uMax)
    else:
        uMax = None
    for _ in range(numSpacecraft):
        for _ in range(numArms):
            module.addThruster()
            for _ in range(numJoints):
                module.addHingedJoint()
    unitTestSim.AddModelToTask(unitTaskName, module)

    # create mass matrix input message
    rng = np.random.default_rng(12345)
    M = np.zeros(((6 + (numJoints*numArms)) * numSpacecraft, (6 + (numJoints*numArms)) * numSpacecraft), dtype=float)
    for sc in range(numSpacecraft):
        startIdx = sc * (6 + (numJoints*numArms))
        endIdx = startIdx + 6 + (numJoints*numArms)
        m = 10.0
        I = np.diag([8.0, 6.0, 4.0])
        dofSC = 6 + (numJoints*numArms)
        A = rng.standard_normal((dofSC, dofSC))
        SPD = A @ A.T + 1e-3 * np.eye(dofSC)
        SPD[0:3,0:3] = m * np.eye(3)
        SPD[3:6,3:6] = I
        M[startIdx:endIdx, startIdx:endIdx] = SPD
    massMatrixInMsgData = messaging.MJSysMassMatrixMsgPayload()
    massMatrixInMsgData.massMatrix.clear()
    for v in M.flatten():
        massMatrixInMsgData.massMatrix.push_back(v)

    # create the joint states input messages vectors
    if numJoints == 1:
        theta = [0.1]
    elif numJoints == 4:
        theta = [0.1, -0.5, 0.3, 0.7]
    jointStatesInMsgDataList = []
    for _ in range(numSpacecraft):
        for _ in range(numArms):
            for j in range(numJoints):
                jointStatesInMsgData = messaging.ScalarJointStateMsgPayload()
                jointStatesInMsgData.state = theta[j]
                jointStatesInMsgDataList.append(jointStatesInMsgData)

    # create the joint reactions input message
    JointReactionsInMsgData = messaging.MJJointReactionsMsgPayload()
    jointTreeIdx = []
    jointTypes = []
    jointDOFStart = []
    currentDOF = 0
    for sc in range(numSpacecraft):
        idx = [sc] * ((numJoints*numArms)+1)
        jointTreeIdx.extend(idx)
        types = [0] + [3] * (numJoints*numArms)
        jointTypes.extend(types)
        jointDOFStart.append(currentDOF)
        currentDOF += 6
        for j in range(numJoints*numArms):
            jointDOFStart.append(currentDOF)
            currentDOF += 1
    if nonActForces:
        biasForce = -0.2
        biasForces = (biasForce * np.arange(1, (6 + (numJoints*numArms)) * numSpacecraft + 1, dtype=float)).tolist()
    else:
        biasForces = [0.0] * ((6 + (numJoints*numArms)) * numSpacecraft)
    passiveForces = [0.0] * ((6 + (numJoints*numArms)) * numSpacecraft)
    constraintForces = [0.0] * ((6 + (numJoints*numArms)) * numSpacecraft)
    appliedForces = [0.0] * ((6 + (numJoints*numArms)) * numSpacecraft)
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

    # create the spacecraft configuration input message
    axisPattern = [[1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [1.0, 0.0, 0.0]]
    linkPattern = [[0.5, 0.0, 0.0],
                   [0.0, 0.0, 0.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0]]
    armConfigInMsgData = messaging.THRArmConfigMsgPayload()
    thrArmIdx = []
    thrArmJointIdx = []
    armTreeIdx = []
    armJointCount = []
    r_CP_P = []
    r_TP_P = []
    shat_P = []
    fhat_P = []
    dcm_C0P = []
    for sc in range(numSpacecraft):
        for arm in range(numArms):
            thrArmIdx.append(sc*numArms+arm)
            thrArmJointIdx.append(numJoints - 1)
            armTreeIdx.append(sc)
            armJointCount.append(numJoints)
            packVec(r_TP_P, [0.1, 0.0, 0.0])
            packVec(fhat_P, [-1.0, 0.0, 0.0])
            for j in range(numJoints):
                packVec(r_CP_P, linkPattern[j])
                packVec(shat_P, axisPattern[j])
                if arm == 1 and j == 0:
                    C0P = [[-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0],
                            [0.0, 0.0, 1.0]]
                else:
                    C0P = [[1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0]]
                packDCM(dcm_C0P, C0P)
    armCfg = {
        "thrArmIdx": thrArmIdx,
        "thrArmJointIdx": thrArmJointIdx,
        "armTreeIdx": armTreeIdx,
        "armJointCount": armJointCount,
        "r_CP_P": r_CP_P,
        "r_TP_P": r_TP_P,
        "shat_P": shat_P,
        "fhat_P": fhat_P,
        "dcm_C0P": dcm_C0P,
    }
    armConfigInMsgData.thrArmIdx.clear()
    armConfigInMsgData.thrArmJointIdx.clear()
    armConfigInMsgData.armTreeIdx.clear()
    armConfigInMsgData.armJointCount.clear()
    armConfigInMsgData.r_CP_P.clear()
    armConfigInMsgData.r_TP_P.clear()
    armConfigInMsgData.shat_P.clear()
    armConfigInMsgData.fhat_P.clear()
    armConfigInMsgData.dcm_C0P.clear()
    for v in thrArmIdx:
        armConfigInMsgData.thrArmIdx.push_back(v)
    for v in thrArmJointIdx:
        armConfigInMsgData.thrArmJointIdx.push_back(v)
    for v in armTreeIdx:
        armConfigInMsgData.armTreeIdx.push_back(v)
    for v in armJointCount:
        armConfigInMsgData.armJointCount.push_back(v)
    for v in r_CP_P:
        armConfigInMsgData.r_CP_P.push_back(v)
    for v in r_TP_P:
        armConfigInMsgData.r_TP_P.push_back(v)
    for v in shat_P:
        armConfigInMsgData.shat_P.push_back(v)
    for v in fhat_P:
        armConfigInMsgData.fhat_P.push_back(v)
    for v in dcm_C0P:
        armConfigInMsgData.dcm_C0P.push_back(v)

    # create the thr force input messages vectors
    thrForce = 0.25
    thrVec = (thrForce + 0.1*np.arange(numSpacecraft*numArms, dtype=float)).tolist()
    thrForcesInMsgDataList = []
    for sc in range(numSpacecraft):
        for arm in range(numArms):
            thrForcesInMsgData = messaging.SingleActuatorMsgPayload()
            thrForcesInMsgData.input = thrVec[sc*numArms + arm]
            thrForcesInMsgDataList.append(thrForcesInMsgData)

    # write input messages
    massMatrixInMsg = messaging.MJSysMassMatrixMsg().write(massMatrixInMsgData)
    reactionForcesInMsg = messaging.MJJointReactionsMsg().write(JointReactionsInMsgData)
    armConfigInMsg = messaging.THRArmConfigMsg().write(armConfigInMsgData)
    jointStatesInMsgList = []
    thrForcesInMsgList = []
    for i in range(numArms * numSpacecraft):
        thrForcesInMsg = messaging.SingleActuatorMsg().write(thrForcesInMsgDataList[i])
        thrForcesInMsgList.append(thrForcesInMsg)
        for j in range(numJoints):
            idx = i*(numJoints) + j
            jointStatesInMsg = messaging.ScalarJointStateMsg().write(jointStatesInMsgDataList[idx])
            jointStatesInMsgList.append(jointStatesInMsg)

    # subscribe input messages to module
    module.armConfigInMsg.subscribeTo(armConfigInMsg)
    module.massMatrixInMsg.subscribeTo(massMatrixInMsg)
    module.reactionForcesInMsg.subscribeTo(reactionForcesInMsg)
    for i in range(numArms * numSpacecraft):
        module.thrForcesInMsgs[i].subscribeTo(thrForcesInMsgList[i])
        for j in range(numJoints):
            idx = i*(numJoints) + j
            module.jointStatesInMsgs[idx].subscribeTo(jointStatesInMsgList[idx])

    # setup output message recorder objects
    motorTorqueRecorders = []
    for i in range(numJoints * numArms * numSpacecraft):
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

    # compute expected torques
    expectedTorques = jointTorques(numSpacecraft, numArms, numJoints, M, theta, biasForces, thrVec, armCfg, uMax)

    # Assert the motor torques are correct
    np.testing.assert_allclose(moduleTorques, expectedTorques, atol=1e-8,
                               err_msg=f"Motor torques {moduleTorques} do not match expected values {expectedTorques}")


if __name__ == "__main__":
    test_thrJointCompensation(False, False, 1, 1, 1)
