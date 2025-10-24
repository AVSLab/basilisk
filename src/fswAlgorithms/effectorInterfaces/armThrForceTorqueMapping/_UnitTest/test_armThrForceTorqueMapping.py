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
from Basilisk.fswAlgorithms import armThrForceTorqueMapping

def M1(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0,  c,   s ],
                     [0.0, -s,   c ]], dtype=float)

def M2(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[ c,  0.0, -s ],
                     [0.0, 1.0,  0.0],
                     [ s,  0.0,  c ]], dtype=float)

def M3(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[ c,  s,  0.0],
                     [-s,  c,  0.0],
                     [0.0, 0.0, 1.0]], dtype=float)

def compute_truth(thetas, thrForce, dcm_H1B, dcm_H2B, thr_F_T, r_S2S1_S1):
    """
    Replicates the module's internal math to generate truth data.
    """
    thetas = np.asarray(thetas, dtype=float).reshape(-1)
    thrForce = np.asarray(thrForce, dtype=float).reshape(-1)
    dcm_H1B = np.asarray(dcm_H1B, dtype=float).reshape(3,3)
    dcm_H2B = np.asarray(dcm_H2B, dtype=float).reshape(3,3)
    thr_F_T  = np.asarray(thr_F_T,  dtype=float).reshape(3,2)
    r_S2S1_S1 = np.asarray(r_S2S1_S1, dtype=float).reshape(3,10)
    # DCM chain (module uses dcm_J?B = M*(theta)*previous)
    dcm_J1B = M1(thetas[0]) @ dcm_H1B
    dcm_J2B = M2(thetas[1]) @ dcm_J1B
    dcm_J3B = M3(thetas[2]) @ dcm_J2B
    dcm_J4B = M2(thetas[3]) @ dcm_J3B
    dcm_T1B = dcm_J4B
    dcm_J5B = M1(thetas[4]) @ dcm_H2B
    dcm_J6B = M2(thetas[5]) @ dcm_J5B
    dcm_J7B = M3(thetas[6]) @ dcm_J6B
    dcm_J8B = M2(thetas[7]) @ dcm_J7B
    dcm_T2B = dcm_J8B

    # site positions (3x10)
    r = np.zeros((3, 10), dtype=float)
    r[:, 0] = r_S2S1_S1[:, 0]
    r[:, 1] = dcm_J1B.T @ r_S2S1_S1[:, 1] + r[:, 0]
    r[:, 2] = dcm_J2B.T @ r_S2S1_S1[:, 2] + r[:, 1]
    r[:, 3] = dcm_J3B.T @ r_S2S1_S1[:, 3] + r[:, 2]
    r[:, 4] = dcm_J4B.T @ r_S2S1_S1[:, 4] + r[:, 3]
    r[:, 5] = r_S2S1_S1[:, 5]
    r[:, 6] = dcm_J5B.T @ r_S2S1_S1[:, 6] + r[:, 5]
    r[:, 7] = dcm_J6B.T @ r_S2S1_S1[:, 7] + r[:, 6]
    r[:, 8] = dcm_J7B.T @ r_S2S1_S1[:, 8] + r[:, 7]
    r[:, 9] = dcm_J8B.T @ r_S2S1_S1[:, 9] + r[:, 8]

    # spin axes in body frame
    s1_B = dcm_J1B.T @ np.array([1, 0, 0], dtype=float)
    s2_B = dcm_J2B.T @ np.array([0, 1, 0], dtype=float)
    s3_B = dcm_J3B.T @ np.array([0, 0, 1], dtype=float)
    s4_B = dcm_J4B.T @ np.array([0, 1, 0], dtype=float)
    s5_B = dcm_J5B.T @ np.array([1, 0, 0], dtype=float)
    s6_B = dcm_J6B.T @ np.array([0, 1, 0], dtype=float)
    s7_B = dcm_J7B.T @ np.array([0, 0, 1], dtype=float)
    s8_B = dcm_J8B.T @ np.array([0, 1, 0], dtype=float)

    # thruster forces in body frame
    thr_F_B = np.zeros((3, 2), dtype=float)
    thr_F_B[:, 0] = dcm_T1B.T @ thr_F_T[:, 0] * thrForce[0]
    thr_F_B[:, 1] = dcm_T2B.T @ thr_F_T[:, 1] * thrForce[1]

    # hub torques from each thruster
    thr_Hub_T_B = np.zeros((3, 2), dtype=float)
    thr_Hub_T_B[:, 0] = np.cross(r[:, 4], thr_F_B[:, 0])
    thr_Hub_T_B[:, 1] = np.cross(r[:, 9], thr_F_B[:, 1])

    # joint torques
    jt = np.zeros(8, dtype=float)
    jt[0] = np.dot(s1_B, np.cross((r[:, 4] - r[:, 0]), thr_F_B[:, 0]))
    jt[1] = np.dot(s2_B, np.cross((r[:, 4] - r[:, 1]), thr_F_B[:, 0]))
    jt[2] = np.dot(s3_B, np.cross((r[:, 4] - r[:, 2]), thr_F_B[:, 0]))
    jt[3] = np.dot(s4_B, np.cross((r[:, 4] - r[:, 3]), thr_F_B[:, 0]))
    jt[4] = np.dot(s5_B, np.cross((r[:, 9] - r[:, 5]), thr_F_B[:, 1]))
    jt[5] = np.dot(s6_B, np.cross((r[:, 9] - r[:, 6]), thr_F_B[:, 1]))
    jt[6] = np.dot(s7_B, np.cross((r[:, 9] - r[:, 7]), thr_F_B[:, 1]))
    jt[7] = np.dot(s8_B, np.cross((r[:, 9] - r[:, 8]), thr_F_B[:, 1]))

    bodyF = thr_F_B.sum(axis=1)
    bodyT = thr_Hub_T_B.sum(axis=1)
    return bodyF, bodyT, jt

@pytest.mark.parametrize("thetas", [
    np.deg2rad([0, 0, 0, 0,  0, 0, 0, 0]),
    np.deg2rad([10, -15, 20, -25,  30, -35, 40, -45]),
    np.deg2rad([90, 0, 0, 0,  0, 0, 0, 0]),
    ([-0.00295787, -0.20997741, -0.20554589, -0.04484504, -0.09704499, -0.4633467, 0.59123222, -0.00996162])
])
@pytest.mark.parametrize("thrForce", [
    [0.0, 0.0],
    [1.0, 1.0],
])

def test_armThrForceTorqueMapping(thetas, thrForce):
    """Module Unit Test"""
    [testResults, testMessage] = armThrForceTorqueMappingTestFunction(thetas, thrForce)
    assert testResults < 1, testMessage


def armThrForceTorqueMappingTestFunction(thetas, thrForce):
    r"""
    **Validation Test Description**

    This unit test validates the forces and torques on the body and joints that result from the thrusters firing.

    **Test Parameters**

    The joint angles and thruster forces can be varied.

    Args:
        thetas (list): the joint angles in radians
        thrForce (list): the thruster forces in Newtons

    **Description of Variables Being Tested**

    In this test the body forces, body torques, and joint torques output by the module are compared to the expected values calculated in python.
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
    module = armThrForceTorqueMapping.ArmThrForceTorqueMapping()
    module.ModelTag = "armThrForceTorqueMappingTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # create the Config input message
    structConfigOut = messaging.MJSCConfigMsgPayload()
    dcm_H1B = np.eye(3)
    dcm_H2B = np.array([[-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0],
                       [0.0, 0.0, 1.0]])
    dcms_HB = [
                [ 1.0,  0.0,  0.0, -1.0,  0.0,  0.0],
                [ 0.0,  1.0,  0.0,  0.0, -1.0,  0.0],
                [ 0.0,  0.0,  1.0,  0.0,  0.0,  1.0]
                ]
    thr_F_T = [
                [-1.0, -1.0],
                [ 0.0,  0.0],
                [ 0.0,  0.0]
                ]
    r_S2S1_S1 = [
                [0.79, 0.0, 1.0, 0.0, 0.1, -0.79, 0.0, 1.0, 0.0, 0.1],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                ]
    structConfigOut.dcm_HB = dcms_HB
    structConfigOut.thr_F_T = thr_F_T
    structConfigOut.r_S2S1_S1 = r_S2S1_S1

    # Build the joint states input message
    jointStatesInMsgData = messaging.JointArrayStateMsgPayload()
    jointStatesInMsgData.thetas = thetas

    # Build the thruster force input message
    thrForceInMsgData = messaging.THRArrayCmdForceMsgPayload()
    thrForceInMsgData.thrForce = thrForce

    # Configure blank module input messages
    configInMsg = messaging.MJSCConfigMsg().write(structConfigOut)
    jointStatesInMsg = messaging.JointArrayStateMsg().write(jointStatesInMsgData)
    thrForceInMsg = messaging.THRArrayCmdForceMsg().write(thrForceInMsgData)

    # subscribe input messages to module
    module.configInMsg.subscribeTo(configInMsg)
    module.jointStatesInMsg.subscribeTo(jointStatesInMsg)
    module.thrForceInMsg.subscribeTo(thrForceInMsg)

    # setup output message recorder objects
    bodyForceOutMsgRec = module.bodyForceOutMsg.recorder()
    bodyForceOutMsgRecC = module.bodyForceOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, bodyForceOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, bodyForceOutMsgRecC)
    bodyTorqueOutMsgRec = module.bodyTorqueOutMsg.recorder()
    bodyTorqueOutMsgRecC = module.bodyTorqueOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, bodyTorqueOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, bodyTorqueOutMsgRecC)
    jointTorqueOutMsgRec = module.jointTorqueOutMsg.recorder()
    jointTorqueOutMsgRecC = module.jointTorqueOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, jointTorqueOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, jointTorqueOutMsgRecC)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.0))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    modBodyF = bodyForceOutMsgRec.forceRequestBody[-1,0:3]
    modBodyT = bodyTorqueOutMsgRec.torqueRequestBody[-1,0:3]
    modJT = jointTorqueOutMsgRec.motorTorque[-1,0:8]
    modBodyFC = bodyForceOutMsgRecC.forceRequestBody[-1,0:3]
    modBodyTC = bodyTorqueOutMsgRecC.torqueRequestBody[-1,0:3]
    modJTC = jointTorqueOutMsgRecC.motorTorque[-1,0:8]

    # calculate truth values
    truthBodyF, truthBodyT, truthJT = compute_truth(thetas, thrForce, dcm_H1B, dcm_H2B, thr_F_T, r_S2S1_S1)

    # compare the module results to the truth values
    try:
        assert_allclose(modBodyF, truthBodyF, rtol=1e-12, atol=1e-12)
    except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Body Forces fail {err}\n")
    try:
        assert_allclose(modBodyT, truthBodyT, rtol=1e-12, atol=1e-12)
    except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Body Torques fail {err}\n")
    try:
        assert_allclose(modJT, truthJT, rtol=1e-12, atol=1e-12)
    except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Joint Torques fail {err}\n")
    try:
        assert_allclose(modBodyF, modBodyFC, rtol=1e-12, atol=1e-12)
    except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Body Force C/C++ msg not equal {err}\n")
    try:
        assert_allclose(modBodyT, modBodyTC, rtol=1e-12, atol=1e-12)
    except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Body Torque C/C++ msg not equal {err}\n")
    try:
        assert_allclose(modJT, modJTC, rtol=1e-12, atol=1e-12)
    except AssertionError as err:
            testFailCount += 1
            testMessages.append(f"Joint Torque C/C++ msg not equal {err}\n")


    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_armThrForceTorqueMapping(
        thetas=([-0.00295787, -0.20997741, -0.20554589, -0.04484504, -0.09704499, -0.4633467, 0.59123222, -0.00996162]),
        thrForce=[1.0, 1.0]
    )
