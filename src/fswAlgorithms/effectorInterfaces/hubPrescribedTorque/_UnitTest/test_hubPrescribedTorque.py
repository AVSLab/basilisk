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
from Basilisk.fswAlgorithms import hubPrescribedTorque

def prescribedTorque(M, nonActForce, jointTorque):
    M = np.asarray(M, dtype=float)
    nonActForce = np.asarray(nonActForce, dtype=float).reshape(-1)
    jointTorque = np.asarray(jointTorque, dtype=float).reshape(-1)

    nDOF = M.shape[0]
    nj = jointTorque.size

    if nDOF == nj:
        # if base is fixed no prescribed torques are needed
        tau_prescribed = np.zeros(3)

    else:
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

        tau_prescribed = -baseRotBias + (Mrt @ r_ddot) + (Mrth @ theta_ddot)

    return tau_prescribed

@pytest.mark.parametrize("nonActForces", [False, True])
@pytest.mark.parametrize("numJoints", [1,4])
@pytest.mark.parametrize("base", [False, True])

def test_hubPrescribedTorque(nonActForces, numJoints, base):
    """Module Unit Test"""
    [testResults, testMessage] = hubPrescribedTorqueTestFunction(nonActForces, numJoints, base)
    assert testResults < 1, testMessage

def hubPrescribedTorqueTestFunction(nonActForces, numJoints, base):
    r"""
    **Validation Test Description**

    This unit test validates the prescribed hub torques needed to offset hub acceleration \
        as a result of joint torques.

    **Test Parameters**

    The presence of nonActuator forces, number of joints, and whether the base is fixed or not are varied between tests.

    Args:
        nonActForces (bool): Flag indicating presence of non-actuator forces
        numJoints (int): Number of joints in the system
        base (bool): Flag indicating whether the base is fixed

    **Description of Variables Being Tested**

    In this test we are checking that the hub torques found by the hubPrescribedTorque module match the expected values.
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
    module = hubPrescribedTorque.HubPrescribedTorque()
    module.ModelTag = "hubPrescribedTorqueTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Create the Mass Matrix input message
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

    # create the NonActuator Forces input message
    nonActForceInMsgData = messaging.MJNonActuatorForcesMsgPayload()
    if base:
        bias = np.zeros(nbase + nj)
        bias[0:3] = 2.0 if nonActForces else 0.0
        bias[3:6] = 2.0 if nonActForces else 0.0
        bias[nbase:] = 2.0 if nonActForces else 0.0
        nonActForceInMsgData.baseTransForces = bias[0:3].tolist()
        nonActForceInMsgData.baseRotForces = bias[3:6].tolist()
        nonActForceInMsgData.jointForces = bias[nbase:].tolist()
    else:
        bias = np.full(nj, 2.0 if nonActForces else 0.0, dtype=float)
        nonActForceInMsgData.jointForces = bias.tolist()

    # create the Joint Torque input message
    jointTorqueInMsgData = messaging.ArrayMotorTorqueMsgPayload()
    u_H = np.full(nj, 1.0, dtype=float)
    jointTorqueInMsgData.motorTorque = u_H.tolist()

    # write input messages
    massMatrixInMsg = messaging.MJSysMassMatrixMsg().write(massMatrixInMsgData)
    nonActForceInMsg = messaging.MJNonActuatorForcesMsg().write(nonActForceInMsgData)
    jointTorqueInMsg = messaging.ArrayMotorTorqueMsg().write(jointTorqueInMsgData)

    # subscribe input messages to module
    module.massMatrixInMsg.subscribeTo(massMatrixInMsg)
    module.nonActForceInMsg.subscribeTo(nonActForceInMsg)
    module.jointTorqueInMsg.subscribeTo(jointTorqueInMsg)

    # setup output message recorder objects
    cmdTorqueOutMsgRec = module.cmdTorqueOutMsg.recorder()
    cmdTorqueOutMsgRecC = module.cmdTorqueOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, cmdTorqueOutMsgRec)
    unitTestSim.AddModelToTask(unitTaskName, cmdTorqueOutMsgRecC)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # pull module data
    modTorque = cmdTorqueOutMsgRec.torqueRequestBody[0, 0:3]
    modTorqueC = cmdTorqueOutMsgRecC.torqueRequestBody[0, 0:3]

    # calculate truth data
    truthTorque = prescribedTorque(M, bias, u_H)

    # compare module results to truth values
    try:
        assert_allclose(modTorque, truthTorque, atol=1e-12)
    except AssertionError as err:
        testFailCount += 1
        testMessages.append(f"Torques fail {err}\n")
    try:
        assert_allclose(modTorqueC, modTorque, atol=1e-12)
    except AssertionError as err:
        testFailCount += 1
        testMessages.append(f"Torque C/C++ msg not equal {err}\n")

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_hubPrescribedTorque(False, 1, False)
