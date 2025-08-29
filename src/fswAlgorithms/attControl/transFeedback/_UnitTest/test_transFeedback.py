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

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import transFeedback

def forceRequest(refStates, scStates, mass, K, P, F_ext):
    # compute the error
    deltaX = scStates[:, 0] - refStates[:, 0]
    deltaXdot = scStates[:, 1] - refStates[:, 1]

    # compute the control force
    F_control = -K @ deltaX - P @ deltaXdot + mass*refStates[:, 2] + F_ext

    return F_control

@pytest.mark.parametrize("problem", ["regulation", "tracking"])
@pytest.mark.parametrize("External_Force", [True, False])

def test_transFeedback(show_plots, problem, External_Force):
   """Module Unit Test"""
   [testResults, testMessage] = transFeedbackTest(show_plots, External_Force, problem)
   assert testResults < 1, testMessage

def transFeedbackTest(show_plots, External_Force, problem):
    r"""
    **Validation Test Description**

    This unit test sets up a translational motion PD controller

    **Test Parameters**

    The type of control being done and the presence of external forces is varied between tests

    Args:
        problem (str): The type of control problem being solved
        External_Force (bool): Flag indicating the presence of external forces

    **Description of Variables Being Tested**

    In this test we are checking that the computed feedback forces match the expected values.
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
    module = transFeedback.transFeedback()
    module.ModelTag = "transFeedbackTag"
    unitTestSim.AddModelToTask(unitTaskName, module)
    K = [[1., 0., 0.],
         [0., 1., 0.],
         [0., 0., 1.]]
    P = [[5., 0., 0.],
         [0., 5., 0.],
         [0., 0., 5.]]
    module.K = K
    module.P = P
    massSC = 500
    if External_Force:
        F_ext = [2., 3.5, -1.2]
        module.knownForcePntC_N = F_ext
    else:
        F_ext = [0., 0., 0.]

    # Setup vehicle config message
    vehConfigInMsgData = messaging.VehicleConfigMsgPayload()
    vehConfigInMsgData.massSC = massSC
    vehConfigInMsg = messaging.VehicleConfigMsg().write(vehConfigInMsgData)

    # Setup reference spacecraft states message
    refStateInMsgData = messaging.TransRefMsgPayload()
    if problem == "tracking":
        r_RN_N = [0.83, -1.0, 0.2]
        v_RN_N = [-0.15, 0.08, 0.1]
        a_RN_N = [0.01, -0.02, 0.03]
        refStateInMsgData.r_RN_N = r_RN_N
        refStateInMsgData.v_RN_N = v_RN_N
        refStateInMsgData.a_RN_N = a_RN_N
    else:
        r_RN_N = [0., 0., 0.]
        v_RN_N = [0., 0., 0.]
        a_RN_N = [0., 0., 0.]
    refStateInMsg = messaging.TransRefMsg().write(refStateInMsgData)

    # Setup current spacecraft states message
    scStateInMsgData = messaging.SCStatesMsgPayload()
    r_CN_N = [0.9, -1.1, 0.03]
    v_CN_N = [-0.2, 0.1, 0.06]
    scStateInMsgData.r_CN_N = r_CN_N
    scStateInMsgData.v_CN_N = v_CN_N
    scStateInMsg = messaging.SCStatesMsg().write(scStateInMsgData)

    # subscribe input messages to module
    module.transRefInMsg.subscribeTo(refStateInMsg)
    module.scStateInMsg.subscribeTo(scStateInMsg)
    module.vehConfigInMsg.subscribeTo(vehConfigInMsg)

    # setup output message recorder objects
    cmdForceOutMsgRec = module.cmdForceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, cmdForceOutMsgRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.1))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    moduleF_N = cmdForceOutMsgRec.forceRequestInertial[0, :]

    # compute the truth data
    referenceStates = np.column_stack((r_RN_N, v_RN_N, a_RN_N))
    scStates = np.column_stack((r_CN_N, v_CN_N))
    trueF_N = forceRequest(referenceStates, scStates, massSC, np.array(K), np.array(P), np.array(F_ext))

    # compare the module output to the truth values
    accuracy = 1e-12
    testFailCount, testMessages = unitTestSupport.compareArrayND([trueF_N], [moduleF_N], accuracy, "ControlForces",
                                                                 3, testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_transFeedback(False, "regulation", False)
