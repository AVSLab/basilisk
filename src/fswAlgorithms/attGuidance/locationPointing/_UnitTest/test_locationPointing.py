# 
#  ISC License
# 
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

#   Unit Test Script
#   Module Name:        locationPointing
#   Author:             Lewis Redner
#   Creation Date:      April 12, 2021
#

import pytest

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms.attGuidance import locationPointing
import numpy as np

# @pytest.mark.parametrize("accuracy", [1e-12])
# @pytest.mark.parametrize("pHat, r_LN_N, r_CN_N, r_BN_N, omega_BN_B", [
#      (1, 1)
#     ,(1, 3)
# ])

@pytest.mark.parametrize("case", [
     (1)        # location is visible, pointing vector not aligned
    ,(2)        # location is not visible, pointing vector not aligned
    ,(3)        # location is visible, pointing vector aligned
    ,(4)        # location is visible, pointing vector anti-parallel
    ,(5)        # location is visible, pointing vector perpendicular
])

def test_locationPointing(show_plots, param1, param2, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that an assigned pointing vector and ground location are used to compute the necessary
    Attitude Guidance message content
    

    **Test Parameters**

    Discuss the test parameters used.

    Args:
        param1 (int): Dummy test parameter for this parameterized unit test
        param2 (int): Dummy test parameter for this parameterized unit test
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    Here discuss what variables and states are being checked. 
    """
    [testResults, testMessage] = locationPointingTestFunction(show_plots, param1, param2, accuracy)
    assert testResults < 1, testMessage


def locationPointingTestFunction(show_plots, param1, param2, accuracy):
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
    moduleConfig = locationPointing.locationPointingConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "locationPointingTag"
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Configure blank module input messages
    SCInMsgData = messaging.SCStatesMsgPayload()

    # populate with key information
    r_BN_N = np.array([1000, 0, 0])
    r_CN_N = np.array([1000, 0, 0])
    omega_BN_B = np.array([0, 0, 0.1])
    pHat = np.array([0, 1, 1])



    # adjust for cases
        #
    r_LN_N = [-1000, 0, 0]
    if case == 3:
        pHat = np.array([-1, 0, 0])
    if case == 4:
        pHat = np.array([1, 0, 0])
    if case == 5:
        pHat = np.array([0, 1, 0])
    # normalise to unit vector
    pHat = pHat/np.linalg.norm(pHat)
    moduleConfig.pHat = pHat

    # populate SCStates input msg
    SCInMsgData.r_BN_N = r_BN_N
    SCInMsgData.r_CN_N = r_CN_N
    SCInMsgData.omega_BN_B = omega_BN_B

    SCInMsg = messaging.SCStatesMsg().write(SCInMsgData)

    LocationInMsgData = messaging.GroundStateMsgPayload()

    # populate r_LN_N (position vector of location wrt the inertial origin in the inertial frame)
    LocationInMsgData.r_LN_N = r_LN_N

    LocationInMsg = messaging.GroundStateMsg().write(LocationInMsgData)

    # subscribe input messages to module
    moduleConfig.SCInMsg.subscribeTo(SCInMsg)
    moduleConfig.LocationInMsg.subscribeTo(LocationInMsg)


    # setup output message recorder objects
    AttGuidOutMsgRec = moduleConfig.AttGuidOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, AttGuidOutMsgRec)

    # configure the inputs

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    ###############################################################################
    # Check outputs of simulation
    # OUTPUTS: sigma_BR, omega_BR_B, omega_RN_B, domega_RN_B

    # TODO update the truth values to make sure they're right
    #############
    # sigma_BR

    moduleOutput = AttGuidOutMsgRec.sigma_BR

    # calculate the truth values natively
    r_LB_N = -r_BN_N + r_LN_N
    eHat = np.cross(pHat, r_LB_N)
    eHat = eHat / np.linalg.norm(eHat)
    Phi = np.arccos(np.dot(pHat / np.linalg.norm(r_LB_N), r_LB_N))
    sigmaTrue = eHat * np.tan(Phi / 4.0)
    trueVector = [
        sigmaTrue.tolist(),
        sigmaTrue.tolist(),
        sigmaTrue.tolist()
    ]

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0, len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed sigma_BR unit test at t=" +
                                str(moduleOutput[i, 0] * macros.NANO2SEC) +
                                "sec\n")

    #############
    # omega_BR_B

    moduleOutput = AttGuidOutMsgRec.omega_BR_B

    # set the filtered output truth states
    trueVector = [
        [0., 0., 0.267949192431],
        [0., 0., 0.267949192431],
        [0., 0., 0.267949192431]
    ]
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0, len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed omega_BR_B unit test at t=" +
                                str(moduleOutput[i, 0] * macros.NANO2SEC) +
                                "sec\n")

    #############
    # omega_RN_B

    moduleOutput = AttGuidOutMsgRec.omega_RN_B

    # set the filtered output truth states
    trueVector = [
        [0., 0., 0.267949192431],
        [0., 0., 0.267949192431],
        [0., 0., 0.267949192431]
    ]
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0, len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed omega_RN_B unit test at t=" +
                                str(moduleOutput[i, 0] * macros.NANO2SEC) +
                                "sec\n")

    #############
    # domega_RN_B

    moduleOutput = AttGuidOutMsgRec.domega_RN_B

    # set the filtered output truth states
    trueVector = [
        [0., 0., 0.267949192431],
        [0., 0., 0.267949192431],
        [0., 0., 0.267949192431]
    ]
    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0, len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed domega_RN_B unit test at t=" +
                                str(moduleOutput[i, 0] * macros.NANO2SEC) +
                                "sec\n")


    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_locationPointing(False, 1, 1, 1e-12)


