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

import pytest
import numpy as np

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros
from Basilisk.utilities import fswSetupThrusters

from Basilisk.architecture import messaging

from Basilisk.fswAlgorithms import forceTorqueThrForceMapping

def test_forceTorqueThrForceMapping(show_plots):
    r"""
    **Validation Test Description**

    Compose a general description of what is being tested in this unit test script.

    **Test Parameters**

    Discuss the test parameters used.

    Args:
        param1 (int): Dummy test parameter for this parameterized unit test
        param2 (int): Dummy test parameter for this parameterized unit test
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    Here discuss what variables and states are being checked. 
    """
    [testResults, testMessage] = forceTorqueThrForceMappingTestFunction(show_plots)
    assert testResults < 1, testMessage


def forceTorqueThrForceMappingTestFunction(show_plots):
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
    moduleConfig = forceTorqueThrForceMapping.forceTorqueThrForceMappingConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "forceTorqueThrForceMappingTag"
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Configure blank module input messages
    cmdTorqueInMsgData = messaging.CmdTorqueBodyMsgPayload()
    cmdTorqueInMsgData.torqueRequestBody = [0.4, 0.2, 0.4]
    cmdTorqueInMsg = messaging.CmdTorqueBodyMsg().write(cmdTorqueInMsgData)

    cmdForceInMsgData = messaging.CmdForceBodyMsgPayload()
    cmdForceInMsgData.forceRequestBody = [0.9, 1.1, 0.]
    cmdForceInMsg = messaging.CmdForceBodyMsg().write(cmdForceInMsgData)

    numThrusters = 8
    maxThrust = 2.0  # N
    MAX_EFF_CNT = messaging.MAX_EFF_CNT
    rcsLocationData = np.zeros((MAX_EFF_CNT, 3))
    rcsDirectionData = np.zeros((MAX_EFF_CNT, 3))

    rcsLocationData[0:8] = [[-0.86360, -0.82550, 1.79070],
                            [-0.82550, -0.86360, 1.79070],
                            [0.82550, 0.86360, 1.79070],
                            [0.86360, 0.82550, 1.79070],
                            [-0.86360, -0.82550, -1.79070],
                            [-0.82550, -0.86360, -1.79070],
                            [0.82550, 0.86360, -1.79070],
                            [0.86360, 0.82550, -1.79070]]

    rcsDirectionData[0:8] = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, -1.0, 0.0],
                             [-1.0, 0.0, 0.0],
                             [1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, -1.0, 0.0],
                             [-1.0, 0.0, 0.0]]

    fswSetupThrusters.clearSetup()
    for i in range(numThrusters):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], maxThrust)
    thrConfigInMsg = fswSetupThrusters.writeConfigMessage()

    CoM_B = [0., 0., 0.]
    vehConfigInMsgData = messaging.VehicleConfigMsgPayload()
    vehConfigInMsgData.CoM_B = CoM_B
    vehConfigInMsg = messaging.VehicleConfigMsg().write(vehConfigInMsgData)

    # subscribe input messages to module
    moduleConfig.cmdTorqueInMsg.subscribeTo(cmdTorqueInMsg)
    moduleConfig.cmdForceInMsg.subscribeTo(cmdForceInMsg)
    moduleConfig.thrConfigInMsg.subscribeTo(thrConfigInMsg)
    moduleConfig.vehConfigInMsg.subscribeTo(vehConfigInMsg)

    # setup output message recorder objects
    thrForceCmdOutMsgRec = moduleConfig.thrForceCmdOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, thrForceCmdOutMsgRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    print(thrForceCmdOutMsgRec.thrForce)

    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_forceTorqueThrForceMapping(True)


