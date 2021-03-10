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

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.simulation import msmForceTorque

@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("param1, param2", [
     (1, 1)
    ,(1, 3)
])

def test_msmForceTorque(show_plots, param1, param2, accuracy):
    r"""
    **Validation Test Description**

    The behavior of the MSM e-force and torque evaluation is tested.

    **Test Parameters**

    Discuss the test parameters used.

    Args:
        param1 (int): Dummy test parameter for this parameterized unit test
        param2 (int): Dummy test parameter for this parameterized unit test
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    Here discuss what variables and states are being checked. 
    """
    [testResults, testMessage] = msmForceTorqueTestFunction(show_plots, param1, param2, accuracy)
    assert testResults < 1, testMessage


def msmForceTorqueTestFunction(show_plots, param1, param2, accuracy):
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
    module = msmForceTorque.MsmForceTorque()
    module.ModelTag = "msmForceTorqueTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure blank module input messages
    scStateInMsgsData = messaging.SCStatesMsgPayload()
    scStateInMsgsData.r_BN_N = [10., 20., 30.]
    scStateInMsgs = messaging.SCStatesMsg().write(scStateInMsgsData)

    voltInMsgData = messaging.VoltageMsgPayload()
    voltInMsgData.voltage = 10000.
    voltInMsg = messaging.VoltageMsg().write(voltInMsgData)

    # add spacecraft to state
    spPosList = [
        [1., 2., 3.]
        , [4., 5., 6.]
    ]
    rList = [1., 2.]

    module.addSpacecraftToModel(scStateInMsgs
                                , messaging.DoubleVector(rList)
                                , unitTestSupport.npList2EigenXdVector(spPosList))
    module.addSpacecraftToModel(scStateInMsgs
                                , messaging.DoubleVector(rList)
                                , unitTestSupport.npList2EigenXdVector(spPosList))

    # subscribe input messages to module
    module.voltInMsgs[0].subscribeTo(voltInMsg)
    module.voltInMsgs[1].subscribeTo(voltInMsg)

    # setup output message recorder objects
    eTorqueOutMsgsRec = module.eTorqueOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, eTorqueOutMsgsRec)
    eForceOutMsgsRec = module.eForceOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, eForceOutMsgsRec)

    unitTestSim.InitializeSimulation()
    # unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    # unitTestSim.ExecuteSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data and make sure it is correct

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_msmForceTorque(False, 1, 1, 1e-12)


