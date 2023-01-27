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
from Basilisk.architecture import messaging
from Basilisk.simulation import msmForceTorque
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


@pytest.mark.parametrize("accuracy", [1e-4])
def test_msmForceTorque(show_plots, accuracy):
    r"""
    **Validation Test Description**

    The behavior of the MSM e-force and torque evaluation is tested.  3 space objects locations and
    orientations are setup.  Each object is assigned 2-3 sphere locations and radii.  The voltage
    input messages are setup such that each space object has its own potential.  The simulation
    is run for a single update cycle and the resulting forces and torques acting on each body
    are compared to hand-computed truth values.

    **Test Parameters**

    Args:
        accuracy (float): relative accuracy value used in the validation tests

    **Description of Variables Being Tested**

    The module output messages for the inertial force vector and body torque vector are compared to
    hand-calculated truth values using their relative accuracy.
    """
    [testResults, testMessage] = msmForceTorqueTestFunction(show_plots, accuracy)
    assert testResults < 1, testMessage


def msmForceTorqueTestFunction(show_plots, accuracy):
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

    # Configure space object state and voltage input messages
    sc0StateInMsgsData = messaging.SCStatesMsgPayload()
    sc0StateInMsgsData.r_BN_N = [10., 2., 3.]
    sc0StateInMsgsData.sigma_BN = [0.1, 0.2, 0.3]
    sc0StateInMsg = messaging.SCStatesMsg().write(sc0StateInMsgsData)

    sc1StateInMsgsData = messaging.SCStatesMsgPayload()
    sc1StateInMsgsData.r_BN_N = [-10., -2., 3.]
    sc1StateInMsgsData.sigma_BN = [-0.1, 0.2, 0.3]
    sc1StateInMsg = messaging.SCStatesMsg().write(sc1StateInMsgsData)

    sc2StateInMsgsData = messaging.SCStatesMsgPayload()
    sc2StateInMsgsData.r_BN_N = [1., 1., 0.]
    sc2StateInMsgsData.sigma_BN = [0.1, 0.2, -0.3]
    sc2StateInMsg = messaging.SCStatesMsg().write(sc2StateInMsgsData)

    volt0InMsgData = messaging.VoltMsgPayload()
    volt0InMsgData.voltage = 30000.
    volt0InMsg = messaging.VoltMsg().write(volt0InMsgData)

    volt1InMsgData = messaging.VoltMsgPayload()
    volt1InMsgData.voltage = -10000.
    volt1InMsg = messaging.VoltMsg().write(volt1InMsgData)

    volt2InMsgData = messaging.VoltMsgPayload()
    volt2InMsgData.voltage = 20000.
    volt2InMsg = messaging.VoltMsg().write(volt2InMsgData)

    # create a list of sphere body-fixed locations and associated radii
    spPosList = [
        [1., 2., 3.]
        , [4., 5., 6.]
        , [14., 5., 6.]
    ]
    rList = [1., 2., 1.5]

    # add spacecraft to state
    module.addSpacecraftToModel(sc0StateInMsg
                                , messaging.DoubleVector(rList[:-1])
                                , unitTestSupport.npList2EigenXdVector(spPosList[:-1]))
    module.addSpacecraftToModel(sc1StateInMsg
                                , messaging.DoubleVector(rList)
                                , unitTestSupport.npList2EigenXdVector(spPosList))
    module.addSpacecraftToModel(sc2StateInMsg
                                , messaging.DoubleVector(rList[:-1])
                                , unitTestSupport.npList2EigenXdVector(spPosList[:-1]))

    # subscribe input messages to module
    module.voltInMsgs[0].subscribeTo(volt0InMsg)
    module.voltInMsgs[1].subscribeTo(volt1InMsg)
    module.voltInMsgs[2].subscribeTo(volt2InMsg)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # set truth force and torque values
    fTruth = [
        [6.48179e-05, 0.00147205, 0.000924806]
        , [0.00107182, -0.000240543, 0.000110224]
        , [-0.00113664, -0.00123151, -0.00103503]
    ]
    tauTruth = [
        [-0.00268192, 0.00295288, -0.000603687]
        , [0.00688387, -0.00209438, -0.00647544]
        , [0.00581629, 0.007876, -0.00986612]
    ]
    chargeTruth = [
        [1.99932e-6, 5.73861e-6]
        , [-1.06715e-6, -2.51072e-6, -1.94044e-6]
        , [1.30148e-6, 3.23131e-6]
    ]

    # pull module data and make sure it is correct
    for i in range(3):
        f = module.eForceOutMsgs[i].read().forceRequestInertial
        testFailCount, testMessages = \
            unitTestSupport.compareDoubleArrayRelative(f, fTruth[i],
                                                       accuracy, "sc" + str(i) + " force test",
                                                       testFailCount, testMessages)
        tau = module.eTorqueOutMsgs[i].read().torqueRequestBody
        testFailCount, testMessages = \
            unitTestSupport.compareDoubleArrayRelative(tau, tauTruth[i],
                                                       accuracy, "sc" + str(i) + " torque test",
                                                       testFailCount, testMessages)

        charge = unitTestSupport.columnToRowList(module.chargeMsmOutMsgs[i].read().q)
        testFailCount, testMessages = \
            unitTestSupport.compareListRelative(charge, chargeTruth[i],
                                                       accuracy, "sc" + str(i) + " charge test",
                                                       testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_msmForceTorque(False, 1e-4)


