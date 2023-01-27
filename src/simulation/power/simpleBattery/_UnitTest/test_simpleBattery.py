#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import inspect
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.architecture import messaging
from Basilisk.simulation import simpleBattery
from Basilisk.utilities import macros


def test_module(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = storage_limits(show_plots)
    assert testResults < 1, testMessage


def storage_limits(show_plots):
    """
    **Validation Test Description**

    1. Whether the simpleBattery can add multiple nodes (core base class functionality);
    2. That the battery correctly evaluates how much stored power it should have given a pair of five-watt
       input messages.

    :param show_plots: Not used; no plots to be shown.

    :return:
    """

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    test_battery = simpleBattery.SimpleBattery()
    test_battery.storedCharge_Init = 5.
    test_battery.storageCapacity = 10.  # 10 W-s capacity.

    powerMsg1 = messaging.PowerNodeUsageMsgPayload()
    powerMsg1.netPower = 5.0
    pw1Msg = messaging.PowerNodeUsageMsg().write(powerMsg1)
    powerMsg2 = messaging.PowerNodeUsageMsgPayload()
    powerMsg2.netPower = 5.0
    pw2Msg = messaging.PowerNodeUsageMsg().write(powerMsg2)

    # Test the addNodeToStorage method:
    test_battery.addPowerNodeToModel(pw1Msg)
    test_battery.addPowerNodeToModel(pw2Msg)

    unitTestSim.AddModelToTask(unitTaskName, test_battery)

    dataLog = test_battery.batPowerOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(5.0))

    unitTestSim.ExecuteSimulation()

    storedChargeLog = dataLog.storageLevel
    capacityLog = dataLog.storageCapacity
    netPowerLog = dataLog.currentNetPower

    #   Check 1 - is net power equal to 10.?
    for ind in range(0,len(netPowerLog)):
        currentPower = netPowerLog[ind]
        if currentPower !=10.:
            testFailCount +=1
            testMessages.append("FAILED: SimpleBattery did not correctly log the net power.")

    if not unitTestSupport.isDoubleEqualRelative((10.),storedChargeLog[-1], 1e-8):
        testFailCount+=1
        testMessages.append("FAILED: SimpleBattery did not track integrated power. Returned "+str(storedChargeLog[-1,1])+", expected "+str((10.)))


    for ind in range(0,len(storedChargeLog)):
        if storedChargeLog[ind] > capacityLog[ind]:
            testFailCount +=1
            testMessages.append("FAILED: SimpleBattery's stored charge exceeded its capacity.")

        if storedChargeLog[ind] < 0.:
            testFailCount +=1
            testMessages.append("FAILED: SimpleBattery's stored charge was negative.")

    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    print(test_module(False))
