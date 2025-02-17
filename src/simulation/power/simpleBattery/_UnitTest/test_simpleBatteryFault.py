#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
import pytest
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

from Basilisk.utilities import SimulationBaseClass
from Basilisk.architecture import messaging
from Basilisk.simulation import simpleBattery
from Basilisk.utilities import macros

params_set_data = [(5., 5., 5., 10., 0.3),
                    (1., 1., 5., 10., 0.3),
                    (5., 5., 5., 10., 0),
                    (5., 5., 5., 10., 1),
                    (5, 5, 5, 10, 1e-3),
                    (5., -5., 5., 10., 0.5)]

@pytest.mark.parametrize(
        "storedChargeInit, netPower_1, netPower_2, batteryCapacity, faultCapacityRatio",
        params_set_data)

def test_storage_limits(storedChargeInit, netPower_1, netPower_2, batteryCapacity, faultCapacityRatio):
    """
    **Validation Test Description**

    1. Check if the battery capacity matches its nominal value when a fault occurs.
    2. Verify that the charged capacity does not exceed the faulted capacity or drop below zero.

    :param storedChargeInit: [W] Initial stored charge in the battery.
    :param netPower_1: [W-s] Power input to the battery.
    :param netPower_2: [W-s] Power input to the battery.
    :param batteryCapacity: [W] Battery capacity.
    :param faultCapacityRatio: [-] Fault capacity ratio.

    :return: void
    """

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    test_battery = simpleBattery.SimpleBattery()
    test_battery.storedCharge_Init = storedChargeInit
    test_battery.storageCapacity = batteryCapacity

    powerMsg1 = messaging.PowerNodeUsageMsgPayload()
    powerMsg1.netPower = netPower_1
    pw1Msg = messaging.PowerNodeUsageMsg().write(powerMsg1)
    powerMsg2 = messaging.PowerNodeUsageMsgPayload()
    powerMsg2.netPower = netPower_2
    pw2Msg = messaging.PowerNodeUsageMsg().write(powerMsg2)

    faultMsg = messaging.PowerStorageFaultMsgPayload()
    faultMsg.faultCapacityRatio = faultCapacityRatio
    faultStatusMsg = messaging.PowerStorageFaultMsg().write(faultMsg)
    test_battery.batteryFaultInMsg.subscribeTo(faultStatusMsg)

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

    #   Check 1 - is capacity logged correctly?
    for ind in range(0,len(capacityLog)):
        capacity = capacityLog[ind]
        np.testing.assert_allclose(capacity, batteryCapacity, atol=1e-4,
            err_msg=("FAILED: SimpleBattery with fault did not correctly log the capacity."))

    #   Check 2 - is stored power correct?
    for ind in range(0,len(storedChargeLog)):
        storedCharge = storedChargeLog[ind]
        assert storedCharge <= capacityLog[ind] * faultCapacityRatio, (
            "FAILED: SimpleBattery's stored charge exceeded its faulted capacity.")

        assert storedCharge >= 0., (
            "FAILED: SimpleBattery's stored charge was negative.")


if __name__ == "__main__":
    storedChargeInit = 1.
    netPower_1 = 1.
    netPower_2 = 5.
    batteryCapacity = 10.
    faultCapacityRatio = 0.3
    test_storage_limits(storedChargeInit, netPower_1, netPower_2, batteryCapacity, faultCapacityRatio)
