
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import inspect
import os
import pytest
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import simpleStorageUnit
from Basilisk.architecture import messaging
from Basilisk.utilities import macros

params_storage_limits = [(1200, 1200, 2400, 2400),
                     (600, 1200, 3600, 3600),
                     (600, 600, 10000, 6000)]

@pytest.mark.parametrize("baudRate_1, baudRate_2, storageCapacity, expectedStorage",
                          params_storage_limits)

def test_storage_limits(baudRate_1, baudRate_2, storageCapacity, expectedStorage):
    """
    Tests:

    1. Whether the simpleStorageUnit can add multiple nodes (core base class 
        functionality);
    2. That the simpleStorageUnit correctly evaluates how much stored data it should 
        have given a pair of input messages.

    """

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    test_storage_unit = simpleStorageUnit.SimpleStorageUnit()
    test_storage_unit.storageCapacity = storageCapacity # bit capacity.

    dataMsg1 = messaging.DataNodeUsageMsgPayload()
    dataMsg1.baudRate = baudRate_1 # baud
    dataMsg1.dataName = "node_1_msg"
    dat1Msg = messaging.DataNodeUsageMsg().write(dataMsg1)

    dataMsg2 = messaging.DataNodeUsageMsgPayload()
    dataMsg2.baudRate = baudRate_2 # baud
    dataMsg2.dataName = "node_2_msg"
    dat2Msg = messaging.DataNodeUsageMsg().write(dataMsg2)

    # Test the addNodeToStorage method:
    test_storage_unit.addDataNodeToModel(dat1Msg)
    test_storage_unit.addDataNodeToModel(dat2Msg)

    unitTestSim.AddModelToTask(unitTaskName, test_storage_unit)

    dataLog = test_storage_unit.storageUnitDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(5.0))

    unitTestSim.ExecuteSimulation()

    storedDataLog = dataLog.storageLevel
    capacityLog = dataLog.storageCapacity
    netBaudLog = dataLog.currentNetBaud
    partitionName = dataLog.storedDataName

    #   Check 1 - is net baud rate correct?
    for ind in range(0,len(netBaudLog)):
        currentBaud = netBaudLog[ind]
        np.testing.assert_allclose(currentBaud, baudRate_1 + baudRate_2, atol=1e-1,
            err_msg=("FAILED: PartitionedStorageUnit did not correctly log baud rate."))

    #   Check 2 - is used storage space correct?
    np.testing.assert_allclose(storedDataLog[-1], expectedStorage, atol=1e-4,
        err_msg=("FAILED: PartitionedStorageUnit did not track integrated data."))

    #   Check 3 - is the amount of data more than zero and less than the capacity?
    for ind in range(0,len(storedDataLog)):
        assert storedDataLog[ind] <= capacityLog[ind] or np.isclose(storedDataLog[ind], 
            capacityLog[ind]), (
            "FAILED: PartitionedStorageUnit's stored data exceeded its capacity.")

        assert storedDataLog[ind] >= 0., (
            "FAILED: PartitionedStorageUnit's stored data was negative.")

    #   Check 4 - is there only one partition?
    assert len(partitionName[0]) == 1, (
        "FAILED: PartitionedStorageUnit did use the correct partition.")
    
    #   Check 6 - is the name of the partition correct?
    assert partitionName[0][0] == "STORED DATA", (
        "FAILED: PartitionedStorageUnit did not correctly log the partition name.")


params_set_data = [(1200, 1200, 1200, 2400, 2400),
                   (600, 600, 0, 10000, 6000),
                   (600, 600, 4e3, 10000, 10000),
                   (0, 0, 1000, 2000, 1000),
                   (0, 0, -1000, 2000, 0),
                   (1000, 0, -2000, 6e3, 3000),
                   (0, 0, 3000, 2000, 0),
                   (600, 0, 3000, 10000, 6000),
                   (600, 600, 3000, 10000, 9000),
                   (300, 600, 3000, 10000, 7500)]

@pytest.mark.parametrize(
        "baudRate_1, baudRate_2, add_data, storageCapacity, expectedStorage",
        params_set_data)
def test_set_data_buffer(baudRate_1, baudRate_2, add_data, storageCapacity, 
                         expectedStorage):
    """
    Tests:

    1. Whether the partitionedStorageUnit can add data using the setDataBuffer method;
    2. That the partitionedStorageUnit correctly evaluates how much stored data it 
        should have given a pair of input messages and using setDataBuffer.

    :return:
    """

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    test_storage_unit = simpleStorageUnit.SimpleStorageUnit()
    test_storage_unit.storageCapacity = storageCapacity # bit capacity.

    dataMsg1 = messaging.DataNodeUsageMsgPayload()
    dataMsg1.baudRate = baudRate_1 # baud
    dataMsg1.dataName = "node_1_msg"
    dat1Msg = messaging.DataNodeUsageMsg().write(dataMsg1)

    dataMsg2 = messaging.DataNodeUsageMsgPayload()
    dataMsg2.baudRate = baudRate_2 # baud
    dataMsg2.dataName = "node_2_msg"
    dat2Msg = messaging.DataNodeUsageMsg().write(dataMsg2)

    # Test the addNodeToStorage method:
    test_storage_unit.addDataNodeToModel(dat1Msg)
    test_storage_unit.addDataNodeToModel(dat2Msg)

    unitTestSim.AddModelToTask(unitTaskName, test_storage_unit)

    dataLog = test_storage_unit.storageUnitDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    test_storage_unit.setDataBuffer(0)

    unitTestSim.InitializeSimulation()
    sim_time = 5.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(sim_time - 1.0))
    unitTestSim.ExecuteSimulation()

    test_storage_unit.setDataBuffer(add_data)
    unitTestSim.ConfigureStopTime(macros.sec2nano(sim_time))
    unitTestSim.ExecuteSimulation()

    storedDataLog = dataLog.storageLevel
    capacityLog = dataLog.storageCapacity
    netBaudLog = dataLog.currentNetBaud
    partitionName = dataLog.storedDataName
    partitionData = dataLog.storedData

    #   Check 1 - is net baud rate correct?
    for ind in range(0,len(netBaudLog)):
        currentBaud = netBaudLog[ind]
        np.testing.assert_allclose(currentBaud, baudRate_1 + baudRate_2, atol=1e-4,
            err_msg=("FAILED: PartitionedStorageUnit did not correctly log baud rate."))

    #   Check 2 - is used storage space correct?
    np.testing.assert_allclose(storedDataLog[-1], expectedStorage, atol=1e-4,
        err_msg=("FAILED: PartitionedStorageUnit did not track integrated data."))

    #   Check 3 - is the amount of data more than zero and less than the capacity?
    for ind in range(0,len(storedDataLog)):
        assert storedDataLog[ind] <= capacityLog[ind] or np.isclose(storedDataLog[ind], 
            capacityLog[ind]), (
            "FAILED: PartitionedStorageUnit's stored data exceeded its capacity.")

        assert storedDataLog[ind] >= 0., (
            "FAILED: PartitionedStorageUnit's stored data was negative.")

    #   Check 4 - is the data in the partitioned storage unit correct?
    assert partitionData[-1][0] == storedDataLog[-1], (
        "FAILED: PartitionedStorageUnit did not correctly log the stored data.")
    
    #   Check 5 - is there only one partition?
    assert len(partitionName[0]) == 1, (
        "FAILED: PartitionedStorageUnit should have just one partition.")
   
    #   Check 6 - is the name of the partition correct?
    assert partitionName[0][0] == "STORED DATA", (
        "FAILED: PartitionedStorageUnit did not correctly log the partition name.")


if __name__ == "__main__":
    baudRate_1 = 1200
    baudRate_2 = 1200
    storageCapacity = 2400
    expectedStorage = 2400
    test_storage_limits(baudRate_1, baudRate_2, 
                        storageCapacity, expectedStorage)
    add_data = 1200
    test_set_data_buffer(baudRate_1, baudRate_2, add_data, 
                         storageCapacity, expectedStorage)