
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
from Basilisk.simulation import spaceToGroundTransmitter
from Basilisk.simulation import simpleInstrument
from Basilisk.simulation import partitionedStorageUnit
from Basilisk.simulation import simpleStorageUnit
from Basilisk.architecture import messaging
from Basilisk.utilities import macros

@pytest.mark.parametrize(
        "deviceStatus, accessStatus",
        [(0, 0),
         (1, 1)])
def test_baudRate(deviceStatus, accessStatus):
    """
    **Validation Test Description**

    1. Whether the simpleTransmitter provides the right output message (baudRate) while on;
    2. Whether the simpleTransmitter provides the right output message (baudRate) while off.
    3. Whether the simpleTransmitter provides the right output message (baudRate) while out of access.
    """
    expectedValue = deviceStatus * accessStatus

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create fake access messages
    accMsg1 = messaging.AccessMsgPayload()
    accMsg1.hasAccess = 0 # We'll never see this one, sadly
    acc1Msg = messaging.AccessMsg().write(accMsg1)

    accMsg2 = messaging.AccessMsgPayload()
    accMsg2.hasAccess = accessStatus
    acc2Msg = messaging.AccessMsg().write(accMsg2)

    # Create the test module
    testModule = spaceToGroundTransmitter.SpaceToGroundTransmitter()
    testModule.ModelTag = "transmitter"
    testModule.nodeBaudRate = -9600. # baud
    testModule.packetSize = -9600 # bits
    testModule.numBuffers = 1
    testModule.dataStatus = deviceStatus
    testModule.addAccessMsgToTransmitter(acc1Msg)
    testModule.addAccessMsgToTransmitter(acc2Msg)
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Create an instrument
    instrument = simpleInstrument.SimpleInstrument()
    instrument.ModelTag = "instrument1"
    instrument.nodeBaudRate = 9600. # baud
    instrument.nodeDataName = "Instrument 1"  # baud
    unitTestSim.AddModelToTask(unitTaskName, instrument)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageCapacity = 8E9 # bits (1 GB)
    dataMonitor.addDataNodeToModel(instrument.nodeDataOutMsg)
    dataMonitor.addDataNodeToModel(testModule.nodeDataOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, dataMonitor)

    testModule.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsg)

    datLog = testModule.nodeDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))
    unitTestSim.ExecuteSimulation()

    generatedData = datLog.baudRate
    print(generatedData)

    trueData = -9600. # Module should be on after enough data is accrued
    testArray = [0, 0, 0, expectedValue*trueData, expectedValue*trueData, expectedValue*trueData, expectedValue*trueData] # Should go through three iterations of no data downlinked

    np.testing.assert_array_equal(
        generatedData, testArray,
        err_msg="Generated data does not match expected values."
    )


@pytest.mark.parametrize(
        "partition_data_0s, partition_data_2s, partition_data_5s, packetSize",
        [
            (28800, 9600, 0, -9600),
            (9599, 9599, 9599, -9600),
            (9601, 1, 1, -9600),
            (3600, 3600, 3600, -9600),
            (3600, 0, 0, 0),
        ]
)
def test_downlink(partition_data_0s, partition_data_2s, partition_data_5s, packetSize):
    """
    **Validation Test Description**

    1. Whether the packetSize is correctly set in the spaceToGroundTransmitter module;
    2. Whether the spaceToGroundTransmitter module correctly downlinks data from the simpleStorageUnit;
    3. Whether the spaceToGroundTransmitter module correctly handles data less than the packet size;
    """
    accessStatus = 1  # 1 means the access is granted, 0 means it is denied

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create fake access messages
    accMsg2 = messaging.AccessMsgPayload()
    accMsg2.hasAccess = accessStatus
    acc2Msg = messaging.AccessMsg().write(accMsg2)

    # Create the test module
    testModule = spaceToGroundTransmitter.SpaceToGroundTransmitter()
    testModule.ModelTag = "transmitter"
    testModule.nodeBaudRate = -9600. # baud
    testModule.packetSize = packetSize # bits
    testModule.numBuffers = 0
    testModule.dataStatus = 1
    testModule.addAccessMsgToTransmitter(acc2Msg)
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = simpleStorageUnit.SimpleStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageCapacity = 8E9 # bits (1 GB)
    dataMonitor.addDataNodeToModel(testModule.nodeDataOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, dataMonitor)

    testModule.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsg)

    datLog = testModule.nodeDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    dataMonitor.setDataBuffer(partition_data_0s)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))
    unitTestSim.ExecuteSimulation()

    np.testing.assert_array_equal(
        dataMonitor.storageUnitDataOutMsg.read().storedData, partition_data_2s,
        err_msg="Intermediate partition data at 2 seconds does not match expected values."
    )

    unitTestSim.ConfigureStopTime(macros.sec2nano(5.0))
    unitTestSim.ExecuteSimulation()

    np.testing.assert_array_equal(
        dataMonitor.storageUnitDataOutMsg.read().storedData, partition_data_5s,
        err_msg="Intermediate partition data at 5 seconds does not match expected values."
    )


@pytest.mark.parametrize(
        "partition_data_0s, partition_data_2s, partition_data_5s, packetSize",
        [
            ([0, 19200, 19200], [0, 9600, 9600], [0, 0, 0], -9600),
            ([0, 9600, 9600], [0, 0, 0], [0, 0, 0], -9600),
            ([0, 0, 9599], [0, 0, 9599], [0, 0, 9599], -9600),
            ([0, 0, 9601], [0, 0, 1], [0, 0, 1], -9600),
            ([1200, 2400, 3600], [1200, 2400, 3600], [1200, 2400, 3600], -9600),
            ([1200, 2400, 3600], [0, 0, 0], [0, 0, 0], 0),
            ([1200, 2400, 3600], [0, 0, 0], [0, 0, 0], -1),
        ]
)
def test_downlink_from_partition(partition_data_0s, partition_data_2s, partition_data_5s, packetSize):
    """
    **Validation Test Description**

    1. Whether the spaceToGroundTransmitter module correctly downlinks data from the partitionedStorageUnit;
    2. Whether the spaceToGroundTransmitter module correctly handles data less than the packet size;
    3. Whether the spaceToGroundTransmitter module completely empties the partitionedStorageUnit after downlinking data.;
    """
    accessStatus = 1  # 1 means the access is granted, 0 means it is denied

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create fake access messages
    accMsg = messaging.AccessMsgPayload()
    accMsg.hasAccess = accessStatus
    acc_msg = messaging.AccessMsg().write(accMsg)

    # Create the test module
    testModule = spaceToGroundTransmitter.SpaceToGroundTransmitter()
    testModule.ModelTag = "transmitter"
    testModule.nodeBaudRate = -9600. # baud
    testModule.packetSize = packetSize # bits
    testModule.numBuffers = 0
    testModule.dataStatus = 1
    testModule.addAccessMsgToTransmitter(acc_msg)
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageCapacity = 8E9 # bits (1 GB)
    dataMonitor.addDataNodeToModel(testModule.nodeDataOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, dataMonitor)

    testModule.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsg)

    datLog = testModule.nodeDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    data_partition_names = ["P_1", "P_2", "P_3"]
    dataMonitor.setDataBuffer(data_partition_names, partition_data_0s)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))
    unitTestSim.ExecuteSimulation()

    np.testing.assert_array_equal(
        [data_i for data_i in dataMonitor.storageUnitDataOutMsg.read().storedData], partition_data_2s,
        err_msg="Partition data at 2 seconds does not match expected values."
    )

    unitTestSim.ConfigureStopTime(macros.sec2nano(5.0))
    unitTestSim.ExecuteSimulation()

    np.testing.assert_array_equal(
        [data_i for data_i in dataMonitor.storageUnitDataOutMsg.read().storedData], partition_data_5s,
        err_msg="Partition data at 5 seconds does not match expected values."
    )


@pytest.mark.parametrize(
        "partition_data_0s, partition_data_2s, partition_data_5s, packetSize",
        [
            ([0, 0, 38400], [0, 0, 19200], [0, 0, 19200], -38400),
            ([0, 0, 38400], [0, 0, 19200], [0, 0, 0], 0)

        ]
)
def test_downlink_with_disrupted_connection(partition_data_0s, partition_data_2s, partition_data_5s, packetSize):
    """
    **Validation Test Description**

    1. Whether the spaceToGroundTransmitter module correctly downlinks data from the partitionedStorageUnit with interrupted connection with ground station;
    """

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create fake access messages
    accMsg = messaging.AccessMsgPayload()
    accMsg.hasAccess = 1
    acc_msg = messaging.AccessMsg().write(accMsg)

    # Create the test module
    testModule = spaceToGroundTransmitter.SpaceToGroundTransmitter()
    testModule.ModelTag = "transmitter"
    testModule.nodeBaudRate = -9600. # baud
    testModule.packetSize = packetSize # bits
    testModule.numBuffers = 0
    testModule.dataStatus = 1
    testModule.addAccessMsgToTransmitter(acc_msg)
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageCapacity = 8E9 # bits (1 GB)
    dataMonitor.addDataNodeToModel(testModule.nodeDataOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, dataMonitor)

    testModule.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsg)

    datLog = testModule.nodeDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    data_partition_names = ["P_1", "P_2", "P_3"]
    dataMonitor.setDataBuffer(data_partition_names, partition_data_0s)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))
    unitTestSim.ExecuteSimulation()

    np.testing.assert_array_equal(
        [data_i for data_i in dataMonitor.storageUnitDataOutMsg.read().storedData], partition_data_2s,
        err_msg="Partition data at 2 seconds does not match expected values."
    )

    accMsg.hasAccess = 0 # Satellite loses connection with ground station
    acc_msg.write(accMsg)
    unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))
    unitTestSim.ExecuteSimulation()

    accMsg.hasAccess = 1 # Satellite reestablishes connection with ground station
    acc_msg.write(accMsg)
    unitTestSim.ConfigureStopTime(macros.sec2nano(5.0))
    unitTestSim.ExecuteSimulation()

    np.testing.assert_array_equal(
        [data_i for data_i in dataMonitor.storageUnitDataOutMsg.read().storedData], partition_data_5s,
        err_msg="Partition data at 5 seconds does not match expected values."
    )

#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    test_baudRate(1, 1)
    test_downlink(28800, 9600, 0, -9600)
    test_downlink_from_partition([0, 19200, 19200], [0, 9600, 9600], [0, 0, 0], -9600)
    test_downlink_with_disrupted_connection([0, 0, 38400], [0, 0, 19200], [0, 0, 19200], -38400)
