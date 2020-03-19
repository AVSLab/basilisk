
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




import pytest
import os, inspect
import numpy as np
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import spaceToGroundTransmitter
from Basilisk.simulation import simpleInstrument
from Basilisk.simulation import partitionedStorageUnit
from Basilisk.simulation import simMessages
from Basilisk.simulation import simFswInterfaceMessages
from Basilisk.utilities import macros


@pytest.mark.parametrize("deviceStatus", [0,1])
@pytest.mark.parametrize("accessStatus", [0,1])
def test_module(show_plots,deviceStatus, accessStatus):
    """
    **Validation Test Description**

    1. Whether the simpleTransmitter provides the right output message (baudRate) while on;
    2. Whether the simpleTransmitter provides the right output message (baudRate) while off.
    3. Whether the simpleTransmitter provides the right output message (baudRate) while out of access.

    :param show_plots: Not used; no plots to be shown.

    :return:
    """

    default_results, default_message = run(deviceStatus, accessStatus)

    testResults = sum([default_results])
    testMessage = [default_message]

    assert testResults < 1, testMessage


def run(deviceStatus, accessStatus):

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
    accMsg1 = simMessages.AccessSimMsg()
    accMsg1.hasAccess = 0 # We'll never see this one, sadly
    unitTestSupport.setMessage(unitTestSim.TotalSim, "TestProcess",'accessSimMsg1', accMsg1, 'AccessSimMsg')

    accMsg2 = simMessages.AccessSimMsg()
    accMsg2.hasAccess = accessStatus
    unitTestSupport.setMessage(unitTestSim.TotalSim, "TestProcess",'accessSimMsg2', accMsg2, 'AccessSimMsg')

    # Create the test module
    testModule = spaceToGroundTransmitter.SpaceToGroundTransmitter()
    testModule.ModelTag = "transmitter"
    testModule.nodeBaudRate = 9600. # baud
    testModule.packetSize = -9600 # bits
    testModule.numBuffers = 1
    testModule.dataStatus = deviceStatus
    testModule.nodeDataOutMsgName = "TransmitterMsg"
    testModule.addAccessMsgToTransmitter('accessSimMsg1')
    testModule.addAccessMsgToTransmitter('accessSimMsg2')
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Create an instrument
    instrument = simpleInstrument.SimpleInstrument()
    instrument.ModelTag = "instrument1"
    instrument.nodeBaudRate = 9600. # baud
    instrument.nodeDataName = "Instrument 1" # baud
    instrument.nodeDataOutMsgName = "Instrument1Msg"
    unitTestSim.AddModelToTask(unitTaskName, instrument)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageUnitDataOutMsgName = "dataMonitorMsg"
    dataMonitor.storageCapacity = 8E9 # bits (1 GB)
    dataMonitor.addDataNodeToModel(instrument.nodeDataOutMsgName)
    dataMonitor.addDataNodeToModel(testModule.nodeDataOutMsgName)
    unitTestSim.AddModelToTask(unitTaskName, dataMonitor)

    testModule.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsgName)

    unitTestSim.TotalSim.logThisMessage(testModule.nodeDataOutMsgName, testProcessRate)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))
    unitTestSim.ExecuteSimulation()

    generatedData = unitTestSim.pullMessageLogData(testModule.nodeDataOutMsgName + ".baudRate")
    print(generatedData[:,1])
    accuracy = 1e-16

    trueData = 9600. # Module should be on after enough data is accrued
    testArray = [0, 0, 0, expectedValue*trueData, expectedValue*trueData, expectedValue*trueData, expectedValue*trueData] # Should go through three iterations of no data downlinked

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        testArray, generatedData, accuracy, "dataOutput",
        testFailCount, testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]
#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    test_module(False, 1,1)