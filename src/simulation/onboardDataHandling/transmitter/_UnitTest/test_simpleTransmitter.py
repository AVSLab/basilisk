
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

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import simpleTransmitter
from Basilisk.simulation import simpleInstrument
from Basilisk.simulation import partitionedStorageUnit
from Basilisk.utilities import macros

# update "module" in this function name to reflect the module name

@pytest.mark.parametrize("function", ["checkDefault"
                                      , "checkStatus"
                                      ])
def test_simpleTransmitterAll(show_plots, function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '()')
    assert testResults < 1, testMessage


def checkDefault():
    """
    **Validation Test Description**

    1. Whether the simpleTransmitter provides the right output message (baudRate) while on;
    2. Whether the simpleTransmitter provides the right output message (baudRate) while off.

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
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the test module
    testModule = simpleTransmitter.SimpleTransmitter()
    testModule.ModelTag = "transmitter"
    testModule.nodeBaudRate = 9600.  # baud
    testModule.packetSize = -9600  # bits
    testModule.numBuffers = 1
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Create an instrument
    instrument = simpleInstrument.SimpleInstrument()
    instrument.ModelTag = "instrument1"
    instrument.nodeBaudRate = 9600.  # baud
    instrument.nodeDataName = "Instrument 1" # baud
    unitTestSim.AddModelToTask(unitTaskName, instrument)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageCapacity = 8E9  # bits (1 GB)
    dataMonitor.addDataNodeToModel(instrument.nodeDataOutMsg)
    dataMonitor.addDataNodeToModel(testModule.nodeDataOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, dataMonitor)

    testModule.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsg)

    dataLog = testModule.nodeDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    generatedData = dataLog.baudRate

    print(generatedData)

    # compare the module results to the truth values
    accuracy = 1e-16

    trueData = 9600.  # Module should be on after enough data is accrued
    testArray = [0, 0, 0, trueData, trueData, trueData, trueData]   # Should go through three iterations of no data downlinked

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        testArray, generatedData, accuracy, "dataOutput",
        testFailCount, testMessages)

    if testFailCount:
        print(testMessages)
    else:
        print("Passed")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


def checkStatus():
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    testModule = simpleTransmitter.SimpleTransmitter()
    testModule.ModelTag = "transmitter"
    testModule.nodeBaudRate = 9600. # baud
    testModule.packetSize = -9600 # bits
    testModule.numBuffers = 1
    testModule.dataStatus = 0
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Create an instrument
    instrument = simpleInstrument.SimpleInstrument()
    instrument.ModelTag = "instrument1"
    instrument.nodeBaudRate = 1200. # baud
    instrument.nodeDataName = "Instrument 1" # baud
    unitTestSim.AddModelToTask(unitTaskName, instrument)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageCapacity = 8E9 # bits (1 GB)
    dataMonitor.addDataNodeToModel(instrument.nodeDataOutMsg)
    dataMonitor.addDataNodeToModel(testModule.nodeDataOutMsg)
    unitTestSim.AddModelToTask(unitTaskName, dataMonitor)

    testModule.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsg)

    # Setup logging on the test module output message so that we get all the writes to it

    dataLog = testModule.nodeDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    drawData = dataLog.baudRate

    # compare the module results to the truth values
    accuracy = 1e-16

    trueData = 0.0  # Module should be off

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        [trueData]*3, drawData, accuracy, "transmitterStatusTest",
        testFailCount, testMessages)

    if testFailCount:
        print(testMessages)
    else:
        print("Passed")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    checkDefault()
    checkStatus()