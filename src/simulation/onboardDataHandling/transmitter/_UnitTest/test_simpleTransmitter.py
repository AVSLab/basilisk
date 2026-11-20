
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
from Basilisk.architecture import messaging
from Basilisk.simulation import simpleTransmitter, spaceToGroundTransmitter
from Basilisk.simulation import simpleInstrument
from Basilisk.simulation import partitionedStorageUnit
from Basilisk.utilities import macros


@pytest.fixture(params=[
    simpleTransmitter.SimpleTransmitter,
    spaceToGroundTransmitter.SpaceToGroundTransmitter,
])
def tx(request):
    """Configure a transmitter that needs three one-second steps per packet."""
    tx = request.param()
    tx.nodeBaudRate = -1.0  # [bit/s]
    tx.packetSize = -3.0  # [bit]
    if isinstance(tx, spaceToGroundTransmitter.SpaceToGroundTransmitter):
        silent_station = messaging.AccessMsg()
        access = messaging.AccessMsgPayload()
        access.hasAccess = 1
        active_station = messaging.AccessMsg().write(access)
        tx.addAccessMsgToTransmitter(silent_station)
        tx.addAccessMsgToTransmitter(active_station)
    return tx


def storage_status(names, bits):
    """Build a storage status with the given partition names and sizes in bits."""
    status = messaging.DataStorageStatusMsgPayload()
    status.storedDataName = messaging.StringVector(names)
    status.storedData = messaging.DoubleVector(bits)
    return status


def test_transmitter_waits_for_storage(tx):
    """Keep idle output zero and account for only the active steps of the first packet."""
    tx.UpdateState(0)  # [ns] No storage attached.
    assert tx.nodeDataOutMsg.read().baudRate == 0.0
    assert tx.nodeDataOutMsg.read().dataName == ""

    unused_storage = messaging.DataStorageStatusMsg()
    storage = messaging.DataStorageStatusMsg()
    tx.addStorageUnitToTransmitter(unused_storage)
    tx.addStorageUnitToTransmitter(storage)
    tx.UpdateState(macros.sec2nano(5.0))  # [ns] Storage has yet to publish.
    assert tx.nodeDataOutMsg.read().baudRate == 0.0
    assert tx.nodeDataOutMsg.read().dataName == ""

    status = storage_status(["A", "B"], [10.0, 0.0])  # [bit]
    storage.write(status)
    tx.UpdateState(macros.sec2nano(6.0))  # [ns]
    assert tx.nodeDataOutMsg.read().dataName == "A"
    # B becomes the largest partition after the first step of the packet from A.
    status.storedData = messaging.DoubleVector([10.0, 20.0])  # [bit]
    storage.write(status)
    for second, expected_name in [(7, "A"), (8, "A"), (9, "B")]:  # [s]
        tx.UpdateState(macros.sec2nano(second))
        usage = tx.nodeDataOutMsg.read()
        assert usage.baudRate == tx.nodeBaudRate
        assert usage.dataName == expected_name

    # Switching to an unwritten storage input interrupts the unfinished packet from B.
    replacement = messaging.DataStorageStatusMsg()
    tx.addStorageUnitToTransmitter(replacement)
    tx.UpdateState(macros.sec2nano(10.0))  # [ns]
    assert tx.nodeDataOutMsg.read().baudRate == 0.0
    assert tx.nodeDataOutMsg.read().dataName == ""
    status.storedData = messaging.DoubleVector([30.0, 20.0])  # [bit]
    replacement.write(status)
    tx.UpdateState(macros.sec2nano(11.0))  # [ns]
    assert tx.nodeDataOutMsg.read().dataName == "A"


@pytest.mark.parametrize("disconnected, expected_name, expected_rate", [
    (0, "B", -1.0), (1, "", 0.0),  # [bit/s]
], ids=["earlier", "selected"])
def test_disconnected_storage(tx, disconnected, expected_name, expected_rate):
    """A disconnected reader only stops transmission when it is the selected input."""
    for name in ("A", "B"):
        storage = messaging.DataStorageStatusMsg().write(storage_status([name], [10.0]))  # [bit]
        tx.addStorageUnitToTransmitter(storage)
    tx.storageUnitInMsgs[disconnected].unsubscribe()
    tx.UpdateState(macros.sec2nano(1.0))  # [ns]
    usage = tx.nodeDataOutMsg.read()
    assert (usage.dataName, usage.baudRate) == (expected_name, expected_rate)


@pytest.mark.parametrize("connection", ["append", "append-same-source", "resubscribe"])
def test_switch_to_written_storage(tx, connection):
    """Adding a storage input or observing a different source starts a new packet."""
    first = messaging.DataStorageStatusMsg().write(storage_status(["A"], [10.0]))  # [bit]
    tx.addStorageUnitToTransmitter(first)
    tx.UpdateState(macros.sec2nano(1.0))  # [ns]
    assert tx.nodeDataOutMsg.read().dataName == "A"

    replacement = first if connection == "append-same-source" else messaging.DataStorageStatusMsg()
    replacement.write(storage_status(["A", "B"], [10.0, 20.0]))  # [bit]
    if connection == "resubscribe":
        tx.storageUnitInMsgs[0].subscribeTo(replacement)
    else:
        tx.addStorageUnitToTransmitter(replacement)
    tx.UpdateState(macros.sec2nano(2.0))  # [ns]
    usage = tx.nodeDataOutMsg.read()
    assert (usage.dataName, usage.baudRate) == ("B", tx.nodeBaudRate)


def test_switch_to_empty_storage(tx):
    """A written storage unit with no partitions must not inherit the previous partition."""
    first = messaging.DataStorageStatusMsg().write(storage_status(["A"], [10.0]))  # [bit]
    tx.addStorageUnitToTransmitter(first)
    tx.UpdateState(macros.sec2nano(1.0))  # [ns]
    assert tx.nodeDataOutMsg.read().dataName == "A"

    empty = partitionedStorageUnit.PartitionedStorageUnit()
    empty.storageCapacity = 100  # [bit]
    empty.UpdateState(0)  # [ns] Publish the empty partition list.
    tx.addStorageUnitToTransmitter(empty.storageUnitDataOutMsg)
    empty.addDataNodeToModel(tx.nodeDataOutMsg)
    now = macros.sec2nano(2.0)  # [ns]
    tx.UpdateState(now)
    usage = tx.nodeDataOutMsg.read()
    assert (usage.dataName, usage.baudRate) == ("", 0.0)
    empty.UpdateState(now)
    assert list(empty.storageUnitDataOutMsg.read().storedDataName) == []


@pytest.mark.parametrize("observe_disconnect", [False, True], ids=["between-updates", "observed-disconnect"])
def test_restore_same_storage_reader(tx, observe_disconnect):
    """Restoring the same source restarts the packet only if an update observed the disconnect."""
    storage = messaging.DataStorageStatusMsg().write(storage_status(["A", "B"], [10.0, 5.0]))  # [bit]
    tx.addStorageUnitToTransmitter(storage)
    tx.UpdateState(macros.sec2nano(1.0))  # [ns]
    assert tx.nodeDataOutMsg.read().dataName == "A"

    storage.write(storage_status(["A", "B"], [10.0, 20.0]))  # [bit]
    readers = tx.storageUnitInMsgs
    saved = messaging.DataStorageStatusMsgInMsgsVector(readers)
    readers[0].unsubscribe()
    if observe_disconnect:
        tx.UpdateState(macros.sec2nano(2.0))  # [ns]
        usage = tx.nodeDataOutMsg.read()
        assert (usage.dataName, usage.baudRate) == ("", 0.0)
    readers[0] = saved[0]
    tx.UpdateState(macros.sec2nano(3.0))  # [ns]
    usage = tx.nodeDataOutMsg.read()
    assert (usage.dataName, usage.baudRate) == ("B" if observe_disconnect else "A", tx.nodeBaudRate)


@pytest.mark.parametrize("active", [False, True], ids=["idle", "partial-packet"])
def test_reinitialize_transmitter(tx, active):
    """Restart the packet and its clock at the reset epoch, including after an active downlink."""
    sim = SimulationBaseClass.SimBaseClass()
    sim.CreateNewProcess("process").addTask(sim.CreateNewTask("task", macros.sec2nano(1.0)))  # [ns]
    sim.AddModelToTask("task", tx)
    storage = messaging.DataStorageStatusMsg()
    tx.addStorageUnitToTransmitter(storage)
    if active:
        storage.write(storage_status(["A", "B"], [0.0, 30.0]))  # [bit]
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(5.0))  # [ns]
    sim.ExecuteSimulation()
    if active:
        assert tx.nodeDataOutMsg.read().dataName == "B"

    storage.write(storage_status(["A", "B"], [10.0, 5.0]))  # [bit]
    sim.InitializeSimulation()
    sim.ConfigureStopTime(0)  # [ns]
    sim.ExecuteSimulation()
    usage = tx.nodeDataOutMsg.read()
    assert (usage.dataName, usage.baudRate) == ("A", tx.nodeBaudRate)
    sim.ConfigureStopTime(macros.sec2nano(1.0))  # [ns]
    sim.ExecuteSimulation()
    storage.write(storage_status(["A", "B"], [10.0, 20.0]))  # [bit]
    for second, name in [(2, "A"), (3, "A"), (4, "B")]:  # [s]
        sim.ConfigureStopTime(macros.sec2nano(second))
        sim.ExecuteSimulation()
        usage = tx.nodeDataOutMsg.read()
        assert (usage.dataName, usage.baudRate) == (name, tx.nodeBaudRate)

    storage.write(storage_status([], []))
    tx.Reset(0)  # [ns]
    tx.UpdateState(0)  # [ns]
    usage = tx.nodeDataOutMsg.read()
    assert (usage.dataName, usage.baudRate) == ("", 0.0)


@pytest.mark.parametrize("function", ["checkDefault"
                                      , "checkStatus"
                                      ])
def test_simpleTransmitterAll(show_plots, function):
    """Module Unit Test"""
    func = globals().get(function)

    if func is None:
        raise ValueError(f"Function '{function}' not found in global scope")

    [testResults, testMessage] = func()

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
