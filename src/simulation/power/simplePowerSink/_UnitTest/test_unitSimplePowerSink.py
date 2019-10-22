''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
#
#   Unit Test Script
#   Module Name:        simplePowerSink
#   Author:             Andrew Harris
#   Creation Date:      July 17th 2019
#

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
from Basilisk.simulation import simplePowerSink
from Basilisk.simulation import simMessages
from Basilisk.simulation import simFswInterfaceMessages
from Basilisk.utilities import macros

# update "module" in this function name to reflect the module name
def test_module():
    # each test method requires a single assert method to be called

    default_results, default_message = test_default()
    status_results, status_message = test_status()

    testResults = sum([default_results, status_results])
    testMessage = [default_message, status_message]

    assert testResults < 1, testMessage


def test_default():
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    testModule = simplePowerSink.SimplePowerSink()
    testModule.ModelTag = "powerSink"
    testModule.nodePowerOut = 10. # Watts
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    unitTestSim.TotalSim.logThisMessage(testModule.nodePowerOutMsgName, testProcessRate)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    drawData = unitTestSim.pullMessageLogData(testModule.nodePowerOutMsgName + ".netPower")

    # compare the module results to the truth values
    accuracy = 1e-16

    truePower = 10.0 #Module should be off

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        [truePower]*3, drawData, accuracy, "powerSinkOutput",
        testFailCount, testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


def test_status():
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    testModule = simplePowerSink.SimplePowerSink()
    testModule.ModelTag = "powerSink"
    testModule.nodeStatusInMsgName="PowerStatusMsg"
    testModule.nodePowerOut = 10. # Watts
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # create the input messages
    powerStatusMsg = simFswInterfaceMessages.PowerNodeStatusIntMsg()  # Create a structure for the input message
    powerStatusMsg.powerStatus=0
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               testModule.nodeStatusInMsgName,
                               powerStatusMsg)

    # Setup logging on the test module output message so that we get all the writes to it

    unitTestSim.TotalSim.logThisMessage(testModule.nodePowerOutMsgName, testProcessRate)

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
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    drawData = unitTestSim.pullMessageLogData(testModule.nodePowerOutMsgName + ".netPower")

    # compare the module results to the truth values
    accuracy = 1e-16

    truePower = 0.0 #Module should be off


    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        [truePower]*3, drawData, accuracy, "powerSinkStatusTest",
        testFailCount, testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    test_module()
