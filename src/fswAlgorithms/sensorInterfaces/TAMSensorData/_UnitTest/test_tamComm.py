''' '''
'''
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#   Module Name:        tamComm
#   Author:             Demet Cilden-Guler
#   Creation Date:      October 22, 2019
#

import pytest
import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.fswAlgorithms import tamComm
from Basilisk.utilities import macros
from Basilisk.simulation import simFswInterfaceMessages

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.

# update "module" in this function name to reflect the module name
def test_module(show_plots):
    """
    Validation Test Description
    ---------------------------
    This section describes the specific unit tests conducted on this module. \
    The test contains 1 test and is located at 'test_tamComm.py'. \
    The success criteria is to match the outputs with the generated truth.

    Description of Variables Being Tested
    -------------------------------------
    In this file, we are checking the values of the variable:

        tam_B

    which is pulled from the log data from TAMSensorBodyFswMsg to see if they match with the expected truth values.
    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = tamCommTestFunction(show_plots)
    assert testResults < 1, testMessage

def tamCommTestFunction(show_plots):
    """ Test the tamComm module """
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = tamComm.tamConfigData()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "tamComm"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.tamOutMsgName = "tamSampleOutput"
    moduleConfig.tamInMsgName = "tamSampleInput"
    dcm3, _ = np.linalg.qr(np.random.normal(0, 1, (3, 3)))
    moduleConfig.dcm_BS = dcm3.reshape(9, 1)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = simFswInterfaceMessages.TAMSensorIntMsg()
    inputMessageData.tam_S = [-1e-5, 2e-6, -3e-5]  # Tesla
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.tamInMsgName,
                               inputMessageData)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.tamOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # compare the module results to the truth values
    accuracy = 1e-12

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "tam_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.tamOutMsgName + '.' + moduleOutputName,
                                                  list(range(3)))

    # set the filtered output truth states
    trueVector = [
        [-1e-5, 2e-6, -3e-5],
        [-1e-5, 2e-6, -3e-5],
        [-1e-5, 2e-6, -3e-5]
    ]
    for i in range(len(trueVector)):
        trueVector[i] = np.dot(dcm3, trueVector[i])

    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "TAM Output Vector",
                                                               testFailCount, testMessages)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag)
        print("This test uses an accuracy value of " + str(accuracy))
    else:
        print("Failed: " + moduleWrap.ModelTag)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(False)
