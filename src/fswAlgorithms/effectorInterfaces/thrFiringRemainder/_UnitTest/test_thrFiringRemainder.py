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
#   Module Name:        thrFiringRemainder
#   Author:             John Alcorn, modified code by Hanspeter Schaub
#   Creation Date:      August 25, 2016
#

import sys, os, inspect
import numpy as np
import pytest








# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import thrFiringRemainder            # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.utilities import fswSetupThrusters


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("resetCheck, dvOn", [
    (False,False),
    (True,False),
    (False,True),
    (True,True)
])

# update "module" in this function name to reflect the module name
def test_thrFiringRemainder(show_plots, resetCheck, dvOn):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = thrFiringRemainderTestFunction(show_plots, resetCheck, dvOn)
    assert testResults < 1, testMessage


def thrFiringRemainderTestFunction(show_plots, resetCheck, dvOn):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = thrFiringRemainder.thrFiringRemainderConfig()                          # update with current values
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "thrFiringRemainder"                                        # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.thrMinFireTime = 0.2
    if dvOn == 1:
        moduleConfig.baseThrustState = 1
    else:
        moduleConfig.baseThrustState = 0

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    moduleConfig.thrConfInMsgName = "rcs_config_data"
    moduleConfig.thrForceInMsgName = "thr_config_data"
    moduleConfig.onTimeOutMsgName = "outputName"


    # setup thruster cluster message
    fswSetupThrusters.clearSetup()
    rcsLocationData = [ \
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070] \
        ]
    rcsDirectionData = [ \
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0] \
        ]

    for i in range(len(rcsLocationData)):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], 0.5)
    fswSetupThrusters.writeConfigMessage(  moduleConfig.thrConfInMsgName,
                                           unitTestSim.TotalSim,
                                           unitProcessName)
    numThrusters = fswSetupThrusters.getNumOfDevices()

    # setup thruster impulse request message
    inputMessageData = thrFiringRemainder.THRArrayCmdForceFswMsg()
    messageSize = inputMessageData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.thrForceInMsgName,
                                          messageSize,
                                          2)


    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.onTimeOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    # unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation


    if dvOn:
        # inputMessageData.thrForce = [0.0, -0.1, -0.2, -0.3, -0.349, -0.351, -0.451, -0.5]
        inputMessageData.thrForce = [-0.5, 0.0, -0.1, -0.2, -0.3, -0.34, -0.39, -0.44]
    else:
        inputMessageData.thrForce = [0.5, 0.05, 0.1, 0.15, 0.19, 0.0, 0.2, 0.49]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.thrForceInMsgName,
                                          messageSize,
                                          0,
                                          inputMessageData)
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.5))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    if resetCheck:
        # reset the module to test this functionality
        moduleWrap.Reset(macros.sec2nano(3.0))     # this module reset function needs a time input (in NanoSeconds)

        # run the module again for an additional 1.0 seconds
        unitTestSim.ConfigureStopTime(macros.sec2nano(5.5))        # seconds to stop simulation
        unitTestSim.ExecuteSimulation()


    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "OnTimeRequest"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.onTimeOutMsgName + '.' + moduleOutputName,
                                                  range(numThrusters))
    # print moduleOutput

    # set the filtered output truth states
    if resetCheck==1:
        if dvOn:
            trueVector = [
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.24],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.24],
                   ]
        else:
            trueVector = [
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
                   [0.55, 0.0, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
                   [0.55, 0.2, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
                   [0.55, 0.0, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
                   [0.55, 0.2, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
                   ]

    else:
        if dvOn:
            trueVector = [
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.24],
                   [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
                   ]
        else:
            trueVector = [
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
                   [0.55, 0.0, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
                   [0.55, 0.2, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
                   [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
                   ]

        # else:
        #     testFailCount+=1
        #     testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed with unsupported input parameters")

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector[i], numThrusters, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0]*macros.NANO2SEC) +
                                "sec\n")


    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    # if show_plots:
        # plot a sample variable.
        # plt.figure(1)
        # plt.plot(variableState[:,0]*macros.NANO2SEC, variableState[:,1], label='Sample Variable')
        # plt.legend(loc='upper left')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Variable Description [unit]')
        # plt.show()

    #   print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thrFiringRemainder(
                 True,            # show plots
                 False,           #
                 False
               )
