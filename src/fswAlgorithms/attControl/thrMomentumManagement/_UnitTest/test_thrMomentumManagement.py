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
#   Module Name:        thrMomentumManagement
#   Author:             Hanspeter Schaub
#   Creation Date:      August 18, 2016
#

import sys, os, inspect
import numpy as np
import pytest








# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import thrMomentumManagement            # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.utilities import fswSetupRW


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("hsMinCheck", [
    (0),
    (1)
])

# update "module" in this function name to reflect the module name
def test_thrMomentumManagement(show_plots, hsMinCheck):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = thrMomentumManagementTestFunction(show_plots, hsMinCheck)
    assert testResults < 1, testMessage


def thrMomentumManagementTestFunction(show_plots, hsMinCheck):
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
    moduleConfig = thrMomentumManagement.thrMomentumManagementConfig()                          # update with current values
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        thrMomentumManagement.Update_thrMomentumManagement,     # update with current values
                                        thrMomentumManagement.SelfInit_thrMomentumManagement,   # update with current values
                                        thrMomentumManagement.CrossInit_thrMomentumManagement,  # update with current values
                                        thrMomentumManagement.Reset_thrMomentumManagement)      # update with current values
    moduleWrap.ModelTag = "thrMomentumManagement"                                        # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    if hsMinCheck:
        moduleConfig.hs_min = 1000./6000.*100.               # Nms
    else:
        moduleConfig.hs_min = 100./6000.*100.               # Nms


    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    moduleConfig.vehicleConfigDataInMsgName = "vehicleConfigName"
    moduleConfig.rwSpeedsInMsgName = "reactionwheel_speeds"
    moduleConfig.rwConfigDataInMsgName = "rwa_config_data"
    moduleConfig.deltaHOutMsgName = "outputName"

    # wheelSpeeds Message
    rwSpeedMessage = thrMomentumManagement.RWSpeedIntMsg()
    inputMessageSize = rwSpeedMessage.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.rwSpeedsInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    rwSpeedMessage.wheelSpeeds = [10.0, -25.0, 50.0, 100.]
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.rwSpeedsInMsgName,
                                          inputMessageSize,
                                          0,
                                          rwSpeedMessage)

    # vehicleConfigData Message:
    vehicleConfigOut = thrMomentumManagement.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.vehicleConfigDataInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.vehicleConfigDataInMsgName,
                                          inputMessageSize,
                                          0,
                                          vehicleConfigOut)


    # wheelConfigData Message
    fswSetupRW.clearSetup()
    Js = 0.1
    fswSetupRW.create([1.0, 0.0, 0.0], Js)
    fswSetupRW.create([0.0, 1.0, 0.0], Js)
    fswSetupRW.create([0.0, 0.0, 1.0], Js)
    fswSetupRW.create([0.5773502691896258, 0.5773502691896258, 0.5773502691896258], Js)
    fswSetupRW.writeConfigMessage(moduleConfig.rwConfigDataInMsgName,
                                  unitTestSim.TotalSim,
                                  unitProcessName)



    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.deltaHOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # # reset the module to test this functionality
    # moduleWrap.Reset(1)     # this module reset function needs a time input (in NanoSeconds)
    #
    # # run the module again for an additional 1.0 seconds
    # unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    # unitTestSim.ExecuteSimulation()


    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "torqueRequestBody"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.deltaHOutMsgName + '.' + moduleOutputName,
                                                  range(3))

    print moduleOutput

    # set the filtered output truth states
    if hsMinCheck==1:
        trueVector = [
                   [0.0, 0.0, 0.0]
                   ]
    else:
        trueVector = [
                   [5.914369484146579,2.858300248464629,9.407020039211664]
                   ]*(-1)

        # else:
        #     testFailCount+=1
        #     testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed with unsupported input parameters")

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector[i], 3, accuracy):
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
    test_thrMomentumManagement(              # update "module" in function name
                 True,
                 0            # hsMinCheck
               )
