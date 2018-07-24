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
#   Module Name:        PRV_Steering
#   Author:             Hanspeter Schaub
#   Creation Date:      December 18, 2015
#
import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.
import numpy as np
import ctypes
import math
import logging








#   Import all of the modules that we are going to call in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
# general support files with common unit test functions
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
import matplotlib.pyplot as plt
# import the module that is to be tested
from Basilisk.fswAlgorithms import PRV_Steering
from Basilisk.fswAlgorithms import rateServoFullNonlinear
# import module(s) that creates the needed input message declaration
from Basilisk.fswAlgorithms import fswMessages
from Basilisk.simulation import simFswInterfaceMessages

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_PRV_Steering(show_plots):     # update "subModule" in this function name to reflect the module name
    # each test method requires a single assert method to be called
    [testResults, testMessage] = subModuleTestFunction(show_plots)
    assert testResults < 1, testMessage


def subModuleTestFunction(show_plots):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()          # this is needed if multiple unit test scripts are run
                                                        # this create a fresh and consistent simulation environment for each test run

    #   Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    #   Construct algorithm and associated C++ container
    moduleConfig = PRV_Steering.PRV_SteeringConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "PRV_Steering"

    servoConfig = rateServoFullNonlinear.rateServoFullNonlinearConfig()
    servoWrap = unitTestSim.setModelDataWrap(servoConfig)
    servoWrap.ModelTag = "rate_servo"

    #   Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    unitTestSim.AddModelToTask(unitTaskName, servoWrap, servoConfig)

    #   Initialize the test module configuration data

    moduleConfig.inputGuidName  = "inputGuidName"
    moduleConfig.outputDataName = "rate_steering"
    servoConfig.inputGuidName = moduleConfig.inputGuidName
    servoConfig.vehConfigInMsgName = "vehicleConfigName"
    servoConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    servoConfig.rwAvailInMsgName = "rw_availability"
    servoConfig.inputRWSpeedsName = "reactionwheel_speeds"
    servoConfig.inputRateSteeringName = moduleConfig.outputDataName
    servoConfig.outputDataName = "outputName"

    moduleConfig.K1 =   0.15
    moduleConfig.K3 =   1.0
    moduleConfig.omega_max = 1.5*macros.D2R
    servoConfig.Ki =   0.01
    servoConfig.P  = 150.0
    servoConfig.integralLimit = 2./servoConfig.Ki * 0.1;
    servoConfig.knownTorquePntB_B = [0., 0., 0.]


    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.

    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.

    #   attGuidOut Message:
    guidCmdData = fswMessages.AttGuidFswMsg()  # Create a structure for the input message
    inputMessageSize = guidCmdData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName, servoConfig.inputGuidName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    sigma_BR = np.array([0.3, -0.5, 0.7])
    guidCmdData.sigma_BR = sigma_BR
    omega_BR_B = np.array([0.010, -0.020, 0.015])
    guidCmdData.omega_BR_B = omega_BR_B
    omega_RN_B = np.array([-0.02, -0.01, 0.005])
    guidCmdData.omega_RN_B = omega_RN_B
    domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    guidCmdData.domega_RN_B = domega_RN_B
    unitTestSim.TotalSim.WriteMessageData(servoConfig.inputGuidName, inputMessageSize,
                                          0, guidCmdData)

    # vehicleConfigData Message:
    vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName, servoConfig.vehConfigInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    unitTestSim.TotalSim.WriteMessageData(servoConfig.vehConfigInMsgName,
                                          inputMessageSize,
                                          0, vehicleConfigOut)

    # wheelSpeeds Message
    rwSpeedMessage = simFswInterfaceMessages.RWSpeedIntMsg()
    inputMessageSize = rwSpeedMessage.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          servoConfig.inputRWSpeedsName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    Omega = [10.0, 25.0, 50.0, 100.0]
    rwSpeedMessage.wheelSpeeds = Omega
    unitTestSim.TotalSim.WriteMessageData(servoConfig.inputRWSpeedsName,
                                          inputMessageSize,
                                          0,
                                          rwSpeedMessage)

    # wheelConfigData message
    def writeMsgInWheelConfiguration():
        rwConfigParams = fswMessages.RWArrayConfigFswMsg()
        inputMessageSize = rwConfigParams.getStructSize()
        unitTestSim.TotalSim.CreateNewMessage(unitProcessName, servoConfig.rwParamsInMsgName,
                                              inputMessageSize, 2)  # number of buffers (leave at 2 as default)
        rwConfigParams.GsMatrix_B = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]
        rwConfigParams.JsList = [0.1, 0.1, 0.1, 0.1]
        rwConfigParams.numRW = 4
        unitTestSim.TotalSim.WriteMessageData(servoConfig.rwParamsInMsgName, inputMessageSize,
                                              0, rwConfigParams)

    if len(servoConfig.rwParamsInMsgName) > 0:
        writeMsgInWheelConfiguration()

    # wheelAvailability message
    def writeMsgInWheelAvailability():
        rwAvailabilityMessage = rateServoFullNonlinear.RWAvailabilityFswMsg()
        inputMessageSize = rwAvailabilityMessage.getStructSize()
        unitTestSim.TotalSim.CreateNewMessage(unitProcessName, servoConfig.rwAvailInMsgName,
                                              inputMessageSize, 2)  # number of buffers (leave at 2 as default)
        avail = [rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE]
        rwAvailabilityMessage.wheelAvailability = avail
        unitTestSim.TotalSim.WriteMessageData(servoConfig.rwAvailInMsgName, inputMessageSize,
                                              0, rwAvailabilityMessage)

    if len(servoConfig.rwAvailInMsgName) > 0:
        writeMsgInWheelAvailability()


    #   Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(servoConfig.outputDataName, testProcessRate)

    #   Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    servoWrap.Reset(1)     # this module reset function needs a time input (in NanoSeconds)
    
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()


    #   This pulls the actual data log from the simulation run.
    #   Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "torqueRequestBody"
    moduleOutput = unitTestSim.pullMessageLogData(servoConfig.outputDataName + '.' + moduleOutputName,
                                                    range(3))
    print '\n Lr = ', moduleOutput[:, 1:]

    # set the filtered output truth states
    trueVector = [
               [-2.9352922876097969, +6.2831737715827778, -4.0554726129822907]
              ,[-2.9352922876097969, +6.2831737715827778, -4.0554726129822907]
              ,[-2.9353853745179044, +6.2833455830962901, -4.0556481491012084]
              ,[-2.9352922876097969, +6.2831737715827778, -4.0554726129822907]
              ,[-2.9353853745179044, +6.2833455830962901, -4.0556481491012084]
               ]

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName + " unit test at t=" + str(moduleOutput[i,0]*macros.NANO2SEC) + "sec\n")





    ## plot a sample variable
    #plt.figure(1)
    #plt.plot(dummyState[:,0]*macros.NANO2SEC, dummyState[:,1], label='Sample Variable')
    #plt.legend(loc='upper left')
    #plt.xlabel('Time [s]')
    #plt.ylabel('Variable Description [unit]')





    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
          plt.show()

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]





#
#   This statement below ensures that the unitTestScript can be run as a stand-along python scripts
#   authmatically executes the runUnitTest() method
#
if __name__ == "__main__":
    test_PRV_Steering(False)
