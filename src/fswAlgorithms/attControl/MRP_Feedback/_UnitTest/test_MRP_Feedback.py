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
#   Module Name:        MRP_Feedback
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
from Basilisk.fswAlgorithms import MRP_Feedback
# import module(s) that creates the needed input message declaration
from Basilisk.fswAlgorithms import rwMotorTorque

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
#@pytest.mark.xfail()
# provide a unique test method name, starting with test_


@pytest.mark.parametrize("intGain", [0.01, -1])
@pytest.mark.parametrize("rwNum", [4, 0])

def test_MRP_Feedback(show_plots, intGain, rwNum):
    # each test method requires a single assert method to be called

    [testResults, testMessage] = run(show_plots,intGain, rwNum)

    assert testResults < 1, testMessage


def run(show_plots, intGain, rwNum):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()          # this is needed if multiple unit test scripts are run
                                                        # this creates a fresh and consistent simulation environment for each test run

    #   Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    #   Construct algorithm and associated C++ container
    moduleConfig = MRP_Feedback.MRP_FeedbackConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "MRP_Feedback"

    #   Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    #   Initialize the test module configuration data

    moduleConfig.inputGuidName  = "inputGuidName"
    moduleConfig.vehConfigInMsgName  = "vehicleConfigName"
    moduleConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    moduleConfig.rwAvailInMsgName = "rw_availability"
    moduleConfig.inputRWSpeedsName = "reactionwheel_speeds"
    moduleConfig.outputDataName = "outputName"

    moduleConfig.K  =   0.15
    moduleConfig.Ki = intGain
    moduleConfig.P  = 150.0
    moduleConfig.integralLimit = 2./moduleConfig.Ki * 0.1
    moduleConfig.domega0 = [0., 0., 0.]
    moduleConfig.knownTorquePntB_B = [0., 0., 0.]


    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.

    #   AttGuidFswMsg Message:
    guidCmdData = MRP_Feedback.AttGuidFswMsg()  # Create a structure for the input message
    sigma_BR = [0.3, -0.5, 0.7]
    guidCmdData.sigma_BR = sigma_BR
    omega_BR_B = [0.010, -0.020, 0.015]
    guidCmdData.omega_BR_B = omega_BR_B
    omega_RN_B = [-0.02, -0.01, 0.005]
    guidCmdData.omega_RN_B = omega_RN_B
    domega_RN_B = [0.0002, 0.0003, 0.0001]
    guidCmdData.domega_RN_B = domega_RN_B
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.inputGuidName,
                               guidCmdData)

    # vehicleConfigData Message:
    vehicleConfigOut = MRP_Feedback.VehicleConfigFswMsg()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.vehConfigInMsgName,
                               vehicleConfigOut)

    # wheelSpeeds Message
    rwSpeedMessage = MRP_Feedback.RWSpeedIntMsg()
    Omega = [10.0, 25.0, 50.0, 100.0] # rad/sec
    rwSpeedMessage.wheelSpeeds = Omega
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.inputRWSpeedsName,
                               rwSpeedMessage)

    # wheelConfigData message
    def writeMsgInWheelConfiguration():
        rwConfigParams = MRP_Feedback.RWArrayConfigFswMsg()

        rwConfigParams.GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.577350269190, 0.577350269190, 0.577350269190
        ]
        rwConfigParams.JsList = [0.1, 0.1, 0.1, 0.1]
        rwConfigParams.numRW = rwNum
        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                   unitProcessName,
                                   moduleConfig.rwParamsInMsgName,
                                   rwConfigParams)

    if len(moduleConfig.rwParamsInMsgName)>0:
        writeMsgInWheelConfiguration()

    # wheelAvailability message
    def writeMsgInWheelAvailability():
        rwAvailabilityMessage = rwMotorTorque.RWAvailabilityFswMsg()

        avail = [rwMotorTorque.AVAILABLE, rwMotorTorque.AVAILABLE, rwMotorTorque.AVAILABLE, rwMotorTorque.AVAILABLE]
        rwAvailabilityMessage.wheelAvailability = avail
        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                   unitProcessName,
                                   moduleConfig.rwAvailInMsgName,
                                   rwAvailabilityMessage)

    if len(moduleConfig.rwAvailInMsgName)>0:
        writeMsgInWheelAvailability()



    #   Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    #   Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()
    
    moduleWrap.Reset(1)     # this module reset function needs a time input (in NanoSeconds) 
    
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()


    #   This pulls the actual data log from the simulation run.
    #   Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "torqueRequestBody"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                    range(3))
    # print '\n Lr = ', moduleOutput[:, 1:]


    # set the filtered output truth states
    if intGain == 0.01 and rwNum == 4:
        expectedVec = [
                [-18.98045163, 25.04949262, -21.68220099],
                [-18.98045163, 25.04949262, -21.68220099],
                [-19.01598385, 25.0980235, -21.76570084],
                [-18.98045163, 25.04949262, -21.68220099],
                [-19.01598385, 25.0980235, -21.76570084]]

    elif intGain ==0.01 and rwNum == 0:
        expectedVec = [
             [-16.115, 25.065, -23.495],
             [-16.115, 25.065, -23.495],
             [-16.14215, 25.1124, -23.5829],
             [-16.115, 25.065, -23.495],
             [-16.14215, 25.1124, -23.5829]]

    elif intGain == -1 and rwNum == 4:
        expectedVec = [
                [-1.58409754, 4.1143559, -1.59267836],
                [-1.58409754, 4.1143559, -1.59267836],
                [-1.58409754, 4.1143559, -1.59267836],
                [-1.58409754, 4.1143559, -1.59267836],
                [-1.58409754, 4.1143559, -1.59267836]]
    elif intGain == -1 and rwNum == 0:
        expectedVec = [
            [-1.435, 3.865, -1.495],
            [-1.435, 3.865, -1.495],
            [-1.435, 3.865, -1.495],
            [-1.435, 3.865, -1.495],
            [-1.435, 3.865, -1.495]]

    else:
         testFailCount += 1
         testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed with unsupported input parameters!")

    # compare the module results to the truth values
    accuracy = 1e-8
    for i in range(0, len(expectedVec)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],expectedVec[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName + " unit test at t=" + str(moduleOutput[i,0]*macros.NANO2SEC) + "sec\n")





    # print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED: " + moduleWrap.ModelTag
    else:
        print "Failed: " + moduleWrap.ModelTag


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]




#
#   This statement below ensures that the unitTestScript can be run as a stand-along python scripts
#   automatically executes the test_MRP_Feedback() method
#
if __name__ == "__main__":
    test_MRP_Feedback(False,    # showplots
                      0.01,     # intGain
                      4         # rwNum
                      )
