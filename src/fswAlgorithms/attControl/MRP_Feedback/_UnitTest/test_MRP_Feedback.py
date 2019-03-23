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








#   Import all of the modules that we are going to call in this simulation

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.fswAlgorithms import MRP_Feedback
from Basilisk.fswAlgorithms import fswMessages

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
#@pytest.mark.xfail()
# provide a unique test method name, starting with test_


@pytest.mark.parametrize("intGain", [0.01, -1])
@pytest.mark.parametrize("rwNum", [4, 0])
@pytest.mark.parametrize("integralLimit", [0, 20])
@pytest.mark.parametrize("useRwAvailability", ["NO", "ON", "OFF"])
@pytest.mark.parametrize("setdomega0", [(0., 0., 0.), (1., -1, 2.)])

def test_MRP_Feedback(show_plots, intGain, rwNum, integralLimit, useRwAvailability, setdomega0):
    # each test method requires a single assert method to be called

    [testResults, testMessage] = run(show_plots,intGain, rwNum, integralLimit, useRwAvailability, setdomega0)

    assert testResults < 1, testMessage


def run(show_plots, intGain, rwNum, integralLimit, useRwAvailability, setdomega0):
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
    if rwNum > 0:
        moduleConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    moduleConfig.inputRWSpeedsName = "reactionwheel_speeds"
    moduleConfig.outputDataName = "outputName"

    moduleConfig.K  =   0.15
    moduleConfig.Ki = intGain
    moduleConfig.P  = 150.0
    moduleConfig.integralLimit = integralLimit
    moduleConfig.domega0 = setdomega0
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
    jsList = []
    GsMatrix_B = []
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
        jsList = rwConfigParams.JsList
        GsMatrix_B = rwConfigParams.GsMatrix_B
        return jsList, GsMatrix_B

    if len(moduleConfig.rwParamsInMsgName) > 0:
        jsList, GsMatrix_B = writeMsgInWheelConfiguration()

    # wheelAvailability message
    rwAvailabilityMessage = fswMessages.RWAvailabilityFswMsg()
    if useRwAvailability is not "NO":
        moduleConfig.rwAvailInMsgName = "rw_availability"
        if useRwAvailability is "ON":
            rwAvailabilityMessage.wheelAvailability  = [MRP_Feedback.AVAILABLE, MRP_Feedback.AVAILABLE,
                                                        MRP_Feedback.AVAILABLE, MRP_Feedback.AVAILABLE]
        elif useRwAvailability is "OFF":
            rwAvailabilityMessage.wheelAvailability  = [MRP_Feedback.UNAVAILABLE, MRP_Feedback.UNAVAILABLE,
                                                        MRP_Feedback.UNAVAILABLE, MRP_Feedback.UNAVAILABLE]
        else:
            print "WARNING: unknown rw availability status"

        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                   unitProcessName,
                                   moduleConfig.rwAvailInMsgName,
                                   rwAvailabilityMessage)
    else:
        # set default availability
        rwAvailabilityMessage.wheelAvailability = [MRP_Feedback.AVAILABLE, MRP_Feedback.AVAILABLE,
                                                   MRP_Feedback.AVAILABLE, MRP_Feedback.AVAILABLE]


    LrTrue = findTrueTorques(moduleConfig, guidCmdData, rwSpeedMessage, vehicleConfigOut, jsList,
                             rwNum, GsMatrix_B, rwAvailabilityMessage)

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

    # compare the module results to the truth values
    accuracy = 1e-8
    for i in range(0, len(LrTrue)):
        # check vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], LrTrue[i], 3, accuracy):
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


def findTrueTorques(moduleConfig,guidCmdData,rwSpeedMessage,vehicleConfigOut,jsList,numRW,GsMatrix_B,rwAvailMsg):
    Lr = []

    #Read in variables
    L = np.asarray(moduleConfig.knownTorquePntB_B)
    steps = [0, 0, .5, 0, .5]
    omega_BR_B = np.asarray(guidCmdData.omega_BR_B)
    omega_RN_B = np.asarray(guidCmdData.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guidCmdData.domega_RN_B)
    sigma_BR = np.asarray(guidCmdData.sigma_BR)
    Isc = np.asarray(vehicleConfigOut.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = moduleConfig.Ki
    K = moduleConfig.K
    P = moduleConfig.P
    jsVec = jsList
    GsMatrix_B_array = np.asarray(GsMatrix_B)
    GsMatrix_B_array = np.reshape(GsMatrix_B_array[0:numRW * 3], (numRW, 3))
    sigmaInt = np.asarray([0, 0, 0])

    #Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            sigmaInt = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0: #if integral feedback is on
            sigmaInt = K * dt * sigma_BR + sigmaInt
            if np.linalg.norm(sigmaInt) > moduleConfig.integralLimit: #Make sure z is less than the intergralLimit to mitigate windup issues
                sigmaInt = sigmaInt/np.linalg.norm(sigmaInt)*moduleConfig.integralLimit
            zVec = sigmaInt + Isc.dot(omega_BR_B - moduleConfig.domega0)
        else: #integral gain turned off/negative setting
            zVec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = K * sigma_BR  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BR_B  # +P*deltaOmega
        Lr2 = Lr1 + P * Ki * zVec  # +P*Ki*z
        GsHs = np.array([0,0,0])

        if numRW>0:
            for i in range(numRW):
                if rwAvailMsg.wheelAvailability[i] == 0:  #Make RW availability check
                    GsHs = GsHs + np.dot(GsMatrix_B_array[i, :], jsVec[i]*(np.dot(omega_BN_B, GsMatrix_B_array[i, :])+rwSpeedMessage.wheelSpeeds[i]))
                    #J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        Lr3 = Lr2 - np.cross((omega_RN_B+Ki*zVec), (Isc.dot(omega_BN_B)+GsHs)) # -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s)

        Lr4 = Lr3 + Isc.dot(-domega_RN_B + np.cross(omega_BN_B, omega_RN_B)) #+[I](-d(omega_r)/dt + omega x omega_r)
        Lr5 = Lr4 + L
        Lr5 = -Lr5
        Lr.append(np.ndarray.tolist(Lr5))
    return Lr




#   This statement below ensures that the unitTestScript can be run as a stand-along python scripts
#   automatically executes the test_MRP_Feedback() method
#
if __name__ == "__main__":
    test_MRP_Feedback(False,    # showplots
                      0.01,     # intGain
                      4,        # rwNum
                      0.0,      # integralLimit
                      "NO",     # useRwAvailability ("NO", "ON", "OFF")
                      (0,0,0)   # setdomega0
                      )
