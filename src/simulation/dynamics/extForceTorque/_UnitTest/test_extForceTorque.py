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
#   Integrated Unit Test Script
#   Purpose:  Run the external ForceTorque dynEffector by specifying the forces and torques
#             in either messages and/or direct array input
#   Author:  Hanspeter Schaub
#   Creation Date:  Oct. 30, 2016
#

import pytest
import sys, os, inspect
import numpy as np
import ctypes
import math
import csv
import logging








from Basilisk.simulation import sim_model
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.simulation import extForceTorque






# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
# 0 - no external force/torque is set
# 1 - external force/torque is set directly from python as an Eigen Vector
# 2 - external force/torque is set through input message
# 3 - external force/torque is set through both a Class array and input message
caseList = []
for i in range(0, 4):
    for j in range(0, 4):
        for k in range(0, 4):
            caseList.append([i,j,k])
@pytest.mark.parametrize("torqueInput, forceNInput, forceBInput", caseList)


# provide a unique test method name, starting with test_
def test_unitDynamicsModes(show_plots, torqueInput, forceNInput, forceBInput):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitDynamicsModesTestFunction(
            show_plots, torqueInput, forceNInput, forceBInput)
    assert testResults < 1, testMessage



def unitDynamicsModesTestFunction(show_plots, torqueInput, forceNInput, forceBInput):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"
    unitProcessName = "testProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()


    #
    #  create the dynamics simulation process
    #

    dynProcess = scSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))


    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"

    if torqueInput==1 or torqueInput==3:
        extFTObject.extTorquePntB_B = [[-1], [1],[ -1]]
    if torqueInput==2 or torqueInput==3:
        msgName = "extTorquePntB_B_cmds"
        msgData = extForceTorque.CmdTorqueBodyIntMsg()
        msgData.torqueRequestBody = [-1.0, 1.0, -1.0]
        unitTestSupport.setMessage(scSim.TotalSim, unitProcessName, msgName, msgData)

    if forceNInput==1 or forceNInput==3:
        extFTObject.extForce_N = [[-10.], [-5.], [5.]]
    if forceNInput==2 or forceNInput==3:
        msgName = "extForce_N_cmds"
        msgData = extForceTorque.CmdForceInertialIntMsg()
        msgData.forceRequestInertial = [-10.0, -5.0, 5.0]
        unitTestSupport.setMessage(scSim.TotalSim, unitProcessName, msgName, msgData)

    if forceBInput==1 or forceBInput==3:
        extFTObject.extForce_B = [[10.], [20.], [30.]]
    if forceBInput==2 or forceBInput==3:
        msgName = "extForce_B_cmds"
        msgData = extForceTorque.CmdForceBodyIntMsg()
        msgData.forceRequestBody = [10.0, 20.0, 30.0]
        unitTestSupport.setMessage(scSim.TotalSim, unitProcessName, msgName, msgData)

    scSim.AddModelToTask(unitTaskName, extFTObject)

    #
    #   initialize the simulation
    #
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(0.001))


    #
    #   Setup data logging
    #
    testProcessRate = macros.sec2nano(0.1)
    variableForceN = "forceExternal_N"            # name the module variable to be logged
    scSim.AddVariableForLogging (extFTObject.ModelTag + "." + variableForceN, testProcessRate, 0, 2, 'double')
    variableForceB = "forceExternal_B"            # name the module variable to be logged
    scSim.AddVariableForLogging (extFTObject.ModelTag + "." + variableForceB, testProcessRate, 0, 2, 'double')
    variableTorque = "torqueExternalPntB_B"       # name the module variable to be logged
    scSim.AddVariableForLogging (extFTObject.ModelTag + "." + variableTorque, testProcessRate, 0, 2, 'double')

    #
    #   run the simulation
    #
    scSim.ExecuteSimulation()

    extFTObject.computeForceTorque(scSim.TotalSim.CurrentNanos)
    scSim.TotalSim.SingleStepProcesses()
    scSim.RecordLogVars()


    # log the data
    dataForceN = scSim.GetLogVariableData(extFTObject.ModelTag+"."+variableForceN)
    dataForceB = scSim.GetLogVariableData(extFTObject.ModelTag+"."+variableForceB)
    dataTorque = scSim.GetLogVariableData(extFTObject.ModelTag+"."+variableTorque)

    np.set_printoptions(precision=16)

    # Remove time zero from list
    dataForceN = dataForceN[1:len(dataForceN),:]
    dataForceB = dataForceB[1:len(dataForceB),:]
    dataTorque = dataTorque[1:len(dataTorque),:]


    #
    #   set true position information
    #

    if torqueInput == 3:
        trueTorque_B = [
              [-2., 2., -2.]
        ]
    elif torqueInput>0:
        trueTorque_B = [
            [-1., 1., -1.]
        ]
    else:
        trueTorque_B = [
            [0, 0, 0]
        ]

    if forceBInput == 3:
        trueForceB = [
            [20, 40, 60]
        ]
    elif forceBInput>0:
        trueForceB = [
            [10, 20, 30]
        ]
    else:
        trueForceB = [
            [0, 0, 0]
        ]

    if forceNInput == 3:
        trueForceN = [
            [-20., -10., 10.]
        ]
    elif forceNInput > 0:
        trueForceN = [
            [-10., -5., 5.]
        ]
    else:
        trueForceN = [
            [0, 0, 0]
        ]


    # compare the module results to the truth values
    accuracy = 1e-12
    if (len(trueTorque_B) != len(dataTorque)):
        testFailCount += 1
        testMessages.append("FAILED:  ExtForceTorque failed torque unit test (unequal array sizes)\n")
    else:
        for i in range(0,len(trueTorque_B)):
            # check a vector values
            if not unitTestSupport.isArrayEqual(dataTorque[i],trueTorque_B[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  ExtForceTorque failed torque unit test at t=" + str(dataTorque[i,0]*macros.NANO2SEC) + "sec\n")

    if (len(trueForceN) != len(dataForceN)):
        testFailCount += 1
        testMessages.append("FAILED:  ExtForceTorque failed force_N unit test (unequal array sizes)\n")
    else:
        for i in range(0,len(trueForceN)):
            # check a vector values
            if not unitTestSupport.isArrayEqual(dataForceN[i],trueForceN[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  ExtForceTorque failed force_N unit test at t=" + str(dataForceN[i,0]*macros.NANO2SEC) + "sec\n")

    if (len(trueForceB) != len(dataForceB)):
        testFailCount += 1
        testMessages.append("FAILED:  ExtForceTorque failed force_B unit test (unequal array sizes)\n")
    else:
        for i in range(0, len(trueForceB)):
            # check a vector values
            if not unitTestSupport.isArrayEqual(dataForceB[i], trueForceB[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  ExtForceTorque failed force_B unit test at t="+str(
                    dataForceB[i, 0]*macros.NANO2SEC)+"sec\n")

    #   print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED "

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_unitDynamicsModes(False,       # show_plots
                           1,           # torqueInput
                           0,           # forceNInput
                           0            # forceBInput
                           )
