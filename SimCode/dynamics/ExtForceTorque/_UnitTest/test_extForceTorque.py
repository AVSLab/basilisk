'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import ctypes
import math
import csv
import logging


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')

import sim_model
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import macros
import ExtForceTorque






# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
# 0 - no external force/torque is set
# 1 - external force/torque is set directly in Class array
# 2 - external force/torque is set through input message
# 3 - external force/torque is set through both a Class array and input message
caseList = []
for i in range(0, 4):
    for j in range(0, 4):
        for k in range(0, 4):
            caseList.append([i,j,k])
# @pytest.mark.parametrize("torqueInput, forceNInput, forceBInput", caseList)
@pytest.mark.parametrize("torqueInput, forceNInput, forceBInput", [
    (0,0,0)
])

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


    extFTObject = ExtForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"

    if torqueInput==1 or torqueInput==3:
        extFTObject.extTorquePntB_B = [-1, 1, -1]
    if torqueInput==2 or torqueInput==3:
        msgName = "extTorquePntB_B_cmds"
        scSim.TotalSim.CreateNewMessage(unitProcessName, msgName, 8*3, 2)
        cmdArray = sim_model.new_doubleArray(3)
        sim_model.doubleArray_setitem(cmdArray, 0,-1.0)
        sim_model.doubleArray_setitem(cmdArray, 1, 1.0)
        sim_model.doubleArray_setitem(cmdArray, 2,-1.0)
        scSim.TotalSim.WriteMessageData(msgName, 8*3, 1, cmdArray )

    if forceNInput==1 or forceNInput==3:
        extFTObject.extForce_N = [-10., -5., 5.]
    if forceNInput==2 or forceNInput==3:
        msgName = "extForce_N_cmds"
        scSim.TotalSim.CreateNewMessage(unitProcessName, msgName, 8*3, 2)
        cmdArray = sim_model.new_doubleArray(3)
        sim_model.doubleArray_setitem(cmdArray, 0,-10.)
        sim_model.doubleArray_setitem(cmdArray, 1, -5.)
        sim_model.doubleArray_setitem(cmdArray, 2,  5.)
        scSim.TotalSim.WriteMessageData(msgName, 8*3, 1, cmdArray)

    if forceBInput==1 or forceBInput==3:
        extFTObject.extForce_B = [10., 20., 30.]
    if forceBInput==2 or forceBInput==3:
        msgName = "extForce_B_cmds"
        scSim.TotalSim.CreateNewMessage(unitProcessName, msgName, 8*3, 2)
        cmdArray = sim_model.new_doubleArray(3)
        sim_model.doubleArray_setitem(cmdArray, 0, 10.)
        sim_model.doubleArray_setitem(cmdArray, 1, 20.)
        sim_model.doubleArray_setitem(cmdArray, 2, 30.)
        scSim.TotalSim.WriteMessageData(msgName, 8*3, 1, cmdArray)

    scSim.AddModelToTask(unitTaskName, extFTObject)

    #
    #   Setup data logging
    #
    testProcessRate = macros.sec2nano(0.1);
    # scSim.TotalSim.logThisMessage("inertial_state_output", macros.sec2nano(testProcessRate))
    variableForceN = "dynEffectorForce_N"            # name the module variable to be logged
    scSim.AddVariableForLogging(extFTObject.ModelTag + "." + variableForceN, testProcessRate)
    variableForceB = "dynEffectorForce_B"            # name the module variable to be logged
    scSim.AddVariableForLogging(extFTObject.ModelTag + "." + variableForceB, testProcessRate)
    variableTorque = "dynEffectorTorquePntB_B"       # name the module variable to be logged
    scSim.AddVariableForLogging(extFTObject.ModelTag + "." + variableTorque, testProcessRate)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(0.1))
    scSim.ExecuteSimulation()

    # log the data
    # dataSigma = scSim.pullMessageLogData("inertial_state_output.sigma", range(3))
    # dataPos = scSim.pullMessageLogData("inertial_state_output.r_N", range(3))
    dataForceN = scSim.GetLogVariableData(extFTObject.ModelTag+"."+variableForceN)
    dataForceB = scSim.GetLogVariableData(extFTObject.ModelTag+"."+variableForceB)
    dataTorque = scSim.GetLogVariableData(extFTObject.ModelTag+"."+variableTorque)

    np.set_printoptions(precision=16)


    # Remove time zero from list
    # dataPos = dataPos[1:len(dataPos),:]
    # dataSigma = dataSigma[1:len(dataSigma),:]

    print dataForceN

    #
    #   set true position information
    #
    if forceBInput and not forceBInput==3 and torqueInput and not torqueInput==3 and not forceNInput:
        trueForceN = [
                      [-4.0463169323943728e+06, 7.4733456548928013e+06, 5.2534810749004250e+06]
                    , [-4.0722537044125204e+06, 7.4560493935972452e+06, 5.2586109125909992e+06]
                ]
    elif forceBInput and not forceBInput==3 and torqueInput == 3 and not forceNInput:
        trueForceN = [
              [-4.0463169316231445e+06, 7.4733456542484760e+06, 5.2534810722646564e+06]
            , [-4.0722536909460793e+06, 7.4560493838592488e+06, 5.2586108703443157e+06]
        ]
    elif forceBInput and not forceBInput==3 and torqueInput == 3 and forceNInput and not forceNInput==3:
        trueForceN = [
              [-4.0463170982897207e+06, 7.4733455709153023e+06, 5.2534811555979857e+06]
            , [-4.0722543576113195e+06, 7.4560490505284928e+06, 5.2586112036775714e+06]
        ]
    elif forceBInput==3 and torqueInput == 3 and not forceNInput:
        trueForceN = [
              [-4.0463164166177893e+06, 7.4733453698796201e+06, 5.2534812786559518e+06]
            , [-4.0722516080985265e+06, 7.4560483028106857e+06, 5.2586117110610902e+06]
        ]
    elif forceBInput == 3 and not torqueInput and forceNInput and not forceNInput==3:
        trueForceN = [
              [-4.0463165864312178e+06, 7.4733452891314961e+06, 5.2534813725136584e+06]
            , [-4.0722523325808612e+06, 7.4560480087901447e+06, 5.2586122120233215e+06]
        ]
    elif forceBInput == 3 and not torqueInput and not forceNInput:
        trueForceN = [
              [-4.0463164197646393e+06, 7.4733453724646717e+06, 5.2534812891803300e+06]
            , [-4.0722516659156200e+06, 7.4560483421209017e+06, 5.2586118786900649e+06]
        ]
    elif forceBInput==3 and torqueInput and not torqueInput == 3 and not forceNInput:
        trueForceN = [
              [-4.0463164181602467e+06, 7.4733453711682595e+06, 5.2534812839274779e+06]
            , [-4.0722516350314110e+06, 7.4560483222866654e+06, 5.2586117955544451e+06]
        ]
    elif forceBInput and not forceBInput==3 and not torqueInput and not forceNInput:
        trueForceN = [
                      [-4.0463169331965689e+06, 7.4733456555410009e+06, 5.2534810775268460e+06]
                    , [-4.0722537198546235e+06, 7.4560494035143545e+06, 5.2586109541588034e+06]
                ]
    elif forceBInput==3 and torqueInput == 3 and forceNInput==3:
        trueForceN = [
              [-4.0463167499509458e+06, 7.4733452032132689e+06, 5.2534814453226039e+06]
            , [-4.0722529414290129e+06, 7.4560476361491689e+06, 5.2586123777275961e+06]
        ]
    elif forceBInput == 3 and torqueInput and not torqueInput == 3 and forceNInput == 3:
        trueForceN = [
              [-4.0463167514934023e+06, 7.4733452045019185e+06, 5.2534814505941356e+06]
            , [-4.0722529683618979e+06, 7.4560476556251580e+06, 5.2586124622209538e+06]
        ]

    elif forceBInput == 3 and torqueInput == 3 and forceNInput and not forceNInput == 3:
        trueForceN = [
              [-4.0463165832843678e+06, 7.4733452865464399e+06, 5.2534813619892774e+06]
            , [-4.0722522747637695e+06, 7.4560479694799213e+06, 5.2586120443943422e+06]
        ]
    elif forceBInput == 3 and torqueInput and not torqueInput == 3 and forceNInput and not forceNInput == 3:
        trueForceN = [
              [-4.0463165848268252e+06, 7.4733452878350904e+06, 5.2534813672608081e+06]
            , [-4.0722523016966567e+06, 7.4560479889559140e+06, 5.2586121288877027e+06]
        ]
    elif forceNInput and not forceNInput==3 and not forceBInput:
        trueForceN = [
                      [-4.0463176132950732e+06, 7.4733458552841591e+06,  5.2534809492066912e+06]
                    , [-4.0722564404588663e+06, 7.4560501315770540e+06,  5.2586103629607968e+06]
                ]
    elif forceNInput== 3 and not forceBInput:
        trueForceN = [
              [-4.0463177799616507e+06, 7.4733457719509872e+06, 5.2534810325400205e+06]
            , [-4.0722571071241098e+06, 7.4560497982463026e+06, 5.2586106962940544e+06]
        ]
    elif forceNInput == 3 and forceBInput==3 and not torqueInput:
        trueForceN = [
              [-4.0463167530977940e+06, 7.4733452057983251e+06, 5.2534814558469830e+06]
            , [-4.0722529992461042e+06, 7.4560476754593877e+06, 5.2586125453565726e+06]
        ]
    elif forceNInput == 3 and forceBInput and not forceBInput==3 and torqueInput==3:
        trueForceN = [
              [-4.0463172649562974e+06, 7.4733454875821304e+06, 5.2534812389313169e+06]
            , [-4.0722550242765583e+06, 7.4560487171977386e+06, 5.2586115370108327e+06]
        ]
    elif forceNInput == 3 and forceBInput and not forceBInput==3 and torqueInput and not torqueInput==3:
        trueForceN = [
              [-4.0463172657275270e+06, 7.4733454882264519e+06, 5.2534812415670753e+06]
            , [-4.0722550377430045e+06, 7.4560487269357294e+06, 5.2586115792575041e+06]
        ]
    elif forceBInput and not forceBInput==3 and forceNInput and not forceNInput==3 and torqueInput and not torqueInput==3:
        trueForceN = [
                      [-4.0463170990609489e+06, 7.4733455715596257e+06, 5.2534811582337497e+06]
                    , [-4.0722543710777601e+06, 7.4560490602664845e+06, 5.2586112459242502e+06]
                ]
    elif forceBInput and not forceBInput==3 and forceNInput and not forceNInput==3 and not torqueInput:
        trueForceN = [
                      [-4.0463170998631455e+06, 7.4733455722078299e+06, 5.2534811608601771e+06]
                    , [-4.0722543865198651e+06, 7.4560490701835994e+06, 5.2586112874920620e+06]
                ]
    elif forceBInput and not forceBInput==3 and forceNInput==3 and not torqueInput:
        trueForceN = [
              [-4.0463172665297249e+06, 7.4733454888746552e+06, 5.2534812441935046e+06]
            , [-4.0722550531851086e+06, 7.4560487368528442e+06, 5.2586116208253168e+06]
        ]
    else: # natural translation
        trueForceN = [
                     [-4.0463174466284965e+06, 7.4733459386173338e+06, 5.2534808658733666e+06]
                    ,[-4.0722557737936252e+06, 7.4560504649078129e+06, 5.2586100296275448e+06]
                    ]

    #
    #   set true attitude information
    #
    if torqueInput==1 or torqueInput==2:
        trueTorque_B = [
                      [ 1.0309611264147077e-01, 1.8303698007460178e-01, -2.6640848671012080e-01]
                    , [ 1.0090551379486196e-01, 1.8043272125508222e-01, -2.4021749832085976e-01]
                    ]
    elif torqueInput==3:
        trueTorque_B = [
                      [ 1.0082018046016203e-01, 1.9016757756711092e-01, -2.6985587162024699e-01]
                    , [ 9.1353606836461987e-02, 2.0861673338936709e-01, -2.5371926456051602e-01]
                    ]
    else: # without torques
        trueTorque_B = [
                     [1.0531951264579087e-01, 1.7592737942758579e-01, -2.6295400835245564e-01]
                    ,[1.0963176719705761e-01, 1.5254256011876685e-01, -2.2659471990463115e-01]
                    ]


    # compare the module results to the truth values
    accuracy = 1e-12
    if (len(trueTorque_B) != len(dataTorque)):
        testFailCount += 1
        testMessages.append("FAILED:  ExtForceTorque failed attitude unit test (unequal array sizes)\n")
    else:
        for i in range(0,len(trueTorque_B)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataTorque[i],trueTorque_B[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  ExtForceTorque failed torque unit test at t=" + str(dataTorque[i,0]*macros.NANO2SEC) + "sec\n")

    if (len(trueForceN) != len(dataForceN)):
        testFailCount += 1
        testMessages.append("FAILED:  ExtForceTorque failed force_N unit test (unequal array sizes)\n")
    else:
        for i in range(0,len(trueForceN)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataForceN[i],trueForceN[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  ExtForceTorque failed force_N unit test at t=" + str(dataForceN[i,0]*macros.NANO2SEC) + "sec\n")


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
                           3,           # torqueInput
                           0,           # forceNInput
                           1            # forceBInput
                           )

