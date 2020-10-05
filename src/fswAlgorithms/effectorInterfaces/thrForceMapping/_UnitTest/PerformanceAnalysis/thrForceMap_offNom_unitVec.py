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
#   Module Name:        thrForceMapping
#   Author:             Hanspeter Schaub
#   Creation Date:      July 4, 2016
#

import pytest

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.utilities import macros
from Basilisk.utilities import fswSetupThrusters
from Basilisk.simulation import simFswInterfaceMessages

from fswAlgorithms.effectorInterfaces.thrForceMapping._UnitTest.Support import Results_thrForceMapping

import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.

@pytest.mark.parametrize("useDVThruster", [True, False])
@pytest.mark.parametrize("useCOMOffset", [True, False])
@pytest.mark.parametrize("dropThruster, asymmetricDrop", [ #asymmetric drop makes sure the thrusters that are lost aren't symmetric around the COM
    (0, False),
    (1, False),
    (2, False),
    (2, True),
    (3, False),
    (4, False),
    (4, True)
    ]) # Odd drops already incorporate symmetry so no need to test for them.
@pytest.mark.parametrize("numControlAxis", [1, 2, 3])
@pytest.mark.parametrize("saturateThrusters", [0, 1, 2])
@pytest.mark.parametrize("misconfigThruster", [False])



# update "module" in this function name to reflect the module name
def test_module(show_plots, useDVThruster, useCOMOffset, dropThruster, asymmetricDrop, numControlAxis, saturateThrusters, misconfigThruster):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = thrusterForceTest(show_plots, useDVThruster, useCOMOffset, dropThruster, asymmetricDrop,
                                                   numControlAxis, saturateThrusters, misconfigThruster)
    assert testResults < 1, testMessage


def thrusterForceTest(show_plots, useDVThruster, useCOMOffset, dropThruster, asymmetricDrop, numControlAxis, saturateThrusters, misconfigThruster):
    """thruster force test function"""
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
    moduleConfig = thrForceMapping.thrForceMappingConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "thrForceMapping"


    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.inputVehControlName = "LrRequested"
    moduleConfig.inputThrusterConfName = "RCSThrusters"
    moduleConfig.outputDataName = "thrusterForceOut"
    moduleConfig.inputVehicleConfigDataName = "vehicleConfigName"

    # write vehicle configuration message
    vehicleConfigOut = thrForceMapping.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputVehicleConfigDataName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    if useCOMOffset == 1:
        CoM_B = [0.03,0.001,0.02]
    else:
        CoM_B = [0,0,0]
    vehicleConfigOut.CoM_B = CoM_B
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputVehicleConfigDataName,
                                          inputMessageSize,
                                          0,
                                          vehicleConfigOut)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = thrForceMapping.CmdTorqueBodyIntMsg()  # Create a structure for the input message
    inputMessageSize = inputMessageData.getStructSize()                           # 3 doubles
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputVehControlName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)

    requestedTorque = [1.0, -0.5, 0.7]             # Set up a list as a 3-vector
    if saturateThrusters>0:        # default angErrThresh is 0, thus this should trigger scaling
        requestedTorque = [10.0, -5.0, 7.0]
    if saturateThrusters==2:        # angle is set and small enough to trigger scaling
        moduleConfig.angErrThresh = 10.0*macros.D2R
    if saturateThrusters==3:        # angle is too large enough to trigger scaling
        moduleConfig.angErrThresh = 40.0*macros.D2R

    inputMessageData.torqueRequestBody = requestedTorque   # write torque request to input message
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputVehControlName,
                                          inputMessageSize,
                                          0,
                                          inputMessageData)             # write data into the simulator

    moduleConfig.epsilon = 0.0005
    fswSetupThrusters.clearSetup()
    MAX_EFF_CNT = simFswInterfaceMessages.MAX_EFF_CNT
    rcsLocationData = np.zeros((MAX_EFF_CNT, 3))
    rcsDirectionData = np.zeros((MAX_EFF_CNT, 3))

    controlAxes_B = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    controlAxes_B = controlAxes_B[0:numControlAxis]
    if len(controlAxes_B) is 0:
        controlAxes_B = np.array([[]])

    controlAxes_B = np.reshape(controlAxes_B, (1, 3 * numControlAxis))
    moduleConfig.controlAxes_B = controlAxes_B[0].tolist()

    if useDVThruster:
        # DV thruster setup
        moduleConfig.thrForceSign = -1
        numThrusters = 6
        rcsLocationData[0:6] = [ \
            [0, 0.413, -0.1671],
            [0, -0.413, -0.1671],
            [0.35766849176297305, 0.20650000000000013, -0.1671],
            [0.3576684917629732, -0.20649999999999988, -0.1671],
            [-0.35766849176297333, 0.20649999999999968, -0.1671],
            [-0.35766849176297305, -0.20650000000000018, -0.1671] \
            ]
        rcsDirectionData[0:6] = [ \
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0] \
            ]
    else:
        # RCS thruster setup
        moduleConfig.thrForceSign = +1
        numThrusters = 8
        rcsLocationData[0:8] = [ \
                [-0.86360, -0.82550, 1.79070],
                [-0.82550, -0.86360, 1.79070],
                [0.82550, 0.86360, 1.79070],
                [0.86360, 0.82550, 1.79070],
                [-0.86360, -0.82550, -1.79070],
                [-0.82550, -0.86360, -1.79070],
                [0.82550, 0.86360, -1.79070],
                [0.86360, 0.82550, -1.79070] \
                ]

        rcsDirectionData[0:8] = [ \
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0] \
            ]


    if dropThruster > 0:
        if (dropThruster % 2==0) and asymmetricDrop: # Drop thrusters that dont share the same torque direction
            removedThrusters = 0
            for i in range(0, numThrusters, 2):
                rcsLocationData[i] = [0.0, 0.0, 0.0]
                rcsDirectionData[i] = [0.0, 0.0, 0.0]
                removedThrusters += 1
            if removedThrusters < dropThruster:
                rcsLocationData[1] = [0.0, 0.0, 0.0]
                removedThrusters += 1
        else:
            for i in range(dropThruster):
                rcsLocationData[numThrusters - 1 - i, :] = [0.0, 0.0, 0.0]
                rcsDirectionData[numThrusters - 1 - i, :] = [0.0, 0.0, 0.0]

        indices = []
        for i in range(numThrusters):
            if np.linalg.norm(rcsLocationData[i]) == 0:
                indices = np.append(indices, i)

        offset = 0
        for i in indices:
            rcsLocationData = np.delete(rcsLocationData, i-offset ,axis=0)
            rcsDirectionData = np.delete(rcsDirectionData, i-offset, axis=0)
            rcsLocationData = np.append(rcsLocationData,[[0.0, 0.0, 0.0]], axis=0)
            rcsDirectionData = np.append(rcsDirectionData, [[0.0, 0.0, 0.0]], axis=0)
            offset = offset + 1

        numThrusters = numThrusters - dropThruster
    maxThrust = 0.95
    if useDVThruster:
        maxThrust = 10.0


    for i in range(numThrusters):
        if misconfigThruster and i == 0:
            maxThrustConfig = 0.0
        else:
            maxThrustConfig = maxThrust
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], maxThrustConfig)
    fswSetupThrusters.writeConfigMessage(  moduleConfig.inputThrusterConfName,
                                           unitTestSim.TotalSim,
                                           unitProcessName)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "thrForce"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  list(range(MAX_EFF_CNT)))

    if misconfigThruster:
        return [testFailCount, ''.join(testMessages)] # We don't handle cases where a thruster is configured incorrectly.

    if useDVThruster and numControlAxis == 3:
        return [testFailCount, ''.join(testMessages)] # 3 control axes doesn't work for dv thrusters (only two axes controllable)


    results = Results_thrForceMapping.Results_thrForceMapping(requestedTorque, moduleConfig.controlAxes_B,
                                         vehicleConfigOut.CoM_B, rcsLocationData,
                                         rcsDirectionData, moduleConfig.thrForceSign,
                                         moduleConfig.thrForcMag, moduleConfig.angErrThresh,
                                         numThrusters, moduleConfig.epsilon)
    F, DNew = results.results_thrForceMapping()

    trueVector = np.zeros((2, MAX_EFF_CNT))
    trueVector[0,:] = F
    trueVector[1,:] = F


    C = np.reshape(controlAxes_B, (numControlAxis, 3))
    CT = np.transpose(C)
    D = np.cross(rcsDirectionData,rcsLocationData-CoM_B)
    receivedTorque = -1.0*np.array([np.matmul(np.transpose(D), np.transpose(moduleOutput[0, 1:MAX_EFF_CNT+1]))])
    receivedTorque = np.append(np.array([0.0]), receivedTorque)


    Lr_offset = np.array([0.0, 0.0, 0.0])
    Lr_B = np.array([0.0, 0.0, 0.0])
    for i in range(0,numThrusters):
        if moduleConfig.thrForceSign < 0 and moduleConfig.thrForcMag[i] >= 0:
            Lr_offset -= moduleConfig.thrForcMag[i]*np.cross(rcsLocationData[i,:]-CoM_B,rcsDirectionData[i,:]) # off pulsing

    Lr_B = requestedTorque + Lr_offset

    # This computes the requested torque direction and the received torque directions
    Lr_Req_B_Unit = Lr_B / np.linalg.norm(Lr_B)
    Lr_Rec_B_Unit= receivedTorque[1:4] / np.linalg.norm(receivedTorque[1:4])

    # This is the requested and recieved torque projected onto the control axes
    Lr_Req_Bar_B = np.matmul(CT, np.matmul(C,Lr_B))
    Lr_Rec_Bar_B = np.matmul(CT, np.matmul(C,receivedTorque[1:4]))
    Lr_Rec_Bar_B = np.append(0, Lr_Rec_Bar_B)

    # This computes the projected requested and received control torque directions
    Lr_Req_Bar_B_Unit = Lr_Req_Bar_B/np.linalg.norm(Lr_Req_Bar_B)
    Lr_Rec_Bar_B_Unit = Lr_Rec_Bar_B[1:4]/np.linalg.norm(Lr_Rec_Bar_B[1:4])
    if np.linalg.norm(Lr_Rec_Bar_B[1:4]) == 0.0:
        Lr_Rec_Bar_B_Unit = [0.0, 0.0, 0.0]
    Lr_Rec_Bar_B_Unit = np.append(0, Lr_Rec_Bar_B_Unit)



    accuracy = 1E-6

    # Check that Python Math and C Math are Identical
    testFailCount, testMessages = unitTestSupport.compareArrayND(np.array([F]), np.array([moduleOutput[0]]), accuracy,
                                                                 "CompareForces",
                                                                 numThrusters, testFailCount, testMessages)
    if testFailCount > 0:
        print(F)
        print(moduleOutput[0])

    # Checks to make sure that no forces are negative
    if not useDVThruster and np.any(moduleOutput[0,1:]<0):
        testFailCount += 1
        print("A negative force exists in the C RCS solution. This is not allowed!\n")

    if not useDVThruster and np.any(F<0):
        testFailCount += 1
        print("A negative force exists in the Python RCS solution. This is not allowed!\n")

    if testFailCount > 0:
        return [testFailCount, ''.join(testMessages)]


    # Check that Torques are Sensible
    print("\nReq Lr_Bar [B]: " + str(Lr_Req_Bar_B))
    print("Rec Lr_Bar [B]: " + str(Lr_Rec_Bar_B[1:4]))


    testFailCount = 0
    testFailCount, testMessages = unitTestSupport.compareArrayND(np.array([Lr_Req_Bar_B_Unit]),
                                                                 np.array([Lr_Rec_Bar_B_Unit]), accuracy,
                                                                 "CompareTorques",
                                                                 3, testFailCount, testMessages)


    snippetName = "LrData_" + str(useDVThruster) + "_" + str(dropThruster) + "_" + str(numControlAxis) + "_" + str(useCOMOffset) + "____" + str(asymmetricDrop) + "_" + str(saturateThrusters) + "_" + str(misconfigThruster)

    snippetTex = "DV Thrusters:\t" + str(useDVThruster) + "\n"
    snippetTex += "Number of Dropped Thrusters:\t" + str(dropThruster)+ "\n"
    snippetTex += "Number of Control Axes:\t" + str(numControlAxis) + "\n"
    snippetTex += "COM Offset:\t" + str(useCOMOffset) + "\n\n"
    snippetTex += "Was the drop asymmetric about the COM?\t" + str(asymmetricDrop) + "\n"
    snippetTex += "Number of Saturated Thrusters:\t" + str(saturateThrusters) + "\n"
    snippetTex += "Misconfigured Thruster?:\t" + str(misconfigThruster) + "\n\n"


    snippetTex += "Original [B]:\t" + str(requestedTorque) + "\n"

    snippetTex += "Requested (Original + Offset) [B]:\t" + str(Lr_B) + "\n"
    snippetTex += "Received [B]:\t\t" + str(receivedTorque[1:4]) + "\n\n"
    snippetTex += "Requested Unit:\t\t" + str(Lr_Req_B_Unit) + "\n"
    snippetTex += "Received Unit:\t\t" + str(Lr_Rec_B_Unit) + "\n\n"

    snippetTex += "Requested On Control Axes (Original + Offset) [B]:\t" + str(Lr_Req_Bar_B) + "\n"
    snippetTex += "Received On Control Axes [B]:\t\t" + str(Lr_Rec_Bar_B[1:4]) + "\n\n"
    snippetTex += "Requested On Control Axes Unit:\t\t" + str(Lr_Req_Bar_B_Unit) + "\n"
    snippetTex += "Received On Control Axes Unit:\t\t" + str(Lr_Rec_Bar_B_Unit[1:4]) + "\n\n"

    snippetTex += "D-Matrix:\n" + str(D) + "\n\n"
    snippetTex += "Forces:\n" + str(np.transpose(F)) + "\n\n"

    directory = "OffNom/UnitVec/"


    # Any solutions that dont have the correct torque, but do have the correct unit direction are called successful.

    if testFailCount > 0:
        unitTestSupport.writeTeXSnippet(directory+"Failed/"+snippetName, snippetTex, path)
        print("FAILED: " + moduleWrap.ModelTag)
        testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                            moduleOutputName + " unit test at t=" +
                            str(moduleOutput[0, 0] * macros.NANO2SEC) +
                            "sec\n")
    else:
        unitTestSupport.writeTeXSnippet(directory+"/Passed/" + snippetName, snippetTex, path)
        print("PASSED: " + moduleWrap.ModelTag)


    unitTestSupport.writeTeXSnippet('toleranceValue', str(accuracy), path)

    snippentName = "passFail_" + str(useDVThruster) + "_" + str(useCOMOffset) + "_" + str(dropThruster) + "_" + str(
        numControlAxis) + "_" + str(saturateThrusters) + "_" + str(misconfigThruster)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + moduleWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + moduleWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                 False,
                 True,           # useDVThruster
                 True,           # use COM offset
                 3,               # num drop thruster(s)
                 False,            # asymmetric drop
                 2,               # num control axis
                 1,               # saturateThrusters
                 False            # misconfigThruster

    )
