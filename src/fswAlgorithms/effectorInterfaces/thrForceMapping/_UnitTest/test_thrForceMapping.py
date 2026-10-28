
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
#   Unit Test Script
#   Module Name:        thrForceMapping
#   Author:             Hanspeter Schaub
#   Creation Date:      July 4, 2016
#
import inspect
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

import pytest

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.utilities import macros
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import simHelpers
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging

import numpy as np


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.

@pytest.mark.parametrize("useDVThruster", [True, False])
@pytest.mark.parametrize(["useCOMOffset","dropThruster", "use2ndLoop"],[
                         (False, 0, False),
                         (False, 1, True),
                         (False, 2, True), # Any time we drop a thruster we should recompute the solution
                         (True, 0, False)]) # We don't handle the case where there is a dropped thruster and and COM offset--see performance analysis.
@pytest.mark.parametrize("asymmetricDrop", [False])
@pytest.mark.parametrize("numControlAxis", [1, 2, 3])
@pytest.mark.parametrize("saturateThrusters", [0,1,2])
@pytest.mark.parametrize("misconfigThruster", [False])



# update "module" in this function name to reflect the module name
def test_module(show_plots, useDVThruster, useCOMOffset, dropThruster, asymmetricDrop, numControlAxis, saturateThrusters,
                misconfigThruster, use2ndLoop):
    """
    Validate the thruster forces and the torque angle error against the Python reference model using the leading
    ``numControlAxis`` body axes as control axes.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = thrusterForceTest(show_plots, useDVThruster, useCOMOffset, dropThruster, asymmetricDrop,
                                                   numControlAxis, saturateThrusters, misconfigThruster, use2ndLoop,
                                                   "leading")
    assert testResults < 1, testMessage


@pytest.mark.parametrize("useDVThruster", [True, False])
@pytest.mark.parametrize(["dropThruster", "use2ndLoop"], [(0, False), (1, True)])
@pytest.mark.parametrize("numControlAxis", [1, 2, 3])
@pytest.mark.parametrize("saturateThrusters", [0, 2])
# For numControlAxis == 1, "reversed" is identical to "leading" and "rotated" only negates the axis, so these cases
# exercise the same code path. They are kept for clarity so that every axesOrder is run for every numControlAxis.
@pytest.mark.parametrize("axesOrder", ["reversed", "rotated"])
def test_moduleAxesOrder(show_plots, useDVThruster, dropThruster, numControlAxis, saturateThrusters, use2ndLoop,
                         axesOrder):
    """
    Validate the thruster forces and the torque angle error against the Python reference model when the control axes
    are not the leading body axes.

    The ``axesOrder`` parameter spans the same controlled subspace as the leading body axes, either with the same axes
    in reversed order or with an orthonormal rotation of these axes. The thruster forces and ``outTorqAngErr`` must
    not depend on this choice. To limit the test runtime, only a representative subset of the
    :func:`test_module` parameters is used.
    """
    [testResults, testMessage] = thrusterForceTest(show_plots, useDVThruster, False, dropThruster, False,
                                                   numControlAxis, saturateThrusters, False, use2ndLoop, axesOrder)
    assert testResults < 1, testMessage


def controlAxesSet(numControlAxis, axesOrder):
    """
    Return orthonormal control axes that span the same subspace as the leading ``numControlAxis`` body axes.

    :param numControlAxis: number of control axes
    :param axesOrder: ``"leading"`` for the leading body axes, ``"reversed"`` for the same axes in reverse
        order, or ``"rotated"`` for an orthonormal rotation of these axes
    :return: control axes as rows of a ``numControlAxis`` x 3 array
    """
    axes = np.eye(3)[0:numControlAxis]
    if axesOrder == "reversed":
        axes = axes[::-1]
    elif axesOrder == "rotated":
        if numControlAxis == 1:
            axes = -axes
        elif numControlAxis == 2:
            angle = 30.0 * macros.D2R  # [rad]
            axes = np.array([[np.cos(angle), np.sin(angle), 0.0],
                             [-np.sin(angle), np.cos(angle), 0.0]])
        else:
            axes = np.array(rbk.PRV2C([0.3, -0.4, 0.5]))  # [rad] principal rotation vector
    return axes


def thrusterForceTest(show_plots, useDVThruster, useCOMOffset, dropThruster, asymmetricDrop, numControlAxis,
                      saturateThrusters, misconfigThruster, use2ndLoop, axesOrder="leading"):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    module = thrForceMapping.thrForceMapping()
    module.ModelTag = "thrForceMapping"


    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    module.use2ndLoop = use2ndLoop

    # write vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    if useCOMOffset == 1:
        CoM_B = [0.03,0.001,0.02]
    else:
        CoM_B = [0,0,0]
    vehicleConfigOut.CoM_B = CoM_B
    vcInMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = messaging.CmdTorqueBodyMsgPayload()  # Create a structure for the input message
    requestedTorque = [1.0, -0.5, 0.7]             # Set up a list as a 3-vector
    if saturateThrusters>0:        # default angErrThresh is 0, thus this should trigger scaling
        requestedTorque = [10.0, -5.0, 7.0]
    if saturateThrusters==2:        # angle is set and small enough to trigger scaling
        module.angErrThresh = 10.0*macros.D2R
    if saturateThrusters==3:        # angle is too large enough to trigger scaling
        module.angErrThresh = 40.0*macros.D2R

    inputMessageData.torqueRequestBody = requestedTorque   # write torque request to input message
    cmdTorqueInMsg = messaging.CmdTorqueBodyMsg().write(inputMessageData)

    module.epsilon = 0.0005
    fswSetupThrusters.clearSetup()
    MAX_EFF_CNT = messaging.MAX_EFF_CNT
    rcsLocationData = np.zeros((MAX_EFF_CNT, 3))
    rcsDirectionData = np.zeros((MAX_EFF_CNT, 3))

    controlAxes_B = controlAxesSet(numControlAxis, axesOrder)
    if len(controlAxes_B) == 0:
        controlAxes_B = np.array([[]])

    controlAxes_B = np.reshape(controlAxes_B, (1, 3 * numControlAxis))
    module.controlAxes_B = controlAxes_B[0].tolist()

    if useDVThruster:
        # DV thruster setup
        module.thrForceSign = -1
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
        module.thrForceSign = +1
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
        if (dropThruster % 2==0) and asymmetricDrop: # Drop thrusters that don't share the same torque direction
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
            idx = (int) (i - offset)
            rcsLocationData = np.delete(rcsLocationData, idx, axis=0)
            rcsDirectionData = np.delete(rcsDirectionData, idx, axis=0)
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
    thrConfigInMsg = fswSetupThrusters.writeConfigMessage()

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.thrForceCmdOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.cmdTorqueInMsg.subscribeTo(cmdTorqueInMsg)
    module.thrConfigInMsg.subscribeTo(thrConfigInMsg)
    module.vehConfigInMsg.subscribeTo(vcInMsg)

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
    moduleOutput = dataLog.thrForce

    if misconfigThruster:
        return [testFailCount, ''.join(testMessages)] # We don't handle cases where a thruster is configured incorrectly.

    if useDVThruster and numControlAxis == 3:
        return [testFailCount, ''.join(testMessages)] # 3 control axes doesn't work for dv thrusters (only two axes controllable)

    results = thrForceMapping.Results_thrForceMapping(requestedTorque, module.controlAxes_B,
                                         vehicleConfigOut.CoM_B, rcsLocationData,
                                         rcsDirectionData, module.thrForceSign,
                                         module.thrForcMag, module.angErrThresh,
                                         numThrusters, module.epsilon, use2ndLoop)

    F, DNew = results.results_thrForceMapping()

    trueVector = np.zeros((2, MAX_EFF_CNT))
    trueVector[0,:] = F
    trueVector[1,:] = F

    C = np.reshape(controlAxes_B, (numControlAxis, 3))
    CT = np.transpose(C)
    D = np.cross(rcsDirectionData,rcsLocationData-CoM_B)
    receivedTorque = -1.0*np.array([np.matmul(np.transpose(D), np.transpose(moduleOutput[0]))])
    receivedTorque = np.append(np.array([]), receivedTorque)

    Lr_offset = np.array([0.0, 0.0, 0.0])
    Lr_B = np.array([0.0, 0.0, 0.0])
    for i in range(0,numThrusters):
        if module.thrForceSign < 0 and module.thrForcMag[i] >= 0:
            Lr_offset -= module.thrForcMag[i]*np.cross(rcsLocationData[i,:]-CoM_B,rcsDirectionData[i,:]) # off pulsing

    Lr_B = requestedTorque + Lr_offset

    # This computes the requested torque direction and the received torque directions
    Lr_Req_B_Unit = Lr_B / np.linalg.norm(Lr_B)
    Lr_Rec_B_Unit = receivedTorque / np.linalg.norm(receivedTorque)

    # This is the requested and recieved torque projected onto the control axes
    Lr_Req_Bar_B = np.matmul(CT, np.matmul(C, Lr_B))
    Lr_Rec_Bar_B = np.matmul(CT, np.matmul(C, receivedTorque))

    # This computes the projected requested and received control torque directions
    Lr_Req_Bar_B_Unit = Lr_Req_Bar_B/np.linalg.norm(Lr_Req_Bar_B)
    Lr_Rec_Bar_B_Unit = Lr_Rec_Bar_B/np.linalg.norm(Lr_Rec_Bar_B)
    if np.linalg.norm(Lr_Rec_Bar_B) == 0.0:
        Lr_Rec_Bar_B_Unit = [0.0, 0.0, 0.0]

    accuracy = 1E-6

    # Check that Python Math and C Math are Identical
    testFailCount, testMessages = unitTestSupport.compareArrayND(np.array([F]), np.array([moduleOutput[0]]), accuracy,
                                                                 "CompareForces",
                                                                 MAX_EFF_CNT, testFailCount, testMessages)

    # Checks to make sure that no forces are negative
    if not useDVThruster and np.any(moduleOutput[0] < 0):
        testFailCount += 1
        print("A negative force exists in the C RCS solution. This is not allowed!\n")

    if not useDVThruster and np.any(F < 0):
        testFailCount += 1
        print("A negative force exists in the Python RCS solution. This is not allowed!\n")

    if testFailCount > 0:
        return [testFailCount, ''.join(testMessages)]

    # Check that the torque angle error compares [C]^T [C] Lr with [D] F in body frame components
    expectedAngErr = torqueAngErr(np.matmul(CT, np.matmul(C, Lr_B)), rcsLocationData[0:numThrusters] - CoM_B,
                                  rcsDirectionData[0:numThrusters], moduleOutput[0][0:numThrusters],
                                  np.array(module.thrForcMag[0:numThrusters]))
    if not np.isclose(module.outTorqAngErr, expectedAngErr, atol=accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + module.ModelTag + " outTorqAngErr " + str(module.outTorqAngErr)
                            + " rad does not match the expected " + str(expectedAngErr) + " rad\n")
        return [testFailCount, ''.join(testMessages)]

    # Check that Torques are Sensible
    print("\nReq Lr_Bar [B]: " + str(Lr_Req_Bar_B))
    print("Rec Lr_Bar [B]: " + str(Lr_Rec_Bar_B))

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
    snippetTex += "Received [B]:\t\t" + str(receivedTorque) + "\n\n"
    snippetTex += "Requested Unit:\t\t" + str(Lr_Req_B_Unit) + "\n"
    snippetTex += "Received Unit:\t\t" + str(Lr_Rec_B_Unit) + "\n\n"

    snippetTex += "Requested On Control Axes (Original + Offset) [B]:\t" + str(Lr_Req_Bar_B) + "\n"
    snippetTex += "Received On Control Axes [B]:\t\t" + str(Lr_Rec_Bar_B) + "\n\n"
    snippetTex += "Requested On Control Axes Unit:\t\t" + str(Lr_Req_Bar_B_Unit) + "\n"
    snippetTex += "Received On Control Axes Unit:\t\t" + str(Lr_Rec_Bar_B_Unit) + "\n\n"

    snippetTex += "D-Matrix:\n" + str(D) + "\n\n"
    snippetTex += "Forces:\n" + str(np.transpose(F)) + "\n\n"

    if axesOrder != "leading":
        # the AutoTeX snippets only document the leading control axes cases
        return [testFailCount, ''.join(testMessages)]

    directory = "Nom/UnitVec/"

    # Any solutions that dont have the correct torque, but do have the correct unit direction are called successful.

    if testFailCount > 0:
        simHelpers.writeTeXSnippet(directory+"Failed/"+snippetName, snippetTex, path)
        print("FAILED: " + module.ModelTag)
        testMessages.append("FAILED: " + module.ModelTag + " Module failed  unit test at t=" +
                            str(dataLog.times()[0] * macros.NANO2SEC) +
                            "sec\n")
    else:
        simHelpers.writeTeXSnippet(directory+"/Passed/" + snippetName, snippetTex, path)
        print("PASSED: " + module.ModelTag)


    simHelpers.writeTeXSnippet('toleranceValue', str(accuracy), path)

    snippentName = "passFail_" + str(useDVThruster) + "_" + str(useCOMOffset) + "_" + str(dropThruster) + "_" + str(
        numControlAxis) + "_" + str(saturateThrusters) + "_" + str(misconfigThruster)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + module.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + module.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    simHelpers.writeTeXSnippet(snippentName, passedText, path)

    if testFailCount > 0:
        print("Python:\t " + str(F))
        print("C: \t:" + str(moduleOutput[0]))

    return [testFailCount, ''.join(testMessages)]


def torqueAngErr(LrBar_B, r_B, gt_B, F, FMag):
    """
    Compute the angle between the reduced requested torque and the saturation limited thruster torque.

    :param LrBar_B: reduced requested torque :math:`[C]^T [C] {}^B\\bm L_r` in body frame components [Nm]
    :param r_B: thruster locations relative to the center of mass, one row per thruster [m]
    :param gt_B: thruster force unit direction vectors, one row per thruster
    :param F: thruster force solution [N]
    :param FMag: thruster force saturation limits [N]
    :return: angle between the two torque vectors [rad]
    """
    if np.linalg.norm(LrBar_B) < 1e-12:
        return 0.0
    FClip = np.clip(F, -FMag, FMag)
    tau_B = np.cross(r_B, gt_B).T @ FClip
    cosAngle = np.dot(LrBar_B, tau_B) / (np.linalg.norm(LrBar_B) * np.linalg.norm(tau_B))
    return np.arccos(np.clip(cosAngle, -1.0, 1.0))


def runCubeRcsMapping(controlAxes, Lr_B, maxThrust, angErrThresh):
    """
    Run the module once with 8 RCS thrusters at the corners of a 2 m cube that fire tangentially.

    :param controlAxes: control axes as rows of an array with up to 3 rows
    :param Lr_B: requested control torque in body frame components [Nm]
    :param maxThrust: thruster force saturation limit [N]
    :param angErrThresh: torque angle error beyond which saturated forces are scaled [rad]
    :return: thruster forces [N], ``outTorqAngErr`` [rad], thruster locations [m] and thruster directions
    """
    rThr_B = np.array([[1, 1, 1], [1, 1, -1], [-1, 1, 1], [-1, 1, -1],
                       [1, -1, 1], [1, -1, -1], [-1, -1, 1], [-1, -1, -1]], dtype=float)  # [m]
    gtThr_B = np.array([[-1, 0, 0], [-1, 0, 0], [1, 0, 0], [1, 0, 0],
                        [0, 0, -1], [0, 0, 1], [0, 0, -1], [0, 0, 1]], dtype=float)

    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.CreateNewProcess("TestProcess").addTask(unitTestSim.CreateNewTask("unitTask", macros.sec2nano(1.0)))

    module = thrForceMapping.thrForceMapping()
    module.ModelTag = "thrForceMapping"
    module.thrForceSign = 1
    module.use2ndLoop = 0
    module.epsilon = 1e-6
    module.angErrThresh = angErrThresh
    axes = np.array(controlAxes, dtype=float)
    module.controlAxes_B = axes.flatten().tolist() + [0.0] * (9 - axes.size)

    fswSetupThrusters.clearSetup()
    for r, g in zip(rThr_B, gtThr_B):
        fswSetupThrusters.create(r, g, maxThrust)
    thrConfigInMsg = fswSetupThrusters.writeConfigMessage()
    vcInMsg = messaging.VehicleConfigMsg().write(messaging.VehicleConfigMsgPayload(CoM_B=[0.0, 0.0, 0.0]))
    cmdTorqueInMsg = messaging.CmdTorqueBodyMsg().write(
        messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=list(Lr_B)))
    module.thrConfigInMsg.subscribeTo(thrConfigInMsg)
    module.vehConfigInMsg.subscribeTo(vcInMsg)
    module.cmdTorqueInMsg.subscribeTo(cmdTorqueInMsg)

    dataLog = module.thrForceCmdOutMsg.recorder()
    unitTestSim.AddModelToTask("unitTask", module)
    unitTestSim.AddModelToTask("unitTask", dataLog)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(0)
    unitTestSim.ExecuteSimulation()

    return np.array(dataLog.thrForce[-1][0:len(rThr_B)]), module.outTorqAngErr, rThr_B, gtThr_B


@pytest.mark.parametrize("controlAxes", [
    np.eye(3),
    [[0, 0, 1], [1, 0, 0], [0, 1, 0]],
    [[1, 0, 0], [0, 0, 1]],
    [[0, 1, 0], [1, 0, 0]],
    [[0, 1, 0]],
    rbk.PRV2C([0.3, -0.4, 0.5]),
    rbk.PRV2C([-0.2, 0.6, 0.1])[0:2],
], ids=["identity", "permutedZXY", "nonLeadingXZ", "reversedYX", "singleY", "rotated3", "rotated2"])
@pytest.mark.parametrize("saturated", [False, True])
def test_controlAxesOrdering(controlAxes, saturated):
    """
    Validate the torque angle error for several controlled subspaces and the saturation scaling decision
    for alternative orthonormal bases of the full three-dimensional space.

    The test uses 8 RCS thrusters and checks that

    - the produced torque matches the requested torque on the control axes, :math:`[C][D]\\bm F = [C]\\bm L_r`,
    - ``outTorqAngErr`` is the angle between :math:`[C]^T [C] \\bm L_r` and the saturation-clipped torque
      :math:`[D]\\bm F_{\\mathrm{clip}}` in body frame components,
    - for the three-axis cases, with saturated thrusters and ``angErrThresh`` between the true angle error
      and the angle error obtained by comparing control axis components with body frame components, the
      forces are not scaled and match the forces found with the identity control axes.
    """
    Lr_B = np.array([0.3, -0.5, 0.8])  # [Nm]
    C = np.array(controlAxes, dtype=float)
    accuracy = 1e-9
    if saturated:
        maxThrust = 0.2  # [N]
        angErrThresh = 10.0 * macros.D2R  # [rad]
    else:
        maxThrust = 10.0  # [N]
        angErrThresh = 0.0  # [rad]

    F, angErr, rThr_B, gtThr_B = runCubeRcsMapping(C, Lr_B, maxThrust, angErrThresh)
    D = np.cross(rThr_B, gtThr_B).T  # [m]

    expectedAngErr = torqueAngErr(C.T @ C @ Lr_B, rThr_B, gtThr_B, F, maxThrust * np.ones(len(F)))
    # arccos amplifies round-off near zero angle, so the angle uses a looser tolerance than the forces
    angAccuracy = 1e-6  # [rad]
    assert angErr == pytest.approx(expectedAngErr, abs=angAccuracy), "outTorqAngErr is not the body frame angle error"

    if not saturated:
        np.testing.assert_allclose(C @ D @ F, C @ Lr_B, atol=accuracy,
                                   err_msg="the produced torque does not match the request on the control axes")
    elif len(C) == 3:
        FIdentity, angErrIdentity, _, _ = runCubeRcsMapping(np.eye(3), Lr_B, maxThrust, angErrThresh)
        assert angErrIdentity < angErrThresh, "the test threshold must be above the true angle error"
        assert np.max(F) > maxThrust, "the forces must stay unscaled below the angle error threshold"
        np.testing.assert_allclose(F, FIdentity, atol=accuracy,
                                   err_msg="the forces depend on the order or orientation of the control axes")
        assert angErr == pytest.approx(angErrIdentity, abs=angAccuracy)


@pytest.mark.parametrize("controlAxes, expectWarning", [
    (np.eye(3), False),
    ([[1, 0, 0], [1, 1, 0]], True),
], ids=["orthonormal", "nonOrthogonal"])
def test_controlAxesOrthogonalityWarning(capfd, controlAxes, expectWarning):
    """
    Validate that ``Reset()`` logs a warning if the control axes are not mutually orthogonal, and stays silent for
    orthonormal control axes.
    """
    runCubeRcsMapping(controlAxes, [0.3, -0.5, 0.8], 10.0, 0.0)  # [Nm], [N], [rad]
    warned = "controlAxes_B are not orthogonal" in capfd.readouterr().out
    assert warned == expectWarning, "unexpected control axes orthogonality warning behavior"


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                 False,
                 False,           # useDVThruster
                 False,           # use COM offset
                 2,               # num drop thruster(s)
                 False,           # asymmetric drop
                 3,               # num control axis
                 2,               # saturateThrusters
                 False,           # misconfigThruster
                 False,           # Use 2nd loop
                 "leading"        # control axes order
    )
