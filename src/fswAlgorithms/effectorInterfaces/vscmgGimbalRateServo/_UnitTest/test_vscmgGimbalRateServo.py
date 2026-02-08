#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#
import inspect
import os

import numpy as np
import pytest
import math

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import fswSetupVSCMGs
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import vscmgGimbalRateServo

def defaultVSCMG():
    VSCMG = messaging.VSCMGConfigMsgPayload()

    VSCMG.rGB_B = [[0.],[0.],[0.]]
    VSCMG.gsHat0_B = [[0.],[0.],[0.]]
    VSCMG.gtHat0_B = [[0.],[0.],[0.]]
    VSCMG.ggHat_B = [[0.],[0.],[0.]]
    VSCMG.u_s_max = -1
    VSCMG.u_s_min = -1
    VSCMG.u_s_f = 0.
    VSCMG.wheelLinearFrictionRatio = -1
    VSCMG.u_g_current = 0.
    VSCMG.u_g_max = -1
    VSCMG.u_g_min = -1
    VSCMG.u_g_f = 0.
    VSCMG.gimbalLinearFrictionRatio = -1
    VSCMG.Omega = 14.4
    VSCMG.gamma = 0.
    VSCMG.gammaDot = 0.
    VSCMG.Omega_max = 6000. * macros.RPM
    VSCMG.gammaDot_max = -1
    VSCMG.IW1 = 0.1
    VSCMG.IW2 = 0.04
    VSCMG.IW3 = 0.03
    VSCMG.IG1 = 0.03
    VSCMG.IG2 = 0.
    VSCMG.IG3 = 0.
    VSCMG.U_s = 0.
    VSCMG.U_d = 0.
    VSCMG.l = 0.
    VSCMG.L = 0.
    VSCMG.rGcG_G = [[0.],[0.],[0.]]
    VSCMG.massW = 0.
    VSCMG.massG = 0.
    VSCMG.VSCMGModel = 0
    return VSCMG

def setupVSCMGs(numVSCMGs):
    VSCMGs = []

    ang = 54.75 * np.pi/180

    VSCMGs.append(defaultVSCMG())
    VSCMGs[0].ggHat_B = [[math.cos(ang)], [0.0], [math.sin(ang)]]
    VSCMGs[0].gsHat0_B = [[0.0], [1.0], [0.0]]
    VSCMGs[0].gtHat0_B = np.cross(np.array([math.cos(ang), 0.0, math.sin(ang)]),np.array([0.0, 1.0, 0.0]))

    if numVSCMGs > 1:
        VSCMGs.append(defaultVSCMG())
        VSCMGs[1].gsHat0_B = [[0.0], [-1.0], [0.0]]
        VSCMGs[1].ggHat_B = [[-math.cos(ang)], [0.0], [math.sin(ang)]]
        VSCMGs[1].gtHat0_B = np.cross(np.array([-math.cos(ang), 0.0, math.sin(ang)]),np.array([0.0, -1.0, 0.0]))

        if numVSCMGs == 4:
            VSCMGs.append(defaultVSCMG())
            VSCMGs[2].gamma = np.deg2rad(90.0)
            gsHat2_t0_B = np.array([1.0, 0.0, 0.0])
            ggHat2_B = np.array([0.0, math.cos(ang), math.sin(ang)])
            gtHat2_t0_B = np.cross(ggHat2_B, gsHat2_t0_B)
            gtHat2_t0_B /= np.linalg.norm(gtHat2_t0_B)
            BG2 = np.column_stack([gsHat2_t0_B,
                            gtHat2_t0_B,
                            ggHat2_B])
            GG02 = np.array([
                [-math.cos(VSCMGs[2].gamma), math.sin(VSCMGs[2].gamma), 0.0],
                [-math.sin(VSCMGs[2].gamma),  -math.cos(VSCMGs[2].gamma), 0.0],
                [0.0,             0.0,          1.0]
                ])
            BG02 = BG2 @ GG02
            VSCMGs[2].gsHat0_B = [[BG02[0][0]], [BG02[1][0]], [BG02[2][0]]]
            VSCMGs[2].gtHat0_B = [[BG02[0][1]], [BG02[1][1]], [BG02[2][1]]]
            VSCMGs[2].ggHat_B = [[BG02[0][2]], [BG02[1][2]], [BG02[2][2]]]

            VSCMGs.append(defaultVSCMG())
            VSCMGs[3].gamma = -np.deg2rad(90.0)
            gsHat3_t0_B = np.array([-1.0, 0.0, 0.0])
            ggHat3_B = np.array([0.0, -math.cos(ang), math.sin(ang)])
            gtHat3_t0_B = np.cross(ggHat3_B, gsHat3_t0_B)
            gtHat3_t0_B /= np.linalg.norm(gtHat3_t0_B)
            BG3 = np.column_stack([gsHat3_t0_B,
                            gtHat3_t0_B,
                            ggHat3_B])
            GG03 = np.array([
                [-math.cos(VSCMGs[3].gamma), math.sin(VSCMGs[3].gamma), 0.0],
                [-math.sin(VSCMGs[3].gamma),  -math.cos(VSCMGs[3].gamma), 0.0],
                [0.0,             0.0,          1.0]
                ])
            BG03 = BG3 @ GG03
            VSCMGs[3].gsHat0_B = [[BG03[0][0]], [BG03[1][0]], [BG03[2][0]]]
            VSCMGs[3].gtHat0_B = [[BG03[0][1]], [BG03[1][1]], [BG03[2][1]]]
            VSCMGs[3].ggHat_B = [[BG03[0][2]], [BG03[1][2]], [BG03[2][2]]]

    return VSCMGs

def computeTorques(VSCMGs, numVSCMGs, omega_BN_B, K_gamma, gamma_dot_d, Omega_dot_d):

    u_s_list = np.zeros(numVSCMGs)
    u_g_list = np.zeros(numVSCMGs)
    for i in range(numVSCMGs):
        BG0 = np.column_stack((
            np.array(VSCMGs[i].gsHat0_B).flatten(),
            np.array(VSCMGs[i].gtHat0_B).flatten(),
            np.array(VSCMGs[i].ggHat_B).flatten()
        ))
        GG0 = np.identity(3)
        GG0[0,0] = np.cos(VSCMGs[i].gamma)
        GG0[0,1] = np.sin(VSCMGs[i].gamma)
        GG0[1,0] = -GG0[0,1]
        GG0[1,1] = GG0[0,0]
        BG = BG0 @ GG0.T
        gsHat = BG[:, 0]
        gtHat = BG[:, 1]

        Js = VSCMGs[i].IG1 + VSCMGs[i].IW1
        Jt = VSCMGs[i].IG2 + VSCMGs[i].IW2
        Jg = VSCMGs[i].IG3 + VSCMGs[i].IW3
        Iws = VSCMGs[i].IW1

        omega = np.array(omega_BN_B).flatten()
        omega_s = float(gsHat @ omega)
        omega_t = float(gtHat @ omega)
        u_s_list[i] = VSCMGs[i].IW1 * (Omega_dot_d[i] + VSCMGs[i].gammaDot * omega_t)

        # Calculate the gimbal acceleration
        gamma_ddot = -K_gamma*(VSCMGs[i].gammaDot - gamma_dot_d[i])

        # Calculate the gimbal torque
        u_g_list[i] = Jg * gamma_ddot - (Js - Jt) * omega_s * omega_t - Iws * VSCMGs[i].Omega * omega_t

    return u_s_list, u_g_list

@pytest.mark.parametrize("numVSCMGs", [1,2,4])

def test_vscmgGimbalRateServo(show_plots, numVSCMGs):
   """Module Unit Test"""
   [testResults, testMessage] = vscmgGimbalRateServoTest(show_plots, numVSCMGs)
   assert testResults < 1, testMessage

def vscmgGimbalRateServoTest(show_plots, numVSCMGs):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with VSCMGs attached to a rigid hub.

    **Test Parameters**

    The number of VSCMGs is varied between each test.

    Args:
        numVSCMGs (int): number of VSCMGs for this parameterized unit test

    **Description of Variables Being Tested**

    In this file we are checking that the VSCMG gimbal rate servo module produces the desired
    wheel and motor torques.
    Therefore, the output torques are checked against their expected values
    """
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = vscmgGimbalRateServo.VscmgGimbalRateServo()
    module.ModelTag = "vscmgGimbalRateServo"
    K_gammaDot = 1.0
    module.setK_gammaDot(K_gammaDot)
    unitTestSim.AddModelToTask(unitTaskName, module)

    # setup hub angular velocity
    omega_BN_B = [0.01, -0.01, 0.005]
    attIn = messaging.NavAttMsgPayload()
    attIn.omega_BN_B = omega_BN_B

    # setup VSCMGs
    VSCMGs = setupVSCMGs(numVSCMGs)
    fswSetupVSCMGs.clearSetup()
    for i in range(numVSCMGs):
        fswSetupVSCMGs.create(
            VSCMGs[i].gsHat0_B, VSCMGs[i].gtHat0_B, VSCMGs[i].ggHat_B,
            VSCMGs[i].IG1, VSCMGs[i].IG2, VSCMGs[i].IG3,
            VSCMGs[i].IW1, VSCMGs[i].IW2, VSCMGs[i].IW3,
            VSCMGs[i].Omega, VSCMGs[i].gamma, VSCMGs[i].gammaDot)

    # setup reference states message
    refArray = messaging.VSCMGRefStatesMsgPayload()
    Omega_dot_d = np.deg2rad(np.array([0, 1, -1, -1]))
    gamma_dot_d = np.deg2rad(np.array([0, 1, -1, -1]))
    refWheelAccels = []
    refGimbalRates = []
    for i in range(numVSCMGs):
        refWheelAccels.append(Omega_dot_d[i])
        refGimbalRates.append(gamma_dot_d[i])
    refArray.wheelAccels = refWheelAccels
    refArray.gimbalRates = refGimbalRates

    # setup speeds message
    speedsArray = messaging.VSCMGSpeedMsgPayload()
    gimbalAngles = []
    gimbalRates = []
    wheelSpeeds = []
    for i in range(numVSCMGs):
        gimbalAngles.append(VSCMGs[i].gamma)
        gimbalRates.append(VSCMGs[i].gammaDot)
        wheelSpeeds.append(VSCMGs[i].Omega)
    speedsArray.gimbalAngles = gimbalAngles
    speedsArray.gimbalRates = gimbalRates
    speedsArray.wheelSpeeds = wheelSpeeds

    # write messages
    vscmgParamMsg = fswSetupVSCMGs.writeConfigMessage()
    refInMsg = messaging.VSCMGRefStatesMsg().write(refArray)
    attInMsg = messaging.NavAttMsg().write(attIn)
    speedsInMsg = messaging.VSCMGSpeedMsg().write(speedsArray)

    # subscribe input messages to module
    module.vsmcgParamsInMsg.subscribeTo(vscmgParamMsg)
    module.vscmgRefStatesInMsg.subscribeTo(refInMsg)
    module.attInMsg.subscribeTo(attInMsg)
    module.speedsInMsg.subscribeTo(speedsInMsg)

    # setup logging for the output message
    dataLog = module.cmdsOutMsg.recorder()
    dataLogC = module.cmdsOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    unitTestSim.AddModelToTask(unitTaskName, dataLogC)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.1))
    unitTestSim.ExecuteSimulation()

    # pull module data and make sure it is correct
    wheelTorques = dataLog.wheelTorque[0,:numVSCMGs]
    gimbalTorques = dataLog.gimbalTorque[0,:numVSCMGs]
    wheelTorquesC = dataLogC.wheelTorque[0,:numVSCMGs]
    gimbalTorquesC = dataLogC.gimbalTorque[0,:numVSCMGs]

    moduleTorques = np.concatenate((wheelTorques, gimbalTorques))
    moduleTorquesC = np.concatenate((wheelTorquesC, gimbalTorquesC))

    # set the output truth values
    trueWheelTorques, trueGimbalTorques = computeTorques(VSCMGs, numVSCMGs, omega_BN_B, K_gammaDot, refGimbalRates, refWheelAccels)
    trueTorques = np.concatenate((trueWheelTorques, trueGimbalTorques))

    #compare the module output values to the truth values
    accuracy = 1e-12
    testFailCount, testMessages = unitTestSupport.compareArrayND([trueTorques], [moduleTorques], accuracy, "VSCMGTorques",
                                                                 2 * numVSCMGs, testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArrayND([moduleTorques], [moduleTorquesC], accuracy, "cMsgTorques",
                                                                 2 * numVSCMGs, testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_vscmgGimbalRateServo(False, 4)
