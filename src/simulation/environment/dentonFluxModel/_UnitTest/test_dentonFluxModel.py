# 
#  ISC License
# 
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
#   Unit Test Script
#   Module Name:        dentonFluxModel
#   Author:             Julian Hammerl
#   Creation Date:      December 12, 2021
#
import math
import matplotlib.pyplot as plt
import numpy as np
import pytest
import os, inspect
from Basilisk import __path__

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.simulation import dentonFluxModel

Kps = [0, 21]
LTs = [0.00, 14.73]
z_offsets = [0., 3500e3]
r_EN_Ns = np.array([[0., 0., 0.], [400e3, 300e3, -200e3]])

@pytest.mark.parametrize("accuracy", [1e-6])
@pytest.mark.parametrize("param1_Kp, param2_LT, param3_z, param4_r_EN", [
    (Kps[0], LTs[0], z_offsets[0], r_EN_Ns[0]),
    (Kps[1], LTs[1], z_offsets[1], r_EN_Ns[0]),
    (Kps[1], LTs[1], z_offsets[0], r_EN_Ns[1]),
    (Kps[1], LTs[1], z_offsets[1], r_EN_Ns[1]),
    (Kps[1], LTs[0], z_offsets[1], r_EN_Ns[1]),
])

def test_dentonFluxModel(show_plots, param1_Kp, param2_LT, param3_z, param4_r_EN, accuracy):
    r"""
    **Validation Test Description**

    The Denton Flux Module is tested for several different Kp indices, local times, and spacecraft/Sun/Earth positions

    **Test Parameters**

    Args:
        show_plots (bool): specify if plots should be shown
        param1_Kp (int): Kp Index
        param2_LT (float): Local Time (use 2 decimals)
        param3_z (float): z-offset to test spacecraft and Sun position with offset to equatorial plane
        param4_r_EN (float np.array): r_EN_N position vector of Earth w.r.t. N frame, in N frame components
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    The electron and ion energies are compared to make sure the flux data is computed for the same energy. The main
    part of the unitTest is to compare the electron and ion flux.
    """
    [testResults, testMessage] = dentonFluxModelTestFunction(show_plots, param1_Kp, param2_LT, param3_z, param4_r_EN,
                                                             accuracy)
    assert testResults < 1, testMessage


def dentonFluxModelTestFunction(show_plots, param1_Kp, param2_LT, param3_z, param4_r_EN, accuracy):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create sim module and test thread
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = dentonFluxModel.DentonFluxModel()
    module.ModelTag = "dentonFluxModule"
    module.kpIndex = param1_Kp
    module.numOutputEnergies = 6
    module.dataPath = bskPath + '/supportData/DentonGEO/'

    unitTestSim.AddModelToTask(unitTaskName, module)

    # Set up position vectors (param3_z is used to offset S/C and sun from equatorial plane)
    LT = param2_LT
    angle = LT * 360./24. * np.pi/180 - np.pi
    OrbitRadius = 42000 * 1e3  # GEO orbit

    r_BE_N = np.array([OrbitRadius*math.cos(angle), OrbitRadius*math.sin(angle), param3_z])
    r_SE_N = np.array([149000000000.0, 0., -2.73*param3_z])
    r_EN_N = param4_r_EN
    r_BN_N = r_BE_N + r_EN_N
    r_SN_N = r_SE_N + r_EN_N

    # Configure input messages
    scStateInMsgData = messaging.SCStatesMsgPayload()
    scStateInMsgData.r_BN_N = r_BN_N
    scStateInMsg = messaging.SCStatesMsg().write(scStateInMsgData)

    sunStateInMsgData = messaging.SpicePlanetStateMsgPayload()
    sunStateInMsgData.PositionVector = r_SN_N
    sunStateInMsg = messaging.SpicePlanetStateMsg().write(sunStateInMsgData)

    earthStateInMsgData = messaging.SpicePlanetStateMsgPayload()
    earthStateInMsgData.PositionVector = r_EN_N
    earthStateInMsg = messaging.SpicePlanetStateMsg().write(earthStateInMsgData)

    # subscribe input messages to module
    module.scStateInMsg.subscribeTo(scStateInMsg)
    module.earthStateInMsg.subscribeTo(earthStateInMsg)
    module.sunStateInMsg.subscribeTo(sunStateInMsg)

    # setup output message recorder objects
    fluxOutMsgRec = module.fluxOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, fluxOutMsgRec)

    # run simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    EnergyData = fluxOutMsgRec.energies[0][0:module.numOutputEnergies]
    ElectronFluxData = np.log10(fluxOutMsgRec.meanElectronFlux[0][0:module.numOutputEnergies])
    IonFluxData = np.log10(fluxOutMsgRec.meanIonFlux[0][0:module.numOutputEnergies])

    # load true data from corresponding support file (note that Python indexing starts at 0 and Fortran indexing
    # starts at 1, relevant for Kp index)
    filename = 'FluxData_' + str(param1_Kp+1) + '_' + str("%.2f" % param2_LT) + '.txt'
    filepath = path + '/Support/' + filename
    with open(filepath, 'r') as file:
        rows = np.loadtxt(file, delimiter=",", unpack=False)
        trueEnergyData = rows[0]
        trueElectronFluxData = rows[1]
        trueIonFluxData = rows[2]

    # make sure module output data is correct
    ParamsString = ' for Kp-Index=' + str(param1_Kp) + ', LT=' + str(param2_LT)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        trueEnergyData, EnergyData, accuracy, ('electron and ion energies' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        trueElectronFluxData, ElectronFluxData, accuracy, ('electron and ion energies' + ParamsString),
        testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        trueIonFluxData, IonFluxData, accuracy, ('electron and ion energies' + ParamsString),
        testFailCount, testMessages)

    # print out success or failure message
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
        print("This test uses an accuracy value of " + str(accuracy) + " (true values don't have higher precision)")
    else:
        print("FAILED " + module.ModelTag)
        print(testMessages)

    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(EnergyData, ElectronFluxData)
    plt.xlabel('Energy [eV]')
    plt.ylabel('log10 Electron Flux [e$^{-}$ cm$^{-2}$ s$^{-1}$ str$^{-1}$ eV$^{-1}$]')

    if show_plots:
        plt.show()

    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    test_dentonFluxModel(True, Kps[1], LTs[0], z_offsets[1], r_EN_Ns[1], 1e-6)