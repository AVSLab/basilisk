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
import inspect
import math
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk import __path__

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.simulation import dentonFluxModel

Kps = ['0o', '4-', '5+']
LTs = [0.00, 14.73]
z_offsets = [0., 3500e3]
r_EN_Ns = np.array([[0., 0., 0.], [400e3, 300e3, -200e3]])

@pytest.mark.parametrize("accuracy", [1e2])
@pytest.mark.parametrize("param1_Kp, param2_LT, param3_z, param4_r_EN", [
    (Kps[0], LTs[0], z_offsets[0], r_EN_Ns[0]),
    (Kps[1], LTs[1], z_offsets[1], r_EN_Ns[0]),
    (Kps[1], LTs[1], z_offsets[0], r_EN_Ns[1]),
    (Kps[1], LTs[1], z_offsets[1], r_EN_Ns[1]),
    (Kps[1], LTs[0], z_offsets[1], r_EN_Ns[1]),
    (Kps[2], LTs[1], z_offsets[1], r_EN_Ns[1]),
    (Kps[2], LTs[0], z_offsets[1], r_EN_Ns[1]),
])

def test_dentonFluxModel(show_plots, param1_Kp, param2_LT, param3_z, param4_r_EN, accuracy):
    r"""
    **Validation Test Description**

    The Denton Flux Module is tested for several different Kp indices, local times, and spacecraft/Sun/Earth positions

    **Test Parameters**

    Args:
        show_plots (bool): specify if plots should be shown
        param1_Kp (str): Kp Index
        param2_LT (float): Local Time (use 2 decimals)
        param3_z (float): z-offset to test spacecraft and Sun position with offset to equatorial plane
        param4_r_EN (float np.array): r_EN_N position vector of Earth w.r.t. N frame, in N frame components
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    The electron and ion energies are compared to make sure the flux data is computed for the same energy. The main
    part of the unitTest is to compare the electron and ion flux.
    """
    dentonFluxModelTestFunction(show_plots, param1_Kp, param2_LT, param3_z, param4_r_EN,
                                                             accuracy)


def dentonFluxModelTestFunction(show_plots, param1_Kp, param2_LT, param3_z, param4_r_EN, accuracy):
    """Test method"""
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
    orbitRadius = 42000 * 1e3  # GEO orbit

    r_BE_N = np.array([orbitRadius * math.cos(angle), orbitRadius * math.sin(angle), param3_z])
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
    energyData = fluxOutMsgRec.energies[0]
    electronFluxData = fluxOutMsgRec.meanElectronFlux[0]
    ionFluxData = fluxOutMsgRec.meanIonFlux[0]

    # convert Kp index to Kp index counter (between 0 and 27)
    kpMain = param1_Kp[0]  # main Kp index, between 0 and 9
    kpSub = param1_Kp[1]  # sub Kp index, either '-', 'o', or '+'
    if kpSub == '-':
        kpIndexCounter = 3*int(kpMain) - 1
    elif kpSub == 'o':
        kpIndexCounter = 3*int(kpMain)
    elif kpSub == '+':
        kpIndexCounter = 3*int(kpMain) + 1

    # load true data from corresponding support file (note that Python indexing starts at 0 and Fortran indexing
    # starts at 1, relevant for Kp index counter)
    filename = 'FluxData_' + str(kpIndexCounter+1) + '_' + str("%.2f" % param2_LT) + '.txt'
    filepath = path + '/Support/' + filename
    trueEnergyData = np.array([0.0] * messaging.MAX_PLASMA_FLUX_SIZE)
    trueElectronFluxData = np.array([0.0] * messaging.MAX_PLASMA_FLUX_SIZE)
    trueIonFluxData = np.array([0.0] * messaging.MAX_PLASMA_FLUX_SIZE)
    with open(filepath, 'r') as file:
        rows = np.loadtxt(file, delimiter=",", unpack=False)
        # true flux data provided by Denton is in Units of [cm^-2 s^-1 sr^-2 eV^-1], but DentonFluxModel converts it to
        # [m^-2 s^-1 sr^-2 eV^-1]. Need to multiply by 1e4
        trueEnergyData[0:module.numOutputEnergies] = rows[0]
        trueElectronFluxData[0:module.numOutputEnergies] = 10.**(rows[1]) * 1e4
        trueIonFluxData[0:module.numOutputEnergies] = 10.**(rows[2]) * 1e4

    # make sure module output data is correct
    paramsString = ' for Kp-Index={}, LT={}, accuracy={}'.format(
        str(param1_Kp),
        str(param2_LT),
        str(accuracy))

    np.testing.assert_allclose(energyData,
                               trueEnergyData,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: energyData,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(electronFluxData,
                               trueElectronFluxData,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: electronFluxData,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(ionFluxData,
                               trueIonFluxData,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: ionFluxData,' + paramsString),
                               verbose=True)

    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    plt.semilogy(energyData[0:module.numOutputEnergies], electronFluxData[0:module.numOutputEnergies])
    plt.xlabel('Energy [eV]')
    plt.ylabel('Electron Flux [e$^{-}$ cm$^{-2}$ s$^{-1}$ str$^{-1}$ eV$^{-1}$]')

    if show_plots:
        plt.show()


if __name__ == "__main__":
    test_dentonFluxModel(False, '4-', LTs[1], z_offsets[1], r_EN_Ns[1], 1e2)
