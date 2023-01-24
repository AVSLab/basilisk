# 
#  ISC License
# 
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
#   Module Name:        scCharging
#   Author:             Julian Hammerl
#   Creation Date:      January 18, 2023
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
from Basilisk.simulation import scCharging

@pytest.mark.parametrize("accuracy", [1e-2])

def test_scCharging(show_plots, accuracy):
    r"""
    **Validation Test Description**

    !!! Insert Description here

    **Test Parameters**

    Args:
        show_plots (bool): specify if plots should be shown
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    !!! Insert Description here
    """
    [testResults, testMessage] = scChargingTestFunction(show_plots, accuracy)
    assert testResults < 1, testMessage


def scChargingTestFunction(show_plots, accuracy):
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
    module = scCharging.ScCharging()
    module.ModelTag = "scCharging"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Set up spacecraft position vector
    LT = 12.  # GEO local time
    angle = LT * 360./24. * np.pi/180 - np.pi
    orbitRadius = 42000 * 1e3  # GEO orbit
    r_B0N_N = np.array([orbitRadius * math.cos(angle), orbitRadius * math.sin(angle), 0.])
    r_B1B0_N = np.array([0., 10., 0.])
    r_B1N_N = r_B1B0_N + r_B0N_N

    # # # Load plasma data from corresponding support file
    # particle energies
    filepath = path + '/Support/' + 'particleEnergies.txt'
    with open(filepath, 'r') as file:
        EnergyData = np.loadtxt(file, delimiter=",", unpack=False)

    # electron flux
    filepath = path + '/Support/' + 'electronFlux.txt'
    with open(filepath, 'r') as file:
        ElectronFluxData = np.loadtxt(file, delimiter=",", unpack=False)
    # ion flux
    filepath = path + '/Support/' + 'ionFlux.txt'
    with open(filepath, 'r') as file:
        IonFluxData = np.loadtxt(file, delimiter=",", unpack=False)

    # Configure input messages
    sc0StateInMsgData = messaging.SCStatesMsgPayload()
    sc0StateInMsgData.r_BN_N = r_B0N_N
    sc0StateInMsg = messaging.SCStatesMsg().write(sc0StateInMsgData)

    sc1StateInMsgData = messaging.SCStatesMsgPayload()
    sc1StateInMsgData.r_BN_N = r_B1N_N
    sc1StateInMsg = messaging.SCStatesMsg().write(sc1StateInMsgData)

    plasmaFluxInMsgData = messaging.PlasmaFluxMsgPayload()
    plasmaFluxInMsgData.energies = EnergyData
    plasmaFluxInMsgData.meanElectronFlux = ElectronFluxData
    plasmaFluxInMsgData.meanIonFlux = IonFluxData
    plasmaFluxInMsg1 = messaging.PlasmaFluxMsg().write(plasmaFluxInMsgData)

    # add spacecraft to state
    module.addSpacecraft(sc0StateInMsg)
    module.addSpacecraft(sc1StateInMsg)

    # subscribe input messages to module
    module.plasmaFluxInMsg.subscribeTo(plasmaFluxInMsg1)

    # run simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    scPotential0 = module.voltOutMsgs[0].read().voltage
    scPotential1 = module.voltOutMsgs[1].read().voltage
    scPotentials = np.array([scPotential0, scPotential1])

    # load true data from corresponding support file
    filepath = path + '/Support/' + 'trueEqPotential.txt'
    with open(filepath, 'r') as file:
        trueEqPotentials = np.loadtxt(file, delimiter=",", unpack=False)
    trueEqPotentials = np.array([-1000., -1000.])

    # make sure module output data is correct
    testFailCount, testMessages = unitTestSupport.compareDoubleArray(
        trueEqPotentials, scPotentials, accuracy, 'Electric potentials are not correct',
        testFailCount, testMessages)

    # print out success or failure message
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
        print("This test uses an accuracy value of " + str(accuracy))
    else:
        print("FAILED " + module.ModelTag)
        print(testMessages)

    if show_plots:
        plt.show()

    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    test_scCharging(False, 1e-2)
