''' '''
'''
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#   Module Name:        Magnetometer - TAM
#   Author:             Demet Cilden-Guler
#   Creation Date:      September 25, 2019
#

import pytest
import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

bskPath = path.split('src')[0]

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import magneticFieldWMM
from Basilisk.simulation import magneticFieldCenteredDipole
from Basilisk.simulation import magnetometer
from Basilisk.simulation import simMessages
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import simSetPlanetEnvironment

# methods
def wmmInertial(Bx, By, Bz, phi, long, refPlanetDCM):
    B_M = np.array([Bx, By, Bz])
    M2 = rbk.euler2(phi + np.pi / 2.0)
    M3 = rbk.euler3(-long)
    PM = np.dot(M3, M2)
    NM = np.dot(refPlanetDCM.transpose(), PM)
    magField_N = [np.dot(NM, B_M).tolist()]
    return magField_N  # nT

def centeredDipole(pos_N, X, refPlanetRadius, refPlanetDCM):
    radius = np.linalg.norm(pos_N)
    planetPos_E = refPlanetDCM.dot(pos_N)
    rHat_E = planetPos_E/radius
    magField_E = (refPlanetRadius/radius)**3 * (3*rHat_E*np.dot(rHat_E, X)-X)
    magField_N = [((refPlanetDCM.transpose()).dot(magField_E)).tolist()]
    return magField_N  # nT

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.

# @pytest.mark.parametrize("noiseStd, errTol", [(0, 1e-10), (300e-9, 1e-6)])
# @pytest.mark.parametrize("bias", [(0.0, 0.0, 0.0), (1e-6, 1e-6, 1e-5)])
# @pytest.mark.parametrize("minOut, maxOut", [(-1e3, 1e3), (-1e-4, 1e-4)])
# @pytest.mark.parametrize("magModel", ['WMM', 'CenteredDipole'])

@pytest.mark.parametrize(
    "show_plots, noiseStd, bias, minOut, maxOut, magModel, errTol, name",
    [
        (False, 0, (0, 0, 0), -1e3, 1e3, "WMM", 1e-10, "cleanwmm"),
        (False, 0, (1e-6, 1e-6, 1e-5), -1e3, 1e3, "WMM", 1e-10, "biaswmm"),
        (False, 0, (0, 0, 0), -1e-4, 1e-4, "WMM", 1e-10, "boundwmm"),
        (False, 300e-9, (0, 0, 0), -1e3, 1e3, "WMM", 1e-6, "noisewmm"),
        (False, 0, (1e-6, 1e-6, 1e-5), -1e-4, 1e-4, "WMM", 1e-10, "boundbiaswmm"),
        (False, 300e-9, (0, 0, 0), -1e-4, 1e-4, "WMM", 1e-6, "boundnoisewmm"),
        (False, 300e-9, (1e-6, 1e-6, 1e-5), -1e3, 1e3, "WMM", 1e-6, "noisebiaswmm"),
        (False, 300e-9, (1e-6, 1e-6, 1e-5), -1e-4, 1e-4, "WMM", 1e-6, "combinedwmm"),
        (False, 0, (0, 0, 0), -1e3, 1e3, "CenteredDipole", 1e-10, "cleandipole"),
        (False, 0, (1e-6, 1e-6, 1e-5), -1e3, 1e3, "CenteredDipole", 1e-10, "biasdipole"),
        (False, 0, (0, 0, 0), -1e-4, 1e-4, "CenteredDipole", 1e-10, "bounddipole"),
        (False, 300e-9, (0, 0, 0), -1e3, 1e3, "CenteredDipole", 1e-6, "noisedipole"),
        (False, 0, (1e-6, 1e-6, 1e-5), -1e-4, 1e-4, "CenteredDipole", 1e-10, "boundbiasdipole"),
        (False, 300e-9, (0, 0, 0), -1e-4, 1e-4, "CenteredDipole", 1e-6, "boundnoisedipole"),
        (False, 300e-9, (1e-6, 1e-6, 1e-5), -1e3, 1e3, "CenteredDipole", 1e-6, "noisebiasdipole"),
        (False, 300e-9, (1e-6, 1e-6, 1e-5), -1e-4, 1e-4, "CenteredDipole", 1e-6, "combineddipole"),
    ])

# update "module" in this function name to reflect the module name
def test_module(show_plots, noiseStd, bias, minOut, maxOut, magModel, errTol, name):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, noiseStd, bias, minOut, maxOut, magModel, errTol, name)
    assert testResults < 1, testMessage

def run(show_plots, noiseStd, bias, minOut, maxOut, magModel, errTol, name):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    if magModel == 'WMM':
        # Construct algorithm and associated C++ container
        magModule = magneticFieldWMM.MagneticFieldWMM()
        magModule.ModelTag = "WMM"
        magModule.dataPath = bskPath + '/supportData/MagneticField/'
        decimalYear = 2015
        Height = 0
        Lat = 80
        Lon = 0
        BxTrue = 6636.6 / 1e9
        ByTrue = -451.9 / 1e9
        BzTrue = 54408.9 / 1e9
        magModule.epochDateFractionalYear = decimalYear
        # define the spacecraft locations
        r0 = (orbitalMotion.REQ_EARTH + Height) * 1000.0  # meters
        phi = Lat * macros.D2R
        long = Lon * macros.D2R
        r0P = np.array([np.cos(phi) * np.cos(long), np.cos(phi) * np.sin(long), np.sin(phi)]) * r0
        refPlanetDCM = np.array(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
        r0N = np.dot(refPlanetDCM.transpose(), r0P)
    else: # Centered Dipole
        magModule = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
        magModule.ModelTag = "CenteredDipole"
        simSetPlanetEnvironment.centeredDipoleMagField(magModule, "earth")
        refg10 = magModule.g10
        refg11 = magModule.g11
        refh11 = magModule.h11
        refPlanetRadius = magModule.planetRadius
        refPlanetDCM = np.array(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
        # define the spacecraft locations
        r0 = 6571 * 1000.0  # meters
        #
        #   setup orbit and simulation time
        oe = orbitalMotion.ClassicElements()
        mu = 0.3986004415E+15  # meters^3/s^2
        oe.a = r0
        oe.e = 0.0
        oe.i = 45.0 * macros.D2R
        oe.Omega = 30.0 * macros.D2R
        oe.omega = 120.0 * macros.D2R
        oe.f = 0.0 * macros.D2R
        r0N, v0N = orbitalMotion.elem2rv(mu, oe)

    planetPosition = np.array([0.0, 0.0, 0.0])
    # add spacecraft to environment model
    scStateMsgName = "sc_state"
    magModule.addSpacecraftToModel(scStateMsgName)
    unitTestSim.AddModelToTask(unitTaskName, magModule)

    # Construct algorithm and associated C++ container
    yaw = 0.7854  # should be given as parameter [rad]
    pitch = 1.0  # [rad]
    roll = 0.1  # [rad]
    testModule = magnetometer.Magnetometer()
    testModule.ModelTag = "TAM_sensor"
    testModule.scaleFactor = 1.0
    testModule.tamDataOutMsgName = "TAM_output"
    testModule.senNoiseStd = noiseStd
    testModule.senBias = bias  # Tesla
    testModule.minOutput = minOut
    testModule.maxOutput = maxOut
    testModule.setBodyToSensorDCM(yaw, pitch, roll)
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # create the input messages
    scStateMsg = simMessages.SCPlusStatesSimMsg()  # Create a structure for the input message
    scStateMsg.r_BN_N = np.array(r0N) + np.array(planetPosition)
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               scStateMsgName,
                               scStateMsg)
    testModule.stateIntMsgName = scStateMsgName

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(magModule.envOutMsgNames[0], testProcessRate)
    unitTestSim.TotalSim.logThisMessage(testModule.tamDataOutMsgName, testProcessRate)
    testModule.magIntMsgName = magModule.envOutMsgNames[0]

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    unitTestSim.TotalSim.SingleStepProcesses()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    # unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    # unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    magData = unitTestSim.pullMessageLogData(magModule.envOutMsgNames[0] + ".magField_N", list(range(3)))
    tamData = unitTestSim.pullMessageLogData(testModule.tamDataOutMsgName + ".OutputData", list(range(3)))

    # compare the module results to the truth values
    unitTestSupport.writeTeXSnippet("unitTestToleranceValue" + str(errTol), str(errTol), path)


    if magModel == 'WMM':
        trueMagField = wmmInertial(BxTrue, ByTrue, BzTrue, phi, long, refPlanetDCM)
    else: # CenteredDipole
        trueMagField = centeredDipole(r0N, np.array([refg11, refh11, refg10]), refPlanetRadius, refPlanetDCM)

    # check the exponential atmosphere results
    #
    # check spacecraft magnetic field measurements
    if len(tamData) > 0:
        dcm_BN = rbk.MRP2C(scStateMsg.sigma_BN)
        dcm_SB = rbk.euler3212C([yaw, pitch, roll])
        dcm_SN = np.dot(dcm_SB, dcm_BN)
        magField_S = np.transpose(np.dot(dcm_SN, np.transpose(trueMagField)))
        magField_S = magField_S + bias
        for k in range(len(magField_S[0])):
            if magField_S[0][k] > maxOut:
                magField_S[0][k] = maxOut
            if magField_S[0][k] < minOut:
                magField_S[0][k] = minOut

        testFailCount, testMessages = unitTestSupport.compareArray(
            magField_S, tamData, errTol, "SC magnetometer",
            testFailCount, testMessages)
    else:
        print("Length of the magnetic field measurement vector is zero!")

    #   print out success or failure message
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + testModule.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + testModule.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    # Write some snippets for AutoTex
    snippentName = "PassedText" + name
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    # write pytest parameters to AutoTex folder
    # "noiseStd, bias, minOut, maxOut, magModel, errTol"
    biasSnippetName = name + "Bias1"
    biasSnippetContent = '{:1.1e}'.format(bias[0])
    unitTestSupport.writeTeXSnippet(biasSnippetName, biasSnippetContent, path)
    biasSnippetName = name + "Bias2"
    biasSnippetContent = '{:1.1e}'.format(bias[1])
    unitTestSupport.writeTeXSnippet(biasSnippetName, biasSnippetContent, path)
    biasSnippetName = name + "Bias3"
    biasSnippetContent = '{:1.1e}'.format(bias[2])
    unitTestSupport.writeTeXSnippet(biasSnippetName, biasSnippetContent, path)
    #
    noiseStdSnippetName = name + "NoiseStd"
    noiseStdSnippetContent = '{:1.1e}'.format(noiseStd)
    unitTestSupport.writeTeXSnippet(noiseStdSnippetName, noiseStdSnippetContent, path)
    #
    saturationMaxSnippetName = name + "MaxSaturation"
    saturationMaxSnippetContent = '{:1.1e}'.format(maxOut)
    unitTestSupport.writeTeXSnippet(saturationMaxSnippetName, saturationMaxSnippetContent, path)
    #
    saturationMinSnippetName = name + "MinSaturation"
    saturationMinSnippetContent = '{:1.1e}'.format(minOut)
    unitTestSupport.writeTeXSnippet(saturationMinSnippetName, saturationMinSnippetContent, path)

    magModelSnippetName = name + "magModel"
    magModelSnippetContent = '{:.14s}'.format(magModel)
    unitTestSupport.writeTeXSnippet(magModelSnippetName, magModelSnippetContent, path)
    #
    errTolSnippetName = name + "ErrTol"
    errTolSnippetContent = '{:1.1e}'.format(errTol)
    unitTestSupport.writeTeXSnippet(errTolSnippetName, errTolSnippetContent, path)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                 False,         # showplots
                 0,          # noiseStd
                 [0, 0, 0],   # bias
                 -1e-5,           # minOut
                 1e200,           # maxOut
                 'WMM',            # magModel (WMM, CenteredDipole)
                 1e-5,            # errTol
                 'clean'         # name
               )
