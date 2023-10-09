
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
#   Module Name:        planetEphemeris
#   Author:             Hanspeter Schaub
#   Creation Date:      April 24, 2019
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import planetEphemeris
from Basilisk.utilities import macros
from Basilisk.architecture import bskLogging


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("setRAN", [ True, False])
@pytest.mark.parametrize("setDEC", [ True, False])
@pytest.mark.parametrize("setLST", [ True, False])
@pytest.mark.parametrize("setRate", [ True, False])


# update "module" in this function name to reflect the module name
def test_module(show_plots, setRAN, setDEC, setLST, setRate):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = planetEphemerisTest(show_plots, setRAN, setDEC, setLST, setRate)
    assert testResults < 1, testMessage


def planetEphemerisTest(show_plots, setRAN, setDEC, setLST, setRate):
    bskLogging.setDefaultLogLevel(bskLogging.BSK_SILENT)

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
    module = planetEphemeris.PlanetEphemeris()
    module.ModelTag = 'planetEphemeris'

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    planetNames = ["earth", "venus"]
    module.setPlanetNames(planetEphemeris.StringVector(planetNames))

    # set gravitational constant of the sun

    mu = orbitalMotion.MU_SUN*1000.*1000.*1000  # m^3/s^2
    # setup planet ephemeris states
    oeEarth = planetEphemeris.ClassicElementsMsgPayload()
    oeEarth.a = planetEphemeris.SMA_EARTH*1000  # meters
    oeEarth.e = 0.001
    oeEarth.i = 10.0*macros.D2R
    oeEarth.Omega = 30.0*macros.D2R
    oeEarth.omega = 20.0*macros.D2R
    oeEarth.f = 90.0*macros.D2R

    oeVenus = planetEphemeris.ClassicElementsMsgPayload()
    oeVenus.a = planetEphemeris.SMA_VENUS*1000  # meters
    oeVenus.e = 0.001
    oeVenus.i = 5.0*macros.D2R
    oeVenus.Omega = 110.0*macros.D2R
    oeVenus.omega = 220.0*macros.D2R
    oeVenus.f = 180.0*macros.D2R

    module.planetElements = planetEphemeris.classicElementVector([oeEarth, oeVenus])

    evalAttitude = 1
    if setRAN:
        # setup planet local right ascension angle at epoch
        RANlist = [0.*macros.D2R, 272.76*macros.D2R]
        module.rightAscension = planetEphemeris.DoubleVector(RANlist)
    else:
        evalAttitude = 0

    if setDEC:
        # setup planet local declination angle at epoch
        DEClist = [90.*macros.D2R, 67.16*macros.D2R]
        module.declination = planetEphemeris.DoubleVector(DEClist)
    else:
        evalAttitude = 0

    if setLST:
        # setup planet local sidereal time at epoch
        lstList = [10.*macros.D2R, 30.*macros.D2R]
        module.lst0 = planetEphemeris.DoubleVector(lstList)
    else:
        evalAttitude = 0

    if setRate:
        # setup planet rotation rate about polar axis
        omegaList = [planetEphemeris.OMEGA_EARTH, planetEphemeris.OMEGA_VENUS]
        module.rotRate = planetEphemeris.DoubleVector(omegaList)
    else:
        evalAttitude = 0

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = []
    for c in range(0, len(planetNames)):
        dataLog.append(module.planetOutMsgs[c].recorder())
        unitTestSim.AddModelToTask(unitTaskName, dataLog[-1])

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    accuracy = 1e-3
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    c = 0
    for c in range(0, len(planetNames)):
        planet = planetNames[c]
        J2000Current = dataLog[c].J2000Current
        PositionVector = dataLog[c].PositionVector
        VelocityVector = dataLog[c].VelocityVector
        J20002Pfix = dataLog[c].J20002Pfix
        J20002Pfix_dot = dataLog[c].J20002Pfix_dot
        computeOrient = dataLog[c].computeOrient

        # check that the proper planet name string is set
        FinalPlanetMessage = module.planetOutMsgs[c].read()

        if planet != FinalPlanetMessage.PlanetName:
            testFailCount += 1
            testMessages.append("FAILED: planetEphemeris() didn't set the desired plane name " + planet)

        # check that the time information is correct
        timeTrue = [0.0, 0.5, 1.0]
        testFailCount, testMessages = unitTestSupport.compareDoubleArray(
            timeTrue, J2000Current, accuracy, "J2000Current", testFailCount, testMessages)

        # check that the position and velocity vectors are correct
        if planet == "earth":
            oe = oeEarth
        else:
            oe = oeVenus
        f0 = oe.f
        E0 = orbitalMotion.f2E(f0, oe.e)
        M0 = orbitalMotion.E2M(E0, oe.e)
        rTrue = []
        vTrue = []
        for time in timeTrue:
            Mt = M0 + np.sqrt(mu/oe.a/oe.a/oe.a)*time
            Et = orbitalMotion.M2E(Mt, oe.e)
            oe.f = orbitalMotion.E2f(Et, oe.e)

            rv, vv = orbitalMotion.elem2rv(mu, oe)
            rTrue.append(rv)
            vTrue.append(vv)
        testFailCount, testMessages = unitTestSupport.compareArray(rTrue, PositionVector,
                                                                   accuracy, "Position Vector",
                                                                   testFailCount, testMessages)
        testFailCount, testMessages = unitTestSupport.compareArray(vTrue, VelocityVector,
                                                                   accuracy, "Velocity Vector",
                                                                   testFailCount, testMessages)

        # check if the planet DCM and DCM rate is correct
        dcmTrue = []
        dcmRateTrue = []
        if evalAttitude:
            RAN = RANlist[c]
            DEC = DEClist[c]
            lst0 = lstList[c]
            omega_NP_P = np.array([0.0, 0.0, -omegaList[c]])
            tilde = rbk.v3Tilde(omega_NP_P)
            for time in timeTrue:
                lst = lst0 + omegaList[c]*time
                DCM = rbk.euler3232C([RAN, np.pi/2.0 - DEC, lst])
                dcmTrue.append(DCM)
                dDCMdt = np.matmul(tilde, DCM)
                dcmRateTrue.append(dDCMdt)
        else:
            for time in timeTrue:
                dcmTrue.append(np.identity(3))
                dcmRateTrue.append([0.0]*9)

        testFailCount, testMessages = unitTestSupport.compareArrayND(dcmTrue, J20002Pfix,
                                                                   accuracy, "DCM", 9,
                                                                   testFailCount, testMessages)
        testFailCount, testMessages = unitTestSupport.compareArrayND(dcmRateTrue, J20002Pfix_dot,
                                                                   1e-10, "DCM Rate", 9,
                                                                   testFailCount, testMessages)

        # check if the orientation evaluation flag is set correctly
        flagTrue = [evalAttitude] * 3
        testFailCount, testMessages = unitTestSupport.compareDoubleArray(
            flagTrue, computeOrient, accuracy, "computeOrient", testFailCount, testMessages)

        c = c+1

    # print out success message if no error were found
    snippentName = "passFail" + str(setRAN) + str(setDEC) + str(setLST) + str(setRate)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + module.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + module.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(
                 False,           # show plots flag
                 True,           # setRAN
                 True,           # setDEC
                 True,           # setLST
                 True            # setRate
    )
