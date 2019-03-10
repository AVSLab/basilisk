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
#   Module Name:        magneticField
#   Author:             Hanspeter Schaub
#   Creation Date:      March 10, 2019
#

import pytest
import os, inspect
import numpy as np
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)







# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import magneticField
from Basilisk.simulation import simMessages
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
# from Basilisk.utilities import simSetPlanetEnvironment


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useDefault", [ True, False])
@pytest.mark.parametrize("useMinReach", [ True, False])
@pytest.mark.parametrize("useMaxReach", [ True, False])
@pytest.mark.parametrize("usePlanetEphemeris", [ True, False])


# update "module" in this function name to reflect the module name
def test_module(show_plots, useDefault, useMinReach, useMaxReach, usePlanetEphemeris):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, useDefault, useMinReach, useMaxReach, usePlanetEphemeris)
    assert testResults < 1, testMessage


def run(show_plots, useDefault, useMinReach, useMaxReach, usePlanetEphemeris):
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


    # Construct algorithm and associated C++ container
    testModule = magneticField.MagneticField()
    testModule.ModelTag = "magneticFieldModel"

    if useDefault:
        refg10 = -30926.00/1e9     # Tesla
        refg11 =  -2318.00/1e9     # Tesla
        refh11 =   5817.00/1e9     # Tesla
        refPlanetRadius = 6371.2*1000   # meters
    else:
        refg10 = -30926.00/1e9  # Tesla
        refg11 =  -2318.00/1e9  # Tesla
        refh11 =   5817.00/1e9  # Tesla
        refPlanetRadius = 6371.2 * 1000  # meters
        testModule.dipoleParams.g10 = refg10
        testModule.dipoleParams.g11 = refg11
        testModule.dipoleParams.h11 = refh11

    minReach = -1.0
    if useMinReach:
        minReach = 200*1000.0     # meters
        testModule.envMinReach = minReach
    maxReach = -1.0
    if useMaxReach:
        maxReach = 200*1000.0     # meters
        testModule.envMaxReach = maxReach
    planetPosition = np.array([0.0, 0.0, 0.0])
    refPlanetDCM = np.array(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    if usePlanetEphemeris:
        planetStateMsg = simMessages.SpicePlanetStateSimMsg()
        planetPosition = [1000.0, 2000.0, -1000.0]
        planetStateMsg.PositionVector = planetPosition
        refPlanetDCM = np.array(((-1, 0, 0), (0, -1, 0), (0, 0, 1)))
        planetStateMsg.J20002Pfix = refPlanetDCM.tolist()
        planetStateMsgName = "planet_ephemeris"
        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                   unitProcessName,
                                   planetStateMsgName,
                                   planetStateMsg)
        testModule.planetPosInMsgName = planetStateMsgName


    # add spacecraft to environment model
    sc0StateMsgName = "sc0_state"
    sc1StateMsgName = "sc1_state"
    testModule.addSpacecraftToModel(sc0StateMsgName)
    testModule.addSpacecraftToModel(sc1StateMsgName)

    unitTestSim.AddModelToTask(unitTaskName, testModule)



    # define the spacecraft locations
    r0 = 6571 * 1000.0  # meters
    r1 = 6600 * 1000.0  # meters
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
    oe.a = r1
    r1N, v1N = orbitalMotion.elem2rv(mu, oe)


    # create the input messages
    sc0StateMsg = simMessages.SCPlusStatesSimMsg()  # Create a structure for the input message
    sc0StateMsg.r_BN_N = np.array(r0N) + np.array(planetPosition)
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               sc0StateMsgName,
                               sc0StateMsg)
    sc1StateMsg = simMessages.SCPlusStatesSimMsg()  # Create a structure for the input message
    sc1StateMsg.r_BN_N = np.array(r1N) + np.array(planetPosition)
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               sc1StateMsgName,
                               sc1StateMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    moduleOutputSc0MsgName = testModule.envType + "_0_data"
    unitTestSim.TotalSim.logThisMessage(moduleOutputSc0MsgName, testProcessRate)
    moduleOutputSc1MsgName = testModule.envType + "_1_data"
    unitTestSim.TotalSim.logThisMessage(moduleOutputSc1MsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()


    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    mag0Data = unitTestSim.pullMessageLogData(moduleOutputSc0MsgName + ".magField_N", range(3))
    mag1Data = unitTestSim.pullMessageLogData(moduleOutputSc1MsgName + ".magField_N", range(3))

    def centeredDipole(pos_N, X, refPlanetRadius, refPlanetDCM, minReach, maxReach):
        radius = np.linalg.norm(pos_N)
        planetPos_E = refPlanetDCM.dot(pos_N)
        rHat_E = planetPos_E/radius

        magField_E = (refPlanetRadius/radius)**3 * (3*rHat_E*np.dot(rHat_E, X)-X)

        magField_N = [((refPlanetDCM.transpose()).dot(magField_E)).tolist()]*3

        if radius < minReach:
            magField_N = [[0.0, 0.0, 0.0]]*3
        if radius > maxReach and maxReach > 0:
            magField_N = [[0.0, 0.0, 0.0]]*3
        return magField_N


    # compare the module results to the truth values
    accuracy = 1e-5
    unitTestSupport.writeTeXSnippet("unitTestToleranceValue", str(accuracy), path)


    # check the exponential atmosphere results
    #
    # check spacecraft 0 neutral density results
    if len(mag0Data) > 0:
        trueMagField = centeredDipole(r0N, np.array([refg11, refh11, refg10]), refPlanetRadius, refPlanetDCM, minReach, maxReach)
        testFailCount, testMessages = unitTestSupport.compareArrayRelative(
            trueMagField, mag0Data, accuracy, "SC0 mag vector",
            testFailCount, testMessages)

    if len(mag1Data) > 0:
        trueMagField = centeredDipole(r1N, np.array([refg11, refh11, refg10]), refPlanetRadius, refPlanetDCM, minReach, maxReach)
        testFailCount, testMessages = unitTestSupport.compareArrayRelative(
            trueMagField, mag1Data, accuracy, "SC1 mag vector",
            testFailCount, testMessages)


    #   print out success or failure message
    snippentName = "unitTestPassFail" + str(useDefault) + str(useMinReach) + str(useMaxReach) + str(usePlanetEphemeris)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print "PASSED: " + testModule.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print "Failed: " + testModule.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)


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
                 False,          # useDefault
                 False,         # useMinReach
                 False,         # useMaxReach
                 True          # usePlanetEphemeris
               )
