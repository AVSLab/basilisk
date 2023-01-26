
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
#   Module Name:        atmosphere
#   Author:             Hanspeter Schaub
#   Creation Date:      March 9, 2019
#

import inspect
import math
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simSetPlanetEnvironment


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
    """Module Unit Test"""
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

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    testModule = exponentialAtmosphere.ExponentialAtmosphere()
    testModule.ModelTag = "exponential"

    if useDefault:
        refBaseDens = 0
        refScaleHeight = 1.0
        refPlanetRadius = 0.0
    else:
        simSetPlanetEnvironment.exponentialAtmosphere(testModule, "earth")
        refPlanetRadius = testModule.planetRadius
        refBaseDens = testModule.baseDensity
        refScaleHeight = testModule.scaleHeight

    minReach = -1.0
    if useMinReach:
        minReach = 200*1000.0     # meters
        testModule.envMinReach = minReach
        testModule.planetRadius =  6378136.6 #meters
    maxReach = -1.0
    if useMaxReach:
        maxReach = 200*1000.0     # meters
        testModule.envMaxReach = maxReach
        testModule.planetRadius =  6378136.6
    planetPosition = [0.0, 0.0, 0.0]
    if usePlanetEphemeris:
        planetStateMsg = messaging.SpicePlanetStateMsgPayload()
        planetPosition = [1000.0, 2000.0, -1000.0]
        planetStateMsg.PositionVector = planetPosition
        plMsg = messaging.SpicePlanetStateMsg().write(planetStateMsg)
        testModule.planetPosInMsg.subscribeTo(plMsg)

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
    sc0StateMsg = messaging.SCStatesMsgPayload()  # Create a structure for the input message
    sc0StateMsg.r_BN_N = np.array(r0N) + np.array(planetPosition)
    sc0InMsg = messaging.SCStatesMsg().write(sc0StateMsg)

    sc1StateMsg = messaging.SCStatesMsgPayload()  # Create a structure for the input message
    sc1StateMsg.r_BN_N = np.array(r1N) + np.array(planetPosition)
    sc1InMsg = messaging.SCStatesMsg().write(sc1StateMsg)

    # add spacecraft to environment model
    testModule.addSpacecraftToModel(sc0InMsg)
    testModule.addSpacecraftToModel(sc1InMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog0 = testModule.envOutMsgs[0].recorder()
    dataLog1 = testModule.envOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog0)
    unitTestSim.AddModelToTask(unitTaskName, dataLog1)

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
    dens0Data = dataLog0.neutralDensity
    dens1Data = dataLog1.neutralDensity

    def expAtmoComp(alt, baseDens, scaleHeight, minReach, maxReach):
        density = baseDens * math.exp(-alt/scaleHeight)
        if alt < minReach:
            density = 0.0
        if alt > maxReach and maxReach > 0:
            density = 0.0
        return density

    # compare the module results to the truth values
    accuracy = 1e-5
    unitTestSupport.writeTeXSnippet("unitTestToleranceValue", str(accuracy), path)

    # check the exponential atmosphere results
    #
    # check spacecraft 0 neutral density results
    alt = r0 - refPlanetRadius
    trueDensity = expAtmoComp(alt, refBaseDens, refScaleHeight, minReach, maxReach)
    if trueDensity != 0:
        testFailCount, testMessages = unitTestSupport.compareDoubleArrayRelative(
            [trueDensity]*3, dens0Data, accuracy, "density sc0",
            testFailCount, testMessages)
    else:
        testFailCount, testMessages = unitTestSupport.compareDoubleArray(
            [trueDensity] * 3, dens0Data, accuracy, "density sc0",
            testFailCount, testMessages)

    # check spacecraft 1 neutral density results
    alt = r1 - refPlanetRadius
    trueDensity = expAtmoComp(alt, refBaseDens, refScaleHeight, minReach, maxReach)
    if trueDensity != 0:
        testFailCount, testMessages = unitTestSupport.compareDoubleArrayRelative(
            [trueDensity]*3, dens1Data, accuracy, "density sc1",
            testFailCount, testMessages)
    else:
        testFailCount, testMessages = unitTestSupport.compareDoubleArray(
            [trueDensity] * 3, dens1Data, accuracy, "density sc1",
            testFailCount, testMessages)

    #   print out success or failure message
    snippentName = "unitTestPassFail" + str(useDefault) + str(useMinReach) + str(useMaxReach) + str(usePlanetEphemeris)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + testModule.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + testModule.ModelTag)
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
    test_module(              # update "module" in function name
                 False,         # showplots
                 False,         # useDefault
                 False,         # useMinReach
                 True,         # useMaxReach
                 False          # usePlanetEphemeris
               )
