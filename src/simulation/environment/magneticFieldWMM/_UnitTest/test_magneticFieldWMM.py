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
#   Module Name:        MagneticFieldWMM
#   Author:             Hanspeter Schaub
#   Creation Date:      June 18, 2019
#

import pytest
import os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)





# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import magneticFieldWMM
from Basilisk.simulation import simMessages
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("decimalYear, Height, Lat, Lon, BxTrue, ByTrue, BzTrue", [
      (2015,   0,  80,   0,  6636.6,   -451.9,  54408.9)
    , (2015,   0,   0, 120, 39521.1,    377.7, -11228.8)
    , (2015,   0, -80, 240,  5796.3,  15759.1, -52927.1)
    , (2015, 100,  80,   0,  6323.4,   -477.6,  52249.1)
    , (2015, 100,   0, 120, 37538.1,    351.1, -10751.1)
    , (2015, 100, -80, 240,  5612.2,  14789.3, -50385.8)
    , (2017.5, 0, 80, 0, 6605.2, -298.7, 54506.3)
    , (2017.5, 0, 0, 120, 39569.4, 252.3, -11067.9)
    , (2017.5, 0, -80, 240, 5864.6, 15764.1, -52706.1)
    , (2017.5, 100, 80, 0, 6294.3, -331.1, 52337.8)
    , (2017.5, 100, 0, 120, 37584.4, 235.7, -10600.5)
    , (2017.5, 100, -80, 240, 5674.9, 14793.1, -50179.5)
])
@pytest.mark.parametrize("useDefault, useMsg", [
    (False, False)
    , (False, True)
    , (True, True)
])
@pytest.mark.parametrize("useMinReach", [ True, False])
@pytest.mark.parametrize("useMaxReach", [ True, False])
@pytest.mark.parametrize("usePlanetEphemeris", [ True, False])


# update "module" in this function name to reflect the module name
def test_module(show_plots, decimalYear, Height, Lat, Lon, BxTrue, ByTrue, BzTrue, useDefault, useMsg, useMinReach, useMaxReach, usePlanetEphemeris):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, decimalYear, Height, Lat, Lon, BxTrue, ByTrue, BzTrue, useDefault, useMsg, useMinReach, useMaxReach, usePlanetEphemeris)
    assert testResults < 1, testMessage


def run(show_plots, decimalYear, Height, Lat, Lon, BxTrue, ByTrue, BzTrue, useDefault, useMsg, useMinReach, useMaxReach, usePlanetEphemeris):
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
    testModule = magneticFieldWMM.MagneticFieldWMM()
    testModule.ModelTag = "WMM"
    testModule.dataPath = splitPath[0] + 'Basilisk/supportData/MagneticField/'

    if not useDefault:
        testModule.epochDateFractionalYear = decimalYear

    if useMsg:
        testModule.epochInMsgName = "simEpoch"
        epochMsg = simMessages.EpochSimMsg()
        dt = unitTestSupport.decimalYearToDateTime(decimalYear)
        epochMsg.year = dt.year
        epochMsg.month = dt.month
        epochMsg.day = dt.day
        epochMsg.hours = dt.hour
        epochMsg.minutes = dt.minute
        epochMsg.seconds = dt.second
        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                  unitProcessName,
                                  testModule.epochInMsgName,
                                  epochMsg)
        if not useDefault:
            testModule.epochDateFractionalYear = decimalYear + 1.0

    minReach = -1.0
    if useMinReach:
        minReach = (orbitalMotion.REQ_EARTH+200.)*1000.0     # meters
        testModule.envMinReach = minReach
    maxReach = -1.0
    if useMaxReach:
        maxReach = (orbitalMotion.REQ_EARTH-200.)*1000.0     # meters
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
    r0 = (orbitalMotion.REQ_EARTH + Height) * 1000.0  # meters
    phi = Lat * macros.D2R
    long = Lon * macros.D2R
    r0P = np.array([np.cos(phi)*np.cos(long),np.cos(phi)*np.sin(long),np.sin(phi)])*r0
    r0N = np.dot(refPlanetDCM.transpose(),r0P)

    # create the input messages
    sc0StateMsg = simMessages.SCPlusStatesSimMsg()  # Create a structure for the input message
    sc0StateMsg.r_BN_N = np.array(r0N) + np.array(planetPosition)
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               sc0StateMsgName,
                               sc0StateMsg)
    sc1StateMsg = simMessages.SCPlusStatesSimMsg()  # Create a structure for the input message
    sc1StateMsg.r_BN_N = np.array(r0N) + np.array(planetPosition)
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               sc1StateMsgName,
                               sc1StateMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(testModule.envOutMsgNames[0], testProcessRate)
    unitTestSim.TotalSim.logThisMessage(testModule.envOutMsgNames[1], testProcessRate)

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
    mag0Data = unitTestSim.pullMessageLogData(testModule.envOutMsgNames[0] + ".magField_N", range(3))*1e9
    mag1Data = unitTestSim.pullMessageLogData(testModule.envOutMsgNames[1] + ".magField_N", range(3))*1e9
    print "BSK:" + str(mag0Data)

    def wmmInertial(pos_N, Bx, By, Bz, phi, long, refPlanetDCM, minReach, maxReach):
        radius = np.linalg.norm(pos_N)
        B_M = np.array([Bx, By, Bz])
        M2 = rbk.euler2(phi + np.pi/2.0)
        M3 = rbk.euler3(-long)
        PM = np.dot(M3,M2)
        NM = np.dot(refPlanetDCM.transpose(), PM)
        print NM
        print B_M
        magField_N = [np.dot(NM, B_M).tolist()]


        if radius < minReach:
            magField_N = [[0.0, 0.0, 0.0]]
        if radius > maxReach and maxReach > 0:
            magField_N = [[0.0, 0.0, 0.0]]
        return magField_N


    # compare the module results to the truth values
    accuracy = 1e-1
    unitTestSupport.writeTeXSnippet("unitTestToleranceValue", str(accuracy), path)


    # check the exponential atmosphere results
    #
    # check spacecraft 0 neutral density results
    if len(mag0Data) > 0:
        trueMagField = wmmInertial(r0N, BxTrue, ByTrue, BzTrue, phi, long, refPlanetDCM, minReach, maxReach)

        print "B(true):" + str(trueMagField)
        testFailCount, testMessages = unitTestSupport.compareArray(
            trueMagField, mag0Data, accuracy, "SC0 mag vector",
            testFailCount, testMessages)

    if len(mag1Data) > 0:
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
                 2017.5,        # decimalYear
                 100,             # Height (km)
                 0,            # latitude (deg)
                 120,             # longitude (deg)
                 37584.4,        # BxTrue (nT)
                 235.7,        # ByTrue (nT)
                 -10600.5,       # BzTrue (nT)
                 True,          # useDefault
                 True,         # useMsg
                 False,         # useMinReach
                 False,         # useMaxReach
                 False          # usePlanetEphemeris
               )
