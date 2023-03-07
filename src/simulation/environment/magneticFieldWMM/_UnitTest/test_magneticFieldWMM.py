
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
#   Module Name:        MagneticFieldWMM
#   Author:             Hanspeter Schaub
#   Creation Date:      June 18, 2019
#

import inspect
import os

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import magneticFieldWMM
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskPath = path.split('src')[0]


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("decimalYear, Height, Lat, Lon, BxTrue, ByTrue, BzTrue", [
      (2020,    0,  80,   0,  6570.4,   -146.3,  54606.0)
    , (2020,    0,   0, 120, 39624.3,    109.9, -10932.5)
    , (2020,    0, -80, 240,  5940.6,  15772.1, -52480.8)
    , (2020,  100,  80,   0,  6261.8,   -185.5,  52429.1)
    , (2020,  100,   0, 120, 37636.7,    104.9, -10474.8)
    , (2020,  100, -80, 240,  5744.9,  14799.5, -49969.4)
    , (2022.5,   0,  80,   0,  6529.9,     1.1,  54713.4)
    , (2022.5,   0,   0, 120, 39684.7,   -42.2, -10809.5)
    , (2022.5,   0, -80, 240,  6016.5, 15776.7, -52251.6)
    , (2022.5, 100,  80,   0,  6224.0,   -44.5,  52527.0)
    , (2022.5, 100,   0, 120, 37694.0,   -35.3, -10362.0)
    , (2022.5, 100, -80, 240,  5815.0, 14803.0, -49755.3)
])
@pytest.mark.parametrize("useDefault, useMsg", [
    (False, False)
    , (False, True)
    , (True, True)
])
@pytest.mark.parametrize("useMinReach", [True, False])
@pytest.mark.parametrize("useMaxReach", [True, False])
@pytest.mark.parametrize("usePlanetEphemeris", [True, False])
@pytest.mark.parametrize("accuracy", [0.1])
# update "module" in this function name to reflect the module name
def test_module(show_plots, decimalYear, Height, Lat, Lon, BxTrue, ByTrue, BzTrue,
                useDefault, useMsg, useMinReach, useMaxReach, usePlanetEphemeris, accuracy):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, decimalYear, Height, Lat, Lon, BxTrue, ByTrue, BzTrue,
                                     useDefault, useMsg, useMinReach, useMaxReach, usePlanetEphemeris, accuracy)
    assert testResults < 1, testMessage


def run(show_plots, decimalYear, Height, Lat, Lon, BxTrue, ByTrue, BzTrue, useDefault, useMsg, useMinReach,
        useMaxReach, usePlanetEphemeris, accuracy):
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
    testModule = magneticFieldWMM.MagneticFieldWMM()
    testModule.ModelTag = "WMM"
    testModule.dataPath = bskPath + '/supportData/MagneticField/'

    if not useDefault:
        testModule.epochDateFractionalYear = decimalYear

    if useMsg:
        epochMsgData = messaging.EpochMsgPayload()
        dt = unitTestSupport.decimalYearToDateTime(decimalYear)
        epochMsgData.year = dt.year
        epochMsgData.month = dt.month
        epochMsgData.day = dt.day
        epochMsgData.hours = dt.hour
        epochMsgData.minutes = dt.minute
        epochMsgData.seconds = dt.second
        epMsg = messaging.EpochMsg().write(epochMsgData)
        testModule.epochInMsg.subscribeTo(epMsg)

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
        planetStateMsg = messaging.SpicePlanetStateMsgPayload()
        planetPosition = [1000.0, 2000.0, -1000.0]
        planetStateMsg.PositionVector = planetPosition
        refPlanetDCM = np.array(((-1, 0, 0), (0, -1, 0), (0, 0, 1)))
        planetStateMsg.J20002Pfix = refPlanetDCM.tolist()
        plMsg = messaging.SpicePlanetStateMsg().write(planetStateMsg)
        testModule.planetPosInMsg.subscribeTo(plMsg)

    # add spacecraft to environment model
    sc0StateMsg = messaging.SCStatesMsg()
    sc1StateMsg = messaging.SCStatesMsg()
    testModule.addSpacecraftToModel(sc0StateMsg)
    testModule.addSpacecraftToModel(sc1StateMsg)

    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # define the spacecraft locations
    r0 = (orbitalMotion.REQ_EARTH + Height) * 1000.0  # meters
    phi = Lat * macros.D2R
    long = Lon * macros.D2R
    r0P = np.array([np.cos(phi)*np.cos(long),np.cos(phi)*np.sin(long),np.sin(phi)])*r0
    r0N = np.dot(refPlanetDCM.transpose(),r0P)

    # create the input messages
    sc0StateMsgData = messaging.SCStatesMsgPayload()  # Create a structure for the input message
    sc0StateMsgData.r_BN_N = np.array(r0N) + np.array(planetPosition)
    sc0StateMsg.write(sc0StateMsgData)

    sc1StateMsgData = messaging.SCStatesMsgPayload()  # Create a structure for the input message
    sc1StateMsgData.r_BN_N = np.array(r0N) + np.array(planetPosition)
    sc1StateMsg.write(sc1StateMsgData)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog0 = testModule.envOutMsgs[0].recorder()
    dataLog1 = testModule.envOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog0)
    unitTestSim.AddModelToTask(unitTaskName, dataLog1)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    unitTestSim.TotalSim.SingleStepProcesses()

    # This pulls the actual data log from the simulation run and converts to nano-Tesla
    mag0Data = dataLog0.magField_N*1e9
    mag1Data = dataLog1.magField_N*1e9

    def wmmInertial(pos_N, Bx, By, Bz, phi, long, refPlanetDCM, minReach, maxReach):
        radius = np.linalg.norm(pos_N)
        B_M = np.array([Bx, By, Bz])
        M2 = rbk.euler2(phi + np.pi/2.0)
        M3 = rbk.euler3(-long)
        PM = np.dot(M3,M2)
        NM = np.dot(refPlanetDCM.transpose(), PM)
        magField_N = [np.dot(NM, B_M).tolist()]

        if radius < minReach:
            magField_N = [[0.0, 0.0, 0.0]]
        if radius > maxReach > 0:
            magField_N = [[0.0, 0.0, 0.0]]
        return magField_N

    # compare the module results to the truth values
    unitTestSupport.writeTeXSnippet("unitTestToleranceValue", str(accuracy), path)

    # check the exponential atmosphere results
    #
    # check spacecraft 0 neutral density results
    if len(mag0Data) > 0:
        trueMagField = wmmInertial(r0N, BxTrue, ByTrue, BzTrue, phi, long, refPlanetDCM, minReach, maxReach)
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
        print("PASSED: " + testModule.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + testModule.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                 False,         # showplots
                 2020,        # decimalYear
                 0,             # Height (km)
                 80,            # latitude (deg)
                 0,             # longitude (deg)
                 6570.4,        # BxTrue (nT)
                 -146.3,        # ByTrue (nT)
                 54606.0,       # BzTrue (nT)
                 True,          # useDefault
                 False,         # useMsg
                 False,         # useMinReach
                 False,         # useMaxReach
                 False,         # usePlanetEphemeris
                 0.1            # accuracy
               )
