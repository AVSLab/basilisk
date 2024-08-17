#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        tabularAtmosphere
#   Author:             Mikaela Felix
#   Creation Date:      Feb 11, 2022
#

import os

import numpy as np
import pytest
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import tabularAtmosphere
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities.readAtmTable import readAtmTable

@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("altitude", [42.0, 33.33333, 10000.0, -10.0]) # exact, interpolate, above, below
@pytest.mark.parametrize("useMinReach", [ True, False])
@pytest.mark.parametrize("useMaxReach", [ True, False])

def test_tabularAtmosphere(altitude, accuracy, useMinReach, useMaxReach):
    r"""
    **Validation Test Description**

    TabularAtmosphere interpolates from user-provided data to compute density and temperature at the current s/c
    altitude. The unit test checks altitudes at, between, above, and below the values included in the table. This
    test uses a python helper function to provide data from EarthGRAM (see supportData\AtmosphereData\support).
    Data lists can also be manually-input, but check sorting and units per documentation and support info (above).
    The module returns 0 for both density and temperature if ANY ONE of the following conditions is met:

    - altitude below minimum value in provided table
    - altitude above maximum value in provided table
    - altitude below envMinReach
    - altitude above envMaxReach

    Note that this results in nonphysical behavior for temperature (absolute zero) when outside defined range.

    **Test Parameters**

    Args:

    - altitude (float): Spacecraft altitude for which density, temperature are returned
    - accuracy (float): accuracy value used in validation tests
    - useMinReach (bool): set value of envMinReach
    - useMaxReach (bool): set value of envMaxReach

    **Description of Variables Being Tested**

    The unit test checks density (kg/m^3) and temperature (K) against their expected values:

    - ``densData[0]``
    - ``tempData[0]``
    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = tabularAtmosphereTestFunction(altitude, accuracy, useMinReach, useMaxReach)
    assert testResults < 1, testMessage

def tabularAtmosphereTestFunction(altitude, accuracy, useMinReach, useMaxReach):
    testFailCount = 0                       # zero unit test result counter
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = tabularAtmosphere.TabularAtmosphere()   # update with current values
    module.ModelTag = "tabularAtmosphere"            # update python name of test module

    # define constants & load data
    r_eq = 6378136.6
    filename = bskPath + '/supportData/AtmosphereData/EarthGRAMNominal.txt'
    altList, rhoList, tempList = readAtmTable(filename,'EarthGRAM')

    # assign constants & ref. data to module
    module.planetRadius = r_eq
    module.altList = tabularAtmosphere.DoubleVector(altList)
    module.rhoList = tabularAtmosphere.DoubleVector(rhoList)
    module.tempList = tabularAtmosphere.DoubleVector(tempList)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # CHECK - env min and max
    if useMinReach:
        minReach = 50.0 * 1000
        module.envMinReach = minReach
    else:
        minReach = -1.0 * 1000
    if useMaxReach:
        maxReach = 20.0 * 1000
        module.envMaxReach = maxReach
    else:
        maxReach = 5000.0 * 1000

    #   setup orbit and simulation time
    r0 = r_eq + (altitude * 1000.0)  # meters
    oe = orbitalMotion.ClassicElements()
    mu = 0.3986004415E+15  # meters^3/s^2
    oe.a = r0
    oe.e = 0.0
    oe.i = 45.0 * macros.D2R
    oe.Omega = 30.0 * macros.D2R
    oe.omega = 120.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    r0N, v0N = orbitalMotion.elem2rv(mu, oe)

    # create the input messages
    scStateMsg = messaging.SCStatesMsgPayload()  # Create a structure for the input message
    scStateMsg.r_BN_N = np.array(r0N)
    scInMsg = messaging.SCStatesMsg().write(scStateMsg)

    # add spacecraft to environment model
    module.addSpacecraftToModel(scInMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.envOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

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
    densData = dataLog.neutralDensity
    tempData = dataLog.localTemp

    # define python function to compute truth values
    def tabAtmoComp(val, xList, yList):
        if (val < xList[0]) or (val <= minReach):
            out = 0.0
            return out
        elif (val > xList[-1]) or (val >= maxReach):
            out = 0.0
            return out
        else:
            for i, x in enumerate(xList):
                if x >= val:
                    x0 = xList[i-1]
                    y0 = yList[i-1]
                    y1 = yList[i]
                    m = (y1 - y0)/(x - x0)
                    out = y0 + (val - x0) * m
                    return out

    # compute truth values
    trueDensity = tabAtmoComp(altitude * 1000, altList, rhoList)
    print('\nmodule density: {0:.6e}'.format(densData[0]))
    print('true density: {0:.6e}'.format(trueDensity))

    trueTemp = tabAtmoComp(altitude * 1000, altList, tempList)
    print('\nmodule temperature: {0:.6e}'.format(tempData[0]))
    print('true temperature: {0:.6e}\n'.format(trueTemp))

    # compare truth values to module results
    if trueDensity != 0:
        testFailCount = not unitTestSupport.isDoubleEqualRelative(densData[0], trueDensity, accuracy)
    else:
        testFailCount = not unitTestSupport.isDoubleEqual(densData[0], trueDensity, accuracy)
    if testFailCount == 0:
        testMessage = "density computed correctly"
    else:
        testMessage = "density computed incorrectly"

    # compare truth values to module results for temperature
    if trueTemp != 0 :    # needs checking
        testFailCount = not unitTestSupport.isDoubleEqualRelative(tempData[0], trueTemp, accuracy)
    else:
        testFailCount = not unitTestSupport.isDoubleEqual(tempData[0], trueTemp, accuracy)
    if testFailCount == 0:
        testMessage += " and temperature computed correctly"
    else:
        testMessage += " and temperature computed incorrectly"

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)

    return [testFailCount, testMessage]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_tabularAtmosphere(
                 10000.0,          # altitude
                 1e-12,       # accuracy
                 True,
                 True
               )
