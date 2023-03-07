#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        sensorThermal
#   Author:             Adam Herrmann
#   Creation Date:      January 12th, 2023
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
from Basilisk.simulation import sensorThermal                    # import the module that is to be tested
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.utilities import macros, astroFunctions


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
# Accuracy parametrization for pytest-html generation
@pytest.mark.parametrize("accuracy", [1e-6])


def test_sensorThermal(show_plots, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the temperature modeling of a sensor with an insulated back and a power input as it
    radiates to the outside environment and takes in heat from the sun. This module tests temperature change in three
    separate cases:

    - Zero: The starting temperature is zero degrees Celsius.
    - Hot: The starting temperature is the converged hot temperature case at approximately 98 degrees Celsius.
    - Cold: the starting temperature is the converged cold temperature case at approximately -75 degrees Celsius.

    The thermal sensor is set to match the parameters in `scenarioSensorThermal.py`, which assumes a 2 kg aluminum
    sensor with the reflectivity and absorptivity coefficients of kapton tape. The sensor is in orbit about the Earth
    and is set to point directly in the direction of the sun.

    **Test Parameters**

    Args:
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    In this file we are checking the values of the variables

    - ``sensorTemp``

    which represents the temperature of the sensor. This array is compared to the ``truthTemp`` array, which contains
    the true values of the temperature.
    """
    [testResults, testMessage] = sensorThermalTest(show_plots, accuracy)
    assert testResults < 1, testMessage


def sensorThermalTest(show_plots, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Create simulation variable names
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.sec2nano(1.)

    #
    #  create the simulation process
    #

    testProcessRate = macros.sec2nano(1.0)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    #
    #   setup the simulation tasks/objects
    #

    #  set device status message
    sensorStatusMsgPayload = messaging.DeviceStatusMsgPayload()
    sensorStatusMsgPayload.deviceStatus = 1
    sensorStatusMsg = messaging.DeviceStatusMsg().write(sensorStatusMsgPayload)

    #  set the spacecraft message
    scStateMsgPayload = messaging.SCStatesMsgPayload()
    scStateMsgPayload.r_BN_N = [6378*1000., 0., 0.]
    scStateMsgPayload.sigma_BN = [0., 0., 0.]
    scStateMsg = messaging.SCStatesMsg().write(scStateMsgPayload)

    #  set the sun message
    sunMsgPayload = messaging.SpicePlanetStateMsgPayload()
    sunMsgPayload.PositionVector = [astroFunctions.AU*1000., 0., 0.]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunMsgPayload)


    #
    #   Setup the temperature modeling
    #
    sensorThermalModel = sensorThermal.SensorThermal()
    sensorThermalModel.ModelTag = 'sensorThermalModel'
    sensorThermalModel.nHat_B = [0, 0, 1]
    sensorThermalModel.sensorArea = 1.0  # m^2
    sensorThermalModel.sensorAbsorptivity = 0.25
    sensorThermalModel.sensorEmissivity = 0.34
    sensorThermalModel.sensorMass = 2.0  # kg
    sensorThermalModel.sensorSpecificHeat = 890
    sensorThermalModel.sensorPowerDraw = 30.0  # W
    sensorThermalModel.T_0 = 0  # [ºC]
    sensorThermalModel.sunInMsg.subscribeTo(sunMsg)
    sensorThermalModel.stateInMsg.subscribeTo(scStateMsg)
    sensorThermalModel.sensorStatusInMsg.subscribeTo(sensorStatusMsg)
    unitTestSim.AddModelToTask(unitTaskName, sensorThermalModel)

    #
    # log data
    #

    # log the RW temperature
    tempLog = sensorThermalModel.temperatureOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, tempLog)

    #
    #   initialize Simulation
    #

    unitTestSim.InitializeSimulation()
    numTests = 0

    # run the first test (nominal)
    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # sensorThermalModel the test conditions
    sensorThermalModel.T_0 = 97.961
    sensorThermalModel.Reset(simulationTime)

    # run the second test (hot case)
    unitTestSim.ConfigureStopTime(2*simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # change the test conditions
    sensorThermalModel.T_0 = -74.8505
    sensorThermalModel.Reset(2*simulationTime)

    # run the second test (cold case)
    unitTestSim.ConfigureStopTime(3*simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    #
    # retrieve the logged data
    #

    sensorTemp = np.array(tempLog.temperature)

    #
    # set the truth vectors
    #

    trueTemp = np.array([0.0, -0.0443986873, 97.7691478, -74.8506601])

    #
    # compare the module results to the true values
    #

    # do the comparison
    for i in range(numTests):
        # check a vector values
        if not unitTestSupport.isDoubleEqual(sensorTemp[i], trueTemp[i], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Sensor Temperature Test failed test " + str(i+1) + "\n")

    if not testFailCount:
        print("PASSED")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# Run this unitTest as a stand-along python script
#
if __name__ == "__main__":
    test_sensorThermal(
        False,  # show_plots
        1e-6    # accuracy
    )
