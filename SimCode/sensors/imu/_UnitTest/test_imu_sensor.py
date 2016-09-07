'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#   Integrated Unit Test Script
#   Purpose:  Run a test of the IMU sensor module
#   Author:  John Alcorn
#   Creation Date:  September 6, 2016
#

import pytest
import sys, os, inspect
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy
import ctypes
import math
import csv
import logging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('basilisk')
sys.path.append(splitPath[0]+'/basilisk/modules')
sys.path.append(splitPath[0]+'/basilisk/PythonModules')

import MessagingAccess
import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import macros
import imu_sensor

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag", [
    (False),
])
# provide a unique test method name, starting with test_
def test_unitSimIMU(show_plots, useFlag):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimIMU(show_plots, useFlag)
    assert testResults < 1, testMessage


def unitSimIMU(show_plots, useFlag):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcName = "TestProcess"  # arbitrary name (don't change)

    unitSim = SimulationBaseClass.SimBaseClass()
    unitSim.TotalSim.terminateSimulation()

    # create the task and specify the integration update time
    unitProcRate = macros.sec2nano(0.1)
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitProc.addTask(unitSim.CreateNewTask(unitTaskName, unitProcRate))

    # module specific data
    def turnOffCorruption():
        rotBiasValue = 0.0
        rotNoiseStdValue = 0.0
        transBiasValue = 0.0
        transNoiseStdValue = 0.0
        return (rotBiasValue, rotNoiseStdValue, transBiasValue, transNoiseStdValue)

    rotBiasValue = 0.0
    rotNoiseStdValue = 0.000001
    transBiasValue = 0.0
    transNoiseStdValue = 1.0E-6

    testModule = imu_sensor.ImuSensor()
    testModule.ModelTag = "imusensor"
    testModule.SensorPosStr = imu_sensor.DoubleVector([1.5, 0.1, 0.1])
    testModule.setStructureToPlatformDCM(0.0, 0.0, 0.0)
    testModule.accelLSB = 2.77E-4 * 9.80665
    testModule.gyroLSB = 8.75E-3 * math.pi / 180.0

    # Turn off corruption of IMU data
    (rotBiasValue, rotNoiseStdValue, transBiasValue, transNoiseStdValue) = turnOffCorruption()

    testModule.senRotBias = [rotBiasValue, rotBiasValue, rotBiasValue]
    testModule.senRotNoiseStd = [rotNoiseStdValue, rotNoiseStdValue, rotNoiseStdValue]
    testModule.senTransBias = [transBiasValue, transBiasValue, transBiasValue]
    testModule.senTransNoiseStd = [transNoiseStdValue, transNoiseStdValue, transNoiseStdValue]

    # add objects to the task process
    unitSim.AddModelToTask(unitTaskName, testModule)

    # log data
    unitSim.TotalSim.logThisMessage(testModule.OutputDataMsg, unitProcRate)

    # run simulation
    unitSim.InitializeSimulation()
    unitSim.ConfigureStopTime(macros.sec2nano(1.0))  # Just a simple run to get initial conditions from ephem
    unitSim.ExecuteSimulation()

    # dataSigma = unitSim.pullMessageLogData("inertial_state_output.sigma", range(3))
    data = unitSim.pullMessageLogData(testModule.OutputDataMsg + '.' + "AngVelPlatform", range(3))
    print data


    numpy.set_printoptions(precision=16)

    # print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED "

        # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_unitSimIMU(False,False)   # show_plots
