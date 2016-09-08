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
import six_dof_eom
import sim_model

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag, biasOn, noiseOn", [
    (False,False,False),
    (False,True,False),
])
# provide a unique test method name, starting with test_
def test_unitSimIMU(show_plots, useFlag, biasOn, noiseOn):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimIMU(show_plots, useFlag, biasOn, noiseOn)
    assert testResults < 1, testMessage


def unitSimIMU(show_plots, useFlag, biasOn, noiseOn):
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
    def setCorruption(biasOn,noiseOn):
        if biasOn:
            rotBiasValue = 1.0
            transBiasValue = 1.0
        else:
            rotBiasValue = 0.0
            transBiasValue = 0.0
        if noiseOn:
            rotNoiseStdValue = 0.333 # 0.000001
            transNoiseStdValue = 0.333 # 1.0E-6
        else:
            rotNoiseStdValue = 0.0
            transNoiseStdValue = 0.0
        return (rotBiasValue, rotNoiseStdValue, transBiasValue, transNoiseStdValue)

    testModule = imu_sensor.ImuSensor()
    testModule.ModelTag = "imusensor"
    testModule.SensorPosStr = imu_sensor.DoubleVector([0.0, 0.0, 0.0])
    testModule.setStructureToPlatformDCM(0.0, 0.0, 0.0)
    testModule.accelLSB = 0.0 # 2.77E-4 * 9.80665
    testModule.gyroLSB = 0.0 # 8.75E-3 * math.pi / 180.0

    (rotBiasValue, rotNoiseStdValue, transBiasValue, transNoiseStdValue) = setCorruption(biasOn,noiseOn)

    testModule.senRotBias = [rotBiasValue, rotBiasValue, rotBiasValue]
    testModule.senRotNoiseStd = [rotNoiseStdValue, rotNoiseStdValue, rotNoiseStdValue]
    testModule.senTransBias = [transBiasValue, transBiasValue, transBiasValue]
    testModule.senTransNoiseStd = [transNoiseStdValue, transNoiseStdValue, transNoiseStdValue]

    # add objects to the task process
    unitSim.AddModelToTask(unitTaskName, testModule)


    # configure and write mass props data
    props = six_dof_eom.MassPropsData()
    props.Mass = -99
    props.CoM = [0,0,0]
    props.InertiaTensor = [-99,0,0,0,-99,0,0,0,-99]
    props.T_str2Bdy = [1,0,0,0,1,0,0,0,1]
    unitSim.TotalSim.CreateNewMessage("TestProcess", "spacecraft_mass_props", 8*3*4, 2)
    unitSim.TotalSim.WriteMessageData("spacecraft_mass_props", 8*3*4, 1, props )


    # configure command message
    unitSim.TotalSim.CreateNewMessage("TestProcess", "inertial_state_output", 8*3*4, 2)

    cmd = six_dof_eom.OutputStateData()
    cmd.r_N = [0,0,0]
    cmd.v_N = [0,0,0]
    cmd.sigma = [0,0,0]
    cmd.omega = [0,0,0]
    # cmd.T_str2Bdy = [[1,0,0],[0,1,0],[0,0,1]]
    # sim_model.doubleArray_setitem(cmd.T_str2Bdy, 0, 1.0)
    cmd.TotalAccumDVBdy = [0,0,0]
    cmd.MRPSwitchCount = 0

    # write module input message
    unitSim.TotalSim.WriteMessageData("inertial_state_output", 8*3*4, 1, cmd )

    # log module output message
    unitSim.TotalSim.logThisMessage(testModule.OutputDataMsg, unitProcRate)

    # run simulation
    unitSim.InitializeSimulation()
    unitSim.ConfigureStopTime(macros.sec2nano(0.5))  # Just a simple run to get initial conditions from ephem
    unitSim.ExecuteSimulation()

    # collect and print the module output data
    data = unitSim.pullMessageLogData(testModule.OutputDataMsg + '.' + "AngVelPlatform", range(3))
    print "\n\ngyro out: \n"
    print data
    data = unitSim.pullMessageLogData(testModule.OutputDataMsg + '.' + "AccelPlatform", range(3))
    print "\n\naccel out: \n"
    print data


    numpy.set_printoptions(precision=16)

    # print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED "

        # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unit test script can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_unitSimIMU(
        False, # show_plots
        False, # useFlag
        False, # biasOn
        False # noiseOn
    )
