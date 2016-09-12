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
import numpy as np
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

# methods
def listStack(vec,simStopTime,unitProcRate):
    return [vec] * int(simStopTime/(float(unitProcRate)/float(macros.sec2nano(1))))

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

testNames = ['zero','bias','noise','discretization','misalignment','CoM offset','walk bounds']

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag, testCase", [
    (False,testNames[0]),
    (False,testNames[1]),
    (False,testNames[2]),
    (False,testNames[3]),
])

# provide a unique test method name, starting with test_
def test_unitSimIMU(show_plots, useFlag, testCase):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimIMU(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage


def unitSimIMU(show_plots, useFlag, testCase):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcName = "TestProcess"  # arbitrary name (don't change)

    # initialize SimulationBaseClass
    unitSim = SimulationBaseClass.SimBaseClass()
    unitSim.TotalSim.terminateSimulation()

    # create the task and specify the integration update time
    unitProcRate = macros.sec2nano(0.1)
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitProc.addTask(unitSim.CreateNewTask(unitTaskName, unitProcRate))


    # configure module
    ImuSensor = imu_sensor.ImuSensor()
    ImuSensor.ModelTag = "imusensor"
    ImuSensor.SensorPosStr = imu_sensor.DoubleVector([0.0, 0.0, 0.0])
    ImuSensor.setStructureToPlatformDCM(0.0, 0.0, 0.0)
    ImuSensor.accelLSB = 0.0 # 2.77E-4 * 9.80665
    ImuSensor.gyroLSB = 0.0 # 8.75E-3 * math.pi / 180.0
    ImuSensor.senRotBias = [0.0] * 3
    ImuSensor.senRotNoiseStd = [0.0] * 3 # 0.000001
    ImuSensor.senTransBias = [0.0] * 3
    ImuSensor.senTransNoiseStd = [0.0] * 3 # 1.0E-6

    # configure MassPropsData
    MassPropsData = six_dof_eom.MassPropsData()
    MassPropsData.Mass = -99
    MassPropsData.CoM = [0,0,0]
    MassPropsData.InertiaTensor = [-99,0,0,0,-99,0,0,0,-99]
    MassPropsData.T_str2Bdy = [1,0,0,0,1,0,0,0,1]

    # configure module input message
    StateCurrent = six_dof_eom.OutputStateData()
    StateCurrent.r_N = [0,0,0]
    StateCurrent.v_N = [0,0,0]
    StateCurrent.sigma = [0,0,0]
    StateCurrent.omega = [0,0,0]
    # StateCurrent.T_str2Bdy = [[1,0,0],[0,1,0],[0,0,1]]
    # sim_model.doubleArray_setitem(StateCurrent.T_str2Bdy, 0, 1.0)
    StateCurrent.TotalAccumDVBdy = [0,0,0]
    StateCurrent.MRPSwitchCount = 0

    # get module output fields
    ImuSensorOutput = imu_sensor.ImuSensorOutput()
    fieldNames = list()
    for fieldName in dir(ImuSensorOutput):
        if fieldName.find('__') < 0 and fieldName.find('this') < 0:
            fieldNames.append(fieldName)

    trueVector = dict()

    # configure test
    if testCase == 'zero':
        # keep everything zero
        simStopTime = 0.5
        for moduleOutputName in fieldNames:
            trueVector[moduleOutputName] = listStack([0.0,0.0,0.0],simStopTime,unitProcRate)

    elif testCase == 'bias':
        # turn static bias on
        simStopTime = 0.5
        ImuSensor.senRotBias = [0.01,0.02,-0.03]
        ImuSensor.senTransBias = [0.001,0.002,-0.003]
        trueVector['AccelPlatform'] = listStack([1.0e-03,2.0e-03,-3.0e-03],simStopTime,unitProcRate)
        trueVector['AngVelPlatform'] = listStack([1.0e-02,2.0e-02,-3.0e-02],simStopTime,unitProcRate)
        trueVector['DRFramePlatform'] = listStack([1.0e-03,2.0e-03,-3.0e-03],simStopTime,unitProcRate)
        trueVector['DVFramePlatform'] = listStack([1.0e-04,2.0e-04,-3.0e-04],simStopTime,unitProcRate)

    elif testCase == 'noise':
        # turn noise on
        simStopTime = 10000.0
        senRotNoiseStd = [1.0,2.0,3.0]
        ImuSensor.senRotNoiseStd = senRotNoiseStd
        senTransNoiseStd = [10.0,20.0,30.0]
        ImuSensor.senTransNoiseStd = senTransNoiseStd
        trueVector['AccelPlatform'] = senTransNoiseStd
        trueVector['AngVelPlatform'] = senRotNoiseStd
        trueVector['DRFramePlatform'] = [x*0.1 for x in senRotNoiseStd]
        trueVector['DVFramePlatform'] = [x*0.1 for x in senTransNoiseStd]

    elif testCase == 'discretization':
        # apply discretization
        simStopTime = 0.5

        accelLSB = 0.0
        # gyroLSB = 0.0123
        gyroLSB = 0.0123
        ImuSensor.accelLSB = accelLSB # 2.77E-4 * 9.80665
        ImuSensor.gyroLSB = gyroLSB # 8.75E-3 * math.pi / 180.0

        omega = [0.321,0.231,0.312]
        StateCurrent.omega = omega

        omegaDiscretized = np.floor(np.asarray(omega)/gyroLSB)*gyroLSB

        trueVector['AngVelPlatform'] = listStack(omegaDiscretized,simStopTime,unitProcRate)

        trueVector['AccelPlatform'] = [
                [0.0,   0.0,  0.0],
                [0.0,   0.0,  0.0],
                [0.0,   0.0,  0.0],
                [0.0,   0.0,  0.0],
                [0.0,   0.0,  0.0]
                ]

        trueVector['DVFramePlatform'] = [
                [0.0,   0.0,  0.0],
                [0.0,   0.0,  0.0],
                [0.0,   0.0,  0.0],
                [0.0,   0.0,  0.0],
                [0.0,   0.0,  0.0]
                ]
        trueVector['DRFramePlatform'] = [
            [-1.20000000e-04,  -9.60000000e-04,  -4.50000000e-04],
             [-1.20000000e-04,  -9.60000000e-04,  -4.50000000e-04],
             [-1.20000000e-04,  -9.60000000e-04,  -4.50000000e-04],
             [-1.20000000e-04,  -9.60000000e-04,  -4.50000000e-04],
             [-1.20000000e-04,  -9.60000000e-04,  -4.50000000e-04]]

    else:
        raise Exception('invalid test case')


    # add module to the task
    unitSim.AddModelToTask(unitTaskName, ImuSensor)

    # configure and write spacecraft_mass_props message
    unitSim.TotalSim.CreateNewMessage("TestProcess", "spacecraft_mass_props", 8*3*4, 2)
    unitSim.TotalSim.WriteMessageData("spacecraft_mass_props", 8*3*4, 1, MassPropsData )

    # configure and write inertial_state_output message
    unitSim.TotalSim.CreateNewMessage("TestProcess", "inertial_state_output", 8*3*4, 2)
    unitSim.TotalSim.WriteMessageData("inertial_state_output", 8*3*4, 1, StateCurrent )

    # log module output message
    unitSim.TotalSim.logThisMessage(ImuSensor.OutputDataMsg, unitProcRate)

    # run simulation
    unitSim.InitializeSimulation()
    unitSim.ConfigureStopTime(macros.sec2nano(simStopTime))  # Just a simple run to get initial conditions from ephem
    unitSim.ExecuteSimulation()

    # pull message log data and assemble into dict
    moduleOutput = dict()
    for moduleOutputName in fieldNames:
        moduleOutput[moduleOutputName] = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + moduleOutputName, range(3))
        print "\n\n" + moduleOutputName
        print str(moduleOutput[moduleOutputName]) + "\n\n"


    # compare the module results to the truth values
    accuracy = 1e-12
    testFail = False
    for moduleOutputName in fieldNames:
        if testCase == 'noise':
            for i in range(0,3):
                if np.abs(np.mean(moduleOutput[moduleOutputName][:,i+1])) > 0.1 \
                                or np.abs(np.std(moduleOutput[moduleOutputName][:,i+1]) - trueVector[moduleOutputName][i]) > 0.1 :
                    testFail = True

        else:
            for i in range(0,len(trueVector[moduleOutputName])):
                if not unitTestSupport.isArrayEqual(moduleOutput[moduleOutputName][i], trueVector[moduleOutputName][i], 3, accuracy):
                    testFail = True

        if testFail:
            testFailCount += 1
            testMessages.append("FAILED: " + ImuSensor.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[moduleOutputName][i,0]*macros.NANO2SEC) +
                                "sec\n")


    np.set_printoptions(precision=16)

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
        testNames[0] # testCase
    )
