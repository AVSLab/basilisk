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
#   Purpose:  Run a test of the star tracker module
#   Author:  John Alcorn
#   Creation Date:  October 12, 2016
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
sys.path.append(splitPath[0]+'/basilisk/ADCSAlgorithms/sensorInterfaces/STSensorData')

import MessagingAccess
import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import macros
import star_tracker
import six_dof_eom
import sim_model
import RigidBodyKinematics as rbk
import spice_interface

# methods
def listStack(vec,simStopTime,unitProcRate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(simStopTime/(float(unitProcRate)/float(macros.sec2nano(1))))

def setRandomWalk(self, senNoiseStd = 0.0, errorBounds = [1e6] * 3):
    # sets the module random walk variables
    PMatrix = [0.0] * 3 * 3
    PMatrix[0*3+0] = PMatrix[1*3+1] = PMatrix[2*3+2] = senNoiseStd
    self.PMatrix = sim_model.DoubleVector(PMatrix)
    self.walkBounds = sim_model.DoubleVector(errorBounds)

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed

testNames = ['basic']

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag, testCase", [
    (False,'basic'),
    (False,'time'),
])

# provide a unique test method name, starting with test_
def test_unitSimIMU(show_plots, useFlag, testCase):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimIMU(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage


def unitSimIMU(show_plots, useFlag, testCase):
    testFail = False
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcName = "TestProcess"  # arbitrary name (don't change)

    # initialize SimulationBaseClass
    unitSim = SimulationBaseClass.SimBaseClass()
    unitSim.TotalSim.terminateSimulation()

    # create the task and specify the integration update time
    unitProcRate = macros.sec2nano(0.1)
    unitProcRate_s = macros.NANO2SEC*unitProcRate
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitProc.addTask(unitSim.CreateNewTask(unitTaskName, unitProcRate))

    # configure module
    StarTracker = star_tracker.StarTracker()
    StarTracker.ModelTag = "StarTracker"
    setRandomWalk(StarTracker)

    # configure module input message
    OutputStateData = six_dof_eom.OutputStateData()
    OutputStateData.r_N = [0,0,0]
    OutputStateData.v_N = [0,0,0]
    OutputStateData.sigma = np.array([0,0,0])
    OutputStateData.omega = [0,0,0]
    # OutputStateData.T_str2Bdy = [[1,0,0],[0,1,0],[0,0,1]]
    OutputStateData.TotalAccumDVBdy = [0,0,0]
    OutputStateData.MRPSwitchCount = 0

    SpiceTimeOutput = spice_interface.SpiceTimeOutput()
    SpiceTimeOutput.J2000Current = 0        # s Current J2000 elapsed time double
    SpiceTimeOutput.JulianDateCurrent = 0   # s Current JulianDate double
    SpiceTimeOutput.GPSSeconds = 0          # s Current GPS seconds double
    SpiceTimeOutput.GPSWeek = 0           # -- Current GPS week value uint16_t
    SpiceTimeOutput.GPSRollovers = 0      # -- Count on the number of GPS rollovers uint64_t

    # get module output fields and lengths
    StarTrackerOutput = star_tracker.StarTrackerHWOutput()
    fieldNames = list()
    fieldLengths = list()
    for fieldName in dir(StarTrackerOutput):
        if fieldName.find('__') < 0 and fieldName.find('this') < 0:
            fieldNames.append(fieldName)
            if type(getattr(StarTrackerOutput,fieldName)).__name__ == 'list':
                fieldLengths.append(len(getattr(StarTrackerOutput,fieldName)))
            elif type(getattr(StarTrackerOutput,fieldName)).__name__ == 'float':
                fieldLengths.append(1)

    trueVector = dict()
    print testCase
    if testCase == 'basic':
        # this test verifies basic input and output
        simStopTime = 0.5
        sigma = np.array([-0.390614710591786, -0.503642740963740, 0.462959869561285])
        OutputStateData.sigma = sigma
        trueVector['qInrtl2Case'] = listStack(rbk.MRP2EP(sigma),simStopTime,unitProcRate)
        trueVector['timeTag'] =  np.arange(0,simStopTime+unitProcRate_s,unitProcRate_s)

    elif testCase == 'time':
        simStopTime = 0.5
        J2000Current = 6129.15171032306 # 12-Oct-2016 15:38:27.7719122171402
        SpiceTimeOutput.J2000Current = J2000Current
        trueVector['qInrtl2Case'] = listStack(rbk.MRP2EP(np.array([0,0,0])),simStopTime,unitProcRate)
        trueVector['timeTag'] =  np.arange(J2000Current,J2000Current+simStopTime,unitProcRate_s)

    else:
        raise Exception('invalid test case')

    # add module to the task
    unitSim.AddModelToTask(unitTaskName, StarTracker)

    # log module output message
    unitSim.TotalSim.logThisMessage(StarTracker.outputStateMessage, unitProcRate)

    # configure inertial_state_output message
    OutputStateData_messageSize = 8*3*11
    unitSim.TotalSim.CreateNewMessage("TestProcess", "inertial_state_output", OutputStateData_messageSize, 2)
    unitSim.TotalSim.WriteMessageData("inertial_state_output", OutputStateData_messageSize, 0, OutputStateData )

    # configure spice_time_output_data message
    SpiceTimeOutput_messageSize = 8*3+16+64
    unitSim.TotalSim.CreateNewMessage("TestProcess", "spice_time_output_data", SpiceTimeOutput_messageSize, 2)
    unitSim.TotalSim.WriteMessageData("spice_time_output_data", SpiceTimeOutput_messageSize, 0, SpiceTimeOutput )

    unitSim.InitializeSimulation()
    unitSim.ConfigureStopTime(macros.sec2nano(0.5))
    unitSim.ExecuteSimulation()


    # pull message log data and assemble into dict
    moduleOutput = dict()
    for i in range(0,len(fieldNames)):
        moduleOutputName = fieldNames[i]
        moduleOutput[moduleOutputName] = unitSim.pullMessageLogData(StarTracker.outputStateMessage + '.' + moduleOutputName, range(fieldLengths[i]))
        print "\n\n" + moduleOutputName
        print str(moduleOutput[moduleOutputName]) + "\n"
        print str(trueVector[moduleOutputName]) + "\n\n"


    if not 'accuracy' in vars():
        accuracy = 1e-3

    if len(trueVector[moduleOutputName]) < 1:
        raise Exception('something went wrong')

    for j in range(0,len(fieldNames)):
        moduleOutputName = fieldNames[j]
        if testCase == 'noise':
            for i in range(0,3):
                if np.abs(np.mean(moduleOutput[moduleOutputName][:,i+1])) > 0.1 \
                                or np.abs(np.std(moduleOutput[moduleOutputName][:,i+1]) - trueVector[moduleOutputName][i]) > 0.1 :
                    testFail = True

        elif testCase == 'walk bounds':
            for i in range(0,3):
                if np.max(np.abs(np.asarray(moduleOutput[moduleOutputName][:,i+1]))) > trueVector[moduleOutputName][i]:
                    testFail = True

        else:
            for i in range(0,len(trueVector[moduleOutputName])):
                if fieldLengths[j] > 1:
                    if not unitTestSupport.isArrayEqual(moduleOutput[moduleOutputName][i], trueVector[moduleOutputName][i], fieldLengths[j], accuracy):
                        testFail = True
                else:
                    if not unitTestSupport.isDoubleEqual(moduleOutput[moduleOutputName][i], trueVector[moduleOutputName][i], accuracy):
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


# This statement below ensures that the unit test script can be run as a
# stand-along python script
if __name__ == "__main__":
    test_unitSimIMU(
        False, # show_plots
        False, # useFlag
        'time' # testCase
    )
