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
#   Integrated Unit Test Script
#   Purpose:  Run a test of the star tracker module
#   Author:  John Alcorn
#   Creation Date:  October 12, 2016
#

import pytest
import sys, os, inspect
import numpy as np
import ctypes
import math
import csv
import logging

from Basilisk.utilities import MessagingAccess
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.simulation import star_tracker
from Basilisk.simulation import sim_model
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import spice_interface

# methods
def listStack(vec,simStopTime,unitProcRate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(simStopTime/(float(unitProcRate)/float(macros.sec2nano(1))))

def setRandomWalk(self, senNoiseStd = 0.0, errorBounds = [[1e6],[1e6],[1e6]]):
    # sets the module random walk variables
    PMatrix = [[senNoiseStd, 0., 0.], [0., senNoiseStd, 0.], [0., 0., senNoiseStd]]
    self.PMatrix = PMatrix
    self.walkBounds = errorBounds

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag, testCase", [
    (False,'basic'),
    (False,'noise'),
    (False,'walk bounds')
])

# provide a unique test method name, starting with test_
def test_unitSimStarTracker(show_plots, useFlag, testCase):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimStarTracker(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage


def unitSimStarTracker(show_plots, useFlag, testCase):
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
    OutputStateData = star_tracker.SCPlusStatesSimMsg()
    OutputStateData.r_BN_N = [0,0,0]
    OutputStateData.v_BN_N = [0,0,0]
    OutputStateData.sigma_BN = np.array([0,0,0])
    OutputStateData.omega_BN_B = [0,0,0]
    OutputStateData.TotalAccumDVBdy = [0,0,0]
    OutputStateData.MRPSwitchCount = 0

    SpiceTimeOutput = spice_interface.SpiceTimeSimMsg()
    SpiceTimeOutput.J2000Current = 0        # s Current J2000 elapsed time double
    SpiceTimeOutput.JulianDateCurrent = 0   # s Current JulianDate double
    SpiceTimeOutput.GPSSeconds = 0          # s Current GPS seconds double
    SpiceTimeOutput.GPSWeek = 0           # -- Current GPS week value uint16_t
    SpiceTimeOutput.GPSRollovers = 0      # -- Count on the number of GPS rollovers uint64_t

    # get module output fields and lengths
    StarTrackerOutput = star_tracker.STSensorIntMsg()
    fieldNames = list()
    fieldLengths = list()
    for fieldName in dir(StarTrackerOutput):
        if fieldName.find('__') < 0 and fieldName.find('this') < 0:
            if(callable(getattr(StarTrackerOutput,fieldName))):
                continue
            fieldNames.append(fieldName)
            if type(getattr(StarTrackerOutput,fieldName)).__name__ == 'list':
                fieldLengths.append(len(getattr(StarTrackerOutput,fieldName)))
            elif isinstance(getattr(StarTrackerOutput,fieldName), (float, long, int)):
                fieldLengths.append(1)

    trueVector = dict()
    print testCase
    if testCase == 'basic':
        # this test verifies basic input and output
        simStopTime = 0.5
        sigma = np.array([-0.390614710591786, -0.503642740963740, 0.462959869561285])
        OutputStateData.sigma_BN = sigma
        J2000Current = 6129.15171032306 # 12-Oct-2016 15:38:27.7719122171402
        SpiceTimeOutput.J2000Current = J2000Current
        trueVector['qInrtl2Case'] = listStack(rbk.MRP2EP(sigma),simStopTime,unitProcRate)
        trueVector['timeTag'] =  np.arange(0,0+simStopTime*1E9,unitProcRate_s*1E9)

    elif testCase == 'noise':
        simStopTime = 1000.
        noiseStd = 0.1
        stdCorrectionFactor = 1.5 # this needs to be used because of the Gauss Markov module. need to fix the GM module
        setRandomWalk(StarTracker, noiseStd*stdCorrectionFactor, [[1.0e-13],[1.0e-13],[1.0e-13]])
        sigma = np.array([0,0,0])
        OutputStateData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = [noiseStd] * 3
        trueVector['timeTag'] =  np.arange(0,0+simStopTime*1E9,unitProcRate_s*1E9)

    elif testCase == 'walk bounds':
        # this test checks the walk bounds of random walk
        simStopTime = 1000.
        noiseStd = 0.01
        stdCorrectionFactor = 1.5 # this needs to be used because of the Gauss Markov module. need to fix the GM module
        walkBound = 0.1
        setRandomWalk(StarTracker, noiseStd*stdCorrectionFactor, [[walkBound],[walkBound],[walkBound]])
        sigma = np.array([0,0,0])
        OutputStateData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = [walkBound + noiseStd*3] * 3
        trueVector['timeTag'] =  np.arange(0,0+simStopTime*1E9,unitProcRate_s*1E9)

    else:
        raise Exception('invalid test case')

    # add module to the task
    unitSim.AddModelToTask(unitTaskName, StarTracker)

    # log module output message
    unitSim.TotalSim.logThisMessage(StarTracker.outputStateMessage, unitProcRate)

    # configure inertial_state_output message
    # OutputStateData_messageSize = 8*3*11
    OutputStateData_messageSize = OutputStateData.getStructSize()
    unitSim.TotalSim.CreateNewMessage("TestProcess", "inertial_state_output", OutputStateData_messageSize, 2)
    unitSim.TotalSim.WriteMessageData("inertial_state_output", OutputStateData_messageSize, 0, OutputStateData )

    # configure spice_time_output_data message
    SpiceTimeOutput_messageSize = 8*3+16+64
    SpiceTimeOutput_messageSize = SpiceTimeOutput.getStructSize()
    unitSim.TotalSim.CreateNewMessage("TestProcess", "spice_time_output_data", SpiceTimeOutput_messageSize, 2)
    unitSim.TotalSim.WriteMessageData("spice_time_output_data", SpiceTimeOutput_messageSize, 0, SpiceTimeOutput )

    unitSim.InitializeSimulation()
    unitSim.ConfigureStopTime(macros.sec2nano(simStopTime))
    unitSim.ExecuteSimulation()


    # pull message log data and assemble into dict
    moduleOutput = dict()
    for i in range(0,len(fieldNames)):
        moduleOutputName = fieldNames[i]
        moduleOutput[moduleOutputName] = unitSim.pullMessageLogData(StarTracker.outputStateMessage + '.' + moduleOutputName, range(fieldLengths[i]))




    # convert quaternion output to prv
    moduleOutput['prvInrtl2Case'] = np.zeros([int(simStopTime/unitProcRate_s)+1,3])
    for i in range(0,int(simStopTime/unitProcRate_s)+1):
        moduleOutput['prvInrtl2Case'][i,:] = rbk.EP2PRV(moduleOutput['qInrtl2Case'][i,1:])

    # # plot the output as prv
    # plt.plot(moduleOutput['qInrtl2Case'][:,0]*macros.NANO2SEC,moduleOutput['prvInrtl2Case'])
    # plt.show()


    # for i in range(0,4):
    #     # plt.figure()
    #     plt.plot(moduleOutput['qInrtl2Case'][:,0]*macros.NANO2SEC,moduleOutput['qInrtl2Case'][:,i+1])
    #     plt.xlabel('Time (s)')
    #     plt.ylabel('Quaternion')
    #     # plt.xlim((0,1000))
    # plt.show()


    if not 'accuracy' in vars():
        accuracy = 1e-6

    for moduleOutputName in fieldNames:
        if moduleOutputName is 'qInrtl2Case':
            if testCase == 'noise':
                for i in range(0,3):
                    if np.abs(np.mean(moduleOutput['prvInrtl2Case'][:,i])) > 0.01 \
                                    or np.abs(np.std(moduleOutput['prvInrtl2Case'][:,i]) - trueVector['qInrtl2Case'][i]) > 0.01 :
                        testFail = True
                        break

            elif testCase == 'walk bounds':
                for i in range(0,3):
                    print np.max(np.abs(np.asarray(moduleOutput['prvInrtl2Case'][:,i])))
                    if np.max(np.abs(np.asarray(moduleOutput['prvInrtl2Case'][:,i]))) > trueVector['qInrtl2Case'][i]:
                        testFail = True
                        break

            else:
                for i in range(0,len(trueVector['qInrtl2Case'])):
                    if not unitTestSupport.isArrayEqual(moduleOutput['qInrtl2Case'][i], trueVector['qInrtl2Case'][i], 3, accuracy):
                        testFail = True
                        break

            if testFail: # break outer loop
                break

        #elif moduleOutputName is 'timeTag':
        #    # check timeTag
        #    for i in range(0,len(trueVector['timeTag'])):
        #        if not unitTestSupport.isDoubleEqual(moduleOutput['timeTag'][i], trueVector['timeTag'][i], accuracy):
        #            testFail = True
        #            print "Ugh."
        #            print moduleOutput['timeTag'][i]
        #            print trueVector['timeTag'][i]
        #            break



    if testFail:
        testFailCount += 1
        testMessages.append("FAILED: " + StarTracker.ModelTag + " Module failed " +
                            moduleOutputName + " unit test")

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
    test_unitSimStarTracker(
        False, # show_plots
        False, # useFlag
        'walk bounds' # testCase
    )
