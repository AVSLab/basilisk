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
#   Module Name:        spacecraftPointing
#   Author:             Simon van Overeem
#   Creation Date:      January 7, 2018
#

# The test that is performed in this file checks whether the spacecraftPointing module computes the correct output.
# The inputs of the test are five data points of the chief spacecraft and the deputy spacecraft. From these datapoints
# the orientation of the reference frame with respect to the inertial frame (sigma_R1N) is determined. Furthermore,
# the angular velocity (omega_RN_N) and the angular acceleration (domega_RN_N) are calculated. The outcomes are compared
# to the expected outcome of the module.

import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation.alg_contain import alg_contain
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms.spacecraftPointing import spacecraftPointing           # import the module that is to be tested
from Basilisk.fswAlgorithms.cheby_pos_ephem import cheby_pos_ephem
from Basilisk.utilities import macros
import numpy as np
from Basilisk.utilities import astroFunctions as af

@pytest.mark.parametrize("case", [
     (1)        # Regular alignment vector
    ,(2)        # Alignment vector aligns with the z-axis of the body frame
])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_spacecraftPointing(show_plots, case):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = spacecraftPointingTestFunction(show_plots, case)
    assert testResults < 1, testMessage


def spacecraftPointingTestFunction(show_plots, case):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = spacecraftPointing.spacecraftPointingConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "spacecraftPointing"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.chiefPositionInMsgName = "chiefInMsg"
    moduleConfig.deputyPositionInMsgName = "deputyInMsg"
    moduleConfig.attReferenceOutMsgName = "attRefOutMsg"
    moduleConfig.alignmentVector_B = [1.0, 0.0, 0.0]
    if (case == 2):
        moduleConfig.alignmentVector_B = [0.0, 0.0, 1.0]

    r_BN_N = [[np.cos(0.0), np.sin(0.0), 0.0],
              [np.cos(0.001), np.sin(0.001), 0.0],
              [np.cos(0.002), np.sin(0.002), 0.0],
              [np.cos(0.003), np.sin(0.003), 0.0],
              [np.cos(0.004), np.sin(0.004), 0.0]]

    r_BN_N2 = [[0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0]]

    # Create input message and size it because the regular creator of that message
    # is not part of the test.

    #
    #   Chief Input Message
    #
    chiefInputData = spacecraftPointing.NavTransIntMsg()  # Create a structure for the input message
    chiefInputMsgSize = chiefInputData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.chiefPositionInMsgName,
                                          chiefInputMsgSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)

    #
    #   Deputy Input Message
    #
    deputyInputData = spacecraftPointing.NavTransIntMsg()  # Create a structure for the input message
    deputyInputMsgSize = deputyInputData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.deputyPositionInMsgName,
                                          deputyInputMsgSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)

    chiefInputData.r_BN_N = r_BN_N[0]
    deputyInputData.r_BN_N = r_BN_N2[0]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.chiefPositionInMsgName,
                                          chiefInputMsgSize,
                                          0,
                                          chiefInputData)

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.deputyPositionInMsgName,
                                          deputyInputMsgSize,
                                          0,
                                          deputyInputData)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.attReferenceOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()



    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.1))        # seconds to stop simulation

    # Begin the simulation time run set above
    # Because it is decided to give the module a set of coordinates for each timestep, a new message has
    # to be send for each timestep.
    unitTestSim.ExecuteSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(0.2))

    chiefInputData.r_BN_N = r_BN_N[1]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.chiefPositionInMsgName,
                                          chiefInputMsgSize,
                                          0,
                                          chiefInputData)

    deputyInputData.r_BN_N = r_BN_N2[1]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.deputyPositionInMsgName,
                                          deputyInputMsgSize,
                                          0,
                                          deputyInputData)

    unitTestSim.ExecuteSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(0.3))

    chiefInputData.r_BN_N = r_BN_N[2]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.chiefPositionInMsgName,
                                          chiefInputMsgSize,
                                          0,
                                          chiefInputData)

    deputyInputData.r_BN_N = r_BN_N2[2]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.deputyPositionInMsgName,
                                          deputyInputMsgSize,
                                          0,
                                          deputyInputData)

    unitTestSim.ExecuteSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(0.4))

    chiefInputData.r_BN_N = r_BN_N[3]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.chiefPositionInMsgName,
                                          chiefInputMsgSize,
                                          0,
                                          chiefInputData)

    deputyInputData.r_BN_N = r_BN_N2[3]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.deputyPositionInMsgName,
                                          deputyInputMsgSize,
                                          0,
                                          deputyInputData)

    unitTestSim.ExecuteSimulation()

    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))

    chiefInputData.r_BN_N = r_BN_N[4]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.chiefPositionInMsgName,
                                          chiefInputMsgSize,
                                          0,
                                          chiefInputData)

    deputyInputData.r_BN_N = r_BN_N2[4]

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.deputyPositionInMsgName,
                                          deputyInputMsgSize,
                                          0,
                                          deputyInputData)

    unitTestSim.ExecuteSimulation()

    if (case == 1):
        # This pulls the actual data log from the simulation run.
        # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
        #
        # check sigma_RN
        #
        moduleOutputName = "sigma_RN"
        moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attReferenceOutMsgName + '.' + moduleOutputName,
                                                      list(range(3)))
        # set the filtered output truth states
        trueVector = [
                   [0.,              0.,              0.0],
                   [0.,              0.,              0.0],
                   [0.,              0.,              0.0002500000052],
                   [0.,              0.,              0.0005000000417],
                   [0.,              0.,              0.0007500001406],
                   [0.,              0.,              0.001000000333]
                   ]
        # compare the module results to the truth values
        accuracy = 1e-12
        unitTestSupport.writeTeXSnippet("toleranceValue1", str(accuracy), path)
        for i in range(0,len(trueVector)):
            # check a vector values
            if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                    moduleOutputName + " unit test at t=" +
                                    str(moduleOutput[i,0]*macros.NANO2SEC) +
                                    "sec\n")

        #
        # check omega_RN_N
        #
        moduleOutputName = "omega_RN_N"
        moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attReferenceOutMsgName + '.' + moduleOutputName,
                                                      list(range(3)))

        # set the filtered output truth states
        trueVector = [
                   [0.,              0.,              0.0],
                   [0.,              0.,              0.0],
                   [0.,              0.,              0.01],
                   [0.,              0.,              0.01],
                   [0.,              0.,              0.01],
                   [0.,              0.,              0.01]
                   ]

        # compare the module results to the truth values
        # The first three values of the simulation have to be ignored for omega_RN_N. For this reason, comparing from index 3.
        accuracy = 1e-9
        unitTestSupport.writeTeXSnippet("toleranceValue2", str(accuracy), path)
        for i in range(0,len(trueVector)):
            # check a vector values
            if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                    moduleOutputName + " unit test at t=" +
                                    str(moduleOutput[i,0]*macros.NANO2SEC) +
                                    "sec\n")

        #
        # check domega_RN_N
        #
        moduleOutputName = "domega_RN_N"
        moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attReferenceOutMsgName + '.' + moduleOutputName,
                                                      list(range(3)))
        # set the filtered output truth states
        trueVector = [
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0]
                      ]
        # compare the module results to the truth values
        # The first three values of the simulation have to be ignored for domega_RN_N. For this reason, comparing from index 3.
        accuracy = 1e-12
        unitTestSupport.writeTeXSnippet("toleranceValue3", str(accuracy), path)
        for i in range(0,len(trueVector)):
            # check a vector values
            if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                    moduleOutputName + " unit test at t=" +
                                    str(moduleOutput[i,0]*macros.NANO2SEC) +
                                    "sec\n")
    elif (case == 2):
        trueVector = [-1.0/3.0, 1.0/3.0, -1.0/3.0]
        # compare the module results to the truth values
        accuracy = 1e-12
        unitTestSupport.writeTeXSnippet("toleranceValue4", str(accuracy), path)
        # check a vector values
        if not unitTestSupport.isVectorEqual(np.array(moduleConfig.sigma_BA), np.array(trueVector), accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed, sigma_BA is calculated incorrectly\n")

    #   print out success message if no error were found
    snippentName = "passFail" + str(case)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + moduleWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("FAILED: " + moduleWrap.ModelTag)
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
    test_spacecraftPointing(False)
