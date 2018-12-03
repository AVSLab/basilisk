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
#   Module Name:        sunlineEphem()
#   Author:             John Martin
#   Creation Date:      November 30, 2018
#

import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import sunlineEphem  # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.simulation import simFswInterfaceMessages
from Basilisk.simulation import simMessages
from Basilisk.utilities import RigidBodyKinematics
import numpy as np


class DataStore:
    """Container for developer defined variables to be used in test data post-processing and plotting.

        Attributes:
            variableState (list): an example variable to hold test result data.
    """

    def __init__(self):
        self.variableState = None  # replace/add with appropriate variables for test result data storing

    def plotData(self):
        """All test plotting to be performed here.

        """
        plt.figure(1) # plot a sample variable.
        plt.plot(self.variableState[:, 0]*macros.NANO2SEC, self.variableState[:, 1], label='Sample Variable')
        plt.legend(loc='upper left')
        plt.xlabel('Time [s]')
        plt.ylabel('Variable Description [unit]')
        plt.show()


@pytest.fixture(scope="module")
def plotFixture(show_plots):
    dataStore = DataStore()
    yield dataStore
    if show_plots:
        dataStore.plotData()


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_module(show_plots):     # update "module" in this function name to reflect the module name
    # each test method requires a single assert method to be called
    # pass on the testPlotFixture so that the main test function may set the DataStore attributes
    [testResults, testMessage] = sunlineEphemTestFunction(show_plots)
    assert testResults < 1, testMessage


def sunlineEphemTestFunction(show_plots):
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
    sunlineEphemConfig = sunlineEphem.sunlineEphemConfig() # update with current values
    sunlineEphemWrap = unitTestSim.setModelDataWrap(sunlineEphemConfig)
    sunlineEphemWrap.ModelTag = "sunlineEphem"           # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, sunlineEphemWrap, sunlineEphemConfig)

    # Initialize the test module configuration data
    sunlineEphemConfig.scPositionInMsgName = "simple_trans_nav_output"
    sunlineEphemConfig.scAttitudeInMsgName = "simple_att_nav_output"
    sunlineEphemConfig.sunPositionInMsgName = "sun_position_output"
    sunlineEphemConfig.navStateOutMsgName = "sunline_ephem_output"        # update with current values

    # Create input message and size it because the regular creator of that message
    # is not part of the test.

    vehAttData = sunlineEphem.NavAttIntMsg()
    vehPosData = sunlineEphem.NavTransIntMsg()
    sunData = sunlineEphem.EphemerisIntMsg()

    q = RigidBodyKinematics.addEuler123([np.pi/2, 0, 0],[0,0,0])
    sigma = RigidBodyKinematics.PRV2MRP(q)

    vehAttData.sigma_BN = sigma
    vehPosData.r_BN_N = [0.0, 0.0, 0.0]
    sunData.r_BdyZero_N = [0.0, 0.0, 0.0]


    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               sunlineEphemConfig.scAttitudeInMsgName,
                               vehAttData)

    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               sunlineEphemConfig.sunPositionInMsgName,
                               sunData)


    # Initial test is all of the principal body axes
    TestVectors = [[-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0],
                   [0.0, 0.0, -1.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0]]

    estVector = np.zeros((6,4))

    for i in range(len(TestVectors)):
        testVec = TestVectors[i]
        vehPosData.r_BN_N = testVec
        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                   unitProcessName,
                                   sunlineEphemConfig.scPositionInMsgName,
                                   vehPosData)

        unitTestSim.TotalSim.logThisMessage(sunlineEphemConfig.navStateOutMsgName, testProcessRate)

        # Need to call the self-init and cross-init methods
        unitTestSim.InitializeSimulation()
        unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
        unitTestSim.ExecuteSimulation()

        moduleOutputName = "vehSunPntBdy"
        moduleOutput = unitTestSim.pullMessageLogData(sunlineEphemConfig.navStateOutMsgName + '.' + moduleOutputName,
                                                      range(3))

        estVector[i] = moduleOutput[0,:]

        # reset the module to test this functionality
        sunlineEphemWrap.Reset(1)


    # set the filtered output truth states
    trueVector = [
               [1.0, 0.0, 0.0],
               [0.0, 0.0, -1.0],
               [0.0, 1.0, 0.0],
               [-1.0, 0.0, 0.0],
               [0.0, 0.0, 1.0],
                [0.0, -1.0, 0.0]
               ]

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(estVector[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + sunlineEphemWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0]*macros.NANO2SEC) +
                                "sec\n")


    #   print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED: " + sunlineEphemWrap.ModelTag

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(            # update "subModule" in function name
               False        # show_plots
    )
