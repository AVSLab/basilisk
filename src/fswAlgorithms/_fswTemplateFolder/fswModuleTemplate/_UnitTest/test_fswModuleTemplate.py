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
#   Module Name:        fswModuleTemplate
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
#

import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import fswModuleTemplate                 # import the module that is to be tested
from Basilisk.utilities import macros



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
def test_module(plotFixture, show_plots):     # update "module" in this function name to reflect the module name
    # each test method requires a single assert method to be called
    # pass on the testPlotFixture so that the main test function may set the DataStore attributes
    [testResults, testMessage] = fswModuleTestFunction(plotFixture, show_plots)
    assert testResults < 1, testMessage


def fswModuleTestFunction(plotFixture, show_plots):
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
    moduleConfig = fswModuleTemplate.fswModuleTemplateConfig()                          # update with current values
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "fswModuleTemplate"           # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.dataInMsgName  = "sampleInput"         # update with current values
    moduleConfig.dataOutMsgName = "sampleOutput"        # update with current values
    moduleConfig.dummy = 1                              # update module parameter with required values
    moduleConfig.dumVector = [1., 2., 3.]

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = fswModuleTemplate.FswModuleTemplateFswMsg()     # Create a structure for the input message
    inputMessageData.outputVector = [1.0, -0.5, 0.7]       # Set up a list as a 3-vector
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.dataInMsgName,
                               inputMessageData)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.dataOutMsgName, testProcessRate)
    variableName = "dummy"                              # name the module variable to be logged
    unitTestSim.AddVariableForLogging(moduleWrap.ModelTag + "." + variableName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # reset the module to test this functionality
    moduleWrap.Reset(1)     # this module reset function needs a time input (in NanoSeconds)

    # run the module again for an additional 1.0 seconds
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()


    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "outputVector"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.dataOutMsgName + '.' + moduleOutputName,
                                                  range(3))
    variableState = unitTestSim.GetLogVariableData(moduleWrap.ModelTag + "." + variableName)

    # Set the results variable(s) to the fixture data storage variables so that it is accessible for plotting
    plotFixture.variableState = variableState

    # set the filtered output truth states
    trueVector = [
               [2.0, -0.5, 0.7],
               [3.0, -0.5, 0.7],
               [4.0, -0.5, 0.7],
               [2.0, -0.5, 0.7],
               [3.0, -0.5, 0.7]
               ]

    # compare the module results to the truth values
    accuracy = 1e-12
    dummyTrue = [1.0, 2.0, 3.0, 1.0, 2.0]
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[i,0]*macros.NANO2SEC) +
                                "sec\n")

        # check a scalar double value
        if not unitTestSupport.isDoubleEqual(variableState[i],dummyTrue[i],accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                                variableName + " unit test at t=" +
                                str(variableState[i,0]*macros.NANO2SEC) +
                                "sec\n")

    # Note that we can continue to step the simulation however we feel like.
    # Just because we stop and query data does not mean everything has to stop for good
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.6))    # run an additional 0.6 seconds
    unitTestSim.ExecuteSimulation()

    #   print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED: " + moduleWrap.ModelTag

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
