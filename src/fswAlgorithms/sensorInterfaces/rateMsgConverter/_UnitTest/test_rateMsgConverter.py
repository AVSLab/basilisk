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
#   Module Name:        rateMsgConverter
#   Author:             Hanspeter Schaub
#   Creation Date:      June 30, 2018
#

import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)







# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.fswAlgorithms import rateMsgConverter
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import fswMessages

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.


# update "module" in this function name to reflect the module name
def test_module(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = rateMsgConvertFunction(show_plots)
    assert testResults < 1, testMessage


def rateMsgConvertFunction(show_plots):
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
    moduleConfig = rateMsgConverter.rateMsgConverterConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "rateMsgConverter"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.navRateOutMsgName = "sampleOutput"
    moduleConfig.imuRateInMsgName = "sampleInput"

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = fswMessages.IMUSensorBodyFswMsg()
    inputMessageData.AngVelBody = [-0.1, 0.2, -0.3]
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.imuRateInMsgName,
                               inputMessageData)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.navRateOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # compare the module results to the truth values
    accuracy = 1e-12
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "omega_BN_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.navRateOutMsgName + '.' + moduleOutputName,
                                                  range(3))

    # set the filtered output truth states
    trueVector = [
        [-0.1, 0.2, -0.3],
        [-0.1, 0.2, -0.3],
        [-0.1, 0.2, -0.3]
    ]
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output Vector",
                                                               testFailCount, testMessages)

    # write TeX Tables for documentation
    resultTable = moduleOutput
    resultTable[:,0] = macros.NANO2SEC*resultTable[:,0]
    diff = np.delete(moduleOutput,0,1) - trueVector
    resultTable = np.insert(resultTable,range(2,2+len(diff.transpose())), diff, axis=1)

    tableName = "testRateMsgConverter"       # make this a unique name
    tableHeaders = ["time [s]", "Output 1", "Error", "Output 2", "Error", "Output 3 $\\bm r$", "Error"]
    caption = 'Unit test output table.'
    unitTestSupport.writeTableLaTeX(
        tableName,
        tableHeaders,
        caption,
        resultTable,
        path)


    moduleOutputName = "omega_BN_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.navRateOutMsgName + '.' + moduleOutputName,
                                                  range(3))

    # set the filtered output truth states
    trueVector = [
        [-0.1, 0.2, -0.3],
        [-0.1, 0.2, -0.3],
        [-0.1, 0.2, -0.3]
    ]
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output Rate Vector",
                                                               testFailCount, testMessages)

    # write TeX Tables for documentation
    resultTable = moduleOutput
    resultTable[:,0] = macros.NANO2SEC*resultTable[:,0]
    diff = np.delete(moduleOutput,0,1) - trueVector
    resultTable = np.insert(resultTable,range(2,2+len(diff.transpose())), diff, axis=1)

    tableName = "testRate"       # make this a unique name
    tableHeaders = ["time [s]", "Output 1", "Error", "Output 2", "Error", "Output 3 $\\bm \\omega_{B/N}$", "Error"]
    caption = 'Unit test output table for rate values.'
    unitTestSupport.writeTableLaTeX(
        tableName,
        tableHeaders,
        caption,
        resultTable,
        path)


    moduleOutputName = "sigma_BN"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.navRateOutMsgName + '.' + moduleOutputName,
                                                  range(3))

    # set the filtered output truth states
    trueVector = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output MRP Vector",
                                                               testFailCount, testMessages)

    # write TeX Tables for documentation
    resultTable = moduleOutput
    resultTable[:,0] = macros.NANO2SEC*resultTable[:,0]
    diff = np.delete(moduleOutput,0,1) - trueVector
    resultTable = np.insert(resultTable,range(2,2+len(diff.transpose())), diff, axis=1)

    tableName = "testMRP"       # make this a unique name
    tableHeaders = ["time [s]", "Output 1", "Error", "Output 2", "Error", "Output 3 $\\bm \\sigma$", "Error"]
    caption = 'Unit test output table for MRPs.'
    unitTestSupport.writeTableLaTeX(
        tableName,
        tableHeaders,
        caption,
        resultTable,
        path)

    moduleOutputName = "vehSunPntBdy"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.navRateOutMsgName + '.' + moduleOutputName,
                                                  range(3))

    # set the filtered output truth states
    trueVector = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output sun heading Vector",
                                                               testFailCount, testMessages)

    # write TeX Tables for documentation
    resultTable = moduleOutput
    resultTable[:,0] = macros.NANO2SEC*resultTable[:,0]
    diff = np.delete(moduleOutput,0,1) - trueVector
    resultTable = np.insert(resultTable,range(2,2+len(diff.transpose())), diff, axis=1)

    tableName = "testSunHeading"       # make this a unique name
    tableHeaders = ["time [s]", "Output 1", "Error", "Output 2", "Error", "Output 3 $\\bm d$", "Error"]
    caption = 'Unit test output table for sun heading vector.'
    unitTestSupport.writeTableLaTeX(
        tableName,
        tableHeaders,
        caption,
        resultTable,
        path)





    #   print out success message if no error were found
    snippentName = "passFail"
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print "PASSED: " + moduleWrap.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print "Failed: " + moduleWrap.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(
                 False
               )
