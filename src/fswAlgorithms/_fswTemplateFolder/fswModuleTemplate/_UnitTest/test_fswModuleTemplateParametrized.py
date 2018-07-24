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
#   Module Name:        fswModuleTemplateParametrized
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
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
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import fswModuleTemplate                # import the module that is to be tested
from Basilisk.utilities import macros

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("param1, param2", [
     (1, 1)
    ,(1, 3)
    ,(2, 2)
])

# update "module" in this function name to reflect the module name
def test_module(show_plots, param1, param2):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = fswModuleTestFunction(show_plots, param1, param2)
    assert testResults < 1, testMessage


def fswModuleTestFunction(show_plots, param1, param2):
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
    moduleWrap.ModelTag = "fswModuleTemplate"                                        # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.dataInMsgName = "sampleInput"          # update with current values
    moduleConfig.dataOutMsgName = "sampleOutput"        # update with current values
    moduleConfig.dummy = 1                              # update module parameter with required values
    moduleConfig.dumVector = [1., 2., 3.]

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = fswModuleTemplate.FswModuleTemplateFswMsg()  # Create a structure for the input message
    inputMessageData.outputVector = [param1, param2, 0.7]       # Set up a list as a 3-vector
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

    # set the filtered output truth states
    trueVector=[];
    if param1==1:
        if param2==1:
            trueVector = [
                       [2.0, 1.0, 0.7],
                       [3.0, 1.0, 0.7],
                       [4.0, 1.0, 0.7],
                       [2.0, 1.0, 0.7],
                       [3.0, 1.0, 0.7]
                       ]
        else:
            if param2==3:
                trueVector = [
                       [2.0, 3.0, 0.7],
                       [3.0, 3.0, 0.7],
                       [4.0, 3.0, 0.7],
                       [2.0, 3.0, 0.7],
                       [3.0, 3.0, 0.7]
                       ]
            else:
                testFailCount+=1
                testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed with unsupported input parameters")
    else:
        if param1==2:
            trueVector = [
                       [3.0, 2.0, 0.7],
                       [4.0, 2.0, 0.7],
                       [5.0, 2.0, 0.7],
                       [3.0, 2.0, 0.7],
                       [4.0, 2.0, 0.7]
                       ]
        else:
            testFailCount+=1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed with unsupported input parameters")

    # compare the module results to the truth values
    accuracy = 1e-12
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    dummyTrue = [1.0, 2.0, 3.0, 1.0, 2.0]

    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output Vector",
                                                               testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareDoubleArray(dummyTrue, variableState,
                                                               accuracy, "dummy parameter",
                                                               testFailCount, testMessages)

    # Note that we can continue to step the simulation however we feel like.
    # Just because we stop and query data does not mean everything has to stop for good
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.6))    # run an additional 0.6 seconds
    unitTestSim.ExecuteSimulation()

    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    # plot a sample variable.
    plt.close("all")    # close all prior figures so we start with a clean slate
    plt.figure(1)
    plt.plot(variableState[:, 0]*macros.NANO2SEC, variableState[:, 1],
             label='Case param1 = ' + str(param1) + ' and param2 = ' + str(param2))
    plt.legend(loc='upper left')
    plt.xlabel('Time [s]')
    plt.ylabel('Variable Description [unit]')
    if show_plots:
        plt.show()

    #   print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag

    # export a plot to be included in the documentation
    unitTestSupport.writeFigureLaTeX(
        "testPlot"+str(param1)+str(param2),
        "Illustration of Sample Plot",
        plt,
        "width=0.5\\textwidth",
        path)

    # write TeX Tables for documentation
    resultTable = moduleOutput
    resultTable[:,0] = macros.NANO2SEC*resultTable[:,0]
    diff = np.delete(moduleOutput,0,1) - trueVector
    resultTable = np.insert(resultTable,range(2,2+len(diff.transpose())), diff, axis=1)

    tableName = "test" + str(param1) + str(param2)      # make this a unique name
    tableHeaders = ["time [s]", "Output 1", "Error", "Output 2", "Error", "Output 3 $\\bm r$", "Error"]
    caption = 'Sample output table for param1 = ' + str(param1) + ' and param2 = ' + str(param2) + '.'
    unitTestSupport.writeTableLaTeX(
        tableName,
        tableHeaders,
        caption,
        resultTable,
        path)

    #   print out success message if no error were found
    snippentName = "passFail" + str(param1) + str(param2)
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
    test_module(              # update "module" in function name
                 False,
                 1,           # param1 value
                 1            # param2 value
               )
