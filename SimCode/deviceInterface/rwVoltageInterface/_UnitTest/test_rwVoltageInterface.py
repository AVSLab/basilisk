''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#   Module Name:        rwVoltageInterface
#   Author:             Hanspeter Schaub
#   Creation Date:      January 29, 2017
#

import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
# if this script is run from a custom folder outside of the Basilisk folder, then uncomment the
# following line and specify the absolute bath to the Basilisk folder
#bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')

# Import all of the modules that we are going to be called in this simulation
import SimulationBaseClass
import alg_contain
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import rwVoltageInterface               # import the module that is to be tested
import macros

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("voltage", [
     (5.0)
     ,(-7.5)
     ,(0.0)
])

# update "module" in this function name to reflect the module name
def test_module(show_plots, voltage):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, voltage)
    assert testResults < 1, testMessage


def run(show_plots, voltage):
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
    testModule = rwVoltageInterface.RWVoltageInterface()
    testModule.ModelTag = "rwVoltageInterface"

    # set module parameters(s)
    testModule.voltage2TorqueGain = 1.32        # [Nm/V] conversion gain

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, testModule)


    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    voltageData = rwVoltageInterface.RWArrayVoltageIntMsg()
    voltageData.voltage = [voltage, voltage+1.0, voltage+1.5]
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               testModule.rwVoltageInMsgName,
                               voltageData)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(testModule.rwMotorTorqueOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()


    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "motorTorque"
    moduleOutput = unitTestSim.pullMessageLogData(testModule.rwMotorTorqueOutMsgName + '.' + moduleOutputName,
                                                  range(3))

    # set truth states
    voltageTrue = np.array([1.0, 1.0, 1.0])*voltage + np.array([0.0, 1.0, 1.5])

    trueVector = [
         voltageTrue*testModule.voltage2TorqueGain
        , voltageTrue * testModule.voltage2TorqueGain
        , voltageTrue * testModule.voltage2TorqueGain
    ]

    # compare the module results to the truth values
    accuracy = 1e-12
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output Vector",
                                                               testFailCount, testMessages)

    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    # plot a sample variable.
    plt.close("all")    # close all prior figures so we start with a clean slate
    plt.figure(1)
    # plt.plot(variableState[:, 0]*macros.NANO2SEC, variableState[:, 1],
    #          label='Case param1 = ' + str(param1) + ' and param2 = ' + str(param2))
    # plt.legend(loc='upper left')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Variable Description [unit]')
    # if show_plots:
    #     plt.show()


    resultTable = moduleOutput
    resultTable[:, 0] = macros.NANO2SEC * resultTable[:, 0]
    diff = np.delete(moduleOutput, 0, 1) - trueVector
    resultTable = np.insert(resultTable, range(2, 2 + len(diff.transpose())), diff, axis=1)

    tableName = "baseVoltage" + str(voltage)
    tableHeaders = ["time [s]", "$u_{s,1}$ (Nm)", "Error", "$u_{s,2}$ (Nm)", "Error", "$u_{u,3}$ (Nm)", "Error"]
    caption = 'RW motoor torque output for Base Voltaget = ' + str(voltage) + 'V.'
    unitTestSupport.writeTableLaTeX(
        tableName,
        tableHeaders,
        caption,
        resultTable,
        path)


    #   print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + testModule.ModelTag

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
                 5.0,           # param1 value
               )
