
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
#   Unit Test Script
#   Module Name:        motorVoltageInterface
#   Author:             João Vaz Carneiro
#   Creation Date:      February 13, 2021
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import motorVoltageInterface           # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging


def addTimeColumn(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))

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
    r"""
    **Test Parameters**

    Three base voltages are tested where :math:`V_0\in(5.0,-7.5,0.0)`. The input voltages are then setup as

    .. math::

        {\bf V}=V_0 \begin{bmatrix}
        1\\
        1\\
        1
        \end{bmatrix} + \begin{bmatrix}
        0.0\\
        1.0\\
        1.5
        \end{bmatrix}

    Other inputs to the module are:

    .. code-block:: python
        :linenos:

        testModule.voltage2TorqueGain =[ 1.32, 0.99, 1.31] # [Nm/V] conversion gain
        testModule.scaleFactor =[ 1.01, 1.00, 1.02] #[unitless] scale factor
        testModule.bias =[0.01, 0.02, 0.04] # [Nm] bias
    """
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

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    testModule = motorVoltageInterface.MotorVoltageInterface()
    testModule.ModelTag = "motorVoltageInterface"

    # set module parameters(s)
    testModule.setGains(np.array([1.32, 0.99, 1.31]))      # [Nm/V] conversion gain
    testModule.setScaleFactors(np.array([1.01, 1.00, 1.02]))               # [ul] error scale factor
    testModule.setBiases(np.array([0.01, 0.02, 0.04]))                      # [Nm] Torque bias from converter output

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    voltageData = messaging.ArrayMotorVoltageMsgPayload()
    voltageData.voltage = [voltage, voltage+1.0, voltage+1.5]
    voltageMsg = messaging.ArrayMotorVoltageMsg().write(voltageData)
    testModule.motorVoltageInMsg.subscribeTo(voltageMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = testModule.motorTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

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
    moduleOutput = dataLog.motorTorque[:, :3]

    # set truth states
    voltageTrue = np.array([1.0, 1.0, 1.0])*voltage + np.array([0.0, 1.0, 1.5])
    trueVector = [
          voltageTrue[0] * testModule.voltage2TorqueGain[0][0]*testModule.scaleFactor[0][0] + testModule.bias[0][0],
          voltageTrue[1] * testModule.voltage2TorqueGain[1][0]*testModule.scaleFactor[1][0] + testModule.bias[1][0],
          voltageTrue[2] * testModule.voltage2TorqueGain[2][0]*testModule.scaleFactor[2][0] + testModule.bias[2][0]
    ]
    trueVector = np.array([trueVector, trueVector, trueVector])

    # compare the module results to the truth values
    accuracy = 1e-12
    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, moduleOutput,
                                                               accuracy, "Output Vector",
                                                               testFailCount, testMessages)
    moduleOutput = addTimeColumn(dataLog.times(), moduleOutput)
    resultTable = moduleOutput
    resultTable[:, 0] = macros.NANO2SEC * resultTable[:, 0]
    diff = np.delete(moduleOutput, 0, 1) - trueVector
    resultTable = np.insert(resultTable, list(range(2, 2 + len(diff.transpose()))), diff, axis=1)

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
    snippetName = "passFail" + '{:1.1f}'.format(voltage)
    if testFailCount == 0:
        colorText = "ForestGreen"
        print("PASSED: " + testModule.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = "Red"
        passedText = r'\textcolor{' + colorText + '}{' + "FAILED" + '}'
    unitTestSupport.writeTeXSnippet(snippetName, passedText, path)


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
