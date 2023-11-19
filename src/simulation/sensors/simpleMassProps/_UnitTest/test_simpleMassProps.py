#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
#   Unit Test Script
#   Module Name:        cppModuleTemplateParametrized
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
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
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.simulation import simpleMassProps
from Basilisk.utilities import macros
from Basilisk.architecture import messaging  # import the message definitions


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("accuracy", [1e-8])

# update "module" in this function name to reflect the module name
def test_module(show_plots, accuracy):
    r"""
    **Validation Test Description**

    This unit test compares the output of the type ``vehicleConfigMsg`` with the input of the type ``scMassPropsMsg``.
    For the test, a ``scMAssPropsMAsgPayload`` message is created and populated with certain mass, inertia and center of
    mass values. The simulation is run for one time step, and then the mass properties of the input message are changed
    again. The simulation is run again for one time step, and then the output message mass properties are compared to
    the values that were set at each simulation step.

    **Test Parameters**

    Only the ``accuracy`` variable is set as an input to the test function. ITs default value is 1e-8.

    Args:
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    The parameters being checked correspond to the variables inside the ``vehicleConfigMsgPayload`` type message:

    - ``massSc``: total mass of the spacecraft
    - ``ISCPntB_B[9]``: total inertia of the spacecraft with respect to the origin of the B frame in B frame components
    - ``CoM_B``: spacecraft's center of mass expressed in the B frame
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = simpleMassPropsTestFunction(show_plots, accuracy)
    assert testResults < 1, testMessage


def simpleMassPropsTestFunction(show_plots, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1.0)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    scMassPropsModule = simpleMassProps.SimpleMassProps()  # update with current values
    scMassPropsModule.ModelTag = "scMassPropsModule"  # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scMassPropsModule)

    # Create input message
    scMassPropsData = messaging.SCMassPropsMsgPayload()  # Create a structure for the input message
    scMassPropsData.massSC = 100
    scMassPropsData.ISC_PntB_B = np.array([[40, 0, 0],
                                  [0, 50, 0],
                                  [0, 0, 60]])
    scMassPropsData.c_B = np.array([0.0, 0.0, 0.0])
    scMassPropsMsg = messaging.SCMassPropsMsg().write(scMassPropsData)
    scMassPropsModule.scMassPropsInMsg.subscribeTo(scMassPropsMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    vehicleDataLog = scMassPropsModule.vehicleConfigOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, vehicleDataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Run for one time step
    unitTestSim.TotalSim.SingleStepProcesses()

    # Change the mass, inertia and center of mass properties
    scMassPropsData.massSC = 500
    scMassPropsData.ISC_PntB_B = [[200, 0, 0],
                                  [0, 300, 0],
                                  [0, 0, 400]]
    scMassPropsData.c_B = [1.0, -1.0, 0.0]
    scMassPropsMsg = messaging.SCMassPropsMsg().write(scMassPropsData)
    scMassPropsModule.scMassPropsInMsg.subscribeTo(scMassPropsMsg)

    # Run for another time step
    unitTestSim.TotalSim.SingleStepProcesses()

    # set the filtered output truth states
    trueMass = [100, 500]
    trueInertia = [[40, 0, 0, 0, 50, 0, 0, 0, 60],
                   [200, 0, 0, 0, 300, 0, 0, 0, 400]]
    trueCoM = [[0.0, 0.0, 0.0],
               [1.0, -1.0, 0.0]]

    for i in range(2):
        # check a vector values
        if not unitTestSupport.isDoubleEqual(vehicleDataLog.massSC[i], trueMass[i], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: simpleMassProps mass test " + str(i+1) + "\n")
        if not unitTestSupport.isArrayEqual(vehicleDataLog.ISCPntB_B[i], trueInertia[i], 9, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: simpleMassProps inertia test " + str(i+1) + "\n")
        if not unitTestSupport.isArrayEqual(vehicleDataLog.CoM_B[i], trueCoM[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: simpleMassProps center of mass test " + str(i+1) + "\n")

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + scMassPropsModule.ModelTag)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(  # update "module" in function name
        False,
        1e-8  # accuracy
    )
