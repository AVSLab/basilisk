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
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.moduleTemplates import cppModuleTemplate                # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.architecture import bskLogging


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("param1, param2", [
     (1, 1)
    ,(1, 3)
    ,(2, 2)
])

# update "module" in this function name to reflect the module name
def test_module(show_plots, param1, param2, accuracy):
    r"""
    **Validation Test Description**

    Compose a general description of what is being tested in this unit test script.  Add enough information so the
    reader understands the purpose and limitations of the test.  As this test script is not parameterized, only one
    version of this script will run.  Note that the ``pytest`` HTML report will list each parameterized test case
    individually.  This way it is clear what set of parameters passed.  But, this also means that this doc-string
    content will be copied into each report so each test description is individually complete.  If there is a
    discussion you want to include that is specific to the a parameterized test case, then include this at the
    end of the file with a conditional print() statement that only executes for that particular parameterized test.

    **Test Parameters**

    As this is a parameterized unit test, note that the test case parameters values are shown automatically in the
    pytest HTML report.  This sample script has the parameters param1 and param 2.  Provide a description of what
    each parameter controls.  This is a convenient location to include the accuracy variable used in the
    validation test.

    Args:
        param1 (int): Dummy test parameter for this parameterized unit test
        param2 (int): Dummy test parameter for this parameterized unit test
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    Here discuss what parameters are being checked.  For example, in this file we are checking the values of the
    variables

    - ``dummy``
    - ``dataVector[3]``

    **Figure Discussion**

    If the test script produces figures you might include a brief discussion on what the simulation results show.
    Discuss why these results validate the operation of the BSK module.

    **General Documentation Comments**

    If the script generates figures, these figures will be automatically pulled from ``matplotlib`` and included below.
    Make sure that the figures have appropriate axes labels and a figure title if needed.  The figures content
    should be understood by just looking at the figure.

    At the end of the script where a print statement says that the script passes.

    Don't use any of the AutoTeX methods we used to use as the goal is to have all the validation reporting
    contained within this HTML ``pytest`` report.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = cppModuleTestFunction(show_plots, param1, param2, accuracy)
    assert testResults < 1, testMessage


def cppModuleTestFunction(show_plots, param1, param2, accuracy):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    module = cppModuleTemplate.CppModuleTemplate()   # update with current values
    module.ModelTag = "cppModuleTemplate"            # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    module.dummy = 1                              # update module parameter with required values
    module.dumVector = [1., 2., 3.]

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = messaging.CModuleTemplateMsgPayload() # Create a structure for the input message
    inputMessageData.dataVector = [param1, param2, 0.7]       # Set up a list as a 3-vector
    inputMsg = messaging.CModuleTemplateMsg().write(inputMessageData)
    module.dataInMsg.subscribeTo(inputMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.dataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    variableName = "dummy"                              # name the module variable to be logged
    moduleLog = module.logger(variableName)
    unitTestSim.AddModelToTask(unitTaskName, moduleLog)

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
    module.Reset(1)     # this module reset function needs a time input (in NanoSeconds)

    # run the module again for an additional 1.0 seconds
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()


    # This pulls the BSK module internal varialbe log from the simulation run.
    # Note, this should only be done for debugging as it is a slow process
    variableState = unitTestSupport.addTimeColumn(moduleLog.times(), getattr(moduleLog, variableName))

    # set the filtered output truth states
    trueVector = []
    if param1 == 1:
        if param2 == 1:
            trueVector = [
                       [2.0, 1.0, 0.7],
                       [3.0, 1.0, 0.7],
                       [4.0, 1.0, 0.7],
                       [2.0, 1.0, 0.7],
                       [3.0, 1.0, 0.7]
                       ]
        else:
            if param2 == 3:
                trueVector = [
                       [2.0, 3.0, 0.7],
                       [3.0, 3.0, 0.7],
                       [4.0, 3.0, 0.7],
                       [2.0, 3.0, 0.7],
                       [3.0, 3.0, 0.7]
                       ]
            else:
                testFailCount += 1
                testMessages.append("FAILED: " + module.ModelTag
                                    + " Module failed with unsupported input parameters")
    else:
        if param1 == 2:
            trueVector = [
                       [3.0, 2.0, 0.7],
                       [4.0, 2.0, 0.7],
                       [5.0, 2.0, 0.7],
                       [3.0, 2.0, 0.7],
                       [4.0, 2.0, 0.7]
                       ]
        else:
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed with unsupported input parameters")

    # compare the module results to the truth values
    dummyTrue = [1.0, 2.0, 3.0, 1.0, 2.0]

    testFailCount, testMessages = unitTestSupport.compareArray(trueVector, dataLog.dataVector,
                                                               accuracy, "Output Vector",
                                                               testFailCount, testMessages)
    variableState = np.transpose(variableState)[1]
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
    plt.plot(dataLog.times()*macros.NANO2SEC, variableState,
             label='Case param1 = ' + str(param1) + ' and param2 = ' + str(param2))
    plt.legend(loc='upper left')
    plt.xlabel('Time [s]')
    plt.ylabel('Variable Description [unit]')
    plt.suptitle('Title of Sample Plot')

    plt.figure(2)
    for idx in range(3):
        plt.plot(dataLog.times() * macros.NANO2MIN, dataLog.dataVector[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$s_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Msg Output Vector States')

    if show_plots:
        plt.show()

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)

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
                 1,           # param2 value
                 1e-12        # accuracy
               )
