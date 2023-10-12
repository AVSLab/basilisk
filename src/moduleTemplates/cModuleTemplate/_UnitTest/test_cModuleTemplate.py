#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        cModuleTemplate
#   Author:             (First Name) (Last Name)
#   Creation Date:      Month Day, Year
#

# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging  # import the message definitions
from Basilisk.moduleTemplates import cModuleTemplate  # import the module that is to be tested
# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_module(show_plots):     # update "module" in this function name to reflect the module name
    r"""
    **Validation Test Description**

    Compose a general description of what is being tested in this unit test script.  Add enough information so
    the reader understands the purpose and limitations of the test.  As this test script is not parameterized, only
    one version of this script will run.

    **Description of Variables Being Tested**

    Here discuss what parameters are being checked.  For example, in this file we are checking the values of the
    variables

    - ``dummy``
    - ``dataVector[3]``

    **General Documentation Comments**
    
    If the script generates figures, these figures will be automatically pulled from ``matplotlib`` and included below.
    Make sure that the figures have appropriate axes labels and a figure title if needed.  The figures content
    should be understood by just looking at the figure.

    At the end of the script where a print statement says that the script passes, also add a print statement
    saying what accuracy tolerance(s) were used.

    Don't use any of the AutoTeX methods we used to use as the goal is to have all the validation reporting
    contained within this HTML ``pytest`` report.
    """
    # each test method requires a single assert method to be called
    # pass on the testPlotFixture so that the main test function may set the DataStore attributes
    [testResults, testMessage] = fswModuleTestFunction(show_plots)
    assert testResults < 1, testMessage


def fswModuleTestFunction(show_plots):
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
    module = cModuleTemplate.cModuleTemplate()
    module.ModelTag = "cModuleTemplate"           # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Initialize the test module configuration data
    module.dummy = 1                              # update module parameter with required values
    module.dumVector = [1., 2., 3.]

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = messaging.CModuleTemplateMsgPayload()  # Create a structure for the input message
    inputMessageData.dataVector = [1.0, -0.5, 0.7]             # Set up a list as a 3-vector
    inputMsg = messaging.CModuleTemplateMsg().write(inputMessageData)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.dataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    variableName = "dummy"                              # name the module variable to be logged
    moduleLog = module.logger(variableName)
    unitTestSim.AddModelToTask(unitTaskName, moduleLog)

    # connect the message interfaces
    module.dataInMsg.subscribeTo(inputMsg)


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


    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    variableState = unitTestSupport.addTimeColumn(moduleLog.times(), getattr(moduleLog, variableName))

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
    variableStateNoTime = np.transpose(variableState)[1]
    for i in range(0, len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(dataLog.dataVector[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed dataVector" +
                                " unit test at t=" +
                                str(dataLog.times()[i]*macros.NANO2SEC) +
                                "sec\n")

        # check a scalar double value
        if not unitTestSupport.isDoubleEqual(variableStateNoTime[i], dummyTrue[i], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed " +
                                variableName + " unit test at t=" +
                                str(variableState[i, 0]*macros.NANO2SEC) +
                                "sec\n")

    # Note that we can continue to step the simulation however we feel like.
    # Just because we stop and query data does not mean everything has to stop for good
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.6))    # run an additional 0.6 seconds
    unitTestSim.ExecuteSimulation()

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
        print("This test uses an accuracy value of " + str(accuracy))
    else:
        print("FAILED " + module.ModelTag)
        print(testMessages)

    plt.close("all")  # close all prior figures so we start with a clean slate
    plt.figure(1)
    plt.plot(variableState[:, 0] * macros.NANO2SEC, variableState[:, 1])
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

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    fswModuleTestFunction(
               True        # show_plots
    )
