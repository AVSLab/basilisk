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
#   Module Name:        mrpRotation
#   Author:             Hanspeter Schaub
#   Creation Date:      May 20, 2018
#

import pytest
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

import numpy as np


# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.fswAlgorithms import mrpRotation                    # import the module that is to be tested
from Basilisk.utilities import macros as mc
from Basilisk.fswAlgorithms import fswMessages


sys.path.append(path + '/Support')
import truth_mrpRotation as truth


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)

@pytest.mark.parametrize("cmdStateFlag, stateOutputFlag, testReset", [
      ("False", "False", "False")
     ,("True", "False", "False")
    , ("False", "True", "False")
    , ("False", "False", "True")
    , ("True", "False", "True")
])


# provide a unique test method name, starting with test_
def test_mrpRotation(show_plots, cmdStateFlag, stateOutputFlag, testReset):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, cmdStateFlag, stateOutputFlag, testReset)
    assert testResults < 1, testMessage


def run(show_plots, cmdStateFlag, stateOutputFlag, testReset):
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

    # Test times
    updateTime = 0.5     # update process rate update time
    totalTestSimTime = 1.5

    # Create test thread
    testProcessRate = mc.sec2nano(updateTime)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = mrpRotation.mrpRotationConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "mrpRotation"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.attRefInMsgName = "inputRefName"
    moduleConfig.attRefOutMsgName = "outputName"
    sigma_RR0 = np.array([0.3, .5, 0.0])
    moduleConfig.mrpSet = sigma_RR0
    omega_RR0_R = np.array([0.1, 0.0, 0.0]) * mc.D2R
    moduleConfig.omega_RR0_R = omega_RR0_R
    unitTestSupport.writeTeXSnippet("sigma_RR0", str(sigma_RR0), path)
    unitTestSupport.writeTeXSnippet("omega_RR0_R", str(omega_RR0_R*mc.R2D) + "deg/sec", path)


    if cmdStateFlag:
        moduleConfig.desiredAttInMsgName = "desiredAttName"
        desiredAtt = fswMessages.AttStateFswMsg()
        sigma_RR0 = np.array([0.1, 0.0, -0.2])
        desiredAtt.state = sigma_RR0
        omega_RR0_R = np.array([0.1, 1.0, 0.5]) * mc.D2R
        desiredAtt.rate = omega_RR0_R
        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                   unitProcessName,
                                   moduleConfig.desiredAttInMsgName,
                                   desiredAtt)
        unitTestSupport.writeTeXSnippet("sigma_RR0Cmd", str(sigma_RR0), path)
        unitTestSupport.writeTeXSnippet("omega_RR0_RCmd", str(omega_RR0_R * mc.R2D) + "deg/sec", path)

    if stateOutputFlag:
        moduleConfig.attitudeOutMsgName = "optAttOut"


    # Create input message and size it because the regular creator of that message
    # is not part of the test.

    #
    # Reference Frame Message
    #
    RefStateInData = fswMessages.AttRefFswMsg()  # Create a structure for the input message
    sigma_R0N = np.array([0.1, 0.2, 0.3])
    RefStateInData.sigma_RN = sigma_R0N
    omega_R0N_N = np.array([0.1, 0.0, 0.0])
    RefStateInData.omega_RN_N = omega_R0N_N
    domega_R0N_N = np.array([0.0, 0.0, 0.0])
    RefStateInData.domega_RN_N = domega_R0N_N
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.attRefInMsgName,
                               RefStateInData)


    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.attRefOutMsgName, testProcessRate)
    if stateOutputFlag:
        unitTestSim.TotalSim.logThisMessage(moduleConfig.attitudeOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(mc.sec2nano(totalTestSimTime))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    if testReset:
        moduleWrap.Reset(1)
        unitTestSim.ConfigureStopTime(mc.sec2nano(totalTestSimTime+1.0))        # seconds to stop simulation
        unitTestSim.ExecuteSimulation()


    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    accuracy = 1e-12
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)
    trueSigma, trueOmega, truedOmega, trueOptSigma, trueOptOmega \
        = truth.results(sigma_RR0,omega_RR0_R,RefStateInData,updateTime, cmdStateFlag, testReset)

    #
    # check sigma_RN
    #
    moduleOutputName = "sigma_RN"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attRefOutMsgName + '.' + moduleOutputName,
                                                  range(3))
    testFailCount, testMessages = unitTestSupport.compareArray(trueSigma, moduleOutput,
                                                               accuracy, "sigma_RN Set",
                                                               testFailCount, testMessages)

    #
    # check omega_RN_N
    #
    moduleOutputName = "omega_RN_N"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attRefOutMsgName + '.' + moduleOutputName,
                                                  range(3))
    testFailCount, testMessages = unitTestSupport.compareArray(trueOmega, moduleOutput,
                                                               accuracy, "omega_RN_N Vector",
                                                               testFailCount, testMessages)

    #
    # check domega_RN_N
    #
    moduleOutputName = "domega_RN_N"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attRefOutMsgName + '.' + moduleOutputName,
                                                  range(3))
    testFailCount, testMessages = unitTestSupport.compareArray(truedOmega, moduleOutput,
                                                               accuracy, "domega_RN_N Vector",
                                                               testFailCount, testMessages)

    if stateOutputFlag:
        #
        # check sigma_RR0
        #
        moduleOutputName = "state"
        moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attitudeOutMsgName + '.' + moduleOutputName,
                                                      range(3))
        testFailCount, testMessages = unitTestSupport.compareArray(trueOptSigma, moduleOutput,
                                                                   accuracy, "sigma_RR0 Set",
                                                                   testFailCount, testMessages)

        #
        # check omega_RR0_R
        #
        moduleOutputName = "rate"
        moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.attitudeOutMsgName + '.' + moduleOutputName,
                                                      range(3))
        testFailCount, testMessages = unitTestSupport.compareArray(trueOptOmega, moduleOutput,
                                                                   accuracy, "omega_RR0_R Set",
                                                                   testFailCount, testMessages)

    snippentName = "passFail" + str(cmdStateFlag) + str(stateOutputFlag) + str(testReset)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print "PASSED: " + moduleWrap.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print "Failed: " + moduleWrap.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
#    if show_plots:
#        # plot a sample variable.
#        plt.figure(1)
#        plt.plot(variableState[:,0]*macros.NANO2SEC, variableState[:,1], label='Sample Variable')
#        plt.legend(loc='upper left')
#        plt.xlabel('Time [s]')
#        plt.ylabel('Variable Description [unit]')
#        plt.show()

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        False           # show plots
        , False         # cmdStateFlag
        , False         # stateOutputFlag
        , False         # testReset
    )
