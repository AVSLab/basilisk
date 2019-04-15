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
#   Module Name:        attTrackingError
#   Author:             Hanspeter Schaub
#   Creation Date:      January 15, 2016
#

import pytest
import sys, os, inspect
import numpy as np
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))







# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport              # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import attTrackingError                  # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_attTrackingError(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = subModuleTestFunction(show_plots)
    assert testResults < 1, testMessage


def subModuleTestFunction(show_plots):
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
    moduleConfig = attTrackingError.attTrackingErrorConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        attTrackingError.Update_attTrackingError,
                                        attTrackingError.SelfInit_attTrackingError,
                                        attTrackingError.CrossInit_attTrackingError)
    moduleWrap.ModelTag = "attTrackingError"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.inputNavName  = "inputNavName"
    moduleConfig.inputRefName  = "inputRefName"
    moduleConfig.outputDataName = "outputName"

    vector = [0.01, 0.05, -0.55]
    moduleConfig.sigma_R0R = vector


    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    #
    # Navigation Message
    #
    NavStateOutData = attTrackingError.NavAttIntMsg()  # Create a structure for the input message
    inputMessageSize = NavStateOutData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputNavName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)
    sigma_BN = [0.25, -0.45, 0.75]
    NavStateOutData.sigma_BN = sigma_BN
    omega_BN_B = [-0.015, -0.012, 0.005]
    NavStateOutData.omega_BN_B = omega_BN_B
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputNavName,
                                          inputMessageSize,
                                          0,
                                          NavStateOutData)

    #
    # Reference Frame Message
    #
    RefStateOutData = attTrackingError.AttRefFswMsg()  # Create a structure for the input message
    inputMessageSize = RefStateOutData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputRefName,
                                          inputMessageSize,
                                          2)            # number of buffers (leave at 2 as default, don't make zero)
    sigma_RN = [0.35, -0.25, 0.15]
    RefStateOutData.sigma_RN = sigma_RN
    omega_RN_N = [0.018, -0.032, 0.015]
    RefStateOutData.omega_RN_N = omega_RN_N
    domega_RN_N = [0.048, -0.022, 0.025]
    RefStateOutData.domega_RN_N = domega_RN_N
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputRefName,
                                          inputMessageSize,
                                          0,
                                          RefStateOutData)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.3))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    #
    # check sigma_BR
    #
    moduleOutputName = "sigma_BR"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))[0]

    sigma_RN2 = rbk.addMRP(np.array(sigma_RN), -np.array(vector))
    RN = rbk.MRP2C(sigma_RN2)
    BN = rbk.MRP2C(np.array(sigma_BN))
    BR = np.dot(BN, RN.T)
    # set the filtered output truth states
    trueVector = rbk.C2MRP(BR)
   
    # compare the module results to the truth values
    accuracy = 1e-12
    if not unitTestSupport.isArrayEqual(moduleOutput,trueVector,3,accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                            moduleOutputName + " unit test at t=" +
                            str(moduleOutput[0]*macros.NANO2SEC) +
                            "sec\n")
        unitTestSupport.writeTeXSnippet("passFail_sigBR", "FAILED", path)
    else:
        unitTestSupport.writeTeXSnippet("passFail_sigBR", "PASSED", path)

    #
    # check omega_BR_B
    #
    moduleOutputName = "omega_BR_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))[0]

    # set the filtered output truth states
    trueVector = np.array(omega_BN_B) - np.dot(BN, np.array(omega_RN_N))

    # compare the module results to the truth values
    if not unitTestSupport.isArrayEqual(moduleOutput,trueVector,3,accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                            moduleOutputName + " unit test at t=" +
                            str(moduleOutput[0]*macros.NANO2SEC) +
                            "sec\n")
        unitTestSupport.writeTeXSnippet("passFail_omega_BR_B", "FAILED", path)
    else:
        unitTestSupport.writeTeXSnippet("passFail_omega_BR_B", "PASSED", path)

    #
    # check omega_RN_B
    #
    moduleOutputName = "omega_RN_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))[0]

    # set the filtered output truth states
    trueVector = np.dot(BN, np.array(omega_RN_N))


# compare the module results to the truth values
    if not unitTestSupport.isArrayEqual(moduleOutput,trueVector,3,accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                            moduleOutputName + " unit test at t=" +
                            str(moduleOutput[0]*macros.NANO2SEC) +
                            "sec\n")
        unitTestSupport.writeTeXSnippet("passFail_omega_RN_B", "FAILED", path)
    else:
        unitTestSupport.writeTeXSnippet("passFail_omega_RN_B", "PASSED", path)

    #
    # check domega_RN_B
    #
    moduleOutputName = "domega_RN_B"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))[0]

    # set the filtered output truth states
    trueVector = np.dot(BN, np.array(domega_RN_N))


    # compare the module results to the truth values
    if not unitTestSupport.isArrayEqual(moduleOutput,trueVector,3,accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " +
                            moduleOutputName + " unit test at t=" +
                            str(moduleOutput[0]*macros.NANO2SEC) +
                            "sec\n")
        unitTestSupport.writeTeXSnippet("passFail_domega_RN_B", "FAILED", path)
    else:
        unitTestSupport.writeTeXSnippet("passFail_domega_RN_B", "PASSED", path)
    
    # Note that we can continue to step the simulation however we feel like.
    # Just because we stop and query data does not mean everything has to stop for good
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.6))    # run an additional 0.6 seconds
    unitTestSim.ExecuteSimulation()

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
    test_attTrackingError(False)
