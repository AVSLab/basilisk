#
#   Unit Test Script
#   Module Name:        dvGuidance
#   Creation Date:      October 5, 2018
#

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import dvGuidance
from Basilisk.fswAlgorithms import fswMessages
import matplotlib.pyplot as plt
import os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))



def test_dv_guidance(show_plots):
    """ Test dvGuidance. """
    [testResults, testMessage] = dvGuidanceTestFunction(show_plots)
    assert testResults < 1, testMessage

def dvGuidanceTestFunction(show_plots):
    """ Test the dvGuidance module. Setup a simulation, write a DvBurnCmdFswMsg, and confirm that dvGuidance outputs the
        correct values. """

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # This is needed if multiple unit test scripts are run
    # This create a fresh and consistent simulation environment for each test run
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # Add a new task to the process

    # Construct the dvGuidance module
    moduleConfig = dvGuidance.dvGuidanceConfig()  # Create a config struct
    moduleConfig.outputDataName = "dv_guidance_output"
    moduleConfig.inputBurnDataName = "input_burn_data"

    # This calls the algContain to setup the selfInit, crossInit, and update
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "dvGuidance"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # The dvGuidance module reads in from the dvBurnCmd, so create that message here
    dvBurnCmdMsg = fswMessages.DvBurnCmdFswMsg()
    # NOTE: This is nonsense. These are random numbers
    dvBurnCmdMsg.dvInrtlCmd = [5, 5, 5]
    dvBurnCmdMsg.dvRotVecUnit = [1, 0, 0]
    dvBurnCmdMsg.dvRotVecMag = .5
    dvBurnCmdMsg.burnStartTime = macros.sec2nano(0.5)
    # Write this message
    msgSize = dvBurnCmdMsg.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputBurnDataName,
                                          msgSize,
                                          2)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputBurnDataName,
                                          msgSize,
                                          0,
                                          dvBurnCmdMsg)

    # Log the output message
    # unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0)) # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # Get the output from this simulation
    moduleOutputName = 'dvAttGuidance'
    outSigma = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + 'sigma_RN', range(3))
    outOmega = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + 'omega_RN_N', range(3))
    outDOmega = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + 'domega_RN_N', range(3))

    # NOTE: these values are just from a previous run. These should be validated
    trueSigma = [[5.69822629e-01, 1.99143700e-01, 2.72649472e-01],
                 [6.12361487e-01, 1.31298090e-01, 3.16981631e-01],
                 [6.50967464e-01, 5.62624705e-02, 3.61117890e-01]]
    trueOmega = [[4.08248290e-01, -2.04124145e-01, -2.04124145e-01],
                 [4.08248290e-01, -2.04124145e-01, -2.04124145e-01],
                 [4.08248290e-01, -2.04124145e-01, -2.04124145e-01]]
    trueDOmega =[[0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
                 [0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
                 [0.00000000e+00, 0.00000000e+00, 0.00000000e+00]]

    accuracy = 1e-9
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    for i in range(len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(outSigma[i], trueSigma[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName + " unit test at t=" + str(
                    outSigma[i, 0] * macros.NANO2SEC) + "sec\n")
        if not unitTestSupport.isArrayEqual(outOmega[i], trueOmega[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName + " unit test at t=" + str(
                    outOmega[i, 0] * macros.NANO2SEC) + "sec\n")
        if not unitTestSupport.isArrayEqual(outDOmega[i], trueDOmega[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName + " unit test at t=" + str(
                    outDOmega[i, 0] * macros.NANO2SEC) + "sec\n")

    # print(outSigma)
    # print(outOmega)
    # print(outDOmega)

    plt.figure()
    plt.plot(outSigma[:, 0] * macros.NANO2SEC, outSigma[:,1], label="Sigma 1")
    plt.plot(outSigma[:, 0] * macros.NANO2SEC, outSigma[:, 2], label="Sigma 2")
    plt.plot(outSigma[:, 0] * macros.NANO2SEC, outSigma[:, 3], label="Sigma 3")
    plt.legend(loc='upper left')
    plt.xlabel('Time [s]')
    plt.ylabel('Sigma')

    plt.figure()
    plt.plot(outOmega[:, 0] * macros.NANO2SEC, outOmega[:, 1], label="Omega 1")
    plt.plot(outOmega[:, 0] * macros.NANO2SEC, outOmega[:, 2], label="Omega 2")
    plt.plot(outOmega[:, 0] * macros.NANO2SEC, outOmega[:, 3], label="Omega 3")
    plt.legend(loc='upper left')
    plt.xlabel('Time [s]')
    plt.ylabel('Omega [rad/s]')

    plt.figure()
    plt.plot(outDOmega[:, 0] * macros.NANO2SEC, outDOmega[:, 1], label="DOmega 1")
    plt.plot(outDOmega[:, 0] * macros.NANO2SEC, outDOmega[:, 2], label="DOmega 2")
    plt.plot(outDOmega[:, 0] * macros.NANO2SEC, outDOmega[:, 3], label="DOmega 3")
    plt.legend(loc='upper left')
    plt.xlabel('Time [s]')
    plt.ylabel('DOmega')

    if show_plots:
        plt.show()

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

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    test_dv_guidance(show_plots=False)