#
#   Unit Test Script
#   Module Name:        dvAccumulation
#   Creation Date:      October 5, 2018
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms import dvAccumulation, fswMessages
from numpy import random
import os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


def generateAccData():
    """ Returns a list of random AccPktDataFswMsg."""
    accPktList = list()
    for _ in range(120):
        accPacketData = fswMessages.AccPktDataFswMsg()
        accPacketData.measTime = abs(long(random.normal(5e7, 1e7)))
        accPacketData.accel_B = random.normal(0.1, 0.2, 3)  # Acceleration in platform frame [m/s2]
        accPktList.append(accPacketData)

    return accPktList

def test_dv_accumulation():
    """ Test dvAccumulation. """
    [testResults, testMessage] = dvAccumulationTestFunction()
    assert testResults < 1, testMessage

def dvAccumulationTestFunction():
    """ Test the dvAccumulation module. Setup a simulation, """

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

    # Construct the dvAccumulation module
    # Set the names for the input messages
    moduleConfig = dvAccumulation.DVAccumulationData()  # Create a config struct
    moduleConfig.accPktInMsgName = "inputs_acceleration_packets"
    moduleConfig.outputNavName = "output_navigation_name"

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "dvAccumulation"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Create the input message.
    inputAccData = fswMessages.AccDataFswMsg()

    # Set this as the packet data in the acceleration data
    random.seed(12345)
    inputAccData.accPkts = generateAccData()

    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.accPktInMsgName, inputAccData)

    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputNavName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # Create the input message again to simulate multiple acceleration inputs.
    inputAccData = fswMessages.AccDataFswMsg()

    # Set this as the packet data in the acceleration data. Test the module with different inputs.
    inputAccData.accPkts = generateAccData()

    # Write this message
    msgSize = inputAccData.getStructSize()
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.accPktInMsgName, msgSize, 0, inputAccData)

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # This doesn't work if only 1 number is passed in as the second argument, but we don't need the second
    outputNavMsgData = unitTestSim.pullMessageLogData(moduleConfig.outputNavName + '.' + 'vehAccumDV', range(3))
    timeMsgData = unitTestSim.pullMessageLogData(moduleConfig.outputNavName + '.' + 'timeTag')

    # print(outputNavMsgData)
    # print timeMsgData

    trueDVVector = [[4.82820079e-03,   7.81971465e-03,   2.29605663e-03],
                 [ 4.82820079e-03,   7.81971465e-03,   2.29605663e-03],
                 [ 4.82820079e-03,   7.81971465e-03,   2.29605663e-03],
                 [ 6.44596343e-03,   9.00203561e-03,   2.60580728e-03],
                 [ 6.44596343e-03,   9.00203561e-03,   2.60580728e-03]]
    trueTime = [ [7.2123026e+07], [7.2123026e+07], [7.2123026e+07], [7.6667436e+07], [7.6667436e+07]]

    accuracy = 1e-6
    unitTestSupport.writeTeXSnippet("toleranceValue", str(accuracy), path)

    # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    testFailCount, testMessages = unitTestSupport.compareArrayND(trueDVVector, outputNavMsgData,
                                                                 accuracy,
                                                                 "dvAccumulation output",
                                                                 2, testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArrayND(trueTime, timeMsgData,
                                                                 accuracy, "timeTag", 1,
                                                                 testFailCount, testMessages)

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
    test_dv_accumulation()
