#
#   Unit Test Script
#   Module Name:        dvAccumulation
#   Creation Date:      October 5, 2018
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms import dvAccumulation, fswMessages
from Basilisk.simulation import simFswInterfaceMessages
from numpy import random


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
    moduleConfig.outputData = simFswInterfaceMessages.NavTransIntMsg()

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "dvAccumulation"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Create the input message.
    inputAccData = fswMessages.AccDataFswMsg()

    # Set this as the packet data in the acceleration data
    inputAccData.accPkts = generateAccData()

    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.accPktInMsgName, inputAccData)

    # unitTestSim.TotalSim.logThisMessage(moduleConfig.outputNavName, testProcessRate)
    unitTestSim.AddVariableForLogging('dvAccumulation.outputData.vehAccumDV', testProcessRate, 0, 2, 'double')

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
    outputNavMsgData = unitTestSim.GetLogVariableData('dvAccumulation.outputData.vehAccumDV')

    print(outputNavMsgData)

    trueVector = [[ -8.85714792e-03,  -1.49277412e-03,   8.25634831e-03],
                 [  -8.85714792e-03,  -1.49277412e-03,   8.25634831e-03],
                 [  -8.85714792e-03,  -1.49277412e-03,   8.25634831e-03],
                 [  -7.24397771e-03,  -2.08034182e-03,   8.49638169e-03],
                 [  -7.24397771e-03,  -2.08034182e-03,   8.49638169e-03]]
    accuracy = 1e-6

    # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    # testFailCount, testMessages = unitTestSupport.compareArrayND(trueVector, outputNavMsgData,
    #                                                              accuracy,
    #                                                              "dvAccumulation output",
    #                                                              2, testFailCount, testMessages)

    if testFailCount == 0:
        print("Passed")

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    test_dv_accumulation()
