#
#   Unit Test Script
#   Module Name:        ephemNavConverter
#   Creation Date:      October 16, 2018
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms import ephem_nav_converter, pixelLineConverter
from Basilisk.utilities import astroFunctions

import os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def test_pixelLine_converter():
    """ Test ephemNavConverter. """
    [testResults, testMessage] = pixelLineConverterTestFunction()
    assert testResults < 1, testMessage

def pixelLineConverterTestFunction():
    """ Test the ephemNavConverter module. Setup a simulation """

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

    # Construct the ephemNavConverter module
    # Set the names for the input messages
    pixelLine = pixelLineConverter.PixelLineConvertData()  # Create a config struct
    pixelLine.circlesInMsgName = "circles_name"
    pixelLine.cameraConfigMsgName = "camera_config_name"
    # ephemNavConfig.outputState = simFswInterfaceMessages.NavTransIntMsg()

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    pixelLineWrap = unitTestSim.setModelDataWrap(pixelLine)
    pixelLineWrap.ModelTag = "pixelLineConverter"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, pixelLineWrap, pixelLine)

    # Create the input messages.
    inputCamera = pixelLineConverter.CameraConfigMsg()
    inputCircles = pixelLineConverter.CirclesOpNavMsg()

    # Get the Earth's position and velocity
    pixelLine.planetTarget = 2
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.ephInMsgName, inputCamera)

    unitTestSim.TotalSim.logThisMessage(pixelLine.stateOutMsgName)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    posAcc = 1e1
    velAcc = 1e-4
    unitTestSupport.writeTeXSnippet("toleranceValuePos", str(posAcc), path)
    unitTestSupport.writeTeXSnippet("toleranceValueVel", str(velAcc), path)

    # outputR = unitTestSim.pullMessageLogData(pixelLine.stateOutMsgName + '.r_BN_N',  range(3))
    # outputV = unitTestSim.pullMessageLogData(pixelLine.stateOutMsgName + '.v_BN_N',  range(3))
    # outputTime = unitTestSim.pullMessageLogData(pixelLine.stateOutMsgName + '.timeTag')
    #
    # trueR = [position, position]
    # trueV = [velocity, velocity]
    # trueTime = [inputEphem.timeTag, inputEphem.timeTag]
    #
    # # At each timestep, make sure the vehicleConfig values haven't changed from the initial values
    # testFailCount, testMessages = unitTestSupport.compareArrayND(trueR, outputR,
    #                                                              posAcc,
    #                                                              "ephemNavConverter output Position",
    #                                                              2, testFailCount, testMessages)
    # testFailCount, testMessages = unitTestSupport.compareArrayND(trueV, outputV,
    #                                                              velAcc,
    #                                                              "ephemNavConverter output Velocity",
    #                                                              2, testFailCount, testMessages)
    # testFailCount, testMessages = unitTestSupport.compareDoubleArray(trueTime, outputTime,
    #                                                              velAcc,
    #                                                              "ephemNavConverter output Time",
    #                                                              testFailCount, testMessages)
    #
    # #   print out success message if no error were found
    # snippentName = "passFail"
    # if testFailCount == 0:
    #     colorText = 'ForestGreen'
    #     print "PASSED: " + ephemNavWrap.ModelTag
    #     passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    # else:
    #     colorText = 'Red'
    #     print "Failed: " + ephemNavWrap.ModelTag
    #     passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    # unitTestSupport.writeTeXSnippet(snippentName, passedText, path)


    return [testFailCount, ''.join(testMessages)]


if __name__ == '__main__':
    test_pixelLine_converter()
