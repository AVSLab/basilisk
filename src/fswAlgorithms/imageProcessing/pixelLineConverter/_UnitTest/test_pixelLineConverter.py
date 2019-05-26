#
#   Unit Test Script
#   Module Name:        ephemNavConverter
#   Creation Date:      October 16, 2018
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms import pixelLineConverter
from Basilisk.utilities import astroFunctions

import os, inspect
import numpy as np
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def mapState(state, planet, camera):
    D = planet["diameter"]
    f = camera["focal"]
    d_x = camera["pixelSizeX"]
    d_y = camera["pixelSizeY"]
    A = 2 * np.arctan(state[2]*d_x/f)

    norm = 0.5 * D/np.sin(0.5*A)
    vec = np.array([state[0]*d_x/f, state[1]*d_y/f, 1.])
    return norm*vec/np.linalg.norm(vec)

# def algebraic((state, planet, camera):
#     A = 1./())


def mapCovar(CovarXYR, rho, planet, camera):
    D = planet["diameter"]
    f = camera["focal"]
    d_x = camera["pixelSizeX"]
    d_y = camera["pixelSizeY"]
    A = 2 * np.arctan(rho*d_x/f)

    # rho_map = (0.33 * D * np.cos(A)/np.sin(A/2.)**2. * 2./f * 1./(1. + (rho/f)**2.) * (d_x/f) )
    rho_map = 0.5*D*(-f*np.sqrt(1 + rho**2*d_x**2/f**2)/(rho**2*d_x) + d_x/(f*np.sqrt(1 + rho**2*d_x**2/f**2)))
    x_map =   0.5 * D/np.sin(0.5*A)*(d_x/f)
    y_map =  0.5 * D/np.sin(0.5*A)*(d_y/f)
    CovarMap = np.array([[x_map,0.,0.],[0., y_map, 0.],[0.,0., rho_map]])
    return np.dot(CovarMap, np.dot(CovarXYR, CovarMap.T))

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
    pixelLine.attInMsgName = "nav_att_name"
    # ephemNavConfig.outputState = simFswInterfaceMessages.NavTransIntMsg()

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    pixelLineWrap = unitTestSim.setModelDataWrap(pixelLine)
    pixelLineWrap.ModelTag = "pixelLineConverter"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, pixelLineWrap, pixelLine)

    # Create the input messages.
    inputCamera = pixelLineConverter.CameraConfigMsg()
    inputCircles = pixelLineConverter.CirclesOpNavMsg()
    inputAtt = pixelLineConverter.NavAttIntMsg()

    # Set camera
    inputCamera.focalLength = 1.
    inputCamera.sensorSize = [10, 10] # In mm
    inputCamera.resolution = [512, 512]
    inputCamera.sigma_BC = [1.,0.,0.]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.cameraConfigMsgName, inputCamera)

    # Set circles
    inputCircles.circlesCenters = [152, 251]
    inputCircles.circlesRadii = [75]
    inputCircles.uncertainty = [0.5, 0.5, 1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.circlesInMsgName, inputCircles)

    # Set attitude
    inputAtt.sigma_BN = [0., 1., 0.]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.attInMsgName, inputAtt)

    # Get the Earth's position and velocity
    pixelLine.planetTarget = 2
    pixelLine.opNavOutMsgName = "output_nav_msg"
    unitTestSim.TotalSim.logThisMessage(pixelLine.opNavOutMsgName)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()
    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    posAcc = 1e1
    velAcc = 1e-4
    unitTestSupport.writeTeXSnippet("toleranceValuePos", str(posAcc), path)
    unitTestSupport.writeTeXSnippet("toleranceValueVel", str(velAcc), path)

    outputR = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.r_B',  range(3))
    outputCovar = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.covar',  range(9))
    outputTime = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.timeTag')
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
