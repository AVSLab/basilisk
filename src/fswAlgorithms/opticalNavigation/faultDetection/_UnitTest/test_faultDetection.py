#
#   Unit Test Script
#   Module Name:        pixelLineConverter.py
#   Creation Date:      May 16, 2019
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms.faultDetection import faultDetection
from Basilisk.utilities import RigidBodyKinematics as rbk

import os, inspect
import numpy as np
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

@pytest.mark.parametrize("r_c, valid", [
                      ([10,10,1],  1), #Mars image
                      ([1, 1, 1],  1), # Moon images
    ])

def test_faultdetection(show_plots, r_c, valid):
    """ Test opNav fault detection. """
    [testResults, testMessage] = faultdetection(show_plots, r_c, valid)
    assert testResults < 1, testMessage

def faultdetection(show_plots, r_c, valid):
    """ Test the faultdetection module. Setup a simulation """

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
    faults = opNavFaultDetection.FaultDetection()  # Create a config struct
    faults.navMeasPrimaryMsgName = "primary_opnav"
    faults.navMeasSecondaryMsgName = "secondary_opnav"
    faults.cameraConfigMsgName = "camera_config_name"
    faults.attInMsgName = "nav_att_name"
    # ephemNavConfig.outputState = simFswInterfaceMessages.NavTransIntMsg()

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    faultsWrap = unitTestSim.setModelDataWrap(faults)
    faultsWrap.ModelTag = "faultDet"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, pixelLineWrap, pixelLine)

    # Create the input messages.
    inputPrimary = pixelLineConverter.OpNavFswMsg()
    inputSecondary = pixelLineConverter.OpNavFswMsg()
    inputCamera = pixelLineConverter.CameraConfigMsg()
    inputAtt = pixelLineConverter.NavAttIntMsg()

    # Set camera
    inputCamera.focalLength = 1.
    inputCamera.sensorSize = [10, 10] # In mm
    inputCamera.resolution = [512, 512]
    inputCamera.sigma_CB = [1.,0.3,0.1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.cameraConfigMsgName, inputCamera)

    # Set attitude
    inputAtt.sigma_BN = [0.6, 1., 0.1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.attInMsgName, inputAtt)

    BN = rbk.MRP2C(inputAtt.sigma_BN)
    CB = rbk.MRP2C(inputCamera.sigma_CB)
    # Set circles
    inputPrimary.r_BN_C = r_c
    inputPrimary.covar_C = [0.5, 0., 0., 0., 0.5, 0., 0., 0., 1.]
    inputPrimary.timeTag = 12345
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.circlesInMsgName, inputCircles)
    # Set module for Mars
    pixelLine.opNavOutMsgName = "output_nav_msg"
    unitTestSim.TotalSim.logThisMessage(pixelLine.opNavOutMsgName)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()
    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # Truth Vlaues

    timTagExp = inputCircles.timeTag

    posErr = 1e-10
    covarErr = 1e-10
    unitTestSupport.writeTeXSnippet("toleranceValuePos", str(posErr), path)
    unitTestSupport.writeTeXSnippet("toleranceValueVel", str(covarErr), path)

    outputR = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.r_BN_N',  list(range(3)))
    outputCovar = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.covar_N',  list(range(9)))
    outputTime = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.timeTag')
    #
    #
    for i in range(len(outputR[-1, 1:])):
        if np.abs(r_Nexp[i] - outputR[-1, i+1]) > 1E-10 and np.isnan(outputR.any()):
            testFailCount += 1
            testMessages.append("FAILED: Position Check in pixelLine")

    for i in range(len(outputCovar[-1, 1:])):
        if np.abs((covar_Nexp[i] - outputCovar[-1, i+1])) > 1E-10 and np.isnan(outputTime.any()):
            testFailCount += 1
            testMessages.append("FAILED: Covar Check in pixelLine")

    if np.abs((timTagExp - outputTime[-1, 1])/timTagExp) > 1E-10 and np.isnan(outputTime.any()):
        testFailCount += 1
        testMessages.append("FAILED: Time Check in pixelLine")
    #
    #   print out success message if no error were found
    snippentName = "passFail"
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + pixelLineWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + pixelLineWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)


    return [testFailCount, ''.join(testMessages)]


if __name__ == '__main__':
    test_pixelLine_converter()
