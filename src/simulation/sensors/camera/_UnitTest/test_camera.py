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
'''
  Unit Test Script
  Module Name:        Camera
  Author:             Thibaud Teil
  Creation Date:      March 13, 2019
'''

import pytest
import sys, os, inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
importErr = False
reasonErr = ""
try:
    from PIL import Image, ImageDraw
except ImportError:
    importErr = True
    reasonErr = "python Pillow package not installed---can't test Cameras module"

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass, unitTestSupport
from Basilisk.utilities import macros

try:
    from Basilisk.simulation import camera
except ImportError:
    importErr = True
    reasonErr = "Camera not built---check OpenCV option"

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.

@pytest.mark.skipif(importErr, reason= reasonErr)
@pytest.mark.parametrize("image, gauss, darkCurrent, saltPepper, cosmic, blurSize, saveImage", [
                    ("mars.jpg",    2,          2,      2,   1,   3 , False) #Mars image
    ])

# update "module" in this function name to reflect the module name
def test_module(show_plots, image, gauss, darkCurrent, saltPepper, cosmic, blurSize, saveImage):
    """
        **Validation Test Description**

        This module tests the proper functioning of the camera module. This is done by first ensuring that the reading
        and writing of the camera parameters are properly executed. The test then corrupts a test image accordingly.

        **Description of Variables Being Tested**

        The camera parameters tested are the camera position MRP and the isOn value for the camera. These ensure that
        the position is properly written and read. The image is also corrupted with the parameterized test information.
        Although not directly tested this ensures that the variables are properly read and that all the openCV functions
        are executing properly.

        - ``camera_MRP``
        - ``isON``

        **General Documentation Comments**

        The script could benefit from more profound image processing testing. Currently the bulk of the image processing
        is only tested by the result image.
        """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = cameraTest(show_plots, image, gauss, darkCurrent, saltPepper, cosmic, blurSize, saveImage)
    assert testResults < 1, testMessage


def cameraTest(show_plots, image, gauss, darkCurrent, saltPepper, cosmic, blurSize, saveImage):

    # Truth values from python
    imagePath = path + '/' + image
    input_image = Image.open(imagePath)
    input_image.load()
    #################################################

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

    bitmapArray = []

    # # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = camera.Camera()
    moduleConfig.ModelTag = "cameras"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleConfig)
    moduleConfig.imageInMsgName = "sample_image"
    moduleConfig.cameraOutMsgName = "cameraOut"
    moduleConfig.imageOutMsgName = "out_image"
    moduleConfig.filename = imagePath
    moduleConfig.saveImages = 1 if saveImage else 0
    moduleConfig.saveDir = '/'.join(imagePath.split('/')[:-1]) + '/'

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = camera.CameraImageMsg()
    inputMessageData.timeTag = int(1E9)
    inputMessageData.cameraID = 1
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.imageInMsgName,
                               inputMessageData)

    moduleConfig.cameraIsOn = 1
    moduleConfig.sigma_CB = [0,0,1]

    # Noise parameters
    moduleConfig.gaussian = gauss
    moduleConfig.darkCurrent = darkCurrent
    moduleConfig.saltPepper = saltPepper
    moduleConfig.cosmicRays = cosmic
    moduleConfig.blurParam = blurSize

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.cameraOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(5.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    isOnValues = unitTestSim.pullMessageLogData(moduleConfig.cameraOutMsgName + ".isOn")
    pos = unitTestSim.pullMessageLogData(moduleConfig.cameraOutMsgName + ".sigma_CB", list(range(3)))


    #   print out success message if no error were found
    for i in range(3):
        if np.abs(pos[-1,i+1] - moduleConfig.sigma_CB[i])>1E-10:
            testFailCount+=1
            testMessages.append("Test failed position " + image)

    if np.abs(isOnValues[-1,1] - moduleConfig.cameraIsOn)>1E-10:
        testFailCount+=1
        testMessages.append("Test failed isOn " + image)


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    cameraTest(True, "mars.jpg", 2,          0,      2,   1,   3 , True) # Mars image
