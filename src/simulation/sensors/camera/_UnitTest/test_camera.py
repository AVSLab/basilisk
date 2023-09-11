# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

# Unit Test Script
# Module Name:        Camera
# Author:             Thibaud Teil
# Creation Date:      March 13, 2019


import inspect
import os

import numpy as np
import pytest

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
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
try:
    from Basilisk.simulation import camera
except ImportError:
    importErr = True
    reasonErr += "\nCamera not built---check OpenCV option"


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.

@pytest.mark.skipif(importErr, reason=reasonErr)
@pytest.mark.parametrize("gauss, darkCurrent, saltPepper, cosmic, blurSize", [
    (0, 0, 0, 0, 0)
    , (2, 2, 2, 1, 3)
])
def test_module(show_plots, gauss, darkCurrent, saltPepper, cosmic, blurSize):
    """
        **Validation Test Description**

        This module tests the proper functioning of the camera module. This is done by first ensuring that the reading
        and writing of the camera parameters are properly executed. The test then corrupts a test image accordingly.

        **Description of Variables Being Tested**

        The camera parameters tested are the camera position MRP and the isOn value for the camera. These ensure that
        the position is properly written and read. The image is also corrupted with the parameterized test information.
        This is directly tested by differencing the initial and processed image to see a change.
        and also ensures that the variables are properly read and that all the openCV functions
        are executing properly.

        - ``camera_MRP``
        - ``isON``
        - ``imageNorm Values``

        The comparative value for the test on the image is 1E-2 which depends on the corruptions but is allowed to me small
        as the relative difference of the images is taken (whereas pixel values can get large).

        The two parameterized test are set with and without corruptions.

        **General Documentation Comments**

        The script could benefit from more profound image processing testing. Currently the bulk of the image processing
        is only tested by the result image.
        """
    # each test method requires a single assert method to be called
    image = "mars.jpg"
    [testResults, testMessage] = cameraTest(show_plots, image, gauss, darkCurrent, saltPepper, cosmic, blurSize)

    # Clean up
    imagePath = path + '/' + image
    savedImage1 = '/'.join(imagePath.split('/')[:-1]) + '/' + str(gauss) + str(darkCurrent) \
                  + str(saltPepper) + str(cosmic) + str(blurSize) + '0.000000.png'
    savedImage2 = '/'.join(imagePath.split('/')[:-1]) + '/' + str(gauss) + str(darkCurrent) \
                  + str(saltPepper) + str(cosmic) + str(blurSize) + '0.500000.png'
    try:
        os.remove(savedImage1)
        os.remove(savedImage2)
    except FileNotFoundError:
        pass

    assert testResults < 1, testMessage


def cameraTest(show_plots, image, gauss, darkCurrent, saltPepper, cosmic, blurSize):
    if importErr:
        print(reasonErr)
        exit()

    # Truth values from python
    imagePath = path + '/' + image
    input_image = Image.open(imagePath)
    input_image.load()
    #################################################
    corrupted = (gauss > 0) or (darkCurrent > 0) or (saltPepper > 0) or (cosmic > 0) or (blurSize > 0)

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    bitmapArray = []

    # # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = camera.Camera()
    module.ModelTag = "cameras"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)
    module.filename = imagePath
    module.saveImages = True
    # make each image saved have a unique name for this test case
    module.saveDir = '/'.join(imagePath.split('/')[:-1]) + '/' + str(gauss) + str(darkCurrent) \
                           + str(saltPepper) + str(cosmic) + str(blurSize)

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = messaging.CameraImageMsgPayload()
    inputMessageData.timeTag = int(1E9)
    inputMessageData.cameraID = 1
    inCamMsg = messaging.CameraImageMsg().write(inputMessageData)
    module.imageInMsg.subscribeTo(inCamMsg)

    module.cameraIsOn = 1
    module.sigma_CB = [0, 0, 1]

    # Noise parameters
    module.gaussian = gauss
    module.darkCurrent = darkCurrent
    module.saltPepper = saltPepper
    module.cosmicRays = cosmic
    module.blurParam = blurSize

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.cameraConfigOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))  # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # Truth values from python
    if corrupted:
        corruptedPath = module.saveDir + '0.000000.png'
    else:
        corruptedPath = module.saveDir + '0.500000.png'
    output_image = Image.open(corruptedPath)

    isOnValues = dataLog.isOn
    pos = dataLog.sigma_CB

    #  Error check for corruption
    err = np.linalg.norm(np.linalg.norm(input_image, axis=2) - np.linalg.norm(output_image, axis=2)) / np.linalg.norm(
        np.linalg.norm(input_image, axis=2))

    if (err < 1E-2 and corrupted):
        testFailCount += 1
        testMessages.append("Image not corrupted and show be: " + image)

    if (err > 1E-2 and not corrupted):
        testFailCount += 1
        testMessages.append("Image corrupted and show not be: " + image)

    #   print out success message if no error were found
    for i in range(3):
        if np.abs(pos[-1, i] - module.sigma_CB[i]) > 1E-10:
            testFailCount += 1
            testMessages.append("Test failed position " + image)

    if np.abs(isOnValues[-1] - module.cameraIsOn) > 1E-10:
        testFailCount += 1
        testMessages.append("Test failed isOn " + image)

    if testFailCount:
        print(testMessages)
    else:
        print("Passed")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    cameraTest(False, "mars.jpg", 2, 0, 2, 1, 3)
