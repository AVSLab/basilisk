''' '''
import math

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
import sys, os, inspect, time
import numpy as np
import colorsys

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
    ("mars.jpg",    0,          0,      0,   0,   0,  True),  # Mars image
    ("mars.jpg",    2,          2,      2,   1,   3 , True) #Mars image
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
    [testResults, testMessage] = cameraTest(show_plots, image, gauss, darkCurrent, saltPepper, cosmic, blurSize, saveImage)
    assert testResults < 1, testMessage


def cameraTest(show_plots, image, gauss, darkCurrent, saltPepper, cosmic, blurSize, saveImage):
    # Truth values from python
    imagePath = path + '/' + image
    input_image = Image.open(imagePath)
    input_image.load()
    #################################################
    corrupted = (gauss>0) or (darkCurrent>0) or (saltPepper>0) or (cosmic>0) or (blurSize>0)

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
    moduleConfig.HSV = camera.DoubleVector([0, 0, 0])
    moduleConfig.RGB = camera.DoubleVector([0, 0, 0])

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.cameraOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # Truth values from python
    if corrupted:
        corruptedPath = path + '/' + '0.000000.jpg'
    else:
        corruptedPath = path + '/' + '0.500000.jpg'
    output_image = Image.open(corruptedPath)

    isOnValues = unitTestSim.pullMessageLogData(moduleConfig.cameraOutMsgName + ".isOn")
    pos = unitTestSim.pullMessageLogData(moduleConfig.cameraOutMsgName + ".sigma_CB", list(range(3)))

    #  Error check for corruption
    err = np.linalg.norm(np.linalg.norm(input_image, axis=2) - np.linalg.norm(output_image, axis=2))/np.linalg.norm(np.linalg.norm(input_image, axis=2))

    if (err < 1E-2 and corrupted):
        testFailCount += 1
        testMessages.append("Image not corrupted and show be: " + image)

    if (err > 1E-2 and not corrupted):
        testFailCount += 1
        testMessages.append("Image corrupted and show not be: " + image)

    #   print out success message if no error were found
    for i in range(3):
        if np.abs(pos[-1,i+1] - moduleConfig.sigma_CB[i])>1E-10:
            testFailCount+=1
            testMessages.append("Test failed position " + image)

    if np.abs(isOnValues[-1,1] - moduleConfig.cameraIsOn)>1E-10:
        testFailCount+=1
        testMessages.append("Test failed isOn " + image)

    # Comment this out to keep files after each test for debugging purposes
    # Clean up
    # try:
    #     os.remove(path + "/0.000000.jpg")
    #     os.remove(path + "/0.500000.jpg")
    # except FileNotFoundError:
    #     pass

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

# which = 0 -> test BGR
# which = 1 - > test HSV
# can't test both at same time
def cameraColorTest(image, saveImage, BGR, HSV, which):
    # Truth values from python
    imagePath = path + '/' + image
    input_image = Image.open(imagePath)
    input_image.load()
    #################################################

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
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
    moduleConfig.sigma_CB = [0, 0, 1]

    # Noise parameters
    # BGR and HSV are python lists of the form [0, 0, 0]
    if which == 0:
        moduleConfig.bgrPercent = camera.IntVector(BGR)
    elif which == 1:
        moduleConfig.hsv = camera.DoubleVector(HSV)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.cameraOutMsgName, testProcessRate)
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))  # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    corruptedPath = path + '/' + '0.000000.jpg'
    output_image = Image.open(corruptedPath)

    #   print out error message if test failed
    if which == 0:
        if not testBGR(imagePath, corruptedPath, BGR):
            testFailCount += 1
            testMessages.append("Test failed BGR " + image)
    elif which == 1:
        if not testHSV(imagePath, corruptedPath, HSV):
            testFailCount += 1
            testMessages.append("Test failed HSV " + image)

    # Comment this out to keep files after each test for debugging purposes
    # Clean up
    # try:
    #     os.remove(path + "/0.000000.jpg")
    #     os.remove(path + "/0.500000.jpg")
    # except FileNotFoundError:
    #     pass

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# these points correspond to the included 'tv_test.png'
testPoints = [(100, 300), (250, 300), (450, 300), (600, 300), (700, 300), (950, 300), (1100, 300), (300, 800),
              (880, 780)]


def testBGR(image, corrupted, BGR):
    input_rgb = Image.open(image).load()
    output = Image.open(corrupted).load()

    for point in testPoints:
        px = point[0]
        py = point[1]

        expected = [0, 0, 0]
        for i in range(3):
            expected[i] = int((BGR[2-i]/100 + 1) * input_rgb[px, py][i])
            if expected[i] > 255:
                expected[i] = 255
            if expected[i] < 0:
                expected[i] = 0
            if abs(output[px, py][i] - expected[i]) > 2:
                print("Failed BGR at point: ", end="")
                print(point)
                return False
    print("Passed BGR")
    return True


def rgb_to_hsv(rgb):
    hsv = colorsys.rgb_to_hsv(rgb[0], rgb[1], rgb[2])
    return [hsv[0] * 180, hsv[1] * 255, hsv[2]]


def hsv_to_rgb(hsv):
    rgb = colorsys.hsv_to_rgb(hsv[0]/180, hsv[1]/255, hsv[2]/255)
    return [rgb[0] * 255, rgb[1] * 255, rgb[2] * 255]


def testHSV(image, corrupted, HSV):
    input_rgb = Image.open(image).load()
    output = Image.open(corrupted).load()

    for point in testPoints:
        px = point[0]
        py = point[1]

        hsv = rgb_to_hsv(input_rgb[px, py])
        expected = [0, 0, 0]
        input_degrees = math.degrees(HSV[0])
        h_360 = (hsv[0] * 2) + input_degrees
        h_360 -= 360. * math.floor(h_360 * (1. / 360.))
        h_360 = int(h_360 / 2)
        if h_360 == 180:
            h_360 = 0
        expected[0] = h_360

        for i in range(2):
            expected[i+1] = hsv[i+1] * (HSV[i+1]/100 + 1)
            if expected[i+1] < 0:
                expected[i+1] = 0
            if expected[i+1] > 255:
                expected[i+1] = 255

        for i in range(3):
            expected_rgb = hsv_to_rgb(expected)
            if abs(int(output[px, py][i]) - expected_rgb[i]) > 3:
                print("Failed HSV at point: ", end="")
                print(point)
                return False
    print("Passed HSV")
    return True

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    #cameraTest(True, "mars.jpg", 2,          0,      2,   1,   3 , True) # Mars image
    bgr = [0, 0, 0]
    hsv = [3, 30, 90]
    cameraColorTest("tv_test.png", 1, bgr, hsv, 1)