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
# Author:             Lucas Webb and Hanspeter Schaub
# Creation Date:      April 30, 2020


import colorsys
import inspect
import math
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
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass

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

@pytest.mark.skipif(importErr, reason= reasonErr)
@pytest.mark.parametrize("HSV", [
    [0, 0, 0]
    , [1.0, 20.0, -30.0]
    , [-1.0, 20.0, -30.0]
    , [3.14159, 100, -100]
])
@pytest.mark.parametrize("BGR", [
    [0, 0, 0]
    , [10, 20, 30]
    , [-10, -30, 50]
    , [-100, 200, 20]
])
# update "module" in this function name to reflect the module name
def test_module(show_plots, HSV, BGR):
    """
        **Validation Test Description**

        This module tests the color shifting capability of the camera module. Multiple HSV and BGR color
        adjustments are tests on a TV test image.

        **Description of Variables Being Tested**

        Multiple points on the test images are adjusted in python and compared to the BSK camera module
        saved image.  The HSV and BGR color corrections are applied on a series of points in the test image.
        The integer red, green and blue color values are checked to be identical.

    """
    # each test method requires a single assert method to be called
    image = "tv_test.png"
    [testResults, testMessage] = cameraColorTest(image, HSV, BGR)

    # Clean up
    imagePath = path + '/' + image
    savedImage = '/'.join(imagePath.split('/')[:-1]) + '/hsv' + str(HSV) + 'bgr' + str(BGR) + '0.000000.png'
    try:
        os.remove(savedImage)
    except FileNotFoundError:
        pass

    assert testResults < 1, testMessage


def cameraColorTest(image, HSV, BGR):
    """
    Test method to apply the HSV and BGR image adjustments.

    :param image: image name to load from local folder
    :param HSV: 3d vector of HSV adjustments
    :param BGR: 3d vector of BGR adjustments

    """
    if importErr:
        print(reasonErr)
        exit()

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

    # Create test thread
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
    module.saveDir = '/'.join(imagePath.split('/')[:-1]) + '/hsv' + str(HSV) + 'bgr' + str(BGR)

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
    module.bgrPercent = np.array(BGR)
    module.hsv = np.array(HSV)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.cameraConfigOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    corruptedPath = module.saveDir + '0.000000.png'

    #   print out error message if test failed
    if not trueColorAdjust(imagePath, corruptedPath, HSV, BGR):
        testFailCount += 1
        testMessages.append("Test failed color adjustment  " + image)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


def rgb_to_hsv(rgb):
    hsv = colorsys.rgb_to_hsv(rgb[0], rgb[1], rgb[2])
    return [hsv[0] * 180., hsv[1] * 255., hsv[2]]


def hsv_to_rgb(hsv):
    rgb = colorsys.hsv_to_rgb(hsv[0]/180., hsv[1]/255., hsv[2]/255.)
    return [rgb[0] * 255, rgb[1] * 255, rgb[2] * 255]


def trueColorAdjust(image, corrupted, HSV, BGR):
    input_rgb = Image.open(image).load()
    output = Image.open(corrupted).load()

    # these points correspond to the included 'tv_test.png'
    testPoints = [(100, 300), (250, 300), (450, 300), (600, 300), (700, 300), (950, 300), (1100, 300), (300, 800),
                  (880, 780)]

    for point in testPoints:
        px = point[0]
        py = point[1]

        # do HSV adjustment
        hsv = rgb_to_hsv(input_rgb[px, py])
        expected = [0, 0, 0]
        input_degrees = math.degrees(HSV[0])
        h_360 = (hsv[0] * 2) + input_degrees
        h_360 -= 360. * math.floor(h_360 * (1. / 360.))
        h_360 = int(h_360 / 2)
        if h_360 == 180:
            h_360 = 0
        expected[0] = int(h_360)

        for i in range(2):
            expected[i+1] = int(hsv[i+1] * (HSV[i+1]/100. + 1.))
            if expected[i+1] < 0:
                expected[i+1] = 0
            if expected[i+1] > 255:
                expected[i+1] = 255

        expectedAfterHSV = hsv_to_rgb(expected)
        expectedAfterHSV = [int(i) for i in expectedAfterHSV]

        # do BGR adjustment
        for i in range(3):
            expected[i] = int((BGR[2 - i] / 100. + 1.) * expectedAfterHSV[i])
            if expected[i] > 255:
                expected[i] = 255
            if expected[i] < 0:
                expected[i] = 0

        for i in range(3):
            if abs(int(output[px, py][i]) - expected[i]) > 3:
                print("Failed HSV at point: px=" + str(px) + " py= + " + str(py))
                return False
    print("Passed Color Check")
    return True


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    hsvAdjust = [1.0, 20.0, -30.0]
    bgrAdjust = [-100, 0, 0]
    cameraColorTest("tv_test.png", hsvAdjust, bgrAdjust)
