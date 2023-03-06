
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


import pytest
import os, inspect, glob
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
    reasonErr = "python Pillow package not installed---can't test CenterOfBrightness module"

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass, unitTestSupport
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

try:
    from Basilisk.fswAlgorithms import centerOfBrightness
except ImportError:
    importErr = True
    reasonErr = "Center Of Brightness not built---check OpenCV option"

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.

@pytest.mark.skipif(importErr, reason= reasonErr)
@pytest.mark.parametrize("image, blur,  saveTest, valid, saveImage", [
                    ("full_circle.png",   1, False, 1,  False)
                    ,("test_circle.png",  1, False, 1,  False)
                    ,("half_half.png",    1, True , 1,  False)
])

# update "module" in this function name to reflect the module name
def test_module(show_plots, image, blur, saveTest, valid, saveImage):
    """
    Unit test for Center of Brightness algorithm. The unit test specifically runs on 2 images:

        1. A crescent Mars: This image only contains a slim Mars crescent
        2. Moons: This image contains several Moon crescents

    This modules compares directly to the expected circles from the images.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = cobTest(show_plots, image, blur, saveTest, valid, saveImage)
    assert testResults < 1, testMessage


def cobTest(show_plots, image, blur, saveTest, valid, saveImage):

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

    # # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    # Construct algorithm and associated C++ container
    moduleConfig = centerOfBrightness.CenterOfBrightness()
    moduleConfig.ModelTag = "centerOfBrightness"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleConfig)

    moduleConfig.filename = imagePath
    moduleConfig.blurSize = blur
    moduleConfig.threshold = 50
    moduleConfig.saveDir = path + '/result_save.png'
    if saveTest:
        moduleConfig.saveImages = 1
    cob_ref = [input_image.width/2, input_image.height/2]
    if image == "half_half.png":
        cob_ref = [(3*input_image.width/4 + 1)*valid, int(input_image.height/2)*valid]
        pixelNum_ref = (int(input_image.width*input_image.height/2) + input_image.width)*valid

    # Create input message and size it because the regular creator of that message is not part of the test.
    inputMessageData = messaging.CameraImageMsgPayload()
    inputMessageData.timeTag = int(1E9)
    inputMessageData.cameraID = 1
    inputMessageData.valid = valid
    imgInMsg = messaging.CameraImageMsg().write(inputMessageData)
    moduleConfig.imageInMsg.subscribeTo(imgInMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = moduleConfig.opnavCOBOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    center = dataLog.centerOfBrightness[-1,:]
    pixelNum = dataLog.pixelsFound[-1]

    # Output image:
    output_image = Image.new("RGB", input_image.size)
    output_image.paste(input_image)
    draw_result = ImageDraw.Draw(output_image)

    imageProcCircles = []
    if pixelNum > 0:
        imageProcCircles.append((center[0], center[1], np.sqrt(pixelNum)/50))
    for x, y, r in imageProcCircles:
        draw_result.ellipse((x - r, y - r, x + r, y + r), outline=(255, 0, 0, 0))

    # Remove saved images for the test of that functionality
    files = glob.glob(path + "/result_*")
    for f in files:
        os.remove(f)
    # Save output image
    if saveImage:
        output_image.save("result_" + image)

    if show_plots:
        output_image.show()

    #   print out success message if no error were found
    for i in range(2):
        if np.abs((center[i] - cob_ref[i])/cob_ref[i]) > 1E-1:
            testFailCount+=1
            testMessages.append("Test failed processing " + image + " due to center offset")

    if image == "half_half.png":
        if np.abs((pixelNum - pixelNum_ref)) > 1E-1:
            testFailCount+=1
            testMessages.append("Test failed processing " + image + " due to number of detected pixels")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    cobTest(True, "test_circle.png",     5, "", 1, True) # Moon images
