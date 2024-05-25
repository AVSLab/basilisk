
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


import cv2
import pytest, os, inspect, glob
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

importErr = False
reasonErr = ""
try:
    from PIL import Image, ImageDraw
except ImportError:
    importErr = True
    reasonErr = "python Pillow package not installed---can't test CenterOfBrightness module"

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

try:
    from Basilisk.fswAlgorithms import centerOfBrightness
except ImportError:
    importErr = True
    reasonErr = "Center Of Brightness not built---check OpenCV option"


def computeWindowCenter(windowPointTopLeft, windowPointBottomRight):
    center_x = int(windowPointTopLeft[0] + (windowPointBottomRight[0] - windowPointTopLeft[0])/2)
    center_y = int(windowPointTopLeft[1] + (windowPointBottomRight[1] - windowPointTopLeft[1])/2)

    return np.array([center_x, center_y])


def computeWindowSize(windowPointTopLeft, windowPointBottomRight):
    width = int(windowPointBottomRight[0] - windowPointTopLeft[0])
    height = int(windowPointBottomRight[1] - windowPointTopLeft[1])

    return [width, height]


@pytest.mark.skipif(importErr, reason= reasonErr)
@pytest.mark.parametrize("image, blur,  saveTest, validImage, saveImage, windowPointTopLeft, windowPointBottomRight",
                         [("full_circle.png", 1, False, False, False, [0, 0], [0, 0]),
                          ("full_circle.png", 1, False, True, False, [0, 0], [0, 0]),
                          ("test_circle.jpeg", 5, False, True, False, [0, 0], [0, 0]),
                          ("half_half.png", 1, True, True, False, [0, 0], [0, 0]),
                          ("half_half.png", 1, True, True, False, [50, 0], [275, 91])
                          ])
def test_module(show_plots, image, blur, saveTest, validImage, saveImage, windowPointTopLeft, windowPointBottomRight):
    centerOfBrightnessTest(show_plots, image, blur, saveTest, validImage, saveImage, windowPointTopLeft,
                           windowPointBottomRight)


def centerOfBrightnessTest(show_plots, image, blur, saveTest, validImage, saveImage, windowPointTopLeft,
                           windowPointBottomRight):
    imagePath = path + '/' + image
    imagePath_module = path + '/temp_' + image
    input_image = Image.open(imagePath)
    input_image.load()

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    windowCenter = computeWindowCenter(windowPointTopLeft, windowPointBottomRight)
    [windowWidth, windowHeight] = computeWindowSize(windowPointTopLeft, windowPointBottomRight)
    moduleConfig = centerOfBrightness.CenterOfBrightness()
    moduleConfig.ModelTag = "centerOfBrightness"
    if windowCenter.all() != 0 and windowWidth != 0 and windowHeight != 0:
        moduleConfig.setWindowCenter(windowCenter)
        moduleConfig.setWindowSize(windowWidth, windowHeight)
    unitTestSim.AddModelToTask(unitTaskName, moduleConfig)

    numberOfPointsBrightnessAverage = 3
    moduleConfig.numberOfPointsBrightnessAverage = numberOfPointsBrightnessAverage
    moduleConfig.filename = imagePath_module
    moduleConfig.blurSize = blur
    moduleConfig.threshold = 50
    moduleConfig.saveDir = path + '/result_save.png'
    if saveTest:
        moduleConfig.saveImages = True

    cob_ref = [input_image.width/2, input_image.height/2]
    if image == "half_half.png":
        # left half black, right half white, and a 1px wide grey stripe in the center with brightness 116/255
        white_width = 138
        grey_width = 1
        height = 183
        if np.array_equal(windowPointTopLeft, [50, 0]) and np.array_equal(windowPointBottomRight, [275, 91]):
            height = 91

        cob_ref = [(3/4 * 1 * white_width + 1/2 * 116/255 * grey_width)/(white_width + grey_width) * input_image.width,
                   int(height/2)*validImage]
        pixelNum_ref = ((white_width+grey_width)*height)*validImage

    inputMessageData = messaging.CameraImageMsgPayload()
    inputMessageData.timeTag = int(1E9)
    inputMessageData.cameraID = 1
    inputMessageData.valid = validImage
    imgInMsg = messaging.CameraImageMsg().write(inputMessageData)
    moduleConfig.imageInMsg.subscribeTo(imgInMsg)
    dataLog = moduleConfig.opnavCOBOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()

    # run simulation for 5 time steps (excluding initial time step at 0 ns), scale brightness each time step
    # necessary to test rolling brightness average
    scaler = np.array([1.0, 0.9, 0.8, 0.7, 0.6])
    brightness_ref = np.zeros([len(scaler)])
    brightnessAverage_ref = np.zeros([len(scaler)])
    for i in range(0, len(scaler)):
        im = cv2.imread(imagePath)
        th, im_th = cv2.threshold(im, int(scaler[i] * 255), 255, cv2.THRESH_TRUNC)
        out_image_path = imagePath_module
        cv2.imwrite(out_image_path, im_th)

        unitTestSim.ConfigureStopTime(i * testProcessRate)
        unitTestSim.ExecuteSimulation()

        # true rolling brightness average
        if image == "half_half.png":
            brightness_raw = int(scaler[i] * 255)*white_width*height + 116*grey_width*height
            brightness_ref[i] = brightness_raw / 255
            lower_idx = max(0, i-(numberOfPointsBrightnessAverage-1))
            brightnessAverage_ref[i] = np.mean(brightness_ref[lower_idx:i+1])

    center = dataLog.centerOfBrightness[-1,:]
    pixelNum = dataLog.pixelsFound[-1]
    brightnessAverage = dataLog.rollingAverageBrightness

    output_image = Image.new("RGB", input_image.size)
    output_image.paste(input_image)
    draw_result = ImageDraw.Draw(output_image)

    if pixelNum > 0:
        data = [center[0], center[1], np.sqrt(pixelNum)/50]
        draw_result.ellipse((data[0] - data[2], data[1] - data[2], data[0] + data[2], data[1] + data[2]),
                            outline=(255, 0, 0, 0))
    if windowCenter.all() != 0 and windowWidth != 0 and windowHeight != 0:
        draw_result.rectangle((windowPointTopLeft[0], windowPointTopLeft[1], windowPointBottomRight[0],
                               windowPointBottomRight[1]), outline=(0, 255, 0, 0))

    input_image.close()

    # Remove saved images for the test of that functionality
    files = glob.glob(path + "/result_*")
    for f in files:
        os.remove(f)
    # Save output image with center of brightness
    if saveImage:
        output_image.save("result_" + image)

    if show_plots:
        output_image.show()

    # Remove temporarily created images
    files = glob.glob(imagePath_module)
    for f in files:
        os.remove(f)

    # make sure module output data is correct
    tolerance = 0.6  # just above half a pixel
    np.testing.assert_allclose(center,
                               cob_ref,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: rhat_COB_N',
                               verbose=True)

    if image == "half_half.png":
        np.testing.assert_allclose(pixelNum,
                                   pixelNum_ref,
                                   rtol=0,
                                   atol=tolerance,
                                   err_msg='Variable: pixelNum',
                                   verbose=True)

        np.testing.assert_allclose(brightnessAverage,
                                   brightnessAverage_ref,
                                   rtol=tolerance,
                                   atol=0,
                                   err_msg='Variable: brightnessAverage',
                                   verbose=True)


if __name__ == "__main__":
    centerOfBrightnessTest(True, "half_half.png", 1, True, True, False, [50, 0], [275, 91])
