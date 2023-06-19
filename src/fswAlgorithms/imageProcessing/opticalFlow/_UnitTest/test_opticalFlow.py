# ISC License
#
# Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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

import inspect
import os

import numpy as np
import numpy.testing
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.architecture import messaging

importErr = False
reasonErr = ""
try:
    from Basilisk.fswAlgorithms import opticalFlow
except ImportError:
    importErr = True
    reasonErr = "Optical Flow not built---check OpenCV option"

PIL = pytest.importorskip("PIL")

@pytest.mark.skipif(importErr, reason= reasonErr)
@pytest.mark.parametrize("image, sigma_BN, maxFeatures, searchSize, maskSize, slidingWindow, saveImage", [
    ("itokawa",  [1., 0.3, 0.1], 200, 10, 20, False, False), #Itokawa image
    ("itokawa",  [0., 0.5, 0.8], 100, 20, 10, False, False), #Itokawa image
    ("ryugu",  [0., 0.5, 0.8], 100, 20, 50, False, False), #Itokawa image
    ("mars",  [0., 0., 0.], 250, 20, 20, False, False), #Mars image
    ])

# update "module" in this function name to reflect the module name
def test_module(show_plots, image, sigma_BN, maxFeatures, searchSize, maskSize, slidingWindow, saveImage):
    """
    Unit test for Optical flow. The unit test specifically runs on pairs of images:

        1. Two images of Mars taking up half of the field of view
        2. Two images of Itokawa horizontally rotating

    In the tests, the spacecraft attitude message is varied, as well as the maximum number of features that can
    be detected, as well as the search window size for the algorithm, and the size of the limb masking.

    The pair of Itokawa images is credited to 3DShoot (https://www.flickr.com/photos/3dshoot/8478564535)
    and added under the Attribution-NonCommercial-ShareAlike 2.0 Generic (CC BY-NC-SA 2.0) license:
    https://creativecommons.org/licenses/by-nc-sa/2.0/

    """
    # each test method requires a single assert method to be called
    opticalFlowTest(show_plots, image, sigma_BN, maxFeatures, searchSize, maskSize, slidingWindow, saveImage)
    return

def opticalFlowTest(show_plots, image, sigma_BN, maxFeatures, searchSize, maskSize, slidingWindow, saveImage):
    # Truth values from python
    imagePath1 = path + '/' + image + "-0.jpg"
    imagePath2 = path + '/' + image + "-60.jpg"
    input_image1 = PIL.Image.open(imagePath1)
    input_image2 = PIL.Image.open(imagePath2)
    input_image1.load()
    input_image2.load()

    #################################################
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # # Create test thread
    dt = 1
    testProcessRate = macros.sec2nano(dt)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = opticalFlow.OpticalFlow()
    moduleConfig.ModelTag = "opticalFlow"
    moduleConfig.minTimeBetweenPairs = 5
    moduleConfig.slidingWindowImages = slidingWindow

    # OpenCV specific arguments needed for Features */
    moduleConfig.maxNumberFeatures = maxFeatures
    moduleConfig.qualityLevel = 0.1
    moduleConfig.minumumFeatureDistance = 5
    moduleConfig.blockSize = searchSize

    # OpenCV specific arguments needed for OpticalFlow */
    moduleConfig.criteriaMaxCount = 10
    moduleConfig.criteriaEpsilon = 0.01
    moduleConfig.flowSearchSize = searchSize
    moduleConfig.flowMaxLevel = 2

    # OpenCV specific arguments needed for masking */
    moduleConfig.thresholdMask = maskSize
    moduleConfig.limbMask = maskSize

    # Create the input messages.
    inputImage = messaging.CameraImageMsgPayload()
    inputAtt = messaging.NavAttMsgPayload()
    inputEphem = messaging.EphemerisMsgPayload()

    # Create input message and size it because the regular creator of that message
    # is not part of the test.
    inputImage.timeTag = int(1E9)
    inputImage.cameraID = 1
    imageInMsg = messaging.CameraImageMsg().write(inputImage)
    moduleConfig.imageInMsg.subscribeTo(imageInMsg)

    # Set body attitude relative to inertial
    inputAtt.sigma_BN = sigma_BN
    attInMsg = messaging.NavAttMsg().write(inputAtt)
    moduleConfig.attitudeMsg.subscribeTo(attInMsg)

    # Set target attitude relative to inertial
    inputEphem.sigma_BN = (-np.array(sigma_BN)).tolist()
    ephemInMsg = messaging.EphemerisMsg().write(inputEphem)
    moduleConfig.ephemerisMsg.subscribeTo(ephemInMsg)

    dataLog = moduleConfig.keyPointsMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    unitTestSim.AddModelToTask(unitTaskName, moduleConfig)

    t1 = 100
    deltaT = 60
    attitudeUpdate = np.array(sigma_BN)
    moduleConfig.filename = ""
    attitudes = []

    unitTestSim.InitializeSimulation()
    for i in range(0, t1, 5):
        if i % deltaT == 5:
            moduleConfig.filename = path + '/' + image + "-" + str(i-5) + ".jpg"
            attitudeUpdate += np.random.normal(0, 1E-2, 3)
            inputAtt.sigma_BN = attitudeUpdate.tolist()
            inputEphem.sigma_BN = (-attitudeUpdate).tolist()
            attitudes.append(attitudeUpdate.tolist())
            attInMsg.write(inputAtt, unitTestSim.TotalSim.CurrentNanos)
            ephemInMsg.write(inputEphem, unitTestSim.TotalSim.CurrentNanos)
        unitTestSim.ConfigureStopTime(macros.sec2nano(i * dt))
        unitTestSim.ExecuteSimulation()

    deltaT += 2
    timeOld = dataLog.timeTag_firstImage[deltaT]
    timeNew = dataLog.timeTag_secondImage[deltaT]
    numPoints = int(dataLog.keyPointsFound[deltaT])
    pointsOld = dataLog.keyPoints_firstImage
    pointsNew = dataLog.keyPoints_secondImage
    oldAtt = dataLog.sigma_BN_firstImage[deltaT]
    newAtt = dataLog.sigma_BN_secondImage[deltaT]
    oldEphem = dataLog.sigma_TN_firstImage[deltaT]
    newEphem = dataLog.sigma_TN_secondImage[deltaT]

    newFeatures = pointsNew[deltaT][:2*numPoints].reshape([numPoints, 2])
    oldFeatures = pointsOld[deltaT][:2*numPoints].reshape([numPoints, 2])

    # Draw the features tracked on the second input image:
    output_image = PIL.Image.new("RGB", input_image2.size)
    output_image.paste(input_image2)
    from PIL import ImageDraw
    draw_result = ImageDraw.Draw(output_image)
    avg = np.zeros(2)
    for i in range(numPoints):
        draw_result.line((oldFeatures[i,:].tolist() + newFeatures[i,:].tolist()), fill=128)
        avg += np.array(oldFeatures[i,:].tolist()) - np.array(newFeatures[i,:].tolist())
    avg /= numPoints

    #Save output image
    if saveImage:
        output_image.save(path + '/' + "result_"+ image + "-" + str(maxFeatures) + ".jpg")
        output_image.show()
    if show_plots:
        output_image.show()

    np.testing.assert_allclose(np.array(newAtt),
                                  attitudes[1],
                                  atol=1e-10,
                                  err_msg='Attitude Msg Error',
                                  verbose=True)

    np.testing.assert_allclose(np.array(newEphem),
                               -np.array(attitudes[1]),
                               atol=1e-10,
                               err_msg='New Ephemeris Attitude Msg Error',
                               verbose=True)

    np.testing.assert_allclose(timeOld,
                                  1E9,
                                  atol=1e-10,
                                  err_msg='Old Time Tag Error',
                                  verbose=True)

    np.testing.assert_allclose(timeNew,
                                  (deltaT - 1)*1E9,
                                  atol=1e-10,
                                  err_msg='New Time Tag Error',
                                  verbose=True)

    np.testing.assert_allclose(np.array(oldAtt),
                                  np.array(attitudes[0]),
                                  atol=1e-10,
                                  err_msg='New Attitude Msg Error',
                                  verbose=True)

    np.testing.assert_allclose(np.array(oldEphem),
                               -np.array(attitudes[0]),
                               atol=1e-10,
                               err_msg='Old Ephemeris Attitude Msg Error',
                               verbose=True)

    np.testing.assert_array_less(np.linalg.norm(avg[1]/avg[0]),
                                  1,
                                  err_msg='Flow in wrong direction',
                                  verbose=True)

    return

# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
if __name__ == "__main__":
    opticalFlowTest(True, "ryugu", [0., 0., 0.],  1000, 30, 50, False, True) # Mars images
