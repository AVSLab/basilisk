#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import inspect
import numpy as np
import os
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import cobConverter
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

'''
Main method that is writen in the module to map the state from pixel space to position
'''
def mapState(state, inputCamera):
    # Compute pixel pitch divided by focal length (often denoted d_x and d_y)
    sensorWidth_focal_ratio = 2.*np.tan(inputCamera.fieldOfView*inputCamera.resolution[0]/inputCamera.resolution[1]/2.0)
    sensorHeight_focal_ratio = 2.*np.tan(inputCamera.fieldOfView/2.0)
    d_x = sensorWidth_focal_ratio/inputCamera.resolution[0]
    d_y = sensorHeight_focal_ratio/inputCamera.resolution[1]

    rHat_BN_C = np.zeros(3)
    rHat_BN_C[0] = (state[0] - inputCamera.resolution[0]/2 + 0.5)*d_x
    rHat_BN_C[1] = (state[1] - inputCamera.resolution[1]/2 + 0.5)*d_y
    rHat_BN_C[2] = 1.0

    return rHat_BN_C

'''
Secondary method to map the covariance in pixel space to position
'''
def mapCovar(pixels, inputCamera):
    if pixels >0:
        scaleFactor = np.sqrt(pixels)/6.28
    else:
        scaleFactor = 1
    # Compute pixel pitch divided by focal length (often denoted d_x and d_y)
    sensorWidth_focal_ratio = 2.*np.tan(inputCamera.fieldOfView*inputCamera.resolution[0]/inputCamera.resolution[1]/2.0)
    sensorHeight_focal_ratio = 2.*np.tan(inputCamera.fieldOfView/2.0)
    d_x = sensorWidth_focal_ratio/inputCamera.resolution[0]
    d_y = sensorHeight_focal_ratio/inputCamera.resolution[1]

    covar = np.zeros([3,3])
    covar[0,0] = d_x**2
    covar[1,1] = d_y**2
    covar[2,2] = 1

    return 1/scaleFactor*covar

@pytest.mark.parametrize("cameraResolution, sigma_CB, sigma_BN, centerOfBrightness, numberOfPixels",
    [([512, 512], [1., 0.3, 0.1], [0.6, 1., 0.1], [152, 251], 75)
    ,([128, 345], [0, 0, 0], [0, 0, 0], [251, 120], 100)
    ,([742, 512], [-1., -0.3, -0.1], [-0.6, -1., -0.1], [0, 0], 0)
    ,([2048, 2048], [-1., -0.3, -0.1], [-0.6, -1., -0.1], [1021, 1891], 1000)
    ])

def test_cob_converter(show_plots, cameraResolution, sigma_CB, sigma_BN, centerOfBrightness, numberOfPixels):
    [testResults, testMessage] = cobConverterTestFunction(show_plots,
                                                          cameraResolution,
                                                          sigma_CB,
                                                          sigma_BN,
                                                          centerOfBrightness,
                                                          numberOfPixels)
    assert testResults < 1, testMessage

""" Test the center of brightness converter module. """
def cobConverterTestFunction(show_plots, cameraResolution, sigma_CB, sigma_BN, centerOfBrightness, numberOfPixels):
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))
    module = cobConverter.CobConverter()
    unitTestSim.AddModelToTask(unitTaskName, module, module)

    # Create the input messages.
    inputCamera = messaging.CameraConfigMsgPayload()
    inputCob = messaging.OpNavCOBMsgPayload()
    inputAtt = messaging.NavAttMsgPayload()

    # Set camera parameters
    inputCamera.fieldOfView = 2.0 * np.arctan(10*1e-3 / 2.0 / (1.*1e-3) )  # 2*arctan(size/2 / focal)
    inputCamera.resolution = cameraResolution
    inputCamera.sigma_CB = sigma_CB
    camInMsg = messaging.CameraConfigMsg().write(inputCamera)
    module.cameraConfigInMsg.subscribeTo(camInMsg)

    # Set center of brightness
    inputCob.centerOfBrightness = centerOfBrightness
    inputCob.pixelsFound = numberOfPixels
    inputCob.timeTag = 12345
    cobInMsg = messaging.OpNavCOBMsg().write(inputCob)
    module.opnavCOBInMsg.subscribeTo(cobInMsg)

    # Set body attitude relative to inertial
    inputAtt.sigma_BN = sigma_BN
    attInMsg = messaging.NavAttMsg().write(inputAtt)
    module.navAttInMsg.subscribeTo(attInMsg)

    dataLog = module.opnavUnitVecOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    # Truth Values
    state = [inputCob.centerOfBrightness[0], inputCob.centerOfBrightness[1]]
    pixels = inputCob.pixelsFound

    r_Cexp = mapState(state, inputCamera)
    covar_Cexp = mapCovar(pixels, inputCamera)

    dcm_CB = rbk.MRP2C(inputCamera.sigma_CB)
    dcm_BN = rbk.MRP2C(inputAtt.sigma_BN)
    dcm_NC = np.dot(dcm_CB, dcm_BN).T

    r_Nexp = np.dot(dcm_NC, r_Cexp)
    covar_Nexp = np.dot(dcm_NC, np.dot(covar_Cexp, dcm_NC.T)).flatten()
    timTagExp = inputCob.timeTag

    outputR = dataLog.rhat_BN_N
    outputCovar = dataLog.covar_N
    outputTime = dataLog.timeTag

    tollerance = 1e-10
    for i in range(len(outputR[-1, 1:])):
        if np.abs(r_Nexp[i] - outputR[-1, i]) > tollerance and np.isnan(outputR.any()):
            testFailCount += 1
            testMessages.append("Position Check in cobConvert")

    for i in range(len(outputCovar[-1, 0:])):
        if np.abs((covar_Nexp[i] - outputCovar[-1, i])) > tollerance and np.isnan(outputTime.any()):
            testFailCount += 1
            testMessages.append("Covar Check in cobConvert")

    if np.abs((timTagExp - outputTime[-1])/timTagExp) > tollerance and np.isnan(outputTime.any()):
        testFailCount += 1
        testMessages.append("Time Check in cobConvert")

    return [testFailCount, ''.join(testMessages)]


if __name__ == '__main__':
    test_cob_converter(False, [512, 512], [1., 0.3, 0.1], [0.6, 1., 0.1], [152, 251], 75)
