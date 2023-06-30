# 
#  ISC License
# 
# Copyright (c) 2023, Laboratory for Atmospheric Space Physics, University of Colorado Boulder
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
#
import copy
import itertools

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import visualOdometry
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from odometry_support import odometry
# parameters
def test_visualOdometryGeneral(show_plots):
    r"""
    **Validation Test Description**
    This test checks the performance of the visual odometry module in a general case with random attitudes and
    pixel offsets between features
    """
    odometryTest(show_plots)

def test_xShift(show_plots):
    r"""
    **Validation Test Description**
    This test checks the direction of the direction of motion given a -x pixel shift
    """
    odometryTest(show_plots, dx = -10, dy = 0,
                 mrpBN1 = [0., 0., 0.], mrpBN2 =  [0., 0., 0.],
                 mrpTN1 = [0., 0., 0.], mrpTN2 = [0., 0., 0.])

def test_yShift(show_plots):
    r"""
    **Validation Test Description**
    This test checks the direction of the direction of motion given a -y pixel shift
    """
    odometryTest(show_plots, dx = 0, dy = -8,
                 mrpBN1 = [0., 0., 0.], mrpBN2 =  [0., 0., 0.],
                 mrpTN1 = [0., 0., 0.], mrpTN2 = [0., 0., 0.])

def test_generalShift(show_plots):
    r"""
    **Validation Test Description**
    This test checks the direction of the direction of motion given a (x,y) pixel shift
    """
    odometryTest(show_plots, dx = 2, dy = 5,
                 mrpBN1 = [0., 0., 0.], mrpBN2 =  [0., 0., 0.],
                 mrpTN1 = [0., 0., 0.], mrpTN2 = [0., 0., 0.])

def test_attitudeShift(show_plots):
    r"""
    **Validation Test Description**
    This test checks that a positive rotation in y_C leads to the perception of a -x direction of motion
    """
    odometryTest(show_plots, dx = 0, dy = 0,
                 mrpBN1 = [0., 0.1, 0.], mrpBN2 =  [0., 0.2, 0.],
                 mrpTN1 = [0., 0., 0.], mrpTN2 = [0., 0., 0.])

def odometryTest(show_plots, dx = 5, dy = -3,
                 mrpBN1 = [0., 0.1, 0.2], mrpBN2 =  [0.1, 0., -0.2],
                 mrpTN1 = [0.2, 0., 0.0], mrpTN2 = [0., -0.1, 0.0]):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = visualOdometry.VisualOdometry()
    module.ModelTag = "directionOfMotion"
    module.errorTolerance = 1E-5
    module.sigma_uv = 1
    unitTestSim.AddModelToTask(unitTaskName, module)

    pixels = [20, 25, 30, 35, 40, 1000, 50, 950, 10, 1250, 80, 1550, 10, 1250, 85, 1250]
    # Configure input messages
    keyPointData = messaging.PairedKeyPointsMsgPayload()
    keyPointData.valid = True
    keyPointData.timeTag_firstImage = 0
    keyPointData.timeTag_secondImage = 5
    keyPointData.keyPoints_firstImage = pixels
    pixelsSecond = []
    for i in range(0, len(pixels) - 1, 2):
        pixelsSecond += [keyPointData.keyPoints_firstImage[i] + dx,
                                              keyPointData.keyPoints_firstImage[i+1] + dy]
    keyPointData.keyPoints_secondImage = pixelsSecond
    keyPointData.sigma_BN_firstImage = mrpBN1
    keyPointData.sigma_BN_secondImage = mrpBN2
    keyPointData.sigma_TN_firstImage = mrpTN1
    keyPointData.sigma_TN_secondImage = mrpTN2
    keyPointData.keyPointsFound = int(len(pixels)/2)
    keyPointMsg = messaging.PairedKeyPointsMsg().write(keyPointData)
    module.keyPointPairInMsg.subscribeTo(keyPointMsg)

    # Configure input messages
    cameraData = messaging.CameraConfigMsgPayload()
    cameraData.cameraID = 1
    cameraData.fieldOfView = 0.3
    cameraData.resolution = [2000, 1000]
    cameraMsg = messaging.CameraConfigMsg().write(cameraData)
    module.cameraConfigInMsg.subscribeTo(cameraMsg)

    # setup output message recorder objects
    dirMotionRec = module.dirOfMotionMsgOutput.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dirMotionRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    v_hat_camera = dirMotionRec.v_C_hat[0]
    covar_camera = np.array(dirMotionRec.covar_C[0]).reshape([3,3])
    cameraID = dirMotionRec.cameraID[0]
    time = dirMotionRec.timeOfDirectionEstimate[0]

    s_test, R_test = odometry(keyPointData, cameraData, module.sigma_uv)

    # If attitudes are stable, the flow of the pixels should go against the direction of motion
    if mrpBN1 == [0.,0.,0.]:
        np.testing.assert_equal(np.sign(s_test[0]),
                                -np.sign(dx),
                                err_msg=('Motion in the corresponding x direction'))
        np.testing.assert_equal(np.sign(s_test[1]),
                                -np.sign(dy),
                                err_msg=('Motion in the corresponding y direction'))
    if mrpBN1 == [0.,0.1,0.] and mrpBN2 == [0.,0.2,0.]:
        np.testing.assert_equal(np.sign(s_test[0]),
                                -1,
                                err_msg=('Motion with attitude slew'))
    np.testing.assert_allclose(v_hat_camera,
                               s_test,
                               rtol = 1E-4,
                               atol = 1E-10,
                               err_msg = ('direction of motion error'),
                               verbose = True)
    np.testing.assert_allclose(covar_camera,
                               R_test,
                               rtol = 1E-2,
                               atol = 1E-10,
                               err_msg=('covariance error'),
                               verbose = True)
    np.testing.assert_equal(time,
                            keyPointData.timeTag_secondImage,
                            err_msg=('Time not written correctly'))
    np.testing.assert_equal(cameraID,
                            cameraData.cameraID,
                            err_msg=('Camera ID not written correctly'))


if __name__ == "__main__":
    odometryTest(True)
