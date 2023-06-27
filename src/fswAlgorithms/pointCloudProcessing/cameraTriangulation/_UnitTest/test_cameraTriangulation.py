# 
#  ISC License
# 
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import itertools

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import cameraTriangulation
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# parameters
trueAnomalies = [0., 10., 350.]
cameraPositions = np.array([[1., 3., 4.], [0., 0., 0.]])
scRotation = [0., 5., 70.]

paramArray = [trueAnomalies, cameraPositions, scRotation]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))

@pytest.mark.parametrize("accuracy", [1e-4])
@pytest.mark.parametrize("p1_f, p2_cam, p3_scRot", paramList)

def test_cameraTriangulation(show_plots, p1_f, p2_cam, p3_scRot, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the camera triangulation module works correctly for different spacecraft positions, camera
    positions (w.r.t. the spacecraft) and spacecraft orientations.

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown
        :param p1_f: spacecraft true anomaly
        :param p2_cam: camera position
        :param p3_scRot: spacecraft principal rotation angle, around a fixed vector
        :param accuracy: accuracy of the test

    **Description of Variables Being Tested**

    The content of the CameraLocalizationMsg output message is compared with the true values.
    """
    cameraTriangulationTestFunction(show_plots, p1_f, p2_cam, p3_scRot, accuracy)


def cameraTriangulationTestFunction(show_plots, p1_f, p2_cam, p3_scRot, accuracy):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(100)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # set up the transfer orbit using classical orbit elements
    mu = 3.986004418e14
    oe = orbitalMotion.ClassicElements()
    r = 10000. * 1.e3  # meters
    oe.a = r
    oe.e = 0.01
    oe.i = 5. * macros.D2R
    oe.Omega = 25. * macros.D2R
    oe.omega = 30. * macros.D2R
    oe.f = p1_f * macros.D2R
    r_BN_N, v_BN_N = orbitalMotion.elem2rv(mu, oe)

    time = macros.sec2nano(500)
    ID = 0
    fov = 20. * macros.D2R
    res = np.array([1000, 800])
    r_CB_B = p2_cam
    dcm_CB = np.array([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]])  # make sure camera z-axis points out of image plane
    rotAxis = np.array([1., 0.3, 0.5])
    prv_BN = p3_scRot * macros.D2R * rotAxis/np.linalg.norm(rotAxis)
    sigma_BN = RigidBodyKinematics.PRV2MRP(prv_BN)
    numberOfPoints = 100

    sigma_CB = RigidBodyKinematics.C2MRP(dcm_CB)
    dcm_BN = RigidBodyKinematics.MRP2C(sigma_BN)
    dcm_CN = np.dot(dcm_CB, dcm_BN)

    r_CN_N = r_BN_N + np.dot(dcm_BN, r_CB_B)

    # camera parameters
    alpha = 0.
    resX = res[0]
    resY = res[1]
    pX = 2.*np.tan(fov*resX/resY/2.0)
    pY = 2.*np.tan(fov/2.0)
    dX = resX/pX
    dY = resY/pY
    up = resX/2.
    vp = resY/2.
    # build camera calibration matrix (not the inverse of it)
    K = np.array([[dX, alpha, up], [0., dY, vp], [0., 0., 1.]])

    # generate point cloud
    pointCloud = createCircularPointCloud(1000. * 1.e3, numberOfPoints)
    # reshape point cloud from Nx3 to 1x3*N stacked vector
    pointCloudStacked = np.reshape(np.array(pointCloud), 3*numberOfPoints)

    # generate images (keyPoints in pixel space)
    keyPoints = createImages(pointCloud, r_CN_N, dcm_CN, K)
    # reshape key points from Nx2 to 1x2*N stacked vector
    keyPointsStacked = np.reshape(np.array(keyPoints), 2*numberOfPoints)

    # setup module to be tested
    module = cameraTriangulation.CameraTriangulation()
    module.ModelTag = "cameraTriangulation"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure input messages
    pointCloudInMsgData = messaging.PointCloudMsgPayload()
    pointCloudInMsgData.timeTag = time
    pointCloudInMsgData.valid = True
    pointCloudInMsgData.numberOfPoints = numberOfPoints
    pointCloudInMsgData.points = pointCloudStacked
    pointCloudInMsg = messaging.PointCloudMsg().write(pointCloudInMsgData)

    keyPointsInMsgData = messaging.PairedKeyPointsMsgPayload()
    keyPointsInMsgData.valid = True
    keyPointsInMsgData.cameraID = ID
    keyPointsInMsgData.timeTag_secondImage = time
    keyPointsInMsgData.keyPoints_secondImage = keyPointsStacked
    keyPointsInMsgData.sigma_BN_secondImage = sigma_BN
    keyPointsInMsgData.keyPointsFound = numberOfPoints
    keyPointsInMsg = messaging.PairedKeyPointsMsg().write(keyPointsInMsgData)

    cameraConfigInMsgData = messaging.CameraConfigMsgPayload()
    cameraConfigInMsgData.cameraID = ID
    cameraConfigInMsgData.fieldOfView = fov
    cameraConfigInMsgData.resolution = res
    cameraConfigInMsgData.cameraPos_B = r_CB_B
    cameraConfigInMsgData.sigma_CB = sigma_CB
    cameraConfigInMsg = messaging.CameraConfigMsg().write(cameraConfigInMsgData)

    # subscribe input messages to module
    module.pointCloudInMsg.subscribeTo(pointCloudInMsg)
    module.keyPointsInMsg.subscribeTo(keyPointsInMsg)
    module.cameraConfigInMsg.subscribeTo(cameraConfigInMsg)

    # setup output message recorder objects
    cameraLocationOutMsgRec = module.cameraLocationOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, cameraLocationOutMsgRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    cameraLocation = cameraLocationOutMsgRec.cameraPos_N[0]
    valid = cameraLocationOutMsgRec.valid[0]

    # true data
    cameraLocationTrue = r_CN_N
    if (pointCloudInMsgData.valid and
    keyPointsInMsgData.valid and
    keyPointsInMsgData.cameraID == cameraConfigInMsgData.cameraID and
    pointCloudInMsgData.timeTag == keyPointsInMsgData.timeTag_secondImage and
    pointCloudInMsgData.numberOfPoints == keyPointsInMsgData.keyPointsFound):
        validTrue = True
    else:
        validTrue = False

    # make sure module output data is correct
    paramsString = ' for true anomaly={}, camera position={}, SC rotation={}, accuracy={}'.format(
        str(p1_f),
        str(p2_cam),
        str(p3_scRot),
        str(accuracy))

    np.testing.assert_allclose(cameraLocation,
                               cameraLocationTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: cameraLocation,' + paramsString),
                               verbose=True)

    np.testing.assert_equal(valid,
                            validTrue,
                            err_msg=('Variable: valid flag,' + paramsString),
                            verbose=True)


def createCircularPointCloud(radius, numberOfPoints):
    phi = np.linspace(0., 2*np.pi, numberOfPoints)

    # generate circular point cloud
    points = []
    for angle in phi:
        rPoint_N = np.array([0., radius*np.cos(angle), radius*np.sin(angle)]).transpose()
        points.append(rPoint_N)

    return np.array(points)


def createImages(pointCloud, cameraLocation, dcm_CN, cameraCalibrationMatrix):
    r_CN_N = cameraLocation
    K = cameraCalibrationMatrix
    images = []
    for r_PN_N in pointCloud:
        r_PC_N = r_PN_N - r_CN_N
        r_PC_C = np.dot(dcm_CN, r_PC_N)
        xBar = r_PC_C/r_PC_C[2]
        uBar = np.dot(K, xBar)
        u = uBar[0:2]
        images.append(u)

    return np.array(images)


if __name__ == "__main__":
    test_cameraTriangulation(False, trueAnomalies[0], cameraPositions[0], scRotation[0], 1e-4)
