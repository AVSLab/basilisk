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
imageNoise = [0., 0.25, 0.5]

paramArray = [trueAnomalies, cameraPositions, scRotation, imageNoise]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))

@pytest.mark.parametrize("accuracy", [1e-4])
@pytest.mark.parametrize("p1_f, p2_cam, p3_scRot, p4_noise", paramList)

def test_cameraTriangulation(show_plots, p1_f, p2_cam, p3_scRot, p4_noise, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the camera triangulation module works correctly for different spacecraft positions, camera
    positions (w.r.t. the spacecraft), spacecraft orientations, and image noise.

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown
        :param p1_f: spacecraft true anomaly
        :param p2_cam: camera position
        :param p3_scRot: spacecraft principal rotation angle, around a fixed vector
        :param p4_noise: image noise in pixels
        :param accuracy: accuracy of the test

    **Description of Variables Being Tested**

    The content of the CameraLocalizationMsg output message is compared with the true values.
    """
    cameraTriangulationTestFunction(show_plots, p1_f, p2_cam, p3_scRot, p4_noise, accuracy)


def cameraTriangulationTestFunction(show_plots, p1_f, p2_cam, p3_scRot, p4_noise, accuracy):
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
    # image measurement noise
    Ru = p4_noise**2 * np.identity(2)
    S = np.hstack((np.identity(2), np.zeros((2, 1))))
    Kinv = np.linalg.inv(K)
    Rx = Kinv.dot(S.transpose()).dot(Ru).dot(S).dot(Kinv.transpose())

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
    module.uncertaintyImageMeasurement = p4_noise
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
    covariance = np.reshape(cameraLocationOutMsgRec.covariance_N[0], (3, 3))
    valid = cameraLocationOutMsgRec.valid[0]

    # true data
    cameraLocationTrue = r_CN_N
    covarianceTrue = computeTriangulationCovariance(pointCloud, keyPoints, K, dcm_CN, Rx)
    if (pointCloudInMsgData.valid and
    keyPointsInMsgData.valid and
    keyPointsInMsgData.cameraID == cameraConfigInMsgData.cameraID and
    pointCloudInMsgData.timeTag == keyPointsInMsgData.timeTag_secondImage and
    pointCloudInMsgData.numberOfPoints == keyPointsInMsgData.keyPointsFound):
        validTrue = True
    else:
        validTrue = False

    # make sure module output data is correct
    paramsString = ' for true anomaly={}, camera position={}, SC rotation={}, noise={}, accuracy={}'.format(
        str(p1_f),
        str(p2_cam),
        str(p3_scRot),
        str(p4_noise),
        str(accuracy))

    np.testing.assert_allclose(cameraLocation,
                               cameraLocationTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: cameraLocation,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(covariance,
                               covarianceTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: covariance,' + paramsString),
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


def computeTriangulationCovariance(pointCloud, imagePoints, cameraCalibrationMatrix, dcmCamera, Rx):
    Kinv = np.linalg.inv(cameraCalibrationMatrix)
    dcm_CF = dcmCamera
    dcm_FC = dcm_CF.transpose()
    covarianceSumTerm = np.zeros((3, 3))

    H = np.zeros((3*len(pointCloud), 3))
    for i in range(0, len(pointCloud)):
        ui = imagePoints[i, :]
        uiBar = np.hstack((ui, 1))
        xiBar = np.dot(Kinv, uiBar)
        xiBarTilde = np.array(RigidBodyKinematics.v3Tilde(xiBar))
        pi = pointCloud[i, :]
        H[i*3:(i+1)*3, :] = np.dot(xiBarTilde, dcm_CF)

        j = i + 1
        if j > len(pointCloud)-1:
            j = 0

        uj = imagePoints[j, :]
        ujBar = np.hstack((uj, 1))
        pj = pointCloud[j, :]
        dcm_FCj = dcm_FC
        dij = pj - pi
        ai = dcm_FC.dot(Kinv).dot(uiBar)
        aj = dcm_FCj.dot(Kinv).dot(ujBar)
        gamma = np.linalg.norm(np.cross(dij, aj))/np.linalg.norm(np.cross(ai, aj))
        if np.linalg.norm(np.cross(ai, aj)) < 1e-10:
            gamma = 0
        Re = -gamma**2 * xiBarTilde.dot(Rx).dot(xiBarTilde)
        covarianceSumTerm += dcm_FC.dot(xiBarTilde).dot(Re).dot(xiBarTilde).dot(dcm_CF)

    HHinv = np.linalg.inv(np.dot(H.transpose(), H))
    P = -HHinv.dot(covarianceSumTerm).dot(HHinv)

    return P


if __name__ == "__main__":
    test_cameraTriangulation(False, trueAnomalies[0], cameraPositions[0], scRotation[0], 0.25, 1e-4)
