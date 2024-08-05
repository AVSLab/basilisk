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

import copy
import itertools

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import pointCloudTriangulation
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# parameters
numInitialSteps = [0, 2, 3]
cameraPositions = np.array([[1., 3., 4.], [0., 0., 0.]])
scRotation = [0., 70.]

paramArray = [numInitialSteps, cameraPositions, scRotation]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))

@pytest.mark.parametrize("accuracy", [1e-4])
@pytest.mark.parametrize("p1_n, p2_cam, p3_scRot", paramList)

def test_pointCloudTriangulation(show_plots, p1_n, p2_cam, p3_scRot, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the point cloud triangulation module works correctly for different camera
    positions (w.r.t. the spacecraft) and spacecraft orientations.

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown
        :param p1_n: number of initial steps
        :param p2_cam: camera position
        :param p3_scRot: spacecraft principal rotation angle, around a fixed vector
        :param accuracy: accuracy of the test

    **Description of Variables Being Tested**

    The content of the PointCloudMsg output message is compared with the true values.
    """
    pointCloudTriangulationTestFunction(show_plots, p1_n, p2_cam, p3_scRot, accuracy)


def pointCloudTriangulationTestFunction(show_plots, p1_n, p2_cam, p3_scRot, accuracy):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(10)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = pointCloudTriangulation.PointCloudTriangulation()
    module.ModelTag = "pointCloudTriangulation"
    module.numberTimeStepsInitialPhase = p1_n
    unitTestSim.AddModelToTask(unitTaskName, module)

    # set up the transfer orbit using classical orbit elements
    mu = 3.986004418e14
    oe = orbitalMotion.ClassicElements()
    r = 10000. * 1.e3  # meters
    oe.a = r
    oe.e = 0.01
    oe.i = 5. * macros.D2R
    oe.Omega = 25. * macros.D2R
    oe.omega = 30. * macros.D2R
    oe.f = 40.

    time = macros.sec2nano(0)
    dt = testProcessRate * macros.NANO2SEC
    ID = 0
    fov = 20. * macros.D2R
    res = np.array([1000, 800])
    r_CB_B = p2_cam
    dcm_CB = np.array([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]])  # make sure camera z-axis points out of image plane
    rotAxis = np.array([1., 0.3, 0.5])
    omega_BN_hat = rotAxis/np.linalg.norm(rotAxis)
    omega_BN_mag = 0.01
    omega_BN = omega_BN_mag * omega_BN_hat
    numberOfPoints = 20

    # generate point cloud
    pointCloudReference = createCircularPointCloud(1000. * 1.e3, numberOfPoints)
    # reshape point cloud from Nx3 to 1x3*N stacked vector
    pointCloudReferenceStacked = np.reshape(np.array(pointCloudReference), 3*numberOfPoints)

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

    sigma_CB = RigidBodyKinematics.C2MRP(dcm_CB)

    # Configure input messages
    ephemerisInMsgData = messaging.EphemerisMsgPayload()
    ephemerisInMsg = messaging.EphemerisMsg().write(ephemerisInMsgData)

    navTransInMsgData = messaging.NavTransMsgPayload()
    navTransInMsg = messaging.NavTransMsg().write(navTransInMsgData)

    directionOfMotionInMsgData = messaging.DirectionOfMotionMsgPayload()
    directionOfMotionInMsg = messaging.DirectionOfMotionMsg().write(directionOfMotionInMsgData)

    keyPointsInMsgData = messaging.PairedKeyPointsMsgPayload()
    keyPointsInMsg = messaging.PairedKeyPointsMsg().write(keyPointsInMsgData)

    cameraConfigInMsgData = messaging.CameraConfigMsgPayload()
    cameraConfigInMsg = messaging.CameraConfigMsg().write(cameraConfigInMsgData)

    # subscribe input messages to module
    module.ephemerisInMsg.subscribeTo(ephemerisInMsg)
    module.navTransInMsg.subscribeTo(navTransInMsg)
    module.directionOfMotionInMsg.subscribeTo(directionOfMotionInMsg)
    module.keyPointsInMsg.subscribeTo(keyPointsInMsg)
    module.cameraConfigInMsg.subscribeTo(cameraConfigInMsg)

    # setup output message recorder objects
    pointCloudOutMsgRec = module.pointCloudOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, pointCloudOutMsgRec)

    unitTestSim.InitializeSimulation()

    # run simulation for several time steps
    numTimeSteps = p1_n + 2
    pointCloud = np.zeros([numTimeSteps, 3*numberOfPoints])
    for i in range(0, numTimeSteps):
        time1 = time + macros.sec2nano(dt*(i))
        time2 = time + macros.sec2nano(dt*(i+1))

        # SC state at time1
        oeNew = copy.deepcopy(oe)
        M0 = orbitalMotion.E2M(orbitalMotion.f2E(oe.f, oeNew.e), oeNew.e)
        n = np.sqrt(mu / (oeNew.a) ** 3)
        M1 = M0 + n*(time1-time)*macros.NANO2SEC
        oeNew.f = orbitalMotion.E2f(orbitalMotion.M2E(M1, oeNew.e), oeNew.e)
        r_BN_N, v_BN_N = orbitalMotion.elem2rv(mu, oeNew)

        prv_B1N = (p3_scRot + omega_BN_mag*dt*(i)) * macros.D2R * rotAxis/np.linalg.norm(rotAxis)
        prv_B2N = (p3_scRot + omega_BN_mag*dt*(i+1)) * macros.D2R * rotAxis/np.linalg.norm(rotAxis)
        sigma_B1N = RigidBodyKinematics.PRV2MRP(prv_B1N)
        sigma_B2N = RigidBodyKinematics.PRV2MRP(prv_B2N)

        dcm_B1N = RigidBodyKinematics.MRP2C(sigma_B1N)
        dcm_C1N = np.dot(dcm_CB, dcm_B1N)
        dcm_B2N = RigidBodyKinematics.MRP2C(sigma_B2N)
        dcm_C2N = np.dot(dcm_CB, dcm_B2N)

        # First position vector follows orbital motion such that velocity also changes over time.
        # Second position vector is obtained using linear motion since this is assumed inside the module
        r_C1B_N = np.dot(dcm_B1N, r_CB_B)
        r_C1N_N = r_BN_N + r_C1B_N
        v_C1N_N = v_BN_N + np.cross(omega_BN, r_C1B_N)
        vScale = np.linalg.norm(v_C1N_N)
        v_N_hat = v_C1N_N / np.linalg.norm(v_C1N_N)
        v_C_hat = np.dot(dcm_C1N, v_N_hat)
        r_C2N_N = r_C1N_N + vScale * dt * v_N_hat  # use linear motion for unit test

        # generate images (keyPoints in pixel space)
        keyPoints1 = createImages(pointCloudReference, r_C1N_N, dcm_C1N, K)
        keyPoints2 = createImages(pointCloudReference, r_C2N_N, dcm_C2N, K)
        # reshape key points from Nx2 to 1x2*N stacked vector
        keyPoints1Stacked = np.reshape(np.array(keyPoints1), 2*numberOfPoints)
        keyPoints2Stacked = np.reshape(np.array(keyPoints2), 2*numberOfPoints)

        # Update input messages

        # For first p1_n time steps, ephemeris message should be used.
        # For all subsequent time steps, navigation message should be used.
        # Thus, set the message that shouldn't be used to zero (would cause unit test to fail if that message was
        # accidentally used by the module
        if i < p1_n:
            ephemerisInMsgData.v_BdyZero_N = v_C1N_N  # use v_C1_N instead of v_BN_N for accurate unit test
            ephemerisInMsgData.timeTag = time1 * macros.NANO2SEC
            ephemerisInMsg.write(ephemerisInMsgData, unitTestSim.TotalSim.getCurrentNanos())

            navTransInMsgData.v_BN_N = np.array([0., 0., 0.])
            navTransInMsgData.timeTag = 0.
            navTransInMsg.write(navTransInMsgData, unitTestSim.TotalSim.getCurrentNanos())
        else:
            ephemerisInMsgData.v_BdyZero_N = np.array([0., 0., 0.])
            ephemerisInMsgData.timeTag = 0.
            ephemerisInMsg.write(ephemerisInMsgData, unitTestSim.TotalSim.getCurrentNanos())

            navTransInMsgData.v_BN_N = v_C1N_N  # use v_C1_N instead of v_BN_N for accurate unit test
            navTransInMsgData.timeTag = time1 * macros.NANO2SEC
            navTransInMsg.write(navTransInMsgData, unitTestSim.TotalSim.getCurrentNanos())

        directionOfMotionInMsgData.valid = True
        directionOfMotionInMsgData.cameraID = ID
        directionOfMotionInMsgData.timeOfDirectionEstimate = time2
        directionOfMotionInMsgData.v_C_hat = v_C_hat
        directionOfMotionInMsg.write(directionOfMotionInMsgData, unitTestSim.TotalSim.getCurrentNanos())

        keyPointsInMsgData.valid = True
        keyPointsInMsgData.cameraID = ID
        keyPointsInMsgData.timeTag_firstImage = time1
        keyPointsInMsgData.keyPoints_firstImage = keyPoints1Stacked
        keyPointsInMsgData.sigma_BN_firstImage = sigma_B1N
        keyPointsInMsgData.timeTag_secondImage = time2
        keyPointsInMsgData.keyPoints_secondImage = keyPoints2Stacked
        keyPointsInMsgData.sigma_BN_secondImage = sigma_B2N
        keyPointsInMsgData.keyPointsFound = numberOfPoints
        keyPointsInMsg.write(keyPointsInMsgData, unitTestSim.TotalSim.getCurrentNanos())

        cameraConfigInMsgData.cameraID = ID
        cameraConfigInMsgData.fieldOfView = fov
        cameraConfigInMsgData.resolution = res
        cameraConfigInMsgData.cameraPos_B = r_CB_B
        cameraConfigInMsgData.sigma_CB = sigma_CB
        cameraConfigInMsg.write(cameraConfigInMsgData, unitTestSim.TotalSim.getCurrentNanos())

        unitTestSim.ConfigureStopTime(i * testProcessRate)
        unitTestSim.ExecuteSimulation()

        # pull module data
        pointCloudMeasuredStacked = pointCloudOutMsgRec.points[i]
        pointCloudMeasuredSize = pointCloudOutMsgRec.numberOfPoints[i]
        pointCloudMeasured = np.reshape(np.array(pointCloudMeasuredStacked[0:3*pointCloudMeasuredSize]), (pointCloudMeasuredSize, 3))

        # convert point cloud: module output is w.r.t. first camera location expressed in camera frame
        # convert to be w.r.t. inertial frame, expressed in inertial frame
        pointCloudConverted = convertPointCloud(pointCloudMeasured, dcm_C1N, r_C1N_N)
        pointCloudConvertedStacked = np.reshape(np.array(pointCloudConverted), 3*numberOfPoints)

        pointCloud[i, :] = pointCloudConvertedStacked

    # true data
    pointCloudTrue = np.tile(pointCloudReferenceStacked, [numTimeSteps, 1])

    # make sure module output data is correct
    paramsString = ' for number initial steps={}, camera position={}, SC rotation={}, accuracy={}'.format(
        str(p1_n),
        str(p2_cam),
        str(p3_scRot),
        str(accuracy))

    np.testing.assert_allclose(pointCloud,
                               pointCloudTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: point cloud,' + paramsString),
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


def convertPointCloud(pointCloudIn, dcm_CN, r_CN_N):
    pointCloudOut = []
    for r_PC_C in pointCloudIn:
        r_PC_N = np.dot(dcm_CN.transpose(), r_PC_C)
        r_PN_N = r_PC_N + r_CN_N
        pointCloudOut.append(r_PN_N)

    return np.array(pointCloudOut)


if __name__ == "__main__":
    test_pointCloudTriangulation(False, numInitialSteps[0], cameraPositions[1], scRotation[0], 1e-4)
