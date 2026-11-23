#
#   Unit Test Script
#   Module Name:        pixelLineConverter.py
#   Creation Date:      May 16, 2019
#

import inspect
import os

import numpy as np
import pytest
from Basilisk.architecture import bskLogging, messaging
from Basilisk.fswAlgorithms import pixelLineConverter
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
)
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def mapState(state, planet, camera):
    D = planet["diameter"]

    pX = 2. * np.tan(camera.fieldOfView * camera.resolution[0] / camera.resolution[1] / 2.0)
    pY = 2. * np.tan(camera.fieldOfView/2.0)
    d_x = pX/camera.resolution[0]
    d_y = pY/camera.resolution[1]

    A = 2 * np.arctan(state[2]*d_x)

    norm = 0.5 * D/np.sin(0.5*A)
    center = (np.asarray(camera.resolution) - 1) / 2
    vec = np.array([(state[0] - center[0]) * d_x, (state[1] - center[1]) * d_y, 1.0])
    # The image ray points from the camera to Mars. The reported position points back to the spacecraft.
    return -norm * vec / np.linalg.norm(vec)


def mapCovar(CovarXYR, rho, planet, camera):
    D = planet["diameter"]

    pX = 2. * np.tan(camera.fieldOfView * camera.resolution[0] / camera.resolution[1] / 2.0)
    pY = 2. * np.tan(camera.fieldOfView/2.0)
    d_x = pX / camera.resolution[0]
    d_y = pY / camera.resolution[1]

    A = 2 * np.arctan(rho*d_x)

    # rho_map = (0.33 * D * np.cos(A)/np.sin(A/2.)**2. * 2./f * 1./(1. + (rho/f)**2.) * (d_x/f) )
    rho_map = 0.5*D*(-np.sqrt(1 + rho**2*d_x**2)/(rho**2*d_x) + d_x/(np.sqrt(1 + rho**2*d_x**2)))
    x_map =   0.5 * D/np.sin(0.5*A)*(d_x)
    y_map =  0.5 * D/np.sin(0.5*A)*(d_y)
    CovarMap = np.array([[x_map,0.,0.],[0., y_map, 0.],[0.,0., rho_map]])
    CoarIn = np.array(CovarXYR).reshape([3,3])
    return np.dot(CovarMap, np.dot(CoarIn, CovarMap.T))

def test_pixelLine_converter():
    """Verify inertial position, covariance, and measurement time for a valid Mars detection."""
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # Add a new task to the process

    # Construct the pixel-line converter
    # Set the names for the input messages
    pixelLine = pixelLineConverter.pixelLineConverter()

    # This calls the algContain to setup the selfInit, update, and reset
    pixelLine.ModelTag = "pixelLineConverter"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, pixelLine)

    # Create the input messages.
    inputCamera = messaging.CameraConfigMsgPayload()
    inputCircles = messaging.OpNavCirclesMsgPayload()
    inputAtt = messaging.NavAttMsgPayload()

    # Set camera
    inputCamera.fieldOfView = 2.0 * np.arctan(10*1e-3 / 2.0 / (1.*1e-3) )  # 2*arctan(s/2 / f)
    inputCamera.resolution = [512, 512]
    inputCamera.sigma_CB = [1., 0.3, 0.1]
    camInMsg = messaging.CameraConfigMsg().write(inputCamera)
    pixelLine.cameraConfigInMsg.subscribeTo(camInMsg)

    # Set circles
    inputCircles.circlesCenters = [152, 251]
    inputCircles.circlesRadii = [75]
    inputCircles.uncertainty = [0.5, 0., 0., 0., 0.5, 0., 0., 0., 1.]
    inputCircles.valid = 1
    inputCircles.planetIds = [2]  # Mars
    circlesInMsg = messaging.OpNavCirclesMsg().write(inputCircles)
    pixelLine.circlesInMsg.subscribeTo(circlesInMsg)

    # Set attitude
    inputAtt.sigma_BN = [0.6, 1., 0.1]
    attInMsg = messaging.NavAttMsg().write(inputAtt)
    pixelLine.attInMsg.subscribeTo(attInMsg)

    dataLog = pixelLine.opNavOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(0)  # [ns]
    unitTestSim.ExecuteSimulation()
    inputCircles.timeTag = 12345  # [ns]
    circlesInMsg.write(inputCircles, testProcessRate)
    unitTestSim.ConfigureStopTime(testProcessRate)  # [ns]
    unitTestSim.ExecuteSimulation()

    # Truth Values
    planet = {"diameter": 2 * 3_396_190.0}  # [m]

    state = [inputCircles.circlesCenters[0], inputCircles.circlesCenters[1], inputCircles.circlesRadii[0]]

    r_Cexp = mapState(state, planet, inputCamera)
    covar_Cexp = mapCovar(inputCircles.uncertainty, state[2], planet, inputCamera)

    dcm_CB = rbk.MRP2C(inputCamera.sigma_CB)
    dcm_BN = rbk.MRP2C(inputAtt.sigma_BN)

    dcm_NC = np.dot(dcm_CB, dcm_BN).T

    r_Nexp = np.dot(dcm_NC, r_Cexp)
    covar_Nexp = np.dot(dcm_NC, np.dot(covar_Cexp, dcm_NC.T)).flatten()
    rtol = 1e-10  # [-]
    simHelpers.writeTeXSnippet("toleranceValuePos", str(rtol), path)
    simHelpers.writeTeXSnippet("toleranceValueVel", str(rtol), path)
    try:
        assert dataLog.valid[-1] == 1
        np.testing.assert_allclose(dataLog.r_BN_N[-1], r_Nexp, rtol=rtol, atol=0, equal_nan=False)
        np.testing.assert_allclose(dataLog.covar_N[-1], covar_Nexp, rtol=rtol, atol=0, equal_nan=False)
        assert dataLog.timeTag[-1] == inputCircles.timeTag
    except AssertionError:
        simHelpers.writeTeXSnippet("passFail", r"\textcolor{Red}{Failed}", path)
        raise
    simHelpers.writeTeXSnippet("passFail", r"\textcolor{ForestGreen}{PASSED}", path)


if __name__ == '__main__':
    test_pixelLine_converter()


@pytest.mark.parametrize("planet_id", [0, -1, 4, 1.5, float("nan"), float("inf")])
def test_pixel_line_converter_rejects_unknown_planet(planet_id):
    """Reject invalid circle identifiers before casting or using a planet radius."""
    module = pixelLineConverter.pixelLineConverter()
    camera = messaging.CameraConfigMsg().write(messaging.CameraConfigMsgPayload())
    attitude = messaging.NavAttMsg().write(messaging.NavAttMsgPayload())
    payload = messaging.OpNavCirclesMsgPayload()
    payload.valid = 1
    payload.planetIds = [planet_id]
    circles = messaging.OpNavCirclesMsg().write(payload)
    module.cameraConfigInMsg.subscribeTo(camera)
    module.attInMsg.subscribeTo(attitude)
    module.circlesInMsg.subscribeTo(circles)
    module.SelfInit()
    module.Reset(0)
    with pytest.raises(bskLogging.BasiliskError, match="pixelLineConverter: unsupported circle planet ID"):
        module.UpdateState(0)


@pytest.mark.parametrize(
    "resolution, center, expected_ray",  # [pixels], [pixels], [-]
    [
        ((512, 512), (255.5, 255.5), (0, 0, -1)),
        ((511, 511), (255.0, 255.0), (0, 0, -1)),
        ((511, 512), (255.0, 255.5), (0, 0, -1)),
        ((512, 511), (255.5, 255.0), (0, 0, -1)),
        # A 90-degree field of view across 512 pixels gives a focal length of 256 pixels.
        ((512, 512), (256.5, 254.5), (-1, 1, -256)),
    ],
    ids=["even-even", "odd-odd", "odd-even", "even-odd", "off-center"],
)
def test_pixel_line_converter_camera_direction(resolution, center, expected_ray):
    """Check the camera-frame direction for centered detections and a one-pixel offset."""
    module = pixelLineConverter.pixelLineConverter()
    camera = messaging.CameraConfigMsgPayload()
    camera.resolution = resolution
    camera.fieldOfView = np.pi / 2  # [rad]
    circle = messaging.OpNavCirclesMsgPayload()
    circle.valid = 1
    circle.planetIds = [2]  # Mars
    circle.circlesCenters = center
    circle.circlesRadii = [75.0]  # [pixels]
    module.cameraConfigInMsg.subscribeTo(messaging.CameraConfigMsg().write(camera))
    module.circlesInMsg.subscribeTo(messaging.OpNavCirclesMsg().write(circle))
    module.attInMsg.subscribeTo(messaging.NavAttMsg().write(messaging.NavAttMsgPayload()))
    module.SelfInit()
    module.Reset(0)
    module.UpdateState(0)

    output = module.opNavOutMsg.read()
    assert output.valid == 1
    position = np.asarray(output.r_BN_C)
    ray = np.asarray(expected_ray)
    np.testing.assert_allclose(
        position / np.linalg.norm(position), ray / np.linalg.norm(ray), rtol=0, atol=1e-14
    )
