# ISC License
#
# Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import os, inspect
import numpy as np
import pytest
from matplotlib import pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.architecture import messaging
from Basilisk.simulation import pinholeCamera
from Basilisk.simulation import spacecraft
from Basilisk.utilities import unitTestSupport
from Basilisk import __path__

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = __path__[0]


def test_visibility():
    """
    Tests whether pinholeCamera:

    1. Computes correctly pixels for center and corners.
    2. Detects correctly that some landmarks are not within field of view.

    :return:
    """
    simTime = 1.

    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTime = macros.sec2nano(simTime)
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Set the landmarks distribution to be tested
    # The first five landmarks shall correspond to center and corners of the image (for the prescribed situation)
    # The last five landmarks shall be detected as not within FOV
    FOVx = 38.17124015837933 * np.pi/180  # this is the horizontal FOV, check camera.FOVx
    FOVy = 29.094758030219015 * np.pi/180  # this is the vertical FOV, check camera.FOVy
    n_lmk = 10
    pos_lmk = np.array([[-16*1e3, 0, 0],                        # Image center
                        [-16*1e3, 18*1e3 * np.tan(FOVy/2), 0],  # Image corner (18 km = horizontal distance sc-lmk)
                        [-16*1e3, -18*1e3*np.tan(FOVy/2), 0],   # Image corner
                        [-16*1e3, 0, 18*1e3*np.tan(FOVx/2)],    # Image corner
                        [-16*1e3, 0, -18*1e3*np.tan(FOVx/2)],   # Image corner
                        [16*1e3, 0, 0],                         # Not visible
                        [0, 16*1e3, 0],                         # Not visible
                        [0, -16*1e3, 0],                        # Not visible
                        [0, 0, 16*1e3],                         # Not visible
                        [0, 0, -16*1e3]])                       # Not visible
    normal_lmk = np.array([[-1, 0, 0],
                           [-1, 0, 0],                          # Mock to ensure visibility
                           [-1, 0, 0],                          # Mock to ensure visibility
                           [-1, 0, 0],                          # Mock to ensure visibility
                           [-1, 0, 0],                          # Mock to ensure visibility
                           [1, 0, 0],
                           [0, 1, 0],
                           [0, -1, 0],
                           [0, 0, 1],
                           [0, 0, -1]])

    # Set the pinhole camera module
    camera = pinholeCamera.PinholeCamera()
    camera.f = 25*1e-3
    camera.nxPixel = 2048
    camera.nyPixel = 1536
    camera.wPixel = (17.3*1e-3) / 2048
    dcm_CB = np.array([[0, 0, -1],
                       [0, 1, 0],
                       [1, 0, 0]])
    camera.dcm_CB = dcm_CB.tolist()
    for i in range(n_lmk):
        camera.addLandmark(pos_lmk[i, 0:3], normal_lmk[i, 0:3])
    scSim.AddModelToTask(simTaskName, camera)

    # Write out mock planet ephemeris message
    planet_message = messaging.EphemerisMsgPayload()
    aP = 1.4583 * 149597870.7*1e3
    r_PN_N = np.array([aP, 0, 0])
    v_PN_N = np.array([0, np.sqrt(orbitalMotion.MU_SUN/aP), 0])
    dcm_PN = np.identity(3)
    mrp_PN = rbk.C2MRP(dcm_PN)
    planet_message.r_BdyZero_N = r_PN_N
    planet_message.v_BdyZero_N = v_PN_N
    planet_message.sigma_BN = mrp_PN
    planetMsg = messaging.EphemerisMsg().write(planet_message)
    camera.ephemerisInMsg.subscribeTo(planetMsg)

    # Write out mock spacecraft message (pointing towards planet shall be ensured)
    a = 34 * 1e3
    mu = 4.4631 * 1e5
    r_BP_P = np.array([-a, 0, 0])
    v_BP_P = np.array([0, -np.sqrt(mu/a), 0])
    r_BN_N = r_BP_P + r_PN_N
    v_BN_N = v_BP_P + v_PN_N
    ir = r_BP_P / np.linalg.norm(r_BP_P)
    ih = np.cross(ir, v_BP_P/np.linalg.norm(v_BP_P))
    it = np.cross(ir, ih)
    dcm_BP = np.zeros((3, 3))
    dcm_BP[0:3, 0] = -ir
    dcm_BP[0:3, 1] = ih
    dcm_BP[0:3, 2] = it
    dcm_BN = np.matmul(dcm_BP, dcm_PN)
    mrp_BN = rbk.C2MRP(dcm_BN)
    sc1_message = messaging.SCStatesMsgPayload()
    sc1_message.r_BN_N = r_BN_N
    sc1_message.v_BN_N = v_BN_N
    sc1_message.sigma_BN = mrp_BN
    scMsg = messaging.SCStatesMsg().write(sc1_message)
    camera.scStateInMsg.subscribeTo(scMsg)

    # Log the landmark messages
    numDataPoints = 2
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = []
    for i in range(len(pos_lmk)):
        dataLog.append(camera.landmarkOutMsgs[i].recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, dataLog[i])

    # Run the sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Get the logged data
    isvisibleLmk = np.zeros((2, n_lmk))
    pixelLmk = np.zeros((2, n_lmk, 2))
    for i in range(n_lmk):
        isvisibleLmk[:, i] = dataLog[i].isVisible
        pixelLmk[:, i, 0:2] = dataLog[i].pL

    # Define expected values
    # The corners shall correspond to the maximum pixel resolution
    # The default behaviour for a point lying in the origin is +1
    accuracy = 1e-8
    ref_pixel = np.array([[1, 1],
                         [1, 1536/2],
                         [1, -1536/2],
                         [-2048/2, 1],
                         [2048/2, 1],
                         [0, 0],
                         [0, 0],
                         [0, 0],
                         [0, 0],
                         [0, 0]])
    ref_isvisible = np.array([1, 1, 1, 1, 1, 0, 0, 0, 0, 0])

    # Compare to expected values
    pixel_worked = pixelLmk[0, :, :] == pytest.approx(ref_pixel, accuracy)
    isvisible_worked = isvisibleLmk[0, :] == pytest.approx(ref_isvisible, accuracy)

    assert (pixel_worked and isvisible_worked)


if __name__ == '__main__':
    test_visibility()
