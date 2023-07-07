#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

r"""
Overview
--------

Discusses how to use the pinhole camera module to obtain landmark-based measurements around a small body.
This script sets up a 6-DOF spacecraft which is orbiting asteroid 433 Eros.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioSmallBodyLandmarks.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation, flight software and pinhole camera modules.

.. image:: /_images/static/test_scenarioSmallBodyLandmarks.svg
   :align: center

When the simulation completes 5 plots are shown for the 3D configuration at :math:`t_0`, the 3D configuration
at :math:`t_f`, the landmark pixels at :math:`t_f`, the number of visible landmarks evolution and the camera frame C axes
projection :math:`\hat c_1`, :math:`\hat c_2` and :math:`\hat c_3` onto the Hill or Orbit frame axes
:math:`\hat\imath_r`, :math:`\hat\imath_{\theta}` and :math:`\hat\imath_h`.

The basic simulation setup is very similar as the one used in :ref:`scenarioAttitudeGuidance`.
The dynamics simulation is setup using a :ref:`Spacecraft` module to which a gravity
effector is attached. Eros gravity is setup through a :ref:`PlanetEphemeris` module which simulates
Eros heliocentric motion. The flight software modules are copied from :ref:`scenarioAttitudeGuidance`
as to align the spacecraft body frame, thus also the camera frame, with the Hill frame. The goal is to
keep the camera pointing towards Eros center of mass in order to observe surface landmarks. In this
case, the attitude control flight software makes :math:`\hat b_1` body axis to point towards the
asteroid center of mass :math:`-\hat\imath_r` (negative radial direction ). The camera direction cosine matrix
is prescribed by the user as :math:`\hat c_3=\hat b_1`, then it follows that attitude control is ensuring
the camera focal direction (:math:`\hat c_3`) is pointing towards the asteroid. This way, the camera is able
to track surface landmarks.

The landmarks are setup based on Eros polyhedron shape. A landmark distribution is defined by surface
positions and their normals expressed in the planet rotating frame. The normal is used to check field
of view and lighting conditions from the camera and Sun respectively.

Since dynamics simulations can be computationally expensive, the module has a ``processBatch(rBatch_CP_P,
mrpBatch_BP, eBatch_SP_P, show_progress)`` method that is detached from the Simulation Base class. This method
requires batches (as matrices) of spacecraft position in the planet frame, spacecraft orientation in the planet
frame and Sun's unit vector in the planet frame. The results can be accessed from the pinhole camera class
as illustrated in the subsequent script.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useBatch = False

The ``useBatch`` flag is turned off by default but it showcases how the module can be executed detached from the
dynamics simulation. This is very convenient as it allows isolated experimentation with the pinholeCamera module
without the need of rerunning dynamics simulations (which can be slow if non-Keplerian gravity models are used
e.g. polyhedron). Alternatively, it allows to set up customized situations that may not be in accordance with a
dynamics simulation (e.g. manually prescribe radial pointing towards the asteroid)

The 3D situation is shown in these first two plots. In the first one, the attitude control has not converged
and the camera is not pointing towards the asteroid. Consequently, no landmarks are visible. In the second
one, the attitude control has aligned the camera focal direction (in blue) with the negative radial direction,
thus some landmarks are visible (in blue) on the surface. The third figure shows the visible landmark pixels
(with their labels) in the image plane.

.. image:: /_images/Scenarios/scenarioSmallBodyLandmarks1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyLandmarks2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyLandmarks3.svg
   :align: center

The next two plots show time evolution of the number of visible landmarks and the alignment of the camera
frame with respect to the Hill frame. Note that :math:`\hat c_3` is aligned with the negative radial direction
of the Hill frame as expected.

.. image:: /_images/Scenarios/scenarioSmallBodyLandmarks4.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyLandmarks5.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the pinholeCamera() module. Illustrates how
#           the pinhole camera module can be integrated with spacecraft,
#           planet and attitude control modules.
# Author:   Julio C. Sanchez
# Creation Date:  Jun. 26, 2023
#

import os
import matplotlib.pyplot as plt
import numpy as np

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

# import message declarations
from Basilisk.architecture import messaging

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import hillPoint
from Basilisk.fswAlgorithms import mrpFeedback

# import simulation related support
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import pinholeCamera
from Basilisk.simulation import planetEphemeris
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft
from Basilisk.simulation.gravityEffector import loadPolyFromFileToList

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions

# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Landmark distribution function
def landmark_distribution(vert_list, face_list, n_vert, n_face, n_lmk):
     """Creates a landmark distribution based on a polyhedron shape."""
     pos_vert = np.array(vert_list)
     order_face = np.array(face_list) - 1
     pos_face = np.zeros((n_face,3))
     normal_face = np.zeros((n_face,3))
     for i in range(n_face):
         pos0 = pos_vert[order_face[i, 0], 0:3]
         pos1 = pos_vert[order_face[i, 1], 0:3]
         pos2 = pos_vert[order_face[i, 2], 0:3]
         pos_face[i, 0:3] = (pos0 + pos1 + pos2) / 3
         normal_face[i, 0:3] = np.cross(pos1-pos0, pos2-pos0)
         normal_face[i, 0:3] /= np.linalg.norm(normal_face[i, 0:3])

     np.random.seed(0)
     idx_lmk = np.random.choice(n_face, n_lmk, replace=False)
     idx_lmk.sort()
     pos_lmk = pos_face[idx_lmk, 0:3]
     normal_lmk = normal_face[idx_lmk, 0:3]

     return pos_lmk, normal_lmk


def plot_3D(t, r, xyz_vert, order_face, posLmk, isvisibleLmk, dcm_CP):
    """Plot the 3D situation of asteroid, landmarks, spacecraft and camera frame."""
    idxOn = isvisibleLmk == 1
    idxOff = isvisibleLmk != 1
    color_asteroid = [105 / 255, 105 / 255, 105 / 255]

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.plot(r[0] / 1000, r[1] / 1000, r[2] / 1000, 'k', marker='s', markersize=5)
    ax.plot_trisurf(xyz_vert[:, 0] / 1000, xyz_vert[:, 1] / 1000, xyz_vert[:, 2] / 1000,
                    triangles=order_face-1, color=color_asteroid, zorder=0, alpha=0.1)
    ax.plot(posLmk[idxOn, 0] / 1000, posLmk[idxOn, 1] / 1000, posLmk[idxOn, 2] / 1000, 'b', linestyle='',
            marker='.', markersize=5)
    ax.plot(posLmk[idxOff, 0] / 1000, posLmk[idxOff, 1] / 1000, posLmk[idxOff, 2] / 1000, 'r', linestyle='',
            marker='.', markersize=5, alpha=0.25)
    ax.quiver(r[0] / 1000, r[1] / 1000, r[2] / 1000, dcm_CP[0,0], dcm_CP[1,0], dcm_CP[2,0], length=10, normalize=True,
              color='black', alpha=0.25)
    ax.quiver(r[0] / 1000, r[1] / 1000, r[2] / 1000, dcm_CP[0,1], dcm_CP[1,1], dcm_CP[2,1], length=10, normalize=True,
              color='black', alpha=0.25)
    ax.quiver(r[0] / 1000, r[1] / 1000, r[2] / 1000, dcm_CP[0,2], dcm_CP[1,2], dcm_CP[2,2], length=10, normalize=True,
              color='blue')
    ax.set_xlabel('${}^{P}r_{x}$ [km]')
    ax.set_ylabel('${}^{P}r_{y}$ [km]')
    ax.set_zlabel('${}^{P}r_{z}$ [km]')
    t_str = str(int(t/60))
    ax.set_title('Asteroid rotating frame, t=' + t_str + ' min')


def plot_pixel(pixelLmk, statusLmk):
    """Plot landmarks labeled pixels in the camera image."""
    idxOn = statusLmk == 1
    pixelOn = pixelLmk[idxOn, :]

    fig = plt.figure()
    ax = fig.gca()
    plt.plot(pixelOn[:,0], pixelOn[:,1], linestyle='', marker='s', color='b', markersize=5)
    for i in range(len(pixelOn)):
        ax.annotate(str(i), (pixelOn[i,0], pixelOn[i,1]))
    ax.set_xlim([-1024, 1024])
    ax.set_ylim([-768, 768])
    plt.xlabel('$p_x$ [-]')
    plt.ylabel('$p_y$ [-]')


def plot_nLmk(t, nvisibleLmk):
    """Plot visible landmarks evolution."""
    fig = plt.figure()
    ax = fig.gca()
    plt.plot(t/3600, nvisibleLmk, linestyle='--', marker='.', markersize=8)
    ax.set_xlim([t[0]/3600, t[-1]/3600])
    plt.xlabel('Time [h]')
    plt.ylabel('Visible landmarks [-]')


def plot_orientation(t, dcm_HP, dcm_CP):
    """Plot the camera frame orientation with respect to Hill frame."""
    data = np.zeros((len(t), 3))
    for i in range(len(t)):
        data[i,0:3] = [np.dot(dcm_HP[i, 0:3, 0], dcm_CP[i, 0:3, 2]),
                       np.dot(dcm_HP[i, 0:3, 1], dcm_CP[i, 0:3, 0]),
                       np.dot(dcm_HP[i, 0:3, 2], dcm_CP[i, 0:3, 1])]
    fig = plt.figure()
    ax = fig.gca()
    labelStrings = (r'$\hat\imath_r\cdot \hat c_3$'
                    , r'${\hat\imath}_{\theta}\cdot \hat c_1$'
                    , r'$\hat\imath_h\cdot \hat c_2$')
    for idx in range(3):
        plt.plot(t/3600, data[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=labelStrings[idx])
    ax.set_xlim([t[0]/3600, t[-1]/3600])
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Orientation Illustration')


def run(show_plots, useBatch):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useBatch (bool): Specify if the pinhole camera model batch process is to be checked.
    """

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.SetProgressBar(True)

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Define the simulation duration
    simulationTime = macros.min2nano(100.)

    #  Create the process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Add the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   Add modules to the task
    #

    # Setup celestial object ephemeris module
    gravBodyEphem = planetEphemeris.PlanetEphemeris()
    gravBodyEphem.ModelTag = 'erosEphemeris'
    gravBodyEphem.setPlanetNames(planetEphemeris.StringVector(["eros"]))

    # Specify asteroid orbit elements and rotational state January 21st, 2022
    # https://ssd.jpl.nasa.gov/horizons.cgi#results
    oeAsteroid = planetEphemeris.ClassicElementsMsgPayload()
    oeAsteroid.a = 1.4583 * 149597870.7*1e3  # meters
    oeAsteroid.e = 0.2227
    oeAsteroid.i = 10.829 * np.pi/180
    oeAsteroid.Omega = 304.3 * np.pi/180
    oeAsteroid.omega = 178.9 * np.pi/180
    oeAsteroid.f = 246.9 * np.pi/180
    AR = 11.369 * np.pi/180
    dec = 17.227 * np.pi/180
    lst0 = 0 * macros.D2R
    gravBodyEphem.planetElements = planetEphemeris.classicElementVector([oeAsteroid])
    gravBodyEphem.rightAscension = planetEphemeris.DoubleVector([AR])
    gravBodyEphem.declination = planetEphemeris.DoubleVector([dec])
    gravBodyEphem.lst0 = planetEphemeris.DoubleVector([lst0])
    gravBodyEphem.rotRate = planetEphemeris.DoubleVector([360 * macros.D2R / (5.27 * 3600.)])
    dcm_PN = RigidBodyKinematics.euler3232C([AR, np.pi/2 - dec,  lst0])

    # Set up asteroid gravity effector (only Keplerian gravity for simplicity)
    # https://ssd.jpl.nasa.gov/tools/gravity.html#/vesta
    gravFactory = simIncludeGravBody.gravBodyFactory()
    mu = 4.4631 * 1e5
    asteroid = gravFactory.createCustomGravObject("eros", mu=mu, radEquator=16*1000)
    asteroid.isCentralBody = True
    asteroid.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])

    # Create an ephemeris converter
    ephemConverter = ephemerisConverter.EphemerisConverter()
    ephemConverter.ModelTag = "ephemConverter"
    ephemConverter.addSpiceInputMsg(gravBodyEphem.planetOutMsgs[0])

    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # Set spacecraft initial condition
    oe = orbitalMotion.ClassicElements()
    oe.a = 34 * 1e3
    oe.e = 0.001
    oe.i = 45 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rP, vP = orbitalMotion.elem2rv(mu, oe)
    rN = dcm_PN.transpose().dot(rP)
    vN = dcm_PN.transpose().dot(vP)
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN
    scObject.hub.sigma_BNInit = [[0.7], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    # Attach gravity to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Set extForceTorque module
    # The control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)

    # Add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Set hillPoint guidance module
    attGuidanceConfig = hillPoint.hillPointConfig()
    attGuidanceWrap = scSim.setModelDataWrap(attGuidanceConfig)
    attGuidanceWrap.ModelTag = "hillPoint"
    attGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    attGuidanceConfig.celBodyInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0])

    # Set the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    attErrorConfig.sigma_R0R = [0, 1, 0]
    attErrorConfig.attRefInMsg.subscribeTo(attGuidanceConfig.attRefOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # Set the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1.0  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # Connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    # Prepare 100 landmarks distribution
    polyFile = bskPath + '/supportData/LocalGravData/eros007790.tab'
    vert_list, face_list, n_vert, n_face = loadPolyFromFileToList(polyFile)
    n_lmk = 100
    pos_lmk, normal_lmk = landmark_distribution(vert_list, face_list, n_vert, n_face, n_lmk)

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
    camera.ephemerisInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0])
    camera.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Add models to task
    scSim.AddModelToTask(simTaskName, gravBodyEphem, ModelPriority=100)
    scSim.AddModelToTask(simTaskName, ephemConverter, ModelPriority=99)
    scSim.AddModelToTask(simTaskName, scObject, ModelPriority=98)
    scSim.AddModelToTask(simTaskName, extFTObject, ModelPriority=97)
    scSim.AddModelToTask(simTaskName, sNavObject, ModelPriority=96)
    scSim.AddModelToTask(simTaskName, attGuidanceWrap, attGuidanceConfig, ModelPriority=95)
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig, ModelPriority=94)
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig, ModelPriority=93)
    scSim.AddModelToTask(simTaskName, camera, ModelPriority=92)

    # Add data logging to task
    samplingTime = macros.sec2nano(60)
    asteroidLog = ephemConverter.ephemOutMsgs[0].recorder(samplingTime)
    spacecraftLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, asteroidLog)
    scSim.AddModelToTask(simTaskName, spacecraftLog)
    landmarkLog = []
    for i in range(n_lmk):
        landmarkLog.append(camera.landmarkOutMsgs[i].recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, landmarkLog[i])

    # Create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    configDataMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    mrpControlConfig.vehConfigInMsg.subscribeTo(configDataMsg)

    # Initialize Simulation
    scSim.InitializeSimulation()

    # Configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve the logged data as numpy arrays
    t = np.array(spacecraftLog.times() * macros.NANO2SEC)
    r_PN_N = np.array(asteroidLog.r_BdyZero_N)
    v_PN_N = np.array(asteroidLog.v_BdyZero_N)
    mrp_PN = np.array(asteroidLog.sigma_BN)
    r_BN_N = np.array(spacecraftLog.r_BN_N)
    v_BN_N = np.array(spacecraftLog.v_BN_N)
    mrp_BN = np.array(spacecraftLog.sigma_BN)
    pixelLmk = np.zeros((len(t), n_lmk, 2))
    isvisibleLmk = np.zeros((len(t), n_lmk))
    for i in range(n_lmk):
        pixelLmk[:, i, 0:2] = landmarkLog[i].pL
        isvisibleLmk[:, i] = landmarkLog[i].isVisible
    nvisibleLmk = np.sum(isvisibleLmk, axis=1)

    # Compute spacecraft position and velocity w.r.t. asteroid expressed in inertial frame
    r_BP_N = r_BN_N - r_PN_N
    v_BP_N = v_BN_N - v_PN_N

    # Compute variables of interest
    n = len(t)
    r_BP_P = np.zeros((n, 3))
    v_BP_P = np.zeros((n, 3))
    mrp_BP = np.zeros((n, 3))
    r_PN_P = np.zeros((n, 3))
    dcm_PC = np.zeros((n, 3, 3))
    dcm_PH = np.zeros((n, 3, 3))
    for i in range(n):
        # Compute auxiliary dcm
        dcm_PN = RigidBodyKinematics.MRP2C(mrp_PN[i][0:3])
        dcm_BN = RigidBodyKinematics.MRP2C(mrp_BN[i][0:3])
        dcm_BP = np.matmul(dcm_BN, dcm_PN.T)

        # Fill variables of interest
        r_BP_P[i, 0:3] = dcm_PN.dot(r_BP_N[i, 0:3])
        v_BP_P[i, 0:3] = dcm_PN.dot(v_BP_N[i, 0:3])
        mrp_BP[i, 0:3] = RigidBodyKinematics.C2MRP(dcm_BP)
        dcm_PC[i, 0:3, 0:3] = np.matmul(dcm_CB, dcm_BP).T
        r_PN_P[i, 0:3] = dcm_PN.dot(r_PN_N[i, 0:3])

        # Fill dcm between planet and Hill frame
        ir = r_BP_P[i, 0:3] / np.linalg.norm(r_BP_P[i, 0:3])
        hv = np.cross(r_BP_P[i, 0:3], v_BP_P[i, 0:3])
        ih = hv / np.linalg.norm(hv)
        itheta = np.cross(ih, ir)
        dcm_PH[i, 0:3, 0] = ir
        dcm_PH[i, 0:3, 1] = ih
        dcm_PH[i, 0:3, 2] = itheta

    # Extract conditions at t0 and tf
    t0 = t[0]
    r0 = r_BP_P[0, 0:3]
    dcm0 = dcm_PC[0, 0:3, 0:3]
    isvisibleLmk0 = isvisibleLmk[0, :]
    tf = t[-1]
    rf = r_BP_P[-1, 0:3]
    dcmf = dcm_PC[-1, 0:3, 0:3]
    isvisibleLmkf = isvisibleLmk[-1, :]

    # Showcase a batch computation
    # This is useful because oftentimes dynamics simulation is computationally expensive
    # Thus, experimenting with camera parameters can be detached from dynamics
    if useBatch:
        # Preallocate output pixels
        pixelBatchLmk = np.zeros((n, n_lmk, 2))

        # Process pinhole camera as a batch
        camera.processBatch(r_BP_P, mrp_BP, -r_PN_P / np.linalg.norm(r_PN_P, axis=1)[:, None], False)
        isvisibleBatchLmk = np.array(camera.isvisibleBatchLmk)
        nvisibleBatchLmk = np.sum(isvisibleBatchLmk, axis=1)
        pixelBatchLmk[:, :, 0] = np.array(camera.pixelBatchLmk)[:, 0:n_lmk]
        pixelBatchLmk[:, :, 1] = np.array(camera.pixelBatchLmk)[:, n_lmk:2*n_lmk]

        # Ensure that results are equal as BSK sim
        batch_diff = np.array([np.linalg.norm(nvisibleBatchLmk - nvisibleLmk) / np.linalg.norm(nvisibleLmk),
                      np.linalg.norm(isvisibleBatchLmk - isvisibleLmk) / np.linalg.norm(isvisibleLmk),
                      np.linalg.norm(pixelBatchLmk - pixelLmk) / np.linalg.norm(pixelLmk)])
    else:
        batch_diff = 0

    # Plot results of interest
    figureList = {}

    plot_3D(t0, r0, np.array(vert_list), np.array(face_list), pos_lmk, isvisibleLmk0, dcm0)
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_3D(tf, rf, np.array(vert_list), np.array(face_list), pos_lmk, isvisibleLmkf, dcmf)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_pixel(pixelLmk[-1, : , :], isvisibleLmk[-1, :])
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_nLmk(t, nvisibleLmk)
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plot_orientation(t, dcm_PH, dcm_PC)
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

    if show_plots:
        plt.show()

    # Close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return batch_diff, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,   # show_plots
        False   # useBatch
    )
