#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

.. figure:: /_images/static/Vizard1.jpg
   :align: center

   Illustration of Vizard showing a custom spacecraft camera view.

Overview
--------

This scenario demonstrates how instantiate a visualization interface. This includes setting camera
parameters and capture rates. This stems for an attitude detumble scenario, but focuses on
pointing towards a celestial body in order to display the visualization Vizard, and show
the camera capabilities.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioVizPoint.py

When the simulation completes 3 plots are shown for the MRP attitude history, the rate
tracking errors, as well as the control torque vector.  The ``run()`` method is setup to write out the
Vizard data file to sub-folder ``_VizFiles/scenarioVizPoint_UnityViz.bin``.  By running :ref:`Vizard <vizard>`
and playing back this data file you will see the custom camera view that is created as
illustrated in the Vizard snapshot above.

The simulation layout is identical the the :ref:`scenarioAttitudeFeedback` scenario when it comes to FSW modules
The spacecraft starts in a tumble and controls it's rate as well as points to the Earth.

Two mission scenarios can be simulated.
The first one mimics the DSCOVR mission spacecraft and its EPIC camera pointing towards Earth.
The second simulates a spacecraft orbiting about Mars. The attitude results are the same as
the attitude feedback scenario, and pictured in the following plots. The differences lies in
where they are pointing.

.. image:: /_images/Scenarios/scenarioVizPoint1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioVizPoint2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioVizPoint3.svg
   :align: center

In each case a spacecraft fixed camera is simulated.
This is done by connecting to the :ref:`vizInterface` input message
``cameraConfInMsg``  The :ref:`vizInterface` module
checks this input message by default.  If it is linked, then the camera information
is read in and sent across to Vizard to render out that camera view point image.
Open Vizard and play back the resulting simulation binary file to see the camera window.

DSCOVR Mission Setup
--------------------

The first setup has the spacecraft pointing to Earth, from a distant, L1 vantage point.
The scenario controls the spacecraft attitude to Earth pointing mode, and snaps pictures at
a defined rate.
This camera parameters are taken from NASA's `EPIC <https://epic.gsfc.nasa.gov>`__ camera website on the date
2018 OCT 23 04:35:25.000 (UTC time).
In this setup the pointing needs to be set to Earth, given it's position.

Mars Orbit Setup
----------------

The second control scenario points the spacecraft towards Mars on a Mars orbit.

"""


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the vizInterface, spacecraft, simpleNav, mrpFeedback. and inertial3D modules.
# Illustrates a spacecraft pointing with visualization.
# Author:   Thibaud Teil
# Creation Date:  Nov. 01, 2018
#

import os

import numpy as np
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])
fileNamePath = os.path.abspath(__file__)


# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros, orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simpleNav

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError

# import message declarations
from Basilisk.architecture import messaging

# attempt to import vizard
from Basilisk.utilities import vizSupport


def run(show_plots, missionType, saveVizardFile):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        missionType (str):

            ===========  ==================================
            String       Definition
            ===========  ==================================
            'dscovr'     Simulates the NASA DSCOVR mission
            'marsOrbit'  Simulates an orbit about Mars
            ===========  ==================================

        saveVizardFile (bool): Flag to save off the Vizard data file

    """

    missionOptions = ['dscovr', 'marsOrbit'];
    if missionType not in missionOptions:
        print("ERROR: scenarioVizPoint received the wrong mission type " + missionType
              + ". Options include " + str(missionOptions))
        exit(1)

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    if missionType ==  'dscovr':
        # setup Grav Bodies and Spice messages
        gravFactory = simIncludeGravBody.gravBodyFactory()
        bodies = gravFactory.createBodies(['earth', 'sun'])
        bodies['earth'].isCentralBody = True  # ensure this is the central gravitational body
        gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                         '2018 OCT 23 04:35:25.000 (UTC)',
                                         epochInMsg=True)

        gravFactory.spiceObject.zeroBase = 'earth'
        scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)
        # Setup Camera.
        cameraConfig = messaging.CameraConfigMsgPayload()
        cameraConfig.cameraID = 1
        cameraConfig.renderRate = 0
        cameraConfig.sigma_CB = [-0.333333, 0.333333, -0.333333]
        cameraConfig.cameraPos_B = [5000. * 1E-3, 0., 0.]  # in meters
        cameraConfig.fieldOfView = 0.62*macros.D2R  # in degrees
        cameraConfig.resolution = [2048, 2048]  # in pixels
    else:
        simulationTime = macros.min2nano(6.25)
        gravFactory = simIncludeGravBody.gravBodyFactory()
        # setup Earth Gravity Body
        mars = gravFactory.createMarsBarycenter()
        mars.isCentralBody = True  # ensure this is the central gravitational body
        mu = mars.mu
        cameraConfig = messaging.CameraConfigMsgPayload()
        cameraConfig.cameraID = 1
        cameraConfig.renderRate = 0
        cameraConfig.sigma_CB = [-0.333333, 0.333333, -0.333333]
        cameraConfig.cameraPos_B = [5000. * 1E-3, 0., 0.]  # in meters
        cameraConfig.fieldOfView = 50.*macros.D2R
        cameraConfig.resolution = [512, 512]  # in pixels
    camMsg = messaging.CameraConfigMsg().write(cameraConfig)

    #
    #   setup the simulation tasks/objects
    #
    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
#    extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    #
    #   setup the FSW algorithm tasks
    #

    if missionType == 'dscovr':
        # Set up pointing frame and camera position given the initial conditions on Oct 23rd 2018 4:35 UTC
        # and the DDSCOVR data
        earthVec = np.array([129559501208.24178, 68180766143.44236,29544768114.76163])
        normal = np.array([0.,0.,1.])
        sunVec = np.array([-32509693.54023, 1002377617.77831, 423017670.86700])
        dscovrEarthDistance = 1405708000.
        SEVangle = 7.28

        r_sc = dscovrEarthDistance * (sunVec-earthVec)/np.linalg.norm(sunVec-earthVec)
        v_sc = np.zeros(3)

        b1_n = -(sunVec-earthVec)/np.linalg.norm(sunVec-earthVec)
        b3_n = (normal - np.dot(normal, b1_n)*b1_n)/np.linalg.norm(normal - np.dot(normal, b1_n)*b1_n)
        assert np.abs(np.dot(b1_n, b3_n)) < 1E-10, 'Wrong dcm'
        b2_n = np.cross(b3_n, b1_n)/np.linalg.norm( np.cross(b3_n, b1_n))
        NB = np.zeros([3,3])
        NB[:, 0] = b1_n
        NB[:, 1] = b2_n
        NB[:, 2] = b3_n

        earthPoint = rbk.C2MRP(NB.T)
    else:
        earthPoint = np.array([0.,0.,0.1])

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = earthPoint.tolist()  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    mrpControl.K = 3.5
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    cmdRec = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    attErrRec = attError.attGuidOutMsg.recorder(samplingTime)
    dataLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, cmdRec)
    scSim.AddModelToTask(simTaskName, attErrRec)
    scSim.AddModelToTask(simTaskName, dataLog)

    #
    #   set initial Spacecraft States
    #
    # setup the orbit using classical orbit elements
    # for orbit around Earth
    if missionType == 'marsOrbit':
        oe = orbitalMotion.ClassicElements()
        oe.a = 16000000 # meters
        oe.e = 0.1
        oe.i = 10. * macros.D2R
        oe.Omega = 25. * macros.D2R
        oe.omega = 10. * macros.D2R
        oe.f = 160. * macros.D2R
        rN, vN = orbitalMotion.elem2rv(mu, oe)
    else:
        rN = r_sc
        vN = v_sc
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    #
    #   initialize Simulation
    #
    if saveVizardFile:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                                  saveFile=fileNamePath)
        viz.addCamMsgToModule(camMsg)
        viz.settings.viewCameraConeHUD = 1
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataLr = cmdRec.torqueRequestBody
    dataSigmaBR = attErrRec.sigma_BR
    dataOmegaBR = attErrRec.omega_BR_B
    dataPos = dataLog.r_BN_N
    np.set_printoptions(precision=16)


    #
    #   plot the results
    #
    plt.close("all")  # clears out plots from earlier test runs
    timeAxis = cmdRec.times() * macros.NANO2MIN
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,               # show_plots
        'marsOrbit',           # missionType: dscovr or marsOrbit
        True                # saveVizardFile: flag to save the Vizard data file
    )
