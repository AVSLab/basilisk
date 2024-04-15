#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This simulation demonstrates how to put a spacecraft in orbit about a custom gravitational body while conducting several
attitude changes. Several attitude pointing modes are implemented, along with other visual tools including antenna
transmission and thruster visualization.

The spacecraft starts on a elliptical orbit towards the asteroid Bennu. The spacecraft conducts a
burn at periapsis of the elliptical orbit, transferring to a circular orbit about Bennu with a radius of 800
meters. The spacecraft then completes a series of Hohmann transfers while also conducting several attitude changes
until reaching a final elliptical orbit about the asteroid.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioAsteroidArrival.py

.. attention::

    To see the asteroid Bennu in Vizard the asteroid asset bundle must be installed.  See
    the Vizard `Download <http://hanspeterschaub.info/basilisk/Vizard/VizardDownload.html>`__ web page.

Setting Up The Custom Gravitational Body
----------------------------------------

Because Spice will not be used to generate the ephemeris information for Bennu, an instance of the module
``planetEphemeris`` is created to generate Bennu's ephemeris::

    gravBodyEphem = planetEphemeris.PlanetEphemeris()
    gravBodyEphem.ModelTag = 'planetEphemeris'
    scSim.AddModelToTask(simTaskName, gravBodyEphem)
    gravBodyEphem.setPlanetNames(planetEphemeris.StringVector(["Bennu"]))

Next, the module is configured by specifying the orbital parameters of Bennu::

    timeInitString = "2011 January 1 0:00:00.0"
    diam = 2 * 245.03  # m
    G = 6.67408 * (10 ** -11)  # m^3 / kg*s^2
    massBennu = 7.329 * (10 ** 10)  # kg
    mu = G * massBennu  # Bennu grav. parameter, m^3/s^2
    oeAsteroid = planetEphemeris.ClassicElementsMsgPayload()
    oeAsteroid.a = 1.1264 * orbitalMotion.AU * 1000  # m
    oeAsteroid.e = 0.20375
    oeAsteroid.i = 6.0349 * macros.D2R
    oeAsteroid.Omega = 2.0609 * macros.D2R
    oeAsteroid.omega = 66.2231 * macros.D2R
    oeAsteroid.f = 0.0 * macros.D2R
    gravBodyEphem.planetElements = planetEphemeris.classicElementVector([oeAsteroid])

    gravBodyEphem.rightAscension = planetEphemeris.DoubleVector([85.65 * macros.D2R])
    gravBodyEphem.declination = planetEphemeris.DoubleVector([-60.17 * macros.D2R])
    gravBodyEphem.lst0 = planetEphemeris.DoubleVector([0.0 * macros.D2R])
    gravBodyEphem.rotRate = planetEphemeris.DoubleVector([360 * macros.D2R / (4.296057 * 3600.)])  # rad/sec

Next, Bennu can be created as a gravitational body using the ``createCustomGravObject()`` method::

    asteroid = gravFactory.createCustomGravObject("Bennu", mu)
    asteroid.isCentralBody = True  # ensure this is the central gravitational body

Finally, subscribe the custom gravitational body ``planetBodyInMsg`` to the planetEphemeris output message
``planetOutMsgs``::

    asteroid.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])

The spacecraft object is then created and all gravitational bodies are connected to the spacecraft.

Recall that when configuring the ephemeris converter module, Bennu was not created with Spice. Therefore its input
message is of type ``planetEphemeris``::

    ephemObject.addSpiceInputMsg(gravBodyEphem.planetOutMsgs[0])

Implementing Attitude Pointing Modes
------------------------------------

After the spacecraft's initial orbital elements about Bennu are set using the ``orbitalMotion`` module, the attitude
modules and modes are created and configured. The four attitude pointing modes incorporated into this script include
Earth-pointing using the spacecraft's antenna with transmission visualization, Sun-pointing with the spacecraft's
solar panel normal axis, orbital velocity pointing while conducting thruster burn visualizations, and science-pointing
towards the asteroid using a sensor created on the spacecraft.

.. important:: Refer to the integrated example script :ref:`scenarioFlybySpice` for a more detailed discussion on
   configuring attitude modules and modes for a mission scenario.

To execute the desired attitude-pointing mode, the run flight mode function must be called
with the desired simulation time::

    runAntennaEarthPointing(desiredSimTimeSec)

Additional Visualization Features
---------------------------------

To add a visualization of antenna transmission back to Earth during the Earth-pointing mode we
can't use the typical way of adding these generic sensors, thrusters, etc.  The reason is that we want to illustrate a
thruster, but we are not using a thruster effector.  Thus, to add a thruster to the Vizard binary
we need to manually add these to the ``vizInterface`` spacecraft data structure.

First, as is typical, a transceiver is created through the ``vizInterface``::

    transceiverHUD = vizInterface.Transceiver()
    transceiverHUD.r_SB_B = [0., 0., 1.38]
    transceiverHUD.fieldOfView = 40.0 * macros.D2R
    transceiverHUD.normalVector = [0., 0., 1.]
    transceiverHUD.color = vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
    transceiverHUD.label = "antenna"
    transceiverHUD.animationSpeed = 1

To add a sensor visualization for the science-pointing mode, a sensor is created using the ``vizInterface``::

    genericSensor = vizInterface.GenericSensor()
    genericSensor.r_SB_B = cameraLocation
    genericSensor.fieldOfView.push_back(10.0 * macros.D2R)
    genericSensor.fieldOfView.push_back(10.0 * macros.D2R)
    genericSensor.normalVector = cameraLocation
    genericSensor.size = 10
    genericSensor.color = vizInterface.IntVector(vizSupport.toRGBA255("white", alpha=0.1))
    genericSensor.label = "scienceCamera"
    genericSensor.genericSensorCmd = 1

To add a camera to the science-pointing mode, the ``createStandardCamera`` method is used::

    vizSupport.createStandardCamera(viz, setMode=1, spacecraftName=scObject.ModelTag,
                                    fieldOfView=10 * macros.D2R,
                                    pointingVector_B=[0,1,0], position_B=cameraLocation)

Finally, to add a thruster visualization for the thruster burn mode, the ``vizInterface`` is again invoked.
Here we manually add the Vizard interface elements back in to redo what the ``enableUnityVisualization()``
normally does for us.  The main difference is that we are manually setting the thruster information as
the spacecraft dynamics does not contain a thruster effector::

    scData = vizInterface.VizSpacecraftData()
    scData.spacecraftName = scObject.ModelTag
    scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    scData.transceiverList = vizInterface.TransceiverVector([transceiverHUD])
    scData.genericSensorList = vizInterface.GenericSensorVector([genericSensor])

    thrusterMsgInfo = messaging.THROutputMsgPayload()
    thrusterMsgInfo.maxThrust = 1  # Newtons
    thrusterMsgInfo.thrustForce = 0  # Newtons
    thrusterMsgInfo.thrusterLocation = [0, 0, -1.5]
    thrusterMsgInfo.thrusterDirection = [0, 0, 1]
    thrMsg = messaging.THROutputMsg().write(thrusterMsgInfo)
    scData.thrInMsgs = messaging.THROutputInMsgsVector([thrMsg.addSubscriber()])

After running the ``enableUnityVisualization()`` method, we need to clear the ``vizInterface`` spacecraft
data container ``scData`` and push our custom copy to it::

    viz.scData.clear()
    viz.scData.push_back(scData)


Illustration of Simulation Results
----------------------------------

The following image illustrates the expected simulation run return for the case when plots are requested.

.. image:: /_images/Scenarios/scenarioAsteroidArrival1.svg
   :align: center

Visualization In Vizard
----------------------------------

The following image illustrates the expected visualization of this simulation script.

.. image:: /_images/static/scenarioAsteroidArrival2.jpg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing how to setup a flyby capture orbit about a custom gravity body.
# Author:  Leah Kiner
# Creation Date:  Feb. 6, 2022
#

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


from Basilisk.utilities import (SimulationBaseClass, macros, simIncludeGravBody, vizSupport, unitTestSupport, orbitalMotion)
from Basilisk.simulation import spacecraft, extForceTorque, simpleNav, ephemerisConverter, planetEphemeris
from Basilisk.fswAlgorithms import mrpFeedback, attTrackingError, velocityPoint, locationPointing
from Basilisk.architecture import messaging

try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

# The path to the location of Basilisk
# Used to get the location of supporting data.
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Configure the simulation
    scSim = SimulationBaseClass.SimBaseClass()

    # Shows the simulation progress bar in the terminal
    scSim.SetProgressBar(True)

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the simulation time step information
    simulationTimeStep = macros.sec2nano(20.0)

    # Add dynamics task to the simulation process
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Setup celestial object ephemeris module for the asteroid
    gravBodyEphem = planetEphemeris.PlanetEphemeris()
    gravBodyEphem.ModelTag = 'planetEphemeris'
    scSim.AddModelToTask(simTaskName, gravBodyEphem)
    gravBodyEphem.setPlanetNames(planetEphemeris.StringVector(["bennu"]))

    # Specify orbital parameters of the asteroid
    timeInitString = "2011 January 1 0:00:00.0"
    diam = 2 * 245.03  # m
    G = 6.67408 * (10 ** -11)  # m^3 / kg*s^2
    massBennu = 7.329 * (10 ** 10)  # kg
    mu = G * massBennu  # Bennu grav. parameter, m^3/s^2
    oeAsteroid = planetEphemeris.ClassicElementsMsgPayload()
    oeAsteroid.a = 1.1264 * orbitalMotion.AU * 1000  # m
    oeAsteroid.e = 0.20375
    oeAsteroid.i = 6.0349 * macros.D2R
    oeAsteroid.Omega = 2.0609 * macros.D2R
    oeAsteroid.omega = 66.2231 * macros.D2R
    oeAsteroid.f = 0.0 * macros.D2R
    gravBodyEphem.planetElements = planetEphemeris.classicElementVector([oeAsteroid])

    # Specify orientation parameters of the asteroid
    gravBodyEphem.rightAscension = planetEphemeris.DoubleVector([85.65 * macros.D2R])
    gravBodyEphem.declination = planetEphemeris.DoubleVector([-60.17 * macros.D2R])
    gravBodyEphem.lst0 = planetEphemeris.DoubleVector([0.0 * macros.D2R])
    gravBodyEphem.rotRate = planetEphemeris.DoubleVector([360 * macros.D2R / (4.296057 * 3600.)])  # rad/sec

    # Set orbital radii about asteroid
    r0 = diam/2.0 + 800  # capture orbit, meters
    r1 = diam/2.0 + 600  # intermediate orbit, meters
    r2 = diam/2.0 + 400  # final science orbit, meters
    r3 = diam/2.0 + 200  # meters, very close fly-by, elliptic orbit
    rP = r0
    rA = 3*rP

    # Set orbital periods
    P0 = np.pi*2/np.sqrt(mu/(r0**3))
    P01 = np.pi*2/np.sqrt(mu/(((r0+r1)/2)**3))
    P1 = np.pi*2/np.sqrt(mu/(r1**3))
    P12 = np.pi*2/np.sqrt(mu/(((r1+r2)/2)**3))
    P2 = np.pi*2/np.sqrt(mu/(r2**3))
    P23 = np.pi*2/np.sqrt(mu/(((r2+r3)/2)**3))

    # Create additional gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createBodies("earth", "sun")

    # Set gravity body index values
    earthIdx = 0
    sunIdx = 1
    asteroidIdx = 2

    # Create and configure the default SPICE support module. The first step is to store
    # the date and time of the start of the simulation.
    spiceObject = gravFactory.createSpiceInterface(time=timeInitString, epochInMsg=True)

    # Add the SPICE object to the simulation task list
    scSim.AddModelToTask(simTaskName, spiceObject)

    # Create the asteroid custom gravitational body
    asteroid = gravFactory.createCustomGravObject("bennu", mu
                                                  , modelDictionaryKey="Bennu"
                                                  , radEquator=565. / 2.0
                                                  )
    asteroid.isCentralBody = True  # ensures the asteroid is the central gravitational body
    asteroid.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])  # connect asteroid ephem. to custom grav body

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"

    # Connect all gravitational bodies to the spacecraft
    gravFactory.addBodiesTo(scObject)
    scSim.AddModelToTask(simTaskName, scObject)

    # Create an ephemeris converter to convert messages of type
    # 'SpicePlanetStateMsgPayload' to 'EphemerisMsgPayload'
    ephemObject = ephemerisConverter.EphemerisConverter()
    ephemObject.ModelTag = 'EphemData'
    ephemObject.addSpiceInputMsg(spiceObject.planetStateOutMsgs[earthIdx])
    ephemObject.addSpiceInputMsg(spiceObject.planetStateOutMsgs[sunIdx])
    # Recall the asteroid was not created with Spice.
    ephemObject.addSpiceInputMsg(gravBodyEphem.planetOutMsgs[0])
    scSim.AddModelToTask(simTaskName, ephemObject)

    # Define the spacecraft inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # Define the initial spacecraft orbit about the asteroid
    oe = orbitalMotion.ClassicElements()
    oe.a = (rP + rA)/2.0
    oe.e = 1 - (rP / oe.a)
    oe.i = 90.0 * macros.D2R
    oe.Omega = 180.0 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = -45.0 * macros.D2R
    Ecc = np.arctan(np.tan(-oe.f/2)*np.sqrt((1-oe.e)/(1+oe.e)))*2 # eccentric anomaly
    M = Ecc - oe.e*np.sin(Ecc) # mean anomaly
    n = np.sqrt(mu/(oe.a**3))
    h = np.sqrt(mu*oe.a*(1-oe.e**2)) # specific angular momentum
    vP = h/rP
    V_SC_C_B = np.sqrt(mu / rP)     # [m/s] (2) spacecraft circular parking speed relative to bennu.
    Delta_V_Parking_Orbit = V_SC_C_B - vP

    # Setting initial position and velocity vectors using orbital elements
    r_N, v_N = orbitalMotion.elem2rv(mu, oe)
    T1 = M/n  # time until spacecraft reaches periapsis of arrival trajectory

    # Initialize spacecraft states with the initialization variables
    scObject.hub.r_CN_NInit = r_N  # [m]   = r_BN_N
    scObject.hub.v_CN_NInit = v_N  # [m/s] = v_BN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.000], [-0.00], [0.00]]  # rad/s - omega_BN_B

    # Set up the extForceTorque module
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # Add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    #
    #   Set up the FSW algorithm tasks
    #
    # Set up solar panel Sun-pointing guidance module
    sunPointGuidance = locationPointing.locationPointing()
    sunPointGuidance.ModelTag = "panelSunPoint"
    sunPointGuidance.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[sunIdx])
    sunPointGuidance.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    sunPointGuidance.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    sunPointGuidance.pHat_B = [0.0, 0.0, 1.0]
    sunPointGuidance.useBoresightRateDamping = 1
    scSim.AddModelToTask(simTaskName, sunPointGuidance)

    # Set up asteroid-relative velocityPoint guidance module
    velAsteroidGuidance = velocityPoint.velocityPoint()
    velAsteroidGuidance.ModelTag = "velocityPointAsteroid"
    velAsteroidGuidance.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    velAsteroidGuidance.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[asteroidIdx])
    velAsteroidGuidance.mu = mu
    scSim.AddModelToTask(simTaskName, velAsteroidGuidance)

    # Set up sensor science-pointing guidance module
    cameraLocation = [0.0, 1.5, 0.0]
    sciencePointGuidance = locationPointing.locationPointing()
    sciencePointGuidance.ModelTag = "sciencePointAsteroid"
    sciencePointGuidance.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[asteroidIdx])
    sciencePointGuidance.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    sciencePointGuidance.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    sciencePointGuidance.pHat_B = cameraLocation  # y-axis set for science-pointing sensor
    sciencePointGuidance.useBoresightRateDamping = 1
    scSim.AddModelToTask(simTaskName, sciencePointGuidance)

    # Set up an antenna pointing to Earth guidance module
    earthPointGuidance = locationPointing.locationPointing()
    earthPointGuidance.ModelTag = "antennaEarthPoint"
    earthPointGuidance.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[earthIdx])
    earthPointGuidance.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    earthPointGuidance.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    earthPointGuidance.pHat_B = [0.0, 0.0, 1.0]
    earthPointGuidance.useBoresightRateDamping = 1
    scSim.AddModelToTask(simTaskName, earthPointGuidance)

    # Set up the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)
    attError.attRefInMsg.subscribeTo(sunPointGuidance.attRefOutMsg)  # initial flight mode
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # Create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Set up the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.Ki = -1.0  # make value negative to turn off integral feedback
    II = 900.
    mrpControl.P = 2*II/(20*60)
    mrpControl.K = mrpControl.P*mrpControl.P/II
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # Connect the torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)

    # Set the simulation time
    # Set up data logging before the simulation is initialized
    scRec = scObject.scStateOutMsg.recorder()
    astRec = gravBodyEphem.planetOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, scRec)
    scSim.AddModelToTask(simTaskName, astRec)

    if vizSupport.vizFound:
        # Set up the sensor for the science-pointing mode
        genericSensor = vizInterface.GenericSensor()
        genericSensor.r_SB_B = cameraLocation
        genericSensor.fieldOfView.push_back(10.0 * macros.D2R)
        genericSensor.fieldOfView.push_back(10.0 * macros.D2R)
        genericSensor.normalVector = cameraLocation
        genericSensor.size = 10
        genericSensor.color = vizInterface.IntVector(vizSupport.toRGBA255("white", alpha=0.1))
        genericSensor.label = "scienceCamera"
        genericSensor.genericSensorCmd = 1

        # Set up the antenna visualization for transmission to Earth
        transceiverHUD = vizInterface.Transceiver()
        transceiverHUD.r_SB_B = [0., 0., 1.38]
        transceiverHUD.fieldOfView = 40.0 * macros.D2R
        transceiverHUD.normalVector = [0., 0., 1.]
        transceiverHUD.color = vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
        transceiverHUD.label = "antenna"
        transceiverHUD.animationSpeed = 1

        # Set up the thruster visualization info
        # Note: This process is different from the usual procedure of creating a thruster effector.
        # The following code creates a thruster visualization only.
        # before adding the thruster
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject.ModelTag
        scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
        scData.transceiverList = vizInterface.TransceiverVector([transceiverHUD])
        scData.genericSensorList = vizInterface.GenericSensorVector([genericSensor])

        thrusterMsgInfo = messaging.THROutputMsgPayload()
        thrusterMsgInfo.maxThrust = 1  # Newtons
        thrusterMsgInfo.thrustForce = 0  # Newtons
        thrusterMsgInfo.thrusterLocation = [0, 0, -1.5]
        thrusterMsgInfo.thrusterDirection = [0, 0, 1]
        thrMsg = messaging.THROutputMsg().write(thrusterMsgInfo)
        scData.thrInMsgs = messaging.THROutputMsgInMsgsVector([thrMsg.addSubscriber()])

        thrInfo = vizInterface.ThrClusterMap()
        thrInfo.thrTag = "DV"
        scData.thrInfo = vizInterface.ThrClusterVector([thrInfo])

        # Create the Vizard visualization file and set parameters
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  # , saveFile=fileName
                                                  )
        viz.epochInMsg.subscribeTo(gravFactory.epochMsg)
        viz.settings.showCelestialBodyLabels = 1
        viz.settings.scViewToPlanetViewBoundaryMultiplier = 100
        viz.settings.planetViewToHelioViewBoundaryMultiplier = 100
        viz.settings.orbitLinesOn = -1
        viz.settings.keyboardAngularRate = np.deg2rad(0.5)

        # Create the science mode camera
        vizSupport.createStandardCamera(viz, setMode=1, spacecraftName=scObject.ModelTag,
                                        fieldOfView=10 * macros.D2R,
                                        displayName="10˚ FOV Camera",
                                        pointingVector_B=[0, 1, 0], position_B=cameraLocation)

        # Note: After running the enableUnityVisualization() method, we need to clear the
        # vizInterface spacecraft data container, scData, and push our custom copy to it.
        viz.scData.clear()
        viz.scData.push_back(scData)

    # Initialize and execute the simulation for the first section
    scSim.InitializeSimulation()

    # Set up flight modes
    def runPanelSunPointing(simTime):
        nonlocal simulationTime
        attError.attRefInMsg.subscribeTo(sunPointGuidance.attRefOutMsg)
        if vizSupport.vizFound:
            transceiverHUD.transceiverState = 0  # antenna off
            genericSensor.isHidden = 1
            thrusterMsgInfo.thrustForce = 0
            thrMsg.write(thrusterMsgInfo, simulationTime)
        attError.sigma_R0R = [0, 0, 0]
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    def runSensorSciencePointing(simTime):
        nonlocal simulationTime
        attError.attRefInMsg.subscribeTo(sciencePointGuidance.attRefOutMsg)
        if vizSupport.vizFound:
            transceiverHUD.transceiverState = 0  # antenna off
            genericSensor.isHidden = 0
            thrusterMsgInfo.thrustForce = 0
            thrMsg.write(thrusterMsgInfo, simulationTime)
        attError.sigma_R0R = [0, 0, 0]
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    def runAntennaEarthPointing(simTime):
        nonlocal simulationTime
        attError.attRefInMsg.subscribeTo(earthPointGuidance.attRefOutMsg)
        if vizSupport.vizFound:
            transceiverHUD.transceiverState = 3  # antenna in send and receive mode
            genericSensor.isHidden = 1
            thrusterMsgInfo.thrustForce = 0
            thrMsg.write(thrusterMsgInfo, simulationTime)
        attError.sigma_R0R = [0, 0, 0]
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    def runDvBurn(simTime, burnSign, planetMsg):
        nonlocal simulationTime
        attError.attRefInMsg.subscribeTo(planetMsg)
        if vizSupport.vizFound:
            transceiverHUD.transceiverState = 0  # antenna off
            genericSensor.isHidden = 1
        if burnSign > 0:
            attError.sigma_R0R = [np.tan((np.pi/2)/4), 0, 0]
        else:
            attError.sigma_R0R = [-np.tan((np.pi / 2) / 4), 0, 0]
        minTime = 40 * 60
        if simTime < minTime:
            print("ERROR: runPosDvBurn must have simTime larger than " + str(minTime) + " min")
            exit(1)
        else:
            simulationTime += macros.sec2nano(minTime)
            scSim.ConfigureStopTime(simulationTime)
            scSim.ExecuteSimulation()
            if vizSupport.vizFound:
                thrusterMsgInfo.thrustForce = thrusterMsgInfo.maxThrust
                thrMsg.write(thrusterMsgInfo, simulationTime)
            simulationTime += macros.sec2nano(simTime - minTime)
            scSim.ConfigureStopTime(simulationTime)
            scSim.ExecuteSimulation()

    simulationTime = 0
    np.set_printoptions(precision=16)
    burnTime = 200*60

    # Run thruster burn for arrival to the capture orbit with thrusters on
    runDvBurn(T1, -1, velAsteroidGuidance.attRefOutMsg)

    # Get current spacecraft states
    velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)
    vN = scRec.v_BN_N[-1] - astRec.VelocityVector[-1]

    # Apply a delta V and set the new velocity state in the circular capture orbit
    vHat = vN / np.linalg.norm(vN)
    vN = vN + Delta_V_Parking_Orbit * vHat
    velRef.setState(vN)

    # Travel in a circular orbit at r0, incorporating several attitude pointing modes
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)
    runSensorSciencePointing(P0/3.-burnTime)
    runPanelSunPointing(P0/3.)
    runAntennaEarthPointing(P0/3. - burnTime)
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)

    # Get access to dynManager translational states for future access to the states
    velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)

    # Retrieve the latest relative position and velocity components
    rN = scRec.r_BN_N[-1] - astRec.PositionVector[-1]
    vN = scRec.v_BN_N[-1] - astRec.VelocityVector[-1]

    # Conduct the first burn of a Hohmann transfer from r0 to r1
    rData = np.linalg.norm(rN)
    vData = np.linalg.norm(vN)
    at = (rData + r1) * .5
    v0p = np.sqrt((2 * mu / rData) - (mu / at))
    vHat = vN / vData
    vVt = vN + vHat * (v0p - vData)
    # Update state manager's velocity
    velRef.setState(vVt)

    # Run thruster burn mode along with sun-pointing during the transfer orbit
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)
    runPanelSunPointing(P01/2. - burnTime*2)
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)

    # Retrieve the latest relative position and velocity components
    rN = scRec.r_BN_N[-1] - astRec.PositionVector[-1]
    vN = scRec.v_BN_N[-1] - astRec.VelocityVector[-1]

    # Conduct the second burn of the Hohmann transfer to arrive in a circular orbit at r1
    rData = np.linalg.norm(rN)
    vData = np.linalg.norm(vN)
    v1p = np.sqrt(mu / rData)
    vHat = vN / vData
    vVt2 = vN + vHat * (v1p - vData)
    # Update state manager's velocity
    velRef.setState(vVt2)

    # Run thruster burn visualization along with attitude pointing modes
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)
    runSensorSciencePointing(P1/4-burnTime)
    runAntennaEarthPointing(P1/4-burnTime)
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)

    # Retrieve the latest relative position and velocity components
    rN = scRec.r_BN_N[-1] - astRec.PositionVector[-1]
    vN = scRec.v_BN_N[-1] - astRec.VelocityVector[-1]

    # Conduct a second Hohmann transfer from r1 to r2, initial burn
    rData = np.linalg.norm(rN)
    vData = np.linalg.norm(vN)
    at = (rData + r2) * .5
    v2p = np.sqrt((2 * mu / rData) - (mu / at))
    vHat = vN / vData
    vVt = vN + vHat * (v2p - vData)
    # Update state manager's velocity
    velRef.setState(vVt)

    # Run thruster burn section with science pointing mode
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)
    runSensorSciencePointing(P12/2-burnTime*2)
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)

    # Retrieve the latest relative position and velocity components
    rN = scRec.r_BN_N[-1] - astRec.PositionVector[-1]
    vN = scRec.v_BN_N[-1] - astRec.VelocityVector[-1]

    # Conduct the second burn of the second transfer to a cicular orbit at r2
    rData = np.linalg.norm(rN)
    vData = np.linalg.norm(vN)
    v3p = np.sqrt(mu / rData)
    vHat = vN / vData
    vVt = vN + vHat * (v3p - vData)
    # Update state manager's velocity
    velRef.setState(vVt)

    # Run thruster visualization with science pointing mode
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)
    runSensorSciencePointing(P2-burnTime)

    # Retrieve the latest relative position and velocity components
    rN = scRec.r_BN_N[-1] - astRec.PositionVector[-1]
    vN = scRec.v_BN_N[-1] - astRec.VelocityVector[-1]

    # Conduct a third Hohmann transfer from r2 to r3, initial burn
    rData = np.linalg.norm(rN)
    vData = np.linalg.norm(vN)
    at = (rData + r3) * .5
    v3p = np.sqrt((2 * mu / rData) - (mu / at))
    vHat = vN / vData
    vVt = vN + vHat * (v3p - vData)
    # Update state manager's velocity
    velRef.setState(vVt)

    # Run thruster visualization with science-pointing mode
    runDvBurn(burnTime, -1, velAsteroidGuidance.attRefOutMsg)
    runSensorSciencePointing(3*P23-burnTime)

    # Retrieve logged spacecraft position relative to asteroid
    posData1 = scRec.r_BN_N  # inertial pos. wrt. Sun
    posData2 = scRec.r_BN_N - astRec.PositionVector  # relative pos. wrt. Asteroid

    # Call plotting function: plotOrbits
    figureList = plotOrbits(scRec.times(), posData1, posData2, rP, diam)

    if show_plots:
        plt.show()

    # Close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    # Unload Spice kernels
    gravFactory.unloadSpiceKernels()

    return figureList


def plotOrbits(timeAxis, posData1, posData2, rP, diam):
    fileName = os.path.basename(os.path.splitext(__file__)[0])

    plt.close("all")  # Clears out plots from earlier test runs
    figureList = {}

    # Plot arrival to Asteroid
    plt.figure(1, figsize=(5, 5))
    # Draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    ax.set_aspect('equal')
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    planetColor = '#008800'
    planetRadius = .5*(diam) # m
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))

    # Draw the actual orbit from pulled data (DataRec)
    plt.plot(posData2[:, 0], posData2[:, 2], color='orangered',
             label='Simulated Flight')
    plt.xlabel('X Distance, Inertial [m]')
    plt.ylabel('Z Distance, Inertial [m]')

    # Draw desired parking orbit
    fData = np.linspace(0, 2 * np.pi, 100)
    rData = []
    for indx in range(0, len(fData)):
        rData.append(rP)
    plt.plot(rData* np.cos(fData), rData * np.sin(fData), '--', color='#555555', label='Desired Circ.Capture Orbit')
    plt.legend(loc='upper right')
    plt.grid()
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    return figureList


if __name__ == "__main__":
    run(
        True  # show_plots
    )
