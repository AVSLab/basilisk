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

The purpose of this simulation is to illustrate how to set a spacecraft's translational motion using a custom
Spice file for planetary flybys. This allows the user to easily visualize a mission trajectory using Vizard.
Attitude pointing modes are also implemented in this script to enhance the mission simulation and illustrate
other capabilities in Basilisk.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioFlybySpice.py

Configuring Translational Motion Using Custom Spice Files
---------------------------------------------------------

This simulation script allows the user to specify a flyby of either Venus, Mars, or Earth. Note that the custom Spice
file for this scenario incorporates translational information for all three planetary flybys. Therefore, the user must
first specify the correct flyby date and time to initialize the simulation with the correct Spice information. This is
done using the ``timeIntString`` variable::

    if planetCase == "venus":
        timeInitString = "2028 August 13 0:30:30.0"
    elif planetCase == "earth":
        timeInitString = "2029 June 20 5:30:30.0"
    elif planetCase == "mars":
        timeInitString = "2031 October 3 20:00:00.0"
    else:
        print("flyby target not implemented.")
        exit(1)

Next the custom Spice file must be loaded. The ``loadSpiceKernel()`` method of class ``SpiceInterface``
is called to load the file. This method accepts the file name and the path to the desired file to load::

    gravFactory.spiceObject.loadSpiceKernel("max_21T01.bsp", os.path.join(path, "dataForExamples", "Spice/"))

Note that setting up the orbital elements and initial conditions using the ``orbitalMotion`` module is no longer needed.

After the Spice file is loaded, connect the configured Spice translational output message to the spacecraft object's
``transRefInMsg`` input message::

    scObject.transRefInMsg.subscribeTo(gravFactory.spiceObject.transRefStateOutMsgs[0])

Finally, add the Spice object to the simulation task list::

    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)

Implementing Attitude Pointing Modes
------------------------------------

Now that the spacecraft's translational motion is set, the user is free to implement attitude changes to enhance the
mission visualization. Three attitude pointing modes are incorporated into this script for each planetary flyby case.

First, an ``ephemerisConverter`` module must be configured to convert Spice messages of type ``plantetStateOutMsgs``
to ephemeris messages of type ``ephemOutMsgs``. This converter is required for all attitude pointing modules::

    ephemObject = ephemerisConverter.EphemerisConverter()
    ephemObject.ModelTag = 'EphemData'
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[earthIdx])
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[sunIdx])
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[moonIdx])
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[venusIdx])
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[marsIdx])
    scSim.AddModelToTask(simTaskName, ephemObject)

Next, the ``extForceTorque`` and ``simpleNav`` modules must be set configured::

    extFTObject = extForceTorque.ExtForceTorque()
    sNavObject = simpleNav.SimpleNav()

The torque module object is added to the spacecraft as a dynamics effector and the simple navigation module's spacecraft
state input message must be subscribed to the spacecraft object's state output message::

    scObject.addDynamicEffector(extFTObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

Both modules are added to the simulation task::

    scSim.AddModelToTask(simTaskName, extFTObject)
    scSim.AddModelToTask(simTaskName, sNavObject)

Next the ``velocityPoint`` module is configured for each planet case. This module fixes the spacecraft attitude in the
orbit velocity frame. See the `velocityPoint Module
<https://hanspeterschaub.info/basilisk/Documentation/fswAlgorithms/attGuidance/velocityPoint/velocityPoint.html?highlight=velocity%20point>`__
page for a more detailed description of this module. The Mars velocity-pointing case is shown below::

    velMarsGuidanceConfig = velocityPoint.velocityPointConfig()
    velMarsGuidanceWrap = scSim.setModelDataWrap(velMarsGuidanceConfig)
    velMarsGuidanceWrap.ModelTag = "velocityPointMars"
    velMarsGuidanceConfig.mu = marsMu

The velocity pointing module has two input messages that must be connected. First, the module's
``transNavInMsg`` message is subscribed to the simple navigation module's ``transOutMsg`` message. The module's
``celBodyInMsg`` message must be connected to the flyby planet's ephemeris output message::

    velMarsGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    velMarsGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[marsIdx])

Finally, add the module to the simulation task list::

    scSim.AddModelToTask(simTaskName, velMarsGuidanceWrap, velMarsGuidanceConfig)

The other attitude guidance modules used in this simulation are implemented in similar manner. The ``locationPointing``
module points a body-fixed spacecraft axis towards a particular location of interest. Modes for Earth- and Sun-pointing
are implemented by subscribing the module's ``celBodyInMsg`` input message to the desired planet's ephemeris output
message. The module's ``pHat_B`` vector is set according to which body-fixed vector should point towards the location
of interest. See the `locationPointing Module
<https://hanspeterschaub.info/basilisk/Documentation/fswAlgorithms/attGuidance/locationPointing/locationPointing.html?highlight=location%20pointing>`__
page for a more detailed description of this module. The Earth-pointing guidance module setup is shown below::

    earthPointGuidanceConfig = locationPointing.locationPointingConfig()
    earthPointGuidanceWrap = scSim.setModelDataWrap(earthPointGuidanceConfig)
    earthPointGuidanceWrap.ModelTag = "antennaEarthPoint"
    earthPointGuidanceConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    earthPointGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[earthIdx])
    earthPointGuidanceConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    earthPointGuidanceConfig.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    earthPointGuidanceConfig.pHat_B = [0.0, 0.0, 1.0]
    earthPointGuidanceConfig.useBoresightRateDamping = 1
    scSim.AddModelToTask(simTaskName, earthPointGuidanceWrap, earthPointGuidanceConfig)

Next, a science-pointing mode is implemented using the ``hillPoint`` module. This module points a body-fixed location
on the spacecraft designated as a camera towards the flyby planet of interest. See the `hillPoint Module
<https://hanspeterschaub.info/basilisk/Documentation/fswAlgorithms/attGuidance/hillPoint/hillPoint.html?highlight=hill%20point>`__
page for a more detailed description of this module::

    cameraLocation = [0.0, 1.5, 0.0]
    sciencePointGuidanceConfig = hillPoint.hillPointConfig()
    sciencePointGuidanceWrap = scSim.setModelDataWrap(sciencePointGuidanceConfig)
    sciencePointGuidanceWrap.ModelTag = "sciencePointAsteroid"
    sciencePointGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    sciencePointGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[planetIdx])
    scSim.AddModelToTask(simTaskName, sciencePointGuidanceWrap, sciencePointGuidanceConfig)

Next, the attitude tracking error module must be configured with the initial flight mode::

    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.attRefInMsg.subscribeTo(velEarthGuidanceConfig.attRefOutMsg)  # initial flight mode
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

Then, the flight software vehicle configuration message is configured::

    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

The MRP Feedback control module is configured next for attitude control::

    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1.0  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

To complete the feedback loop, the MRP feedback module's ``cmdTorqueOutMsg`` output message is connected to the
external torque module's ``cmdTorqueInMsg``::

    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

Additional Visualization Features
------------------------------------

To add a visualization of antenna transmission back to Earth during the Earth-pointing mode, a transceiver is created
through the ``vizInterface``::

    transceiverHUD = vizInterface.Transceiver()
    transceiverHUD.r_SB_B = [0.23, 0., 1.38]
    transceiverHUD.fieldOfView = 40.0 * macros.D2R
    transceiverHUD.normalVector = [0.0, 0., 1.0]
    transceiverHUD.color = vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
    transceiverHUD.label = "antenna"
    transceiverHUD.animationSpeed = 1

To add a camera to the science-pointing mode, the ``createStandardCamera`` method is used::

    vizSupport.createStandardCamera(viz, setMode=1, spacecraftName=scObject.ModelTag,
                                    fieldOfView=10 * macros.D2R,
                                    pointingVector_B=[0,1,0], position_B=cameraLocation)

After the simulation is initialized, functions are created for each attitude flight mode. Each function accepts the
desired simulation time and executes the simulation after setting additional flight mode parameters. Note that because
the velocity pointing mode depends on which planet is specified, this function is generalized to also accept the
desired guidance configuration module::

    def runVelocityPointing(simTime, planetMsg):
        nonlocal simulationTime
        attErrorConfig.attRefInMsg.subscribeTo(planetMsg)
        transceiverHUD.transceiverState = 0  # antenna off
        attErrorConfig.sigma_R0R = [np.tan(90.*macros.D2R/4), 0, 0]
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

To execute the desired attitude-pointing mode, the ``run`` function must be called with the desired simulation time::

    runVelocityPointing(4*hour, velPlantConfig.attRefOutMsg)

Ensure to unload the Spice kernel at the end of each simulation::

    gravFactory.spiceObject.unloadSpiceKernel("max_21T01.bsp", os.path.join(path, "Data", "Spice/"))


Simulation Visualization In Vizard
----------------------------------

The following image illustrates the expected visualization of this simulation script for a flyby of Mars.

.. image:: /_images/static/scenarioFlybySpice.jpg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose: This simulation illustrates how to use a custom Spice file to specify a spacecraft's translation motion.
# Attitude pointing modes are also implemented for enhanced visualization.
# Author:   Leah Kiner
# Creation Date: February 5 2022
#

import os
import numpy as np
import inspect

from Basilisk import __path__

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation import spacecraft, gravityEffector, extForceTorque, simpleNav, ephemerisConverter
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody, unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import vizSupport

# import general simulation support files
from Basilisk.simulation import vizInterface

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import hillPoint
from Basilisk.fswAlgorithms import mrpFeedback, attTrackingError, velocityPoint, locationPointing


def run(planetCase):
    """
        At the end of the python script you can specify the following example parameters.

        Args:
            planetCase (str): {'venus', 'earth', 'mars'}
    """
    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Configure the simulation
    scSim = SimulationBaseClass.SimBaseClass()

    # Shows the simulation progress bar in the terminal
    scSim.SetProgressBar(True)

    # Create the dynamics process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(2.)

    # Add the dynamics task to the dynamics process
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Configure the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spiceSat"  # Name of the spacecraft

    # Create gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createBodies(["earth", "sun", "moon", "venus", "mars barycenter"])
    earth = gravFactory.gravBodies["earth"]
    earth.isCentralBody = True
    scObject.gravField.setGravBodies(gravityEffector.GravBodyVector(list(gravFactory.gravBodies.values())))

    # Define planet gravitational parameters needed for the attitude pointing modes
    earthMu = earth.mu
    venusMu = gravFactory.gravBodies["venus"].mu
    marsMu = gravFactory.gravBodies["mars barycenter"].mu

    # Set planet index values
    earthIdx = 0
    sunIdx = 1
    moonIdx = 2
    venusIdx = 3
    marsIdx = 4

    # Next, the default SPICE support module is created and configured.  The first step is to store
    # the date and time of the start of the simulation.
    if planetCase == "venus":
        timeInitString = "2028 August 13 0:30:30.0"
    elif planetCase == "earth":
        timeInitString = "2029 June 20 5:30:30.0"
    elif planetCase == "mars":
        timeInitString = "2031 October 3 20:00:00.0"
    else:
        print("flyby target not implemented.")
        exit(1)

    # Create the Spice interface and add the correct path to the ephemeris data
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'Earth'

    # Specify Spice spacecraft name
    scNames = ["-60000"]
    gravFactory.spiceObject.addSpacecraftNames(messaging.StringVector(scNames))

    # Load the custom spacecraft trajectory Spice file using the SpiceInterface class loadSpiceKernel() method
    gravFactory.spiceObject.loadSpiceKernel("spacecraft_21T01.bsp", os.path.join(path, "dataForExamples", "Spice/"))

    # Connect the configured Spice translational output message to spacecraft object's transRefInMsg input message
    scObject.transRefInMsg.subscribeTo(gravFactory.spiceObject.transRefStateOutMsgs[0])

    # Add the Spice and spacecraft objects to the simulation task list.
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)
    scSim.AddModelToTask(simTaskName, scObject)

    # Create an ephemeris converter to convert Spice messages of type plantetStateOutMsgs to ephemeris messages
    # of type ephemOutMsgs. This converter is required for the velocityPoint and locationPointing modules.
    ephemObject = ephemerisConverter.EphemerisConverter()
    ephemObject.ModelTag = 'EphemData'
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[earthIdx])
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[sunIdx])
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[moonIdx])
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[venusIdx])
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[marsIdx])
    scSim.AddModelToTask(simTaskName, ephemObject)

    # Define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.000], [-0.00], [0.00]]  # rad/s - omega_BN_B

    # Set up extForceTorque module
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # Add the simple Navigation sensor module.  This sets the SC attitude, rate, position,
    # and velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    #
    #   setup the FSW algorithm tasks
    #

    # Set up Venus relative velocityPoint guidance module
    velVenusGuidanceConfig = velocityPoint.velocityPointConfig()
    velVenusGuidanceWrap = scSim.setModelDataWrap(velVenusGuidanceConfig)
    velVenusGuidanceWrap.ModelTag = "velocityPointVenus"
    velVenusGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    velVenusGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[venusIdx])
    velVenusGuidanceConfig.mu = venusMu
    scSim.AddModelToTask(simTaskName, velVenusGuidanceWrap, velVenusGuidanceConfig)

    # Set up Earth relative velocityPoint guidance module
    velEarthGuidanceConfig = velocityPoint.velocityPointConfig()
    velEarthGuidanceWrap = scSim.setModelDataWrap(velEarthGuidanceConfig)
    velEarthGuidanceWrap.ModelTag = "velocityPointEarth"
    velEarthGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    velEarthGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[earthIdx])
    velEarthGuidanceConfig.mu = earthMu
    scSim.AddModelToTask(simTaskName, velEarthGuidanceWrap, velEarthGuidanceConfig)

    # Set up Mars relative velocityPoint guidance module
    velMarsGuidanceConfig = velocityPoint.velocityPointConfig()
    velMarsGuidanceWrap = scSim.setModelDataWrap(velMarsGuidanceConfig)
    velMarsGuidanceWrap.ModelTag = "velocityPointMars"
    velMarsGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    velMarsGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[marsIdx])
    velMarsGuidanceConfig.mu = marsMu
    scSim.AddModelToTask(simTaskName, velMarsGuidanceWrap, velMarsGuidanceConfig)

    if planetCase == "venus":
        velPlantConfig = velVenusGuidanceConfig
        planetIdx = venusIdx
    elif planetCase == "earth":
        velPlantConfig = velEarthGuidanceConfig
        planetIdx = earthIdx
    elif planetCase == "mars":
        velPlantConfig = velMarsGuidanceConfig
        planetIdx = marsIdx
    else:
        print("flyby target not implemented.")
        exit(1)

    # Set up the Earth antenna-pointing guidance module
    earthPointGuidanceConfig = locationPointing.locationPointingConfig()
    earthPointGuidanceWrap = scSim.setModelDataWrap(earthPointGuidanceConfig)
    earthPointGuidanceWrap.ModelTag = "antennaEarthPoint"
    earthPointGuidanceConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    earthPointGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[earthIdx])
    earthPointGuidanceConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    earthPointGuidanceConfig.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    earthPointGuidanceConfig.pHat_B = [0.0, 0.0, 1.0]
    earthPointGuidanceConfig.useBoresightRateDamping = 1
    scSim.AddModelToTask(simTaskName, earthPointGuidanceWrap, earthPointGuidanceConfig)

    # Set up the solar panel Sun-pointing guidance module
    sunPointGuidanceConfig = locationPointing.locationPointingConfig()
    sunPointGuidanceWrap = scSim.setModelDataWrap(sunPointGuidanceConfig)
    sunPointGuidanceWrap.ModelTag = "panelSunPoint"
    sunPointGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[sunIdx])
    sunPointGuidanceConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    sunPointGuidanceConfig.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    sunPointGuidanceConfig.pHat_B = [0.0, 0.0, 1.0]
    sunPointGuidanceConfig.useBoresightRateDamping = 1
    scSim.AddModelToTask(simTaskName, sunPointGuidanceWrap, sunPointGuidanceConfig)

    # Set up the sensor science-pointing guidance module
    cameraLocation = [0.0, 1.5, 0.0]
    sciencePointGuidanceConfig = hillPoint.hillPointConfig()
    sciencePointGuidanceWrap = scSim.setModelDataWrap(sciencePointGuidanceConfig)
    sciencePointGuidanceWrap.ModelTag = "sciencePointAsteroid"
    sciencePointGuidanceConfig.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    sciencePointGuidanceConfig.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[planetIdx])
    scSim.AddModelToTask(simTaskName, sciencePointGuidanceWrap, sciencePointGuidanceConfig)

    # Set up the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)
    attErrorConfig.attRefInMsg.subscribeTo(velEarthGuidanceConfig.attRefOutMsg)  # initial flight mode
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # Create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Set up the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_Feedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControlConfig.K = 3.5
    mrpControlConfig.Ki = -1.0  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # Connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    # Set the initial simulation time
    simulationTime = macros.sec2nano(0)

    # Set up antenna transmission to Earth visualization
    transceiverHUD = vizInterface.Transceiver()
    transceiverHUD.r_SB_B = [0.23, 0., 1.38]
    transceiverHUD.fieldOfView = 40.0 * macros.D2R
    transceiverHUD.normalVector = [0.0, 0., 1.0]
    transceiverHUD.color = vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
    transceiverHUD.label = "antenna"
    transceiverHUD.animationSpeed = 1

    # Configure vizard settings
    vizFile = os.path.realpath(__file__).strip(".py") + "_" + planetCase + ".py"
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=vizFile
                                              , transceiverList=transceiverHUD)
    viz.epochInMsg.subscribeTo(gravFactory.epochMsg)
    viz.settings.orbitLinesOn = -1
    viz.settings.keyboardAngularRate = np.deg2rad(0.5)

    vizSupport.createStandardCamera(viz, setMode=1, spacecraftName=scObject.ModelTag,
                                    fieldOfView=10 * macros.D2R,
                                    pointingVector_B=[0,1,0], position_B=cameraLocation)

    # Initialize and execute simulation for the first section (stops at periapsis of hyperbola before delta V)
    scSim.InitializeSimulation()

    # Set up flight modes
    def runVelocityPointing(simTime, planetMsg):
        nonlocal simulationTime
        attErrorConfig.attRefInMsg.subscribeTo(planetMsg)
        transceiverHUD.transceiverState = 0  # antenna off
        attErrorConfig.sigma_R0R = [np.tan(90.*macros.D2R/4), 0, 0]
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    def runAntennaEarthPointing(simTime):
        nonlocal simulationTime
        attErrorConfig.attRefInMsg.subscribeTo(earthPointGuidanceConfig.attRefOutMsg)
        transceiverHUD.transceiverState = 3  # antenna in send and receive mode
        attErrorConfig.sigma_R0R = [0, 0, 0]
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    def runPanelSunPointing(simTime):
        nonlocal simulationTime
        attErrorConfig.attRefInMsg.subscribeTo(sunPointGuidanceConfig.attRefOutMsg)
        transceiverHUD.transceiverState = 0  # antenna off
        attErrorConfig.sigma_R0R = [0, 0, 0]
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    def runSensorSciencePointing(simTime):
        nonlocal simulationTime
        attErrorConfig.attRefInMsg.subscribeTo(sciencePointGuidanceConfig.attRefOutMsg)
        transceiverHUD.transceiverState = 0  # antenna off
        attErrorConfig.sigma_R0R = [-1./3., 1./3., -1./3.]
        simulationTime += macros.sec2nano(simTime)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

    hour = 60*60

    # Execute desired attitude flight modes
    runVelocityPointing(4*hour, velPlantConfig.attRefOutMsg)

    runAntennaEarthPointing(4*hour)

    runSensorSciencePointing(12*hour)

    runAntennaEarthPointing(4*hour)

    runPanelSunPointing(4*hour)

    # Unload custom Spice kernel at the end of each simulation
    gravFactory.unloadSpiceKernels()
    gravFactory.spiceObject.unloadSpiceKernel("spacecraft_21T01.bsp", os.path.join(path, "Data", "Spice/"))

    return

    
if __name__ == "__main__":
    run(
        "mars"   # venus, earth, mars
    )

