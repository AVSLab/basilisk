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

This scenario demonstrates the use of the :ref:`sensorThermal` module, which models a sensor as a flat plate of solid
material with an insulated backing. An optional power input can be used if the sensor consumes power, which is transferred
to heat. The sensor radiates heat to the outside environment, and takes in heat from the sun based on its incidence
angle.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioSensorThermal.py

In this scenario, the :ref:`locationPointing`, :ref:`mrpFeedback`, and :ref:`extForceTorque` modules are used to
point the sensor. In the first orbital period, the sensor is pointed directly at the sun, heating it up. In the second
orbital period, the sensor is pointed opposite of the sun, cooling it.


Illustration of Simulation Results
----------------------------------
The illustration of these results may be found below, which show the temperature in celsius over the length of the
simulation.
::

    show_plots = True

The following plots illustrate the temperature of the sensor.

.. image:: /_images/Scenarios/scenario_ThermalSensor.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  The purpose of this scenario script is to demonstrate the use of the sensorThermal module, which models
#           the temperature of a sensor.
# Author:   Adam Herrmann
# Creation Date:  December 13th, 2022
#

import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import simpleNav
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import sensorThermal

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import locationPointing

# import message declarations
from Basilisk.architecture import messaging

# attempt to import vizard
from Basilisk.utilities import vizSupport
try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
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

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Set the simulation time variable used later on
    simulationTime = macros.min2nano(20.)

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    # Define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # Add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # Create the gravFactory
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # Create the sun
    gravFactory.createSun()

    # Set up Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # Attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Create the spice interface
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/'
                                     , '2021 MAY 04 07:47:48.965 (UTC)'
                                     )
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject, ModelPriority=100)

    # Set up the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 600)*1000.  # meters
    oe.e = 0.01
    oe.i = 63.3 * macros.D2R
    oe.Omega = 88.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 135.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # Set up extForceTorque module
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject, 97)

    # Add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject, ModelPriority=101)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Create the ephemeris converter
    ephemConverter = ephemerisConverter.EphemerisConverter()
    ephemConverter.ModelTag = "ephemConverter"
    ephemConverter.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[0])
    ephemConverter.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[1])
    scSim.AddModelToTask(simTaskName, ephemConverter, ModelPriority=100)

    # Set up sun pointing guidance module
    locPointConfig = locationPointing.locationPointingConfig()
    locPointWrap = scSim.setModelDataWrap(locPointConfig)
    locPointWrap.ModelTag = "locPoint"
    scSim.AddModelToTask(simTaskName, locPointWrap, locPointConfig, ModelPriority=99)
    locPointConfig.pHat_B = [0, 0, 1]
    locPointConfig.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    locPointConfig.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    locPointConfig.celBodyInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0])

    # Set up the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig, ModelPriority=98)
    mrpControlConfig.guidInMsg.subscribeTo(locPointConfig.attGuidOutMsg)
    mrpControlConfig.K = 5.5
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.P = 30.0
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # Connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)

    # Now add the thermal sensor module
    thermalSensor = sensorThermal.SensorThermal()
    thermalSensor.T_0 = 0  # Celsius
    thermalSensor.nHat_B = [0, 0, 1]
    thermalSensor.sensorArea = 1.0  # m^2
    thermalSensor.sensorAbsorptivity = 0.25
    thermalSensor.sensorEmissivity = 0.34
    thermalSensor.sensorMass = 2.0  # kg
    thermalSensor.sensorSpecificHeat = 890
    thermalSensor.sensorPowerDraw = 30.0  # W
    thermalSensor.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    thermalSensor.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, thermalSensor)

    # Setup logging on the power system
    tempLog = thermalSensor.temperatureOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, tempLog)

    # Create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    configDataMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    mrpControlConfig.vehConfigInMsg.subscribeTo(configDataMsg)

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    scSim.ConfigureStopTime(macros.sec2nano(int(P)))        # seconds to stop simulation

    # Begin the simulation time run set above
    scSim.ExecuteSimulation()

    # Change the location pointing vector and run the sim for another period
    locPointConfig.pHat_B = [0, 0, -1]
    scSim.ConfigureStopTime(macros.sec2nano(int(2*P)))        # seconds to stop simulation
    scSim.ExecuteSimulation()

    # Pull the temperature data
    tempData = tempLog.temperature

    tvec = tempLog.times()
    tvec = tvec * macros.NANO2HOUR

    #   Plot the power states
    figureList = {}
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    plt.plot(tvec*60., tempData)
    plt.xlabel('Time (min)')
    plt.ylabel('Temperature (deg C)')
    plt.grid(True)

    pltName = "scenario_thermalSensor"
    figureList[pltName] = plt.figure(1)

    if show_plots:
        plt.show()
    plt.close("all")

    return figureList

# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    run(
        True  # show_plots
    )

