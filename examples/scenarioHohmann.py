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

This Example simulates a single Hohmann transfer about earth. The spacecraft switches between a Hill frame
and an inertial frame aligned with the burn direction, although alignment with the burn is purely visual and the delta V
will be applied in te correct direction regardless of the attitude.

Compared to :ref:`scenarioOrbitManeuver` which also implements impulsive orbit maneuvers, here a set
of pointing modes are defined.  The Basilisk event system is used to then switch between the flight modes
by selectively turning on/off flight mode tasks.

The detail of the simulation script is as follows. This script sets up a basic spacecraft which starts on a circular
orbit around Earth aligned with a Hill reference frame. The spacecraft aligns with the orbit trajectory shortly before
delta V's are applied at the beginning and end of the transfer and returns to the Hill frame for the duration of the
simulation.

The required delta V is found by taking the difference between the velocity on the elliptical transfer orbit, found
using

.. math::
    V_{transfer} = \sqrt{\frac{2\mu}{r} - \frac{\mu}{a_{transfer}}}

and that of the circular orbit, defined as

.. math::
    V = \sqrt{\frac{\mu}{r}}

The delta V is then added to the current spacecraft velocity. This calculation is done at each transfer point using the
respective radius (inner or outer).

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioHohmann.py

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

Plots below illustrate

.. image:: /_images/Scenarios/scenarioHohmann1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioHohmann2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioHohmann3.svg
    :align: center

.. image:: /_images/Scenarios/scenarioHohmann4.svg
    :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing how to set up a Hohmann transfer and use reaction wheels to change the spacecraft's
#           reference attitude
# Author:   Peter Johnson
# Creation Date:  Sep. 6, 2023
#

import os

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d as plt3
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import mrpFeedback, attTrackingError, velocityPoint, hillPoint, mrpRotation, rwMotorTorque
from Basilisk.simulation import reactionWheelStateEffector, simpleNav, spacecraft, ephemerisConverter
from Basilisk.utilities import (SimulationBaseClass, fswSetupRW, macros, orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def plotOrbit(rFirst, rSecond, posData):
    # plot the earth
    ax = plt.axes(projection='3d')
    planetColor = '#008800'
    planetRadius = 6378.1
    u, v = np.mgrid[0:2 * np.pi:40j, 0:np.pi:40j]
    x = planetRadius * np.cos(u) * np.sin(v)
    y = planetRadius * np.sin(u) * np.sin(v)
    z = planetRadius * np.cos(v)
    if rSecond > rFirst:
        ax.set_xlim3d(-1.1 * rSecond / 1000, 1.1 * rSecond / 1000)
        ax.set_ylim3d(-1.1 * rSecond / 1000, 1.1 * rSecond / 1000)
        ax.set_zlim3d(-1.1 * rSecond / 1000, 1.1 * rSecond / 1000)
    else:
        ax.set_xlim3d(-1.1 * rFirst / 1000, 1.1 * rFirst / 1000)
        ax.set_ylim3d(-1.1 * rFirst / 1000, 1.1 * rFirst / 1000)
        ax.set_zlim3d(-1.1 * rFirst / 1000, 1.1 * rFirst / 1000)

    ax.plot_surface(x, y, z, color=planetColor)
    # plot the orbit
    ax.plot3D(posData[:, 0] / 1000, posData[:, 1] / 1000, posData[:, 2] / 1000,
              color='orangered', label='Simulated Flight')
    ax.set_xlabel('x [km]')
    ax.set_ylabel('y [km]')
    ax.set_zlabel('z [km]')
    ax.set_aspect('equal')


def plotAttitudeError(timeAxis, attErrorData):
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attErrorData[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='best')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')


def plotAttitude(timeAxis, attData):
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attData[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='best')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude $\sigma_{B/N}$')


def plotReferenceAttitude(timeAxis, attRefData):
    plt.figure(4)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, attRefData[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='best')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference Attitude $\sigma_{R/N}$')


def run(show_plots, rFirst, rSecond):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        rFirst (double): radius of the initial circular orbit about Earth
        rSecond (double): radius of the final circular orbit about Earth

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "dynProcess"
    fwsProcessName = "fswProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  Create the simulation process
    scSim.dynProcess = scSim.CreateNewProcess(simProcessName, 0)
    scSim.fswProcess = scSim.CreateNewProcess(fwsProcessName, 0)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1)
    scSim.dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep), 10)

    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # Add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, None, 1)

    # Set up Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # Override information with SPICE
    timeInitString = "2021 MAY 04 07:47:48.965 (UTC)"
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'Earth'
    gravFactory.addBodiesTo(scObject)

    # Add ephemeris for Hill frame
    ephemObject = ephemerisConverter.EphemerisConverter()
    ephemObject.ModelTag = "ephem data"
    ephemObject.addSpiceInputMsg(gravFactory.spiceObject.planetStateOutMsgs[0])
    scSim.AddModelToTask(simTaskName, ephemObject, None, 1)

    # Add RW devices
    rwFactory = simIncludeRW.rwFactory()
    RW1 = rwFactory.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100.)
    RW2 = rwFactory.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200.)
    RW3 = rwFactory.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300.)
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(rwStateEffector.ModelTag, rwStateEffector, scObject)
    scSim.AddModelToTask(simTaskName, rwStateEffector, None, 2)

    # Add the navigation sensor module. This sets the SC attitude, rate, position and velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    # Create guidance message to connect 3 flight modes to attitude error
    attRefMsg = messaging.AttRefMsg_C()
    attRefMsg.write(messaging.AttRefMsgPayload())

    # Set up velocity-point guidance module for first burn
    firstBurn = velocityPoint.velocityPoint()
    firstBurn.ModelTag = "velocityPoint"
    firstBurn.mu = mu
    scSim.fswProcess.addTask(scSim.CreateNewTask("firstBurnTask", simulationTimeStep), 5)
    scSim.AddModelToTask("firstBurnTask", firstBurn)

    firstBurnMRPRotation = mrpRotation.mrpRotation()
    firstBurnMRPRotation.ModelTag = "mrpRotation"
    sigma_RR0 = np.array([np.tan(- np.pi / 8), 0, 0])
    firstBurnMRPRotation.mrpSet = sigma_RR0
    scSim.AddModelToTask("firstBurnTask", firstBurnMRPRotation)
    messaging.AttRefMsg_C_addAuthor(firstBurnMRPRotation.attRefOutMsg, attRefMsg)

    # setup velocity-point guidance module for second burn
    secondBurn = velocityPoint.velocityPoint()
    secondBurn.ModelTag = "velocityPoint"
    secondBurn.mu = mu
    scSim.fswProcess.addTask(scSim.CreateNewTask("secondBurnTask", simulationTimeStep), 5)
    scSim.AddModelToTask("secondBurnTask", secondBurn)

    # Need to get this reference attitude to rotate 180
    secondBurnMRPRotation = mrpRotation.mrpRotation()
    secondBurnMRPRotation.ModelTag = "mrpRotation"
    sigma_RR0 = np.array([np.tan(np.pi / 8), 0, 0])
    secondBurnMRPRotation.mrpSet = sigma_RR0
    scSim.AddModelToTask("secondBurnTask", secondBurnMRPRotation)
    messaging.AttRefMsg_C_addAuthor(secondBurnMRPRotation.attRefOutMsg, attRefMsg)

    # Set up hill point guidance module
    attGuidanceHillPoint = hillPoint.hillPoint()
    attGuidanceHillPoint.ModelTag = "hillPoint"
    scSim.fswProcess.addTask(scSim.CreateNewTask("hillPointTask", simulationTimeStep), 5)
    scSim.AddModelToTask("hillPointTask", attGuidanceHillPoint)
    messaging.AttRefMsg_C_addAuthor(attGuidanceHillPoint.attRefOutMsg, attRefMsg)

    # Create flight software task to hold the remaining modules
    fswTaskName = "fswTask"
    scSim.fswProcess.addTask(scSim.CreateNewTask(fswTaskName, simulationTimeStep), 0)

    # Set up the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attError"
    scSim.AddModelToTask(fswTaskName, attError)

    # Set up the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(fswTaskName, mrpControl)
    mrpControl.K = 3.5
    mrpControl.P = 30.0
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.integralLimit = -1

    # Add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B
    scSim.AddModelToTask(fswTaskName, rwMotorTorqueObj)

    # Create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Create the FSW reaction wheel configuration message
    fswSetupRW.clearSetup()
    fswRwParamMsg = rwFactory.getConfigMessage()

    # Set up the spacecraft's initial condition
    oe = orbitalMotion.ClassicElements()
    oe.a = rFirst
    oe.e = 0
    oe.i = 30 * macros.D2R
    oe.Omega = 0
    oe.omega = 0
    oe.f = 0
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # Link and subscribe messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    firstBurn.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    firstBurn.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[0])
    firstBurnMRPRotation.attRefInMsg.subscribeTo(firstBurn.attRefOutMsg)
    secondBurn.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    secondBurn.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[0])
    secondBurnMRPRotation.attRefInMsg.subscribeTo(secondBurn.attRefOutMsg)
    attGuidanceHillPoint.transNavInMsg.subscribeTo(sNavObject.transOutMsg)
    attGuidanceHillPoint.celBodyInMsg.subscribeTo(ephemObject.ephemOutMsgs[0])
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(attRefMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    # Set up data logging before the simulation is initialized
    samplingTime = macros.sec2nano(0.5)
    scRec = scObject.scStateOutMsg.recorder(samplingTime)
    attLog = sNavObject.attOutMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    attRefLog = attRefMsg.recorder(samplingTime)
    scSim.AddModelToTask(fswTaskName, scRec)
    scSim.AddModelToTask(fswTaskName, attLog)
    scSim.AddModelToTask(fswTaskName, attErrorLog)
    scSim.AddModelToTask(fswTaskName, attRefLog)

    # Create tje attitude events (three different reference attitudes are required for the sim)
    scSim.createNewEvent("firstBurnEvent", simulationTimeStep, True,
                         ["self.modeRequest == 'firstBurn'"],
                         ["self.fswProcess.disableAllTasks()",
                          "self.enableTask('firstBurnTask')",
                          "self.enableTask('fswTask')",
                          "self.setAllButCurrentEventActivity('firstBurnEvent', True, useIndex=True)"])

    scSim.createNewEvent("secondBurnEvent", simulationTimeStep, True,
                         ["self.modeRequest == 'secondBurn'"],
                         ["self.fswProcess.disableAllTasks()",
                          "self.enableTask('secondBurnTask')",
                          "self.enableTask('fswTask')",
                          "self.setAllButCurrentEventActivity('secondBurnEvent', True, useIndex=True)"])

    scSim.createNewEvent("hillPointEvent", simulationTimeStep, True,
                         ["self.modeRequest == 'hillPoint'"],
                         ["self.fswProcess.disableAllTasks()",
                          "self.enableTask('hillPointTask')",
                          "self.enableTask('fswTask')",
                          "self.setAllButCurrentEventActivity('hillPointEvent', True, useIndex=True)"])

    # Set the simulation time variable and the period of the first orbit
    simulationTime = 0
    P1 = 2 * np.pi * np.sqrt(oe.a ** 3 / mu)

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    scSim.InitializeSimulation()
    scSim.ShowExecutionOrder()
    scSim.SetProgressBar(True)

    # Start in Hill Point and run 40% of a period on the initial circular orbit
    scSim.modeRequest = 'hillPoint'
    simulationTime += macros.sec2nano(0.4 * P1)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Change to first burn orientation in preparation for burn
    scSim.modeRequest = 'firstBurn'
    simulationTime += macros.sec2nano(0.1 * P1)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Get access to translational states
    velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)
    rN = scRec.r_BN_N[-1]
    vN = scRec.v_BN_N[-1]

    # Conduct the first burn of a Hohmann transfer from rFirst to rSecond
    rData = np.linalg.norm(rN)
    vData = np.linalg.norm(vN)
    at = (rData + rSecond) * .5
    vt1 = np.sqrt((2 * mu / rData) - (mu / at))
    Delta_V_1 = vt1 - vData
    vHat = vN / np.linalg.norm(vN)
    new_v = vN + Delta_V_1 * vHat
    velRef.setState(new_v)

    # Define the transfer time
    time_Transfer = np.pi * np.sqrt(at ** 3 / mu)

    # Continue the simulation with new delta v
    simulationTime += macros.sec2nano(0.2 * time_Transfer)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Change back to hill point for 60% of transfer orbit
    scSim.modeRequest = 'hillPoint'
    simulationTime += macros.sec2nano(0.6 * time_Transfer)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Change to second burn orientation in preparation for second burn
    scSim.modeRequest = 'secondBurn'
    simulationTime += macros.sec2nano(0.2 * time_Transfer)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Get the latest velocity and position vectors
    rN = scRec.r_BN_N[-1]
    vN = scRec.v_BN_N[-1]

    # Conduct the second burn of a Hohmann transfer from rFirst to rSecond
    rData = np.linalg.norm(rN)
    vData = np.linalg.norm(vN)
    at = (rData + rSecond) * .5
    vt2 = np.sqrt((2 * mu / rData) - (mu / at))
    Delta_V_2 = vt2 - vData
    vHat = vN / np.linalg.norm(vN)  # unit vec?
    new_v = vN + Delta_V_2 * vHat
    velRef.setState(new_v)

    # Find the period for second orbit
    a2 = rSecond
    P2 = 2 * np.pi * np.sqrt(a2 ** 3 / mu)

    # Continue the simulation with new delta v
    simulationTime += macros.sec2nano(0.1 * P2)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Change back to Hill Point on second orbit
    scSim.modeRequest = 'hillPoint'
    simulationTime += macros.sec2nano(0.2 * P2)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve logged data
    timeAxis = attErrorLog.times()
    posData = scRec.r_BN_N  # inertial pos. wrt. earth (central body)
    attData = attLog.sigma_BN
    attErrorData = attErrorLog.sigma_BR
    attRefData = attRefLog.sigma_RN

    # Plot the results
    plt.close("all")  # clears out plots from earlier test runs
    figureList = {}

    plotOrbit(rFirst, rSecond, posData)
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plotAttitudeError(timeAxis, attErrorData)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plotAttitude(timeAxis, attData)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plotReferenceAttitude(timeAxis, attRefData)
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

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
        True,  # show_plots
        7000000,  # major axis first orbit (m)
        42164000  # major axis second orbit (m)
    )
