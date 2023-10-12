#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario demonstrates how to use the ``smallBodyWaypointFeedback()`` module for waypoint to waypoint motion about a
small body. In this scenario, the spacecraft holds an inertial pointing attitude while it moves to a waypoint defined in the small
body's Hill frame. The :ref:`simpleNav` and :ref:`planetNav` moduels provide measurements to the control law in the form
of :ref:`navTransMsgPayload`, :ref:`navAttMsgPayload`, and :ref:`ephemerisMsgPayload` input messages. The control law
outputs a :ref:`CmdForceBodyMsgPayload`, which is read in by :ref:`extForceTorque`.

The control output in the spacecraft body frame can be found in the following plot:

.. image:: /_images/Scenarios/scenarioSmallBodyFeedbackControl3.svg
   :align: center

The difference between the spacecraft position and velocity and associated references may be found in the figures below:

.. image:: /_images/Scenarios/scenarioSmallBodyFeedbackControl1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyFeedbackControl2.svg
   :align: center

Finally, the attitude and attitude rate is given in the plots below.

.. image:: /_images/Scenarios/scenarioSmallBodyFeedbackControl4.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyFeedbackControl5.svg
   :align: center

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioSmallBodyFeedbackControl.py

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Example simulation to demonstrate the use of the smallBodyWaypointFeedback module
# Author:   Adam Herrmann
# Creation Date:  March 28th, 2022
#

import math
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import rwMotorTorque
from Basilisk.fswAlgorithms import smallBodyWaypointFeedback
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import planetEphemeris
from Basilisk.simulation import planetNav
from Basilisk.simulation import radiationPressure
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, simIncludeGravBody, vizSupport)
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import unitTestSupport

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

# The path to the location of Basilisk
# Used to get the location of supporting data.
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_position(time, r_BO_O_truth, r_BO_O_meas):
    """Plot the relative position result."""
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, r_BO_O_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(time, r_BO_O_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(time, r_BO_O_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, r_BO_O_truth[:, 0], label='${}^Or_{BO_{1}}$')
    ax[1].plot(time, r_BO_O_truth[:, 1], label='${}^Or_{BO_{2}}$')
    ax[2].plot(time, r_BO_O_truth[:, 2], label='${}^Or_{BO_{3}}$')

    plt.xlabel('Time [sec]')
    plt.title('Relative Spacecraft Position')

    ax[0].set_ylabel('${}^Or_{BO_1}$ [m]')
    ax[1].set_ylabel('${}^Or_{BO_2}$ [m]')
    ax[2].set_ylabel('${}^Or_{BO_3}$ [m]')

    ax[0].legend()

    return


def plot_velocity(time, v_BO_O_truth, v_BO_O_meas):
    """Plot the relative velocity result."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, v_BO_O_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(time, v_BO_O_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(time, v_BO_O_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, v_BO_O_truth[:, 0], label='truth')
    ax[1].plot(time, v_BO_O_truth[:, 1])
    ax[2].plot(time, v_BO_O_truth[:, 2])

    plt.xlabel('Time [sec]')
    plt.title('Relative Spacecraft Velocity')

    ax[0].set_ylabel('${}^Ov_{BO_1}$ [m/s]')
    ax[1].set_ylabel('${}^Ov_{BO_2}$ [m/s]')
    ax[2].set_ylabel('${}^Ov_{BO_3}$ [m/s]')

    ax[0].legend()

    return


def plot_sc_att(time, sigma_BN_truth, sigma_BN_meas):
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, sigma_BN_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(time, sigma_BN_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(time, sigma_BN_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, sigma_BN_truth[:, 0], label='truth')
    ax[1].plot(time, sigma_BN_truth[:, 1])
    ax[2].plot(time, sigma_BN_truth[:, 2])

    plt.xlabel('Time [sec]')

    ax[0].set_ylabel(r'$\sigma_{BN_1}$ [rad]')
    ax[1].set_ylabel(r'$\sigma_{BN_2}$ [rad]')
    ax[2].set_ylabel(r'$\sigma_{BN_3}$ [rad]')

    ax[0].legend()

    return


def plot_sc_rate(time, omega_BN_B_truth, omega_BN_B_meas):
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, omega_BN_B_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(time, omega_BN_B_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(time, omega_BN_B_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, omega_BN_B_truth[:, 0], label='truth')
    ax[1].plot(time, omega_BN_B_truth[:, 1])
    ax[2].plot(time, omega_BN_B_truth[:, 2])

    plt.xlabel('Time [sec]')

    ax[0].set_ylabel(r'${}^B\omega_{BN_{1}}$ [rad/s]')
    ax[1].set_ylabel(r'${}^B\omega_{BN_{2}}$ [rad/s]')
    ax[2].set_ylabel(r'${}^B\omega_{BN_{3}}$ [rad/s]')

    ax[0].legend()

    return


def plot_control(time, u):
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, u[:, 0], 'k-', markersize=1)
    ax[1].plot(time, u[:, 1], 'k-', markersize=1)
    ax[2].plot(time, u[:, 2], 'k-', markersize=1)

    plt.xlabel('Time [sec]')

    ax[0].set_ylabel(r'$\hat{\mathbf{b}}_1$ control [N]')
    ax[1].set_ylabel(r'$\hat{\mathbf{b}}_2$ control [N]')
    ax[2].set_ylabel(r'$\hat{\mathbf{b}}_3$ control [N]')

    return


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    path = os.path.dirname(os.path.abspath(__file__))

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the simulation time step information
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Setup celestial object ephemeris module
    gravBodyEphem = planetEphemeris.PlanetEphemeris()
    gravBodyEphem.ModelTag = 'planetEphemeris'
    gravBodyEphem.setPlanetNames(planetEphemeris.StringVector(["bennu"]))

    # specify orbits of gravitational bodies
    # https://ssd.jpl.nasa.gov/horizons.cgi#results
    # December 31st, 2018
    oeAsteroid = planetEphemeris.ClassicElementsMsgPayload()
    oeAsteroid.a = 1.1259 * orbitalMotion.AU * 1000  # meters
    oeAsteroid.e = 0.20373
    oeAsteroid.i = 6.0343 * macros.D2R
    oeAsteroid.Omega = 2.01820 * macros.D2R
    oeAsteroid.omega = 66.304 * macros.D2R
    oeAsteroid.f = 346.32 * macros.D2R
    r_ON_N, v_ON_N = orbitalMotion.elem2rv(orbitalMotion.MU_SUN*(1000.**3), oeAsteroid)

    # specify celestial object orbit
    gravBodyEphem.planetElements = planetEphemeris.classicElementVector([oeAsteroid])
    gravBodyEphem.rightAscension = planetEphemeris.DoubleVector([0. * macros.D2R])
    gravBodyEphem.declination = planetEphemeris.DoubleVector([90. * macros.D2R])
    gravBodyEphem.lst0 = planetEphemeris.DoubleVector([0.0 * macros.D2R])
    gravBodyEphem.rotRate = planetEphemeris.DoubleVector([360 * macros.D2R / (4.297461 * 3600.)])

    # setup Sun Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createSun()

    # Create a sun spice message, zero it out, required by srp
    sunPlanetStateMsgData = messaging.SpicePlanetStateMsgPayload()
    sunPlanetStateMsg = messaging.SpicePlanetStateMsg()
    sunPlanetStateMsg.write(sunPlanetStateMsgData)

    # Create a sun ephemeris message, zero it out, required by nav filter
    sunEphemerisMsgData = messaging.EphemerisMsgPayload()
    sunEphemerisMsg = messaging.EphemerisMsg()
    sunEphemerisMsg.write(sunEphemerisMsgData)

    mu = 4.892  # m^3/s^2
    asteroid = gravFactory.createCustomGravObject("bennu", mu)
    asteroid.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])

    # create SC object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Create the position and velocity of states of the s/c wrt the small body hill frame origin
    r_BO_N = np.array([-2000., 1500., 1000.]) # Position of the spacecraft relative to the body
    v_BO_N = np.array([0., 0., 0.])  # Velocity of the spacecraft relative to the body

    # Create the inertial position and velocity of the s/c
    r_BN_N = np.add(r_BO_N, r_ON_N)
    v_BN_N = np.add(v_BO_N, v_ON_N)

    # Set the truth ICs for the spacecraft position and velocity
    scObject.hub.r_CN_NInit = r_BN_N  # m   - r_BN_N
    scObject.hub.v_CN_NInit = v_BN_N  # m/s - v_BN_N

    I = [82.12, 0.0, 0.0, 0.0, 98.40, 0.0, 0.0, 0.0, 121.0]

    mass = 330.  # kg
    scObject.hub.mHub = mass
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # Set the truth ICs for the spacecraft attitude and rate
    scObject.hub.sigma_BNInit = np.array([0.1, 0.0, 0.0])  # rad
    scObject.hub.omega_BN_BInit = np.array([0.1, 0.1, 0.1])  # rad/s

    # Create RWs
    rwFactory = simIncludeRW.rwFactory()

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    RW1 = rwFactory.create('Honeywell_HR16', [1, 0, 0], maxMomentum=100., Omega=100.  # RPM
                           )
    RW2 = rwFactory.create('Honeywell_HR16', [0, 1, 0], maxMomentum=100., Omega=200.  # RPM
                           )
    RW3 = rwFactory.create('Honeywell_HR16', [0, 0, 1], maxMomentum=100., Omega=300.  # RPM
                           )

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    rwConfigMsg = rwFactory.getConfigMessage()

    # Create an SRP model
    srp = radiationPressure.RadiationPressure()  # default model is the SRP_CANNONBALL_MODEL
    srp.area = 1.  # m^3
    srp.coefficientReflection = 1.9
    scObject.addDynamicEffector(srp)
    srp.sunEphmInMsg.subscribeTo(sunPlanetStateMsg)

    # Create an ephemeris converter
    ephemConverter = ephemerisConverter.EphemerisConverter()
    ephemConverter.ModelTag = "ephemConverter"
    ephemConverter.addSpiceInputMsg(gravBodyEphem.planetOutMsgs[0])

    # Set up simpleNav for s/c "measurements"
    simpleNavMeas = simpleNav.SimpleNav()
    simpleNavMeas.ModelTag = 'SimpleNav'
    simpleNavMeas.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    pos_sigma_sc = 30.0
    vel_sigma_sc = 0.01
    att_sigma_sc = 0.1 * math.pi / 180.0
    rate_sigma_sc = 0.05 * math.pi / 180.0
    sun_sigma_sc = 0.0
    dv_sigma_sc = 0.0
    p_matrix_sc = [[pos_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., pos_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., pos_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., vel_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., vel_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., vel_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., att_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., att_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., att_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., rate_sigma_sc, 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rate_sigma_sc, 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rate_sigma_sc, 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., sun_sigma_sc, 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., sun_sigma_sc, 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., sun_sigma_sc, 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., dv_sigma_sc, 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., dv_sigma_sc, 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., dv_sigma_sc]]
    walk_bounds_sc = [[10.], [10.], [10.], [0.001], [0.001], [0.001], [0.005], [0.005], [0.005], [0.002], [0.002], [0.002], [0.], [0.], [0.], [0.], [0.], [0.]]
    simpleNavMeas.PMatrix = p_matrix_sc
    simpleNavMeas.walkBounds = walk_bounds_sc

    # Set up planetNav for Bennu "measurements"
    planetNavMeas = planetNav.PlanetNav()
    planetNavMeas.ephemerisInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0])
    # Define the Pmatrix for planetNav, no uncertainty on position and velocity of the body
    pos_sigma_p = 0.0
    vel_sigma_p = 0.0
    att_sigma_p = 2.0 * math.pi / 180.0
    rate_sigma_p = 0.3 * math.pi / 180.0
    p_matrix_p = [[pos_sigma_p, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., pos_sigma_p, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., pos_sigma_p, 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., vel_sigma_p, 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., vel_sigma_p, 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., vel_sigma_p, 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., att_sigma_p, 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., att_sigma_p, 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., att_sigma_p, 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., rate_sigma_p, 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rate_sigma_p, 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rate_sigma_p]]
    walk_bounds_p = [[0.], [0.], [0.], [0.], [0.], [0.], [0.005], [0.005], [0.005], [0.002], [0.002], [0.002]]
    planetNavMeas.PMatrix = p_matrix_p
    planetNavMeas.walkBounds = walk_bounds_p

    # Inertial pointing
    inertialPoint = inertial3D.inertial3D()
    inertialPoint.ModelTag = "inertialPoint"
    inertialPoint.sigma_R0N = [0.1, 0.0, 0.0]

    # Attitude error configuration
    trackingError = attTrackingError.attTrackingError()
    trackingError.ModelTag = "trackingError"
    trackingError.attRefInMsg.subscribeTo(inertialPoint.attRefOutMsg)

    # Specify the vehicle configuration message to tell things what the vehicle inertia is
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I
    vehicleConfigOut.CoM_B = [0.0, 0.0, 0.0]
    vcConfigMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Attitude controller configuration
    mrpFeedbackControl = mrpFeedback.mrpFeedback()
    mrpFeedbackControl.ModelTag = "mrpFeedbackControl"
    mrpFeedbackControl.guidInMsg.subscribeTo(trackingError.attGuidOutMsg)
    mrpFeedbackControl.vehConfigInMsg.subscribeTo(vcConfigMsg)
    mrpFeedbackControl.K = 7.0
    mrpFeedbackControl.Ki = -1
    mrpFeedbackControl.P = 30.
    mrpFeedbackControl.integralLimit = 2. / mrpFeedbackControl.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(rwConfigMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpFeedbackControl.cmdTorqueOutMsg)
    rwMotorTorqueObj.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)

    # Connect the smallBodyEKF output messages to the relevant modules
    trackingError.attNavInMsg.subscribeTo(simpleNavMeas.attOutMsg)

    # Create the Lyapunov feedback controller
    waypointFeedback = smallBodyWaypointFeedback.SmallBodyWaypointFeedback()
    waypointFeedback.asteroidEphemerisInMsg.subscribeTo(planetNavMeas.ephemerisOutMsg)
    waypointFeedback.sunEphemerisInMsg.subscribeTo(sunEphemerisMsg)
    waypointFeedback.navAttInMsg.subscribeTo(simpleNavMeas.attOutMsg)
    waypointFeedback.navTransInMsg.subscribeTo(simpleNavMeas.transOutMsg)
    waypointFeedback.A_sc = 1.  # Surface area of the spacecraft, m^2
    waypointFeedback.M_sc = mass  # Mass of the spacecraft, kg
    waypointFeedback.IHubPntC_B = unitTestSupport.np2EigenMatrix3d(I)  # sc inertia
    waypointFeedback.mu_ast = mu  # Gravitational constant of the asteroid
    waypointFeedback.x1_ref = [-2000., 0., 0.]
    waypointFeedback.x2_ref = [0.0, 0.0, 0.0]

    extForceTorqueModule = extForceTorque.ExtForceTorque()
    extForceTorqueModule.cmdForceBodyInMsg.subscribeTo(waypointFeedback.forceOutMsg)
    scObject.addDynamicEffector(extForceTorqueModule)

    scSim.AddModelToTask(simTaskName, scObject, 200)
    scSim.AddModelToTask(simTaskName, srp, 199)
    scSim.AddModelToTask(simTaskName, gravBodyEphem, 198)
    scSim.AddModelToTask(simTaskName, rwStateEffector, 197)
    scSim.AddModelToTask(simTaskName, ephemConverter, 197)
    scSim.AddModelToTask(simTaskName, simpleNavMeas, 196)
    scSim.AddModelToTask(simTaskName, planetNavMeas, 195)
    scSim.AddModelToTask(simTaskName, inertialPoint, 108)
    scSim.AddModelToTask(simTaskName, trackingError, 106)
    scSim.AddModelToTask(simTaskName, mrpFeedbackControl, 105)
    scSim.AddModelToTask(simTaskName, extForceTorqueModule, 82)
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj, 81)
    scSim.AddModelToTask(simTaskName, waypointFeedback, 78)

    # Setup data logging before the simulation is initialized
    sc_truth_recorder = scObject.scStateOutMsg.recorder()
    ast_truth_recorder = gravBodyEphem.planetOutMsgs[0].recorder()
    ast_ephemeris_recorder = ephemConverter.ephemOutMsgs[0].recorder()
    ast_ephemeris_meas_recorder = planetNavMeas.ephemerisOutMsg.recorder()
    sc_meas_recorder = simpleNavMeas.transOutMsg.recorder()
    sc_att_meas_recorder = simpleNavMeas.attOutMsg.recorder()
    requested_control_recorder = waypointFeedback.forceOutMsg.recorder()
    attitude_error_recorder = trackingError.attGuidOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, sc_truth_recorder)
    scSim.AddModelToTask(simTaskName, ast_truth_recorder)
    scSim.AddModelToTask(simTaskName, sc_meas_recorder)
    scSim.AddModelToTask(simTaskName, sc_att_meas_recorder)
    scSim.AddModelToTask(simTaskName, ast_ephemeris_recorder)
    scSim.AddModelToTask(simTaskName, ast_ephemeris_meas_recorder)
    scSim.AddModelToTask(simTaskName, requested_control_recorder)
    scSim.AddModelToTask(simTaskName, attitude_error_recorder)

    fileName = 'scenarioSmallBodyFeedbackControl'

    vizInterface = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                            # , saveFile=fileName
                                                            )
    vizSupport.createStandardCamera(vizInterface, setMode=0, bodyTarget='bennu', setView=0)

    # vizInterface.settings.showSpacecraftLabels = 1
    vizInterface.settings.showCSLabels = 1
    vizInterface.settings.planetCSon = 1
    vizInterface.settings.orbitLinesOn = -1

    # initialize Simulation
    scSim.InitializeSimulation()

    simulationTime_1 = macros.sec2nano(15000.0)

    waypointFeedback.K1 = unitTestSupport.np2EigenMatrix3d([5e-4, 0e-5, 0e-5, 0e-5, 5e-4, 0e-5, 0e-5, 0e-5, 5e-4])
    waypointFeedback.K2 = unitTestSupport.np2EigenMatrix3d([1., 0., 0., 0., 1., 0., 0., 0., 1.])

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime_1)
    scSim.ExecuteSimulation()

    # retrieve logged spacecraft position relative to asteroid
    r_BN_N_truth = sc_truth_recorder.r_BN_N
    r_BN_N_meas = sc_meas_recorder.r_BN_N
    v_BN_N_truth = sc_truth_recorder.v_BN_N
    v_BN_N_meas = sc_meas_recorder.v_BN_N
    sigma_BN_truth = sc_truth_recorder.sigma_BN
    sigma_BN_meas = sc_att_meas_recorder.sigma_BN
    omega_BN_B_truth = sc_truth_recorder.omega_BN_B
    omega_BN_B_meas = sc_att_meas_recorder.omega_BN_B
    r_AN_N = ast_truth_recorder.PositionVector
    v_AN_N = ast_truth_recorder.VelocityVector
    u_requested = requested_control_recorder.forceRequestBody

    # Compute the relative position and velocity of the s/c in the small body hill frame
    r_BO_O_truth = []
    v_BO_O_truth = []
    r_BO_O_meas = []
    v_BO_O_meas = []
    np.set_printoptions(precision=15)
    for rd_N, vd_N, rc_N, vc_N, rd_N_meas, vd_N_meas in zip(r_BN_N_truth, v_BN_N_truth, r_AN_N, v_AN_N, r_BN_N_meas, v_BN_N_meas):
        # Truth values
        r_BO_O, v_BO_O = orbitalMotion.rv2hill(rc_N, vc_N, rd_N, vd_N)
        r_BO_O_truth.append(r_BO_O)
        v_BO_O_truth.append(v_BO_O)

        # Measurement values
        r_BO_O, v_BO_O = orbitalMotion.rv2hill(rc_N, vc_N, rd_N_meas, vd_N_meas)
        r_BO_O_meas.append(r_BO_O)
        v_BO_O_meas.append(v_BO_O)

        # print(rd_N)
        # print(vd_N)

    # Plot the results
    time = sc_truth_recorder.times() * macros.NANO2SEC
    plot_position(time, np.array(r_BO_O_truth), np.array(r_BO_O_meas))
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_velocity(time, np.array(v_BO_O_truth), np.array(v_BO_O_meas))
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_control(time, np.array(u_requested))
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_sc_att(time, np.array(sigma_BN_truth), np.array(sigma_BN_meas))
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plot_sc_rate(time, np.array(omega_BN_B_truth), np.array(omega_BN_B_meas))
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

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
        True  # show_plots
    )
