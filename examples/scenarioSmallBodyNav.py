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

This scenario demonstrates how to use the ``smallBodyNavEKF()`` for state estimation about a small body. In this example,
Bennu is used. However, any small body could be selected as long as the appropriate gravitational parameter is set.

In this scenario, :ref:`simpleNav` and :ref:`planetEphemeris` provide measurements to the EKF in the form of :ref:`navTransMsgPayload`,
:ref:`navAttMsgPayload`, and :ref:`ephemerisMsgPayload` input messages. The EKF takes in these measurements at each timestep
and updates the state estimate, outputting this state estimate in its own standalone message, a :ref:`smallBodyNavMsgPayload`,
as well as navigation output messages - :ref:`navTransMsgPayload` and :ref:`ephemerisMsgPayload`.

.. note:: This module is only meant to provide a somewhat representative autonomous small body proximity operations navigation solution for POMDP solvers. Therefore, realistic measurement modules do not exist to support this module, and not every source of uncertainty in the problem is an estimated parameter.

.. attention::

    To see the asteroid Bennu in Vizard the asteroid asset bundle must be installed.  See
    the Vizard `Download <http://hanspeterschaub.info/basilisk/Vizard/VizardDownload.html>`__ web page.

The relative position estimate and the estimation error and covariance may be found in the plots below.

.. image:: /_images/Scenarios/scenarioSmallBodyNav1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNav3.svg
   :align: center

Likewise, the relative velocity estimate and the estimation error and covariance may be found in the plots below.

.. image:: /_images/Scenarios/scenarioSmallBodyNav2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNav4.svg
   :align: center

In the next four plots, the attitude and rate estimates and error plots of the small body frame with respect to the
 inertial frame are displayed.

.. image:: /_images/Scenarios/scenarioSmallBodyNav5.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNav6.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNav7.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNav8.svg
   :align: center

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioSmallBodyNav.py


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Example simulation to demonstrate the use of the smallBodyNavEKF
# Author:   Adam Herrmann
# Creation Date:  July 14th, 2021
#

import math
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import hillPoint
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import rwMotorTorque
from Basilisk.fswAlgorithms import smallBodyNavEKF
from Basilisk.fswAlgorithms import smallBodyWaypointFeedback
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import planetEphemeris
from Basilisk.simulation import planetNav
from Basilisk.simulation import extForceTorque
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
def plot_position(time, meas_time, r_BO_O_truth, r_BO_O_est, r_BO_O_meas):
    """Plot the relative position result."""
    #plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(meas_time, r_BO_O_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(meas_time, r_BO_O_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(meas_time, r_BO_O_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, r_BO_O_truth[:, 0], label='${}^Or_{BO_{1}}$')
    ax[1].plot(time, r_BO_O_truth[:, 1], label='${}^Or_{BO_{2}}$')
    ax[2].plot(time, r_BO_O_truth[:, 2], label='${}^Or_{BO_{3}}$')

    ax[0].plot(time, r_BO_O_est[:, 0], label='estimate')
    ax[1].plot(time, r_BO_O_est[:, 1])
    ax[2].plot(time, r_BO_O_est[:, 2])

    plt.xlabel('Time [sec]')
    plt.title('Relative Spacecraft Position')

    ax[0].set_ylabel('${}^Or_{BO_1}$ [m]')
    ax[1].set_ylabel('${}^Or_{BO_2}}$ [m]')
    ax[2].set_ylabel('${}^Or_{BO_3}$ [m]')

    ax[0].legend()

    return


def plot_velocity(time, meas_time, v_BO_O_truth, v_BO_O_est, v_BO_O_meas):
    """Plot the relative velocity result."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(meas_time, v_BO_O_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(meas_time, v_BO_O_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(meas_time, v_BO_O_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, v_BO_O_truth[:, 0], label='truth')
    ax[1].plot(time, v_BO_O_truth[:, 1])
    ax[2].plot(time, v_BO_O_truth[:, 2])

    ax[0].plot(time, v_BO_O_est[:, 0], label='estimate')
    ax[1].plot(time, v_BO_O_est[:, 1])
    ax[2].plot(time, v_BO_O_est[:, 2])

    plt.xlabel('Time [sec]')
    plt.title('Relative Spacecraft Velocity')

    ax[0].set_ylabel('${}^Ov_{BO_1}$ [m/s]')
    ax[1].set_ylabel('${}^Ov_{BO_2}}$ [m/s]')
    ax[2].set_ylabel('${}^Ov_{BO_3}$ [m/s]')

    ax[0].legend()

    return


def plot_pos_error(time, r_err, P):
    """Plot the position estimation error and associated covariance."""
    # plt.figure(3)
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, r_err[:, 0], label='error')
    ax[0].plot(time, 2*np.sqrt(P[:, 0, 0]), 'k--', label=r'$2\sigma$')
    ax[0].plot(time, -2*np.sqrt(P[:, 0, 0]), 'k--')

    ax[1].plot(time, r_err[:, 1])
    ax[1].plot(time, 2*np.sqrt(P[:, 1, 1]), 'k--')
    ax[1].plot(time, -2*np.sqrt(P[:, 1, 1]), 'k--')

    ax[2].plot(time, r_err[:, 2])
    ax[2].plot(time, 2*np.sqrt(P[:, 2, 2]), 'k--')
    ax[2].plot(time, -2*np.sqrt(P[:, 2, 2]), 'k--')

    plt.xlabel('Time [sec]')
    plt.title('Position Error and Covariance')

    ax[0].set_ylabel('${}^Or_{BO_1}$ Error [m]')
    ax[1].set_ylabel('${}^Or_{BO_2}}$ Error [m]')
    ax[2].set_ylabel('${}^Or_{BO_3}$ Error [m]')

    ax[0].legend()

    return


def plot_vel_error(time, v_err, P):
    """Plot the position estimation error and associated covariance."""
    # plt.figure(4)
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, v_err[:, 0], label='error')
    ax[0].plot(time, 2*np.sqrt(P[:, 3, 3]), 'k--', label=r'$2\sigma$')
    ax[0].plot(time, -2*np.sqrt(P[:, 3, 3]), 'k--')

    ax[1].plot(time, v_err[:, 1])
    ax[1].plot(time, 2*np.sqrt(P[:, 4, 4]), 'k--')
    ax[1].plot(time, -2*np.sqrt(P[:, 4, 4]), 'k--')

    ax[2].plot(time, v_err[:, 2])
    ax[2].plot(time, 2*np.sqrt(P[:, 5, 5]), 'k--')
    ax[2].plot(time, -2*np.sqrt(P[:, 5, 5]), 'k--')

    plt.xlabel('Time [sec]')
    plt.title('Velocity Error and Covariance')

    ax[0].set_ylabel('${}^Ov_{BO_1}$ Error [m/s]')
    ax[1].set_ylabel('${}^Ov_{BO_2}}$ Error [m/s]')
    ax[2].set_ylabel('${}^Ov_{BO_3}$ Error [m/s]')

    ax[0].legend()

    return


def plot_sc_att(time, meas_time, sigma_BN_truth, sigma_BN_est, sigma_BN_meas):
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(meas_time, sigma_BN_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(meas_time, sigma_BN_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(meas_time, sigma_BN_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, sigma_BN_truth[:, 0], label='truth')
    ax[1].plot(time, sigma_BN_truth[:, 1])
    ax[2].plot(time, sigma_BN_truth[:, 2])

    ax[0].plot(time, sigma_BN_est[:, 0], label='estimate')
    ax[1].plot(time, sigma_BN_est[:, 1])
    ax[2].plot(time, sigma_BN_est[:, 2])

    plt.xlabel('Time [sec]')

    ax[0].set_ylabel(r'$\sigma_{BN_1}$ [rad]')
    ax[1].set_ylabel(r'$\sigma_{BN_2}$ [rad]')
    ax[2].set_ylabel(r'$\sigma_{BN_3}$ [rad]')

    ax[0].legend()

    return


def plot_sc_rate(time, meas_time, omega_BN_B_truth, omega_BN_B_est, omega_BN_B_meas):
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(meas_time, omega_BN_B_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(meas_time, omega_BN_B_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(meas_time, omega_BN_B_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, omega_BN_B_truth[:, 0], label='truth')
    ax[1].plot(time, omega_BN_B_truth[:, 1])
    ax[2].plot(time, omega_BN_B_truth[:, 2])

    ax[0].plot(time, omega_BN_B_est[:, 0], label='estimate')
    ax[1].plot(time, omega_BN_B_est[:, 1])
    ax[2].plot(time, omega_BN_B_est[:, 2])

    plt.xlabel('Time [sec]')

    ax[0].set_ylabel(r'${}^B\omega_{BN_{1}}$ [rad/s]')
    ax[1].set_ylabel(r'${}^B\omega_{BN_{2}}$ [rad/s]')
    ax[2].set_ylabel(r'${}^B\omega_{BN_{3}}$ [rad/s]')

    ax[0].legend()

    return


def plot_ast_att(time, meas_time, sigma_AN_truth, sigma_AN_est, sigma_AN_meas):
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(meas_time, sigma_AN_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(meas_time, sigma_AN_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(meas_time, sigma_AN_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, sigma_AN_truth[:, 0], label='truth')
    ax[1].plot(time, sigma_AN_truth[:, 1])
    ax[2].plot(time, sigma_AN_truth[:, 2])

    ax[0].plot(time, sigma_AN_est[:, 0], label='estimate')
    ax[1].plot(time, sigma_AN_est[:, 1])
    ax[2].plot(time, sigma_AN_est[:, 2])

    plt.xlabel('Time [sec]')

    ax[0].set_ylabel(r'$\sigma_{AN_{1}}$ [rad]')
    ax[1].set_ylabel(r'$\sigma_{AN_{2}}$ [rad]')
    ax[2].set_ylabel(r'$\sigma_{AN_{3}}$ [rad]')

    ax[0].legend()

    return


def plot_ast_rate(time, meas_time, omega_AN_A_truth, omega_AN_A_est, omega_AN_A_meas):
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(meas_time, omega_AN_A_meas[:, 0], 'k*', label='measurement', markersize=1)
    ax[1].plot(meas_time, omega_AN_A_meas[:, 1], 'k*', markersize=1)
    ax[2].plot(meas_time, omega_AN_A_meas[:, 2], 'k*', markersize=1)

    ax[0].plot(time, omega_AN_A_truth[:, 0], label='truth')
    ax[1].plot(time, omega_AN_A_truth[:, 1])
    ax[2].plot(time, omega_AN_A_truth[:, 2])

    ax[0].plot(time, omega_AN_A_est[:, 0], label='estimate')
    ax[1].plot(time, omega_AN_A_est[:, 1])
    ax[2].plot(time, omega_AN_A_est[:, 2])

    ax[0].set_ylabel(r'${}^A\omega_{AN_{1}}$ [rad/s]')
    ax[1].set_ylabel(r'${}^A\omega_{AN_{2}}$ [rad/s]')
    ax[2].set_ylabel(r'${}^A\omega_{AN_{3}}$ [rad/s]')

    plt.xlabel('Time [sec]')

    ax[0].legend()

    return


def plot_ast_attitude_error(time, sigma_err, P):
    """Plot the asteroid attitude estimation error and associated covariance."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, sigma_err[:, 0], label='error')
    ax[0].plot(time, 2*np.sqrt(P[:, 6, 6]), 'k--', label=r'$2\sigma$')
    ax[0].plot(time, -2*np.sqrt(P[:, 6, 6]), 'k--')

    ax[1].plot(time, sigma_err[:, 1])
    ax[1].plot(time, 2*np.sqrt(P[:, 7, 7]), 'k--')
    ax[1].plot(time, -2*np.sqrt(P[:, 7, 7]), 'k--')

    ax[2].plot(time, sigma_err[:, 2])
    ax[2].plot(time, 2*np.sqrt(P[:, 8, 8]), 'k--')
    ax[2].plot(time, -2*np.sqrt(P[:, 8, 8]), 'k--')

    plt.xlabel('Time [sec]')
    plt.title('Attitude Error and Covariance')

    ax[0].set_ylabel(r'$\sigma_{AN_{1}}$ Error [rad]')
    ax[1].set_ylabel(r'$\sigma_{AN_{2}}$ Error [rad]')
    ax[2].set_ylabel(r'$\sigma_{AN_{3}}$ Error [rad]')

    ax[0].legend()

    return


def plot_ast_rate_error(time, omega_err, P):
    """Plot the asteroid rate estimation error and associated covariance."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time, omega_err[:, 0], label='error')
    ax[0].plot(time, 2*np.sqrt(P[:, 9, 9]), 'k--', label=r'$2\sigma$')
    ax[0].plot(time, -2*np.sqrt(P[:, 9, 9]), 'k--')

    ax[1].plot(time, omega_err[:, 1])
    ax[1].plot(time, 2*np.sqrt(P[:, 10, 10]), 'k--')
    ax[1].plot(time, -2*np.sqrt(P[:, 10, 10]), 'k--')

    ax[2].plot(time, omega_err[:, 2])
    ax[2].plot(time, 2*np.sqrt(P[:, 11, 11]), 'k--')
    ax[2].plot(time, -2*np.sqrt(P[:, 11, 11]), 'k--')

    plt.xlabel('Time [sec]')
    plt.title('Position Error and Covariance')

    ax[0].set_ylabel(r'${}^A\omega_{AN_{1}}$ Error [rad/s]')
    ax[1].set_ylabel(r'${}^A\omega_{AN_{2}}$ Error [rad/s]')
    ax[2].set_ylabel(r'${}^A\omega_{AN_{3}}$ Error [rad/s]')

    ax[0].legend()

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
    measTaskName = "measTask"
    fswTaskName = "fswTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the simulation time step information
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep, 3))
    dynProcess.addTask(scSim.CreateNewTask(measTaskName, simulationTimeStep, 2))
    dynProcess.addTask(scSim.CreateNewTask(fswTaskName, simulationTimeStep, 1))

    # setup celestial object ephemeris module
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
    # specify celestial object orientation
    gravBodyEphem.rightAscension = planetEphemeris.DoubleVector([86.6388 * macros.D2R])
    gravBodyEphem.declination = planetEphemeris.DoubleVector([-65.1086 * macros.D2R])
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
    asteroid = gravFactory.createCustomGravObject("bennu", mu
                                                  , modelDictionaryKey="Bennu"
                                                  , radEquator=565. / 2.0
                                                  )
    asteroid.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])

    # create SC object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Create the position and velocity of states of the s/c wrt the small body hill frame
    r_BO_N = np.array([2000., 1500., 1000.]) # Position of the spacecraft relative to the body
    v_BO_N = np.array([1., 1., 1.])  # Velocity of the spacecraft relative to the body

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
    RW1 = rwFactory.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100.  # RPM
                           )
    RW2 = rwFactory.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200.  # RPM
                           )
    RW3 = rwFactory.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300.  # RPM
                           )

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    rwConfigMsg = rwFactory.getConfigMessage()

    # Create a zero'd out thruster message
    thrusterMsgData = messaging.THROutputMsgPayload()
    thrusterMsg = messaging.THROutputMsg()
    thrusterMsg.write(thrusterMsgData)

    # Create an SRP model
    srp = radiationPressure.RadiationPressure()  # default model is the SRP_CANNONBALL_MODEL
    srp.area = 3.  # m^3
    srp.coefficientReflection = 0.9
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
    pos_sigma_sc = 40.0
    vel_sigma_sc = 0.05
    att_sigma_sc = 0. * math.pi / 180.0
    rate_sigma_sc = 0. * math.pi / 180.0
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
    walk_bounds_sc = [[10.], [10.], [10.], [0.01], [0.01], [0.01], [0.005], [0.005], [0.005], [0.002], [0.002], [0.002], [0.], [0.], [0.], [0.], [0.], [0.]]
    simpleNavMeas.PMatrix = p_matrix_sc
    simpleNavMeas.walkBounds = walk_bounds_sc

    simpleNavMeas2 = simpleNav.SimpleNav()
    simpleNavMeas2.ModelTag = 'SimpleNav2'
    simpleNavMeas2.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    simpleNavMeas2.PMatrix = p_matrix_sc
    simpleNavMeas2.walkBounds = walk_bounds_sc

    # Set up planetNav for Bennu "measurements"
    planetNavMeas = planetNav.PlanetNav()
    planetNavMeas.ephemerisInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0])
    # Define the Pmatrix for planetNav, no uncertainty on position and velocity of the body
    pos_sigma_p = 0.0
    vel_sigma_p = 0.0
    att_sigma_p = 1.0 * math.pi / 180.0
    rate_sigma_p = 0.1 * math.pi / 180.0
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

    # Sun pointing configuration
    sunPointData = hillPoint.hillPointConfig()
    sunPointWrap = scSim.setModelDataWrap(sunPointData)
    sunPointWrap.ModelTag = "sunPoint"
    sunPointData.celBodyInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0])

    # Attitude error configuration
    trackingErrorData = attTrackingError.attTrackingErrorConfig()
    trackingErrorWrap = scSim.setModelDataWrap(trackingErrorData)
    trackingErrorWrap.ModelTag = "trackingError"
    trackingErrorData.attRefInMsg.subscribeTo(sunPointData.attRefOutMsg)

    # Specify the vehicle configuration message to tell things what the vehicle inertia is
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I
    vcConfigMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    #   Attitude controller configuration
    mrpFeedbackControlData = mrpFeedback.mrpFeedbackConfig()
    mrpFeedbackControlWrap = scSim.setModelDataWrap(mrpFeedbackControlData)
    mrpFeedbackControlWrap.ModelTag = "mrpFeedbackControl"
    mrpFeedbackControlData.guidInMsg.subscribeTo(trackingErrorData.attGuidOutMsg)
    mrpFeedbackControlData.vehConfigInMsg.subscribeTo(vcConfigMsg)
    mrpFeedbackControlData.K = 7.
    mrpFeedbackControlData.Ki = -1.0
    mrpFeedbackControlData.P = 35.
    mrpFeedbackControlData.integralLimit = 2. / mrpFeedbackControlData.Ki * 0.1

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
    rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
    rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueConfig.rwMotorTorqueOutMsg)
    rwMotorTorqueConfig.rwParamsInMsg.subscribeTo(rwConfigMsg)
    rwMotorTorqueConfig.vehControlInMsg.subscribeTo(mrpFeedbackControlData.cmdTorqueOutMsg)
    rwMotorTorqueConfig.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwMotorTorqueConfig.rwMotorTorqueOutMsg)

    # Create the Lyapunov feedback controller
    waypointFeedback = smallBodyWaypointFeedback.SmallBodyWaypointFeedback()
    waypointFeedback.asteroidEphemerisInMsg.subscribeTo(planetNavMeas.ephemerisOutMsg)
    waypointFeedback.sunEphemerisInMsg.subscribeTo(sunEphemerisMsg)
    waypointFeedback.navAttInMsg.subscribeTo(simpleNavMeas2.attOutMsg)
    waypointFeedback.A_sc = 1.  # Surface area of the spacecraft, m^2
    waypointFeedback.M_sc = mass  # Mass of the spacecraft, kg
    waypointFeedback.IHubPntC_B = unitTestSupport.np2EigenMatrix3d(I)  # sc inertia
    waypointFeedback.mu_ast = mu  # Gravitational constant of the asteroid
    waypointFeedback.x1_ref = [-2000., 0., 0.]
    waypointFeedback.x2_ref = [0.0, 0.0, 0.0]

    extForceTorqueModule = extForceTorque.ExtForceTorque()
    extForceTorqueModule.cmdForceBodyInMsg.subscribeTo(waypointFeedback.forceOutMsg)
    scObject.addDynamicEffector(extForceTorqueModule)

    # Set up the small body EKF
    smallBodyNav = smallBodyNavEKF.SmallBodyNavEKF()
    smallBodyNav.ModelTag = "smallBodyNavEKF"

    # Set the filter parameters (sc area, mass, gravitational constants, etc.)
    smallBodyNav.A_sc = 1.  # Surface area of the spacecraft, m^2
    smallBodyNav.M_sc = mass  # Mass of the spacecraft, kg
    smallBodyNav.mu_ast = mu  # Gravitational constant of the asteroid

    # Set the process noise
    Q = np.zeros((12,12))
    Q[0,0] = Q[1,1] = Q[2,2] = 0.0000001
    Q[3,3] = Q[4,4] = Q[5,5] = 0.000001
    Q[6,6] = Q[7,7] = Q[8,8] = 0.000001
    Q[9,9] = Q[10,10] = Q[11,11] = 0.0000001
    smallBodyNav.Q = Q.tolist()

    # Set the measurement noise
    R = np.zeros((12,12))
    R[0,0] = R[1,1] = R[2,2] = pos_sigma_sc  # position sigmas
    R[3,3] = R[4,4] = R[5,5] = vel_sigma_sc   # velocity sigmas
    R[6,6] = R[7,7] = R[8,8] = att_sigma_p
    R[9,9] = R[10,10] = R[11,11] = rate_sigma_p
    smallBodyNav.R = np.multiply(R, R).tolist()  # Measurement Noise

    # Set the initial guess, x_0
    x_0 = np.zeros(18)
    x_0[0:3] = np.array([2458., -704.08, 844.275])
    x_0[3:6] = np.array([1.475, -0.176, 0.894])
    x_0[6:9] = np.array([-0.58, 0.615, 0.125])
    x_0[11] = 0.0004
    smallBodyNav.x_hat_k = x_0
    # Set the covariance to something large
    smallBodyNav.P_k = (0.1*np.identity(12)).tolist()

    # Connect the relevant modules to the smallBodyEKF input messages
    smallBodyNav.navTransInMsg.subscribeTo(simpleNavMeas.transOutMsg)
    smallBodyNav.navAttInMsg.subscribeTo(simpleNavMeas2.attOutMsg)
    smallBodyNav.asteroidEphemerisInMsg.subscribeTo(planetNavMeas.ephemerisOutMsg)
    smallBodyNav.sunEphemerisInMsg.subscribeTo(sunEphemerisMsg)
    smallBodyNav.cmdForceBodyInMsg.subscribeTo(waypointFeedback.forceOutMsg)
    smallBodyNav.addThrusterToFilter(thrusterMsg)

    # Connect the smallBodyEKF output messages to the relevant modules
    trackingErrorData.attNavInMsg.subscribeTo(simpleNavMeas2.attOutMsg)
    sunPointData.transNavInMsg.subscribeTo(simpleNavMeas2.transOutMsg)
    waypointFeedback.navTransInMsg.subscribeTo(smallBodyNav.navTransOutMsg)

    # Set the waypoint feedback gains
    waypointFeedback.K1 = unitTestSupport.np2EigenMatrix3d([5e-4, 0e-5, 0e-5, 0e-5, 5e-4, 0e-5, 0e-5, 0e-5, 5e-4])
    waypointFeedback.K2 = unitTestSupport.np2EigenMatrix3d([1., 0., 0., 0., 1., 0., 0., 0., 1.])

    # Add all models to the task
    scSim.AddModelToTask(simTaskName, scObject, ModelPriority=100)
    scSim.AddModelToTask(simTaskName, srp, ModelPriority=99)
    scSim.AddModelToTask(simTaskName, gravBodyEphem, ModelPriority=99)
    scSim.AddModelToTask(simTaskName, rwStateEffector, ModelPriority=91)
    scSim.AddModelToTask(simTaskName, extForceTorqueModule, ModelPriority=91)
    scSim.AddModelToTask(simTaskName, simpleNavMeas2, ModelPriority=90)
    scSim.AddModelToTask(simTaskName, ephemConverter, ModelPriority=98)

    scSim.AddModelToTask(measTaskName, simpleNavMeas, ModelPriority=97)
    scSim.AddModelToTask(measTaskName, planetNavMeas, ModelPriority=96)

    scSim.AddModelToTask(fswTaskName, smallBodyNav, ModelPriority=90)
    scSim.AddModelToTask(fswTaskName, waypointFeedback, ModelPriority=89)
    scSim.AddModelToTask(fswTaskName, sunPointWrap, sunPointData, ModelPriority=95)
    scSim.AddModelToTask(fswTaskName, trackingErrorWrap, trackingErrorData, ModelPriority=94)
    scSim.AddModelToTask(fswTaskName, mrpFeedbackControlWrap, mrpFeedbackControlData, ModelPriority=93)
    scSim.AddModelToTask(fswTaskName, rwMotorTorqueWrap, rwMotorTorqueConfig, ModelPriority=92)

    #   Setup data logging before the simulation is initialized
    sc_truth_recorder = scObject.scStateOutMsg.recorder()
    ast_truth_recorder = gravBodyEphem.planetOutMsgs[0].recorder()
    ast_ephemeris_recorder = ephemConverter.ephemOutMsgs[0].recorder()
    ast_ephemeris_meas_recorder = planetNavMeas.ephemerisOutMsg.recorder()
    state_recorder = smallBodyNav.smallBodyNavOutMsg.recorder()
    sc_meas_recorder = simpleNavMeas.transOutMsg.recorder()
    sc_att_meas_recorder = simpleNavMeas.attOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, sc_truth_recorder)
    scSim.AddModelToTask(simTaskName, ast_truth_recorder)
    scSim.AddModelToTask(fswTaskName, state_recorder)
    scSim.AddModelToTask(measTaskName, sc_meas_recorder)
    scSim.AddModelToTask(measTaskName, sc_att_meas_recorder)
    scSim.AddModelToTask(simTaskName, ast_ephemeris_recorder)
    scSim.AddModelToTask(measTaskName, ast_ephemeris_meas_recorder)

    if vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  # , saveFile=fileName
                                                  )
        viz.settings.showSpacecraftLabels = 1

    #   initialize Simulation
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    simulationTime = macros.sec2nano(1000.0)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(1000.))
    scSim.disableTask(measTaskName)
    scSim.ExecuteSimulation()

    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(2000.))
    scSim.enableTask(measTaskName)
    scSim.ExecuteSimulation()

    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(3000.))
    scSim.disableTask(measTaskName)
    scSim.ExecuteSimulation()

    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(4000.))
    scSim.enableTask(measTaskName)
    scSim.ExecuteSimulation()

    # retrieve logged spacecraft position relative to asteroid
    r_BN_N_truth = sc_truth_recorder.r_BN_N
    r_BN_N_meas = sc_meas_recorder.r_BN_N
    v_BN_N_truth = sc_truth_recorder.v_BN_N
    v_BN_N_meas = sc_meas_recorder.v_BN_N
    r_AN_N = ast_truth_recorder.PositionVector
    v_AN_N = ast_truth_recorder.VelocityVector
    sigma_AN_truth = ast_ephemeris_recorder.sigma_BN
    omega_AN_A_truth = ast_ephemeris_recorder.omega_BN_B
    sigma_AN_meas = ast_ephemeris_meas_recorder.sigma_BN
    omega_AN_A_meas = ast_ephemeris_meas_recorder.omega_BN_B
    x_hat = state_recorder.state
    P = state_recorder.covar

    time = sc_truth_recorder.times() * macros.NANO2SEC
    meas_time = sc_meas_recorder.times() * macros.NANO2SEC

    # Compute the relative position and velocity of the s/c in the small body hill frame
    r_BO_O_truth = []
    v_BO_O_truth = []
    r_BO_O_meas = []
    v_BO_O_meas = []
    for rd_N, vd_N, rc_N, vc_N in zip(r_BN_N_truth, v_BN_N_truth, r_AN_N, v_AN_N):
        dcm_ON = orbitalMotion.hillFrame(rc_N, vc_N)

        r_BO_O_truth.append(np.matmul(dcm_ON, rd_N-rc_N))
        v_BO_O_truth.append(np.matmul(dcm_ON, vd_N-vc_N))

    for idx, t in enumerate(meas_time):
        truth_idx = np.where(time == t)[0][0]

        rc_N = r_AN_N[truth_idx, :]
        vc_N = v_AN_N[truth_idx, :]
        rd_N_meas = r_BN_N_meas[idx, :]
        vd_N_meas = v_BN_N_meas[idx, :]

        dcm_ON = orbitalMotion.hillFrame(rc_N, vc_N)

        r_BO_O_meas.append(np.matmul(dcm_ON, rd_N_meas-rc_N))
        v_BO_O_meas.append(np.matmul(dcm_ON, vd_N_meas-vc_N))

    #
    #   plot the results
    #
    plot_position(time, meas_time, np.array(r_BO_O_truth), x_hat[:,0:3], np.array(r_BO_O_meas))
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_velocity(time, meas_time, np.array(v_BO_O_truth), x_hat[:,3:6], np.array(v_BO_O_meas))
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_pos_error(time, np.subtract(r_BO_O_truth, x_hat[:,0:3]), P)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_vel_error(time, np.subtract(v_BO_O_truth,x_hat[:,3:6]), P)
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plot_ast_att(time, meas_time, np.array(sigma_AN_truth), x_hat[:,6:9], np.array(sigma_AN_meas))
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

    plot_ast_rate(time, meas_time, np.array(omega_AN_A_truth), x_hat[:,9:12], np.array(omega_AN_A_meas))
    pltName = fileName + "6"
    figureList[pltName] = plt.figure(6)

    plot_ast_attitude_error(time, np.subtract(sigma_AN_truth, x_hat[:,6:9]), P)
    pltName = fileName + "7"
    figureList[pltName] = plt.figure(7)

    plot_ast_rate_error(time, np.subtract(omega_AN_A_truth, x_hat[:,9:12]), P)
    pltName = fileName + "8"
    figureList[pltName] = plt.figure(8)

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
