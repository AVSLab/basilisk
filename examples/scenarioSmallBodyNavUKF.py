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

This scenario demonstrates how to use the ``smallBodyNavUKF()`` for translational state and non-Keplerian gravity
acceleration estimation about a small body. In this example, Vesta is chosen (``supportData/LocalGravData/VESTA20H.txt``).
However, any small body could be selected as long as the appropriate gravitational parameter is set.

In this scenario, :ref:`simpleNav` and :ref:`planetEphemeris` provide measurements to the UKF in the form of
:ref:`navTransMsgPayload` and :ref:`ephemerisMsgPayload` input messages. The UKF takes in these measurements at each
timestep and updates the state estimate, outputting this state estimate in its own standalone message, a
:ref:`smallBodyNavUKFMsgPayload`.

.. image:: /_images/static/test_scenarioSmallBodyNavUKF.svg
   :align: center

.. note:: This module is only meant to demonstrate the possibility of estimating non-Keplerian acceleration in a small body environment. Therefore, realistic measurement modules do not exist to support this module, and not every source of uncertainty in the problem is an estimated parameter.

The relative position estimate and the estimation error and covariance may be found in the plots below.

.. image:: /_images/Scenarios/scenarioSmallBodyNavUKF3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNavUKF6.svg
   :align: center

The relative velocity estimate and the estimation error and covariance may be found in the plots below.

.. image:: /_images/Scenarios/scenarioSmallBodyNavUKF4.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNavUKF7.svg
   :align: center

The non-Keplerian acceleration estimate (inhomogeneous gravity field) and the estimation error and covariance may be found in the plots below.

.. image:: /_images/Scenarios/scenarioSmallBodyNavUKF5.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNavUKF8.svg
   :align: center

The spacecraft trajectories as seen from the inertial and asteroid fixed frame may be found in the plots below.

.. image:: /_images/Scenarios/scenarioSmallBodyNavUKF1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSmallBodyNavUKF2.svg
   :align: center

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioSmallBodyNavUKF.py


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Example simulation to demonstrate the use of the smallBodyNavUKF
# Author:   Julio C. Sanchez
# Creation Date:  March 7th, 2022
#

import math
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.fswAlgorithms import smallBodyNavUKF
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import planetEphemeris
from Basilisk.simulation import planetNav
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import (SimulationBaseClass, macros, simIncludeGravBody)
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport

# The path to the location of Basilisk
# Used to get the location of supporting data.
fileName = os.path.basename(os.path.splitext(__file__)[0])

# Plotting functions
def plot_3Dposition(r_truth, title='None'):
    """Plot the relative position in 3D."""
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.plot(r_truth[:, 0] / 1000., r_truth[:, 1]/1000., r_truth[:,2]/1000., 'b')
    if title == 'inertial':
        ax.set_xlabel('${}^{N}r_{x}$ [km]')
        ax.set_ylabel('${}^{N}r_{y}$ [km]')
        ax.set_zlabel('${}^{N}r_{z}$ [km]')
        ax.set_title('Inertial frame')
    elif title == 'asteroid':
        ax.set_xlabel('${}^{A}r_{x}$ [km]')
        ax.set_ylabel('${}^{A}r_{y}$ [km]')
        ax.set_zlabel('${}^{A}r_{z}$ [km]')
        ax.set_title('Small body fixed frame')

def plot_position(time, r_truth, r_est):
    """Plot the relative position result."""
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time/(3600*24), r_truth[:, 0]/1000, 'b', label='truth')
    ax[1].plot(time/(3600*24), r_truth[:, 1]/1000, 'b')
    ax[2].plot(time/(3600*24), r_truth[:, 2]/1000, 'b')

    ax[0].plot(time/(3600*24), r_est[:, 0]/1000, color='orange', label='estimate')
    ax[1].plot(time/(3600*24), r_est[:, 1]/1000, color='orange')
    ax[2].plot(time/(3600*24), r_est[:, 2]/1000, color='orange')

    plt.xlabel('Time [days]')
    plt.title('Spacecraft Position')

    ax[0].set_ylabel('${}^{A}r_{x}$ [km]')
    ax[1].set_ylabel('${}^{A}r_{y}$ [km]')
    ax[2].set_ylabel('${}^{A}r_{z}$ [km]')

    ax[0].legend()

    return

def plot_velocity(time, v_truth, v_est):
    """Plot the relative velocity result."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time/(3600*24), v_truth[:, 0], 'b', label='truth')
    ax[1].plot(time/(3600*24), v_truth[:, 1], 'b')
    ax[2].plot(time/(3600*24), v_truth[:, 2], 'b')

    ax[0].plot(time/(3600*24), v_est[:, 0], color='orange', label='estimate')
    ax[1].plot(time/(3600*24), v_est[:, 1], color='orange')
    ax[2].plot(time/(3600*24), v_est[:, 2], color='orange')

    plt.xlabel('Time [days]')
    plt.title('Spacecraft Velocity')

    ax[0].set_ylabel('${}^{A}v_{x}$ [m/s]')
    ax[1].set_ylabel('${}^{A}v_{y}$ [m/s]')
    ax[2].set_ylabel('${}^{A}v_{z}$ [m/s]')

    ax[0].legend()

    return

def plot_acceleration(time, a_truth, a_est):
    """Plot the non-Keplerian acceleration result."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time/(3600*24), a_truth[:, 0]*1000, 'b', label='truth')
    ax[1].plot(time/(3600*24), a_truth[:, 1]*1000, 'b')
    ax[2].plot(time/(3600*24), a_truth[:, 2]*1000, 'b')

    ax[0].plot(time/(3600*24), a_est[:, 0]*1000, color='orange', label='estimate')
    ax[1].plot(time/(3600*24), a_est[:, 1]*1000, color='orange')
    ax[2].plot(time/(3600*24), a_est[:, 2]*1000, color='orange')

    plt.xlabel('Time [days]')
    plt.title('Inhomogeneous gravity acceleration')

    ax[0].set_ylabel('${}^{A}a_{x}$ [mm/s$^2$]')
    ax[1].set_ylabel('${}^{A}a_{y}$ [mm/s$^2$]')
    ax[2].set_ylabel('${}^{A}a_{z}$ [mm/s$^2$]')

    ax[0].legend()

    return

def plot_pos_error(time, r_err, P):
    """Plot the position estimation error and associated covariance."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time/(3600*24), r_err[:, 0], 'b', label='error')
    ax[0].plot(time/(3600*24), 2*np.sqrt(P[:, 0, 0]), 'k--', label=r'$2\sigma$')
    ax[0].plot(time/(3600*24), -2*np.sqrt(P[:, 0, 0]), 'k--')

    ax[1].plot(time/(3600*24), r_err[:, 1], 'b')
    ax[1].plot(time/(3600*24), 2*np.sqrt(P[:, 1, 1]), 'k--')
    ax[1].plot(time/(3600*24), -2*np.sqrt(P[:, 1, 1]), 'k--')

    ax[2].plot(time/(3600*24), r_err[:, 2], 'b')
    ax[2].plot(time/(3600*24), 2*np.sqrt(P[:, 2, 2]), 'k--')
    ax[2].plot(time/(3600*24), -2*np.sqrt(P[:, 2, 2]), 'k--')

    plt.xlabel('Time [days]')
    plt.title('Position Error and Covariance')

    ax[0].set_ylabel('${}^{A}r_{x}$ [m]')
    ax[1].set_ylabel('${}^{A}r_{y}$ [m]')
    ax[2].set_ylabel('${}^{A}r_{z}$ [m]')

    ax[0].legend()

    return


def plot_vel_error(time, v_err, P):
    """Plot the velocity estimation error and associated covariance."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time/(3600*24), v_err[:, 0], 'b', label='error')
    ax[0].plot(time/(3600*24), 2*np.sqrt(P[:, 3, 3]), 'k--', label=r'$2\sigma$')
    ax[0].plot(time/(3600*24), -2*np.sqrt(P[:, 3, 3]), 'k--')

    ax[1].plot(time/(3600*24), v_err[:, 1], 'b')
    ax[1].plot(time/(3600*24), 2*np.sqrt(P[:, 4, 4]), 'k--')
    ax[1].plot(time/(3600*24), -2*np.sqrt(P[:, 4, 4]), 'k--')

    ax[2].plot(time/(3600*24), v_err[:, 2], 'b')
    ax[2].plot(time/(3600*24), 2*np.sqrt(P[:, 5, 5]), 'k--')
    ax[2].plot(time/(3600*24), -2*np.sqrt(P[:, 5, 5]), 'k--')

    plt.xlabel('Time [days]')
    plt.title('Velocity Error and Covariance')

    ax[0].set_ylabel('${}^{A}v_{x}$ [m/s]')
    ax[1].set_ylabel('${}^{A}v_{y}$ [m/s]')
    ax[2].set_ylabel('${}^{A}v_{z}$ [m/s]')

    ax[0].legend()

    return

def plot_acc_error(time, a_err, P):
    """Plot the non-Keplerian acceleration estimation error and associated covariance."""
    plt.gcf()
    fig, ax = plt.subplots(3, sharex=True, sharey=True, figsize=(12,6))
    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)

    ax[0].plot(time/(3600*24), a_err[:, 0]*1000, 'b', label='error')
    ax[0].plot(time/(3600*24), 2*np.sqrt(P[:, 6, 6])*1000, 'k--', label=r'$2\sigma$')
    ax[0].plot(time/(3600*24), -2*np.sqrt(P[:, 6, 6])*1000, 'k--')

    ax[1].plot(time/(3600*24), a_err[:, 1]*1000, 'b')
    ax[1].plot(time/(3600*24), 2*np.sqrt(P[:, 7, 7])*1000, 'k--')
    ax[1].plot(time/(3600*24), -2*np.sqrt(P[:, 7, 7])*1000, 'k--')

    ax[2].plot(time/(3600*24), a_err[:, 2]*1000, 'b')
    ax[2].plot(time/(3600*24), 2*np.sqrt(P[:, 8, 8])*1000, 'k--')
    ax[2].plot(time/(3600*24), -2*np.sqrt(P[:, 8, 8])*1000, 'k--')

    plt.xlabel('Time [days]')
    plt.title('Acceleration Error and Covariance')

    ax[0].set_ylabel('${}^{A}a_{x}$ [mm/s$^2$]')
    ax[1].set_ylabel('${}^{A}a_{y}$ [mm/s$^2$]')
    ax[2].set_ylabel('${}^{A}a_{z}$ [mm/s$^2$]')

    ax[0].legend()

    return


def run(show_plots):
    from Basilisk import __path__
    bskPath = __path__[0]
    fileName = os.path.basename(os.path.splitext(__file__)[0])

    # create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the simulation time step information
    simulationTimeStep = macros.sec2nano(15)
    simulationTime = macros.sec2nano(4*24*3600.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # setup celestial object ephemeris module
    gravBodyEphem = planetEphemeris.PlanetEphemeris()
    gravBodyEphem.ModelTag = 'vestaEphemeris'
    gravBodyEphem.setPlanetNames(planetEphemeris.StringVector(["vesta"]))

    # specify small body o.e. and rotational state January 21st, 2022
    # https://ssd.jpl.nasa.gov/horizons.cgi#results
    oeAsteroid = planetEphemeris.ClassicElementsMsgPayload()
    oeAsteroid.a = 2.3612 * orbitalMotion.AU * 1000  # meters
    oeAsteroid.e = 0.08823
    oeAsteroid.i = 7.1417*macros.D2R
    oeAsteroid.Omega = 103.8*macros.D2R
    oeAsteroid.omega = 151.1*macros.D2R
    oeAsteroid.f = 7.0315*macros.D2R

    # the rotational state would be prescribed to
    AR = 309.03 * macros.D2R
    dec = 42.23 * macros.D2R
    lst0 = 0 * macros.D2R
    gravBodyEphem.planetElements = planetEphemeris.classicElementVector([oeAsteroid])
    gravBodyEphem.rightAscension = planetEphemeris.DoubleVector([AR])
    gravBodyEphem.declination = planetEphemeris.DoubleVector([dec])
    gravBodyEphem.lst0 = planetEphemeris.DoubleVector([lst0])
    gravBodyEphem.rotRate = planetEphemeris.DoubleVector([360 * macros.D2R / (5.3421 * 3600.)])

    # initialize small body fixed frame dcm w.r.t. inertial and its angular velocity
    dcm_AN = RigidBodyKinematics.euler3232C([AR , np.pi/2 - dec,  lst0])
    omega_AN_A = np.array([0,0,360 * macros.D2R / (5.3421 * 3600.)])

    # setup small body gravity effector (no Sun 3rd perturbation included)
    # https://ssd.jpl.nasa.gov/tools/gravity.html#/vesta
    gravFactory = simIncludeGravBody.gravBodyFactory()
    mu = 17.2882449693*1e9 # m^3/s^2
    asteroid = gravFactory.createCustomGravObject("vesta", mu, radEquator=265*1000)
    asteroid.isCentralBody = True
    nSpherHarm = 14
    asteroid.useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/VESTA20H.txt', nSpherHarm)
    asteroid.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])

    # create an ephemeris converter
    ephemConverter = ephemerisConverter.EphemerisConverter()
    ephemConverter.ModelTag = "ephemConverter"
    ephemConverter.addSpiceInputMsg(gravBodyEphem.planetOutMsgs[0])

    # create SC object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup orbit initial conditions of the sc
    oe = orbitalMotion.ClassicElements()
    oe.a = 500*1000  # meters
    oe.e = 0.001
    oe.i = 175 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    r_CA_A, v_CA_A = orbitalMotion.elem2rv(mu, oe)

    # set the truth ICs for the spacecraft position and velocity
    scObject.hub.r_CN_NInit = dcm_AN.transpose().dot(r_CA_A)
    scObject.hub.v_CN_NInit = dcm_AN.transpose().dot(v_CA_A)

    # set up simpleNav for s/c "measurements"
    simpleNavMeas = simpleNav.SimpleNav()
    simpleNavMeas.ModelTag = 'SimpleNav'
    simpleNavMeas.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    pos_sigma_sc = 10.0
    vel_sigma_sc = 0.1
    att_sigma_sc = 0.0 * math.pi / 180.0
    rate_sigma_sc = 0.0 * math.pi / 180.0
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
    walk_bounds_sc = [[10.], [10.], [10.], [0.1], [0.1], [0.1], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]
    simpleNavMeas.PMatrix = p_matrix_sc
    simpleNavMeas.walkBounds = walk_bounds_sc

    # set up planetNav for small body measurements
    planetNavMeas = planetNav.PlanetNav()
    planetNavMeas.ephemerisInMsg.subscribeTo(ephemConverter.ephemOutMsgs[0])

    # define the Pmatrix for planetNav, perfect state knowledge is assumed
    pos_sigma_p = 0.0
    vel_sigma_p = 0.0
    att_sigma_p = 0 * math.pi / 180.0
    rate_sigma_p = 0 * math.pi / 180.0
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
    walk_bounds_p = [[0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]
    planetNavMeas.PMatrix = p_matrix_p
    planetNavMeas.walkBounds = walk_bounds_p

    # set up the UKF
    smallBodyNav = smallBodyNavUKF.SmallBodyNavUKF()
    smallBodyNav.ModelTag = "smallBodyNavUKF"

    # set the filter parameters (sc area, mass, gravitational constants, etc.)
    smallBodyNav.mu_ast = mu  # Gravitational constant of the asteroid

    # set the process noise
    P_proc_pos = 1000
    P_proc_vel = 1
    P_proc_acc = 1e-6
    P_proc = [[P_proc_pos, 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., P_proc_pos, 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., P_proc_pos, 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., P_proc_vel, 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., P_proc_vel, 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., P_proc_vel, 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., P_proc_acc, 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., P_proc_acc, 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., P_proc_acc]]
    smallBodyNav.P_proc = P_proc

    # set the measurement noise
    R_meas = np.identity(3)
    R_meas[0,0] = R_meas[1,1] = R_meas[2,2] = 100  # position sigmas
    smallBodyNav.R_meas = R_meas.tolist()  # Measurement Noise

    # set the initial guess, x_0
    x_0 = np.zeros(9)
    x_0[0:3] = r_CA_A
    x_0[3:6] = v_CA_A - np.cross(omega_AN_A,r_CA_A)
    smallBodyNav.x_hat_k = x_0

    # set the initial state covariance
    P_k_pos = 1e4
    P_k_vel = 0.1
    P_k_acc = 1e-4
    P_k = [[P_k_pos, 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., P_k_pos, 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., P_k_pos, 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., P_k_vel, 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., P_k_vel, 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., P_k_vel, 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., P_k_acc, 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., P_k_acc, 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., P_k_acc]]
    smallBodyNav.P_k = P_k

    # set UKF hyperparameters
    smallBodyNav.alpha = 0
    smallBodyNav.beta = 2
    smallBodyNav.kappa = 1e-3

    # connect the relevant modules to the smallBodyUKF input messages
    smallBodyNav.navTransInMsg.subscribeTo(simpleNavMeas.transOutMsg)
    smallBodyNav.asteroidEphemerisInMsg.subscribeTo(planetNavMeas.ephemerisOutMsg)

    # add all models to the task
    scSim.AddModelToTask(simTaskName, gravBodyEphem, 100)
    scSim.AddModelToTask(simTaskName, ephemConverter, 99)
    scSim.AddModelToTask(simTaskName, scObject, 98)
    scSim.AddModelToTask(simTaskName, simpleNavMeas, 97)
    scSim.AddModelToTask(simTaskName, planetNavMeas, 96)
    scSim.AddModelToTask(simTaskName, smallBodyNav, 95)

    # setup data logging before the simulation is initialized
    ast_ephem_recorder = gravBodyEphem.planetOutMsgs[0].recorder()
    ast_truth_recorder = ephemConverter.ephemOutMsgs[0].recorder()
    ast_meas_recorder = planetNavMeas.ephemerisOutMsg.recorder()
    sc_truth_recorder = scObject.scStateOutMsg.recorder()
    sc_nav_recorder = smallBodyNav.smallBodyNavUKFOutMsg.recorder()
    sc_meas_recorder = simpleNavMeas.transOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, ast_ephem_recorder)
    scSim.AddModelToTask(simTaskName, ast_truth_recorder)
    scSim.AddModelToTask(simTaskName, ast_meas_recorder)
    scSim.AddModelToTask(simTaskName, sc_truth_recorder)
    scSim.AddModelToTask(simTaskName, sc_nav_recorder)
    scSim.AddModelToTask(simTaskName, sc_meas_recorder)

    # initialize Simulation
    scSim.InitializeSimulation()

    # configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve logged time and length
    time = sc_truth_recorder.times() * macros.NANO2SEC
    N_points = len(time)

    # retrieve truth inertial position and velocity w.r.t small body
    r_truth_N = np.array(sc_truth_recorder.r_BN_N - ast_truth_recorder.r_BdyZero_N)
    v_truth_N = np.array(sc_truth_recorder.v_BN_N - ast_truth_recorder.v_BdyZero_N)

    # retrieve small body rotational state
    sigma_AN = ast_truth_recorder.sigma_BN
    omega_AN_A = ast_truth_recorder.omega_BN_B

    # initialize truth inertial position and velocity w.r.t. asteroid centered fixed frame
    r_truth_A = np.zeros((N_points,3))
    v_truth_A = np.zeros((N_points,3))
    a_truth_A = np.zeros((N_points, 3))

    # loop through simulation points
    for ii in range(N_points):
        # obtain rotation matrix
        R_AN = RigidBodyKinematics.MRP2C(sigma_AN[ii][0:3])

        # rotate position and velocity
        r_truth_A[ii][0:3] = R_AN.dot(r_truth_N[ii][0:3])
        v_truth_A[ii][0:3] = R_AN.dot(v_truth_N[ii][0:3]) - np.cross(omega_AN_A[ii][0:3], r_truth_A[ii][0:3])

        # compute gravity acceleration and substract Keplerian term
        a_truth_A[ii][0:3] = np.array(asteroid.spherHarm.computeField(r_truth_A[ii][0:3], nSpherHarm, False)).reshape(3)

    # get filter output
    x_hat = sc_nav_recorder.state
    P = sc_nav_recorder.covar

    # plot the results
    figureList = {}

    plot_3Dposition(r_truth_N, title='inertial')
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_3Dposition(r_truth_A, title='asteroid')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_position(time, r_truth_A, x_hat[:,0:3])
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plot_velocity(time, v_truth_A, x_hat[:,3:6])
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plot_acceleration(time, a_truth_A, x_hat[:,6:9])
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

    plot_pos_error(time, np.subtract(r_truth_A, x_hat[:,0:3]), P)
    pltName = fileName + "6"
    figureList[pltName] = plt.figure(6)

    plot_vel_error(time, np.subtract(v_truth_A, x_hat[:,3:6]), P)
    pltName = fileName + "7"
    figureList[pltName] = plt.figure(7)

    plot_acc_error(time, np.subtract(a_truth_A,x_hat[:,6:9]), P)
    pltName = fileName + "8"
    figureList[pltName] = plt.figure(8)

    if show_plots:
         plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList

#
# This statement below ensures that the unit test script can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True  # show_plots
    )
