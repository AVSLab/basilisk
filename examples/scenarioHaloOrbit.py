
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This script sets up a 3-DOF spacecraft that is operating near-Halo orbit at L2 Earth-Moon Lagrange points. The purpose
is to illustrate how to set up the spacecraft's initial conditions to create a near-Halo orbit and convert the barycenter focused
non-dimensional ICs to earth-centered inertial frame components.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioHaloOrbit.py

For this simulation, the Earth is assumed stationary, and the Moon's trajectory is generated using SPICE. Refer to
:ref:`scenarioOrbitMultiBody` to learn how to create multiple gravity bodies and read a SPICE trajectory.

When the simulation completes, three plots are shown. The first plot shows the orbits of the Moon and spacecraft in
the Earth-centered inertial frame. The second and third plots show the motion of the spacecraft in a frame rotating
with the Moon. In the second plot, the y-axis represents the Moon's velocity direction, and in the third plot, the
y-axis represents the cross product of the Moon's position vector and velocity vector.

Illustration of Simulation Results
----------------------------------

The following images illustrate the simulation run results with the following settings:

::

    showPlots = True

.. image:: /_images/Scenarios/scenarioHaloOrbitFig1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioHaloOrbitFig2.svg
    :align: center

.. image:: /_images/Scenarios/scenarioHaloOrbitFig3.svg
    :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  This scenario illustrates the near-Halo orbit of a spacecraft.
# Author:   Yumeka Nagano
# Creation Date:  Feb. 12, 2024
#

import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.simulation import spacecraft
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)
from Basilisk.utilities.pyswice_spk_utilities import spkRead

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def run(showPlots=True):
    """
    Args:
        showPlots (bool): Determines if the script should display plots
    """

    # Create simulation variable names
    simTaskName = "dynTask"
    simProcessName = "dynProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.SetProgressBar(True)

    # Create the simulation process (dynamics)
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Add the dynamics task to the dynamics process and specify the integration update time
    timestep = 300
    simulationTimeStep = macros.sec2nano(timestep)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Setup the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "HaloSat"

    # Add spacecraft object to the simulation process
    # Make this model a lower priority than the SPICE object task
    scSim.AddModelToTask(simTaskName, scObject, 0)

    # Setup gravity factory and gravity bodies
    # Include bodies as a list of SPICE names
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies('moon', 'earth')
    gravBodies['earth'].isCentralBody = True

    # Add gravity bodies to the spacecraft dynamics
    gravFactory.addBodiesTo(scObject)

    # Create default SPICE module, specify start date/time.
    timeInitString = "2022 August 31 15:00:00.0"
    bsk_path = __path__[0]
    spiceObject = gravFactory.createSpiceInterface(bsk_path + "/supportData/EphemerisData/", time=timeInitString,
                                                   epochInMsg=True)
    spiceObject.zeroBase = 'earth'

    # Add SPICE object to the simulation task list
    scSim.AddModelToTask(simTaskName, spiceObject, 1)

    # Import SPICE ephemeris data into the python environment
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

    # Set spacecraft ICs
    # Get initial moon data
    moonSpiceName = 'moon'
    moonInitialState = 1000 * spkRead(moonSpiceName, timeInitString, 'J2000', 'earth')
    moon_rN_init = moonInitialState[0:3]
    moon_vN_init = moonInitialState[3:6]
    moon = gravBodies['moon']
    earth = gravBodies['earth']
    oe = orbitalMotion.rv2elem(earth.mu, moon_rN_init, moon_vN_init)
    moon_a = oe.a

    # Direction Cosine Matrix (DCM) from earth centered inertial frame to earth-moon rotation frame
    DCMInit = np.array([moon_rN_init/np.linalg.norm(moon_rN_init),moon_vN_init/np.linalg.norm(moon_vN_init),
                        np.cross(moon_rN_init, moon_vN_init) / np.linalg.norm(np.cross(moon_rN_init, moon_vN_init))])

    # Set up non-dimensional parameters
    T_ND = np.sqrt(moon_a ** 3 / (earth.mu + moon.mu))      # non-dimensional time for one second
    mu_ND = moon.mu/(earth.mu + moon.mu)                    # non-dimensional mass

    # Set up initial conditions for the spacecraft
    x0 = 1.182212 * moon_a + moon_a * mu_ND
    z0 = 0.049 * moon_a
    dy0 = -0.167 * moon_a / T_ND
    X0 = np.array([[x0], [0], [z0]])
    dX0 = np.array([[0], [np.linalg.norm(moon_vN_init) + dy0], [0]])

    rN = np.dot(np.transpose(DCMInit), X0)
    vN = np.dot(np.transpose(DCMInit), dX0)

    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN

    # Set simulation time
    simulationTime = macros.day2nano(17.5)

    # Setup data logging
    numDataPoints = 1000
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)

    # Setup spacecraft data recorder
    scDataRec = scObject.scStateOutMsg.recorder(samplingTime)
    MoonDataRec = spiceObject.planetStateOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, scDataRec)
    scSim.AddModelToTask(simTaskName, MoonDataRec)

    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                                  # saveFile=__file__
                                                  )
        viz.settings.showCelestialBodyLabels = 1
        viz.settings.mainCameraTarget = "earth"
        viz.settings.trueTrajectoryLinesOn = 4
        viz.settings.truePathRotatingFrame = "earth moon"

    # Initialize simulation
    scSim.InitializeSimulation()

    # Execute simulation
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve logged data
    posData = scDataRec.r_BN_N
    velData = scDataRec.v_BN_N
    timeData = scDataRec.times()
    moonPos = MoonDataRec.PositionVector
    moonVel = MoonDataRec.VelocityVector

    # Plot results
    np.set_printoptions(precision=16)
    plt.close("all")
    figureList = {}
    b = oe.a * np.sqrt(1 - oe.e * oe.e)

    # First plot: Draw orbit in inertial frame
    fig = plt.figure(1, figsize=tuple(np.array((1.0, b / oe.a)) * 4.75), dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    ax = fig.gca()
    ax.ticklabel_format(style='scientific', scilimits=[-5, 3])

    # Draw 'cartoon' Earth
    ax.add_artist(plt.Circle((0, 0), 0.2e5, color='b'))

    # Plot spacecraft orbit data
    rDataSpacecraft = []
    fDataSpacecraft = []
    for ii in range(len(posData)):
        oeDataSpacecraft = orbitalMotion.rv2elem(earth.mu, posData[ii], velData[ii])
        rDataSpacecraft.append(oeDataSpacecraft.rmag)
        fDataSpacecraft.append(oeDataSpacecraft.f + oeDataSpacecraft.omega - oe.omega)
    plt.plot(rDataSpacecraft * np.cos(fDataSpacecraft) / 1000, rDataSpacecraft * np.sin(fDataSpacecraft) / 1000,
             color='g', linewidth=3.0, label='Spacecraft')

    # Plot moon orbit data
    rDataMoon = []
    fDataMoon = []
    for ii in range(len(timeData)):
        oeDataMoon = orbitalMotion.rv2elem(earth.mu, moonPos[ii], moonVel[ii])
        rDataMoon.append(oeDataMoon.rmag)
        fDataMoon.append(oeDataMoon.f + oeDataMoon.omega - oe.omega)
    plt.plot(rDataMoon * np.cos(fDataMoon) / 1000, rDataMoon * np.sin(fDataMoon) / 1000, color='0.5',
             linewidth=3.0, label='Moon')

    plt.xlabel(r'$i_e$ Coord. [km]')
    plt.ylabel(r'$i_p$ Coord. [km]')
    plt.grid()
    plt.legend()
    pltName = fileName + "Fig1"
    figureList[pltName] = plt.figure(1)

    # Second plot: Draw orbit in frame rotating with the Moon (the center is L2 point)
    # x axis is moon position vector direction and y axis is moon velocity vector direction
    fig = plt.figure(2, figsize=tuple(np.array((1.0, b / oe.a)) * 4.75), dpi=100)
    plt.axis(np.array([-1e5, 5e5, -3e5, 3e5])  * 1.25)
    ax = fig.gca()
    ax.ticklabel_format(style='scientific', scilimits=[-5, 3])

    # Draw 'cartoon' Earth
    ax.add_artist(plt.Circle((0, 0), 0.2e5, color='b'))

    # Plot spacecraft orbit data
    rSpacecraft = np.zeros((len(posData), 3))

    for ii in range(len(posData)):
        # Get Moon position and velocity
        moon_rN = moonPos[ii]
        moon_vN = moonVel[ii]

        # Direction Cosine Matrix (DCM) from earth centered inertial frame to earth-moon rotation frame
        rSpacecraftMag = np.linalg.norm(posData[ii])
        rMoonMag = np.linalg.norm(moon_rN)
        DCM = [moon_rN / rMoonMag, moon_vN / np.linalg.norm(moon_vN),
               np.cross(moon_rN, moon_vN) / np.linalg.norm(np.cross(moon_rN, moon_vN))]

        # Spacecraft position in rotating frame
        rSpacecraft[ii,:] = np.dot(DCM, posData[ii])

    plt.plot(rSpacecraft[:,0] / 1000, rSpacecraft[:,1] / 1000,
             color='g', linewidth=3.0, label='Spacecraft')

    plt.xlabel('Earth-Moon axis [km]')
    plt.ylabel('Moon Velocity axis [km]')
    plt.grid()
    plt.legend()
    pltName = fileName + "Fig2"
    figureList[pltName] = plt.figure(2)

    # Third plot: Draw orbit in frame rotating with the Moon (the center is L2 point)
    # x axis is moon position vector direction and y axis is the cross product direction of the moon position vector and
    # velocity vector
    fig = plt.figure(3, figsize=tuple(np.array((1.0, b / oe.a)) * 4.75), dpi=100)
    plt.axis(np.array([-1e5, 5e5, -3e5, 3e5]) * 1.25)
    ax = fig.gca()
    ax.ticklabel_format(style='scientific', scilimits=[-5, 3])

    # Draw 'cartoon' Earth
    ax.add_artist(plt.Circle((0, 0), 0.2e5, color='b'))

    plt.plot(rSpacecraft[:, 0] / 1000, rSpacecraft[:, 2] / 1000,
             color='g', linewidth=3.0, label='Spacecraft')

    plt.xlabel('Earth-Moon axis [km]')
    plt.ylabel('Earth-Moon perpendicular axis [km]')
    plt.grid()
    plt.legend()
    pltName = fileName + "Fig3"
    figureList[pltName] = plt.figure(3)

    if showPlots:
        plt.show()

    plt.close("all")

    # Unload spice libraries
    gravFactory.unloadSpiceKernels()
    pyswice.unload_c(spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
    pyswice.unload_c(spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
    pyswice.unload_c(spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
    pyswice.unload_c(spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

    return figureList


if __name__ == "__main__":
    run(
        True    # Show plots
    )
