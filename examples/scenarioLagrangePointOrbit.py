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

This script sets up a 3-DOF spacecraft which is operating at one of five Earth-Moon Lagrange points. The purpose
is to illustrate how to use multiple gravity bodies to create interesting 3-body orbit behavior.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioLagrangePointOrbit.py

For this simulation, the Earth is assumed stationary, and the Moon's trajectory is generated using SPICE. Refer to
:ref:`scenarioOrbitMultiBody` to learn how to create multiple gravity bodies and read a SPICE trajectory.

The initial position of the spacecraft is specified using a Lagrange point index. The positioning of the Lagrange
points is illustrated `here <https://www.spaceacademy.net.au/library/notes/lagrangp.htm>`__.

For Lagrange points 1-3, the initial Earth-spacecraft distance is specified to lowest order
in :math:`\alpha = \mu_{M} / \mu_{E}`, where the subscript M is for the Moon and E is for the Earth.
These are unstable equilibrium points.

.. math::
    r_{L1} = a_{M} \left[ 1-\left(\frac{\alpha}{3}\right)^{1/3} \right]

.. math::
    r_{L2} = a_{M} \left[ 1+\left(\frac{\alpha}{3}\right)^{1/3} \right]

.. math::
    r_{L3} = a_{M} \left[ 1-\frac{7 \alpha}{12} \right]

For Lagrange points 4 and 5, the spacecraft is positioned at :math:`r_{L4} = r_{L5} = a_{M}` at +/- 60
degrees from the Earth-Moon vector. These are stable equilibrium points.

When the simulation completes, two plots are shown. The first plot shows the orbits of the Moon and spacecraft in
the Earth-centered inertial frame. The second plot shows the motion of the Moon and spacecraft in a frame rotating
with the Moon.

Illustration of Simulation Results
----------------------------------

The following images illustrate the simulation run results with the following settings:

::

    nOrbits=1, timestep=300, showPlots=True

When starting at L1, L2, or L3, the spacecraft moves away from the unstable equilibrium point.

::

    lagrangePoint=1

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL1Fig1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL1Fig2.svg
    :align: center

::

    lagrangePoint=2

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL2Fig1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL2Fig2.svg
    :align: center

::

    lagrangePoint=3

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL3Fig1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL3Fig2.svg
    :align: center

When starting at L4 or L5, the spacecraft remains near the stable equilibrium point.

::

    lagrangePoint=4

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL4Fig1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL4Fig2.svg
    :align: center


::

    lagrangePoint=5

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL5Fig1.svg
    :align: center

.. image:: /_images/Scenarios/scenarioLagrangePointOrbitL5Fig2.svg
    :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  This scenario illustrates the orbit of a spacecraft near the Earth-Moon Lagrange points.
# Author:   Scott McKinley
# Creation Date:  Aug. 31, 2022
#

import os
from datetime import datetime, timedelta

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.simulation import orbElemConvert
from Basilisk.simulation import spacecraft
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)
from Basilisk.utilities.pyswice_spk_utilities import spkRead

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def run(lagrangePoint, nOrbits, timestep, showPlots=True):
    """
    Args:
        lagrangePoint (int): Earth-Moon Lagrange point ID [1,2,3,4,5]
        nOrbits (float): Number of Earth orbits to simulate
        timestep (float): Simulation timestep in seconds
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
    simulationTimeStep = macros.sec2nano(timestep)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Setup the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "lagrangeSat"

    # Setup the orbital element converter for planet position message output
    oeObject = orbElemConvert.OrbElemConvert()
    oeObject.ModelTag = "planetObj"

    # Add spacecraft object to the simulation process
    # Make this model a lower priority than the SPICE object task
    scSim.AddModelToTask(simTaskName, scObject, 0)

    # Setup gravity factory and gravity bodies
    # Include bodies as a list of SPICE names
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['moon', 'earth'])
    gravBodies['earth'].isCentralBody = True

    # Add gravity bodies to the spacecraft dynamics
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Create default SPICE module, specify start date/time.
    timeInitString = "2022 August 31 15:00:00.0"
    spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
    timeInit = datetime.strptime(timeInitString, spiceTimeStringFormat)
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'Earth'

    # Add SPICE object to the simulation task list
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject, 1)

    # Import SPICE ephemeris data into the python environment
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

    # Set spacecraft ICs
    # Use Earth data
    moonSpiceName = 'moon'
    moonInitialState = 1000 * spkRead(moonSpiceName, timeInitString, 'J2000', 'earth')
    moon_rN_init = moonInitialState[0:3]
    moon_vN_init = moonInitialState[3:6]
    moon = gravBodies['moon']
    earth = gravBodies['earth']
    oe = orbitalMotion.rv2elem(earth.mu, moon_rN_init, moon_vN_init)
    moon_a = oe.a

    # Delay or advance the spacecraft by a few degrees to prevent strange spacecraft-moon interactions when the
    # spacecraft wanders from the unstable equilibrium points
    if lagrangePoint == 1:
        oe.a = oe.a * (1-np.power(moon.mu / (3*earth.mu), 1./3.))
        oe.f = oe.f + macros.D2R*4
    elif lagrangePoint == 2:
        oe.a = oe.a * (1+np.power(moon.mu / (3*earth.mu), 1./3.))
        oe.f = oe.f - macros.D2R*4
    elif lagrangePoint == 3:
        oe.a = oe.a * (1-(7*moon.mu/(12*earth.mu)))
        oe.f = oe.f + np.pi
    elif lagrangePoint == 4:
        oe.f = oe.f + np.pi/3
    else:
        oe.f = oe.f - np.pi/3

    oe.f = oe.f - macros.D2R*2

    rN, vN = orbitalMotion.elem2rv(earth.mu, oe)

    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN

    # Set simulation time
    n = np.sqrt(earth.mu / np.power(moon_a, 3))
    P = 2 * np.pi/n
    simulationTime = macros.sec2nano(nOrbits*P)

    # Setup data logging
    numDataPoints = 1000
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)

    # Setup spacecraft data recorder
    scDataRec = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, scDataRec)

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                              # saveFile=__file__
                                              )
    # Initialize simulation
    scSim.InitializeSimulation()

    # Execute simulation
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve logged data
    posData = scDataRec.r_BN_N
    velData = scDataRec.v_BN_N
    timeData = scDataRec.times()

    # Plot results
    np.set_printoptions(precision=16)
    plt.close("all")
    figureList = {}
    b = oe.a * np.sqrt(1 - oe.e * oe.e)

    # First plot: Draw orbit in inertial frame
    fig = plt.figure(1, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
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
        fDataSpacecraft.append(oeDataSpacecraft.f + oeDataSpacecraft.omega - oe.omega)  # Why the add/subtract of omegas?
    plt.plot(rDataSpacecraft * np.cos(fDataSpacecraft) / 1000, rDataSpacecraft * np.sin(fDataSpacecraft) / 1000,
             color='g', linewidth=3.0, label='Spacecraft')

    # Plot moon orbit data
    rDataMoon = []
    fDataMoon = []
    for ii in range(len(timeData)):
        simTime = timeData[ii] * macros.NANO2SEC
        sec = int(simTime)
        usec = (simTime - sec) * 1e6
        time = timeInit + timedelta(seconds=sec, microseconds=usec)
        timeString = time.strftime(spiceTimeStringFormat)
        moonState = 1000 * spkRead(moonSpiceName, timeString, 'J2000', 'earth')
        moon_rN = moonState[0:3]
        moon_vN = moonState[3:6]
        oeDataMoon = orbitalMotion.rv2elem(earth.mu, moon_rN, moon_vN)
        rDataMoon.append(oeDataMoon.rmag)
        fDataMoon.append(oeDataMoon.f + oeDataMoon.omega - oe.omega)
    plt.plot(rDataMoon * np.cos(fDataMoon) / 1000, rDataMoon * np.sin(fDataMoon) / 1000, color='0.5', linewidth=3.0,
             label='Moon')

    plt.xlabel('$i_e$ Coord. [km]')
    plt.ylabel('$i_p$ Coord. [km]')
    plt.grid()
    plt.legend()
    pltName = fileName + "L" + str(lagrangePoint) + "Fig1"
    figureList[pltName] = plt.figure(1)

    # Second plot: Draw orbit in frame rotating with the Moon
    fig = plt.figure(2, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    ax = fig.gca()
    ax.ticklabel_format(style='scientific', scilimits=[-5, 3])

    # Draw 'cartoon' Earth
    ax.add_artist(plt.Circle((0, 0), 0.2e5, color='b'))

    # Plot spacecraft and Moon orbit data
    rDataSpacecraft = []
    fDataSpacecraft = []
    rDataMoon = []
    fDataMoon = []
    for ii in range(len(posData)):

        # Get Moon f
        simTime = timeData[ii] * macros.NANO2SEC
        sec = int(simTime)
        usec = (simTime - sec) * 1e6
        time = timeInit + timedelta(seconds=sec, microseconds=usec)
        timeString = time.strftime(spiceTimeStringFormat)
        moonState = 1000 * spkRead(moonSpiceName, timeString, 'J2000', 'earth')
        moon_rN = moonState[0:3]
        moon_vN = moonState[3:6]
        oeDataMoon = orbitalMotion.rv2elem(earth.mu, moon_rN, moon_vN)
        moon_f = oeDataMoon.f

        # Get spacecraft data, with spacecraft f = oe data f - moon f
        oeDataSpacecraft = orbitalMotion.rv2elem(earth.mu, posData[ii], velData[ii])
        rDataSpacecraft.append(oeDataSpacecraft.rmag)
        fDataSpacecraft.append(oeDataSpacecraft.f - moon_f + oeDataSpacecraft.omega - oe.omega)

        # Get Moon data
        rDataMoon.append(oeDataMoon.rmag)
        fDataMoon.append(0)

    plt.plot(rDataSpacecraft * np.cos(fDataSpacecraft) / 1000, rDataSpacecraft * np.sin(fDataSpacecraft) / 1000,
             color='g', linewidth=3.0, label='Spacecraft')
    plt.plot(rDataMoon * np.cos(fDataMoon) / 1000, rDataMoon * np.sin(fDataMoon) / 1000, color='0.5', linewidth=3.0,
             label='Moon')

    plt.xlabel('Earth-Moon axis [km]')
    plt.ylabel('Earth-Moon perpendicular axis [km]')
    plt.grid()
    plt.legend()
    pltName = fileName + "L" + str(lagrangePoint) + "Fig2"
    figureList[pltName] = plt.figure(2)

    if showPlots:
        plt.show()

    plt.close("all")

    # Unload spice libraries
    gravFactory.unloadSpiceKernels()
    pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
    pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
    pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
    pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

    return figureList


if __name__ == "__main__":
    run(
        5,      # Lagrange point
        1,      # Number of Moon orbits
        300,       # Timestep (seconds)
        True    # Show plots
    )


