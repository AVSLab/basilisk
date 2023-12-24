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
Overview
--------

This script sets up a basic spacecraft in orbit about Earth. One option uses ``earth.isCentralBody = True``
and the other uses ``isCentralBody = False``. The nuances of spacecraft position and velocity I/O in these cases are
demonstrated.

.. image:: /_images/static/test_scenarioBasicOrbit.svg
   :align: center

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioCentralBody.py

.. note:: This script is a good reference for configuring the following modules:

          * :ref:`spacecraft`
          * :ref:`gravityEffector`


Illustration of Simulation Results
----------------------------------

Running this example script will yield the following results.

::

    show_plots = True, useCentral = False

.. figure:: /_images/Scenarios/scenarioCentralBody10.svg
   :align: center

.. figure:: /_images/Scenarios/scenarioCentralBody20.svg
   :align: center

::

    show_plots = True, useCentral = True

.. figure:: /_images/Scenarios/scenarioCentralBody11.svg
   :align: center

.. figure:: /_images/Scenarios/scenarioCentralBody21.svg
   :align: center

"""



#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Demonstrate sim set up using isCentralBody=True and isCentralBody=False
# Author:   Scott Carnahan
# Creation Date:  Jul 11 2018
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
# import simulation related support
from Basilisk.simulation import spacecraft
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)
from Basilisk.utilities import planetStates
from numpy import array
from numpy.linalg import norm

# attempt to import vizard
from Basilisk.utilities import vizSupport

# The path to the location of Basilisk
# Used to get the location of supporting data.
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, useCentral):
    """
        Args:
            show_plots (bool): Determines if the script should display plots
            useCentral (bool): Specifies if the planet is the center of the coordinate system
        """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = useCentral          # ensure this is the central gravitational body
    mu = planet.mu

    # set up sun
    gravFactory.createSun()

    #Set up spice with spice time
    UTCInit = "2012 MAY 1 00:28:30.0"
    spiceObject = gravFactory.createSpiceInterface(time=UTCInit, epochInMsg=True)
    scSim.AddModelToTask(simTaskName, spiceObject)

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    oe.a = rLEO
    oe.e = 0.000001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    truth_r = norm(rN) #for test results
    truth_v = norm(vN) #for test results
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements wrt ECI frame

    #
    #   initialize Spacecraft States with the initialization variables
    #
    # Spacecraft positions can be input (and integrated) only relative to the "central body" by
    # using the isCentralBody flag. This is demonstrated in Setup 2. Generally, this is a good
    # practice because it increases the accuracy of the integration.
    #
    # Alternatively, absolute positions and velocities can be input and integrated.
    # To do so, it is useful to be able to get a planet position and velocity from spice
    # to be able to modify your spacecraft inputs by these values. This can be done using the
    # planetStates utility:
    # ~~~~~~~~~~~~~{.py}
    # from Basilisk.utilities import planetStates
    # ~~~~~~~~~~~~~
    # Then, the planetPositionVelocity() method can be used to get the needed information
    if useCentral:
        scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
        scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N
    else:
        #by default planetstates.planetPositionVelocity returns SSB central ICRS coordinates for the planet at the time
        # requested. also pck0010.tpc ephemeris file
        #look in the function for how to use other ephemeris files, reference frames, and observers
        planetPosition, planetVelocity = planetStates.planetPositionVelocity('EARTH', UTCInit)
        scObject.hub.r_CN_NInit = rN + array(planetPosition)
        scObject.hub.v_CN_NInit = vN + array(planetVelocity)
        # In the above call, the first input is the planet to get the states of and the second is the UTC time
        # to get the states at. Additional inputs are available in the method documentation. These states can then be added
        # to the planet-relative states before setting them to the spacecraft initial states

        # This works without frame rotations because all planet states are given in the spice inertial system relative to the
        # zero base. Additionally, it is assumed for this script that the Keplerian orbital elements were given relative
        # to the Earth Centered Inertial Frame which is aligned with the spice inertial frame.


    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.75*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    plLog = spiceObject.planetStateOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, plLog)

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    #
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    # Note: position and velocity are returned relative to the zerobase (SSB by default)
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N
    earthPositionHistory = plLog.PositionVector
    earthVelocityHistory = plLog.VelocityVector

    # Finally, note that the output position and velocity (when reading message logs) will be relative to the spice zero
    # base, even when a central body is being used. So, to plot planet-relative orbits, the outputs are adjusted by the
    # time history of the earth position and velocity.
    #
    # Plots found when running this scenario are the same as the basic orbit scenario and are included for visual inspection that the results are
    # roughly the same regardless of the use of a central body.

    #bring the s/c pos, vel back to earth relative coordinates to plot
    posData[:] -= earthPositionHistory[:]
    velData[:] -= earthVelocityHistory[:]

    out_r = norm(posData[-1])
    out_v = norm(velData[-1])

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(3):
        plt.plot(dataLog.times() * macros.NANO2SEC / P, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1" + str(int(useCentral))
    figureList[pltName] = plt.figure(1)

    # draw orbit in perifocal frame
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(2, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    planetRadius = planet.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(posData[:,0] / 1000, posData[:,1] / 1000, color='#aa0000', linewidth=3.0)
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0, 2 * np.pi, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555')
    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')
    plt.grid()
    pltName = fileName + "2" + str(int(useCentral))
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return out_r, out_v, truth_r, truth_v, figureList

# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,        # show_plots
        False        # useCentral
    )
