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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/uUomHSGQW3c" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

Demonstrates how to setup a custom gravity object in Basilisk that is not directly supported by
the ``simIncludeGravBody.py`` file.  In this simulation the sun is created using standard values, the Earth
is created using custom values, and the asteroid Itokawa is created with custom values.

.. image:: /_images/static/scenarioCustomGravObject.jpg
   :align: center

Further, the Vizard binary file is setup to load up a custom CAD model for the asteroid. The spacecraft
orbit is defined relative to the asteroid.  Note, this feature requires :ref:`Vizard <vizard>` version 1.8 or higher.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioCustomGravBody.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both modules.

.. image:: /_images/static/test_scenarioCustomGravBody.svg
   :align: center

:ref:`planetEphemeris` is used to create the planet ephemeris states. The sun is assumed to be stationary,
while Earth is on a circular orbit and Itokawa is on its elliptical heliocentric orbit.

The method ``createCustomGravObject()`` is used to create the BSK grav bodies for both earth and Itokawa.
The earth body is already supported in :ref:`simIncludeGravBody`, but in this script we show how this could
be customized.  The gravity body ephemeris states are connected to the :ref:`planetEphemeris` planet
state output messages.

Finally, the recorded states will all be relative to the inertial origin at the sun.  :ref:`planetEphemeris` does not
have the ``zeroBase`` capability as :ref:`spiceInterface` has.  This script also records the asteroid
states so that the plot is done of the spacecraft motion relative to the asteroid.

The simulation executes and shows a plot of the spacecraft motion relative to the asteroid.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

.. image:: /_images/Scenarios/scenarioCustomGravBody1.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing how to setup a custom gravity object
# Author:   Hanspeter Schaub
# Creation Date:  Feb. 23, 2021
#

import os

import matplotlib.pyplot as plt
from Basilisk.simulation import planetEphemeris
from Basilisk.simulation import spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, simIncludeGravBody, vizSupport)
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import unitTestSupport


# The path to the location of Basilisk
# Used to get the location of supporting data.
fileName = os.path.basename(os.path.splitext(__file__)[0])



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

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the simulation time step information
    simulationTimeStep = macros.sec2nano(10.0)
    simulationTime = macros.min2nano(1120.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # setup celestial object ephemeris module
    gravBodyEphem = planetEphemeris.PlanetEphemeris()
    gravBodyEphem.ModelTag = 'planetEphemeris'
    scSim.AddModelToTask(simTaskName, gravBodyEphem)
    gravBodyEphem.setPlanetNames(planetEphemeris.StringVector(["Itokawa", "earth"]))

    # specify orbits of gravitational bodies
    oeAsteroid = planetEphemeris.ClassicElementsMsgPayload()
    oeAsteroid.a = 1.3241 * orbitalMotion.AU * 1000  # meters
    oeAsteroid.e = 0.2801
    oeAsteroid.i = 1.6214*macros.D2R
    oeAsteroid.Omega = 69.081*macros.D2R
    oeAsteroid.omega = 162.82*macros.D2R
    oeAsteroid.f = 90.0*macros.D2R

    oeEarth = planetEphemeris.ClassicElementsMsgPayload()
    oeEarth.a = orbitalMotion.AU * 1000  # meters
    oeEarth.e = 0.0167086
    oeEarth.i = 7.155 * macros.D2R
    oeEarth.Omega = 174.9 * macros.D2R
    oeEarth.omega = 288.1 * macros.D2R
    oeEarth.f = 270.0 * macros.D2R

    # specify celestial object orbit
    gravBodyEphem.planetElements = planetEphemeris.classicElementVector([oeAsteroid, oeEarth])
    # specify celestial object orientation
    gravBodyEphem.rightAscension = planetEphemeris.DoubleVector([0.0 * macros.D2R, 0.0 * macros.D2R])
    gravBodyEphem.declination = planetEphemeris.DoubleVector([0.0 * macros.D2R, 0.0 * macros.D2R])
    gravBodyEphem.lst0 = planetEphemeris.DoubleVector([0.0 * macros.D2R, 0.0 * macros.D2R])
    gravBodyEphem.rotRate = planetEphemeris.DoubleVector(
        [360 * macros.D2R / (12.132 * 3600.), 360 * macros.D2R / (24. * 3600.)])

    # setup Sun gravity body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createSun()

    # setup asteroid gravity body
    mu = 2.34268    # meters^3/s^2
    asteroid = gravFactory.createCustomGravObject("Itokawa", mu, radEquator=200)

    asteroid.isCentralBody = True  # ensure this is the central gravitational body
    asteroid.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])

    # setup Earth gravity Body
    earth = gravFactory.createCustomGravObject("earth", 0.3986004415E+15, radEquator=6378136.6)
    earth.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[1])

    # create SC object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    gravFactory.addBodiesTo(scObject)
    scSim.AddModelToTask(simTaskName, scObject)

    # setup orbit initial conditions about the asteroid
    oe = orbitalMotion.ClassicElements()
    oe.a = 500.0  # meters
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    scRec = scObject.scStateOutMsg.recorder(samplingTime)
    astRec = gravBodyEphem.planetOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, scRec)
    scSim.AddModelToTask(simTaskName, astRec)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    # Note that the gravitational body information is pulled automatically from the spacecraft object(s)
    # Even if custom gravitational bodies are added, this information is pulled by the method below
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  # , saveFile=fileName
                                                  )
        viz.settings.showSpacecraftLabels = 1
        # load CAD for custom gravity model
        vizSupport.createCustomModel(viz,
                                     modelPath=os.path.join(path, "dataForExamples", "Itokawa", "ItokawaHayabusa.obj"),
                                     shader=1,
                                     simBodiesToModify=['Itokawa'],
                                     scale=[962, 962, 962])

    #   initialize Simulation
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve logged spacecraft position relative to asteroid
    posData = scRec.r_BN_N - astRec.PositionVector

    #
    #   plot the results
    #
    timeAxis = scRec.times() * macros.NANO2HOUR
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(3):
        plt.plot(timeAxis, posData[:, idx] ,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BI,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Itokawa Relative Position [m]')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

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
