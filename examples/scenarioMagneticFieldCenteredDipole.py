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

This script sets up a 3-DOF spacecraft which is orbiting a planet that
has a magnetic field.  The purpose
is to illustrate how to create and setup the centered dipole
magnetic field, as well as determine the
magnetic field at a spacecraft location.  The orbit setup is similar to that used in
:ref:`scenarioBasicOrbit`.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioMagneticFieldCenteredDipole.py

Simulation Scenario Setup Details
---------------------------------

The simulation layout is shown in the following illustration.
A single simulation process is created
which contains the spacecraft object.  The spacecraft state message
is connected to the magnetic field
module which outputs the local magnetic field in inertial frame components.

.. image:: /_images/static/test_scenario_MagneticFieldCenteredDipole.svg
   :align: center

When the simulation completes 2 plots are shown for each case.  One plot always shows
the inertial position vector components, while the second plot shows the local magnetic field
vector components with respect to the inertial frame.

Note that the magnetic field module are zeroed, and appropriate
parameters must be specified for the planet.  The
following code illustrates setting the Earth dipole parameters::

    magModule.g10 = -30926.00 / 1e9 * 0.5  # Tesla
    magModule.g11 =  -2318.00 / 1e9 * 0.5  # Tesla
    magModule.h11 =   5817.00 / 1e9 * 0.5  # Tesla
    magModule.planetRadius = 6371.2 * 1000  # meters

The python support file ``simSetPlanetEnvironment.py`` provides helper
functions to setup command magnetic field
environments including the centered dipole models for Mercury,
Earth, Jupiter, Saturn, Uranus and Neptune.

The default
planet's position vector is assumed to be the inertial frame origin
and an identity orientation matrix.
If a different planet state message is required this can be specified
through the optional input message ``planetPosInMsgName``.

The magnetic field module can produce the magnetic field for a vector of
spacecraft locations, not just for a
single spacecraft.  Let ``scObject`` be an instance of :ref:`Spacecraft`,
then the spacecraft state output message
is added to the magnetic field module through::

    magModule.addSpacecraftToModel(scObject.scStateOutMsg)

Note that this command can be repeated if the magnetic field should be
evaluated for different spacecraft.

Every time a spacecraft is added to the magnetic field module,
an automated output message name is created.
For `magModule` is "CenteredDipole_0_data" as the ModelTag string is
``CenteredDipole`` and the spacecraft number is 0.
This output name is created in the  ``addSpacecraftToModel()`` function.
However, if the default output name is used for the second planetary
magnetic field model, then both module share
the same output name and one will overwrite the others output.
To ensure the second magnetic field has a unique
output name, the default name is replaced with a unique message.

The reach of the magnetic field model is specified through the module
variables ``envMinReach`` and ``envMaxReach``.
Their default values are -1 which turns off this feature, giving the
magnetic field evaluation infinite reach.
As the elliptical Earth scenario uses 2 Earth-fixed magnetic fields,
we want ``magModule2`` to only evaluate a
magnetic field if the orbit radius is less than ``req*1.3``.  Similarly,
for radii above ``req*1.3`` we want the first
magnetic field model to be used.

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True, orbitCase='circular', planetCase='Earth'

.. image:: /_images/Scenarios/scenarioMagneticFieldCenteredDipole1circularEarth.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMagneticFieldCenteredDipole2circularEarth.svg
   :align: center

::

   show_plots = True, orbitCase='elliptical', planetCase='Earth'

This case illustrates an elliptical Earth orbit inclination where 2 dipole magnetic
fields are attached. One model acts above 1.3 Earth radius, and the other below that region.

.. image:: /_images/Scenarios/scenarioMagneticFieldCenteredDipole1ellipticalEarth.svg
  :align: center

.. image:: /_images/Scenarios/scenarioMagneticFieldCenteredDipole2ellipticalEarth.svg
  :align: center

::

    show_plots = True, orbitCase='elliptical', planetCase='Jupiter'

.. image:: /_images/Scenarios/scenarioMagneticFieldCenteredDipole1ellipticalJupiter.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMagneticFieldCenteredDipole2ellipticalJupiter.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test illustrating how to use a centered dipole magnetic fields attached to a planet.
# Author:   Hanspeter Schaub
# Creation Date:  March 16, 2019
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
from Basilisk.simulation import magneticFieldCenteredDipole
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)
from Basilisk.utilities import simSetPlanetEnvironment

#attempt to import vizard
from Basilisk.utilities import vizSupport
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, orbitCase, planetCase):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        orbitCase (str): {'circular', 'elliptical'}
        planetCase (str): {'Earth', 'Junpiter'}

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
    scObject.ModelTag = "bsk-Sat"

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    if planetCase == 'Jupiter':
        planet = gravFactory.createJupiter()
        planet.isCentralBody = True           # ensure this is the central gravitational body
    else:  # Earth
        planet = gravFactory.createEarth()
        planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    req = planet.radEquator

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))


    # create the magnetic field
    magModule = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()  # default is Earth centered dipole module
    magModule.ModelTag = "CenteredDipole"
    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple

    if planetCase == 'Jupiter':
        # The following command is a support function that sets up the centered dipole parameters.
        # These parameters can also be setup manually
        simSetPlanetEnvironment.centeredDipoleMagField(magModule, 'jupiter')
    else:
        simSetPlanetEnvironment.centeredDipoleMagField(magModule, 'earth')
    scSim.AddModelToTask(simTaskName, magModule)

    if planetCase == 'Earth' and orbitCase == 'elliptical':
        # Note that more then one magnetic field can be attached to a planet.
        # In the elliptic Earth orbit scenario
        # a second magnetic field module `magModule2` is created with a
        # different custom dipole model.  It is connected to the
        # same spacecraft state message as the first magnetic field model.

        magModule2 = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
        magModule2.ModelTag = "CenteredDipole2"
        magModule2.addSpacecraftToModel(scObject.scStateOutMsg)
        # set the 2nd magnetic field through custom dipole settings
        magModule2.g10 = -30926.00 / 1e9 * 0.5  # Tesla
        magModule2.g11 =  -2318.00 / 1e9 * 0.5  # Tesla
        magModule2.h11 =   5817.00 / 1e9 * 0.5  # Tesla
        magModule2.planetRadius = 6371.2 * 1000  # meters
        # set the reach variables such that the fields
        magModule2.envMaxReach = req*1.3
        magModule.envMinReach = magModule2.envMaxReach
        scSim.AddModelToTask(simTaskName, magModule2)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rPeriapses = req*1.1     # meters
    if orbitCase == 'circular':
        oe.a = rPeriapses
        oe.e = 0.0000
    elif orbitCase == 'elliptical':
        rApoapses = req*3.5
        oe.a = (rPeriapses + rApoapses) / 2.0
        oe.e = 1.0 - rPeriapses / oe.a
    else:
        print("Unsupported orbit type " + orbitCase + " selected")
        exit(1)
    oe.i = 85.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are arbitrary

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(1. * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    magLog = magModule.envOutMsgs[0].recorder(samplingTime)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, magLog)
    scSim.AddModelToTask(simTaskName, dataLog)
    if planetCase == 'Earth' and orbitCase == 'elliptical':
        mag2Log = magModule2.envOutMsgs[0].recorder(samplingTime)
        scSim.AddModelToTask(simTaskName, mag2Log)

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    magData = magLog.magField_N
    posData = dataLog.r_BN_N
    if planetCase == 'Earth' and orbitCase == 'elliptical':
        magData2 = mag2Log.magField_N

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs

    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    timeAxis = dataLog.times() * macros.NANO2SEC
    for idx in range(3):
        plt.plot(timeAxis / P, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1" + orbitCase + planetCase
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    for idx in range(3):
        plt.plot(timeAxis / P, magData[:, idx] *1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$B\_N_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Magnetic Field [nT]')
    if planetCase == 'Earth' and orbitCase == 'elliptical':
        for idx in range(3):
            plt.plot(timeAxis / P, magData2[:, idx] * 1e9, '--',
                     color=unitTestSupport.getLineColor(idx, 3),
                     label=r'$B\_N_{' + str(idx) + '}$')
    pltName = fileName + "2" + orbitCase + planetCase
    figureList[pltName] = plt.figure(2)


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
        True,          # show_plots
        'circular',  # orbit Case (circular, elliptical)
        'Earth'      # planetCase (Earth, Jupiter)
    )
