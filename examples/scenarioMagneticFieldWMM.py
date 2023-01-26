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

This script sets up a 3-DOF spacecraft which is orbiting the with a
magnetic field model.  This scenario is similar to the centered dipole model
:ref:`scenarioMagneticFieldCenteredDipole`, but here
the World Magnetic Model (WMM) is employed.  This model is specific
to Earth and not suitable for other planets. The purpose
is to illustrate how to create and setup the WMM magnetic field,
as well as determine the
magnetic field at a spacecraft location.  The orbit setup is similar to that used in
:ref:`scenarioBasicOrbit`.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioMagneticFieldWMM.py

Simulation Scenario Setup Details
---------------------------------

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains the spacecraft object.  The spacecraft state message is connected to the magnetic field
module which outputs the local magnetic field in inertial frame components.

.. image:: /_images/static/test_scenario_MagneticFieldWMM.svg
   :align: center

When the simulation completes 2 plots are shown for each case.  One plot always shows
the inertial position vector components, while the second plot
shows the local magnetic field
vector components with respect to the inertial frame.

As this :ref:`MagneticFieldWMM` model is specific to Earth, there are
no parameters to set of tune.  Rather, the ``WMM.COF`` WMM coefficient
file is loaded from the ``dataPath`` variable.

The default planet's position vector is assumed to be the inertial
frame origin and an identity orientation matrix.
If a different planet state message is required this can be
specified through the optional input message ``planetPosInMsgName``.

As with :ref:`scenarioMagneticFieldCenteredDipole`, the magnetic
field module can produce the magnetic field for a vector of spacecraft
locations, not just for a single spacecraft.

The WMM module requires an epoch time to determine the magnetic field.
If this is not set, then the BSK
default epoch time is used.  To set a general epoch time, the module
can read in an epoch message with a
gregorian UTC date.  This is set using the the support method
``timeStringToGregorianUTCMsg``.

The WMM model is driven of a time variable that is a decimal year value.
The module can set this as well by specifying the module parameter
``epochDateFractionalYear``.  However, note that if the epoch message is
specified, the message information is used instead of the
``epochDateFractionalYear`` variable.

Every time a spacecraft is added to the magnetic field module, an
automated output message name is created. For `magModule` is "WMM_0_data"
as the ModelTag string is ``WMM`` and the spacecraft number is 0.
This output name is created in the  ``addSpacecraftToModel()``
function.  However, if the default output name is used for the second
planetary magnetic field model, then both module share  the same
output name and one will overwrite the others output.

The reach of the magnetic field model is specified through the
module variables ``envMinReach`` and ``envMaxReach``. Their
default values are -1 which turns off this feature, giving
the magnetic field evaluation infinite reach.

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True, orbitCase='circular'

.. image:: /_images/Scenarios/scenarioMagneticFieldWMM1circular.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMagneticFieldWMM2circular.svg
   :align: center

::

   show_plots = True, orbitCase='elliptical'

.. image:: /_images/Scenarios/scenarioMagneticFieldWMM1elliptical.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMagneticFieldWMM2elliptical.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test illustrating how to use a world magnetic model (WMM) for spacecraft about Earth.
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
fileName = os.path.basename(os.path.splitext(__file__)[0])


# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import magneticFieldWMM
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport)

#attempt to import vizard
from Basilisk.utilities import vizSupport


def run(show_plots, orbitCase):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        orbitCase (str): {'circular', 'elliptical'}

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
    simulationTimeStep = macros.sec2nano(60.)
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
    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    req = planet.radEquator

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # create the magnetic field
    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.ModelTag = "WMM"
    magModule.dataPath = bskPath + '/supportData/MagneticField/'

    # set the minReach and maxReach values if on an elliptic orbit
    if orbitCase == 'elliptical':
        magModule.envMinReach = 10000*1000.
        magModule.envMaxReach = 20000*1000.

    # set epoch date/time message
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2019 June 27, 10:23:0.0 (UTC)')

    # add spacecraft to the magnetic field module so it can read the sc position messages
    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple

    # add the magnetic field module to the simulation task stack
    scSim.AddModelToTask(simTaskName, magModule)

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
    # next lines stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are arbitrary
    oe = orbitalMotion.rv2elem(mu, rN, vN)

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(1. * P)

    # connect messages
    magModule.epochInMsg.subscribeTo(epochMsg)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    magLog = magModule.envOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, magLog)

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                                  # saveFile=fileName,
                                                  )
        viz.epochInMsg.subscribeTo(epochMsg)

        viz.settings.show24hrClock = 1
        viz.settings.showDataRateDisplay = 1

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

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs

    timeAxis = dataLog.times() * macros.NANO2SEC
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    rData = []
    for idx in range(0, len(posData)):
        rMag = np.linalg.norm(posData[idx])
        rData.append(rMag / 1000.)
    plt.plot(timeAxis / P, rData, color='#aa0000')
    if orbitCase == 'elliptical':
        plt.plot(timeAxis / P, [magModule.envMinReach/1000.]*len(rData), color='#007700', dashes=[5, 5, 5, 5])
        plt.plot(timeAxis / P, [magModule.envMaxReach / 1000.] * len(rData),
                 color='#007700', dashes=[5, 5, 5, 5])

    plt.xlabel('Time [orbits]')
    plt.ylabel('Radius [km]')
    plt.ylim(min(rData)*0.9, max(rData)*1.1)
    figureList = {}
    pltName = fileName + "1" + orbitCase
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
    pltName = fileName + "2" + orbitCase
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
        'elliptical',  # orbit Case (circular, elliptical)
    )
