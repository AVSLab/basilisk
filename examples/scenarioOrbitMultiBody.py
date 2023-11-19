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

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/oHCyeM1-mKI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

This script sets up a 3-DOF spacecraft which is traveling in a multi-gravity environment.  The purpose
is to illustrate how to attach a multiple gravity model, and compare the output to SPICE generated
trajectories.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioOrbitMultiBody.py

When the simulation completes 2-3 plots are shown for each case.  One plot always shows
the inertial position vector components, while the third plot shows the inertial differences
between the Basilisk simulation trajectory and the SPICE spacecraft trajectory.  Read :ref:`scenarioBasicOrbit`
to learn how to setup an orbit simulation.

The simulation layout is shown in the following illustration.  The SPICE interface object keeps track of
the selection celestial objects, and ensures the gravity body object has the correct locations at
each time step.

.. image:: /_images/static/test_scenarioOrbitMultiBody.svg
   :align: center


Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True, scCase = 0

This scenario simulates the Hubble Space Telescope (HST) spacecraft about the Earth in a LEO orbit.
The resulting position coordinates and orbit illustration are shown below.  A 2000 second simulation is
performed, and the Basilisk and SPICE generated orbits match up very well.

.. image:: /_images/Scenarios/scenarioOrbitMultiBody1Hubble.svg
   :align: center

.. image:: /_images/Scenarios/scenarioOrbitMultiBody2Hubble.svg
   :align: center

.. image:: /_images/Scenarios/scenarioOrbitMultiBody3Hubble.svg
   :align: center

::

    show_plots = True, scCase = 1

This case illustrates a simulation of the New Horizons spacecraft.  Here the craft is already a very
large distance from the sun.  The
resulting position coordinates and trajectories differences are shown below.

.. image:: /_images/Scenarios/scenarioOrbitMultiBody1NewHorizons.svg
   :align: center

.. image:: /_images/Scenarios/scenarioOrbitMultiBody3NewHorizons.svg
   :align: center

"""


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft() and gravity modules.  Illustrates
# how to setup an orbital simulation that uses multiple gravitational bodies.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 8, 2016
#

import os
from datetime import datetime
from datetime import timedelta

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
# import simulation related support
from Basilisk.simulation import spacecraft
# Used to get the location of supporting data.
from Basilisk.topLevelModules import pyswice
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport
from Basilisk.utilities.pyswice_spk_utilities import spkRead

bskPath = __path__[0]

fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, scCase):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        scCase (int):

            ======  ============================
            Int     Definition
            ======  ============================
            0       Hubble trajectory
            1       New Horizon trajectory
            ======  ============================

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
    simulationTimeStep = macros.sec2nano(5.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    # The spacecraft() module is setup as before, except that we need to specify a priority to this task.
    # If BSK modules are added to the simulation task process, they are executed in the order that they are added
    # However, we the execution order needs to be control, a priority can be assigned.  The model with a higher priority
    # number is executed first.  Modules with unset priorities will be given a priority of -1 which
    # puts them at the
    # very end of the execution frame.  They will get executed in the order in which they were added.
    # For this scenario scripts, it is critical that the Spice object task is evaluated
    # before the spacecraft() model.  Thus, below the Spice object is added with a higher priority task.
    scSim.AddModelToTask(simTaskName, scObject, 1)

    # The first step to create a fresh gravity body factor class through
    gravFactory = simIncludeGravBody.gravBodyFactory()
    # This clears out the list of gravitational bodies, especially if the script is
    # run multiple times using 'py.test' or in Monte-Carlo runs.

    # Next a series of gravitational bodies are included.  Note that it is convenient to include them as a
    # list of SPICE names.  The Earth is included in this scenario with the
    # spherical harmonics turned on.  Note that this is true for both spacecraft simulations.
    gravBodies = gravFactory.createBodies(['earth', 'mars barycenter', 'sun', 'moon', "jupiter barycenter"])
    gravBodies['earth'].isCentralBody = True
    # Other possible ways to access specific gravity bodies include the below
    #   earth = gravBodies['earth']
    #   earth = gravFactory.createEarth()
    gravBodies['earth'].useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/GGM03S.txt', 100)
    # The configured gravitational bodies are added to the spacecraft dynamics with the usual command:
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Next, the default SPICE support module is created and configured.  The first step is to store
    # the date and time of the start of the simulation.
    timeInitString = "2012 MAY 1 00:28:30.0"
    spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
    timeInit = datetime.strptime(timeInitString, spiceTimeStringFormat)

    # The following is a support macro that creates a `gravFactory.spiceObject` instance, and fills in typical
    # default parameters.  By setting the epochInMsg argument, this macro provides an epoch date/time
    # message as well.  The spiceObject is set to subscribe to this epoch message.  Using the epoch message
    # makes it trivial to synchronize the epoch information across multiple modules.
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)

    # By default the SPICE object will use the solar system barycenter as the inertial origin
    # If the spacecraft() output is desired relative to another celestial object, the zeroBase string
    # name of the SPICE object needs to be changed.
    # Next the SPICE module is customized.  The first step is to specify the zeroBase.  This is the inertial
    # origin relative to which all spacecraft message states are taken.  The simulation defaults to all
    # planet or spacecraft ephemeris being given in the SPICE object default frame, which is the solar system barycenter
    # or SSB for short.  The spacecraft() state output message is relative to this SBB frame by default.  To change
    # this behavior, the zero based point must be redefined from SBB to another body.
    # In this simulation we use the Earth.
    gravFactory.spiceObject.zeroBase = 'Earth'

    # Finally, the SPICE object is added to the simulation task list.
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)

    # Next we would like to import spacecraft specific SPICE ephemeris data into the python environment.  This is done
    # such that the BSK computed trajectories can be compared in python with the equivalent SPICE directories.
    # Note that this python SPICE setup is different from the BSK SPICE setup that was just completed.  As a result
    # it is required to load in all the required SPICE kernels.  The following code is used to load either
    # spacecraft data.
    # Note: this following SPICE data only lives in the Python environment, and is
    #       separate from the earlier SPICE setup that was loaded to BSK.  This is why
    #       all required SPICE libraries must be included when setting up and loading
    #       SPICE kernals in Python.
    if scCase == 'NewHorizons':
        scEphemerisFileName = 'nh_pred_od077.bsp'
        scSpiceName = 'NEW HORIZONS'
    else:  # default case
        scEphemerisFileName = 'hst_edited.bsp'
        scSpiceName = 'HUBBLE SPACE TELESCOPE'
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + scEphemerisFileName)  # Hubble Space Telescope data
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

    #
    #   Setup spacecraft initial states
    #
    # The initial spacecraft position and velocity vector is obtained via the SPICE function call:
    scInitialState = 1000 * spkRead(scSpiceName, timeInitString, 'J2000', 'EARTH')
    rN = scInitialState[0:3]  # meters
    vN = scInitialState[3:6]  # m/s

    # Note that these vectors are given here relative to the Earth frame.  When we set the spacecraft()
    # initial position and velocity vectors through before initialization
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N
    # the natural question arises, how does Basilisk know relative to what frame these states are defined?  This is
    # actually setup above where we set `.isCentralBody = True` and mark the Earth as are central body.
    # Without this statement, the code would assume the spacecraft() states are
    # relative to the default zeroBase frame.
    # In the earlier basic orbital motion script (@ref scenarioBasicOrbit) this subtleties were not discussed.
    # This is because there
    # the planets ephemeris message is being set to the default messages which zero's both the position and orientation
    # states.  However, if Spice is used to setup the bodies, the zeroBase state must be carefully considered.

    #
    #   Setup simulation time
    #
    simulationTime = macros.sec2nano(2000.)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 50
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataRec)

    # if this scenario is to interface with the BSK Unity Viz, uncomment the following lines
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    #
    #   initialize Simulation
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
    posData = dataRec.r_BN_N
    velData = dataRec.v_BN_N
    timeAxis = dataRec.times()

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
    ax.yaxis.set_major_formatter(matplotlib.ticker.StrMethodFormatter('{x:,.0f}'))
    if scCase == 'NewHorizons':
        axesScale = astroFunctions.AU * 1000.  # convert to AU
        axesLabel = '[AU]'
        timeScale = macros.NANO2MIN  # convert to minutes
        timeLabel = '[min]'
    else:
        axesScale = 1000.  # convert to km
        axesLabel = '[km]'
        timeScale = macros.NANO2MIN  # convert to minutes
        timeLabel = '[min]'
    for idx in range(3):
        plt.plot(timeAxis * timeScale, posData[:, idx] / axesScale,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time ' + timeLabel)
    plt.ylabel('Inertial Position ' + axesLabel)
    figureList = {}
    pltName = fileName + "1" + scCase
    figureList[pltName] = plt.figure(1)

    rBSK = posData[-1]  # store the last position to compare to the SPICE position
    if scCase == 'Hubble':
        #
        # draw orbit in perifocal frame
        #
        oeData = orbitalMotion.rv2elem(gravBodies['earth'].mu, rN, vN)
        omega0 = oeData.omega
        b = oeData.a * np.sqrt(1 - oeData.e * oeData.e)
        p = oeData.a * (1 - oeData.e * oeData.e)
        plt.figure(2, figsize=np.array((1.0, b / oeData.a)) * 4.75, dpi=100)
        plt.axis(np.array([-oeData.rApoap, oeData.rPeriap, -b, b]) / 1000 * 1.25)

        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()
        planetColor = '#008800'
        planetRadius = gravBodies['earth'].radEquator / 1000
        ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))

        # draw the actual orbit
        rData = []
        fData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(gravBodies['earth'].mu, posData[idx], velData[idx])
            rData.append(oeData.rmag)
            fData.append(oeData.f + oeData.omega - omega0)
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000
                 , color='#aa0000'
                 , linewidth=0.5
                 , label='Basilisk'
                 )
        plt.legend(loc='lower right')

        # draw the full SPICE orbit
        rData = []
        fData = []

        for idx in range(len(timeAxis)):
            simTime = timeAxis[idx] * macros.NANO2SEC
            sec = int(simTime)
            usec = (simTime - sec) * 1000000
            time = timeInit + timedelta(seconds=sec, microseconds=usec)
            timeString = time.strftime(spiceTimeStringFormat)
            scState = 1000.0 * spkRead(scSpiceName, timeString, 'J2000', 'EARTH')
            rN = scState[0:3]  # meters
            vN = scState[3:6]  # m/s
            oeData = orbitalMotion.rv2elem(gravBodies['earth'].mu, rN, vN)
            rData.append(oeData.rmag)
            fData.append(oeData.f + oeData.omega - omega0)
            rTrue = rN  # store the last position to compare to the BSK position
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000
                 , '--'
                 , color='#555555'
                 , linewidth=1.0
                 , label='Spice'
                 )
        plt.legend(loc='lower right')
        plt.xlabel('$i_e$ Cord. [km]')
        plt.ylabel('$i_p$ Cord. [km]')
        plt.grid()
        pltName = fileName + "2" + scCase
        figureList[pltName] = plt.figure(2)

    else:
        scState = 1000.0 * spkRead(scSpiceName,
                                   gravFactory.spiceObject.getCurrentTimeString(),
                                   'J2000',
                                   'EARTH')
        rTrue = scState[0:3]

    # plot the differences between BSK and SPICE position data
    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    # ax.yaxis.set_major_formatter(matplotlib.ticker.StrMethodFormatter('{x:,.0f}'))
    posError = []
    for idx in range(len(timeAxis)):
        simTime = timeAxis[idx] * macros.NANO2SEC
        sec = int(simTime)
        usec = (simTime - sec) * 1000000
        time = timeInit + timedelta(seconds=sec, microseconds=usec)
        timeString = time.strftime(spiceTimeStringFormat)
        scState = 1000 * spkRead(scSpiceName, timeString, 'J2000', 'EARTH')
        posError.append(posData[idx] - np.array(scState[0:3]))  # meters
    for idx in range(3):
        plt.plot(dataRec.times() * macros.NANO2MIN, np.array(posError)[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\Delta r_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position Differences [m]')
    pltName = fileName + "3" + scCase
    figureList[pltName] = plt.figure(3)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    #
    #  unload the SPICE libraries that were loaded by the pyswice utility and the spiceObject earlier
    #
    gravFactory.unloadSpiceKernels()
    pyswice.unload_c(scEphemerisFileName)
    pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
    pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
    pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
    pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return rBSK, rTrue, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        'Hubble'  # 'Hubble' or 'NewHorizons'
    )
