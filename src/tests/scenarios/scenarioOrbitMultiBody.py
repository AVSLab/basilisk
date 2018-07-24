''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus() and gravity modules.  Illustrates
# how to setup an orbital simulation that uses multiple gravitational bodies.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 8, 2016
#


import os
import numpy as np
from datetime import datetime
from datetime import timedelta

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import astroFunctions

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import simIncludeGravBody
from Basilisk import pyswice
from Basilisk import __path__
bskPath = __path__[0]



## \defgroup Tutorials_1_3
##   @{
## How to setup orbital simulation with multiple gravitational bodies.
#
# Orbit Setup to Simulate Translation with Multiple Gravitational Bodies {#scenarioOrbitMultiBody}
# ====
#
# Scenario Description
# -----
# This script sets up a 3-DOF spacecraft which is traveling in a multi-gravity environment.  The purpose
# is to illustrate how to attach a multiple gravity model, and compare the output to SPICE generated
# trajectories.  The scenarios can be run with the followings setups
# parameters:
# Setup | scCase
# ----- | -------------------
# 1     | Hubble
# 2     | New Horizon
#
# To run the default scenario 1, call the python script through
#
#       python scenarioOrbitMultiBody.py
#
# When the simulation completes 2-3 plots are shown for each case.  One plot always shows
# the inertial position vector components, while the third plot shows the inertial differences
# between the Basilisk simulation trajectory and the SPICE spacecraft trajectory.  Read
# [scenarioBasicOrbit.py](@ref scenarioBasicOrbit) to learn how to setup an
# orbit simulation.
#
# The simulation layout is shown in the following illustration.  The SPICE interface object keeps track of
# the selection celestial objects, and ensures the gravity body object has the correct locations at
# each time step.
# ![Simulation Flow Diagram](Images/doc/test_scenarioOrbitMultiBody.svg "Illustration")
#
# The spacecraftPlus() module is setup as before, except that we need to specify a priority to this task.
# ~~~~~~~~~~~~~~~~~{.py}
#     # initialize spacecraftPlus object and set properties
#     scObject = spacecraftPlus.SpacecraftPlus()
#     scObject.ModelTag = "spacecraftBody"
#
#     # add spacecraftPlus object to the simulation process
#     scSim.AddModelToTask(simTaskName, scObject, None, 1)
# ~~~~~~~~~~~~~~~~~
# If BSK modules are added to the simulation task process, they are executed in the order that they are added
# However, we the execution order needs to be control, a priority can be assigned.  The model with a higher priority
# number is executed first.  Modules with unset priorities will be given a priority of -1 which
# puts them at the
# very end of the execution frame.  They will get executed in the order in which they were added.
# For this scenario scripts, it is critical that the Spice object task is evaluated
# before the spacecraftPlus() model.  Thus, below the Spice object is added with a higher priority task.
#
# The first step to create a fresh gravity body factor class through
# ~~~~~~~~~~~~~~~~~{.py}
#   gravFactory = simIncludeGravBody.gravBodyFactory()
# ~~~~~~~~~~~~~~~~~
# This clears out the list of gravitational bodies, especially ifthe script is
# run multiple times using 'py.test' or in Monte-Carlo runs.
# Next a series of gravitational bodies are included.  Note that it is convenient to include them as a
# list of SPICE names.  The Earth is included in this scenario with the
# spherical harmonics turned on.  Note that this is true for both spacecraft simulations.
# ~~~~~~~~~~~~~~~~~{.py}
#    gravBodies = gravFactory.createBodies(['earth', 'mars barycenter', 'sun', 'moon', "jupiter barycenter"])
#    gravBodies['earth'].isCentralBody = True
#    gravBodies['earth'].useSphericalHarmParams = True
#    imIncludeGravBody.loadGravFromFile(bskPath +'/supportData/LocalGravData/GGM03S.txt'
#                                     , gravBodies['earth'].spherHarm
#                                     , 100
#                                     )
# ~~~~~~~~~~~~~~~~~
# The configured gravitational bodies are addes to the spacecraft dynamics with the usual command:
# ~~~~~~~~~~~~~~~~~{.py}
#    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())
# ~~~~~~~~~~~~~~~~~
#
# Next, the default SPICE support module is created and configured.  The first step is to store
# the date and time of the start of the simulation.
# ~~~~~~~~~~~~~~~~~{.py}
#       timeInitString = "2012 MAY 1 00:28:30.0"
#       spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
#       timeInit = datetime.strptime(timeInitString,spiceTimeStringFormat)
# ~~~~~~~~~~~~~~~~~
# The following is a support macro that creates a `spiceObject` instance, and fills in typical
# default parameters.
# ~~~~~~~~~~~~~~~~~{.py}
#       gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/', timeInitString)
# ~~~~~~~~~~~~~~~~~
# Next the SPICE module is costumized.  The first step is to specify the zeroBase.  This is the inertial
# origin relative to which all spacecraft message states are taken.  The simulation defaults to all
# planet or spacecraft ephemeris being given in the SPICE object default frame, which is the solar system barycenter
# or SSB for short.  The spacecraftPlus() state output message is relative to this SBB frame by default.  To change
# this behavior, the zero based point must be redefined from SBB to another body.  In this simulation we use the Earth.
# ~~~~~~~~~~~~~~~~~{.py}
#   gravFactory.spiceObject.zeroBase = 'Earth'
# ~~~~~~~~~~~~~~~~~
# Finally, the SPICE object is added to the simulation task list.
# ~~~~~~~~~~~~~~~~~{.py}
#       scSim.AddModelToTask(simTaskName, gravFactory.spiceObject, None, -1)
# ~~~~~~~~~~~~~~~~~
# To unload the loaded SPICE kernels, use
# ~~~~~~~~~~~~~~~~~{.py}
# gravFactory.unloadSpiceKernels()
# ~~~~~~~~~~~~~~~~~
# This will unload all the kernels that the gravital body factory loaded earlier.
#
# Next we would like to import spacecraft specific SPICE ephemeris data into the python enviroment.  This is done
# such that the BSK computed trajectories can be compared in python with the equivalent SPICE directories.
# Note that this python SPICE setup is different from the BSK SPICE setup that was just completed.  As a result
# it is required to load in all the required SPICE kernels.  The following code is used to load either
# spacecraft data.
# ~~~~~~~~~~~~~~~~~{.py}
#     if scCase is 'NewHorizons':
#        scEphemerisFileName = 'nh_pred_od077.bsp'
#         scSpiceName = 'NEW HORIZONS'
#         vizPlanetName = "sun"
#     else:  # default case
#         scEphemerisFileName = 'hst_edited.bsp'
#         scSpiceName = 'HUBBLE SPACE TELESCOPE'
#         vizPlanetName = "earth"
#     pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + scEphemerisFileName)  # Hubble Space Telescope data
#     pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
#     pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
#     pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
#     pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel
# ~~~~~~~~~~~~~~~~~
# To unload the SPICE kernels loaded into the Python environment, use
# ~~~~~~~~~~~~~~~~~{.py}
#     pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
#     pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
#     pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
#     pyswice.unload_c(gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel
# ~~~~~~~~~~~~~~~~~
#
#
# The initial spacecraft position and velocity vector is obtained via the SPICE function call:
# ~~~~~~~~~~~~~~~~~{.py}
#       scInitialState = 1000*pyswice.spkRead(scSpiceName, timeInitString, 'J2000', 'EARTH')
#       rN = scInitialState[0:3]         # meters
#       vN = scInitialState[3:6]         # m/s
# ~~~~~~~~~~~~~~~~~
# Note that these vectors are given here relative to the Earth frame.  When we set the spacecraftPlus()
# initial position and velocity vectors through before initialization
# ~~~~~~~~~~~~~~~~~{.py}
#     scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
#     scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N
# ~~~~~~~~~~~~~~~~~
# the natural question arises, how does Basilisk know relative to what frame these states are defined?  This is
# actually setup above where we set `.isCentralBody = True` and mark the Earth as are central body.
# Without this statement, the code would assume the spacecraftPlus() states are relative to the default zeroBase frame.
# In the earlier basic orbital motion script (@ref scenarioBasicOrbit) this subtleties were not discussed.
# This is because there
# the planets ephemeris message is being set to the default messages which zero's both the position and orientation
# states.  However, if Spice is used to setup the bodies, the zeroBase state must be carefully considered.
#
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,        # show_plots
#          'Hubble'
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The remaining argument(s) control the
# simulation scenario flags to turn on or off certain simulation conditions.  The default
# scenario simulates the Hubble Space Telescope (HST) spacecraft about the Earth in a LEO orbit.
# The resulting position coordinates and orbit illustration are shown below.  A 2000 second simulation is
# performed, and the Basilisk and SPICE generated orbits match up very well.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioOrbitMultiBody1Hubble.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioOrbitMultiBody2Hubble.svg "Orbit Illustration")
# ![Trajectory Differences](Images/Scenarios/scenarioOrbitMultiBody3Hubble.svg "Trajectory Differences")
#
# Setup 2
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,        # show_plots
#          'NewHorizons'
#        )
# ~~~~~~~~~~~~~
# This case illustrates a simulation of the New Horizons spacecraft.  Here the craft is already a very
# large distance from the sun.  The
# resulting position coordinates and trajectorie differences are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioOrbitMultiBody1NewHorizons.svg "Position history")
# ![Trajectory Difference](Images/Scenarios/scenarioOrbitMultiBody3NewHorizons.svg "Trajectory Difference")
#
## @}
def run(show_plots, scCase):
    '''Call this routine directly to run the tutorial scenario.'''

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(5.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # unitTestSupport.enableVisualization(scSim, dynProcess, simProcessName, 'earth')  # The Viz only support 'earth', 'mars', or 'sun'

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, None, 1)

    # setup Gravity Bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'mars barycenter', 'sun', 'moon', "jupiter barycenter"])
    gravBodies['earth'].isCentralBody = True
    # Other possible ways to access specific gravity bodies include the below
    #   earth = gravBodies['earth']
    #   earth = gravFactory.createEarth()
    gravBodies['earth'].useSphericalHarmParams = True
    simIncludeGravBody.loadGravFromFile(bskPath +'/supportData/LocalGravData/GGM03S.txt'
                                     , gravBodies['earth'].spherHarm
                                     , 100
                                     )
    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    # setup simulation start date/time
    timeInitString = "2012 MAY 1 00:28:30.0"
    spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
    timeInit = datetime.strptime(timeInitString, spiceTimeStringFormat)

    # setup SPICE module
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/', timeInitString)
    # by default the SPICE object will use the solar system barycenter as the inertial origin
    # If the spacecraftPlus() output is desired relative to another celestial object, the zeroBase string
    # name of the SPICE object needs to be changed.
    gravFactory.spiceObject.zeroBase = 'Earth'

    # add spice interface object to task list
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject, None, -1)

    # Use the python spice utility to load in spacecraft SPICE ephemeris data
    # Note: this following SPICE data only lives in the Python environment, and is
    #       separate from the earlier SPICE setup that was loaded to BSK.  This is why
    #       all required SPICE libraries must be included when setting up and loading
    #       SPICE kernals in Python.
    if scCase is 'NewHorizons':
        scEphemerisFileName = 'nh_pred_od077.bsp'
        scSpiceName = 'NEW HORIZONS'
        vizPlanetName = "sun"
    else:  # default case
        scEphemerisFileName = 'hst_edited.bsp'
        scSpiceName = 'HUBBLE SPACE TELESCOPE'
        vizPlanetName = "earth"
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + scEphemerisFileName)  # Hubble Space Telescope data
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
    pyswice.furnsh_c(gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel


    #
    #   Setup spacecraft initial states
    #
    scInitialState = 1000 * pyswice.spkRead(scSpiceName, timeInitString, 'J2000', 'EARTH')
    rN = scInitialState[0:3]  # meters
    vN = scInitialState[3:6]  # m/s
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N

    #
    #   Setup simulation time
    #
    simulationTime = macros.sec2nano(2000.)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileName = os.path.basename(os.path.splitext(__file__)[0])

    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    if scCase is 'NewHorizons':
        axesScale = astroFunctions.AU * 1000.  # convert to AU
        axesLabel = '[AU]'
        timeScale = macros.NANO2MIN  # convert to minutes
        timeLabel = '[min]'
    else:
        axesScale = 1000.  # convert to km
        axesLabel = '[km]'
        timeScale = macros.NANO2MIN  # convert to minutes
        timeLabel = '[min]'
    for idx in range(1, 4):
        plt.plot(posData[:, 0] * timeScale, posData[:, idx] / axesScale,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time ' + timeLabel)
    plt.ylabel('Inertial Position ' + axesLabel)
    figureList = {}
    pltName = fileName + "1" + scCase
    figureList[pltName] = plt.figure(1)

    rBSK = posData[-1, 1:4]  # store the last position to compare to the SPICE position
    if scCase is 'Hubble':
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
            oeData = orbitalMotion.rv2elem(gravBodies['earth'].mu, posData[idx, 1:4], velData[idx, 1:4])
            rData.append(oeData.rmag)
            fData.append(oeData.f + oeData.omega - omega0)
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000
                 , color='#aa0000'
                 , linewidth=0.5
                 , label='Basilisk'
                 )
        plt.legend(loc='lower right')

        # draw the full SPICE orbit
        time = timeInit
        rData = []
        fData = []
        sec = int(macros.NANO2SEC * simulationTime / numDataPoints)
        usec = (macros.NANO2SEC * simulationTime / numDataPoints - sec) * 1000000
        for idx in range(0, numDataPoints):
            time += timedelta(seconds=sec, microseconds=usec)
            timeString = time.strftime(spiceTimeStringFormat)
            scState = 1000.0 * pyswice.spkRead(scSpiceName, timeString, 'J2000', 'EARTH')
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
        time = gravFactory.spiceObject.getCurrentTimeString()
        scState = 1000.0 * pyswice.spkRead(scSpiceName,
                                           gravFactory.spiceObject.getCurrentTimeString(),
                                           'J2000',
                                           'EARTH')
        rTrue = scState[0:3]

    # plot the differences between BSK and SPICE position data
    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    posError = []
    numDataPoints = len(posData)
    for idx in range(0, numDataPoints):
        sec = int(macros.NANO2SEC * posData[idx, 0])
        usec = (macros.NANO2SEC * posData[idx, 0] - sec) * 1000000
        time = timeInit + timedelta(seconds=sec, microseconds=usec)
        timeString = time.strftime(spiceTimeStringFormat)
        scState = 1000 * pyswice.spkRead(scSpiceName, timeString, 'J2000', 'EARTH')
        posError.append(posData[idx, 1:4] - np.array(scState[0:3]))  # meters
    for idx in range(1, 4):
        plt.plot(posData[:, 0] * macros.NANO2MIN, np.array(posError)[:, idx - 1],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$\Delta r_{' + str(idx) + '}$')
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
