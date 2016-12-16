''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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



import pytest
import sys, os, inspect
import numpy as np
import ctypes
import math
import csv
import logging

from datetime import datetime
from datetime import timedelta

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')
sys.path.append(splitPath[0] + '/Basilisk/Utilities/pyswice/_UnitTest')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion
import astroFunctions

# import simulation related support
import spacecraftPlus
import gravityEffector
import simIncludeGravity

import spice_interface
import pyswice
import pyswice_ck_utilities







# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
@pytest.mark.xfail(True, reason="Scott's brain no-worky\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("scCase", [
      (0)
    , (1)
])

# provide a unique test method name, starting with test_
def test_scenarioOrbitMultiBody(show_plots, scCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, scCase)
    assert testResults < 1, testMessage



## This scenario demonstrates how to setup orbital simulation with multiple gravitational bodies.
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
# 1     | 0 (Hubble Space Telescope)
# 2     | 1 (New Horizons)
#
# To run the default scenario 1, call the python script through
#
#       python test_scenarioOrbitMultiBody.py
#
# When the simulation completes 2-3 plots are shown for each case.  One plot always shows
# the inertial position vector components, while the third plot shows the inertial differences
# between the Basilisk simulation trajectory and the SPICE spacecraft trajectory.  Read
# [test_scenarioBasicOrbit.py](@ref scenarioBasicOrbit) to learn how to setup an
# orbit simulation.
#
# The spacecraftPlus() module is setup as before, except that it isn't added to the simulation task
# list until all the gravitational bodies are added.  The Earth is included in this scenario with the
# spherical harmonics turned on.  Not that this is true for both spacecraft simulations.
#~~~~~~~~~~~~~~~~~{.py}
#       earthGravBody, ephemData = simIncludeGravity.addEarth()
#       earthGravBody.isCentralBody = True          # ensure this is the central gravitational body
#       earthGravBody.useSphericalHarmParams = True
#       gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/LocalGravData/GGM03S.txt'
#                                      , earthGravBody.spherHarm
#                                      ,100
#                                      )
#~~~~~~~~~~~~~~~~~
# Next, this the gravity support macros are used to create the other planetary bodies.  Note that
# the default (all zero) planet ephemerise data sets from these support macros are not used in this
# scenario.
#~~~~~~~~~~~~~~~~~{.py}
#       sunGravBody, ephemData = simIncludeGravity.addSun()
#       moonGravBody, ephemData = simIncludeGravity.addMoon()
#       marsGravBody, ephemData = simIncludeGravity.addMars()
#       jupiterGravBody, ephemData = simIncludeGravity.addJupiter()
#
#       # attach gravity model to spaceCraftPlus
#       scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([earthGravBody, sunGravBody, marsGravBody,
#                                                                      moonGravBody, jupiterGravBody])
#~~~~~~~~~~~~~~~~~
# The bodies are then attached to spacecraftPlus() as a list of gravitational bodies.  Next, the
# SPICE module is create and configured:
#~~~~~~~~~~~~~~~~~{.py}
#       # setup SPICE ephemerise support
#       spiceObject = spice_interface.SpiceInterface()
#       spiceObject.ModelTag = "SpiceInterfaceData"
#       spiceObject.SPICEDataPath = splitPath[0] + '/Basilisk/External/EphemerisData/'
#       spiceObject.OutputBufferCount = 10000
#       spiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun", "moon", "jupiter barycenter"])
#
#       #
#       # pull in SPICE support libraries
#       #
#       pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de430.bsp')           # solar system bodies
#       pyswice.furnsh_c(spiceObject.SPICEDataPath + 'naif0011.tls')        # leap second file
#       pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de-403-masses.tpc')   # solar system masses
#       pyswice.furnsh_c(spiceObject.SPICEDataPath + 'pck00010.tpc')        # generic Planetary Constants Kernel
#       # load in spacecraft SPICE ephemeris data
#       if scCase == 1:
#           scEphemerisName = 'nh_pred_od077.bsp'
#           scSpiceName = 'NEW HORIZONS'
#           mu = sunGravBody.mu
#       else:   # default case 0
#           scEphemerisName = 'hst_edited.bsp'
#           scSpiceName = 'HUBBLE SPACE TELESCOPE'
#           mu = earthGravBody.mu
#       pyswice.furnsh_c(spiceObject.SPICEDataPath + scEphemerisName)      # Hubble Space Telescope data
#
#       spiceObject.UTCCalInit = "2012 MAY 1 00:28:30.0"
#       timeInitString = spiceObject.UTCCalInit
#       spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
#       timeInit = datetime.strptime(timeInitString,spiceTimeStringFormat)
#
#       scSim.AddModelToTask(simTaskName, spiceObject)
#~~~~~~~~~~~~~~~~~
# Note that the SPICE module requires the time to be provided as a text string formated in a particular
# manner.  Finally, the spacecraftPlus() is added to the task list.
#~~~~~~~~~~~~~~~~~{.py}
#     scSim.AddModelToTask(simTaskName, scObject)
#~~~~~~~~~~~~~~~~~
#
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          0            # orbit Case (0 - HST, 1 - New Horizon)
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The remaining argument(s) control the
# simulation scenario flags to turn on or off certain simulation conditions.  The default
# scenario simulates the Hubble Space Telescope (HST) spacecraft about the Earth in a LEO orbit.
# The resulting position coordinates and orbit illustration are shown below.  A 2000 second simulation is
# performed, and the Basilisk and SPICE generated orbits match up very well.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioOrbitMultiBody10.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioOrbitMultiBody20.svg "Orbit Illustration")
# ![Trajectory Differences](Images/Scenarios/scenarioOrbitMultiBody30.svg "Trajectory Differences")
#
# Setup 2
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          1            # orbit Case (0 - HST, 1 - New Horizon)
#        )
# ~~~~~~~~~~~~~
# This case illustrates a simulation of the New Horizons spacecraft.  Here the craft is already a very
# large distance from the sun.  The
# resulting position coordinates and trajectorie differences are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioOrbitMultiBody11.svg "Position history")
# ![Trajectory Difference](Images/Scenarios/scenarioOrbitMultiBody31.svg "Trajectory Difference")
#
def run(doUnitTests, show_plots, scCase):
    '''Call this routine directly to run the tutorial scenario.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #
    #  From here on there scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

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


    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = False


    # setup Gravity Bodies
    earthGravBody, ephemData = simIncludeGravity.addEarth()
    # NOTE: default ephemData is not used in this setup, this comes from SPICE
    earthGravBody.isCentralBody = True          # ensure this is the central gravitational body
    earthGravBody.useSphericalHarmParams = True
    gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/LocalGravData/GGM03S.txt'
                                     , earthGravBody.spherHarm
                                     ,100
                                     )

    sunGravBody, ephemData = simIncludeGravity.addSun()
    moonGravBody, ephemData = simIncludeGravity.addMoon()
    marsGravBody, ephemData = simIncludeGravity.addMars()
    jupiterGravBody, ephemData = simIncludeGravity.addJupiter()

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([earthGravBody, sunGravBody, marsGravBody,
                                                                   moonGravBody, jupiterGravBody])


    # setup SPICE ephemerise support
    spiceObject = spice_interface.SpiceInterface()
    spiceObject.ModelTag = "SpiceInterfaceData"
    spiceObject.SPICEDataPath = splitPath[0] + '/Basilisk/External/EphemerisData/'
    spiceObject.OutputBufferCount = 10000
    spiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun", "moon", "jupiter barycenter"])

    #
    # pull in SPICE support libraries
    #
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de430.bsp')           # solar system bodies
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'naif0011.tls')        # leap second file
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de-403-masses.tpc')   # solar system masses
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'pck00010.tpc')        # generic Planetary Constants Kernel
    # load in spacecraft SPICE ephemeris data
    if scCase == 1:
        scEphemerisName = 'nh_pred_od077.bsp'
        scSpiceName = 'NEW HORIZONS'
        mu = sunGravBody.mu
    else:   # default case 0
        scEphemerisName = 'hst_edited.bsp'
        scSpiceName = 'HUBBLE SPACE TELESCOPE'
        mu = earthGravBody.mu
    pyswice.furnsh_c(spiceObject.SPICEDataPath + scEphemerisName)      # Hubble Space Telescope data


    spiceObject.UTCCalInit = "2012 MAY 1 00:28:30.0"
    timeInitString = spiceObject.UTCCalInit
    spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
    timeInit = datetime.strptime(timeInitString,spiceTimeStringFormat)

    scSim.AddModelToTask(simTaskName, spiceObject)

    # add spacecraftPlus object to the simulation process
    # Note: this step must happen after the spiceOjbect is added to the task list
    scSim.AddModelToTask(simTaskName, scObject)

    #
    #   Setup spacecraft initial states
    #
    scInitialState = 1000*pyswice_ck_utilities.spkRead(scSpiceName, timeInitString, 'J2000', 'EARTH')
    rN = scInitialState[0:3]         # meters
    vN = scInitialState[3:6]         # m/s


    #
    #   Setup simulation time
    #
    simulationTime = macros.sec2nano(2000.)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)



    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()


    #
    #   initialize Spacecraft States within the state manager
    #   this must occur after the initialization
    #
    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")

    posRef.setState(unitTestSupport.np2EigenVector3d(rN))  # m - r_BN_N
    velRef.setState(unitTestSupport.np2EigenVector3d(vN))  # m - v_BN_N


    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()


    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_BN_N',range(3))

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]

    # draw the inertial position vector components
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    if scCase == 1:
        axesScale = astroFunctions.AU*1000.     # convert to AU
        axesLabel = '[AU]'
        timeScale = macros.NANO2MIN             # convert to minutes
        timeLabel = '[min]'
    else:
        axesScale = 1000.                       # convert to km
        axesLabel = '[km]'
        timeScale = macros.NANO2MIN             # convert to minutes
        timeLabel = '[min]'
    for idx in range(1,4):
        plt.plot(posData[:, 0]*timeScale, posData[:, idx]/axesScale,
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$r_{BN,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time ' + timeLabel)
    plt.ylabel('Inertial Position ' + axesLabel)
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"1"+str(int(scCase))
            , plt, path)

    rBSK = posData[-1, 1:4]  # store the last position to compare to the SPICE position
    if scCase == 0:
        #
        # draw orbit in perifocal frame
        #
        oeData = orbitalMotion.rv2elem(mu,posData[0,1:4],velData[0,1:4])
        omega0 = oeData.omega
        b = oeData.a*np.sqrt(1-oeData.e*oeData.e)
        p = oeData.a*(1-oeData.e*oeData.e)
        plt.figure(2,figsize=np.array((1.0, b/oeData.a))*4.75,dpi=100)
        plt.axis(np.array([-oeData.rApoap, oeData.rPeriap, -b, b])/1000*1.25)

        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()
        planetColor= '#008800'
        planetRadius = earthGravBody.radEquator/1000
        ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))

        # draw the actual orbit
        rData=[]
        fData=[]
        for idx in range(0,len(posData)):
            oeData = orbitalMotion.rv2elem(mu,posData[idx,1:4],velData[idx,1:4])
            rData.append(oeData.rmag)
            fData.append(oeData.f + oeData.omega - omega0)
        plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
                 ,color='#aa0000'
                 ,linewidth = 0.5
                 ,label = 'Basilisk'
                 )
        plt.legend(loc='lower right')

        # draw the full SPICE orbit
        time = timeInit
        rData=[]
        fData=[]
        sec = int(macros.NANO2SEC * simulationTime / numDataPoints)
        usec = (macros.NANO2SEC * simulationTime / numDataPoints - sec) * 1000000
        for idx in range(0,numDataPoints):
            time += timedelta(seconds = sec, microseconds=usec)
            timeString = time.strftime(spiceTimeStringFormat)
            scState = 1000.0*pyswice_ck_utilities.spkRead(scSpiceName, timeString, 'J2000', 'EARTH')
            rN = scState[0:3]  # meters
            vN = scState[3:6]  # m/s
            oeData = orbitalMotion.rv2elem(mu, rN, vN)
            rData.append(oeData.rmag)
            fData.append(oeData.f + oeData.omega - omega0)
            rTrue = rN      # store the last position to compare to the BSK position
        plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
                 ,'--'
                 , color='#555555'
                 , linewidth = 1.0
                 ,label = 'Spice'
                 )
        plt.legend(loc='lower right')
        plt.xlabel('$i_e$ Cord. [km]')
        plt.ylabel('$i_p$ Cord. [km]')
        plt.grid()
        if doUnitTests:     # only save off the figure if doing a unit test run
            unitTestSupport.saveScenarioFigure(
                fileNameString+"2"+str(int(scCase))
                , plt, path)
    else:
        scState = 1000.0*pyswice_ck_utilities.spkRead(scSpiceName, spiceObject.getCurrentTimeString(), 'J2000', 'EARTH')
        rTrue = scState[0:3]

    # plot the differences between BSK and SPICE position data
    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    posError = [];
    numDataPoints = len(posData)
    for idx in range(0, numDataPoints):
        sec = int(macros.NANO2SEC*posData[idx, 0])
        usec = (macros.NANO2SEC*posData[idx, 0] - sec) * 1000000
        time = timeInit +  timedelta(seconds=sec, microseconds=usec)
        timeString = time.strftime(spiceTimeStringFormat)
        scState = 1000*pyswice_ck_utilities.spkRead(scSpiceName, timeString, 'J2000', 'EARTH')
        posError.append(posData[idx,1:4]-np.array(scState[0:3]))  # meters
    for idx in range(1,4):
        plt.plot(posData[:, 0]*macros.NANO2MIN, np.array(posError)[:,idx-1],
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$\Delta r_{'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]' )
    plt.ylabel('Inertial Position Differences [m]')
    if doUnitTests:  # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString + "3" + str(int(scCase))
            , plt, path)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    #
    #   unload the SPICE libraries that were loaded earlier
    #
    pyswice.unload_c(spiceObject.SPICEDataPath + 'de430.bsp')
    pyswice.unload_c(spiceObject.SPICEDataPath + 'naif0011.tls')
    pyswice.unload_c(spiceObject.SPICEDataPath + 'de-403-masses.tpc')
    pyswice.unload_c(spiceObject.SPICEDataPath + 'pck00010.tpc')
    pyswice.unload_c(spiceObject.SPICEDataPath + scEphemerisName)


    #
    #   the python code below is for the unit testing mode.  If you are studying the scenario
    #   to learn how to run BSK, you can stop reading below this line.
    #

    if doUnitTests:

        # compare the results to the truth values
        accuracy = 100.0 # meters
        testFailCount, testMessages = unitTestSupport.compareVector(
            rTrue, rBSK, accuracy, "|r_BN_N| error",
            testFailCount, testMessages)

        #   print out success message if no error were found
        if testFailCount == 0:
            print "PASSED "
        else:
            print testFailCount
            print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run( False,       # do unit tests
         True,        # show_plots
         0            # orbit Case (0 - HST, 1 - New Horizon)
       )

