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

import pyswice
import pyswice_ck_utilities







# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Scott's brain no-worky\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("scCase", [
      ('Hubble')
    , ('NewHorizon')
])

# provide a unique test method name, starting with test_
def test_scenarioOrbitMultiBody(show_plots, scCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, scCase)
    assert testResults < 1, testMessage



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
#       python test_scenarioOrbitMultiBody.py
#
# When the simulation completes 2-3 plots are shown for each case.  One plot always shows
# the inertial position vector components, while the third plot shows the inertial differences
# between the Basilisk simulation trajectory and the SPICE spacecraft trajectory.  Read
# [test_scenarioBasicOrbit.py](@ref scenarioBasicOrbit) to learn how to setup an
# orbit simulation.
#
# The spacecraftPlus() module is setup as before, except that it isn't added to the simulation task
# list until all the gravitational bodies are added.  The first step is to clear any prior gravity body
# settings using
#~~~~~~~~~~~~~~~~~{.py}
#   simIncludeGravity.clearSetup()
#~~~~~~~~~~~~~~~~~
# This is required if the script is run multiple times using 'py.test' or in Monte-Carlo runs.
# The Earth is included in this scenario with the
# spherical harmonics turned on.  Note that this is true for both spacecraft simulations.
#~~~~~~~~~~~~~~~~~{.py}
#       simIncludeGravity.addEarth()
#       simIncludeGravity.gravBodyList[-1].isCentralBody = True          # ensure this is the central gravitational body
#       simIncludeGravity.gravBodyList[-1].useSphericalHarmParams = True
#       gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/LocalGravData/GGM03S.txt'
#                                      , simIncludeGravity.gravBodyList[-1].spherHarm
#                                      ,100
#                                      )
#~~~~~~~~~~~~~~~~~
# Next, this the gravity support macros are used to create the other planetary bodies.  At the end the
# list of gravitational bodies is passed along the gravity field setting of the spacecraftPlus() object.
#~~~~~~~~~~~~~~~~~{.py}
#       simIncludeGravity.addSun()
#       simIncludeGravity.addMoon()
#       simIncludeGravity.addMars()
#       simIncludeGravity.addJupiter()
#
#       # attach gravity model to spaceCraftPlus
#       scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)
#~~~~~~~~~~~~~~~~~
# Next, the default SPICE support module is created and configured.  The first step is to store
# the date and time of the start of the simulation.
#~~~~~~~~~~~~~~~~~{.py}
#       timeInitString = "2012 MAY 1 00:28:30.0"
#       spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
#       timeInit = datetime.strptime(timeInitString,spiceTimeStringFormat)
#~~~~~~~~~~~~~~~~~
# The following is a support macro that creates a `spiceObject` instance, and fills in typical
# default parameters.
#~~~~~~~~~~~~~~~~~{.py}
#       simIncludeGravity.addSpiceInterface(splitPath[0], timeInitString)
#~~~~~~~~~~~~~~~~~
# Next the SPICE module is costumized.  The first step is to specify the zeroBase.  This is the inertial
# origin relative to which all spacecraft message states are taken.  The simulation defaults to all
# planet or spacecraft ephemeris being given in the SPICE object default frame, which is the solar system barycenter
# or SSB for short.  The spacecraftPlus() state output message is relative to this SBB frame by default.  To change
# this behavior, the zero based point must be redefined from SBB to another body.  In this simulation we use the Earth.
#~~~~~~~~~~~~~~~~~{.py}
#   simIncludeGravity.spiceObject.zeroBase = 'Earth'
#~~~~~~~~~~~~~~~~~
# The next customization is importing spacecraft particular SPICE ephemeris data.  This is done with
# Finally, the spacecraftPlus() is added to the task list.
# ~~~~~~~~~~~~~~~~~{.py}
#       if scCase is 'NewHorizons':
#           scEphemerisName = 'nh_pred_od077.bsp'
#           scSpiceName = 'NEW HORIZONS'
#       else:   # default case
#           scEphemerisName = 'hst_edited.bsp'
#           scSpiceName = 'HUBBLE SPACE TELESCOPE'
#       pyswice.furnsh_c(simIncludeGravity.spiceObject.SPICEDataPath + scEphemerisName)
#~~~~~~~~~~~~~~~~~
# Finally, the SPICE object is added to the simulation task list through the typical call
#~~~~~~~~~~~~~~~~~{.py}
#       scSim.AddModelToTask(simTaskName, simIncludeGravity.spiceObject)
#~~~~~~~~~~~~~~~~~
# At this point the spacecraftPlus() object can be added to the simulation task list.
#~~~~~~~~~~~~~~~~~{.py}
#       scSim.AddModelToTask(simTaskName, scObject)
#~~~~~~~~~~~~~~~~~
#
# The initial spacecraft position and velocity vector is obtained via the SPICE function call:
#~~~~~~~~~~~~~~~~~{.py}
#       scInitialState = 1000*pyswice_ck_utilities.spkRead(scSpiceName, timeInitString, 'J2000', 'EARTH')
#       rN = scInitialState[0:3]         # meters
#       vN = scInitialState[3:6]         # m/s
#~~~~~~~~~~~~~~~~~
# Note that these vectors are given here relative to the Earth frame.  When we set the spacecraftPlus()
# initial position and velocity vectors through
#~~~~~~~~~~~~~~~~~{.py}
#       posRef = scObject.dynManager.getStateObject("hubPosition")
#       velRef = scObject.dynManager.getStateObject("hubVelocity")
#
#       posRef.setState(unitTestSupport.np2EigenVector3d(rN))  # m - r_BN_N
#       velRef.setState(unitTestSupport.np2EigenVector3d(vN))  # m - v_BN_N
#~~~~~~~~~~~~~~~~~
# the natural question arises, how does Basilisk know relative to what frame these states are defined?  This is
# actually setup above where we set `.isCentralBody = True` and mark the Earth as are central body.
# Without this statement, the code would assume the spacecraftPlus() states are relative to the default zeroBase frame.
# In the earlier basic orbital motion script (@ref scenarioBasicOrbit) this subtleties were not discussed.  This is because there
# the planets ephemeris message is being set to the default messages which zero's both the position and orientation
# states.  However, if Spice is used to setup the bodies, the zeroBase state must be carefully considered.
#
# After the simulation has run, and Spice is used to load in some libraries, they should be unloaded again using
# the following code:
#~~~~~~~~~~~~~~~~~{.py}
#       simIncludeGravity.unloadDefaultSpiceLibraries()
#       pyswice.unload_c(simIncludeGravity.spiceObject.SPICEDataPath + scEphemerisName)
#~~~~~~~~~~~~~~~~~
# The default libraries loaded with `simIncludeGravity.addSpiceInterface()` unloaded with
# `simIncludeGravity.unloadDefaultSpiceLibraries()`.  Any custom libraries, such as those of particular
# spacercraft, must be unloaded individually as shown.
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
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
#     run( False,       # do unit tests
#          True,        # show_plots
#          'NewHorizon'
#        )
# ~~~~~~~~~~~~~
# This case illustrates a simulation of the New Horizons spacecraft.  Here the craft is already a very
# large distance from the sun.  The
# resulting position coordinates and trajectorie differences are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioOrbitMultiBody1NewHorizon.svg "Position history")
# ![Trajectory Difference](Images/Scenarios/scenarioOrbitMultiBody3NewHorizon.svg "Trajectory Difference")
#
## @}
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

    # clear prior gravitational body and SPICE setup definitions
    simIncludeGravity.clearSetup()

    # setup Gravity Bodies
    simIncludeGravity.addEarth()
    simIncludeGravity.gravBodyList[-1].isCentralBody = True          # ensure this is the central gravitational body
    simIncludeGravity.gravBodyList[-1].useSphericalHarmParams = True
    gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/LocalGravData/GGM03S.txt'
                                     , simIncludeGravity.gravBodyList[-1].spherHarm
                                     ,100
                                     )
    muEarth = simIncludeGravity.gravBodyList[-1].mu

    simIncludeGravity.addSun()
    simIncludeGravity.addMoon()
    simIncludeGravity.addMars()
    simIncludeGravity.addJupiter()

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)

    # setup simulation start date/time
    timeInitString = "2012 MAY 1 00:28:30.0"
    spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
    timeInit = datetime.strptime(timeInitString,spiceTimeStringFormat)

    # setup SPICE interface
    simIncludeGravity.addSpiceInterface(splitPath[0], timeInitString)

    # by default the SPICE object will use the solar system barycenter as the inertial origin
    # If the spacecraftPlus() output is desired relative to another celestial object, the zeroBase string
    # name of the SPICE object needs to be changed.
    simIncludeGravity.spiceObject.zeroBase = 'Earth'

    # load in spacecraft SPICE ephemeris data
    if scCase is 'NewHorizons':
        scEphemerisName = 'nh_pred_od077.bsp'
        scSpiceName = 'NEW HORIZONS'
    else:   # default case
        scEphemerisName = 'hst_edited.bsp'
        scSpiceName = 'HUBBLE SPACE TELESCOPE'
        mu = muEarth
    pyswice.furnsh_c(simIncludeGravity.spiceObject.SPICEDataPath + scEphemerisName)      # Hubble Space Telescope data

    # add spice interface object to task list
    scSim.AddModelToTask(simTaskName, simIncludeGravity.spiceObject)

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
    if scCase is 'NewHorizons':
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
            fileNameString+"1"+scCase
            , plt, path)

    rBSK = posData[-1, 1:4]  # store the last position to compare to the SPICE position
    if scCase is 'Hubble':
        #
        # draw orbit in perifocal frame
        #
        oeData = orbitalMotion.rv2elem(mu,rN,vN)
        omega0 = oeData.omega
        b = oeData.a*np.sqrt(1-oeData.e*oeData.e)
        p = oeData.a*(1-oeData.e*oeData.e)
        plt.figure(2,figsize=np.array((1.0, b/oeData.a))*4.75,dpi=100)
        plt.axis(np.array([-oeData.rApoap, oeData.rPeriap, -b, b])/1000*1.25)

        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()
        planetColor= '#008800'
        planetRadius = simIncludeGravity.gravBodyList[0].radEquator/1000
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
                fileNameString+"2"+scCase
                , plt, path)
    else:
        scState = 1000.0*pyswice_ck_utilities.spkRead(scSpiceName, simIncludeGravity.spiceObject.getCurrentTimeString(), 'J2000', 'EARTH')
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
            fileNameString + "3" + scCase
            , plt, path)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    #
    #   unload the SPICE libraries that were loaded earlier
    #
    simIncludeGravity.unloadDefaultSpiceLibraries()
    pyswice.unload_c(simIncludeGravity.spiceObject.SPICEDataPath + scEphemerisName)


    #
    #   the python code below is for the unit testing mode.  If you are studying the scenario
    #   to learn how to run BSK, you can stop reading below this line.
    #

    if doUnitTests:

        # compare the results to the truth values
        accuracy = 300.0 # meters
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
         'Hubble'
       )

