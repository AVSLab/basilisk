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
#           a 3-DOV spacecraft on a range of orbit types.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 26, 2016
#

import pytest
import sys, os, inspect
import matplotlib
import numpy as np
import ctypes
import math
import csv
import logging

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion

# import simulation related support
import spacecraftPlus
import gravityEffector
import simIncludeGravity

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitCase, useSphericalHarmonics, planetCase", [
      (0, False,0)
    , (1, False,0)
    , (2, False,0)
    , (0, True, 0)
    , (0, False,1)
])
# provide a unique test method name, starting with test_
def test_scenarioBasicOrbit(show_plots, orbitCase, useSphericalHarmonics, planetCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, orbitCase, useSphericalHarmonics, planetCase)
    assert testResults < 1, testMessage

## This scenario demonstrates how to setup basic 3-DOF orbits.
#
# Basic Orbit Setup and Translational Motion Simulation {#scenarioBasicOrbit}
# ====
#
# Scenario Description
# -----
# This script sets up a 3-DOF spacecraft which is orbiting a planet.  The purpose
# is to illustrate how to create a spacecraft, attach a gravity model, and run
# a basic Basilisk simulation.  The scenarios can be run with the followings setups
# parameters:
# Setup | orbitCase           | useSphericalHarmonics | planetCase
# ----- | ------------------- | --------------------- | -----------
# 1     | 0 (LEO)             | False                 | 0 (Earth)
# 2     | 1 (GTO)             | False                 | 0 (Earth)
# 3     | 2 (GEO)             | False                 | 0 (Earth)
# 4     | 0 (LEO)             | True                  | 0 (Earth)
# 5     | 0 (LMO)             | False                 | 1 (Mars)
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioBasicOrbit.py
#
# When the simulation completes 2 plots are shown for each case.  One plot always shows
# the inertial position vector components, while the second plot either shows a planar
# orbit view relative to the perfocal frame (no spherical harmonics), or the
# semi-major axis time history plot (with spherical harmonics turned on).
#
# The dynamics simulation is setup using a SpacecraftPlus() module.  Note that the rotational motion simulation is turned off to leave
# pure 3-DOF translation motion simulation.
#~~~~~~~~~~~~~~~~~{.py}
#     scObject = spacecraftPlus.SpacecraftPlus()
#     scObject.ModelTag = "spacecraftBody"
#     scObject.hub.useTranslation = True
#     scObject.hub.useRotation = False
#~~~~~~~~~~~~~~~~~
# Next, this module is attached to the simulation process
#~~~~~~~~~~~~~~~~~{.py}
#   scSim.AddModelToTask(simTaskName, scObject)
#~~~~~~~~~~~~~~~~~
# To attach an Earth gravity model to this spacecraft, the following macro is invoked:
#~~~~~~~~~~~~~~~~~{.py}
#     gravBody, ephemData = simIncludeGravity.addEarth()
#     gravBody.isCentralBody = True          # ensure this is the central gravitational body
#~~~~~~~~~~~~~~~~~
# If extra customization is required, see teh addEarth() macro to change additional values.
# For example, the spherical harmonics are turned off by default.  To engage them, the following code
# is used
#~~~~~~~~~~~~~~~~~{.py}
#     gravBody.useSphericalHarmParams = True
#     gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/SphericalHarmonics/Earth_GGM03S.txt'
#                                      , gravBody.spherHarm
#                                      ,3
#                                      )
#~~~~~~~~~~~~~~~~~
# The value 3 indidates that the first three harmonics, including the 0th order harmonic,
# is included.
#
# Finally, the planet ephemerise data must be written to a message.  In this simulation the planet is held at
# a fixed location, so this message is not updated.  If the planets move with time, such as with the SPICE
# functions, then this message can be writen dynamically as well.
#~~~~~~~~~~~~~~~~~{.py}
#     messageSize = ephemData.getStructSize()
#     scSim.TotalSim.CreateNewMessage(simProcessName,
#                                           gravBody.bodyMsgName, messageSize, 2)
#     scSim.TotalSim.WriteMessageData(gravBody.bodyMsgName, messageSize, 0,
#                                     ephemData)
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
#          0,           # orbit Case
#          False,       # useSphericalHarmonics
#          0            # planet Case
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last 2 arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The default
# scenario places the spacecraft about the Earth in a LEO orbit and without considering
# gravitational spherical harmonics.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1000.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2000.svg "Orbit Illustration")
#
# Setup 2
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          1,           # orbit Case
#          False,       # useSphericalHarmonics
#          0            # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates an elliptical Geosynchronous Transfer Orbit (GTO) with zero orbit
# inclination.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1100.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2100.svg "Orbit Illustration")
#
# Setup 3
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          2,           # orbit Case
#          False,       # useSphericalHarmonics
#          0            # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular Geosynchronous Orbit (GEO) with zero orbit
# inclination.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1200.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2200.svg "Orbit Illustration")
#
#  Setup 4
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          0,           # orbit Case
#          True,        # useSphericalHarmonics
#          0            # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular LEO with a non-zero orbit
# inclination.  In this case the Earth's spherical harmonics are turned on.  The
# resulting position coordinates and semi-major axis time histories are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1010.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2010.svg "Orbit Illustration")
#
# Setup 5
# -------
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          0,           # orbit Case
#          True,        # useSphericalHarmonics
#          1            # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular Low Mars Orbit or LMO with a non-zero orbit
# inclination.  In this case the Earth's spherical harmonics are turned on.  The
# resulting position coordinates and semi-major axis time histories are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1001.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2001.svg "Orbit Illustration")
#


def run(doUnitTests, show_plots, orbitCase, useSphericalHarmonics, planetCase):
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
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = False

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    if planetCase == 1:     # Mars
        gravBody, ephemData = simIncludeGravity.addMars()
        gravBody.isCentralBody = True          # ensure this is the central gravitational body
        if useSphericalHarmonics:
            gravBody.useSphericalHarmParams = True
            gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/LocalGravData/GGM2BData.txt'
                                             , gravBody.spherHarm
                                             , 3
                                             )
        mu = gravBody.mu
    else:                   # Earth
        gravBody, ephemData = simIncludeGravity.addEarth()
        gravBody.isCentralBody = True          # ensure this is the central gravitational body
        if useSphericalHarmonics:
            gravBody.useSphericalHarmParams = True
            gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/LocalGravData/GGM03S.txt'
                                             , gravBody.spherHarm
                                             ,3
                                             )
        mu = gravBody.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([gravBody])

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.*1000      # meters
    rGEO = 42000.*1000     # meters
    if orbitCase == 2:      # GEO case
        oe.a     = rGEO
        oe.e     = 0.00001
        oe.i     = 0.0*macros.D2R
    elif orbitCase == 1:    # GTO case
        oe.a = (rLEO+rGEO)/2.0
        oe.e = 1.0 - rLEO/oe.a
        oe.i = 0.0*macros.D2R
    else:                   # LEO case, default case 0
        oe.a     = rLEO
        oe.e     = 0.0001
        oe.i     = 33.3*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 347.8*macros.D2R
    oe.f     = 85.3*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
                                                # with circular or equatorial orbit, some angles are
                                                # arbitrary


    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n
    if useSphericalHarmonics:
        simulationTime = macros.sec2nano(3.*P)
    else:
        simulationTime = macros.sec2nano(0.75*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    if useSphericalHarmonics:
        numDataPoints = 400
    else:
        numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    #
    # create simulation messages
    #

    # create the gravity ephemerise message
    messageSize = ephemData.getStructSize()
    scSim.TotalSim.CreateNewMessage(simProcessName,
                                          gravBody.bodyInMsgName, messageSize, 2)
    scSim.TotalSim.WriteMessageData(gravBody.bodyInMsgName, messageSize, 0,
                                    ephemData)

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
    for idx in range(1,4):
        plt.plot(posData[:, 0]*macros.NANO2SEC/P, posData[:, idx]/1000.,
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$r_{BN,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"1"+str(int(orbitCase))+str(int(useSphericalHarmonics))
            +str(int(planetCase))
            , plt, path)

    if useSphericalHarmonics == False:
        # draw orbit in perifocal frame
        b = oe.a*np.sqrt(1-oe.e*oe.e)
        p = oe.a*(1-oe.e*oe.e)
        plt.figure(2,figsize=np.array((1.0, b/oe.a))*4.75,dpi=100)
        plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b])/1000*1.25)
        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()
        if planetCase == 1:
            planetColor = '#884400'
        else:
            planetColor= '#008800'
        planetRadius = gravBody.radEquator/1000
        ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
        # draw the actual orbit
        rData=[]
        fData=[]
        for idx in range(0,len(posData)):
            oeData = orbitalMotion.rv2elem(mu,posData[idx,1:4],velData[idx,1:4])
            rData.append(oeData.rmag)
            fData.append(oeData.f + oeData.omega - oe.omega)
        plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
                 ,color='#aa0000'
                 ,linewidth = 3.0
                 )
        # draw the full osculating orbit from the initial conditions
        fData = np.linspace(0,2*np.pi,100)
        rData = []
        for idx in range(0,len(fData)):
            rData.append(p/(1+oe.e*np.cos(fData[idx])))
        plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
                 ,'--'
                 , color='#555555'
                 )
        plt.xlabel('$i_e$ Cord. [km]')
        plt.ylabel('$i_p$ Cord. [km]')
        plt.grid()
        if doUnitTests:     # only save off the figure if doing a unit test run
            unitTestSupport.saveScenarioFigure(
                fileNameString+"2"+str(int(orbitCase))+str(int(useSphericalHarmonics))
                +str(int(planetCase))
                , plt, path)
    else:
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        smaData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
            smaData.append(oeData.a/1000.)
        plt.plot(posData[:, 0]*macros.NANO2SEC/P, smaData
                 ,color='#aa0000',
                 )
        plt.xlabel('Time [orbits]')
        plt.ylabel('SMA [km]')
        if doUnitTests:     # only save off the figure if doing a unit test run
            unitTestSupport.saveScenarioFigure(
                fileNameString+"2"+str(int(orbitCase))+str(int(useSphericalHarmonics))
                +str(int(planetCase))
                , plt, path)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    #
    #   the python code below is for the unit testing mode.  If you are studying the scenario
    #   to learn how to run BSK, you can stop reading below this line.
    #
    if doUnitTests:
        numTruthPoints = 5
        skipValue = int(len(posData)/(numTruthPoints-1))
        dataPosRed = posData[::skipValue]
        print dataPosRed

        # setup truth data for unit test
        if orbitCase == 0 and useSphericalHarmonics == False and planetCase == 0:
            truePos = [
                  [-2.8168016010206547e6,5.248174846918056e6,3.677157264676744e6]
                , [-6.371031043741273e6,-1.6053383896922336e6,2.4169407041580593e6]
                , [-1.9701254434068222e6,-6.454584884129508e6,-1.8612676350966396e6]
                , [ 4.890526030465493e6,-3.244070209218091e6,-3.815174379843309e6]
            ]
        if orbitCase == 1 and useSphericalHarmonics == False and planetCase == 0:
            truePos = [
                  [-5.889529848066479e6,9.686574890007671e6,0.]
                , [-3.202656563105511e7,-4.305001765487548e6,0.]
                , [-3.624269189590486e7,-1.8990291025241878e7,0.]
                , [-2.9802077625266302e7,-2.8319578366928123e7,0.]
                , [-1.493298171897125e7,-2.939523322070207e7,0.]
            ]
        if orbitCase == 2 and useSphericalHarmonics == False and planetCase == 0:
            truePos = [
                  [-2.1819784817951165e7,3.588724145651873e7,0.]
                , [-4.169969339026251e7,-5.016610995318371e6,0.]
                , [-1.2686253187718624e7,-4.003857352228714e7,0.]
                , [ 3.1201814471707206e7,-2.8114755036203798e7,0.]
                , [ 3.850428067757058e7,1.6774561709148446e7,0.]
            ]
        if orbitCase == 0 and useSphericalHarmonics == True and planetCase == 0:
            truePos = [
                  [-2.8168016010234905e+06, 5.2481748469161475e+06, 3.6771572646772973e+06]
                , [ 5.7872847370997239e+06, 3.7546822535522678e+06,-1.1653523637507784e+06]
                , [ 2.5907840000510132e+06,-5.3804054569751248e+06,-3.6402589543839172e+06]
                , [-5.9057334395625489e+06,-3.5330935652573332e+06, 1.2748615301643584e+06]
                , [-2.3737945302869496e+06, 5.5081619273500638e+06, 3.6087705254912134e+06]
            ]
        if orbitCase == 0 and useSphericalHarmonics == False and planetCase == 1:
            truePos = [
                  [-2.8168016010234966e6,5.248174846916143e6,3.6771572646772987e6]
                , [-6.370345912426974e6,-1.614705816780056e6,2.412503864225198e6]
                , [-1.9520848025381358e6,-6.457181211724201e6,-1.871238638146856e6]
                , [ 4.908764490099604e6,-3.218884221201837e6,-3.8130783208338753e6]
            ]

        # compare the results to the truth values
        if orbitCase == 2:
            accuracy = 10.0
        else:
            accuracy = 1.0  # meters

        testFailCount, testMessages = unitTestSupport.compareArray(
            truePos, dataPosRed, accuracy, "r_BN_N Vector",
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
         0,           # orbit Case (0 - LEO, 1 - GTO, 2 - GEO)
         False,       # useSphericalHarmonics
         0            # planetCase (0 - Earth, 1 - Mars)
       )

