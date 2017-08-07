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
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
# if this script is run from a custom folder outside of the Basilisk folder, then uncomment the
# following line and specify the absolute bath to the Basilisk folder
#bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion

# import simulation related support
import spacecraftPlus
import simIncludeGravBody



# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitCase, useSphericalHarmonics, planetCase", [
      ('LEO', False,'Earth')
    , ('GTO', False,'Earth')
    , ('GEO', False,'Earth')
    , ('LEO', True, 'Earth')
    , ('LEO', False,'Mars')
])
# provide a unique test method name, starting with test_
def test_scenarioBasicOrbit(show_plots, orbitCase, useSphericalHarmonics, planetCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, orbitCase, useSphericalHarmonics, planetCase)
    assert testResults < 1, testMessage



## \defgroup Tutorials_1_0
##   @{
## Demonstration of setup basic 3-DOF orbit simulation setup.
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
# 1     | LEO                 | False                 | Earth
# 2     | GTO                 | False                 | Earth
# 3     | GEO                 | False                 | Earth
# 4     | LEO                 | True                  | Earth
# 5     | LEO                 | False                 | Mars
#
# To run the default scenario 1 from the Basilisk/SimScenarios folder, call the python script through
#
#       python test_scenarioBasicOrbit.py
#
# *However*, to play with any scenario scripts as tutorials, you should make a copy of this
# `test_scenarioXXX.py` file into a custom folder outside of the Basilisk directory.  Next,
# one line must be edited in the scenario script to provide the absolute path to the root Basilisk
# directory.  For example, in `test_scenarioBasicOrbit.py` the line
#~~~~~~~~~~~~~~{.py}
# bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
#~~~~~~~~~~~~~~
# must be uncommented and edited for the particular user's Basilisk directory path.
#
# Qt Visualization Option
# -----
# If you wish to transmit the simulation data to the Qt Visualization, then uncomment the following
# line (line 360 in the script) from the python scenario script.  If the Viz is running, and searching for a connection on
# 127.0.0.1 (using Open Connection command from the File menu), the simulation is visualized in
# realtime
#~~~~~~~~~~~~~~{.py}
# unitTestSupport.enableVisualization(scSim, dynProcess)     (line 360 in the script)
#~~~~~~~~~~~~~~
# Note that by default the Viz is running in realtime mode with a 1x speed up factor.  This Viz
# speed up factor can be increased in the Qt GUI by calling up the
#
#       View/Bottom Playback Controls
#
# The speed up factor is adusting in 2x steps up or down using the green arrows in this GUI.
# This simulation is only updated at the rate of the simulation.  Thus, for this orbit simulation with 10s
# time steps the Viz will be choppy at 1x.  Speeding up the Viz playback with the green arrows quickly illustrates
# the orbital motion.
#
#
# Simulation Scenario Setup Details
# -----
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains the spacecraft object.  Gravity effectors are attached to the spacecraft dynamics to
# simulate the gravitational accelerations.
# ![Simulation Flow Diagram](Images/doc/test_scenarioBasicOrbit.svg "Illustration")
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
# The first step to adding gravity objects is to create the gravity body factor class.  Note that
# this call will create an empty gravitational body list each time this script is called.  Thus, there
# is not need to clear any prior list of gravitational bodies.
#~~~~~~~~~~~~~~~~~{.py}
#   gravFactory = simIncludeGravBody.gravBodyFactory()
#~~~~~~~~~~~~~~~~~
# To attach an Earth gravity model to this spacecraft, the following macro is invoked:
#~~~~~~~~~~~~~~~~~{.py}
#     planet = gravFactory.createEarth()
#     planet.isCentralBody = True          # ensure this is the central gravitational body
#~~~~~~~~~~~~~~~~~
# The gravFactor() class stores the Earth gravitational object within the class, but it also returns a
# handler to this gravitational object as a convenience.
# If extra customization is required, see the createEarth() macro to change additional values.
# For example, the spherical harmonics are turned off by default.  To engage them, the following code
# is used
#~~~~~~~~~~~~~~~~~{.py}
#     planet.useSphericalHarmParams = True
#     simIncludeGravBody.loadGravFromFile(bskPath+'External/LocalGravData/GGM03S-J2-only.txt'
#                                         , planet.spherHarm
#                                         ,2
#                                         )
#~~~~~~~~~~~~~~~~~
# The value 2 indidates that the first two harmonics, excluding the 0th order harmonic,
# are included.  This harmonics data file only includes a zeroth order and J2 term.
#
# Finally, the gravitational body must be connected to the spacecraft object.  This is done with
#~~~~~~~~~~~~~~~~~{.py}
#     scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())
#~~~~~~~~~~~~~~~~~
# Here the complete list of gravitational bodies is automatically assigned to the spacecraft, regardless if
# it is only one body like Earth or Mars, or a list of multiple bodies.
#
# To set the spacecraft initial conditions, the following initial position and velocity variables are set:
#~~~~~~~~~~~~~~~~~{.py}
#    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_CN_N
#    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
#~~~~~~~~~~~~~~~~~
# These vectors specify the inertial position and velocity vectors relative to the planet of the
# spacecraft center of mass location.  Note that there are 2 points that can be tracked.  The user always
# specifies the spacecraft center of mass location with the above code.  If the simulation output should be
# about another body fixed point B, this can be done as well.  This is useful in particular with more challenging
# dynamics where the center of mass moves relative to the body.  The following vector would specify the location of
# the spacecraft hub center of mass (Bc) relative to this body fixed point.
# ~~~~~~~~~~~~~~~~{.py}
#    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
# ~~~~~~~~~~~~~~~~
# If this vector is not specified, as in this tutorial scenario, then it defaults to zero.  If only a rigid hub
# is modeled, the Bc (hub center of mass) is the same as C (spacecraft center of mass).  If the spacecrat contains
# state effectors such as hinged panels, fuel slosh, imbalanced reaction wheels, etc., then the points Bc and C would
# not be the same.  Thus, in this simple simulation the body fixed point B and spacecraft center of mass are
# identical.
#
# Finally, the planet ephemeris data must be written to a message.  In this simulation the planet is held at
# a fixed location with zero position and velocity coordinates, so this message is not updated.
# If the planets move with time, such as with the SPICE
# functions, then this message can be written dynamically as well.
#~~~~~~~~~~~~~~~~~{.py}
#     simIncludeGravBody.generateCentralBodyEphemerisMsg(scSim.TotalSim, simProcessName, planet)
#~~~~~~~~~~~~~~~~~
# Note that this latter default planet ephemeris call should only be used if a single gravitational body
# is being simulated.  It both writes the required planet simulation ephemeris message, as well as a
# Visualization message specifying about which celestial object the motion should be shown.
#  If multiple bodies are simulated, then their positions would need to be
# dynamically updated.  See [test_scenarioOrbitMultiBody.py](@ref scenarioOrbitMultiBody) to learn how this is
# done via a SPICE object.
#
# Before the simulation is ready to run, it must be initialized.  The following code uses a convenient macro routine
# which initializes each BSK module (run self init, cross init and reset) and clears the BSK logging stack.
#~~~~~~~~~~~~~~~~~{.py}
#     scSim.InitializeSimulationAndDiscover()
#~~~~~~~~~~~~~~~~~
# If there are messages that are shared across multiple BSK threads, as shown in
# [test_scenarioAttitudeFeedback2T.py](@ref scenarioAttitudeFeedback2T), then this routine also
# auto-discovers these shared messages.
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          'LEO',       # orbit Case
#          False,       # useSphericalHarmonics
#          'Earth'      # planet Case
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last 2 arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The default
# scenario places the spacecraft about the Earth in a LEO orbit and without considering
# gravitational spherical harmonics.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1LEO0Earth.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2LEO0Earth.svg "Orbit Illustration")
#
# Setup 2
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          'GTO',       # orbit Case
#          False,       # useSphericalHarmonics
#          'Earth'      # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates an elliptical Geosynchronous Transfer Orbit (GTO) with zero orbit
# inclination.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1GTO0Earth.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2GTO0Earth.svg "Orbit Illustration")
#
# Setup 3
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          'GEO',       # orbit Case
#          False,       # useSphericalHarmonics
#          'Earth'      # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular Geosynchronous Orbit (GEO) with zero orbit
# inclination.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1GEO0Earth.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2GEO0Earth.svg "Orbit Illustration")
#
#  Setup 4
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          'LEO,        # orbit Case
#          True,        # useSphericalHarmonics
#          'Earth'      # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular LEO with a non-zero orbit
# inclination.  In this case the Earth's spherical harmonics are turned on.  The
# resulting position coordinates and semi-major axis time histories are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1LEO1Earth.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2LEO1Earth.svg "Orbit Illustration")
#
# Setup 5
# -------
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          'LEO',       # orbit Case
#          True,        # useSphericalHarmonics
#          'Mars'       # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular Low Mars Orbit or LMO with a non-zero orbit
# inclination.  In this case the Earth's spherical harmonics are turned on.  The
# resulting position coordinates and semi-major axis time histories are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1LEO0Mars.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2LEO0Mars.svg "Orbit Illustration")
#
##   @}
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

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    # unitTestSupport.enableVisualization(scSim, dynProcess)


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
    gravFactory = simIncludeGravBody.gravBodyFactory()
    if planetCase is 'Mars':
        planet = gravFactory.createMarsBarycenter()
        planet.isCentralBody = True           # ensure this is the central gravitational body
        if useSphericalHarmonics:
            planet.useSphericalHarmParams = True
            simIncludeGravBody.loadGravFromFile(bskPath + 'External/LocalGravData/GGM2BData.txt'
                                                , planet.spherHarm
                                                , 100
                                                )
    else:                   # Earth
        planet = gravFactory.createEarth()
        planet.isCentralBody = True          # ensure this is the central gravitational body
        if useSphericalHarmonics:
            planet.useSphericalHarmParams = True
            simIncludeGravBody.loadGravFromFile(bskPath+'External/LocalGravData/GGM03S-J2-only.txt'
                                             , planet.spherHarm
                                             ,2
                                             )
    mu = planet.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.*1000      # meters
    rGEO = 42000.*1000     # meters
    if orbitCase is 'GEO':
        oe.a     = rGEO
        oe.e     = 0.00001
        oe.i     = 0.0*macros.D2R
    elif orbitCase is 'GTO':
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
    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N


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
    # simIncludeGravBody.generateCentralBodyEphemerisMsg(scSim.TotalSim, simProcessName, planet)

    #
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
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
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_BN_N',range(3))

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]

    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
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
            fileNameString+"1"+orbitCase+str(int(useSphericalHarmonics))
            +planetCase
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
        if planetCase == 'Mars':
            planetColor = '#884400'
        else:
            planetColor= '#008800'
        planetRadius = planet.radEquator/1000
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
                fileNameString+"2"+orbitCase+str(int(useSphericalHarmonics))
                +planetCase
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
                fileNameString+"2"+orbitCase+str(int(useSphericalHarmonics))
                +planetCase
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

        # setup truth data for unit test
        if orbitCase is 'LEO' and useSphericalHarmonics == False and planetCase is 'Earth':
            truePos = [
                  [-2.8168016010234966e6,5.248174846916143e6,3.6771572646772987e6]
                , [-6.3710310400031125e6,-1.6053384413404597e6,2.4169406797143915e6]
                , [-1.970125344005881e6,-6.454584898598424e6,-1.8612676901068345e6]
                , [ 4.890526131271289e6,-3.2440700705588777e6,-3.815174368497354e6]
            ]
        if orbitCase is 'GTO' and useSphericalHarmonics == False and planetCase is 'Earth':
            truePos = [
                  [-5.889529848066479e6,9.686574890007671e6,0.]
                , [-3.2026565710377645e7,-4.305001879844011e6,0.]
                , [-3.624269187139845e7,-1.8990291195663467e7,0.]
                , [-2.9802077401931673e7,-2.831957848900475e7,0.]
                , [-1.4932981196798025e7,-2.939523308237971e7,0.]
            ]
        if orbitCase is 'GEO' and useSphericalHarmonics == False and planetCase is 'Earth':
            truePos = [
                  [-2.1819784817951165e7,3.588724145651873e7,0.]
                , [-4.16996933506621e7,-5.016611324503355e6,0.]
                , [-1.2686252555573342e7,-4.0038573722578734e7,0.]
                , [ 3.1201815137542922e7,-2.8114754297243357e7,0.]
                , [ 3.850428014786283e7,1.677456292503084e7,0.]
            ]
        if orbitCase is 'LEO' and useSphericalHarmonics == True and planetCase is 'Earth':
            truePos = [
                  [-2.8168016010234915e6,5.248174846916147e6,3.677157264677297e6]
                , [ 5.787240887314784e6,3.7547029876434486e6,-1.1653623184693705e6]
                , [ 2.5908823579481775e6,-5.38042751586389e6,-3.6401355110844015e6]
                , [-5.905673984221732e6,-3.5332208726054016e6,1.2748483822117285e6]
                , [-2.3741237403798397e6,5.508156976353034e6,3.6085612280591857e6]
            ]
        if orbitCase is 'LEO' and useSphericalHarmonics == False and planetCase is 'Mars':
            truePos = [
                  [-2.8168016010234966e6,5.248174846916143e6,3.6771572646772987e6]
                , [-6.370345938284969e6,-1.6147054668864955e6,2.412504030081398e6]
                , [-1.9520854768447054e6,-6.457181115789631e6,-1.8712382659451987e6]
                , [ 4.90876381054031e6,-3.2188851633259663e6,-3.8130784005532693e6]
            ]

        # compare the results to the truth values
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
         'LEO',       # orbit Case (LEO, GTO, GEO)
         False,       # useSphericalHarmonics
         'Earth'      # planetCase (Earth, Mars)
       )

