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
# Purpose:  Integrated test of the spacecraftPlus() and gravity modules illustrating
#           how impulsive Delta_v maneuver can be simulated with stoping and starting the
#           simulation.
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
# bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion

# import simulation related support
import spacecraftPlus
import simIncludeGravBody


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
# The following 'parametrize' function decorator provides the parameters for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("maneuverCase", [
    0
    , 1
])
# provide a unique test method name, starting with test_
def test_scenarioOrbitManeuver(show_plots, maneuverCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(True,
                                     show_plots, maneuverCase)
    assert testResults < 1, testMessage


## \defgroup Tutorials_1_2
##   @{
## Illustration how to start and stop the simulation to perform orbit maneuvers within Python.
#
# Orbit Maneuvers using Simulation Starting/Stopping in Python {#scenarioOrbitManeuver}
# ====
#
# Scenario Description
# -----
# This script sets up a 3-DOF spacecraft which is orbiting Earth.  The purpose
# is to illustrate how to start and stop the Basilisk simulation to apply
# some Delta_v's for simple orbit maneuvers.  Read
# [test_scenarioBasicOrbit.py](@ref scenarioBasicOrbit) to learn how to setup an
# orbit simulation. The scenarios can be run with the followings setups
# parameters:
# Setup | maneuverCase
# ----- | -------------------
# 1     | 0 (Hohmann)
# 2     | 1 (Inclination)
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioOrbitManeuver.py
#
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains the spacecraft object.  The BSK simulation is run for a fixed period.  After stopping, the
# states are changed and the simulation is resumed.
# ![Simulation Flow Diagram](Images/doc/test_scenarioOrbitManeuver.svg "Illustration")
#
# When the simulation completes 2 plots are shown for each case.  One plot always shows
# the inertial position vector components, while the second plot either shows a plot
# of the radius time history (Hohmann maneuver), or the
# inclination angle time history plot (Inclination change maneuver).
#
# The dynamics simulation is setup using a SpacecraftPlus() module with the Earth's
# gravity module attached.  Note that the rotational motion simulation is turned off to leave
# pure 3-DOF translation motion simulation.  After running the simulation for 1/4 of a period
# the simulation is stopped to apply impulsive changes to the inertial velocity vector.
# ~~~~~~~~~~~~~~~~~{.py}
#    scSim.ConfigureStopTime(simulationTime)
#    scSim.ExecuteSimulation()
# ~~~~~~~~~~~~~~~~~
# Next, the state manager objects are called to retrieve the latest inerital position and
# velocity vector components:
# ~~~~~~~~~~~~~~~~~{.py}
#    rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
#    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())
# ~~~~~~~~~~~~~~~~~
# After computing the maneuver specific Delta_v's, the state managers velocity is updated through
# ~~~~~~~~~~~~~~~~~{.py}
#     velRef.setState(unitTestSupport.np2EigenVector3d(vVt))
# ~~~~~~~~~~~~~~~~~
# To start up the simulation again, not that the total simulation time must be provided,
# not just the next incremental simulation time.
# ~~~~~~~~~~~~~~~~~{.py}
#     scSim.ConfigureStopTime(simulationTime+T2)
#     scSim.ExecuteSimulation()
# ~~~~~~~~~~~~~~~~~
# This process is then repeated for the second maneuver.
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
#          0            # Maneuver Case (0 - Hohmann, 1 - Inclination)
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The remaining argument controls the
# type of maneuver that is being simulated.  In this case a classical Hohmann transfer is being
# simulated to go from LEO to reach and stay at GEO. The math behind such maneuvers can be found
# in textbooks such as *Analytical Mechanics of Space Systems*
# (<http://arc.aiaa.org/doi/book/10.2514/4.102400>).
# The resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioOrbitManeuver10.svg "Position history")
# ![Orbit Radius Illustration](Images/Scenarios/scenarioOrbitManeuver20.svg "Radius Illustration")
#
# Setup 2
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          1            # Maneuver Case (0 - Hohmann, 1 - Inclination)
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The remaining argument controls the
# type of maneuver that is being simulated.  In this case a classical plane change is being
# simulated to go rotate the orbit plane first 8 degrees, then another 4 degrees after
# orbiting 90 degrees. The math behind such maneuvers can be found
# in textbooks such as *Analytical Mechanics of Space Systems*
# (<http://arc.aiaa.org/doi/book/10.2514/4.102400>).  The final orbit inclination angle is 8.94 degrees
# which is indicated as a dashed line below.
# The resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioOrbitManeuver11.svg "Position history")
# ![Inclination Angle Time History](Images/Scenarios/scenarioOrbitManeuver21.svg "Inclination Illustration")
#
##  @}
def run(doUnitTests, show_plots, maneuverCase):
    '''Call this routine directly to run the tutorial scenario.'''
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

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

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
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
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([earth])

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    rGEO = math.pow(earth.mu / math.pow((2. * np.pi) / (24. * 3600.), 2), 1. / 3.)
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(earth.mu, oe)
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N

    # set the simulation time
    n = np.sqrt(earth.mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.25 * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    #
    # create single message for earth gravBody
    #
    gravBodyFactory.generateCentralBodyEphemerisMsg(scSim.TotalSim, simProcessName, earth)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #  get access to dynManager translational states for future access to the states
    #
    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # get the current spacecraft states
    rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())
    # compute maneuver Delta_v's
    if maneuverCase == 1:
        # inclination change
        Delta_i = 8.0 * macros.D2R
        rHat = rVt / np.linalg.norm(rVt)
        hHat = np.cross(rVt, vVt)
        hHat = hHat / np.linalg.norm(hHat)
        vHat = np.cross(hHat, rHat)
        v0 = np.dot(vHat, vVt)
        vVt = vVt - (1. - np.cos(Delta_i)) * v0 * vHat + np.sin(Delta_i) * v0 * hHat
        velRef.setState(unitTestSupport.np2EigenVectorXd(vVt))
        T2 = macros.sec2nano(P * 0.25)
    else:
        # Hohmann Transfer to GEO
        v0 = np.linalg.norm(vVt)
        r0 = np.linalg.norm(rVt)
        at = (r0 + rGEO) * .5
        v0p = np.sqrt(earth.mu / at * rGEO / r0)
        n1 = np.sqrt(earth.mu / at / at / at)
        T2 = macros.sec2nano((np.pi) / n1)
        vHat = vVt / v0
        vVt = vVt + vHat * (v0p - v0)
        velRef.setState(unitTestSupport.np2EigenVectorXd(vVt))

    # run simulation for 2nd chunk
    scSim.ConfigureStopTime(simulationTime + T2)
    scSim.ExecuteSimulation()

    # get the current spacecraft states
    rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())
    # compute maneuver Delta_v's
    if maneuverCase == 1:
        # inclination change
        Delta_i = 4.0 * macros.D2R
        rHat = rVt / np.linalg.norm(rVt)
        hHat = np.cross(rVt, vVt)
        hHat = hHat / np.linalg.norm(hHat)
        vHat = np.cross(hHat, rHat)
        v0 = np.dot(vHat, vVt)
        vVt = vVt - (1. - np.cos(Delta_i)) * v0 * vHat + np.sin(Delta_i) * v0 * hHat
        velRef.setState(unitTestSupport.np2EigenVectorXd(vVt))
        T3 = macros.sec2nano(P * 0.25)
    else:
        # Hohmann Transfer to GEO
        v1 = np.linalg.norm(vVt)
        v1p = np.sqrt(earth.mu / rGEO)
        n1 = np.sqrt(earth.mu / rGEO / rGEO / rGEO)
        T3 = macros.sec2nano(0.25 * (np.pi) / n1)
        vHat = vVt / v1
        vVt = vVt + vHat * (v1p - v1)
        velRef.setState(unitTestSupport.np2EigenVectorXd(vVt))

    # run simulation for 3rd chunk
    scSim.ConfigureStopTime(simulationTime + T2 + T3)
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
    fileNameString = filename[len(path) + 6:-3]

    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(1, 4):
        plt.plot(posData[:, 0] * macros.NANO2HOUR, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Inertial Position [km]')
    if doUnitTests:  # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString + "1" + str(int(maneuverCase))
            , plt, path)

    if maneuverCase == 1:
        # show inclination angle
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        iData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(earth.mu, posData[idx, 1:4], velData[idx, 1:4])
            iData.append(oeData.i * macros.R2D)
        plt.plot(posData[:, 0] * macros.NANO2HOUR, np.ones(len(posData[:, 0])) * 8.93845
                 , '--'
                 , color='#444444'
                 )
        plt.plot(posData[:, 0] * macros.NANO2HOUR, iData
                 , color='#aa0000'
                 )
        plt.ylim([-1, 10])
        plt.xlabel('Time [h]')
        plt.ylabel('Inclination [deg]')
        if doUnitTests:  # only save off the figure if doing a unit test run
            unitTestSupport.saveScenarioFigure(
                fileNameString + "2" + str(int(maneuverCase))
                , plt, path)
    else:
        # show SMA
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        rData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem_parab(earth.mu, posData[idx, 1:4], velData[idx, 1:4])
            rData.append(oeData.rmag / 1000.)
        plt.plot(posData[:, 0] * macros.NANO2HOUR, rData
                 , color='#aa0000',
                 )
        plt.xlabel('Time [h]')
        plt.ylabel('Radius [km]')
        if doUnitTests:  # only save off the figure if doing a unit test run
            unitTestSupport.saveScenarioFigure(
                fileNameString + "2" + str(int(maneuverCase))
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
        dataPos = posRef.getState()
        dataPos = [[0.0, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]

        # setup truth data for unit test
        if maneuverCase == 0:
            truePos = [
                [10298352.587758573, 40947481.244493686, 0.0]
            ]
        if maneuverCase == 1:
            truePos = [
                [5937590.072546725, 3675220.9560903916, 477503.77340122446]
            ]

        # compare the results to the truth values
        accuracy = 1e-6
        testFailCount, testMessages = unitTestSupport.compareArray(
            truePos, dataPos, accuracy, "r_BN_N Vector",
            testFailCount, testMessages)

        #   print out success message if no error were found
        if testFailCount == 0:
            print "PASSED "
        else:
            print "testFailCount: " + str(testFailCount)
            print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(False,  # do unit tests
        True,  # show_plots
        0  # Maneuver Case (0 - Hohmann, 1 - Inclination)
        )
