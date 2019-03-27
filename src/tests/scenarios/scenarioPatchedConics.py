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
# Purpose: Patched Conics Interplanetary Trajectory
# Author:   Divinaa Burder
# Creation Date:
#

import os
import numpy as np

import matplotlib.pyplot as plt
from Basilisk import __path__
bskPath = __path__[0]
from Basilisk.simulation import spacecraftPlus, simMessages, gravityEffector
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion,simIncludeGravBody, unitTestSupport



## \defgroup Tutorials_1_5
## @{
## Demonstration of using gravFactory to create multiple celestial bodies and using .isCentralBody to set
## spacecraft initial states in an absolute or relative frame
#
# Patched Conics {#scenarioPatched Conics}
# ====
#
# Scenario Description
# -----
# This tutorial considers a patched conics approach to an interplanetary transfer from Earth to Jupiter, by way of a
# Hohmann transfer.
#
#
# This script sets up a basic spacecraft which starts in circular Low Earth Orbit, with logging with respect to the
# Earth.The spacecraft then leaves on a hyperbolic orbit with respect to Earth until it reaches the edge of the Earth's
# sphere of influence.
#
# The frame of reference is then switched to a Sun centered inertial, and the planetary positions are adjusted
# accordingly. The Earth's velocity is added to the spacecraft (to account for the heliocentric velocity of the
# spacecraft) and the simulation is run until the spacecraft approaches Jupiter's Sphere of Influence.
#
#
# Simulation Scenario Setup Details
# -----
# The basics of the spacecraft and simulation set up are shown in
# [scenarioBasicOrbit.py](@ref scenarioBasicOrbit).
# The basics of the Hohmann transfer is shown in [scenarioOrbitManeuver.py](@ref scenarioOrbitManeuver).
# Setting up multiple gravitational bodies is shown in [scenarioOrbitMultiBody.py](@ref scenarioOrbitMultiBody)
# Providing pseudo-SPICE messages is laid out in [scenarioCSS.py](@ref scenarioCSS).
#
# This simulation combines all those techniques as well as changing logging relative to multiple bodies for a single
# simulation.
#
# In order to specify which body the spacecraft position and velocities are integrated relative to, the isCentralBody
# flag is used.
#
#
# The planets and the Sun are artificially placed and adjusted throughout the tutorial using state messages to mimic
# SPICE states. An example of this is shown below.
#
#~~~~~~~~~~~~~{.py}
# sunStateMsg = simMessages.SpicePlanetStateSimMsg()
# sunStateMsg.PositionVector = [0.0, 0.0, 0.0]
# sunStateMsg.VelocityVector = [0.0, 0.0, 0.0]
# unitTestSupport.setMessage(scSim.TotalSim,
#                            simProcessName,
#                            sun.bodyInMsgName,
#                            sunStateMsg)
#~~~~~~~~~~~~~
#
# The relative logging of the spacecraft is changed by switching the central body. An example is shown below:
#
#~~~~~~~~~~~~~{.py}
# earth.isCentralBody = False
# sun.isCentralBody = True
#~~~~~~~~~~~~~
#
# In order to speed up computation, we need to update the simulation time step for an interplanetary transfer.
# This is done by the following:
#~~~~~~~~~~~~~{.py}
# simulationTimeStep = macros.sec2nano(1 * 7 * 24 * 60 * 60)
#~~~~~~~~~~~~~
#
# The following updates the logging to match the new simulationTimeStep
#~~~~~~~~~~~~~{.py}
# scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, simulationTimeStep)
# This updates the Task period in the simulation.
# dynProcess.updateTaskPeriod(simTaskName, simulationTimeStep)
#~~~~~~~~~~~~~
#
# Similar to scenarioOrbitManeuver.py, the states are retrieved, manipulated and fed back to the simulation by:
#
#~~~~~~~~~~~~~{.py}
# hubR_N = scObject.dynManager.getStateObject("hubPosition")
# hubV_N = scObject.dynManager.getStateObject("hubVelocity")
#~~~~~~~~~~~~~
#
#
#~~~~~~~~~~~~~{.py}
# rInit_BN_N = unitTestSupport.EigenVector3d2np(hubR_N.getState())
# vInit_BN_N = unitTestSupport.EigenVector3d2np(hubV_N.getState())
#~~~~~~~~~~~~~
#
#~~~~~~~~~~~~~{.py}
# hubR_N.setState(unitTestSupport.np2EigenVectorXd(r_Adj_N))
# hubV_N.setState(unitTestSupport.np2EigenVectorXd(v_Adj_N))
#~~~~~~~~~~~~~
#
# Note that the output position and velocity (when reading message logs) will be relative to the planet that is the
# central body during that logging period. So to use the last state in each segment, it needed to be adjusted to account
# for the planet/Sun's position and velocity.
#
# Plots found when running this scenario show the Earth centered departure, the heliocentric Hohmann transfer and the
# Jupiter centered arrival.
#
# ![Earth Centered Departure Perifocal Frame](Images/Scenarios/scenarioPatchedConics1.svg "Earth Centered Departure Perifocal Frame")
# ![Earth Centered Departure Radius](Images/Scenarios/scenarioPatchedConics2.svg "Earth Centered Departure Radius")
# ![Heliocentric Transfer Arc](Images/Scenarios/scenarioPatchedConics3.svg "Heliocentric Transfer Arc")
# ![Heliocentric Transfer Radius](Images/Scenarios/scenarioPatchedConics4.svg "Heliocentric Transfer Radius")
# ![Jupiter Centered Arrival](Images/Scenarios/scenarioPatchedConics5.svg "Jupiter Centered Arrival")
## @}

def run(show_plots):
    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()


    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(5.)

    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"



    gravFactory = simIncludeGravBody.gravBodyFactory()

    earth = gravFactory.createEarth()
    jupiter = gravFactory.createJupiter()
    sun = gravFactory.createSun()


    # by default the SPICE object will use the solar system barycenter as the inertial origin
    # If the spacecraftPlus() output is desired relative to another celestial object, the zeroBase string
    # name of the SPICE object needs to be changed.
    earth.isCentralBody = True
    scObject.gravField.setGravBodies(gravityEffector.GravBodyVector(gravFactory.gravBodies.values()))
    scSim.AddModelToTask(simTaskName, scObject)

    # create simulation messages
    sunStateMsg = simMessages.SpicePlanetStateSimMsg()
    sunStateMsg.PositionVector = [0.0, 149598023 * 1000, 0.0]
    sunStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               sun.bodyInMsgName,
                               sunStateMsg)

    earthStateMsg = simMessages.SpicePlanetStateSimMsg()
    earthStateMsg.PositionVector = [0.0, 0.0, 0.0]
    earthStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               earth.bodyInMsgName,
                               earthStateMsg)

    jupiterStateMsg = simMessages.SpicePlanetStateSimMsg()
    jupiterStateMsg.PositionVector = [0.0, (149598023 + 778298361) * 1000, 0.0]
    jupiterStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               jupiter.bodyInMsgName,
                               jupiterStateMsg)


    #  Earth Centered Circular orbit and hyperbolic departure
    # initialize spacecraftPlus object and set properties
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rEarth = 149598023. * 1000
    rJupiter = 778298361. * 1000
    oe.a = rLEO
    oe.e = 0.0
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(earth.mu, oe)
    oe = orbitalMotion.rv2elem(earth.mu, rN, vN)      # this stores consistent initial orbit elements

    #Hohmann transfer calculations


    at = (rEarth + rJupiter) * .5
    vPt = np.sqrt(2 * sun.mu / rEarth - sun.mu / at)
    vAt = np.sqrt(2 * sun.mu / rJupiter - sun.mu / at)
    n1 = np.sqrt(sun.mu / at / at / at)
    T2 = macros.sec2nano((np.pi) / n1)

    n = np.sqrt(earth.mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n

    v_Earth = 29.7859 * 1000 # speed of the earth
    v_Earth_rel = vPt - v_Earth # Required earth relative velocity

    aHyp = - earth.mu / (v_Earth_rel * v_Earth_rel)  # Semimajor axis of departure hyperbola
    eHyp = rLEO * v_Earth_rel * v_Earth_rel / earth.mu + 1 # Eccentricity of hyperbolic departure orbit
    v0 = np.sqrt( v_Earth_rel * v_Earth_rel + 2 * earth.mu / rLEO) # Earth relative speed s/c needs post burn
    v_c = np.sqrt( earth.mu / rLEO)
    deltaV1 = v0 - v_c
    phi = np.arccos(1 / eHyp) + np.pi # Burn angle
    E = phi #np.arctan(np.tan(phi / 2)) + 2 * np.pi
    t0_Tp = (1 / n) * E


    # initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N

    # set the simulation time

    simulationTime = macros.sec2nano(t0_Tp)
    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)


    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    scSim.InitializeSimulationAndDiscover()


    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()


    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")

    vVt = unitTestSupport.EigenVector3d2np(velRef.getState())


    vHat = vVt / np.linalg.norm(vVt)
    print 'vHat ='+str(vHat)
    vVt = vVt + vHat * deltaV1
    print 'vVt ='+str(vVt)


    velRef.setState(unitTestSupport.np2EigenVectorXd(vVt))


    # run simulation for 2nd chunk
    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(110000))
    scSim.ExecuteSimulation()


    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))

    print 'Final position in ECI frame = '+str(posData[-1])
    print 'Final velocity in ECI frame = '+str(velData[-1])

    endEarthTime = len(posData)

    np.set_printoptions(precision=16)


    # plot the results

    fileName = os.path.basename(os.path.splitext(__file__)[0])

    plt.close("all")  # clears out plots from earlier test runs
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(1, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    planetRadius = earth.radEquator / 1000
    ax.add_artist(plt.Circle((0,0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(earth.mu, posData[idx, 1:4], velData[idx, 1:4])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)


    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0, label='Simulated Flight')
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0, 2 * np.pi, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555', label='Orbit Track')
    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')
    plt.legend(loc='lower left')
    plt.grid()
    pltName = "Earth Circular Orbit"
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    rData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem_parab(earth.mu, posData[idx, 1:4], velData[idx, 1:4])
        rData.append(oeData.rmag / 1000.)
    plt.plot(posData[:, 0] * macros.NANO2HOUR, rData, color='#aa0000',
             )
    plt.xlabel('Time [h]')
    plt.ylabel('Radius [km]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    ##############################################################

    print('*****************************************')
    print('SWITCHING TO SUN CENTERED FRAME')
    print('*****************************************')
    ## SWITCHING CENTRAL BODY TO THE SUN
    #
    earth.isCentralBody = False
    sun.isCentralBody = True



    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1 * 7 * 24 * 60 * 60) # Changing timestep to 1 week

    # create simulation messages
    sunStateMsg = simMessages.SpicePlanetStateSimMsg()
    sunStateMsg.PositionVector = [0.0, 0.0, 0.0]
    sunStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               sun.bodyInMsgName,
                               sunStateMsg)

    earthStateMsg = simMessages.SpicePlanetStateSimMsg()
    earthStateMsg.PositionVector = [0.0, -149598023 * 1000, 0.0]
    earthStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               earth.bodyInMsgName,
                               earthStateMsg)

    jupiterStateMsg = simMessages.SpicePlanetStateSimMsg()
    jupiterStateMsg.PositionVector = [0.0, 778298361 * 1000, 0.0]
    jupiterStateMsg.VelocityVector = [0.0, 0.0 * 1000, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               jupiter.bodyInMsgName,
                               jupiterStateMsg)




    hubR_N = scObject.dynManager.getStateObject("hubPosition")
    hubV_N = scObject.dynManager.getStateObject("hubVelocity")


    rInit_BN_N = unitTestSupport.EigenVector3d2np(hubR_N.getState())
    vInit_BN_N = unitTestSupport.EigenVector3d2np(hubV_N.getState())

    print 'rInit_BN_N = '+str(rInit_BN_N)
    print 'vInit_BN_N = '+str(vInit_BN_N)


    diff = [posData[-1][1] - rInit_BN_N[0], posData[-1][2] - rInit_BN_N[1], posData[-1][3] - rInit_BN_N[2]]
    print 'difference between posData and hubR_N = '+str(diff)


    r_Adj_N = rInit_BN_N + np.array([0.0, -149598023 * 1000, 0.0])
    v_Adj_N = vInit_BN_N + np.array([29.7859 * 1000, 0.0, 0.0])


    hubR_N.setState(unitTestSupport.np2EigenVectorXd(r_Adj_N))
    hubV_N.setState(unitTestSupport.np2EigenVectorXd(v_Adj_N))

    print 'state set to include Earth pos and velocity'

    print 'hubR_N = '+str(unitTestSupport.EigenVector3d2np(hubR_N.getState()))
    print 'hubV_N = '+str(unitTestSupport.EigenVector3d2np(hubV_N.getState()))


    #Updating the simulation timestep and the logging
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, simulationTimeStep)
    dynProcess.updateTaskPeriod(simTaskName, simulationTimeStep)


    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(110000) + T2 - macros.sec2nano(1000000))
    scSim.ExecuteSimulation()

    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))

    endSunTime = len(posData)


    print 'Final position in Sun centered frame = '+str(posData[-1])
    print 'Final velocity in Sun centered frame = ' +str(velData[-1])

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem_parab(sun.mu, posData[idx, 1:4], velData[idx, 1:4])
        rData.append(oeData.rmag / 1000.)
        fData.append(oeData.f + oeData.omega - oeData.omega)

    plt.plot(posData[endEarthTime: -1, 0] * macros.NANO2HOUR, rData[endEarthTime: -1], color='#aa0000')
    plt.xlabel('Time [h]')
    plt.ylabel('Radius [km]')
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)



    # draw the planet
    plt.figure(4)
    fig = plt.gcf()
    ax = fig.gca()
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    planetColor = '#008800'
    planetRadius = sun.radEquator / 1000
    ax.add_artist(plt.Circle((0,0), planetRadius, color=planetColor))
    # draw the actual orbit
    plt.plot(rData[endEarthTime:-1] * np.cos(fData[endEarthTime:-1]), rData[endEarthTime: -1] * np.sin(fData[endEarthTime: -1]), color='#aa0000', linewidth=3.0, label='Simulated Flight')
    plt.legend(loc='lower left')
    plt.grid()
    pltName = "Sun Transfer Orbit"
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)




    print('*****************************************')
    print('CHANGING TIME STEP BEFORE JUPITER ARRIVAL')
    print('*****************************************')

    #This allows the logging to catch up

    simulationTimeStep = macros.sec2nano(5.)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, simulationTimeStep)
    dynProcess.updateTaskPeriod(simTaskName, simulationTimeStep)

    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(110000) + T2)
    scSim.ExecuteSimulation()


    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))
    endTimeStepSwitchTime = len(posData)


    print('Final position prior Jupiter SOI = ' )+str(posData[-1])
    print('Final velocity prior Jupiter SOI = ' )+str(velData[-1])


    print('*****************************************')
    print('SWITCHING TO JUPITER CENTERED FRAME')
    print('*****************************************')


    sun.isCentralBody = False
    jupiter.isCentralBody = True

    # create simulation messages
    sunStateMsg = simMessages.SpicePlanetStateSimMsg()
    sunStateMsg.PositionVector = [0.0, -778398361 * 1000, 0.0]
    sunStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               sun.bodyInMsgName,
                               sunStateMsg)

    earthStateMsg = simMessages.SpicePlanetStateSimMsg()
    earthStateMsg.PositionVector = [0.0, -(149598023 + 778398361) * 1000, 0.0]
    earthStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               earth.bodyInMsgName,
                               earthStateMsg)

    jupiterStateMsg = simMessages.SpicePlanetStateSimMsg()
    jupiterStateMsg.PositionVector = [0.0, 0.0, 0.0]
    jupiterStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    unitTestSupport.setMessage(scSim.TotalSim,
                               simProcessName,
                               jupiter.bodyInMsgName,
                               jupiterStateMsg)



    jupiter.initBody(scObject.gravField.moduleID)

    #  Get current states
    hubPos_N = scObject.dynManager.getStateObject("hubPosition")
    hubVel_N = scObject.dynManager.getStateObject("hubVelocity")


    rJupArrival_N = unitTestSupport.EigenVector3d2np(hubPos_N.getState())
    vJupArrival_N = unitTestSupport.EigenVector3d2np(hubVel_N.getState())

    vJupArrival_N_Norm = np.linalg.norm(vJupArrival_N)

    #Adjusting logged data to be Jupiter centered
    r2 = rJupArrival_N - np.array([1000000.0 * 1000.0, 777598361 * 1000, 0.0])
    v2 = vJupArrival_N - np.array([-13.0697 * 1000, 0.0, 0.0])
    v2Norm = np.linalg.norm(v2)


    hubPos_N.setState(unitTestSupport.np2EigenVectorXd(r2))
    hubVel_N.setState(unitTestSupport.np2EigenVectorXd(v2))

    print 'state set to discount Jupiter pos and velocity'

    print 'hubPos_N = '+str(unitTestSupport.EigenVector3d2np(hubPos_N.getState()))
    print 'hubVel_N = '+str(unitTestSupport.EigenVector3d2np(hubVel_N.getState()))

    #   Setup data logging before the simulation is initialized

    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, simulationTimeStep)
    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(110000) + T2 + macros.sec2nano(900000.))

    scSim.ExecuteSimulation()
    #   retrieve the logged data
    dataPos = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    dataVel = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))

    posJup = dataPos[endTimeStepSwitchTime:-1]
    velJup = dataVel[endTimeStepSwitchTime:-1]

    print('Final position in JCI frame = ')+str(dataPos[-1])
    print('Final velocity in JCI frame = ')+str(dataVel[-1])



    plt.figure(5)
    plt.axis('equal')
    plt.axis([-400000, 400000, -400000, 400000])
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    planetColor = '#008800'
    planetRadius = jupiter.radEquator / 1000
    ax.add_artist(plt.Circle((0,0), planetRadius, color=planetColor))
    #draw actual orbit
    plt.scatter(posJup[:, 1] / 1000., posJup[:, 2] / 1000., color='#aa0000', label='Simulated Flight')
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

    if show_plots:
        plt.show()
    else:

        # close the plots being saved off to avoid over-writing old and new figures
        plt.close("all")

    dataPos = hubPos_N.getState()
    dataPos = [[0.0, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]

    return dataPos, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True  # show_plots
       )