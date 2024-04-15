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

This tutorial considers a patched conics approach to an interplanetary transfer from Earth to Jupiter, by way of a
heliocentric Hohmann transfer.  The patched conic solution is evaluated making the typical conic trajectory
assumptions where an impulsive Dv changes an Earth-centric circular orbit into a hyperbolic departure orbit that
escapes the Earth system in the Earth's heliocentric direction.  After an elliptical heliocentric transfer orbit
the craft reaches the Jupiter region.  The Jupiter relative arrival is a hyperbolic arrival orbit.
To simulate this solution, a continuous multi-body gravity integration is performed to illustrate
how close the patched conic solution is to reaching a Jupiter fly-by.  The simulation is started and stopped to
change the integration time steps to appropriate values, and also change relative to what planet the trajectory is
logged.  However, the simulation state always includes the gravity of Earth, Sun and Jupiter.  To keep this tutorial
simple, the planets are assumed to be at fixed locations and their ephemeris message is not updated.

The detail of the simulation script is as follows.
This script sets up a basic spacecraft which starts in circular Low Earth Orbit, with logging with respect to the
Earth. The spacecraft then leaves on a hyperbolic orbit with respect to Earth until it reaches the edge of the Earth's
sphere of influence. The frame of reference is then switched to a Sun centered inertial, and the planetary positions
are adjusted accordingly.

The Earth's velocity is then added to the spacecraft (to account for the heliocentric velocity of the
spacecraft) and the simulation is run until the spacecraft approaches Jupiter's Sphere of Influence (SOI). To allow
the simulation to catch up, the time step is reduced just prior to approaching Jupiter's SOI. Then the logging is set
to be relative to Jupiter.

Note that the output position and velocity (when reading message logs) will be relative to the planet that is the
central body during that logging period. So to use the last state in each segment, it needed to be adjusted to account
for the planet/Sun's position and velocity.

How to setup a basic spacecraft simulation is shown in the earlier tutorial :ref:`scenarioBasicOrbit`.
Simulating a Hohmann transfer is illustrated in :ref:`scenarioOrbitManeuver`.
Setting up multiple gravitational bodies is shown in :ref:`scenarioOrbitMultiBody`
while providing pseudo-SPICE messages is laid out in :ref:`scenarioCSS`.

This simulation combines all those techniques as well as changing logging relative to multiple bodies for a single
simulation.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioPatchedConics.py

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True

Plots below illustrate the scenario results for the Earth-centered departure, the heliocentric Hohmann transfer, the
Jupiter centered fly-by, and a heliocentric log plot of the entire transfer.

.. image:: /_images/Scenarios/scenarioPatchedConics1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioPatchedConics2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioPatchedConics3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioPatchedConics4.svg
   :align: center

.. image:: /_images/Scenarios/scenarioPatchedConics5.svg
   :align: center

"""


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose: Patched Conics Interplanetary Trajectory
# Author:   Divinaa Burder
# Creation Date: March 27, 2019
#

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__

bskPath = __path__[0]
from Basilisk.simulation import spacecraft, gravityEffector
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody, unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.utilities import vizSupport

def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
    """
    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.SetProgressBar(True)

    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(5.)

    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    gravFactory = simIncludeGravBody.gravBodyFactory()

    earth = gravFactory.createEarth()
    jupiter = gravFactory.createJupiter()
    sun = gravFactory.createSun()

    # by default the SPICE object will use the solar system barycenter as the inertial origin
    # If the spacecraft() output is desired relative to another celestial object, the zeroBase string
    # name of the SPICE object needs to be changed.
    # In order to specify which body the spacecraft position and velocities are integrated relative to, the `isCentralBody`
    # flag is used.
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)
    scSim.AddModelToTask(simTaskName, scObject)

    #
    # create simulation messages
    #
    # The planets and the Sun are artificially placed at the beginning of this tutorial using state messages to mimic
    # SPICE states. An example of this is shown below.
    # However, since this is a simplified scenario where the planets are not moving, both the Earth and Jupiter are
    # assigned zero velocities in the ephemeris messages. The spacecraft's velocity is manually adjusted to add Earth's
    # velocity after the spacecraft reaches the edge of Earth's Sphere of Influence,
    # and to discount Jupiter's velocity upon
    # entering Jupiter's Sphere of Influence. This ensures the spacecraft has the correct heliocentric and relative
    # positions and velocities even when the planets are not moving.
    earthStateMsg = messaging.SpicePlanetStateMsgPayload()
    earthStateMsg.PositionVector = [0.0, -149598023 * 1000, 0.0]
    earthStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    earthMsg = messaging.SpicePlanetStateMsg().write(earthStateMsg)
    gravFactory.gravBodies['earth'].planetBodyInMsg.subscribeTo(earthMsg)

    jupiterStateMsg = messaging.SpicePlanetStateMsgPayload()
    jupiterStateMsg.PositionVector = [0.0, 778298361 * 1000, 0.0]
    jupiterStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    jupiterMsg = messaging.SpicePlanetStateMsg().write(jupiterStateMsg)
    gravFactory.gravBodies['jupiter barycenter'].planetBodyInMsg.subscribeTo(jupiterMsg)

    sunStateMsg = messaging.SpicePlanetStateMsgPayload()
    sunStateMsg.PositionVector = [0.0, 0.0, 0.0]
    sunStateMsg.VelocityVector = [0.0, 0.0, 0.0]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunStateMsg)
    gravFactory.gravBodies['sun'].planetBodyInMsg.subscribeTo(sunMsg)

    #  Earth Centered Circular orbit and hyperbolic departure
    # initialize spacecraft object and set properties
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    r_E, v_E = orbitalMotion.elem2rv(earth.mu, oe)
    oe = orbitalMotion.rv2elem(earth.mu, r_E, v_E)      # this stores consistent initial orbit elements
    vel_N_Earth = [0.0 * 1000, 0, 0]

    # Hohmann transfer calculations
    rEarth = 149598023. * 1000
    rJupiter = 778298361. * 1000
    at = (rEarth + rJupiter) * .5
    vPt = np.sqrt(2 * sun.mu / rEarth - sun.mu / at)
    vAt = np.sqrt(2 * sun.mu / rJupiter - sun.mu / at)
    n1 = np.sqrt(sun.mu / at / at / at)
    T2 = macros.sec2nano((np.pi) / n1)
    n = np.sqrt(earth.mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    v_Earth = 29.7859 * 1000  # speed of the earth
    v_Earth_rel = vPt - v_Earth  # Required earth relative velocity
    aHyp = - earth.mu / (v_Earth_rel * v_Earth_rel)  # Semimajor axis of departure hyperbola
    eHyp = rLEO * v_Earth_rel * v_Earth_rel / earth.mu + 1  # Eccentricity of hyperbolic departure orbit
    v0 = np.sqrt(v_Earth_rel * v_Earth_rel + 2 * earth.mu / rLEO)  # Earth relative speed s/c needs post burn
    v_c = np.sqrt(earth.mu / rLEO)
    deltaV1 = v0 - v_c
    phi = np.arccos(1 / eHyp) + np.pi  # Burn angle
    E = phi
    t0_Tp = (1 / n) * E

    # initialize Spacecraft States with the initialization variables
    scObject.hub.r_CN_NInit = r_E  # m   - r_BN_N
    scObject.hub.v_CN_NInit = v_E  # m/s - v_BN_N

    # set the simulation time
    simulationTime = macros.sec2nano(t0_Tp)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=__file__
                                              )
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    scSim.InitializeSimulation()

    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    posRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubPosition)
    velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)

    rN = unitTestSupport.EigenVector3d2np(posRef.getState())
    vN = unitTestSupport.EigenVector3d2np(velRef.getState())

    v_S_E = vN - vel_N_Earth  # vel of s/c (S) wrt Earth (E)

    vHat = v_S_E / np.linalg.norm(v_S_E)
    v_S_E = v_S_E + vHat * deltaV1

    vN = v_S_E + vel_N_Earth
    velRef.setState(vN)

    # run simulation for 2nd chunk
    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(110000))
    scSim.ExecuteSimulation()

    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N

    endEarthTime = len(posData)

    #
    # Interplanetary Hohmann Transfer
    #
    # change who is the central body
    earth.isCentralBody = False
    sun.isCentralBody = True

    # The simulation is stopped when the spacecraft reaches the edge of Earth's Sphere of Influence, and a new time step of
    # 1 week is specified by the following:
    oneWeek = macros.sec2nano(1 * 7 * 24 * 60 * 60)  # Changing timestep to 1 week
    simulationTimeStep = oneWeek
    # And the task period is updated in the simulation using:
    dynProcess.updateTaskPeriod(simTaskName, simulationTimeStep)

    # The Earth's position and  velocity also need to be added to the spacecraft to account
    # for the heliocentric position
    # and velocity of the spacecraft. Similar to scenarioOrbitManeuver.py, the states are retrieved,
    # manipulated and fed back to the simulation by:
    posRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubPosition)
    velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)
    rN = unitTestSupport.EigenVector3d2np(posRef.getState())
    vN = unitTestSupport.EigenVector3d2np(velRef.getState())

    pos_N_Earth = [0.0, -149598023 * 1000, 0.0]
    depVel_N_Earth = [29.7859 * 1000, 0, 0]
    rN = rN + pos_N_Earth
    vN = vN + depVel_N_Earth
    posRef.setState(rN)
    velRef.setState(vN)

    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(110000) + T2 - oneWeek*1)
    scSim.ExecuteSimulation()

    hohmann_PosData = dataLog.r_BN_N
    hohmann_VelData = dataLog.v_BN_N

    endSunTime = len(hohmann_PosData)

    #
    # CHANGING TIME STEP BEFORE JUPITER ARRIVAL
    #
    # Since the next time-step is pre-calculated, the simulation is stopped prior to approaching Jupiter's Sphere of
    # Influence. This ensures there is no loss or conflicts in the logged data.
    # The time step is reduced to 100 seconds, and
    # the simulation is propagated until it reaches Jupiter's SOI. Similar to the
    # Interplanetary section, the position and
    # velocity states are pulled and manipulated to be Jupiter-centric and then fed back to the simulation.
    simulationTimeStep = macros.sec2nano(500.)
    dynProcess.updateTaskPeriod(simTaskName, simulationTimeStep)
    dataLog.updateTimeInterval(macros.sec2nano(20*60))
    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(110000) + T2 - oneWeek*0.5)
    scSim.ExecuteSimulation()

    timeSwitch_posData = dataLog.r_BN_N
    endTimeStepSwitchTime = len(timeSwitch_posData)

    #
    # Jupiter Fly-by
    #
    # The simulation is then run to log the fly-by data.
    #
    # At the end of the transfer, the logged data (which is always in the inertial frame,
    # in this case, the heliocentric
    # frame) for each section is manipulated for plotting purposes.
    # To show the Earth-centered departure, the position and velocity of
    # the spacecraft are adjusted to be Earth-centric.
    # The heliocentric Hohmann transfer does not need to be adjusted since it is already in the form required.
    # The position and velocity data of the spacecraft within Jupiter's SOI are adjusted to be Jupiter centered so the
    # trajectory of the flyby with respect to Jupiter can be shown.
    #
    sun.isCentralBody = False
    jupiter.isCentralBody = True

    posRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubPosition)
    velRef = scObject.dynManager.getStateObject(scObject.hub.nameOfHubVelocity)
    rN = unitTestSupport.EigenVector3d2np(posRef.getState())
    vN = unitTestSupport.EigenVector3d2np(velRef.getState())

    pos_N_Jup = [0.0, rJupiter, 0.0]
    vel_N_Jup = [-13.0697 * 1000, 0.0, 0.0]

    rN = rN - pos_N_Jup
    vN = vN - vel_N_Jup

    posRef.setState(rN)
    velRef.setState(vN)

    # Setup data logging before the simulation is initialized

    # scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, simulationTimeStep)
    scSim.ConfigureStopTime(simulationTime + macros.sec2nano(110000) + T2 + oneWeek*6)

    scSim.ExecuteSimulation()
    #   retrieve the logged data
    dataPos = dataLog.r_BN_N
    dataVel = dataLog.v_BN_N

    posJup = dataPos[endTimeStepSwitchTime:-1]
    velJup = dataVel[endTimeStepSwitchTime:-1]

    # Earth Centered Departure
    pos_S_EC = []
    vel_S_EC = []
    for idx in range (0, endEarthTime):
        r_S_E = posData[idx] - pos_N_Earth
        v_S_E = velData[idx] - vel_N_Earth

        pos_S_EC.append(r_S_E)
        vel_S_EC.append(v_S_E)

    # Jupiter Centered Arrival
    pos_S_JC = []

    for idx in range(250, len(posJup)):
        r_S_J = posJup[idx] - pos_N_Jup
        pos_S_JC.append(r_S_J)

    pos_S_JC = np.array(pos_S_JC)

    # Plots
    np.set_printoptions(precision=16)
    # plot the results

    fileName = os.path.basename(os.path.splitext(__file__)[0])
    timeAxis = dataLog.times()

    plt.close("all")  # clears out plots from earlier test runs
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(1) #, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    # plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    plt.axis('equal')
    plt.axis([-20000, 50000, -10000, 10000])
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    planetRadius = earth.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, endEarthTime):
        oeData = orbitalMotion.rv2elem(earth.mu, pos_S_EC[idx], vel_S_EC[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)

    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0,
             label='Simulated Flight')
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
    plt.xlabel('Earth velocity direction [km]')
    plt.ylabel('Sunward direction [km]')
    plt.legend(loc='upper right')
    plt.grid()
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    plt.axis([60, 95, 0 , 25000])
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    rData = []
    for idx in range(0, len(posData[0:300])):
        oeData = orbitalMotion.rv2elem_parab(earth.mu, pos_S_EC[idx], vel_S_EC[idx])
        rData.append(oeData.rmag / 1000.)
    plt.plot(timeAxis[0:300] * macros.NANO2MIN, rData, color='#aa0000',
             )
    plt.xlabel('Time [min]')
    plt.ylabel('Earth Relative Radius [km]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)


    fig = plt.figure(3)
    ax = fig.gca()
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    planetColor = '#008800'
    planetRadius = sun.radEquator / 1000 / 149598000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(hohmann_PosData)):
        oeData = orbitalMotion.rv2elem_parab(sun.mu, hohmann_PosData[idx], hohmann_VelData[idx])
        rData.append(oeData.rmag / 1000.)
        fData.append(oeData.f + oeData.omega - oeData.omega)
    rData = np.array(rData) / 149598000
    plt.plot(rData[endEarthTime:-1] * np.cos(fData[endEarthTime:-1]),
             rData[endEarthTime: -1] * np.sin(fData[endEarthTime: -1]), color='#008800', linewidth=3.0,
             label='Simulated Flight')
    plt.legend(loc='lower left')
    plt.grid()
    plt.xlabel('Y axis - Sunward Direction [AU]')
    plt.ylabel('X axis - Velocity Direction [AU]')
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)


    plt.figure(4,figsize=(5,5))
    plt.axis([-20000000, 20000000, -20000000, 20000000])
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    ax.set_aspect('equal')
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    planetColor = '#008800'
    planetRadius = jupiter.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw actual orbit
    plt.plot(pos_S_JC[:,0] / 1000., pos_S_JC[:,1] / 1000., color='orangered', label='Simulated Flight')
    plt.xlabel('Jupiter velocity direction [km]')
    plt.ylabel('Anti-Sunward direction [km]')
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plt.figure(5)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    ax.get_yaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))
    ax.get_xaxis().set_major_formatter(plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x))))

    dataTime_EC = []
    rData_EC = []
    dataTime_HC = []
    rData_HC = []
    dataTime_TS = []
    rData_TS = []
    dataTime_JC = []
    rData_JC = []

    # To speed up the simulation, only certain data points are recorded for plotting
    for idx in range(0, len(dataPos)):

        if idx <= endEarthTime:
            dataTime_EC.append(timeAxis[idx])
            oeData = orbitalMotion.rv2elem_parab(sun.mu, dataPos[idx], dataVel[idx])
            rData_EC.append(oeData.rmag / 1000.)

        elif (endEarthTime < idx <= endSunTime):
            if idx % 2 == 0: # Records every 2nd data point
                dataTime_HC.append(timeAxis[idx])
                oeData = orbitalMotion.rv2elem_parab(sun.mu, dataPos[idx], dataVel[idx])
                rData_HC.append(oeData.rmag / 1000.)


        elif(endSunTime < idx <= endTimeStepSwitchTime):
            if idx % 20 == 0: # Records every 20th data point
                dataTime_TS.append(timeAxis[idx])
                oeData = orbitalMotion.rv2elem_parab(sun.mu, dataPos[idx], dataVel[idx])
                rData_TS.append(oeData.rmag / 1000.)

        else:
            # if idx % 5 == 0: # Records every 5th data point
            dataTime_JC.append(timeAxis[idx])
            oeData = orbitalMotion.rv2elem_parab(sun.mu, dataPos[idx], dataVel[idx])
            rData_JC.append(oeData.rmag / 1000.)

    dataTime_EC = np.array(dataTime_EC)
    dataTime_HC = np.array(dataTime_HC)
    dataTime_TS = np.array(dataTime_TS)
    dataTime_JC = np.array(dataTime_JC)

    plt.plot(dataTime_EC * macros.NANO2HOUR, rData_EC, color='#aa0000', label='Earth Centered')
    plt.plot(dataTime_HC * macros.NANO2HOUR, rData_HC, color='#008800', label='Heliocentric transfer')
    plt.plot(dataTime_TS * macros.NANO2HOUR, rData_TS, color='#555555', label='Time Switch')
    plt.plot(dataTime_JC * macros.NANO2HOUR, rData_JC, color='orangered', label='Jupiter Centered')
    plt.yscale('log')
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Heliocentric Radius [km]')
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(5)

    if show_plots:
        plt.show()
    else:

        # close the plots being saved off to avoid over-writing old and new figures
        plt.close("all")

    hubPos_N = scObject.dynManager.getStateObject(scObject.hub.nameOfHubPosition)
    dataPos = hubPos_N.getState()
    dataPos = [[dataPos[0][0], dataPos[1][0], dataPos[2][0]]]


    return dataPos, figureList

#
# This statement below ensures that the unit test script can be run as a
# stand-along python script

if __name__ == "__main__":
    run(
        True  # show_plots
    )
