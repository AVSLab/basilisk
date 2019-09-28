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
#           a 3-DOV spacecraft on a range of orbit types.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 26, 2016
#

import os
import numpy as np

# The following imports are required to do live plotting
from multiprocessing import Pipe, Process
from time import sleep

import matplotlib.pyplot as plt
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import clock_synch
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)

## @page scenarioBasicOrbitLiveGroup
## @{
#
# Basic Orbit Setup Using Live Plotting {#scenarioBasicOrbitLive}
# ====
#
# Scenario Description
# -----
# This script duplicates the basic orbit simulation in the scenario [scenarioBasicOrbit.py](@ref scenarioBasicOrbit).
# The difference is that instead of plotting the results after the simulation has stopped in this script a separate
# thread is created to update the plots live during the simulation run itself.  For more information on doing live
# plotting see help file [using Live Plotting](@ref usingLivePlotting).
#
# To run the default scenario, call the python script through
#
#       python3 scenarioBasicOrbitLivePlot.py
#
# As with [scenarioBasicOrbit.py](@ref scenarioBasicOrbit), different simulation scenarios are setup which are
# controlled through the simulation flags set at the end of the file.
#
# To enable live plotting with a regular Basilisk simulation additional python packages must be imported.
#~~~~~~~~~~~~~~{.py}
# from multiprocessing import Pipe, Process
# from time import sleep
#~~~~~~~~~~~~~~
#
# After configuring the stop time, in livePlotting mode the simulation must be executed differently.  Note the code:
#~~~~~~~~~~~~~~{.py}
#     if livePlots:
#         #plotting refresh rate in ms
#         refreshRate = 1000
#         plotComm, simComm = Pipe()
#         plotArgs = [showPlots, scSim, scObject, useSphericalHarmonics, planetCase, planet, oe, P, mu, orbitCase]
#         simProc = Process(target = scSim.ExecuteSimulation, args = (showPlots, livePlots, simComm, plot, plotArgs))
#         plotProc = Process(target = live_outputs, args = (plotComm, scObject, useSphericalHarmonics, planetCase, planet, oe, P, mu, refreshRate))
#         # Execute simulation and live plotting
#         simProc.start(), plotProc.start()
#         simProc.join(), plotProc.join()
#         return
#     else:
#         scSim.ExecuteSimulation()
#         posData, figureList = plot(showPlots, scSim, scObject, useSphericalHarmonics, planetCase, planet, oe, P, mu, orbitCase)
#         return posData, figureList
#~~~~~~~~~~~~~~
# Without live plotting you simply call `ExecuteSimulation()` and plot the logged data.  The plotting and logging
# is done now within the method `plot()`.
#
# If `livePlots` is true, then a separate process is created to poll the BSK process for data and plot the data
# incrementally.  The live plotting is done with the method `live_outputs()`.  Be cautious in how much data should be
# plotted live as this can greatly slow down the simulation.  Remember that less can be more.
#
# To avoid the live plotting simulation running too fast, a software-based realtime clock
# module is used with an acceleration factor of 50x.
#~~~~~~~~~~~~~~~{.py}
#   clockSync = clock_synch.ClockSynch()
#   clockSync.accelFactor = 50.0
#   scSim.AddModelToTask(simTaskName, clockSync)
#~~~~~~~~~~~~~~~
## @}
def run(showPlots, livePlots, orbitCase, useSphericalHarmonics, planetCase):
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
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #


    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    if planetCase is 'Mars':
        planet = gravFactory.createMarsBarycenter()
        planet.isCentralBody = True           # ensure this is the central gravitational body
        if useSphericalHarmonics:
            planet.useSphericalHarmParams = True
            simIncludeGravBody.loadGravFromFile(bskPath + '/supportData/LocalGravData/GGM2BData.txt',
                                                planet.spherHarm, 100)
    else:  # Earth
        planet = gravFactory.createEarth()
        planet.isCentralBody = True          # ensure this is the central gravitational body
        if useSphericalHarmonics:
            planet.useSphericalHarmParams = True
            simIncludeGravBody.loadGravFromFile(bskPath + '/supportData/LocalGravData/GGM03S-J2-only.txt',
                                                planet.spherHarm, 2)
    mu = planet.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rGEO = 42000. * 1000     # meters
    if orbitCase is 'GEO':
        oe.a = rGEO
        oe.e = 0.00001
        oe.i = 0.0 * macros.D2R
    elif orbitCase is 'GTO':
        oe.a = (rLEO + rGEO) / 2.0
        oe.e = 1.0 - rLEO / oe.a
        oe.i = 0.0 * macros.D2R
    else:                   # LEO case, default case 0
        oe.a = rLEO
        oe.e = 0.0001
        oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are arbitrary

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N

    if livePlots:
        clockSync = clock_synch.ClockSynch()
        clockSync.accelFactor = 50.0
        scSim.AddModelToTask(simTaskName, clockSync)

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    if useSphericalHarmonics:
        simulationTime = macros.sec2nano(3. * P)
    else:
        simulationTime = macros.sec2nano(0.75 * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    if useSphericalHarmonics:
        numDataPoints = 400
    else:
        numDataPoints = 100
    samplingTime = simulationTime // (numDataPoints - 1)

    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    # vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName)

    #
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time
    #
    scSim.ConfigureStopTime(simulationTime)

    if livePlots:
        #plotting refresh rate in ms
        refreshRate = 1000
        plotComm, simComm = Pipe()
        plotArgs = [showPlots, scSim, scObject, useSphericalHarmonics, planetCase, planet, oe, P, mu, orbitCase]
        simProc = Process(target = scSim.ExecuteSimulation, args = (showPlots, livePlots, simComm, plot, plotArgs))
        plotProc = Process(target = live_outputs, args = (plotComm, scObject, useSphericalHarmonics, planetCase, planet, oe, P, mu, refreshRate))
        # Execute simulation and live plotting
        simProc.start(), plotProc.start()
        simProc.join(), plotProc.join()
        return
    else:
        scSim.ExecuteSimulation()
        posData, figureList = plot(showPlots, scSim, scObject, useSphericalHarmonics, planetCase, planet, oe, P, mu, orbitCase)
        return posData, figureList

def plot(showPlots, scSim, scObject, useSphericalHarmonics, planetCase, planet, oe, P, mu, orbitCase):
    np.set_printoptions(precision=16)
    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', list(range(3)))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', list(range(3)))
    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(1, 4):
        plt.plot(posData[:, 0] * macros.NANO2SEC / P, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1" + orbitCase + str(int(useSphericalHarmonics))+ planetCase
    figureList[pltName] = plt.figure(1)

    if useSphericalHarmonics is False:
        # draw orbit in perifocal frame
        b = oe.a * np.sqrt(1 - oe.e * oe.e)
        p = oe.a * (1 - oe.e * oe.e)
        plt.figure(2, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
        plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()
        if planetCase == 'Mars':
            planetColor = '#884400'
        else:
            planetColor = '#008800'
        planetRadius = planet.radEquator / 1000
        ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
        # draw the actual orbit
        rData = []
        fData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
            rData.append(oeData.rmag)
            fData.append(oeData.f + oeData.omega - oe.omega)
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0
                 )
        # draw the full osculating orbit from the initial conditions
        fData = np.linspace(0, 2 * np.pi, 100)
        rData = []
        for idx in range(0, len(fData)):
            rData.append(p / (1 + oe.e * np.cos(fData[idx])))
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555'
                 )
        plt.xlabel('$i_e$ Cord. [km]')
        plt.ylabel('$i_p$ Cord. [km]')
        plt.grid()

    else:
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        smaData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
            smaData.append(oeData.a / 1000.)
        plt.plot(posData[:, 0] * macros.NANO2SEC / P, smaData, color='#aa0000',
                 )
        plt.xlabel('Time [orbits]')
        plt.ylabel('SMA [km]')

    pltName = fileName + "2" + orbitCase + str(int(useSphericalHarmonics)) + planetCase
    figureList[pltName] = plt.figure(2)

    if showPlots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return posData, figureList

def live_outputs(plotComm, scObject, useSphericalHarmonics, planetCase, planet, oe, P, mu, rate):
    dataRequests = setup_live_outputs(scObject, oe, planetCase, planet)

    while True:
        for request in dataRequests:
            plotComm.send(request)
            response = plotComm.recv()
            if response == "TERM":
                plt.close("all")
                return
            pltArgs = []
            for resp in response["dataResp"]:
                pltArgs.append(np.array(resp))

            #
            #   retrieve the logged data
            #
            posData = pltArgs[0]
            velData = pltArgs[1]

            np.set_printoptions(precision=16)

            #
            #   plot the results
            #
            # draw the inertial position vector components
            plt.figure(1)
            fig = plt.gcf()
            ax = fig.gca()
            ax.ticklabel_format(useOffset=False, style='plain')
            for idx in range(1, 4):
                plt.plot(posData[:, 0] * macros.NANO2SEC / P, posData[:, idx] / 1000.,
                         color=unitTestSupport.getLineColor(idx, 3),
                         label='$r_{BN,' + str(idx) + '}$')
            plt.xlabel('Time [orbits]')
            plt.ylabel('Inertial Position [km]')

            if useSphericalHarmonics is False:
                # draw orbit in perifocal frame
                b = oe.a * np.sqrt(1 - oe.e * oe.e)
                p = oe.a * (1 - oe.e * oe.e)
                plt.figure(2, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
                plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
                # draw the planet
                fig = plt.gcf()
                ax = fig.gca()
                if planetCase == 'Mars':
                    planetColor = '#884400'
                else:
                    planetColor = '#008800'
                planetRadius = planet.radEquator / 1000
                ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
                # draw the actual orbit
                rData = []
                fData = []
                for idx in range(0, len(posData)):
                    oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
                    rData.append(oeData.rmag)
                    fData.append(oeData.f + oeData.omega - oe.omega)
                plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0
                         )
                # draw the full osculating orbit from the initial conditions
                fData = np.linspace(0, 2 * np.pi, 100)
                rData = []
                for idx in range(0, len(fData)):
                    rData.append(p / (1 + oe.e * np.cos(fData[idx])))
                plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555'
                         )
                plt.xlabel('$i_e$ Cord. [km]')
                plt.ylabel('$i_p$ Cord. [km]')
            else:
                plt.figure(2)
                fig = plt.gcf()
                ax = fig.gca()
                ax.ticklabel_format(useOffset=False, style='plain')
                smaData = []
                for idx in range(0, len(posData)):
                    oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
                    smaData.append(oeData.a / 1000.)
                plt.plot(posData[:, 0] * macros.NANO2SEC / P, smaData, color='#aa0000',
                         )
                plt.legend(loc='lower right')
                plt.xlabel('Time [orbits]')
                plt.ylabel('SMA [km]')

        plt.pause(.01)
        sleep(rate/1000.)

def setup_live_outputs(scObject, oe, planetCase, planet):
    #define data of interest
    dataRequests = [{"plotID" : None,
                    "plotFun" : None,
                    "dataReq" : [scObject.scStateOutMsgName + '.r_BN_N',
                                scObject.scStateOutMsgName + '.v_BN_N']}]
    return dataRequests

# Setup | orbitCase           | useSphericalHarmonics | planetCase
# ----- | ------------------- | --------------------- | -----------
# 1     | LEO                 | False                 | Earth
# 2     | GTO                 | False                 | Earth
# 3     | GEO                 | False                 | Earth
# 4     | LEO                 | True                  | Earth
# 5     | LEO                 | False                 | Mars

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,        # showPlots
        True,        # livePlots
        'LEO',       # orbit Case (LEO, GTO, GEO)
        False,       # useSphericalHarmonics
        'Earth'      # planetCase (Earth, Mars)
    )
