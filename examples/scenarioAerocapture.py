#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  5 19:46:35 2022

@author: mikaelafelix
"""
# --- My Comments----
# most copied from scenarioOrbitManuever and referenced test_atmoDrag
# used earth as reference body

import os, inspect
import numpy as np
import math


# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import tabularAtmosphere
from Basilisk.utilities import unitTestSupport, RigidBodyKinematics
#print dir(exponentialAtmosphere)
from Basilisk.simulation import dragDynamicEffector
from Basilisk.architecture import messaging
from Basilisk.utilities.readAtmTable import readAtmTable


# filename = inspect.getframeinfo(inspect.currentframe()).filename
# path = os.path.dirname(os.path.abspath(filename))

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages


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
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

   # Construct algorithm and associated C++ container
   # change module to tabAtmo
    tabAtmo = tabularAtmosphere.TabularAtmosphere()   # update with current values
    tabAtmo.ModelTag = "tabularAtmosphere"            # update python name of test module
    
    # define constants & load data
    r_eq = 6378136.6
    filename = bskPath + '/../../supportData/AtmosphereData/EarthGRAMNominal.txt'
    altList, rhoList, tempList = readAtmTable(filename,'EarthGRAM')
        
    # assign constants & ref. data to module
    tabAtmo.planetRadius = r_eq
    tabAtmo.altList = tabularAtmosphere.DoubleVector(altList)    
    tabAtmo.rhoList = tabularAtmosphere.DoubleVector(rhoList)
    tabAtmo.tempList = tabularAtmosphere.DoubleVector(tempList)

    projArea = 10.0  # Set drag area in m^2
    dragCoeff = 2.2  # Set drag ceofficient

    dragEffector = dragDynamicEffector.DragDynamicEffector()
    dragEffector.ModelTag = "DragEff"

    dragEffectorTaskName = "drag"
    dragEffector.coreParams.projectedArea = projArea
    dragEffector.coreParams.dragCoeff = dragCoeff
    dragEffector.coreParams.comOffset = [1., 0., 0.]

    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName, simulationTimeStep))


    # Add test module to runtime call list
    scSim.AddModelToTask(simTaskName, tabAtmo)


    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu


    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(earth.mu, oe)
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    # set the simulation time
    n = np.sqrt(earth.mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.25 * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    dataNewAtmoLog = tabAtmo.envOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, dataNewAtmoLog)

    scSim.AddVariableForLogging('DragEff.forceExternal_B', samplingTime, StartIndex=0, StopIndex=2)
    #
    #   initialize Spacecraft States with initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N
    attData = dataLog.sigma_BN
    dragForce = scSim.GetLogVariableData('DragEff.forceExternal_B')
    densData = dataNewAtmoLog.neutralDensity
    np.set_printoptions(precision=16)
    
    

    
    if show_plots:
        plt.close("all")  # clears out plots from earlier test runs

        # draw the inertial position vector components
        plt.figure(1)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        for idx in range(0,3):
            plt.plot(dataLog.times()*macros.NANO2SEC/P, posData[:, idx]/1000.,
                     color=unitTestSupport.getLineColor(idx,3),
                     label='$r_{BN,'+str(idx)+'}$')
        plt.legend(loc='lower right')
        plt.xlabel('Time [orbits]')
        plt.ylabel('Inertial Position [km]')

        # draw orbit in perifocal frame
        b = oe.a*np.sqrt(1-oe.e*oe.e)
        p = oe.a*(1-oe.e*oe.e)
        plt.figure(2,figsize=np.array((1.0, b/oe.a))*4.75,dpi=100)
        plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b])/1000*1.25)
        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()

        planetColor= '#008800'
        planetRadius = earth.radEquator/1000
        ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
        # draw the actual orbit
        rData=[]
        fData=[]
        for idx in range(0,len(posData)):
            oeData = orbitalMotion.rv2elem(mu,posData[idx,0:3],velData[idx,0:3])
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

        plt.figure()
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        smaData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(mu, posData[idx, 0:3], velData[idx, 0:3])
            smaData.append(oeData.a/1000.)
        plt.plot(posData[:, 0]*macros.NANO2SEC/P, smaData
                 ,color='#aa0000',
                 )
        plt.xlabel('Time [orbits]')
        plt.ylabel('SMA [km]')


        plt.figure()
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='sci')
        plt.plot( dataNewAtmoLog.times()*macros.NANO2SEC, densData)
        plt.title('Density Data vs. Time')
        plt.xlabel('Time')
        plt.ylabel('Density in kg/m^3')

        plt.show()
        plt.close("all")

    if testFailCount == 0:
        print("PASSED: " + dragEffector.ModelTag)
    else:
        print("Failed: " + dragEffector.ModelTag)

    return testFailCount, testMessages

    # close the plots being saved off to avoid over-writing old and new figures
if __name__ == '__main__':
    run(True,"LPO","Earth")
    
    
