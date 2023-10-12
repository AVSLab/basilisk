
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Test the validity of a simple exponential atmosphere model.
# Author:   Andrew Harris
# Creation Date:  Jan 18, 2017
#

import inspect
import math
import os

import matplotlib.pyplot as plt
import numpy as np
# print dir(exponentialAtmosphere)
from Basilisk.simulation import dragDynamicEffector
from Basilisk.simulation import exponentialAtmosphere, simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport, RigidBodyKinematics

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.

# provide a unique test method name, starting with test_
def test_scenarioDragOrbit():
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called
    earthCase = "Earth"
    marsCase = "Mars"
    orb1 = "LPO"
    orb2 = "LTO"
    showVal = False
    testResults = []
    testMessage = []
    [leoResults, leoMessages] = run(
            showVal, orb1, earthCase)
    #[gtoResults, gtoMessages] = run(
    #    showVal, orb2, earthCase)
    #[lmoResults, lmoMessages] = run(
        #showVal, orb1, marsCase)
    #[mtoResults, mtoMessages] = run(
        #showVal, orb2, marsCase)

    testResults = leoResults#+gtoResults#+lmoResults+mtoResults
    testMessage.append(leoMessages)
    #testMessage.append(gtoMessages)
    ##testMessage.append(lmoMessages)
    #testMessage.append(mtoMessages)

    assert testResults < 1, testMessage

def expAtmoComp(alt, baseDens, scaleHeight):
    dens = baseDens * math.exp(-alt/scaleHeight)
    return dens

def cannonballDragComp(dragCoeff, dens, area, vel, att):
    dragDir_N = -vel / np.linalg.norm(vel)
    dcm_BN = RigidBodyKinematics.MRP2C(att)
    dragDir_B = dcm_BN.dot(dragDir_N)

    dragForce = 0.5 * dragCoeff * dens * area * np.linalg.norm(vel)**2.0 * dragDir_B
    return dragForce


def run(show_plots, orbitCase, planetCase):
    """Call this routine directly to run the tutorial scenario."""
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages


    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))



    #   Initialize new atmosphere and drag model, add them to task
    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmoTaskName = "atmosphere"
    newAtmo.ModelTag = "ExpAtmo"

    projArea = 10.0  # Set drag area in m^2
    dragCoeff = 2.2  # Set drag ceofficient

    dragEffector = dragDynamicEffector.DragDynamicEffector()
    dragEffector.ModelTag = "DragEff"

    dragEffectorTaskName = "drag"
    dragEffector.coreParams.projectedArea = projArea
    dragEffector.coreParams.dragCoeff = dragCoeff
    dragEffector.coreParams.comOffset = [1., 0., 0.]

    dynProcess.addTask(scSim.CreateNewTask(atmoTaskName, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName, simulationTimeStep))
    scSim.AddModelToTask(atmoTaskName, newAtmo)

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    simpleNavObj = simpleNav.SimpleNav()
    scSim.AddModelToTask(simTaskName, simpleNavObj)
    simpleNavObj.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    scObject.addDynamicEffector(dragEffector)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(dragEffectorTaskName, dragEffector)
    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    dragEffector.atmoDensInMsg.subscribeTo(newAtmo.envOutMsgs[0])

    if planetCase == "Earth":
        planet = gravFactory.createEarth()
    elif planetCase == "Mars":
        planet = gravFactory.createMars()
        planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   setup orbit and simulation time
    oe = orbitalMotion.ClassicElements()

    if planetCase == "Earth":
        r_eq = 6371*1000.0
        refBaseDens = 1.217
        refScaleHeight = 8500.0

    elif planetCase == "Mars":
        refBaseDens = 0.020
        refScaleHeight = 11100.0
        r_eq = 3389.5 * 1000.0
    else:
        return 1, "Test failed- did not initialize planets."
    if orbitCase == "LPO":
        orbAltMin = 300.0*1000.0
        orbAltMax = orbAltMin
    elif orbitCase == "LTO":
        orbAltMin = 300*1000.0
        orbAltMax = 800.0 * 1000.0

    newAtmo.planetRadius = r_eq
    newAtmo.scaleHeight = refScaleHeight
    newAtmo.baseDensity = refBaseDens

    rMin = r_eq + orbAltMin
    rMax = r_eq + orbAltMax
    oe.a = (rMin+rMax)/2.0
    oe.e = 1.0 - rMin/oe.a
    oe.i = 0.0*macros.D2R

    oe.Omega = 0.0*macros.D2R
    oe.omega = 0.0*macros.D2R
    oe.f     = 0.0*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
                                                # with circular or equatorial orbit, some angles are
                                                # arbitrary

    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n

    simulationTime = macros.sec2nano(1*P)

    #
    #   Setup data logging before the simulation is initialized
    #

    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    dataNewAtmoLog = newAtmo.envOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, dataNewAtmoLog)

    dragEffectorLog = dragEffector.logger("forceExternal_B", samplingTime)
    scSim.AddModelToTask(simTaskName, dragEffectorLog)

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
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N
    attData = dataLog.sigma_BN
    dragForce = dragEffectorLog.forceExternal_B
    densData = dataNewAtmoLog.neutralDensity
    np.set_printoptions(precision=16)

    #   Compare to expected values

    endInd = dragForce.shape[0]

    refDragForce = np.zeros([endInd,3])
    refDensData = np.zeros([endInd,1])
    accuracy = 1e-13
    # print planetCase
    # print orbitCase
    for ind in range(0, endInd-1):
        # print "Position data:", posData[ind,1:]
        # print "Velocity data:", velData[ind,1:]
        # print "Density data:", densData[ind,1]
        refDragForce[ind] = cannonballDragComp(dragCoeff,densData[ind],projArea,velData[ind], attData[ind])
        # print "Reference drag data:", refDragForce[ind,:]
        # print "Drag Data:", dragForce[ind,:]
        # print ""
        # check a vector values
    for ind in range(1,endInd-1):
        if not unitTestSupport.isArrayEqual(dragForce[ind,:], refDragForce[ind],3,accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED:  DragEffector failed force unit test with a value difference of "
                + str(np.linalg.norm(dragForce[ind,:]-refDragForce[ind])))

    #
    #   plot the results
    #
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
        planetRadius = planet.radEquator/1000
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
