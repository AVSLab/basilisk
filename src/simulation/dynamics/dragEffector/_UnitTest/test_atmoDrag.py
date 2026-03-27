
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
# Purpose:  Test the drag dynamic effector, including inertial and atmosphere-relative velocity modes.
# Author:   Andrew Harris
# Creation Date:  Jan 18, 2017
#

import inspect
import math
import os

import matplotlib.pyplot as plt
import numpy as np
# print dir(exponentialAtmosphere)
from Basilisk.architecture import messaging
from Basilisk.simulation import dragDynamicEffector
from Basilisk.simulation import exponentialAtmosphere, simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import zeroWindModel
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
    orb1 = "LPO"
    showVal = False
    testResults = []
    testMessage = []
    [leoResults, leoMessages] = run(
            showVal, orb1, earthCase)

    testResults = leoResults
    testMessage.append(leoMessages)

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


def setup_basic_drag_sim(rN=None, vN=None, sigmaBN=None,
                         dragCoeff=2.2,  # [-]
                         projArea=10.0):  # [m^2]
    simTaskName = "simTask"
    simProcessName = "simProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    dragEff = dragDynamicEffector.DragDynamicEffector()
    dragEff.ModelTag = "DragEff"
    dragEff.coreParams.projectedArea = projArea
    dragEff.coreParams.dragCoeff = dragCoeff
    dragEff.coreParams.comOffset = [1., 0., 0.]  # [m]

    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    newAtmo.ModelTag = "ExpAtmo"
    newAtmo.baseDensity = 1.217  # [kg/m^3]
    newAtmo.scaleHeight = 8500.0  # [m]
    newAtmo.planetRadius = 6371.0 * 1000.0  # [m]
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    scObject.addDynamicEffector(dragEff)

    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, newAtmo)
    scSim.AddModelToTask(simTaskName, dragEff)

    dragEff.atmoDensInMsg.subscribeTo(newAtmo.envOutMsgs[0])

    if rN is None:
        rN = np.array([7000e3, 0.0, 0.0])  # [m]
    if vN is None:
        vN = np.array([0.0, 7500.0, 0.0])  # [m/s]
    if sigmaBN is None:
        sigmaBN = np.array([0.0, 0.0, 0.0])  # [-] MRP

    scObject.hub.r_CN_NInit = rN  # [m]
    scObject.hub.v_CN_NInit = vN  # [m/s]
    scObject.hub.sigma_BNInit = sigmaBN  # [-] MRP

    dataLog = scObject.scStateOutMsg.recorder()
    atmoLog = newAtmo.envOutMsgs[0].recorder()
    dragLog = dragEff.logger("forceExternal_B")
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, atmoLog)
    scSim.AddModelToTask(simTaskName, dragLog)

    return scSim, scObject, dragEff, dataLog, atmoLog, dragLog


def test_dragInertialVelocity():
    """Verify drag uses inertial velocity when no wind message is linked."""
    rN = np.array([7000e3, 0.0, 0.0])  # [m]
    vN = np.array([0.0, 7600.0, 0.0])  # [m/s]
    sigmaBN = np.array([0.0, 0.0, 0.0])  # [-] MRP

    scSim, _, dragEff, dataLog, atmoLog, dragLog = setup_basic_drag_sim(
        rN=rN, vN=vN, sigmaBN=sigmaBN
    )

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    velData = dataLog.v_BN_N
    attData = dataLog.sigma_BN
    densData = atmoLog.neutralDensity
    dragForce = dragLog.forceExternal_B

    accuracy = 1e-13
    refForce = cannonballDragComp(
        dragEff.coreParams.dragCoeff,
        densData[-1],
        dragEff.coreParams.projectedArea,
        velData[-1],
        attData[-1]
    )

    assert unitTestSupport.isArrayEqual(dragForce[-1, :], refForce, 3, accuracy)


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

    projArea = 10.0  # [m^2]
    dragCoeff = 2.2  # [-]

    dragEffector = dragDynamicEffector.DragDynamicEffector()
    dragEffector.ModelTag = "DragEff"

    dragEffectorTaskName = "drag"
    dragEffector.coreParams.projectedArea = projArea
    dragEffector.coreParams.dragCoeff = dragCoeff
    dragEffector.coreParams.comOffset = [1., 0., 0.]  # [m]

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
    mu = planet.mu  # [m^3/s^2]
    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    #   setup orbit and simulation time
    oe = orbitalMotion.ClassicElements()

    if planetCase == "Earth":
        r_eq = 6371*1000.0  # [m]
        refBaseDens = 1.217  # [kg/m^3]
        refScaleHeight = 8500.0  # [m]

    elif planetCase == "Mars":
        refBaseDens = 0.020  # [kg/m^3]
        refScaleHeight = 11100.0  # [m]
        r_eq = 3389.5 * 1000.0  # [m]
    else:
        return 1, "Test failed- did not initialize planets."
    if orbitCase == "LPO":
        orbAltMin = 300.0*1000.0  # [m]
        orbAltMax = orbAltMin
    elif orbitCase == "LTO":
        orbAltMin = 300*1000.0  # [m]
        orbAltMax = 800.0 * 1000.0  # [m]

    newAtmo.planetRadius = r_eq
    newAtmo.scaleHeight = refScaleHeight
    newAtmo.baseDensity = refBaseDens

    rMin = r_eq + orbAltMin  # [m]
    rMax = r_eq + orbAltMax  # [m]
    oe.a = (rMin+rMax)/2.0  # [m]
    oe.e = 1.0 - rMin/oe.a  # [-]
    oe.i = 0.0*macros.D2R  # [rad]

    oe.Omega = 0.0*macros.D2R  # [rad]
    oe.omega = 0.0*macros.D2R  # [rad]
    oe.f     = 0.0*macros.D2R  # [rad]
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
                                                # with circular or equatorial orbit, some angles are
                                                # arbitrary

    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)  # [rad/s]
    P = 2.*np.pi/n  # [s]

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
    scObject.hub.r_CN_NInit = rN  # [m]
    scObject.hub.v_CN_NInit = vN  # [m/s]

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
    #refDensData = np.zeros([endInd,1])
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
        plt.figure(2,figsize=tuple(np.array((1.0, b/oe.a))*4.75),dpi=100)
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
        plt.xlabel('$i_e$ Coord. [km]')
        plt.ylabel('$i_p$ Coord. [km]')
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

def test_drag_wind_velocity_automatic_usage():
    """Verify drag initializes and runs without error when no wind message is linked."""
    # Create a simple simulation
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("dynamics")
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", simulationTimeStep))

    # Create spacecraft
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"

    # Create atmosphere model
    atmoModel = exponentialAtmosphere.ExponentialAtmosphere()
    atmoModel.ModelTag = "atmosphere"
    atmoModel.baseDensity = 1.217  # [kg/m^3]
    atmoModel.scaleHeight = 8500.0  # [m]
    atmoModel.planetRadius = 6371e3  # [m]

    # Add spacecraft to atmosphere model
    atmoModel.addSpacecraftToModel(scObject.scStateOutMsg)

    # Create drag effector
    dragEff = dragDynamicEffector.DragDynamicEffector()
    dragEff.ModelTag = "drag"
    dragEff.coreParams.projectedArea = 1.0  # [m^2]
    dragEff.coreParams.dragCoeff = 2.2  # [-]

    # Link atmosphere
    dragEff.atmoDensInMsg.subscribeTo(atmoModel.envOutMsgs[0])

    # Add to spacecraft
    scObject.addDynamicEffector(dragEff)

    # Setup basic orbit
    rN = np.array([7000e3, 0.0, 0.0])  # [m]
    vN = np.array([0.0, 7600.0, 0.0])  # [m/s]
    sigmaBN = np.array([0.0, 0.0, 0.0])  # [-] MRP

    scObject.hub.r_CN_NInit = rN  # [m]
    scObject.hub.v_CN_NInit = vN  # [m/s]
    scObject.hub.sigma_BNInit = sigmaBN  # [-] MRP

    # Add models to simulation
    scSim.AddModelToTask("dynamicsTask", scObject)
    scSim.AddModelToTask("dynamicsTask", atmoModel)
    scSim.AddModelToTask("dynamicsTask", dragEff)

    # InitializeSimulation calls Reset internally; should not raise without wind message
    scSim.InitializeSimulation()


def test_drag_wind_velocity_with_wind_message():
    """Verify drag uses atmosphere-relative velocity when wind message is linked.

    Two simulation steps are needed: step 1 populates windInData via UpdateState/ReadInputs;
    step 2's computeForceTorque uses that cached wind velocity.
    The force at step 2 is compared against cannonballDragComp using step-1 cached
    density/wind and step-2 velocity/attitude.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("dynamics")
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.0)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"

    atmoModel = exponentialAtmosphere.ExponentialAtmosphere()
    atmoModel.ModelTag = "atmosphere"
    atmoModel.baseDensity = 1.217  # [kg/m^3]
    atmoModel.scaleHeight = 8500.0  # [m]
    atmoModel.planetRadius = 6371e3  # [m]
    atmoModel.addSpacecraftToModel(scObject.scStateOutMsg)

    windModel = zeroWindModel.ZeroWindModel()
    windModel.ModelTag = "wind"
    windModel.addSpacecraftToModel(scObject.scStateOutMsg)
    planetStateMsg = messaging.SpicePlanetStateMsg().write(messaging.SpicePlanetStateMsgPayload())
    windModel.planetPosInMsg.subscribeTo(planetStateMsg)
    omega_earth = np.array([0.0, 0.0, orbitalMotion.OMEGA_EARTH])  # [rad/s]
    windModel.setPlanetOmega_N(omega_earth)

    dragEff = dragDynamicEffector.DragDynamicEffector()
    dragEff.ModelTag = "drag"
    dragEff.coreParams.projectedArea = 1.0  # [m^2]
    dragEff.coreParams.dragCoeff = 2.2  # [-]
    dragEff.atmoDensInMsg.subscribeTo(atmoModel.envOutMsgs[0])
    dragEff.windVelInMsg.subscribeTo(windModel.envOutMsgs[0])
    scObject.addDynamicEffector(dragEff)

    scObject.hub.r_CN_NInit = np.array([7000e3, 0.0, 0.0])  # [m]
    scObject.hub.v_CN_NInit = np.array([0.0, 7600.0, 0.0])  # [m/s]
    scObject.hub.sigma_BNInit = np.array([0.0, 0.0, 0.0])  # [-] MRP

    scSim.AddModelToTask("dynamicsTask", scObject)
    scSim.AddModelToTask("dynamicsTask", atmoModel)
    scSim.AddModelToTask("dynamicsTask", windModel)
    scSim.AddModelToTask("dynamicsTask", dragEff)

    scSim.InitializeSimulation()

    windLog = windModel.envOutMsgs[0].recorder()
    scStateLog = scObject.scStateOutMsg.recorder()
    atmoLog = atmoModel.envOutMsgs[0].recorder()
    dragLog = dragEff.logger("forceExternal_B")
    scSim.AddModelToTask("dynamicsTask", windLog)
    scSim.AddModelToTask("dynamicsTask", scStateLog)
    scSim.AddModelToTask("dynamicsTask", atmoLog)
    scSim.AddModelToTask("dynamicsTask", dragLog)

    # 2 steps: step 1 loads wind/atmo into cache; step 2 uses cache in computeForceTorque
    scSim.ConfigureStopTime(macros.sec2nano(2.0))
    scSim.ExecuteSimulation()

    # Force at step 2 uses cached step-1 wind/density with step-2 velocity/attitude
    dens_step1 = atmoLog.neutralDensity[0]
    wind_step1 = np.array(windLog.v_air_N[0])
    v_step2 = np.array(scStateLog.v_BN_N[-1])
    sigma_step2 = np.array(scStateLog.sigma_BN[-1])
    drag_force = np.array(dragLog.forceExternal_B[-1])

    refForce = cannonballDragComp(
        dragEff.coreParams.dragCoeff,
        dens_step1,
        dragEff.coreParams.projectedArea,
        v_step2 - wind_step1,
        sigma_step2
    )
    np.testing.assert_allclose(drag_force, refForce, atol=1e-10)


    # close the plots being saved off to avoid over-writing old and new figures

if __name__ == '__main__':
    run(True,"LPO","Earth")
