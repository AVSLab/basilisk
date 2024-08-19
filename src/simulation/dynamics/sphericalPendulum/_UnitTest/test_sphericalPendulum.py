
# ISC License
#
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




import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.simulation import spacecraft
from Basilisk.simulation import sphericalPendulum
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import macros
from Basilisk.utilities import pythonVariableLogger
from Basilisk.simulation import fuelTank
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.utilities import simIncludeThruster
from Basilisk.architecture import messaging

@pytest.mark.parametrize("useFlag, testCase", [
     (False, 1),
     (False, 2),
    (False,3)
])
# provide a unique test method name, starting with test_
def test_scenarioSphericalPendulum(show_plots, useFlag, testCase):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = sphericalPendulumTest(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage

def sphericalPendulumTest(show_plots, useFlag,testCase):
    """Call this routine directly to run the test scenario."""
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    if testCase == 1 or testCase ==  3:
        timeStep = 0.01
    if testCase == 2:
        timeStep = 0.001

    simTaskName = "simTask"
    simProcessName = "simProcess"
    # create simulation
    scSim=SimulationBaseClass.SimBaseClass()
    # close possible other simulation
    #crete a dynamical process
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(timeStep)
    # add task to the dynamical process
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #  create spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    scSim.AddModelToTask(simTaskName, scObject)

    # Pendulum 1
    scSim.pendulum1 = sphericalPendulum.SphericalPendulum()
    # Define Variables for pendulum 1
    scSim.pendulum1.pendulumRadius = 0.3  #  m/s
    scSim.pendulum1.d = [[0.1], [0.1], [0.1]] # m
    scSim.pendulum1.D = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] # N*s/m
    scSim.pendulum1.phiDotInit = 0.01 # rad/s
    scSim.pendulum1.thetaDotInit = 0.05 # rad/s
    scSim.pendulum1.massInit = 20.0 # kg
    scSim.pendulum1.pHat_01=[[np.sqrt(2)/2], [0] , [np.sqrt(2)/2]] # first unit vector of the Pendulum frame
    scSim.pendulum1.pHat_02=[[0],[1],[0]]			               # second unit vector of the Pendulum frame
    scSim.pendulum1.pHat_03=[[-np.sqrt(2)/2],[0],[np.sqrt(2)/2]]   # third unit vector of the Pendulum frame

    # Pendulum 2
    scSim.pendulum2 = sphericalPendulum.SphericalPendulum()
    # Define Variables for pendulum 2
    scSim.pendulum2.pendulumRadius = 0.4  #  m/s
    scSim.pendulum2.d = [[0.1], [0.1], [0.1]] # m
    scSim.pendulum2.D = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] # N*s/m
    scSim.pendulum2.phiDotInit = 0.1 # rad/s
    scSim.pendulum2.thetaDotInit = 0.5 # rad/s
    scSim.pendulum2.massInit =40.0 # kg
    # Pendulum frame same as Body frame

    if testCase == 3:
        # add thruster devices
        thFactory = simIncludeThruster.thrusterFactory()
        thFactory.create('MOOG_Monarc_445',
                                  [1,0,0],                # location in S frame
                                  [0,1,0]                 # direction in S frame
                                  )


        # create thruster object container and tie to spacecraft object
        thrustersDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
        thFactory.addToSpacecraft("Thrusters",
                                  thrustersDynamicEffector,
                                  scObject)

        scSim.fuelTankStateEffector = fuelTank.FuelTank()
        tankModel = fuelTank.FuelTankModelConstantVolume()
        scSim.fuelTankStateEffector.setTankModel(tankModel)
        tankModel.propMassInit = 40.0
        tankModel.r_TcT_TInit = [[0.0],[0.0],[0.0]]
        scSim.fuelTankStateEffector.r_TB_B = [[0.0],[0.0],[0.0]]
        tankModel.radiusTankInit = 46.0 / 2.0 / 3.2808399 / 12.0

        # Add tank and thruster
        scObject.addStateEffector(scSim.fuelTankStateEffector)
        scSim.fuelTankStateEffector.addThrusterSet(thrustersDynamicEffector)

        # set thruster commands
        ThrustMessage = messaging.THRArrayOnTimeCmdMsgPayload()
        ThrustMessage.OnTimeRequest = [5.0]
        thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(ThrustMessage)
        thrustersDynamicEffector.cmdsInMsg.subscribeTo(thrCmdMsg)

        # Add test module to runtime call list
        scSim.AddModelToTask(simTaskName, scSim.fuelTankStateEffector)
        scSim.AddModelToTask(simTaskName, thrustersDynamicEffector)

        fuelLog = scSim.fuelTankStateEffector.fuelTankOutMsg.recorder()
        scSim.AddModelToTask(simTaskName, fuelLog)

        # Add particles to tank to activate mass depletion
        scSim.fuelTankStateEffector.pushFuelSloshParticle(scSim.pendulum1)
        scSim.fuelTankStateEffector.pushFuelSloshParticle(scSim.pendulum2)

    # define hub properties
    scObject.hub.mHub = 1500 # kg
    scObject.hub.r_BcB_B = [[1.0], [0.5], [0.1]] # m
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]] # kg*m^2
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]] # rad
    scObject.hub.omega_BN_BInit = [[1.0], [0.5], [0.1]] # rad/s

    # Add fuel slosh to spacecraft
    scObject.addStateEffector(scSim.pendulum1)
    scObject.addStateEffector(scSim.pendulum2)

    # call for a fresh copy of the gravitational body factory
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True  # ensure this is the central gravitational body

    planetRadius = planet.radEquator
    mu = planet.mu

    # attach gravity to the spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # initialize orbital elements
    oe = orbitalMotion.ClassicElements()
    oe.a=6700.0*1000
    oe.e=0.01
    oe.omega=100.0*macros.D2R
    oe.Omega=100.0*macros.D2R
    oe.i=30.0*macros.D2R
    oe.f=0.0
    # convert them in position and velocity
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # attach the state to the spacecraft
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    simulationTime = macros.sec2nano(10)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    scSim.AddModelToTask(simTaskName, scObjectLog)

    if testCase == 3:
        stateLog = pythonVariableLogger.PythonVariableLogger({
            "sphericalPendulumMass1": lambda _: scObject.dynManager.getStateObject(scSim.pendulum1.nameOfMassState).getState(),
            "sphericalPendulumMass2": lambda _: scObject.dynManager.getStateObject(scSim.pendulum2.nameOfMassState).getState(),
        })
        scSim.AddModelToTask(simTaskName, stateLog)

    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    if testCase == 3:
        fuelMass = fuelLog.fuelMass
        fuelMassDot = fuelLog.fuelMassDot
        mass1Out = unitTestSupport.addTimeColumn(stateLog.times(), stateLog.sphericalPendulumMass1)
        mass2Out = unitTestSupport.addTimeColumn(stateLog.times(), stateLog.sphericalPendulumMass2)


    # request energy and momentum
    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)

    if timeStep == 0.01:
        testCaseName = "OneHundredth"
    if timeStep == 0.001:
        testCaseName = "OneThousandth"

    plt.close("all")  # clears out plots from earlier test runs

    if testCase != 3:
        plt.figure(1,figsize=(5,4))
        plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
        plt.xlabel('Time (s)')
        plt.ylabel('Relative Orbital Angular Momentum Variation')
        unitTestSupport.writeFigureLaTeX("ChangeInOrbitalAngularMomentum" + testCaseName, "Change in Orbital Angular Momentum " + testCaseName, plt, r"width=0.8\textwidth", path)

        plt.figure(2,figsize=(5,4))
        plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
        plt.xlabel('Time (s)')
        plt.ylabel('Relative Orbital Energy Variation')
        unitTestSupport.writeFigureLaTeX("ChangeInOrbitalEnergy" + testCaseName, "Change in Orbital Energy " + testCaseName, plt, r"width=0.8\textwidth", path)

        plt.figure(3,figsize=(5,4))
        plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
        plt.xlabel('Time (s)')
        plt.ylabel('Relative Rotational Angular Momentum Variation')
        unitTestSupport.writeFigureLaTeX("ChangeInRotationalAngularMomentum" + testCaseName, "Change in Rotational Angular Momentum " + testCaseName, plt, r"width=0.8\textwidth", path)

        plt.figure(4,figsize=(5,4))
        plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
        plt.xlabel('Time (s)')
        plt.ylabel('Relative Rotational Energy Variation')
        unitTestSupport.writeFigureLaTeX("ChangeInRotationalEnergy" + testCaseName, "Change in Rotational Energy " + testCaseName, plt, r"width=0.8\textwidth", path)
    if testCase == 3:
        plt.figure()
        plt.plot(fuelLog.times()*1e-9, fuelMass)
        plt.title("Tank Fuel Mass")
        plt.figure()
        plt.plot(fuelLog.times()*1e-9, fuelMassDot)
        plt.title("Tank Fuel Mass Dot")
        plt.figure()
        plt.plot(mass1Out[:,0]*1e-9, mass1Out[:,1])
        plt.title("Fuel Particle 1 Mass")
        plt.figure()
        plt.plot(mass2Out[:,0]*1e-9, mass2Out[:,1])
        plt.title("Fuel Particle 2 Mass")
        mDotFuel = -0.19392039093
        mDotParicle1True = mDotFuel*(20./100.)
        mDotParicle2True = mDotFuel*(40./100.)
        mDotParicle1Data = [0,(mass1Out[2,1] - mass1Out[1,1])/((mass1Out[2,0] - mass1Out[1,0])*1e-9)]
        mDotParicle2Data = [0,(mass2Out[2,1] - mass2Out[1,1])/((mass2Out[2,0] - mass2Out[1,0])*1e-9)]

    if show_plots:
        plt.show()
        plt.close('all')

    if testCase != 3:
        accuracy = 1e-8
        for k in range(len((rotAngMom_N[:,1]))):
            if abs((rotAngMom_N[k,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1])>accuracy:
                testFailCount += 1
                testMessages.append("FAILED: SphericalPendulum does not conserve Rotational Angular Momentum around x axes (timeStep={}s)".format(timeStep))
            if abs((rotAngMom_N[k,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2])>accuracy:
                testFailCount += 1
                testMessages.append("FAILED: SphericalPendulum does not conserve Rotational Angular Momentum around y axes (timeStep={}s)".format(timeStep))
            if abs((rotAngMom_N[k,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])>accuracy:
                testFailCount += 1
                testMessages.append("FAILED: SphericalPendulum does not conserve Rotational Angular Momentum around z axes (timeStep={}s)".format(timeStep))
            if abs((orbAngMom_N[k,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1])>accuracy:
                testFailCount += 1
                testMessages.append("FAILED: SphericalPendulum does not conserve Orbital Angular Momentum around x axes (timeStep={}s)".format(timeStep))
            if abs((orbAngMom_N[k,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2])>accuracy:
                testFailCount += 1
                testMessages.append("FAILED: SphericalPendulum does not conserve Orbital Angular Momentum around y axes (timeStep={}s)".format(timeStep))
            if abs((orbAngMom_N[k,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])>accuracy:
                testFailCount += 1
                testMessages.append("FAILED: SphericalPendulum does not conserve Orbital Angular Momentum around z axes (timeStep={}s)".format(timeStep))
            if abs((rotEnergy[k,1] - rotEnergy[0,1])/rotEnergy[0,1])>accuracy:
                testFailCount += 1
                testMessages.append("FAILED: SphericalPendulum does not conserve Rotational Energy (timeStep={}s)".format(timeStep))
            if abs((orbEnergy[k,1] - orbEnergy[0,1])/orbEnergy[0,1])>accuracy:
                testFailCount += 1
                testMessages.append("FAILED: SphericalPendulum does not conserve Orbital Energy (timeStep={}s)".format(timeStep))
    if testCase == 3:
        accuracy = 1e-4
        if not unitTestSupport.isDoubleEqual(mDotParicle1Data[1],mDotParicle1True,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Linear Spring Mass Damper unit test failed mass 1 dot test")
        if not unitTestSupport.isDoubleEqual(mDotParicle2Data[1],mDotParicle2True,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Linear Spring Mass Damper unit test failed mass 2 dot test")

    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testFailCount)
        print(testMessages)

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    sphericalPendulumTest(True,              # showplots
         False,               # useFlag
         3,				 # testCase
       )
