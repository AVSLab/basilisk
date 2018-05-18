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

import sys, os, inspect
import pytest
import matplotlib.pyplot as plt
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import sphericalPendulum
from Basilisk.simulation import gravityEffector
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import macros

@pytest.mark.parametrize("useFlag, timeStep", [
     (False, 0.01),
     (False, 0.001),
])
# provide a unique test method name, starting with test_
def test_scenarioSphericalPendulum(show_plots, useFlag, timeStep):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = sphericalPendulumTest(show_plots, useFlag, timeStep)
    assert testResults < 1, testMessage

def sphericalPendulumTest(show_plots, useFlag,timeStep):
    '''Call this routine directly to run the test scenario.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    simTaskName = "simTask"
    simProcessName = "simProcess"
    # create simulation
    scSim=SimulationBaseClass.SimBaseClass()
    # close possible other simulation
    scSim.TotalSim.terminateSimulation()
    #crete a dynamical process
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(timeStep)
    # add task to the dynamical process
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #  create spacecraft object 
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = True

    scSim.AddModelToTask(simTaskName, scObject)
   
    # Pendulum 1
    scSim.pendulum1 = sphericalPendulum.SphericalPendulum()
    # Define Variables for pendulum 1
    scSim.pendulum1.pendulumRadius = 0.3  #  m/s
    scSim.pendulum1.d = [[0.1], [0.1], [0.1]] # m
    scSim.pendulum1.D = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] # N*s/m
    scSim.pendulum1.nameOfPhiState = "sphericalPendulumPhi1"
    scSim.pendulum1.nameOfPhiDotState = "sphericalPendulumPhiDot1"
    scSim.pendulum1.nameOfThetaState= "sphericalPendulumTheta1"
    scSim.pendulum1.nameOfThetaDotState= "sphericalPendulumThetaDot1"
    scSim.pendulum1.nameOfMassState = "sphericalPendulumMass1"
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
    scSim.pendulum2.nameOfPhiState = "sphericalPendulumPhi2"
    scSim.pendulum2.nameOfPhiDotState = "sphericalPendulumPhiDot2"
    scSim.pendulum2.nameOfThetaState= "sphericalPendulumTheta2"
    scSim.pendulum2.nameOfThetaDotState= "sphericalPendulumThetaDot2"
    scSim.pendulum2.nameOfMassState = "sphericalPendulumMass2"
    scSim.pendulum2.phiDotInit = 0.1 # rad/s
    scSim.pendulum2.thetaDotInit = 0.5 # rad/s
    scSim.pendulum2.massInit =40.0 # kg
    # Pendulum frame same as Body frame

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
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

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
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N

    simulationTime = macros.sec2nano(1)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    scSim.InitializeSimulation()
    
    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", simulationTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", simulationTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", simulationTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", simulationTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('sphericalPendulumPhi1').getState()", simulationTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('sphericalPendulumTheta1').getState()", simulationTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('sphericalPendulumThetaDot1').getState()", simulationTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('sphericalPendulumPhiDot1').getState()", simulationTimeStep, 0, 0, 'double')

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # request energy and momentum
    orbEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")

    plt.close("all")  # clears out plots from earlier test runs

    plt.figure(1,figsize=(5,4))
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Orbital Angular Momentum Variation')
    unitTestSupport.writeFigureLaTeX("ChangeInOrbitalAngularMomentum" + str(timeStep), "Change in Orbital Angular Momentum " + str(timeStep), plt, "width=0.8\\textwidth", path)

    plt.figure(2,figsize=(5,4))
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Orbital Energy Variation')
    unitTestSupport.writeFigureLaTeX("ChangeInOrbitalEnergy" + str(timeStep), "Change in Orbital Energy " + str(timeStep), plt, "width=0.8\\textwidth", path)

    
    plt.figure(3,figsize=(5,4))
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Rotational Angular Momentum Variation')
    unitTestSupport.writeFigureLaTeX("ChangeInRotationalAngularMomentum" + str(timeStep), "Change in Rotational Angular Momentum " + str(timeStep), plt, "width=0.8\\textwidth", path)

    plt.figure(4,figsize=(5,4))
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Rotational Energy Variation')
    unitTestSupport.writeFigureLaTeX("ChangeInRotationalEnergy" + str(timeStep), "Change in Rotational Energy " + str(timeStep), plt, "width=0.8\\textwidth", path)
 
    if show_plots:
        plt.show()#

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

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

    if testFailCount == 0:
        print "PASSED "
    else:
        print testFailCount
        print testMessages

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    sphericalPendulumTest(True,              # showplots
         False,               # useFlag
         0.001,				 # timeStep
       )