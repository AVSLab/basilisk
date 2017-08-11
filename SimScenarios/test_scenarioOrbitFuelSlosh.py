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
import numpy as np
import pytest
import math
# from mpl_toolkits.mplot3d import Axes3D

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

import unitTestSupport  # general support file with common unit test functions
import hingedRigidBodyStateEffector
import fuelSloshParticle
import fuelTank
import sim_model
import macros
import ctypes

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  
import matplotlib.pyplot as plt
import orbitalMotion

# import simulation related support
import spacecraftPlus
import gravityEffector
import simIncludeGravity

import fuelSloshParticle
import fuelTank

@pytest.mark.parametrize("planetCase, a_orbit, damping_parameter, timeStep", [
     ('Earth', 6700.0*1000, 0.0, 0.75),
     ('Earth', 6700.0*1000, 0.0, 0.3),
     ('Earth', 6700.0*1000, 15.0, 0.3),
])
# provide a unique test method name, starting with test_
def test_scenarioOrbitFuelSlosh(show_plots, planetCase, a_orbit, damping_parameter, timeStep):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, planetCase, a_orbit, damping_parameter, timeStep)
    assert testResults < 1, testMessage

## \defgroup Tutorials_3_0
##   @{
## Demonstration of basic 6-DOF orbit and fuel slosh simulation setup.
#
# Orbit Setup and Fuel Slosh Simulation 
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting a planet.  The purpose
# is to illustrate how to create a spacecraft, attach a gravity model and a fuel tank, and run
# a basic Basilisk simulation.  The scenarios can be run with the following setup
# parameters:
# Setup | planetCase   | a_orbit (km)     | damping_parameter (Kg/s)   | timeStep (s)
# ----- | ------------ | ---------------  |--------------------------  | --------------
# 1     | Earth        | 6700             | 0.0                        | 0.75
# 2     | Earth        | 6700             | 0.0                        | 0.3
# 3     | Earth        | 6700             | 15.0                       | 0.3
# 
# To run the default scenario 1 from the Basilisk/SimScenarios folder, call the python script through
#
#       python test_scenarioOrbitFuelSlosh.py
#
# However, to play with any scenario scripts as tutorials, you should make a copy of this
# `test_scenarioXXX.py` file into a custom folder outside of the Basilisk directory.  Next,
# one line must be edited in the scenario script to provide the absolute path to the root Basilisk
# directory.  For example, in `test_scenarioOrbitFuelSlosh.py` the line
#~~~~~~~~~~~~~~{.py}
# bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
#~~~~~~~~~~~~~~
# must be uncommented and edited for the particular user's Basilisk directory path.
#
#
# Simulation Scenario Setup Details
# -----
# The simulation layout is shown in the following illustration.  A single simulation process, containing the spacecraft object, 
# is created. Gravity and the Fuel Tank effectors are attached to the spacecraft dynamics to
# simulate the desired scenario.
# ![Simulation Flow Diagram](Images/doc/test_scenarioOrbitFuelSlosh.svg "Illustration")
#
# When the simulation completes 5 plots are shown for each case.  One plot shows
# a 3D orbit around the planet. The second and third plots show the relative variation of orbital angular momentum and energy,
# respectively. The fourth and fifth plots show the relative changes in rotational angular momentum and energy. The third case shows
# a sixth plot, representing the fuel slosh particle motion.
#
#
# The dynamics simulation is setup using a SpacecraftPlus() module. 
#~~~~~~~~~~~~~~~~~{.py}
#     scObject = spacecraftPlus.SpacecraftPlus()
#     scObject.ModelTag = "spacecraftBody"
#     scObject.hub.useTranslation = True
#     scObject.hub.useRotation = True
#
#~~~~~~~~~~~~~~~~~
#
# Next, this module is attached to the simulation process
#~~~~~~~~~~~~~~~~~{.py}
#   scSim.AddModelToTask(simTaskName, scObject)
#~~~~~~~~~~~~~~~~~
# The fuel slosh particle is added to the simulation using the FuelSloshParticle() module.
# The particle characteristics are set as follows:
#~~~~~~~~~~~~~~~~~{.py}
#     # Particle 1
#     scSim.particle1 = fuelSloshParticle.FuelSloshParticle()
# 
#     # Define Variables for particle 1
#     scSim.particle1.k = 1.0
#     scSim.particle1.c = damping_parameter
#     scSim.particle1.r_PB_B = [[0.1], [0], [-0.1]]
#     scSim.particle1.pHat_B = [[1], [0], [0]]
#     scSim.particle1.nameOfRhoState = "fuelSloshParticleRho1"
#     scSim.particle1.nameOfRhoDotState = "fuelSloshParticleRhoDot1"
#     scSim.particle1.nameOfMassState = "fuelSloshParticleMass1"
#     scSim.particle1.rhoInit = 0.05
#     scSim.particle1.rhoDotInit = 0.0
#     scSim.particle1.massInit = 1500.0
#
#~~~~~~~~~~~~~~~~~
# The same procedure has been used to create particle2 and particle3.
# 
# FuelTank() is used to initialize and define the tank properties and the particles are introduced in it.
#
#~~~~~~~~~~~~~~~~~{.py}
#     	#define the fuel tank
#       scSim.tank1 = fuelTank.FuelTank()
#       scSim.tank1.setTankModel(fuelTank.TANK_MODEL_CONSTANT_VOLUME)
#       tankModel = fuelTank.cvar.FuelTankModelConstantVolume
#       tankModel.propMassInit = 400.0
#       tankModel.r_TcT_TInit = [[0.0],[0.0],[0.0]]
#       tankModel.radiusTankInit = 0.5
#       scSim.tank1.r_TB_B = [[0],[0],[0.1]]
#       scSim.tank1.nameOfMassState = "fuelTankMass1"
#       scSim.tank1.pushFuelSloshParticle(scSim.particle1)
#       scSim.tank1.pushFuelSloshParticle(scSim.particle2)
#       scSim.tank1.pushFuelSloshParticle(scSim.particle3)
#       scSim.tank1.updateOnly = True
#~~~~~~~~~~~~~~~~~
#
# At this point, the tank is attached to the spacecraft as a StateEffector
#~~~~~~~~~~~~~~~~~{.py}
#   scObject.addStateEffector(scSim.tank1)
#~~~~~~~~~~~~~~~~~
#
# It is necessary also to define hub properties because we are considering both translation and rotation.
# This can be done using the following code:
#
#~~~~~~~~~~~~~~~~~{.py}
#    scObject.hub.mHub = 1500
#    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
#    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
#    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
#    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]
#~~~~~~~~~~~~~~~~~
#
# The steps to add gravity objects are the same shown in the BasicOrbit tutorial
#
# Before the simulation is ready to run, it must be initialized.  The following code uses a convenient macro routine
# which initializes each BSK module (run self init, cross init and reset) and clears the BSK logging stack.
# The desired output variables are inserted in the simulation in order to request their values after the execution. 
#~~~~~~~~~~~~~~~~~{.py}
#    scSim.InitializeSimulationAndDiscover()
#    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", simulationTimeStep, 0, 0, 'double')
#    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", simulationTimeStep, 0, 2, 'double')
#    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", simulationTimeStep, 0, 2, 'double')
#    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", simulationTimeStep, 0, 0, 'double')
#~~~~~~~~~~~~~~~~~
#
# In the third setup, it is added also the fuel slosh particle state. This line shows how to do that for particle1. 
#
#~~~~~~~~~~~~~~~~~{.py}
#    scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('fuelSloshParticleRho1').getState()", simulationTimeStep, 0, 0, 'double')
#~~~~~~~~~~~~~~~~~ 
#
# Now the simulation is ready to run and we can obtain the outputs using the following code:
#
#~~~~~~~~~~~~~~~~~{.py}
#    scSim.ExecuteSimulation()
#    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_CN_N',range(3))
#    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_CN_N',range(3))
#    
#    orbEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
#    orbAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
#    rotAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
#    rotEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")
#~~~~~~~~~~~~~~~~~    
# The setup 3 requires the particle displacement in order to plot it, so we need to get it as output. It has been done 
# with the following code:
#~~~~~~~~~~~~~~~~~{.py}
# rhoj1Out = scSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('fuelSloshParticleRho1').getState()")
#~~~~~~~~~~~~~~~~~ 
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,         # doUnitTests
#          True,          # show_plots
#          'Earth',       # planetCase
#          6700*1000,     # a_orbit (semimajor axis)
#          0.0            # damping_parameter
#          0.75,          # timeStep
#        )
# ~~~~~~~~~~~~~
# The first 4 arguments can be left as are.  The last two argument controls the
# simulation scenario damping and time step. The
# resulting orbit and relative changes of Orbital Angular Momentum, Orbital Energy,
# Rotational Angular Momentum and Rotational Energy are shown in the following images.
# 
# ![3D Orbit](Images/Scenarios/scenarioOrbitFuelSloshOrbit.svg "3D Orbit")
# ![Changes in Orbital Angular Momentum](Images/Scenarios/scenarioOrbitFuelSloshOAM1.svg "Changes in Orbital Angular Momentum")
# ![Changes in Orbital Energy](Images/Scenarios/scenarioOrbitFuelSloshOE1.svg "Changes in Orbital Energy")
# ![Changes in Rotational Angular Momentum](Images/Scenarios/scenarioOrbitFuelSloshRAM1.svg "Changes in Rotational Angular Momentum")
# ![Changes in Rotational Energy](Images/Scenarios/scenarioOrbitFuelSloshRE1.svg "Changes in Rotational Energy")
#
# N.B. Here we should see conservation with machine precision in every plot, because we are not considering dissipations. 
# In this case, we do not see a perfect conservation of Rotational Angular Momentum and Rotational Energy because
# of the *high* timeStep. We cannot use too high timeStep because we will lose the periodic behaviour of the fuel slosh;
# and the integration could diverge. On the other hand, small timeStep makes the simulation very long. 
# 
# Setup 2
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,          # doUnitTests
#          True,           # show_plots
#		   'Earth',        # planetCase
#          6700*1000,      # a_orbit (semimajor axis)
#          0.0             # damping_parameter
#          0.3,            # timeStep
#        )
# ~~~~~~~~~~~~~
# This case illustrates, how reducing the simulation time step, the solution is close to machine precision.
# This highlights the conservative nature of the forces used so far, confirming that the simulation is running correctly.
#
# ![3D Orbit](Images/Scenarios/scenarioOrbitFuelSloshOrbit.svg "3D Orbit")
# ![Changes in Orbital Angular Momentum](Images/Scenarios/scenarioOrbitFuelSloshOAM2.svg "Changes in Orbital Angular Momentum")
# ![Changes in Orbital Energy](Images/Scenarios/scenarioOrbitFuelSloshOE2.svg "Changes in Orbital Energy")
# ![Changes in Rotational Angular Momentum](Images/Scenarios/scenarioOrbitFuelSloshRAM2.svg "Changes in Rotational Angular Momentum")
# ![Changes in Rotational Energy](Images/Scenarios/scenarioOrbitFuelSloshRE2.svg "Changes in Rotational Energy")
#
#
# Setup 3
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,         # doUnitTests
#          True,          # show_plots
#          'Earth',       # planetCase
#          6700*1000,     # a_orbit (semimajor axis)
#          15.0           # damping_parameter
#          0.3,           # timeStep
#        )
# ~~~~~~~~~~~~~
# This case illustrates that considering damping we have Rotational Energy dissipation due to fuel slosh. 
# It is interesting to note that the Rotational Angular Momentum shows a more precise conservation compared to the previous case.
# This happens because the damping reduces the particles motion, so the integrator becomes more stable.
# Furthermore, from the last image it is possible to see how the motion is similar to a mass spring damper system;
# but it is slightly different because the motion is coupled with the rest of the spacecraft.
#
# ![3D Orbit](Images/Scenarios/scenarioOrbitFuelSloshOrbit.svg "3D Orbit")
# ![Changes in Orbital Angular Momentum](Images/Scenarios/scenarioOrbitFuelSloshOAM3.svg "Changes in Orbital Angular Momentum")
# ![Changes in Orbital Energy](Images/Scenarios/scenarioOrbitFuelSloshOE3.svg "Changes in Orbital Energy")
# ![Changes in Rotational Angular Momentum](Images/Scenarios/scenarioOrbitFuelSloshRAM3.svg "Changes in Rotational Angular Momentum")
# ![Changes in Rotational Energy](Images/Scenarios/scenarioOrbitFuelSloshRE3.svg "Changes in Rotational Energy")
# ![Fuel Slosh Particle Motion](Images/Scenarios/scenarioOrbitFuelSloshParticleMotion.svg "Fuel Slosh Particle Motion")
#
##   @}


def run(doUnitTests, show_plots, planetCase, a_orbit, damping_parameter, timeStep):
    '''Call this routine directly to run the tutorial scenario.'''
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

   
    # Particle 1
    scSim.particle1 = fuelSloshParticle.FuelSloshParticle()

    # Define Variables for particle 1
    scSim.particle1.k = 1.0
    scSim.particle1.c = damping_parameter
    scSim.particle1.r_PB_B = [[0.1], [0], [-0.1]]
    scSim.particle1.pHat_B = [[1], [0], [0]]
    scSim.particle1.nameOfRhoState = "fuelSloshParticleRho1"
    scSim.particle1.nameOfRhoDotState = "fuelSloshParticleRhoDot1"
    scSim.particle1.nameOfMassState = "fuelSloshParticleMass1"
    scSim.particle1.rhoInit = 0.05
    scSim.particle1.rhoDotInit = 0.0
    scSim.particle1.massInit = 1500.0


    # Particle 2
    scSim.particle2 = fuelSloshParticle.FuelSloshParticle()

    # Define Variables for particle 2
    scSim.particle2.k = 1.0
    scSim.particle2.c = damping_parameter
    scSim.particle2.r_PB_B = [[0], [0], [0.1]]
    scSim.particle2.pHat_B = [[0], [1], [0]]
    scSim.particle2.nameOfRhoState = "fuelSloshParticleRho2"
    scSim.particle2.nameOfRhoDotState = "fuelSloshParticleRhoDot2"
    scSim.particle2.nameOfMassState = "fuelSloshParticleMass2"
    scSim.particle2.rhoInit = -0.025
    scSim.particle2.rhoDotInit = 0.0
    scSim.particle2.massInit = 1400.0


    # Particle 3
    scSim.particle3 = fuelSloshParticle.FuelSloshParticle()

    # Define Variables for particle 3
    scSim.particle3.k = 1.0
    scSim.particle3.c = damping_parameter
    scSim.particle3.r_PB_B = [[-0.1], [0], [0.1]]
    scSim.particle3.pHat_B = [[0], [0], [1]]
    scSim.particle3.nameOfRhoState = "fuelSloshParticleRho3"
    scSim.particle3.nameOfRhoDotState = "fuelSloshParticleRhoDot3"
    scSim.particle3.nameOfMassState = "fuelSloshParticleMass3"
    scSim.particle3.rhoInit = -0.015
    scSim.particle3.rhoDotInit = 0.0
    scSim.particle3.massInit = 1300.0

    #define the fuel tank
    scSim.tank1 = fuelTank.FuelTank()
    scSim.tank1.setTankModel(fuelTank.TANK_MODEL_CONSTANT_VOLUME)
    tankModel = fuelTank.cvar.FuelTankModelConstantVolume
    tankModel.propMassInit = 400.0
    tankModel.r_TcT_TInit = [[0.0],[0.0],[0.0]]
    tankModel.radiusTankInit = 0.5
    scSim.tank1.r_TB_B = [[0],[0],[0.1]]
    scSim.tank1.nameOfMassState = "fuelTankMass1"
    scSim.tank1.pushFuelSloshParticle(scSim.particle1)
    scSim.tank1.pushFuelSloshParticle(scSim.particle2)
    scSim.tank1.pushFuelSloshParticle(scSim.particle3)
    scSim.tank1.updateOnly = True

    # ACTIVATE FUEL SLOSH
    scObject.addStateEffector(scSim.tank1)

    # define hub properties
    scObject.hub.mHub = 1500
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]
    
    # clear prior gravitational body and SPICE setup definitions
    simIncludeGravity.clearSetup()

    if planetCase is 'Mercury':
        simIncludeGravity.addMercury()
        simIncludeGravity.gravBodyList[-1].isCentralBody = True          # ensure this is the central gravitational body
    elif planetCase is 'Venus':                   
        simIncludeGravity.addVenus()
        simIncludeGravity.gravBodyList[-1].isCentralBody = True          # ensure this is the central gravitational body
    elif planetCase is 'Earth':                   
        simIncludeGravity.addEarth()
        simIncludeGravity.gravBodyList[-1].isCentralBody = True  
    elif planetCase is 'Mars':                   
        simIncludeGravity.addMars()
        simIncludeGravity.gravBodyList[-1].isCentralBody = True 
    elif planetCase is 'Jupiter':                   
        simIncludeGravity.addJupiter()
        simIncludeGravity.gravBodyList[-1].isCentralBody = True 
    elif planetCase is 'Uranus':                   
        simIncludeGravity.addUranus()
        simIncludeGravity.gravBodyList[-1].isCentralBody = True 
    elif planetCase is 'Neptune':                   
        simIncludeGravity.addNeptune()
        simIncludeGravity.gravBodyList[-1].isCentralBody = True 
    else: sys.exit("Error: insert a correct planetCase")

    r_planet=simIncludeGravity.gravBodyList[-1].radEquator
    if a_orbit<r_planet:
    	sys.exit("error orbit inside planet! a_orbit<r_planet")

    # request the mu parameter for the spacecraft
    mu = simIncludeGravity.gravBodyList[-1].mu
    # attach gravity to the spacecraft
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)
   

    # initialize orbital elements
    oe = orbitalMotion.ClassicElements()
    oe.a=a_orbit
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


    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n
    simulationTime = macros.sec2nano(P/4)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)

    #
    # create simulation messages
    #    

    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    scSim.InitializeSimulationAndDiscover()
    
    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", simulationTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", simulationTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", simulationTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", simulationTimeStep, 0, 0, 'double')
    if damping_parameter != 0.0:
        scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('fuelSloshParticleRho1').getState()", simulationTimeStep, 0, 0, 'double')
        scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('fuelSloshParticleRho2').getState()", simulationTimeStep, 0, 0, 'double')
        scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('fuelSloshParticleRho3').getState()", simulationTimeStep, 0, 0, 'double')
    

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    
    # request states to the simulation
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_CN_N',range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_CN_N',range(3))
    
    orbEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")
   
    if damping_parameter != 0.0:
        rhoj1Out = scSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('fuelSloshParticleRho1').getState()")
        rhoj2Out = scSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('fuelSloshParticleRho2').getState()")
        rhoj3Out = scSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('fuelSloshParticleRho3').getState()")
    
  
    if doUnitTests:
        if damping_parameter==0.0 and timeStep==0.75:
            setupNo=1
        elif damping_parameter==0.0 and timeStep==0.3:
            setupNo=2
        elif damping_parameter!=0.0 and timeStep==0.3:
            setupNo=3
        else:
            print("No standard setup parameters")

    fileNameString = filename[len(path)+6:-3]

    plt.close("all")  # clears out plots from earlier test runs
    fig = plt.figure(1,figsize=(5,5))
    
    # ax = fig.add_subplot(111, projection='3d')
    # # Plot the orbit
    # ax.plot(posData[:,1]*1e-3, posData[:,2]*1e-3, posData[:,3]*1e-3)

    # Make data
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
  
    x = r_planet*1e-3 * np.outer(np.cos(u), np.sin(v))
    y = r_planet*1e-3 * np.outer(np.sin(u), np.sin(v))
    z = r_planet*1e-3 * np.outer(np.ones(np.size(u)), np.cos(v))

    # Plot the spherical surface
    # ax.plot_surface(x, y, z, color='g')
    # ax.set_xlabel('X (km)')
    # ax.set_ylabel('Y (km)')
    # ax.set_zlabel('Z (km)')

    

    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(fileNameString+"Orbit", plt, path)

    plt.figure(2,figsize=(5,3))
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.title("Change in Orbital Angular Momentum")
    plt.xlabel('Time (s)')

    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(fileNameString+"OAM"+str(setupNo), plt, path)

    plt.figure(3,figsize=(5,3))
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.title("Change in Orbital Energy")
    plt.xlabel('Time (s)')

    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(fileNameString+"OE"+str(setupNo), plt, path)

    plt.figure(4,figsize=(5,3))
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.title("Change in Rotational Angular Momentum")
    plt.xlabel('Time (s)')

    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(fileNameString+"RAM"+str(setupNo), plt, path)

    plt.figure(5,figsize=(5,3))
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.title("Change in Rotational Energy")
    plt.xlabel('Time (s)')
    
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(fileNameString+"RE"+str(setupNo), plt, path)

    if damping_parameter != 0.0:
        plt.figure(6,figsize=(5,3)) 
        plt.plot(rhoj1Out[:,0]*1e-9,rhoj1Out[:,1],rhoj2Out[:,0]*1e-9,rhoj2Out[:,1],rhoj3Out[:,0]*1e-9,rhoj3Out[:,1])
        plt.legend(['Particle 1','Particle 2','Particle 3'])
        plt.title("Fuel Slosh Particle Motion")

        if doUnitTests:     # only save off the figure if doing a unit test run
            unitTestSupport.saveScenarioFigure(fileNameString+"ParticleMotion", plt, path)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    if damping_parameter !=0:
        if doUnitTests:

            zita1=damping_parameter/(2*np.sqrt(1500.0*1.0))
            omegan1=np.sqrt(1.0/1500.0)
            settling_time1=-1.0/(zita1*omegan1)*np.log(0.05*np.sqrt(1-zita1**2))
            index_settling_time1=np.argmax(rhoj1Out[:,0]*1e-9>settling_time1)
        
            zita2=damping_parameter/(2*np.sqrt(1400.0*1.0))
            omegan2=np.sqrt(1.0/1400.0)
            settling_time2=-1.0/(zita2*omegan2)*np.log(0.05*np.sqrt(1-zita2**2))
            index_settling_time2=np.argmax(rhoj2Out[:,0]*1e-9>settling_time2)
        
            zita3=damping_parameter/(2*np.sqrt(1300.0*1.0))
            omegan3=np.sqrt(1.0/1300.0)
            settling_time3=-1.0/(zita3*omegan3)*np.log(0.05*np.sqrt(1-zita3**2))
            index_settling_time3=np.argmax(rhoj3Out[:,0]*1e-9>settling_time3)

            accuracy=0.05
            if abs(rhoj1Out[index_settling_time1,1]-rhoj1Out[-1,1])>accuracy:
                testFailCount=testFailCount+1
                testMessages=[testMessages,"Particle 1 settling time does not match second order systems theories"]
            if abs(rhoj2Out[index_settling_time2,1]-rhoj2Out[-1,1])>accuracy:
                testFailCount=testFailCount+1
                testMessages=[testMessages,"Particle 1 settling time does not match second order systems theories"]
            if abs(rhoj3Out[index_settling_time3,1]-rhoj3Out[-1,1])>accuracy:
                testFailCount=testFailCount+1
                testMessages=[testMessages,"Particle 1 settling time does not match second order systems theories"]


            if testFailCount == 0:
                print "PASSED "
            else:
                print testFailCount
                print testMessages

    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    run( False,              # doUnitTests
         True,               # show_plots
        'Earth',      		 # planetCase
         (6700.0)*1000, 	 # a_orbit
         0.0,				 # damping_parameter
         0.75,				 # timeStep
       )
