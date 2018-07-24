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


import os
import inspect
import numpy as np

from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.simulation import linearSpringMassDamper
from Basilisk.simulation import fuelTank

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
import matplotlib.pyplot as plt
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import macros

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import simIncludeGravBody

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


## \defgroup Tutorials_3_0
## @{
## Demonstration of basic 6-DOF orbit and fuel slosh simulation setup.
#
# Orbit Setup and Fuel Slosh Simulation
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft orbiting a planet.  The purpose
# is to illustrate how to create a spacecraft, attach a gravity model and a fuel tank, and run
# a basic Basilisk simulation.  The scenarios can be run with the following setup parameters:
#
# Setup | damping_parameter (kg/s)   | timeStep (s)
# ----- |--------------------------  | --------------
# 1     | 0.0                        | 0.75
# 2     | 0.0                        | 0.3
# 3     | 15.0                       | 0.75
#
# To run the default scenario 1 from the Basilisk/scenarios folder, call the python script through
#
#       python scenarioFuelSlosh.py
#
# However, to play with any scenario scripts as tutorials, you should make a copy of this
# `scenarioXXX.py` file into a custom folder outside of the Basilisk directory.  Next,
# one line must be edited in the scenario script to provide the absolute path to the root Basilisk
# directory.  For example, in `scenarioFuelSlosh.py` the line
#~~~~~~~~~~~~~~{.py}
# bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
#~~~~~~~~~~~~~~
# must be uncommented and edited for the particular user's Basilisk directory path.
#
#
# Simulation Scenario Initial Setup
# -----
# The simulation layout is shown in the following illustration.  A single simulation process, containing the spacecraft object,
# is created. Gravity and the Fuel Tank effectors are attached to the spacecraft dynamics to
# simulate the desired scenario.
# ![Simulation Flow Diagram](Images/doc/test_scenarioFuelSlosh.svg "Illustration")
#
# When the simulation completes 5 plots are shown for each case.  One plot shows the spacecraft trajectory in the orbital plane.
# The second and third plots show the relative variation of orbital angular momentum and energy,
# respectively. The fourth and fifth plots show the relative changes in rotational angular momentum and energy. The third case shows
# a sixth plot, representing the fuel slosh particle motion.
#
#
# The dynamics simulation is setup using a SpacecraftPlus() module.
#~~~~~~~~~~~~~~~~~{.py}
#     scObject = spacecraftPlus.SpacecraftPlus()
#     scObject.ModelTag = "spacecraftBody"
#
#~~~~~~~~~~~~~~~~~
#
# Next, this module is attached to the simulation process
#~~~~~~~~~~~~~~~~~{.py}
#   scSim.AddModelToTask(simTaskName, scObject)
#~~~~~~~~~~~~~~~~~
#
# State Effectors Setup
# -------
# The model used to simulate the fuel slosh is a classic mass spring damper system coupled with the rest of the spacecraft.
# The fuel slosh particle is added to the simulation using the LinearSpringMassDamper() module.
# The particle characteristics are set as follows:
#~~~~~~~~~~~~~~~~~{.py}
#     # Particle 1
#     scSim.particle1 = linearSpringMassDamper.LinearSpringMassDamper()
#
#     # Define Variables for particle 1
#     scSim.particle1.k = 1.0  # kg/s^2 (N/m)
#     scSim.particle1.c = damping_parameter # kg/s
#     scSim.particle1.r_PB_B = [[0.1], [0], [-0.1]]0], [-0.1]] # m
#     scSim.particle1.pHat_B = [[1], [0], [0]], [0]]
#     scSim.particle1.nameOfRhoState = "linearSpringMassDamperRho1"linearSpringMassDamperRho1"
#     scSim.particle1.nameOfRhoDotState = "linearSpringMassDamperRhoDot1"
#     scSim.particle1.nameOfMassState = "linearSpringMassDamperMass1"
#     scSim.particle1.rhoInit = 0.05 # m
#     scSim.particle1.rhoDotInit = 0.0 # m/s
#     scSim.particle1.massInit = 1500.0 # kg
#
#~~~~~~~~~~~~~~~~~
# Where k is the spring constant in kg/s^2 (N/m), c is the damping coefficient expressed in kg/s.
# As we can see from the following illustrations, r_PB_B is the vector that expresses the particle equilibrium position
# in the body reference frame. pHat_B is the direction of particle motion, expressed in the body reference frame.
# rhoInit and rhoDotInit are the initial particle position and velocity, expressed in m and m/s respectively. massInit is
# fuel mass that is moving in the selected direction.
#
# ![Spacecraft Model](Images/doc/test_scenarioFuelSloshSpacecraft.svg "Spacecraft Model")
# ![Fuel Slosh Particle Model](Images/doc/test_scenarioLinearSpringMassDamper.svg "Fuel Slosh Particle Model")
#
# For further information on the model implemented you can consult this
# <a target='_blank' href="http://hanspeterschaub.info/Papers/Allard2016a.pdf"><b>conference paper.</b></a>
# The same procedure has been used to create particle2 and particle3.
#
# FuelTank() is used to initialize and define the tank properties and the particles are introduced in it.
#
#~~~~~~~~~~~~~~~~~{.py}
#     	#define the fuel tank
#       scSim.tank1 = fuelTank.FuelTank()
#       scSim.tank1.setTankModel(fuelTank.TANK_MODEL_CONSTANT_VOLUME)
#       tankModel = fuelTank.cvar.FuelTankModelConstantVolume
#       tankModel.propMassInit = 400.0 # kg
#       tankModel.r_TcT_TInit = [[0.0],[0.0],[0.0]] # m
#       tankModel.radiusTankInit = 0.5 # m
#       scSim.tank1.r_TB_B = [[0],[0],[0.1]] # m
#       scSim.tank1.nameOfMassState = "fuelTankMass1"
#       scSim.tank1.pushfuelSloshParticle(scSim.particle1)
#       scSim.tank1.pushfuelSloshParticle(scSim.particle2)
#       scSim.tank1.pushfuelSloshParticle(scSim.particle3)
#       scSim.tank1.updateOnly = True
#~~~~~~~~~~~~~~~~~
# The fuel tank is represented by a constant volume sphere. The radius is set to 0.5 m using the radiusTankInit variable.
# propMassInit is the initial propellant mass that does not generate slosh. r_TcT_TInit is the initial position vector
# from B to tank point in B frame components. r_TB_B is the position of the tank in body frame.
#
#
# At this point, the tank is attached to the spacecraft as a StateEffector().
#~~~~~~~~~~~~~~~~~{.py}
#   scObject.addStateEffector(scSim.tank1)
#~~~~~~~~~~~~~~~~~
#
# Here, it is reported how we can define hub properties, as we have already done in
# [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback).
#
#~~~~~~~~~~~~~~~~~{.py}
#    scObject.hub.mHub = 1500 # kg
#    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m
#    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]] # kg*m^2
#    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]] # rad
#    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]] # rad/s
#~~~~~~~~~~~~~~~~~
#
# The steps to add gravity objects are the same shown in the [scenarioBasicOrbit.py](@ref scenarioBasicOrbit).
#
# Simulation Run
# -----
# Before the simulation is ready to run, it must be initialized.  The following code uses a convenient macro routine
# which initializes each Basilisk module (run self init, cross init and reset) and clears the BSK logging stack.
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
#    scSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('linearSpringMassDamperRho1').getState()", simulationTimeStep, 0, 0, 'double')
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
# rhoj1Out = scSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('linearSpringMassDamperRho1').getState()")
#~~~~~~~~~~~~~~~~~
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,          # show_plots
#          0.0            # damping_parameter
#          0.75,          # timeStep
#        )
# ~~~~~~~~~~~~~
# The first two arguments can be left as are.  The last two argument controls the
# simulation scenario damping coefficient and time step. The
# resulting orbit and relative changes of Orbital Angular Momentum, Orbital Energy,
# Rotational Angular Momentum and Rotational Energy are shown in the following images.
#
# ![Orbit](Images/Scenarios/scenarioFuelSloshOrbit.svg "Trajectory the in Orbit Plane")
# ![Changes in Orbital Angular Momentum](Images/Scenarios/scenarioFuelSloshOAM1.svg "Changes in Orbital Angular Momentum")
# ![Changes in Orbital Energy](Images/Scenarios/scenarioFuelSloshOE1.svg "Changes in Orbital Energy")
# ![Changes in Rotational Angular Momentum](Images/Scenarios/scenarioFuelSloshRAM1.svg "Changes in Rotational Angular Momentum")
# ![Changes in Rotational Energy](Images/Scenarios/scenarioFuelSloshRE1.svg "Changes in Rotational Energy")
#
# Here we should see conservation with machine precision in every plot, because we are not considering dissipations.
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
#     run(
#          True,           # show_plots
#          0.0             # damping_parameter
#          0.3,            # timeStep
#        )
# ~~~~~~~~~~~~~
# This case illustrates, how reducing the simulation time step, the solution is close to machine precision.
# This highlights the conservative nature of the forces used so far, confirming that the simulation is running correctly.
#
# ![Orbit](Images/Scenarios/scenarioFuelSloshOrbit.svg "Trajectory the in Orbit Plane")
# ![Changes in Orbital Angular Momentum](Images/Scenarios/scenarioFuelSloshOAM2.svg "Changes in Orbital Angular Momentum")
# ![Changes in Orbital Energy](Images/Scenarios/scenarioFuelSloshOE2.svg "Changes in Orbital Energy")
# ![Changes in Rotational Angular Momentum](Images/Scenarios/scenarioFuelSloshRAM2.svg "Changes in Rotational Angular Momentum")
# ![Changes in Rotational Energy](Images/Scenarios/scenarioFuelSloshRE2.svg "Changes in Rotational Energy")
#
#
# Setup 3
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,          # show_plots
#          15.0           # damping_parameter
#          0.75,          # timeStep
#        )
# ~~~~~~~~~~~~~
# This case illustrates that considering damping we have Rotational Energy dissipation due to fuel slosh.
# It is interesting to note that the Rotational Angular Momentum shows a more precise conservation compared to
# the first case, for equal timeStep.
# This happens because the damping reduces the particles motion, so the integrator becomes more stable.
# Furthermore, from the last image it is possible to see how the motion is similar to a mass spring damper system;
# but it is slightly different because the motion is coupled with the rest of the spacecraft.
#
# ![Orbit](Images/Scenarios/scenarioFuelSloshOrbit.svg "Trajectory the in Orbit Plane")
# ![Changes in Orbital Angular Momentum](Images/Scenarios/scenarioFuelSloshOAM3.svg "Changes in Orbital Angular Momentum")
# ![Changes in Orbital Energy](Images/Scenarios/scenarioFuelSloshOE3.svg "Changes in Orbital Energy")
# ![Changes in Rotational Angular Momentum](Images/Scenarios/scenarioFuelSloshRAM3.svg "Changes in Rotational Angular Momentum")
# ![Changes in Rotational Energy](Images/Scenarios/scenarioFuelSloshRE3.svg "Changes in Rotational Energy")
# ![Fuel Slosh Particle Motion](Images/Scenarios/scenarioLinearSpringMassDamperMotion.svg "Fuel Slosh Particle Motion")
#
## @}
def run(show_plots, damping_parameter, timeStep):
    '''Call this routine directly to run the tutorial scenario.'''

    simTaskName = "simTask"
    simProcessName = "simProcess"
    # create simulation
    scSim = SimulationBaseClass.SimBaseClass()
    # close possible other simulation
    scSim.TotalSim.terminateSimulation()
    # crete a dynamical process
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(timeStep)
    # add task to the dynamical process
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #  create spacecraft object
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    scSim.AddModelToTask(simTaskName, scObject)

    # Particle 1
    scSim.particle1 = linearSpringMassDamper.LinearSpringMassDamper()

    # Define Variables for particle 1
    scSim.particle1.k = 1.0  # kg/s^2 (N/m)
    scSim.particle1.c = damping_parameter  # kg/s
    scSim.particle1.r_PB_B = [[0.1], [0], [-0.1]]  # m
    scSim.particle1.pHat_B = [[1], [0], [0]]
    scSim.particle1.nameOfRhoState = "linearSpringMassDamperRho1"
    scSim.particle1.nameOfRhoDotState = "linearSpringMassDamperRhoDot1"
    scSim.particle1.nameOfMassState = "linearSpringMassDamperMass1"
    scSim.particle1.rhoInit = 0.05  # m
    scSim.particle1.rhoDotInit = 0.0  # m/s
    scSim.particle1.massInit = 1500.0  # kg

    # Particle 2
    scSim.particle2 = linearSpringMassDamper.LinearSpringMassDamper()

    # Define Variables for particle 2
    scSim.particle2.k = 1.0  # kg/s^2 (N/m)
    scSim.particle2.c = damping_parameter  # kg/s
    scSim.particle2.r_PB_B = [[0], [0], [0.1]]  # m
    scSim.particle2.pHat_B = [[0], [1], [0]]
    scSim.particle2.nameOfRhoState = "linearSpringMassDamperRho2"
    scSim.particle2.nameOfRhoDotState = "linearSpringMassDamperRhoDot2"
    scSim.particle2.nameOfMassState = "linearSpringMassDamperMass2"
    scSim.particle2.rhoInit = -0.025  # m
    scSim.particle2.rhoDotInit = 0.0  # m/s
    scSim.particle2.massInit = 1400.0  # kg

    # Particle 3
    scSim.particle3 = linearSpringMassDamper.LinearSpringMassDamper()

    # Define Variables for particle 3
    scSim.particle3.k = 1.0  # kg/s^2 (N/m)
    scSim.particle3.c = damping_parameter  # kg/s
    scSim.particle3.r_PB_B = [[-0.1], [0], [0.1]]  # m
    scSim.particle3.pHat_B = [[0], [0], [1]]
    scSim.particle3.nameOfRhoState = "linearSpringMassDamperRho3"
    scSim.particle3.nameOfRhoDotState = "linearSpringMassDamperRhoDot3"
    scSim.particle3.nameOfMassState = "linearSpringMassDamperMass3"
    scSim.particle3.rhoInit = -0.015  # m
    scSim.particle3.rhoDotInit = 0.0  # m/s
    scSim.particle3.massInit = 1300.0  # kg

    # define the fuel tank
    scSim.tank1 = fuelTank.FuelTank()
    scSim.tank1.setTankModel(fuelTank.TANK_MODEL_CONSTANT_VOLUME)
    tankModel = fuelTank.cvar.FuelTankModelConstantVolume
    tankModel.propMassInit = 400.0  # kg
    tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]  # m
    tankModel.radiusTankInit = 0.5  # m
    scSim.tank1.r_TB_B = [[0], [0], [0.1]]  # m
    scSim.tank1.nameOfMassState = "fuelTankMass1"
    scSim.tank1.pushFuelSloshParticle(scSim.particle1)
    scSim.tank1.pushFuelSloshParticle(scSim.particle2)
    scSim.tank1.pushFuelSloshParticle(scSim.particle3)
    scSim.tank1.updateOnly = True

    # ACTIVATE FUEL SLOSH
    scObject.addStateEffector(scSim.tank1)
    scObject.addStateEffector(scSim.particle1)
    scObject.addStateEffector(scSim.particle2)
    scObject.addStateEffector(scSim.particle3)

    # define hub properties
    scObject.hub.mHub = 1500  # kg
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # kg*m^2
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # rad
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]  # rad/s

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
    oe.a = 6700.0 * 1000
    oe.e = 0.01
    oe.omega = 100.0 * macros.D2R
    oe.Omega = 100.0 * macros.D2R
    oe.i = 30.0 * macros.D2R
    oe.f = 0.0
    # convert them in position and velocity
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # attach the state to the spacecraft
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m   - r_BN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(P / 4)

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
    #
    scSim.InitializeSimulationAndDiscover()

    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", simulationTimeStep, 0, 0, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", simulationTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", simulationTimeStep, 0, 2, 'double')
    scSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", simulationTimeStep, 0, 0, 'double')
    if damping_parameter != 0.0:
        scSim.AddVariableForLogging(
            "spacecraftBody.dynManager.getStateObject('linearSpringMassDamperRho1').getState()", simulationTimeStep, 0, 0, 'double')
        scSim.AddVariableForLogging(
            "spacecraftBody.dynManager.getStateObject('linearSpringMassDamperRho2').getState()", simulationTimeStep, 0, 0, 'double')
        scSim.AddVariableForLogging(
            "spacecraftBody.dynManager.getStateObject('linearSpringMassDamperRho3').getState()", simulationTimeStep, 0, 0, 'double')

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # request states to the simulation
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_CN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_CN_N', range(3))

    orbEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = scSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = scSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")

    rhoj1Out = rhoj2Out = rhoj3Out = []
    if damping_parameter != 0.0:
        rhoj1Out = scSim.GetLogVariableData(
            "spacecraftBody.dynManager.getStateObject('linearSpringMassDamperRho1').getState()")
        rhoj2Out = scSim.GetLogVariableData(
            "spacecraftBody.dynManager.getStateObject('linearSpringMassDamperRho2').getState()")
        rhoj3Out = scSim.GetLogVariableData(
            "spacecraftBody.dynManager.getStateObject('linearSpringMassDamperRho3').getState()")

    fileName = os.path.basename(os.path.splitext(__file__)[0])
    if damping_parameter == 0.0 and timeStep == 0.75:
        setupNo = 1
    elif damping_parameter == 0.0 and timeStep == 0.3:
        setupNo = 2
    elif damping_parameter != 0.0 and timeStep == 0.75:
        setupNo = 3
    else:
        print("No standard setup parameters")

    plt.close("all")  # clears out plots from earlier test runs
    fig = plt.figure(1, figsize=(5, 5))

    ax = fig.gca()

    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)

    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = planetRadius * 1e-3 * np.cos(u)
    y = planetRadius * 1e-3 * np.sin(u)

    ax.add_artist(plt.Circle((0, 0), planetRadius / 1000, color='#008800'))
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000)
    # ax.plot(x,y,color='g')
    plt.xlim([-7000, 7000])
    plt.ylim([-7000, 7000])
    plt.xlabel('X (km)')
    plt.ylabel('Y (km)')

    figureList = {}
    pltName = fileName + "Orbit" + str(setupNo)
    figureList[pltName] = plt.figure(1)

    plt.figure(2, figsize=(5, 4))
    plt.plot(orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 1] - orbAngMom_N[0, 1]) / orbAngMom_N[0, 1],
             orbAngMom_N[:, 0] * 1e-9,
             (orbAngMom_N[:, 2] - orbAngMom_N[0, 2]) / orbAngMom_N[0, 2],
             orbAngMom_N[:, 0] * 1e-9, (orbAngMom_N[:, 3] - orbAngMom_N[0, 3]) / orbAngMom_N[0, 3])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Orbital Angular Momentum Variation')
    pltName = fileName + "OAM" + str(setupNo)
    figureList[pltName] = plt.figure(2)

    plt.figure(3, figsize=(5, 4))
    plt.plot(orbEnergy[:, 0] * 1e-9, (orbEnergy[:, 1] - orbEnergy[0, 1]) / orbEnergy[0, 1])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Orbital Energy Variation')
    pltName = fileName + "OE" + str(setupNo)
    figureList[pltName] = plt.figure(3)

    plt.figure(4, figsize=(5, 4))
    plt.plot(rotAngMom_N[:, 0] * 1e-9,
             (rotAngMom_N[:, 1] - rotAngMom_N[0, 1]) / rotAngMom_N[0, 1],
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 2] - rotAngMom_N[0, 2]) / rotAngMom_N[0, 2],
             rotAngMom_N[:, 0] * 1e-9, (rotAngMom_N[:, 3] - rotAngMom_N[0, 3]) / rotAngMom_N[0, 3])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Rotational Angular Momentum Variation')
    pltName = fileName + "RAM" + str(setupNo)
    figureList[pltName] = plt.figure(4)

    plt.figure(5, figsize=(5, 4))
    plt.plot(rotEnergy[:, 0] * 1e-9, (rotEnergy[:, 1] - rotEnergy[0, 1]) / rotEnergy[0, 1])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Rotational Energy Variation')
    pltName = fileName + "RE" + str(setupNo)
    figureList[pltName] = plt.figure(5)

    if damping_parameter != 0.0:
        plt.figure(6, figsize=(5, 4))
        plt.plot(rhoj1Out[:, 0] * 1e-9, rhoj1Out[:, 1], rhoj2Out[:, 0] *
                 1e-9, rhoj2Out[:, 1], rhoj3Out[:, 0] * 1e-9, rhoj3Out[:, 1])
        plt.legend(['Particle 1', 'Particle 2', 'Particle 3'])
        plt.xlabel('Time (s)')
        plt.ylabel('Displacement (m)')
        pltName = fileName + "ParticleMotion"
        figureList[pltName] = plt.figure(6)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    return rhoj1Out, rhoj2Out, rhoj3Out, figureList


if __name__ == "__main__":
    run(
        True,               # show_plots
        0.0,				 # damping_parameter
        0.75,				 # timeStep
    )
