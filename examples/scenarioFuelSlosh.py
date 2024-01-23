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

Demonstration of basic 6-DOF orbit and fuel slosh simulation setup.
This script sets up a 6-DOF spacecraft orbiting a planet.  The purpose
is to illustrate how to create a spacecraft, attach a gravity model and a fuel tank, and run
a basic Basilisk simulation.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioFuelSlosh.py

Simulation Scenario Initial Setup
---------------------------------
The simulation layout is shown in the following illustration.  A single simulation
process, containing the spacecraft object,
is created. Gravity and the Fuel Tank effectors are attached to the spacecraft dynamics to
simulate the desired scenario.

.. image:: /_images/static/test_scenarioFuelSlosh.svg
   :align: center

When the simulation completes 5 plots are shown for each case.  One plot
shows the spacecraft trajectory in the orbital plane.
The second and third plots show the relative variation of orbital angular momentum and energy,
respectively. The fourth and fifth plots show the relative changes in
rotational angular momentum and energy. The third case shows
a sixth plot, representing the fuel slosh particle motion.

State Effectors Setup
---------------------
The model used to simulate the fuel slosh is a classic mass spring
damper system coupled with the rest of the spacecraft.
The fuel slosh particle is added to the simulation using the
:ref:`LinearSpringMassDamper` module.

The fuel slosh partile :math:`k` is the spring constant in kg/s^2 (N/m),
:math:`c` is the damping coefficient expressed in kg/s.

As we can see from the following illustrations, ``r_PB_B`` is the vector
that expresses the particle equilibrium position
in the body reference frame. ``pHat_B`` is the direction of particle
motion, expressed in the body reference frame.
``rhoInit`` and ``rhoDotInit`` are the initial particle position and velocity,
expressed in m and m/s respectively. ``massInit`` is
fuel mass that is moving in the selected direction.

.. image:: /_images/static/test_scenarioFuelSloshSpacecraft.svg
   :align: center

For further information on the model implemented you can consult this
`conference paper <http://hanspeterschaub.info/Papers/Allard2016a.pdf>`__.

Next the :ref:`FuelTank` module is used to initialize and define the
tank properties and the particles are introduced in it.
The fuel tank is represented by a constant volume sphere. The radius is set
to 0.5 m using the radiusTankInit variable.
propMassInit is the initial propellant mass that does not generate
slosh. ``r_TcT_TInit`` is the initial position vector
# from B to tank point in B frame components. ``r_TB_B`` is the
position of the tank in body frame.

The steps to add gravity objects are the same shown in the
:ref:`scenarioBasicOrbit`.

Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True, damping_parameter = 0.0, timeStep = 0.75

.. image:: /_images/Scenarios/scenarioFuelSloshOAM1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshOE1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshRAM1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshRE1.svg
   :align: center

Here we should see conservation with machine precision in every plot,
because we are not considering dissipations.
In this case, we do not see a perfect conservation of Rotational
Angular Momentum and Rotational Energy because
of the **high** timeStep. We cannot use too high timeStep because
we will lose the periodic behaviour of the fuel slosh;
and the integration could diverge. On the other hand, small
timeStep makes the simulation very long.

::

    show_plots = True, damping_parameter = 0.0, timeStep = 0.30

This case illustrates, how reducing the simulation time step,
the solution is close to machine precision.
This highlights the conservative nature of the forces used so far,
confirming that the simulation is running correctly.


.. image:: /_images/Scenarios/scenarioFuelSloshOAM2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshOE2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshRAM2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshRE2.svg
   :align: center

::

   show_plots = True, damping_parameter = 15.0, timeStep = 0.75

This case illustrates that considering damping we have Rotational
Energy dissipation due to fuel slosh.
It is interesting to note that the Rotational Angular Momentum shows
a more precise conservation compared to
the first case, for equal timeStep.
This happens because the damping reduces the particles motion, so the
integrator becomes more stable.
Furthermore, from the last image it is possible to see how the motion
is similar to a mass spring damper system;
but it is slightly different because the motion is coupled with the
rest of the spacecraft.

.. image:: /_images/Scenarios/scenarioFuelSloshParticleMotion.svg
  :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshOAM3.svg
  :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshOE3.svg
  :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshRAM3.svg
  :align: center

.. image:: /_images/Scenarios/scenarioFuelSloshRE3.svg
  :align: center

"""



import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.simulation import fuelTank
from Basilisk.simulation import linearSpringMassDamper
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import pythonVariableLogger

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


def run(show_plots, damping_parameter, timeStep):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
        damping_parameter (float): Hinge damping coefficient
        timeStep (float): Integration time step

    """

    simTaskName = "simTask"
    simProcessName = "simProcess"
    # create simulation
    scSim = SimulationBaseClass.SimBaseClass()
    # close possible other simulation
    # crete a dynamical process
    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(timeStep)
    # add task to the dynamical process
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #  create spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"

    scSim.AddModelToTask(simTaskName, scObject)

    # Particle 1
    scSim.particle1 = linearSpringMassDamper.LinearSpringMassDamper()

    # Define Variables for particle 1
    scSim.particle1.k = 1.0  # kg/s^2 (N/m)
    scSim.particle1.c = damping_parameter  # kg/s
    scSim.particle1.r_PB_B = [[0.1], [0], [-0.1]]  # m
    scSim.particle1.pHat_B = [[1], [0], [0]]
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
    gravFactory.addBodiesTo(scObject)

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
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(P / 4)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    scLog = scObject.logger(
        ["totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"], 
        simulationTimeStep)
    scSim.AddModelToTask(simTaskName, scLog)

    damperRhoLog = None
    if damping_parameter != 0.0:
        def get_rho(CurrentSimNanos, i):
            stateName = f'linearSpringMassDamperRho{i}'
            return scObject.dynManager.getStateObject(stateName).getState()[0][0]
        
        damperRhoLog = pythonVariableLogger.PythonVariableLogger({
                "damperRho1": lambda CurrentSimNanos: get_rho(CurrentSimNanos, 1),
                "damperRho2": lambda CurrentSimNanos: get_rho(CurrentSimNanos, 2),
                "damperRho3": lambda CurrentSimNanos: get_rho(CurrentSimNanos, 3),
            }, simulationTimeStep)
        
        scSim.AddModelToTask(simTaskName, damperRhoLog)

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.InitializeSimulation()

    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # request states to the simulation
    posData = dataLog.r_CN_N
    velData = dataLog.v_CN_N

    time = scLog.times() * 1e-9
    orbEnergy = scLog.totOrbEnergy
    orbAngMom_N = scLog.totOrbAngMomPntN_N
    rotAngMom_N = scLog.totRotAngMomPntC_N
    rotEnergy = scLog.totRotEnergy

    rhojOuts = None
    if damping_parameter != 0.0:
        rhojOuts = [
            damperRhoLog.damperRho1, damperRhoLog.damperRho2, damperRhoLog.damperRho3]

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
    for idx in range(len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
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
    for i in range(3):
        plt.plot(time, (orbAngMom_N[:, i] - orbAngMom_N[0, i]) / orbAngMom_N[0, i])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Orbital Angular Momentum Variation')
    pltName = fileName + "OAM" + str(setupNo)
    figureList[pltName] = plt.figure(2)

    plt.figure(3, figsize=(5, 4))
    plt.plot(time, (orbEnergy - orbEnergy[0]) / orbEnergy[0])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Orbital Energy Variation')
    pltName = fileName + "OE" + str(setupNo)
    figureList[pltName] = plt.figure(3)

    plt.figure(4, figsize=(5, 4))
    for i in range(3):
        plt.plot(time, (rotAngMom_N[:, i] - rotAngMom_N[0, i]) / rotAngMom_N[0, i])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Rotational Angular Momentum Variation')
    pltName = fileName + "RAM" + str(setupNo)
    figureList[pltName] = plt.figure(4)

    plt.figure(5, figsize=(5, 4))
    plt.plot(time, (rotEnergy - rotEnergy[0]) / rotEnergy[0])
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Rotational Energy Variation')
    pltName = fileName + "RE" + str(setupNo)
    figureList[pltName] = plt.figure(5)

    if damping_parameter != 0.0:
        plt.figure(6, figsize=(5, 4))
        for rhojOut in rhojOuts:
            plt.plot(time, rhojOut)
        
        plt.legend(['Particle 1', 'Particle 2', 'Particle 3'], loc='lower right')
        plt.xlabel('Time (s)')
        plt.ylabel('Displacement (m)')
        pltName = fileName + "ParticleMotion"
        figureList[pltName] = plt.figure(6)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    return time, rhojOuts, figureList


if __name__ == "__main__":
    run(
        True,               # show_plots
        0.0,				 # damping_parameter
        0.75,				 # timeStep
    )
