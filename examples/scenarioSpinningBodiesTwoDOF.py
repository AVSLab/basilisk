#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario demonstrates the capabilities of :ref:`spinningBodyTwoDOFStateEffector`, which represents an effector with
two spin axis. The two-degree-of-freedom formulation allows the simulation of one rigid body attached to the hub through
a universal joint, or a chain of two rigid bodies each attached through a single hinge. The spin axis and mass
distribution of the rigid bodies are arbitrary.

The scenario can be run with either one or two rigid bodies. The one-panel formulation consists of a cylindrical flat
panel that rotates about two axis in a universal-joint configuration. The panel has similar dimensions to the hub. The
two-panel formulation uses two flat cuboid panels that rotate about perpendicular hinges. The first panel connects
directly to the hub, whereas the second connects to the first.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioSpinningBodiesTwoDOF.py

The scenario outputs two plots: one for the time history of both angles, and another for the time history of both angle
rates. the scenario also creates a comprehensive Vizard simulation which creates appropriate to-scale models for each
simulation type.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, numberPanels = 1

Here, only a single panel is simulated. The panel connects to the hub through a universal, dual-axis joint. The time
history for both angles and angle rates is shown below. Note how decoupled the two angles are. This is because the
module is simulating a universal joint, so each axis behaves independently from each other.

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFtheta1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFthetaDot1.svg
   :align: center

::

    show_plots = True, numberPanels = 2

In this case, two panels are simulated. Each rotates about a one-degree-of-freedom hinge. The spin axis are perpendicular
to each other to show how any spin axis can be chosen. Note that in this case, there is much more significant coupling
between the two angles, with them being in-phase with each other.

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFtheta2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFthetaDot2.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Illustrates the different simulation capabilities of the 2DOF modules
# Author:   João Vaz Carneiro
# Creation Date:  Feb 22, 2023
#

import os
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass, vizSupport, simIncludeGravBody
from Basilisk.simulation import spacecraft, spinningBodyTwoDOFStateEffector
from Basilisk.utilities import macros, orbitalMotion

from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, numberPanels):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        numberPanels (int): Choose how many panels to simulate (1 or 2)

    """

    #
    #  From here on scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.01)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    # Set up the simulation tasks/objects
    #

    # Define the spacecraft's properties
    massSC = 400
    diameter = 2
    height = 4

    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    scObject.hub.mHub = massSC
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[massSC / 16 * diameter ** 2 + massSC / 12 * height ** 2, 0.0, 0.0],
                                [0.0, massSC / 16 * diameter ** 2 + massSC / 12 * height ** 2, 0.0],
                                [0.0, 0.0, massSC / 8 * diameter ** 2]]

    # Set up gravity
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Set the spacecraft's initial conditions
    oe = orbitalMotion.ClassicElements()
    oe.a = 8e6  # meters
    oe.e = 0.1
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.05], [-0.05], [0.05]]

    # Create two hinged rigid bodies
    spinningBody = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody.ModelTag = "SpinningBody"

    # Set up the spinning bodies module
    if numberPanels == 1:
        # Define the properties of the panel (cylinder)
        mass = 50
        radius = diameter
        thickness = 0.1

        # Define the module's properties from the panels
        spinningBody.mass1 = 0.0
        spinningBody.mass2 = mass
        spinningBody.IS1PntSc1_S1 = [[0.0, 0.0, 0.0],
                                     [0.0, 0.0, 0.0],
                                     [0.0, 0.0, 0.0]]
        spinningBody.IS2PntSc2_S2 = [[mass / 12 * (3 * radius ** 2 + thickness ** 2), 0.0, 0.0],
                                     [0.0, mass / 12 * (3 * radius ** 2 + thickness ** 2), 0.0],
                                     [0.0, 0.0, mass / 2 * radius ** 2]]
        spinningBody.dcm_S10B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        spinningBody.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        spinningBody.r_Sc1S1_S1 = [[0.0], [0.0], [0.0]]
        spinningBody.r_Sc2S2_S2 = [[0.0], [radius], [0.0]]
        spinningBody.r_S1B_B = [[0.0], [diameter / 2], [height / 2 - thickness / 2]]
        spinningBody.r_S2S1_S1 = [[0.0], [0.0], [0.0]]
        spinningBody.s1Hat_S1 = [[1], [0], [0]]
        spinningBody.s2Hat_S2 = [[0], [1], [0]]
        spinningBody.k1 = 100.0
        spinningBody.k2 = 100.0
        spinningBody.c1 = 50.0
        spinningBody.c2 = 50.0
        spinningBody.theta1Init = 30 * macros.D2R
        spinningBody.theta2Init = 30 * macros.D2R
        spinningBody.theta1DotInit = 0.0 * macros.D2R
        spinningBody.theta2DotInit = 0.0 * macros.D2R
    elif numberPanels == 2:
        # Define the properties of the panels (rectangular cuboids)
        mass = 20
        length = 2 * diameter
        width = diameter
        thickness = 0.1

        # Define the module's properties from the panels
        spinningBody.mass1 = mass
        spinningBody.mass2 = mass
        spinningBody.IS1PntSc1_S1 = [[mass / 12 * (length ** 2 + thickness ** 2), 0.0, 0.0],
                                     [0.0, mass / 12 * (thickness ** 2 + width ** 2), 0.0],
                                     [0.0, 0.0, mass / 12 * (length ** 2 + width ** 2)]]
        spinningBody.IS2PntSc2_S2 = [[mass / 12 * (length ** 2 + thickness ** 2), 0.0, 0.0],
                                     [0.0, mass / 12 * (thickness ** 2 + width ** 2), 0.0],
                                     [0.0, 0.0, mass / 12 * (length ** 2 + width ** 2)]]
        spinningBody.dcm_S10B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        spinningBody.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        spinningBody.r_Sc1S1_S1 = [[0.0], [length / 2], [0.0]]
        spinningBody.r_Sc2S2_S2 = [[-width / 2], [0.0], [0.0]]
        spinningBody.r_S1B_B = [[0.0], [diameter / 2], [height / 2 - thickness / 2]]
        spinningBody.r_S2S1_S1 = [[- width / 2], [length / 2], [0.0]]
        spinningBody.s1Hat_S1 = [[1], [0], [0]]
        spinningBody.s2Hat_S2 = [[0], [1], [0]]
        spinningBody.k1 = 50.0
        spinningBody.k2 = 50.0
        spinningBody.c1 = 30.0
        spinningBody.c2 = 30.0
        spinningBody.theta1Init = 60.0 * macros.D2R
        spinningBody.theta2Init = 60.0 * macros.D2R
        spinningBody.theta1DotInit = 0.0 * macros.D2R
        spinningBody.theta2DotInit = 0.0 * macros.D2R
    else:
        print("Cannot simulate " + str(numberPanels) + " panels.")
        return

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBody)

    # Add Earth gravity to the simulation
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'sun'])
    gravBodies['earth'].isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    timeInitString = "2012 MAY 1 00:28:30.0"
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'earth'

    # Add modules to simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, spinningBody)
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)

    # Set up the message recorders and add them to the task
    datLog = scObject.scStateOutMsg.recorder()
    theta1Data = spinningBody.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBody.spinningBodyOutMsgs[1].recorder()
    scSim.AddModelToTask(simTaskName, datLog)
    scSim.AddModelToTask(simTaskName, theta1Data)
    scSim.AddModelToTask(simTaskName, theta2Data)

    #
    # Set up Vizard visualization
    #
    scBodyList = [scObject]
    if numberPanels == 1:
        scBodyList.append(["panel", spinningBody.spinningBodyConfigLogOutMsgs[1]])
    else:
        scBodyList.append(["panel1", spinningBody.spinningBodyConfigLogOutMsgs[0]])
        scBodyList.append(["panel2", spinningBody.spinningBodyConfigLogOutMsgs[1]])

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scBodyList
                                              # , saveFile=fileName + str(numberPanels)
                                              )

    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=[scObject.ModelTag]
                                 , modelPath="CYLINDER"
                                 , scale=[diameter, diameter, height / 2]
                                 , color=vizSupport.toRGBA255("blue"))
    if numberPanels == 1:
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["panel"]
                                     , modelPath="CYLINDER"
                                     , scale=[2 * radius, 2 * radius, thickness]
                                     , color=vizSupport.toRGBA255("green"))
    elif numberPanels == 2:
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["panel1"]
                                     , modelPath="CUBE"
                                     , scale=[width, length, thickness]
                                     , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["panel2"]
                                     , modelPath="CUBE"
                                     , scale=[width, length, thickness]
                                     , color=vizSupport.toRGBA255("green"))
    viz.settings.orbitLinesOn = -1

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Configure a simulation stop time and execute the simulation run
    simulationTime = macros.min2nano(1)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    scSim.ShowExecutionOrder()

    # Retrieve the logged data
    theta1 = theta1Data.theta
    theta1Dot = theta1Data.thetaDot
    theta2 = theta2Data.theta
    theta2Dot = theta2Data.thetaDot

    #
    #   plot the results
    #
    figureList = {}

    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    plt.clf()
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta1, label=r'$\theta_1$')
    plt.plot(theta2Data.times() * macros.NANO2SEC, macros.R2D * theta2, label=r'$\theta_2$')
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'$\theta$ [deg]')
    pltName = fileName + "theta" + str(int(numberPanels))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    plt.clf()
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta1Dot, label=r'$\dot{\theta}_1$')
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta2Dot, label=r'$\dot{\theta}_2$')
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]')
    pltName = fileName + "thetaDot" + str(int(numberPanels))
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        2,  # numberPanels
    )
