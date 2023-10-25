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
# Author:   Peter Johnson
# Creation Date:  October 23, 2023
#

import os
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import SimulationBaseClass, vizSupport, simIncludeGravBody
from Basilisk.simulation import spacecraft, spinningBodyNDOFStateEffector
from Basilisk.utilities import macros, orbitalMotion

from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

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
    lengthSC = 5
    widthSC = 5
    heightSC = 4

    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    scObject.hub.mHub = massSC
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[massSC / 12 * (lengthSC ** 2 + heightSC ** 2), 0.0, 0.0],
                                [0.0, massSC / 12 * (widthSC ** 2 + heightSC ** 2), 0.0],
                                [0.0, 0.0, massSC / 12 * (lengthSC ** 2 + widthSC ** 2)]]

    # Set up gravity
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Set the spacecraft's initial conditions (didnt change any of this)
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
    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector.ModelTag = "spinningBodyEffector"

    # Define spinning body dimensions
    heightArms = 10.0
    diameterArms = 1.0
    lengthPanel = 5.0
    widthPanel = 1.0
    thicknessPanel = 1.0

    # Define properties of spinning bodies
    spinningBody1 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody1.mass = 0
    spinningBody1.ISPntSc_S = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    spinningBody1.dcm_S0P = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody1.r_ScS_S = [[0.0], [heightArms / 2], [0.0]]
    spinningBody1.r_SP_P = [[0], [lengthSC / 2], [heightSC / 2 - diameterArms / 2]]
    spinningBody1.sHat_S = [[1], [0], [0]]
    spinningBody1.thetaInit = 60 * macros.D2R
    spinningBody1.thetaDotInit = 0 * macros.D2R
    spinningBody1.k = 100.0
    spinningBody1.c = 50.0
    spinningBodyEffector.addSpinningBody(spinningBody1)

    spinningBody2 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody2.mass = 50.0
    spinningBody2.ISPntSc_S = [
        [spinningBody2.mass / 16 * diameterArms ** 2 + spinningBody2.mass / 12 * heightArms ** 2, 0.0, 0.0],
        [0.0, spinningBody2.mass / 8 * diameterArms ** 2, 0.0],
        [0.0, 0.0, spinningBody2.mass / 16 * diameterArms ** 2 + spinningBody2.mass / 12 * heightArms ** 2]]
    spinningBody2.dcm_S0P = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody2.r_ScS_S = [[0.0], [heightArms / 2], [0.0]]
    spinningBody2.r_SP_P = [[0], [0], [0]]
    spinningBody2.sHat_S = [[0], [0], [1]]
    spinningBody2.thetaInit = 60 * macros.D2R
    spinningBody2.thetaDotInit = 0 * macros.D2R
    spinningBody2.k = 100.0
    spinningBody2.c = 50.0
    spinningBodyEffector.addSpinningBody(spinningBody2)

    spinningBody3 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody3.mass = 0
    spinningBody3.ISPntSc_S = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    spinningBody3.dcm_S0P = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody3.r_ScS_S = [[0.0], [heightArms / 2], [0.0]]
    spinningBody3.r_SP_P = [[0.0], [heightArms], [0.0]]
    spinningBody3.sHat_S = [[1], [0], [0]]
    spinningBody3.thetaInit = 30 * macros.D2R
    spinningBody3.thetaDotInit = 0 * macros.D2R
    spinningBody3.k = 100.0
    spinningBody3.c = 50.0
    spinningBodyEffector.addSpinningBody(spinningBody3)

    spinningBody4 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody4.mass = 50.0
    spinningBody4.ISPntSc_S = [
        [spinningBody2.mass / 16 * diameterArms ** 2 + spinningBody2.mass / 12 * heightArms ** 2, 0.0, 0.0],
        [0.0, spinningBody2.mass / 8 * diameterArms ** 2, 0.0],
        [0.0, 0.0, spinningBody2.mass / 16 * diameterArms ** 2 + spinningBody2.mass / 12 * heightArms ** 2]]
    spinningBody4.dcm_S0P = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody4.r_ScS_S = [[0.0], [heightArms / 2], [0.0]]
    spinningBody4.r_SP_P = [[0.0], [0.0], [0.0]]
    spinningBody4.sHat_S = [[0], [0], [1]]
    spinningBody4.thetaInit = 30 * macros.D2R
    spinningBody4.thetaDotInit = 0 * macros.D2R
    spinningBody4.k = 100.0
    spinningBody4.c = 50.0
    spinningBodyEffector.addSpinningBody(spinningBody4)

    spinningBody5 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody5.mass = 10.0
    spinningBody5.ISPntSc_S = [[spinningBody5.mass / 12 * (lengthPanel ** 2 + thicknessPanel ** 2), 0.0, 0.0],
                               [0.0, spinningBody5.mass / 12 * (widthPanel ** 2 + thicknessPanel ** 2), 0.0],
                               [0.0, 0.0, spinningBody5.mass / 12 * (widthPanel ** 2 + lengthPanel ** 2)]]
    spinningBody5.dcm_S0P = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody5.r_ScS_S = [[0.0], [lengthPanel / 2], [0.0]]
    spinningBody5.r_SP_P = [[0.0], [heightArms], [0.0]]
    spinningBody5.sHat_S = [[0], [1], [0]]
    spinningBody5.thetaInit = 45 * macros.D2R
    spinningBody5.thetaDotInit = 60 * macros.D2R
    spinningBody5.k = 100.0
    spinningBody5.c = 1.0
    spinningBodyEffector.addSpinningBody(spinningBody5)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBodyEffector)

    # Add Earth gravity to the simulation
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'sun'])
    gravBodies['earth'].isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    timeInitString = "2012 MAY 1 00:28:30.0"
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'earth'

    # Add modules to simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, spinningBodyEffector)
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)

    # Set up the message recorders and add them to the task
    datLog = scObject.scStateOutMsg.recorder()
    theta1Data = spinningBodyEffector.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBodyEffector.spinningBodyOutMsgs[1].recorder()
    theta3Data = spinningBodyEffector.spinningBodyOutMsgs[2].recorder()
    theta4Data = spinningBodyEffector.spinningBodyOutMsgs[3].recorder()
    theta5Data = spinningBodyEffector.spinningBodyOutMsgs[4].recorder()
    scSim.AddModelToTask(simTaskName, datLog)
    scSim.AddModelToTask(simTaskName, theta1Data)
    scSim.AddModelToTask(simTaskName, theta2Data)
    scSim.AddModelToTask(simTaskName, theta3Data)
    scSim.AddModelToTask(simTaskName, theta4Data)
    scSim.AddModelToTask(simTaskName, theta5Data)

    #
    # Set up Vizard visualization
    #

    scBodyList = [scObject,
                  ["arm1", spinningBodyEffector.spinningBodyConfigLogOutMsgs[1]],
                  ["arm2", spinningBodyEffector.spinningBodyConfigLogOutMsgs[3]],
                  ["panel", spinningBodyEffector.spinningBodyConfigLogOutMsgs[4]]]

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scBodyList
                                              , saveFile=fileName
                                              )

    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=[scObject.ModelTag]
                                 , modelPath="CUBE"
                                 , scale=[widthSC, lengthSC, heightSC]
                                 , color=vizSupport.toRGBA255("blue"))
    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=["arm1"]
                                 , modelPath="CYLINDER"
                                 , scale=[diameterArms, diameterArms, heightArms / 2]
                                 , rotation=[np.pi / 2, 0, 0]
                                 , color=vizSupport.toRGBA255("green"))
    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=["arm2"]
                                 , modelPath="CYLINDER"
                                 , scale=[diameterArms, diameterArms, heightArms / 2]
                                 , rotation=[np.pi / 2, 0, 0]
                                 , color=vizSupport.toRGBA255("yellow"))
    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=["panel"]
                                 , modelPath="CUBE"
                                 , scale=[widthPanel, lengthPanel, thicknessPanel]
                                 , color=vizSupport.toRGBA255("red"))
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
    theta3 = theta3Data.theta
    theta3Dot = theta3Data.thetaDot
    theta4 = theta4Data.theta
    theta4Dot = theta4Data.thetaDot
    theta5 = theta5Data.theta
    theta5Dot = theta5Data.thetaDot

    #
    #   plot the results
    #
    figureList = {}

    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    plt.clf()
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta1, label=r'$\theta_1$')
    plt.plot(theta2Data.times() * macros.NANO2SEC, macros.R2D * theta2, label=r'$\theta_2$')
    plt.plot(theta3Data.times() * macros.NANO2SEC, macros.R2D * theta3, label=r'$\theta_3$')
    plt.plot(theta4Data.times() * macros.NANO2SEC, macros.R2D * theta4, label=r'$\theta_4$')
    plt.plot(theta5Data.times() * macros.NANO2SEC, macros.R2D * theta5, label=r'$\theta_5$')
    plt.title = 'Angles'
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'$\theta$ [deg]')
    pltName = fileName + "theta"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    plt.clf()
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta1Dot, label=r'$\dot{\theta}_1$')
    plt.plot(theta2Data.times() * macros.NANO2SEC, macros.R2D * theta2Dot, label=r'$\dot{\theta}_2$')
    plt.plot(theta3Data.times() * macros.NANO2SEC, macros.R2D * theta3Dot, label=r'$\dot{\theta}_3$')
    plt.plot(theta4Data.times() * macros.NANO2SEC, macros.R2D * theta4Dot, label=r'$\dot{\theta}_4$')
    plt.plot(theta5Data.times() * macros.NANO2SEC, macros.R2D * theta5Dot, label=r'$\dot{\theta}_5$')
    plt.title = 'Angle Rates'
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]')
    pltName = fileName + "thetaDot"
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
    )
