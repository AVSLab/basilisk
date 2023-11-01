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

This scenario demonstrates the capabilities of :ref:`spinningBodyNDOFStateEffector`, which represents an effector with
n spin axes. The n-degree-of-freedom formulation allows the simulation of any number of attached rigid bodies with
rotation capable about any axis. The spin axes and mass distribution of the rigid bodies are arbitrary.

The scenario consists of a flat panel and a system of two cylindrical arms attached on opposite sides of the hub. The
panel is free to rotate about one axis and has similar dimensions to the hub. The arms both rotate about two axes in a
universal-joint configuration with one being attached to the hub and the other to the end of the first. One more body is
attached to the end of the arms that is free to rotate along the arm's vertical axis.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioSpinningBodiesNDOF.py

The scenario outputs two plots: one for the time history of all the angles, and another for the time history of the angle
rates. the scenario also creates a comprehensive Vizard simulation which creates an appropriate to-scale model for the
defined scenario.

Illustration of Simulation Results
----------------------------------

::

The single panel connects to the hub through a single axis of rotation while both of the arms connect through a dual
axis joint. The panel attached to the end of the arms adds one more axis of rotation. Note that this angle settles
relatively quickly since it is independent of every other axis.

.. image:: /_images/Scenarios/scenarioSpinningBodiesNDOFtheta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesNDOFthetaDot.svg
   :align: center

::



"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Illustrates the different simulation capabilities of the NDOF modules
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

    # Create first spinning body instance
    spinningBodyEffector1 = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector1.ModelTag = "spinningBodyEffector1"

    # Define spinning body dimensions
    heightArms = 10.0
    diameterArms = 1.0
    lengthPanel2 = 5.0
    widthPanel2 = 1.0
    thicknessPanel2 = 1.0

    # Define properties of spinning bodies

    # bodies 1 and 2 define the rotation of the first arm
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
    spinningBodyEffector1.addSpinningBody(spinningBody1)

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
    spinningBodyEffector1.addSpinningBody(spinningBody2)

    # bodies 3 and 4 establish the rotation of the second arm
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
    spinningBodyEffector1.addSpinningBody(spinningBody3)

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
    spinningBodyEffector1.addSpinningBody(spinningBody4)

    # rotating panel at the end of the two arms
    spinningBody5 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody5.mass = 10.0
    spinningBody5.ISPntSc_S = [[spinningBody5.mass / 12 * (lengthPanel2 ** 2 + thicknessPanel2 ** 2), 0.0, 0.0],
                               [0.0, spinningBody5.mass / 12 * (widthPanel2 ** 2 + thicknessPanel2 ** 2), 0.0],
                               [0.0, 0.0, spinningBody5.mass / 12 * (widthPanel2 ** 2 + lengthPanel2 ** 2)]]
    spinningBody5.dcm_S0P = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody5.r_ScS_S = [[0.0], [lengthPanel2 / 2], [0.0]]
    spinningBody5.r_SP_P = [[0.0], [heightArms], [0.0]]
    spinningBody5.sHat_S = [[0], [1], [0]]
    spinningBody5.thetaInit = 45 * macros.D2R
    spinningBody5.thetaDotInit = 60 * macros.D2R
    spinningBody5.k = 100.0
    spinningBody5.c = 1.0
    spinningBodyEffector1.addSpinningBody(spinningBody5)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBodyEffector1)

    # create second spinning body instance
    spinningBodyEffector2 = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector2.ModelTag = "spinningBodyEffector2"

    # Define spinning body dimensions
    lengthPanel1 = 10.0
    widthPanel1 = 5.0
    thicknessPanel1= 0.1

    # extra panel attached to opposite side of spacecraft
    spinningBody6 = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody6.mass = 50.0
    spinningBody6.ISPntSc_S = [[spinningBody5.mass / 12 * (lengthPanel1 ** 2 + thicknessPanel1 ** 2), 0.0, 0.0],
                               [0.0, spinningBody5.mass / 12 * (widthPanel1 ** 2 + thicknessPanel1 ** 2), 0.0],
                               [0.0, 0.0, spinningBody5.mass / 12 * (widthPanel1 ** 2 + lengthPanel1 ** 2)]]
    spinningBody6.dcm_S0P = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody6.r_ScS_S = [[0.0], [-lengthPanel1 / 2], [0.0]]
    spinningBody6.r_SP_P = [[0], [-lengthSC / 2], [heightSC / 2 - thicknessPanel1 / 2]]
    spinningBody6.sHat_S = [[1], [0], [0]]
    spinningBody6.thetaInit = 60 * macros.D2R
    spinningBody6.thetaDotInit = 0 * macros.D2R
    spinningBody6.k = 100.0
    spinningBody6.c = 50.0
    spinningBodyEffector2.addSpinningBody(spinningBody6)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBodyEffector2)

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
    scSim.AddModelToTask(simTaskName, spinningBodyEffector1)
    scSim.AddModelToTask(simTaskName, spinningBodyEffector2)
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)

    # Set up the message recorders and add them to the task
    datLog = scObject.scStateOutMsg.recorder()
    theta1Data = spinningBodyEffector1.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBodyEffector1.spinningBodyOutMsgs[1].recorder()
    theta3Data = spinningBodyEffector1.spinningBodyOutMsgs[2].recorder()
    theta4Data = spinningBodyEffector1.spinningBodyOutMsgs[3].recorder()
    theta5Data = spinningBodyEffector1.spinningBodyOutMsgs[4].recorder()
    theta6Data = spinningBodyEffector2.spinningBodyOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, datLog)
    scSim.AddModelToTask(simTaskName, theta1Data)
    scSim.AddModelToTask(simTaskName, theta2Data)
    scSim.AddModelToTask(simTaskName, theta3Data)
    scSim.AddModelToTask(simTaskName, theta4Data)
    scSim.AddModelToTask(simTaskName, theta5Data)
    scSim.AddModelToTask(simTaskName, theta6Data)

    #
    # Set up Vizard visualization
    #

    scBodyList = [scObject,
                  ["arm1", spinningBodyEffector1.spinningBodyConfigLogOutMsgs[1]],
                  ["arm2", spinningBodyEffector1.spinningBodyConfigLogOutMsgs[3]],
                  ["panel1", spinningBodyEffector2.spinningBodyConfigLogOutMsgs[0]],
                  ["panel2", spinningBodyEffector1.spinningBodyConfigLogOutMsgs[4]]]


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
                                 , simBodiesToModify=["panel1"]
                                 , modelPath="CUBE"
                                 , scale=[widthPanel1, lengthPanel1, thicknessPanel1]
                                 , color=vizSupport.toRGBA255("red"))
    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=["panel2"]
                                 , modelPath="CUBE"
                                 , scale=[widthPanel2, lengthPanel2, thicknessPanel2]
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
    theta6 = theta6Data.theta
    theta6Dot = theta6Data.thetaDot

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
    plt.plot(theta6Data.times() * macros.NANO2SEC, macros.R2D * theta6, label=r'$\theta_6$')
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
    plt.plot(theta6Data.times() * macros.NANO2SEC, macros.R2D * theta6Dot, label=r'$\dot{\theta}_6$')
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
