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
module is simulating a universal joint, so each axis behaves independently from each other. The impact of the motion of
the panel is also shown in the velocity and angular velocity plots. The initial transient in the time history of these
two quantities matches the motion of the panel.

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFtheta1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFthetaDot1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFvelocity1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFangularVelocity1.svg
   :align: center

::

    show_plots = True, numberPanels = 2

In this case, two panels are simulated. Each rotates about a one-degree-of-freedom hinge. The spin axis are perpendicular
to each other to show how any spin axis can be chosen. Note that in this case, there is much more significant coupling
between the two angles, with them being in-phase with each other. As before, the transient motion of the velocity and
angular velocity states match the motion of the panels.

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFtheta2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFthetaDot2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFvelocity2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSpinningBodiesTwoDOFangularVelocity2.svg
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
import numpy as np

from Basilisk.utilities import SimulationBaseClass, vizSupport, simIncludeGravBody
from Basilisk.simulation import spacecraft, spinningBodyTwoDOFStateEffector, thrusterDynamicEffector, gravityEffector, linearTranslationOneDOFStateEffector, spinningBodyOneDOFStateEffector
from Basilisk.utilities import macros, orbitalMotion, unitTestSupport, simIncludeThruster
from Basilisk.architecture import messaging

from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

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

    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # [meters^3/s^2]
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    oe = orbitalMotion.ClassicElements()
    oe.a = earthGravBody.radEquator + 7500e3  # meters
    oe.e = 0.01
    oe.i = 30.0 * macros.D2R
    oe.Omega = 60.0 * macros.D2R
    oe.omega = 15.0 * macros.D2R
    oe.f = 90.0 * macros.D2R
    r_CN, rDot_CN = orbitalMotion.elem2rv(earthGravBody.mu, oe)

    # Set the spacecraft's initial conditions
    scObject.hub.r_CN_NInit = r_CN
    scObject.hub.v_CN_NInit = rDot_CN
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.005], [-0.005], [0.005]]

    # Create two hinged rigid bodies
    spinningBody_a = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody_a.ModelTag = "SpinningBody_a"
    spinningBody_b = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody_b.ModelTag = "SpinningBody_b"

    # Set up the spinning bodies modules
    # Define the properties of the panels (rectangular cuboids)
    mass = 20
    length = 1.5 * diameter
    width = 0.1
    thickness = 0.1

    # Define the module's properties from the panels
    spinningBody_a.mass1 = mass
    spinningBody_a.mass2 = mass
    spinningBody_a.IS1PntSc1_S1 = [[mass / 12 * (length ** 2 + thickness ** 2), 0.0, 0.0],
                                    [0.0, mass / 12 * (thickness ** 2 + width ** 2), 0.0],
                                    [0.0, 0.0, mass / 12 * (length ** 2 + width ** 2)]]
    spinningBody_a.IS2PntSc2_S2 = [[mass / 12 * (length ** 2 + thickness ** 2), 0.0, 0.0],
                                    [0.0, mass / 12 * (thickness ** 2 + width ** 2), 0.0],
                                    [0.0, 0.0, mass / 12 * (length ** 2 + width ** 2)]]
    spinningBody_a.dcm_S10B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody_a.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody_a.r_Sc1S1_S1 = [[0.0], [length / 4], [0.0]]
    spinningBody_a.r_Sc2S2_S2 = [[0.0], [length / 4], [0.0]]
    spinningBody_a.r_S1B_B = [[0.0], [diameter / 2], [-height / 2 + thickness / 2]]
    spinningBody_a.r_S2S1_S1 = [[0.0], [length/2], [0.0]]
    spinningBody_a.s1Hat_S1 = [[1], [0], [0]]
    spinningBody_a.s2Hat_S2 = [[1], [0], [0]]
    spinningBody_a.k1 = 50.0
    spinningBody_a.k2 = 50.0
    spinningBody_a.c1 = 30.0
    spinningBody_a.c2 = 30.0
    spinningBody_a.theta1Init = 0.0 * macros.D2R
    spinningBody_a.theta2Init = -30.0 * macros.D2R
    spinningBody_a.theta1DotInit = 0.0 * macros.D2R
    spinningBody_a.theta2DotInit = 0.0 * macros.D2R

    # Define the module's properties from the panels
    spinningBody_b.mass1 = mass
    spinningBody_b.mass2 = mass
    spinningBody_b.IS1PntSc1_S1 = [[mass / 12 * (length ** 2 + thickness ** 2), 0.0, 0.0],
                                    [0.0, mass / 12 * (thickness ** 2 + width ** 2), 0.0],
                                    [0.0, 0.0, mass / 12 * (length ** 2 + width ** 2)]]
    spinningBody_b.IS2PntSc2_S2 = [[mass / 12 * (length ** 2 + thickness ** 2), 0.0, 0.0],
                                    [0.0, mass / 12 * (thickness ** 2 + width ** 2), 0.0],
                                    [0.0, 0.0, mass / 12 * (length ** 2 + width ** 2)]]
    spinningBody_b.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody_b.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody_b.r_Sc1S1_S1 = [[0.0], [length / 4], [0.0]]
    spinningBody_b.r_Sc2S2_S2 = [[0.0], [length / 4], [0.0]]
    spinningBody_b.r_S1B_B = [[0.0], [-diameter / 2], [-height / 2 + thickness / 2]]
    spinningBody_b.r_S2S1_S1 = [[0.0], [length/2], [0.0]]
    spinningBody_b.s1Hat_S1 = [[1], [0], [0]]
    spinningBody_b.s2Hat_S2 = [[1], [0], [0]]
    spinningBody_b.k1 = 50.0
    spinningBody_b.k2 = 50.0
    spinningBody_b.c1 = 30.0
    spinningBody_b.c2 = 30.0
    spinningBody_b.theta1Init = 0.0 * macros.D2R
    spinningBody_b.theta2Init = -30.0 * macros.D2R
    spinningBody_b.theta1DotInit = 0.0 * macros.D2R
    spinningBody_b.theta2DotInit = 0.0 * macros.D2R

    # Create reference angle input messages
    angle1Ref = messaging.HingedRigidBodyMsgPayload()
    angle1Ref.theta = 0.0
    angle1Ref.thetaDot = 0.0
    angle1RefMsg = messaging.HingedRigidBodyMsg().write(angle1Ref)
    spinningBody_a.spinningBodyRefInMsgs[0].subscribeTo(angle1RefMsg)
    spinningBody_b.spinningBodyRefInMsgs[0].subscribeTo(angle1RefMsg)

    angle2Ref = messaging.HingedRigidBodyMsgPayload()
    angle2Ref.theta = -30.0 * macros.D2R
    angle2Ref.thetaDot = 0.0
    angle2RefMsg = messaging.HingedRigidBodyMsg().write(angle2Ref)
    spinningBody_a.spinningBodyRefInMsgs[1].subscribeTo(angle2RefMsg)
    spinningBody_b.spinningBodyRefInMsgs[1].subscribeTo(angle2RefMsg)

    # Add spinning body to spacecraft
    scObject.addStateEffector(spinningBody_a)
    scObject.addStateEffector(spinningBody_b)

    # Use translating body to represent second vehicle
    hub2 = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()
    hub2.ModelTag = "hub2"
    hub2.setMass(scObject.hub.mHub/2)
    hub2.setFHat_B([[-1.0], [0.0], [0.0]])
    hub2.setR_FcF_F([[0.0], [0.0], [-height/4]])
    hub2.setR_F0B_B([[0.0], [0.0], [-height/2]])
    hub2.setIPntFc_F([[50.0, 0.0, 0.0],
                                 [0.0, 80.0, 0.0],
                                 [0.0, 0.0, 60.0]])
    hub2.setDCM_FB([[1.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0],
                               [0.0, 0.0, 1.0]])
    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    hub2.motorLockInMsg.subscribeTo(lockMsg)
    scObject.addStateEffector(hub2)

    # hub2 = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
    # hub2.mass = scObject.hub.mHub/2
    # hub2.IPntSc_S = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    # hub2.dcm_S0B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    # hub2.r_ScS_S = [[0.0], [0.0], [0.0]]
    # hub2.r_SB_B = [[0.0], [0.0], [0.0]]
    # hub2.sHat_S = [[0], [-1], [0]]
    # hub2.thetaInit = 0.0 * macros.D2R
    # hub2.thetaDotInit = 0.0 * macros.D2R
    # hub2.k = 100.0
    # hub2.c = 50
    # hub2.ModelTag = "SpinningBody"

    # Setup thrusters on arms
    thruster_a = thrusterDynamicEffector.ThrusterDynamicEffector()
    thruster_b = thrusterDynamicEffector.ThrusterDynamicEffector()
    thFactory = simIncludeThruster.thrusterFactory()

    pos_B = [0, length/4, 0]
    dir_B = [0, 0, 1]
    thFactory.create('MOOG_Monarc_5', pos_B, dir_B)
    thFactory.addToSpacecraftSubcomponent("dynThruster_a", thruster_a, spinningBody_a, 2, r_PcP_P=spinningBody_a.r_Sc2S2_S2)
    thFactory.addToSpacecraftSubcomponent("dynThruster_b", thruster_b, spinningBody_b, 2, r_PcP_P=spinningBody_b.r_Sc2S2_S2)

    thr_msg = messaging.THRArrayOnTimeCmdMsgPayload()
    thr_msg.OnTimeRequest = [30, 30]
    thr_cmd_msgs = messaging.THRArrayOnTimeCmdMsg().write(thr_msg)
    thruster_a.cmdsInMsg.subscribeTo(thr_cmd_msgs)
    thruster_b.cmdsInMsg.subscribeTo(thr_cmd_msgs)

    # Add modules to simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, spinningBody_a)
    scSim.AddModelToTask(simTaskName, spinningBody_b)
    scSim.AddModelToTask(simTaskName, thruster_a)
    scSim.AddModelToTask(simTaskName, thruster_b)
    scSim.AddModelToTask(simTaskName, hub2)

    # Set up the message recorders and add them to the task
    datLog = scObject.scStateOutMsg.recorder()
    theta1Data = spinningBody_a.spinningBodyOutMsgs[0].recorder()
    theta2Data = spinningBody_a.spinningBodyOutMsgs[1].recorder()
    scStateData = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, datLog)
    scSim.AddModelToTask(simTaskName, theta1Data)
    scSim.AddModelToTask(simTaskName, theta2Data)
    scSim.AddModelToTask(simTaskName, scStateData)

    #
    # Set up Vizard visualization
    #
    scBodyList = [scObject]
    scBodyList.append(["panela1", spinningBody_a.spinningBodyConfigLogOutMsgs[0]])
    scBodyList.append(["panela2", spinningBody_a.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["panelb1", spinningBody_b.spinningBodyConfigLogOutMsgs[0]])
    scBodyList.append(["panelb2", spinningBody_b.spinningBodyConfigLogOutMsgs[1]])
    scBodyList.append(["hub2", hub2.translatingBodyConfigLogOutMsg])

    thrList = [None] * 6
    thrList[2] = [thruster_a]
    thrList[4] = [thruster_b]

    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scBodyList, thrEffectorList=thrList
                                                  , saveFile=fileName
                                                  )

        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject.ModelTag]
                                     , modelPath="CYLINDER"
                                     , scale=[diameter, diameter, height / 2]
                                     , color=vizSupport.toRGBA255("blue"))
        vizSupport.createCustomModel(viz
                                        , simBodiesToModify=["panela1"]
                                        , modelPath="CUBE"
                                        , scale=[width, length/2, thickness]
                                        , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                        , simBodiesToModify=["panela2"]
                                        , modelPath="CUBE"
                                        , scale=[width, length/2, thickness]
                                        , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                        , simBodiesToModify=["panelb1"]
                                        , modelPath="CUBE"
                                        , scale=[width, length/2, thickness]
                                        , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                        , simBodiesToModify=["panelb2"]
                                        , modelPath="CUBE"
                                        , scale=[width, length/2, thickness]
                                        , color=vizSupport.toRGBA255("green"))
        vizSupport.createCustomModel(viz
                                        , simBodiesToModify=["hub2"]
                                        , modelPath="CYLINDER"
                                        , scale=[diameter/2, diameter/2, height/4]
                                        , color=vizSupport.toRGBA255("red"))

        # scData = vizInterface.VizSpacecraftData()
        # scData.spacecraftName = scNames[c]
        # scData.scStateInMsg.subscribeTo(testModule.scStateOutMsgs[c])

        # thrList = []
        # thrInfo = []
        # for thrLogMsg in testModule.thrScOutMsgs[c]:  # loop over the THR cluster log message
        #     thrList.append(thrLogMsg.addSubscriber())
        # k = 0
        # for info in testModule.thrMsgDataSC[c]:
        #     for i in range(thrNumList[c][k]):
        #         thrInfo.append(info)
        #     k += 1
        # scData.thrInMsgs = messaging.THROutputMsgInMsgsVector(thrList)
        # scData.thrInfo = vizInterface.ThrClusterVector(thrInfo)

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
    v_BN_N = scStateData.v_BN_N
    omega_BN_B = scStateData.omega_BN_B

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
    pltName = fileName + "theta"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    plt.clf()
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta1Dot, label=r'$\dot{\theta}_1$')
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta2Dot, label=r'$\dot{\theta}_2$')
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]')
    pltName = fileName + "thetaDot"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    plt.clf()
    for idx in range(3):
        plt.plot(theta1Data.times() * macros.NANO2SEC, v_BN_N[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$v_{BN,' + str(idx) + '}$')
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'Velocity [m/s]')
    pltName = fileName + "velocity"
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    plt.clf()
    for idx in range(3):
        plt.plot(theta1Data.times() * macros.NANO2SEC, omega_BN_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BN,' + str(idx) + '}$')
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'Angular Velocity [rad/s]')
    pltName = fileName + "angularVelocity"
    figureList[pltName] = plt.figure(4)

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
