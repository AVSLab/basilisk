#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DIHubLAIMS ALL WARRANTIES
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
:math:`N` spin axes. The :math:`N`-degree-of-freedom formulation allows the simulation of any number of attached rigid bodies with
rotation capable about any axis. The spin axes and mass distribution of the rigid bodies are arbitrary.

The scenario consists of a box-shaped hub with an attached robotic arm. The arm consists of two links
with spherical joints that can rotate
about two different axis. Each joint degree of freedom is controlled through a profiler that moves the arm
from the nominal position to a final desired angle.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioRoboticArm.py

The scenario outputs four plots: the time history of all angles, of all angle rates, of the hub's attitude and of the
hub's angular velocity. The scenario also creates a comprehensive Vizard simulation which creates an appropriate
to-scale model for the defined scenario.

Illustration of Simulation Results
----------------------------------

.. raw:: html

   <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;
   max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/4mMCCTVHZkA?si=L3Pj9dfy7a8DSwjQ"
        style="position: absolute;
        top: 0; left: 0; width: 100%; height: 100%;" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media;
        gyroscope; picture-in-picture" allowfullscreen></iframe>
    </div>

.. image:: /_images/Scenarios/scenarioRoboticArm_theta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRoboticArm_thetaDot.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRoboticArm_sigma_BN.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRoboticArm_omega_BN_B.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Illustrates the different simulation capabilities of the NDOF modules
# Author:   João Vaz Carneiro
# Creation Date:  May 15, 2024
#

import os
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import (SimulationBaseClass, vizSupport, simIncludeGravBody, macros, orbitalMotion, unitTestSupport)
from Basilisk.simulation import spacecraft, spinningBodyNDOFStateEffector, prescribedRotation1DOF
from Basilisk.architecture import messaging

from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    scSim = createSimBaseClass()

    scGeometry = geometryClass()
    scObject = setUpSpacecraft(scSim, scGeometry)
    roboticArmEffector = setUpEffector(scSim, scObject, scGeometry)

    mu = setUpGravity(scSim, scObject)
    defineInitialConditions(scObject, mu)

    scLog, thetaLog = setUpRecorders(scSim, scObject, roboticArmEffector)
    if vizSupport.vizFound:
        setUpVizard(scSim, scObject, roboticArmEffector, scGeometry)

    runSimulation(scSim)

    figureList = plotting(show_plots, scLog, thetaLog)
    return figureList


class geometryClass:
    massHub = 1000
    lengthHub = 3
    widthHub = 3
    heightHub = 6
    massLink = 50
    heightLink = 3
    diameterLink = 0.6


def createSimBaseClass():
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.simulationTime = macros.min2nano(5.)
    scSim.dynTaskName = "dynTask"
    scSim.dynProcess = scSim.CreateNewProcess("dynProcess")
    dynTimeStep = macros.sec2nano(0.2)
    scSim.dynProcess.addTask(scSim.CreateNewTask(scSim.dynTaskName, dynTimeStep))

    return scSim


def setUpSpacecraft(scSim, scGeometry):
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    scObject.hub.mHub = scGeometry.massHub
    scObject.hub.IHubPntBc_B = [[scGeometry.massHub / 12 * (scGeometry.lengthHub ** 2 + scGeometry.heightHub ** 2), 0.0, 0.0],
                                [0.0, scGeometry.massHub / 12 * (scGeometry.widthHub ** 2 + scGeometry.heightHub ** 2), 0.0],
                                [0.0, 0.0, scGeometry.massHub / 12 * (scGeometry.lengthHub ** 2 + scGeometry.widthHub ** 2)]]
    scSim.AddModelToTask(scSim.dynTaskName, scObject)

    return scObject


def setUpEffector(scSim, scObject, scGeometry):
    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector.ModelTag = "spinningBodyEffector"
    scObject.addStateEffector(spinningBodyEffector)

    createFirstLink(scSim, spinningBodyEffector, scGeometry)
    createSecondLink(scSim, spinningBodyEffector, scGeometry)
    scSim.AddModelToTask(scSim.dynTaskName, spinningBodyEffector)

    return spinningBodyEffector


def createFirstLink(scSim, spinningBodyEffector, scGeometry):
    spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody.setMass(0)
    spinningBody.setISPntSc_S([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    spinningBody.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    spinningBody.setR_ScS_S([[0.0], [scGeometry.heightLink / 2], [0.0]])
    spinningBody.setR_SP_P([[0], [scGeometry.lengthHub / 2], [scGeometry.heightHub / 2 - scGeometry.diameterLink / 2]])
    spinningBody.setSHat_S([[1], [0], [0]])
    spinningBody.setThetaInit(0 * macros.D2R)
    spinningBody.setThetaDotInit(0 * macros.D2R)
    spinningBody.setK(500.0)
    spinningBody.setC(100.0)
    spinningBodyEffector.addSpinningBody(spinningBody)

    profiler = prescribedRotation1DOF.PrescribedRotation1DOF()
    profiler.ModelTag = "profiler"
    profiler.setThetaDDotMax(0.01 * macros.D2R)
    profiler.setThetaInit(spinningBody.getThetaInit())
    profiler.setSmoothingDuration(10)
    scSim.AddModelToTask(scSim.dynTaskName, profiler)
    spinningBodyEffector.spinningBodyRefInMsgs[0].subscribeTo(profiler.spinningBodyOutMsg)

    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    hingedRigidBodyMessageData.theta = -30 * macros.D2R  # [rad]
    hingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    hingedRigidBodyMessage1 = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMessageData)
    hingedRigidBodyMessage1.this.disown()
    profiler.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage1)

    spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody.setMass(scGeometry.massLink)
    spinningBody.setISPntSc_S([[spinningBody.getMass() / 12 * (3 * (scGeometry.diameterLink / 2) ** 2 + scGeometry.heightLink ** 2), 0.0, 0.0],
                               [0.0, spinningBody.getMass() / 12 * (scGeometry.diameterLink / 2) ** 2, 0.0],
                               [0.0, 0.0, spinningBody.getMass() / 12 * (3 * (scGeometry.diameterLink / 2) ** 2 + scGeometry.heightLink ** 2)]])
    spinningBody.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    spinningBody.setR_ScS_S([[0.0], [scGeometry.heightLink / 2], [0.0]])
    spinningBody.setR_SP_P([[0], [0], [0]])
    spinningBody.setSHat_S([[0], [0], [1]])
    spinningBody.setThetaInit(0 * macros.D2R)
    spinningBody.setThetaDotInit(0 * macros.D2R)
    spinningBody.setK(500.0)
    spinningBody.setC(100.0)
    spinningBodyEffector.addSpinningBody(spinningBody)

    profiler = prescribedRotation1DOF.PrescribedRotation1DOF()
    profiler.ModelTag = "profiler"
    profiler.setThetaDDotMax(0.01 * macros.D2R)
    profiler.setThetaInit(spinningBody.getThetaInit())
    profiler.setSmoothingDuration(10)
    scSim.AddModelToTask(scSim.dynTaskName, profiler)
    spinningBodyEffector.spinningBodyRefInMsgs[1].subscribeTo(profiler.spinningBodyOutMsg)

    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    hingedRigidBodyMessageData.theta = 45 * macros.D2R  # [rad]
    hingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    hingedRigidBodyMessage2 = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMessageData)
    hingedRigidBodyMessage2.this.disown()
    profiler.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage2)


def createSecondLink(scSim, spinningBodyEffector, scGeometry):
    spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody.setMass(0)
    spinningBody.setISPntSc_S([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    spinningBody.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    spinningBody.setR_ScS_S([[0.0], [scGeometry.heightLink / 2], [0.0]])
    spinningBody.setR_SP_P([[0], [scGeometry.heightLink], [0]])
    spinningBody.setSHat_S([[1], [0], [0]])
    spinningBody.setThetaInit(0 * macros.D2R)
    spinningBody.setThetaDotInit(0 * macros.D2R)
    spinningBody.setK(500.0)
    spinningBody.setC(100.0)
    spinningBodyEffector.addSpinningBody(spinningBody)

    profiler = prescribedRotation1DOF.PrescribedRotation1DOF()
    profiler.ModelTag = "profiler"
    profiler.setThetaDDotMax(0.02 * macros.D2R)
    profiler.setThetaInit(spinningBody.getThetaInit())
    profiler.setSmoothingDuration(10)
    scSim.AddModelToTask(scSim.dynTaskName, profiler)
    spinningBodyEffector.spinningBodyRefInMsgs[2].subscribeTo(profiler.spinningBodyOutMsg)

    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    hingedRigidBodyMessageData.theta = 90 * macros.D2R  # [rad]
    hingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    hingedRigidBodyMessage1 = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMessageData)
    hingedRigidBodyMessage1.this.disown()
    profiler.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage1)

    spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody.setMass(scGeometry.massLink)
    spinningBody.setISPntSc_S([[spinningBody.getMass() / 12 * (3 * (scGeometry.diameterLink / 2) ** 2 + scGeometry.heightLink ** 2), 0.0, 0.0],
                               [0.0, spinningBody.getMass() / 12 * (scGeometry.diameterLink / 2) ** 2, 0.0],
                               [0.0, 0.0, spinningBody.getMass() / 12 * (3 * (scGeometry.diameterLink / 2) ** 2 + scGeometry.heightLink ** 2)]])
    spinningBody.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    spinningBody.setR_ScS_S([[0.0], [scGeometry.heightLink / 2], [0.0]])
    spinningBody.setR_SP_P([[0], [0], [0]])
    spinningBody.setSHat_S([[0], [0], [1]])
    spinningBody.setThetaInit(0 * macros.D2R)
    spinningBody.setThetaDotInit(0 * macros.D2R)
    spinningBody.setK(500.0)
    spinningBody.setC(100.0)
    spinningBodyEffector.addSpinningBody(spinningBody)

    profiler = prescribedRotation1DOF.PrescribedRotation1DOF()
    profiler.ModelTag = "profiler"
    profiler.setThetaDDotMax(0.02 * macros.D2R)
    profiler.setThetaInit(spinningBody.getThetaInit())
    profiler.setSmoothingDuration(10)
    scSim.AddModelToTask(scSim.dynTaskName, profiler)
    spinningBodyEffector.spinningBodyRefInMsgs[3].subscribeTo(profiler.spinningBodyOutMsg)

    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    hingedRigidBodyMessageData.theta = -20 * macros.D2R  # [rad]
    hingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    hingedRigidBodyMessage2 = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMessageData)
    hingedRigidBodyMessage2.this.disown()
    profiler.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage2)


def setUpGravity(scSim, scObject):
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'sun'])
    gravBodies['earth'].isCentralBody = True
    mu = gravBodies['earth'].mu
    gravFactory.addBodiesTo(scObject)

    timeInitString = "2012 MAY 1 00:28:30.0"
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/', timeInitString, epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'earth'
    scSim.AddModelToTask(scSim.dynTaskName, gravFactory.spiceObject)

    return mu


def defineInitialConditions(scObject, mu):
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
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]


def setUpRecorders(scSim, scObject, roboticArmEffector):
    scLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(scSim.dynTaskName, scLog)
    thetaLog = []
    for outMsg in roboticArmEffector.spinningBodyOutMsgs:
        thetaLog.append(outMsg.recorder())
        scSim.AddModelToTask(scSim.dynTaskName, thetaLog[-1])

    return scLog, thetaLog


def setUpVizard(scSim, scObject, roboticArmEffector, scGeometry):
    scBodyList = [scObject,
                  ["arm1", roboticArmEffector.spinningBodyConfigLogOutMsgs[1]],
                  ["arm2", roboticArmEffector.spinningBodyConfigLogOutMsgs[3]]]

    viz = vizSupport.enableUnityVisualization(scSim, scSim.dynTaskName, scBodyList
                                              # , saveFile=fileName
                                              )

    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=[scObject.ModelTag]
                                 , modelPath="CUBE"
                                 , color=vizSupport.toRGBA255("gold")
                                 , scale=[scGeometry.widthHub, scGeometry.lengthHub, scGeometry.heightHub])
    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=["arm1"]
                                 , modelPath="CYLINDER"
                                 , scale=[scGeometry.diameterLink, scGeometry.diameterLink, scGeometry.heightLink / 2]
                                 , rotation=[np.pi / 2, 0, 0])
    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=["arm2"]
                                 , modelPath="CYLINDER"
                                 , scale=[scGeometry.diameterLink, scGeometry.diameterLink, scGeometry.heightLink / 2]
                                 , rotation=[np.pi / 2, 0, 0])
    viz.settings.orbitLinesOn = -1


def runSimulation(scSim):
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(scSim.simulationTime)
    scSim.ExecuteSimulation()


def plotting(show_plots, scLog, thetaLog):
    sigma_BN = scLog.sigma_BN
    omega_BN = scLog.omega_BN_B
    theta = []
    thetaDot = []
    for thetaData in thetaLog:
        theta.append(thetaData.theta)
        thetaDot.append(thetaData.thetaDot)

    dynTimeMin = scLog.times() * macros.NANO2MIN
    figureList = {}
    plt.close("all")

    plt.figure(1)
    plt.clf()
    ax = plt.axes()
    for idx, angle in enumerate(theta):
        plt.plot(dynTimeMin, macros.R2D * angle, label=r'$\theta_' + str(idx + 1) + '$')
    plt.legend(fontsize='14')
    plt.title('Angles', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\theta$ [deg]', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_theta"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    plt.clf()
    ax = plt.axes()
    for idx, angleRate in enumerate(thetaDot):
        plt.plot(dynTimeMin, macros.R2D * angleRate, label=r'$\dot{\theta}_' + str(idx + 1) + '$')
    plt.legend(fontsize='14')
    plt.title('Angle Rates', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_thetaDot"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    plt.clf()
    ax = plt.axes()
    for idx in range(3):
        plt.plot(dynTimeMin, sigma_BN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(fontsize='14')
    plt.title('Attitude', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\sigma_{B/N}$', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_sigma_BN"
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    plt.clf()
    ax = plt.axes()
    for idx in range(3):
        plt.plot(dynTimeMin, omega_BN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_' + str(idx) + '$')
    plt.legend(fontsize='14')
    plt.title('Attitude Rate', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\omega_{B/N}$', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_omega_BN_B"
    figureList[pltName] = plt.figure(4)

    if show_plots:
        plt.show()

    plt.close("all")

    return figureList


if __name__ == "__main__":
    run(True)
