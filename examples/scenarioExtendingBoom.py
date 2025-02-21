#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario demonstrates the capabilities of :ref:`linearTranslationNDOFStateEffector`, which represents an effector
with up to :math:`N` axis. The N-degree-of-freedom formulation allows the simulation of a component with any number of
links with any number of degrees of freedom. The displacement axis and mass distribution of the rigid bodies are
arbitrary.

The spacecraft comprises a rigid hub with a four-link appendage. The scenario starts with the links stowed at their
nominal position and the spacecraft rotating in space. Then, each link extends, deploying the effector to full
extension. The goal is to show the impact of the change in inertia due to the appendage deployment on the system's
angular velocity.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioExtendingBoom.py

The scenario outputs four plots: one for the time history of the displacements, another for displacement rates, one for
the time history of the spacecraft's attitude and another for the angular velocity. The scenario also creates a
comprehensive Vizard simulation which creates appropriate to-scale models.

Illustration of Simulation Results
----------------------------------

The simulation results are shown below. Each link extends with constant acceleration and deceleration until the end of
the simulation. The key takeaway is how the magnitude of the spacecraft's angular velocity diminishes as the inertia of
the spacecraft increases with the extension of the effector.

.. image:: /_images/Scenarios/scenarioExtendingBoomRho.svg
   :align: center

.. image:: /_images/Scenarios/scenarioExtendingBoomRhoDot.svg
   :align: center

.. image:: /_images/Scenarios/scenarioExtendingBoomSigma.svg
   :align: center

.. image:: /_images/Scenarios/scenarioExtendingBoomOmega.svg
   :align: center

"""

import os
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass, vizSupport, simIncludeGravBody
from Basilisk.simulation import spacecraft, linearTranslationNDOFStateEffector, prescribedLinearTranslation
from Basilisk.utilities import macros, orbitalMotion, unitTestSupport

from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  Create the simulation process
    dynProcess = scSim.CreateNewProcess(dynProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simulationTimeStep))

    # Define the spacecraft's properties
    scGeometry = geometryClass()

    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    scObject.hub.mHub = scGeometry.massHub
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[scGeometry.massHub / 12 * (scGeometry.lengthHub ** 2 + scGeometry.heightHub ** 2), 0.0, 0.0],
                                [0.0, scGeometry.massHub / 12 * (scGeometry.widthHub ** 2 + scGeometry.heightHub ** 2), 0.0],
                                [0.0, 0.0, scGeometry.massHub / 12 * (scGeometry.lengthHub ** 2 + scGeometry.widthHub ** 2)]]
    scSim.AddModelToTask(dynTaskName, scObject)

    # Set the spacecraft's initial conditions
    scObject.hub.r_CN_NInit = np.array([0, 0, 0])  # m   - r_CN_N
    scObject.hub.v_CN_NInit = np.array([0, 0, 0])  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.05], [-0.05], [0.05]]

    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'sun'])
    gravBodies['earth'].isCentralBody = True
    mu = gravBodies['earth'].mu
    gravFactory.addBodiesTo(scObject)

    timeInitString = "2016 JUNE 3 01:34:30.0"
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/', timeInitString, epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'earth'
    scSim.AddModelToTask(dynTaskName, gravFactory.spiceObject)

    oe = orbitalMotion.ClassicElements()
    oe.a = 8e6  # meters
    oe.e = 0.1
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m
    scObject.hub.v_CN_NInit = vN  # m/s
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [0.1], [0.1]]

    translatingBodyEffector = linearTranslationNDOFStateEffector.linearTranslationNDOFStateEffector()
    translatingBodyEffector.ModelTag = "translatingBodyEffector"
    scObject.addStateEffector(translatingBodyEffector)
    scSim.AddModelToTask(dynTaskName, translatingBodyEffector)

    translatingBody1 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody1.setMass(100)
    translatingBody1.setIPntFc_F([[translatingBody1.getMass() / 12 * (3 * (scGeometry.diameterArm / 2) ** 2 + scGeometry.heightArm ** 2), 0.0, 0.0],
                               [0.0, translatingBody1.getMass() / 12 * (scGeometry.diameterArm / 2) ** 2, 0.0],
                               [0.0, 0.0, translatingBody1.getMass() / 12 * (3 * (scGeometry.diameterArm / 2) ** 2 + scGeometry.heightArm ** 2)]])
    translatingBody1.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody1.setR_FcF_F([[0.0], [scGeometry.heightArm / 2], [0.0]])
    translatingBody1.setR_F0P_P([[0], [scGeometry.lengthHub / 2], [0]])
    translatingBody1.setFHat_P([[0], [1], [0]])
    translatingBody1.setRhoInit(0.0)
    translatingBody1.setRhoDotInit(0.0)
    translatingBody1.setC(400.0)
    translatingBody1.setK(100.0)
    translatingBodyEffector.addTranslatingBody(translatingBody1)

    translatingBody2 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody2.setMass(100)
    translatingBody2.setIPntFc_F([[translatingBody2.getMass() / 12 * (3 * (scGeometry.diameterArm / 2) ** 2 + scGeometry.heightArm ** 2), 0.0, 0.0],
                                  [0.0, translatingBody2.getMass() / 12 * (scGeometry.diameterArm / 2) ** 2, 0.0],
                                  [0.0, 0.0, translatingBody2.getMass() / 12 * (
                                              3 * (scGeometry.diameterArm / 2) ** 2 + scGeometry.heightArm ** 2)]])
    translatingBody2.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody2.setR_FcF_F([[0.0], [scGeometry.heightArm / 2], [0.0]])
    translatingBody2.setR_F0P_P([[0], [0], [0]])
    translatingBody2.setFHat_P([[0], [1], [0]])
    translatingBody2.setRhoInit(0.0)
    translatingBody2.setRhoDotInit(0.0)
    translatingBody2.setC(400.0)
    translatingBody2.setK(100.0)
    translatingBodyEffector.addTranslatingBody(translatingBody2)

    profiler2 = prescribedLinearTranslation.PrescribedLinearTranslation()
    profiler2.ModelTag = "profiler"
    profiler2.setTransAccelMax(0.0005)
    profiler2.setTransPosInit(translatingBody2.getRhoInit())
    profiler2.setSmoothingDuration(10)
    scSim.AddModelToTask(dynTaskName, profiler2)
    translatingBodyEffector.translatingBodyRefInMsgs[1].subscribeTo(profiler2.linearTranslationRigidBodyOutMsg)

    translatingRigidBodyMsgData = messaging.LinearTranslationRigidBodyMsgPayload()
    translatingRigidBodyMsgData.rho = scGeometry.heightArm  # [m]
    translatingRigidBodyMsgData.rhoDot = 0  # [m/s]
    translatingRigidBodyMsg2 = messaging.LinearTranslationRigidBodyMsg().write(translatingRigidBodyMsgData)
    profiler2.linearTranslationRigidBodyInMsg.subscribeTo(translatingRigidBodyMsg2)

    translatingBody3 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody3.setMass(100)
    translatingBody3.setIPntFc_F([[translatingBody3.getMass() / 12 * (
                3 * (scGeometry.diameterArm / 2) ** 2 + scGeometry.heightArm ** 2), 0.0, 0.0],
                                  [0.0, translatingBody3.getMass() / 12 * (scGeometry.diameterArm / 2) ** 2, 0.0],
                                  [0.0, 0.0, translatingBody3.getMass() / 12 * (
                                          3 * (scGeometry.diameterArm / 2) ** 2 + scGeometry.heightArm ** 2)]])
    translatingBody3.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody3.setR_FcF_F([[0.0], [scGeometry.heightArm / 2], [0.0]])
    translatingBody3.setR_F0P_P([[0], [0], [0]])
    translatingBody3.setFHat_P([[0], [1], [0]])
    translatingBody3.setRhoInit(0.0)
    translatingBody3.setRhoDotInit(0.0)
    translatingBody3.setC(400.0)
    translatingBody3.setK(100.0)
    translatingBodyEffector.addTranslatingBody(translatingBody3)

    profiler3 = prescribedLinearTranslation.PrescribedLinearTranslation()
    profiler3.ModelTag = "profiler"
    profiler3.setTransAccelMax(0.0005)
    profiler3.setTransPosInit(translatingBody3.getRhoInit())
    profiler3.setSmoothingDuration(10)
    scSim.AddModelToTask(dynTaskName, profiler3)
    translatingBodyEffector.translatingBodyRefInMsgs[2].subscribeTo(profiler3.linearTranslationRigidBodyOutMsg)

    translatingRigidBodyMsgData = messaging.LinearTranslationRigidBodyMsgPayload()
    translatingRigidBodyMsgData.rho = scGeometry.heightArm  # [m]
    translatingRigidBodyMsgData.rhoDot = 0  # [m/s]
    translatingRigidBodyMsg3 = messaging.LinearTranslationRigidBodyMsg().write(translatingRigidBodyMsgData)
    profiler3.linearTranslationRigidBodyInMsg.subscribeTo(translatingRigidBodyMsg3)

    translatingBody4 = linearTranslationNDOFStateEffector.translatingBody()
    translatingBody4.setMass(100)
    translatingBody4.setIPntFc_F([[translatingBody4.getMass() / 12 * (
                3 * (scGeometry.diameterArm / 2) ** 2 + scGeometry.heightArm ** 2), 0.0, 0.0],
                                  [0.0, translatingBody4.getMass() / 12 * (scGeometry.diameterArm / 2) ** 2, 0.0],
                                  [0.0, 0.0, translatingBody4.getMass() / 12 * (
                                          3 * (scGeometry.diameterArm / 2) ** 2 + scGeometry.heightArm ** 2)]])
    translatingBody4.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody4.setR_FcF_F([[0.0], [scGeometry.heightArm / 2], [0.0]])
    translatingBody4.setR_F0P_P([[0], [0], [0]])
    translatingBody4.setFHat_P([[0], [1], [0]])
    translatingBody4.setRhoInit(0.0)
    translatingBody4.setRhoDotInit(0.0)
    translatingBody4.setC(400.0)
    translatingBody4.setK(100.0)
    translatingBodyEffector.addTranslatingBody(translatingBody4)

    profiler4 = prescribedLinearTranslation.PrescribedLinearTranslation()
    profiler4.ModelTag = "profiler"
    profiler4.setTransAccelMax(0.0005)
    profiler4.setTransPosInit(translatingBody4.getRhoInit())
    profiler4.setSmoothingDuration(10)
    scSim.AddModelToTask(dynTaskName, profiler4)
    translatingBodyEffector.translatingBodyRefInMsgs[3].subscribeTo(profiler4.linearTranslationRigidBodyOutMsg)

    translatingRigidBodyMsgData = messaging.LinearTranslationRigidBodyMsgPayload()
    translatingRigidBodyMsgData.rho = scGeometry.heightArm  # [m]
    translatingRigidBodyMsgData.rhoDot = 0  # [m/s]
    translatingRigidBodyMsg4 = messaging.LinearTranslationRigidBodyMsg().write(translatingRigidBodyMsgData)
    profiler4.linearTranslationRigidBodyInMsg.subscribeTo(translatingRigidBodyMsg4)

    scLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, scLog)
    rhoLog = []
    for outMsg in translatingBodyEffector.translatingBodyOutMsgs:
        rhoLog.append(outMsg.recorder())
        scSim.AddModelToTask(dynTaskName, rhoLog[-1])

    if vizSupport.vizFound:
        scBodyList = [scObject,
                      ["arm1", translatingBodyEffector.translatingBodyConfigLogOutMsgs[0]],
                      ["arm2", translatingBodyEffector.translatingBodyConfigLogOutMsgs[1]],
                      ["arm3", translatingBodyEffector.translatingBodyConfigLogOutMsgs[2]],
                      ["arm4", translatingBodyEffector.translatingBodyConfigLogOutMsgs[3]]]
        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scBodyList
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
                                     , color=vizSupport.toRGBA255("gray")
                                     , scale=[scGeometry.diameterArm, scGeometry.diameterArm, scGeometry.heightArm / 2]
                                     , rotation=[np.pi / 2, 0, 0]
                                     )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["arm2"]
                                     , modelPath="CYLINDER"
                                     , color=vizSupport.toRGBA255("darkgray")
                                     , scale=[.95 * scGeometry.diameterArm, .95 * scGeometry.diameterArm, scGeometry.heightArm / 2]
                                     , rotation=[np.pi / 2, 0, 0]
                                     )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["arm3"]
                                     , modelPath="CYLINDER"
                                     , color=vizSupport.toRGBA255("silver")
                                     , scale=[.90 * scGeometry.diameterArm, .90 * scGeometry.diameterArm, scGeometry.heightArm / 2]
                                     , rotation=[np.pi / 2, 0, 0]
                                     )
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=["arm4"]
                                     , modelPath="CYLINDER"
                                     , color=vizSupport.toRGBA255("lightgray")
                                     , scale=[.85 * scGeometry.diameterArm, .85 * scGeometry.diameterArm, scGeometry.heightArm / 2]
                                     , rotation=[np.pi / 2, 0, 0]
                                     )
        viz.settings.orbitLinesOn = -1

    simulationTime = macros.min2nano(3.0)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    timeData = scLog.times() * macros.NANO2MIN
    attLog = scLog.sigma_BN
    omegaLog = scLog.omega_BN_B
    rho = []
    rhoDot = []
    for rhoData in rhoLog:
        rho.append(rhoData.rho)
        rhoDot.append(rhoData.rhoDot)

    figureList = {}
    plt.close("all")
    plotArmTimeHistory(timeData, rho, rhoDot, figureList)
    plotSCStates(timeData, attLog, omegaLog, figureList)

    if show_plots:
        plt.show()

    plt.close("all")
    return figureList


class geometryClass:
    massHub = 400
    lengthHub = 3
    widthHub = 3
    heightHub = 6
    massArm = 50
    heightArm = 3
    diameterArm = 0.6


def plotArmTimeHistory(timeData, rho, rhoDot, figureList):
    plt.figure(1)
    ax = plt.axes()
    for idx, disp in enumerate(rho):
        plt.plot(timeData, disp, label=r'$\rho_' + str(idx+1) + '$')
    plt.legend(fontsize='14')
    # plt.title('Displacements', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\rho$ [m]', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "Rho"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    ax = plt.axes()
    for idx, velo in enumerate(rhoDot):
        plt.plot(timeData, velo, label=r'$\dot{\rho}_' + str(idx + 1) + '$')
    plt.legend(fontsize='14')
    # plt.title('Displacement Rates', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\dot{\rho}$ [m/s]', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "RhoDot"
    figureList[pltName] = plt.figure(2)


def plotSCStates(timeData, attLog, omegaLog, figureList):
    plt.figure(3)
    ax = plt.axes()
    for idx in range(3):
        plt.plot(timeData, attLog[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(fontsize='14')
    # plt.title('Attitude', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\sigma_{B/N}$', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "Sigma"
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    ax = plt.axes()
    for idx in range(3):
        plt.plot(timeData, omegaLog[:, idx],
                 color=unitTestSupport.getLineColor(idx, 4),
                 label=r'$\omega_' + str(idx) + '$')
    plt.plot(timeData, np.linalg.norm(omegaLog, axis=1),
             color=unitTestSupport.getLineColor(3, 4),
             label=r'$|\mathbf{\omega}|$',
             linestyle='dashed')
    plt.legend(fontsize='14')
    # plt.title('Attitude Rate', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\omega_{B/N}$ [rad/s]', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "Omega"
    figureList[pltName] = plt.figure(4)


if __name__ == "__main__":
    run(True)  # show_plots
