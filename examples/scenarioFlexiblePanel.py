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

The scenario consists of a box-shaped hub with a very flexible panel. The panel has two flexible directions: it can bend  or
twist. It parameterized by the number of subsections, which means that the lumped-mass approach to the flexible dynamics
can be refined with an increasing number of subpanels. The discretization into more subsections also allows for the
modeling of more modes.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioFlexiblePanel.py

The scenario outputs six plots: the time history of bending angles, of torsional angles, of bending angle rates,
of torsional angle rates, of the hub's attitude and of the hub's angular velocity. The scenario also creates a
comprehensive Vizard simulation which creates an appropriate to-scale model for the defined scenario.

Illustration of Simulation Results
----------------------------------

.. raw:: html

   <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;
   max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/kbcqUD4MW9c?si=B09rp8jm_m9pbsra"
        style="position: absolute;
        top: 0; left: 0; width: 100%; height: 100%;" frameborder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media;
        gyroscope; picture-in-picture" allowfullscreen></iframe>
    </div>

.. image:: /_images/Scenarios/scenarioFlexiblePanel_theta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanel_thetaDot.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanel_beta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanel_betaDot.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanel_sigma_BR.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanel_omega_BR_B.svg
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

from Basilisk.utilities import (SimulationBaseClass, vizSupport, simIncludeGravBody, macros, orbitalMotion,
                                unitTestSupport, RigidBodyKinematics as rbk)
from Basilisk.simulation import spacecraft, spinningBodyNDOFStateEffector, simpleNav, extForceTorque, svIntegrators
from Basilisk.fswAlgorithms import mrpFeedback, inertial3D, attTrackingError
from Basilisk.architecture import messaging

from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, numberOfSegments):
    scSim = createSimBaseClass()

    scGeometry = geometryClass(numberOfSegments)
    scObject, panel, extForceTorque = setUpDynModules(scSim, scGeometry)
    thetaData, attErrorLog = setUpFSWModules(scSim, scObject, panel, scGeometry, extForceTorque)

    if vizSupport.vizFound:
        setUpVizard(scSim, scObject, panel, scGeometry)

    runSimulation(scSim)

    figureList = plotting(show_plots, thetaData, attErrorLog, scGeometry)
    return figureList


def createSimBaseClass():
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.simulationTime = macros.min2nano(10.)
    scSim.dynTaskName = "dynTask"
    scSim.dynProcess = scSim.CreateNewProcess("dynProcess")
    simTimeStep = macros.sec2nano(0.1)
    scSim.dynProcess.addTask(scSim.CreateNewTask(scSim.dynTaskName, simTimeStep))
    scSim.fswTaskName = "fswTask"
    scSim.fswProcess = scSim.CreateNewProcess("fswProcess")
    fswTimeStep = macros.sec2nano(0.5)
    scSim.fswProcess.addTask(scSim.CreateNewTask(scSim.fswTaskName, fswTimeStep))

    return scSim


class geometryClass:
    massHub = 1000
    lengthHub = 3
    widthHub = 3
    heightHub = 6
    lengthPanel = 18.0
    widthPanel = 3.0
    thicknessPanel = 0.3
    massPanel = 100.0

    def __init__(self, numberOfSegments):
        self.numberOfSegments = numberOfSegments

        self.massSubPanel = self.massPanel / self.numberOfSegments
        self.lengthSubPanel = self.lengthPanel / self.numberOfSegments
        self.widthSubPanel = self.widthPanel
        self.thicknessSubPanel = self.thicknessPanel


def setUpDynModules(scSim, scGeometry):
    scObject = setUpSpacecraft(scSim, scGeometry)
    panel = setUpFlexiblePanel(scSim, scObject, scGeometry)
    extForceTorqueObject = setUpExtForceTorque(scSim, scObject)
    mu = setUpGravity(scSim, scObject)
    setUpInitialConditions(scObject, mu)

    return scObject, panel, extForceTorqueObject


def setUpSpacecraft(scSim, scGeometry):
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scObject"
    scObject.hub.mHub = scGeometry.massHub
    scObject.hub.IHubPntBc_B = [[scGeometry.massHub / 12 * (scGeometry.lengthHub ** 2 + scGeometry.heightHub ** 2), 0.0, 0.0],
                                [0.0, scGeometry.massHub / 12 * (scGeometry.widthHub ** 2 + scGeometry.heightHub ** 2), 0.0],
                                [0.0, 0.0, scGeometry.massHub / 12 * (scGeometry.lengthHub ** 2 + scGeometry.widthHub ** 2)]]
    scSim.AddModelToTask(scSim.dynTaskName, scObject)

    return scObject


def setUpFlexiblePanel(scSim, scObject, scGeometry):
    panel = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    panel.ModelTag = "panel"
    for idx in range(scGeometry.numberOfSegments):
        spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
        spinningBody.setMass(0.0)
        spinningBody.setISPntSc_S([[0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0]])
        spinningBody.setDCM_S0P([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        spinningBody.setR_ScS_S([[0.0], [scGeometry.lengthSubPanel / 2], [0.0]])
        if idx == 0:
            spinningBody.setR_SP_P([[0.0], [scGeometry.lengthHub / 2], [scGeometry.heightHub / 2 - scGeometry.thicknessSubPanel / 2]])
        else:
            spinningBody.setR_SP_P([[0.0], [scGeometry.lengthSubPanel], 0.0])
        spinningBody.setSHat_S([[1], [0], [0]])
        spinningBody.setThetaInit(0.0 * macros.D2R)
        spinningBody.setThetaDotInit(0.0 * macros.D2R)
        spinningBody.setK(10)
        spinningBody.setC(8)
        panel.addSpinningBody(spinningBody)

        spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
        spinningBody.setMass(scGeometry.massSubPanel)
        spinningBody.setISPntSc_S([[scGeometry.massSubPanel / 12 * (scGeometry.lengthSubPanel ** 2 + scGeometry.thicknessSubPanel ** 2), 0.0, 0.0],
                                   [0.0, scGeometry.massSubPanel / 12 * (scGeometry.widthSubPanel ** 2 + scGeometry.thicknessSubPanel ** 2), 0.0],
                                   [0.0, 0.0, scGeometry.massSubPanel / 12 * (scGeometry.widthSubPanel ** 2 + scGeometry.lengthSubPanel ** 2)]])
        spinningBody.setDCM_S0P([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        spinningBody.setR_ScS_S([[0.0], [scGeometry.lengthSubPanel / 2], [0.0]])
        spinningBody.setR_SP_P([[0.0], [0.0], [0.0]])
        spinningBody.setSHat_S([[0], [1], [0]])
        spinningBody.setThetaInit(0.0 * macros.D2R)
        spinningBody.setThetaDotInit(0.0 * macros.D2R)
        spinningBody.setK(1)
        spinningBody.setC(0.8)
        panel.addSpinningBody(spinningBody)
    scObject.addStateEffector(panel)
    scSim.AddModelToTask(scSim.dynTaskName, panel)

    return panel


def setUpExtForceTorque(scSim, scObject):
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(scSim.dynTaskName, extFTObject)

    return extFTObject


def setUpGravity(scSim, scObject):
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'sun'])
    gravBodies['earth'].isCentralBody = True
    mu = gravBodies['earth'].mu
    gravFactory.addBodiesTo(scObject)

    timeInitString = "2012 MAY 1 00:28:30.0"
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'earth'
    scSim.AddModelToTask(scSim.dynTaskName, gravFactory.spiceObject)

    return mu


def setUpInitialConditions(scObject, mu):
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


def setUpFSWModules(scSim, scObject, panel, scGeometry, extFTObject):
    sNavObject = setUpNavigation(scSim, scObject)
    inertial3DObj = setUpReference(scSim)
    attError = setUpGuidance(scSim, sNavObject, inertial3DObj)
    setUpControl(scSim, extFTObject, attError, scGeometry)

    thetaData, attErrorLog = setUpRecorders(scSim, scObject, panel, attError, scGeometry)

    return thetaData, attErrorLog


def setUpNavigation(scSim, scObject):
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(scSim.dynTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    return sNavObject


def setUpReference(scSim):
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(scSim.fswTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0.3, 0.4, 0.5]

    return inertial3DObj


def setUpGuidance(scSim, sNavObject, inertial3DObj):
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(scSim.fswTaskName, attError)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)

    return attError


def setUpControl(scSim, extFTObject, attError, scGeometry):
    IHubPntBc_B = np.array([[scGeometry.massHub / 12 * (scGeometry.lengthHub ** 2 + scGeometry.heightHub ** 2), 0.0, 0.0],
                            [0.0, scGeometry.massHub / 12 * (scGeometry.widthHub ** 2 + scGeometry.heightHub ** 2), 0.0],
                            [0.0, 0.0, scGeometry.massHub / 12 * (scGeometry.lengthHub ** 2 + scGeometry.widthHub ** 2)]])
    IPanelPntSc_B = np.array(
        [[scGeometry.massPanel / 12 * (scGeometry.lengthPanel ** 2 + scGeometry.thicknessPanel ** 2), 0.0, 0.0],
         [0.0, scGeometry.massPanel / 12 * (scGeometry.widthPanel ** 2 + scGeometry.thicknessPanel ** 2), 0.0],
         [0.0, 0.0, scGeometry.massPanel / 12 * (scGeometry.widthPanel ** 2 + scGeometry.lengthPanel ** 2)]])
    r_ScB_B = [0.0, scGeometry.lengthHub / 2 + scGeometry.lengthPanel / 2,
               scGeometry.heightHub / 2 - scGeometry.thicknessSubPanel / 2]
    IHubPntB_B = IHubPntBc_B + IPanelPntSc_B - scGeometry.massPanel * np.array(rbk.v3Tilde(r_ScB_B)) @ np.array(
        rbk.v3Tilde(r_ScB_B))

    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(scSim.fswTaskName, mrpControl)
    decayTime = 50
    xi = 0.9
    mrpControl.P = 2 * np.max(IHubPntB_B) / decayTime
    mrpControl.K = (mrpControl.P / xi) ** 2 / np.max(IHubPntB_B)

    configData = messaging.VehicleConfigMsgPayload()
    configData.IHubPntB_B = list(IHubPntB_B.flatten())
    configDataMsg = messaging.VehicleConfigMsg()
    configDataMsg.write(configData)
    configDataMsg.this.disown()

    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(configDataMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)


def setUpRecorders(scSim, scObject, panel, attError, scGeometry):
    datLog = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(scSim.dynTaskName, datLog)
    thetaData = []
    for idx in range(scGeometry.numberOfSegments):
        thetaData.append(panel.spinningBodyOutMsgs[2 * idx].recorder())
        scSim.AddModelToTask(scSim.dynTaskName, thetaData[-1])
        thetaData.append(panel.spinningBodyOutMsgs[2 * idx + 1].recorder())
        scSim.AddModelToTask(scSim.dynTaskName, thetaData[-1])
    attErrorLog = attError.attGuidOutMsg.recorder()
    scSim.AddModelToTask(scSim.fswTaskName, attErrorLog)

    return thetaData, attErrorLog


def setUpVizard(scSim, scObject, panel, scGeometry):
    scBodyList = [scObject]
    for idx in range(scGeometry.numberOfSegments):
        scBodyList.append([f"subPanel{idx + 1}", panel.spinningBodyConfigLogOutMsgs[2 * idx + 1]])
    viz = vizSupport.enableUnityVisualization(scSim, scSim.dynTaskName, scBodyList
                                              # , saveFile=fileName
                                              )
    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=[scObject.ModelTag]
                                 , modelPath="CUBE"
                                 , color=vizSupport.toRGBA255("gold")
                                 , scale=[scGeometry.widthHub, scGeometry.lengthHub, scGeometry.heightHub])
    for idx in range(scGeometry.numberOfSegments):
        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[f"subPanel{idx + 1}"]
                                     , modelPath="CUBE"
                                     , scale=[scGeometry.widthSubPanel, scGeometry.lengthSubPanel, scGeometry.thicknessSubPanel])
    viz.settings.orbitLinesOn = -1


def runSimulation(scSim):
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(scSim.simulationTime)
    scSim.ExecuteSimulation()


def plotting(show_plots, thetaData, attErrorLog, scGeometry):
    theta = []
    thetaDot = []
    for idx in range(scGeometry.numberOfSegments):
        theta.append(thetaData[2 * idx].theta)
        thetaDot.append(thetaData[2 * idx].thetaDot)
        theta.append(thetaData[2 * idx + 1].theta)
        thetaDot.append(thetaData[2 * idx + 1].thetaDot)

    figureList = {}
    plt.close("all")
    timeSecDyn = thetaData[0].times() * macros.NANO2MIN
    timeSecFSW = attErrorLog.times() * macros.NANO2MIN

    plt.figure(1)
    plt.clf()
    ax = plt.axes()
    for idx in range(scGeometry.numberOfSegments):
        plt.plot(timeSecDyn, macros.R2D * theta[2 * idx], label=r'$\theta_' + str(idx + 1) + '$')
    plt.legend(fontsize='14')
    plt.title('Bending Angles', fontsize='22')
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
    for idx in range(scGeometry.numberOfSegments):
        plt.plot(timeSecDyn, macros.R2D * theta[2 * idx + 1], label=r'$\beta_' + str(idx + 1) + '$')
    plt.legend(fontsize='14')
    plt.title('Torsional Angles', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\beta$ [deg]', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_beta"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    plt.clf()
    ax = plt.axes()
    for idx in range(scGeometry.numberOfSegments):
        plt.plot(timeSecDyn, macros.R2D * thetaDot[2 * idx], label=r'$\dot{\theta}_' + str(idx + 1) + '$')
    plt.legend(fontsize='14')
    plt.title('Bending Angle Rates', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_thetaDot"
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    plt.clf()
    ax = plt.axes()
    for idx in range(scGeometry.numberOfSegments):
        plt.plot(timeSecDyn, macros.R2D * thetaDot[2 * idx + 1], label=r'$\dot{\beta}_' + str(idx + 1) + '$')
    plt.legend(fontsize='14')
    plt.title('Torsional Angle Rates', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\dot{\beta}$ [deg/s]', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_betaDot"
    figureList[pltName] = plt.figure(4)

    plt.figure(5)
    plt.clf()
    ax = plt.axes()
    for idx in range(3):
        plt.plot(timeSecFSW, attErrorLog.sigma_BR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(fontsize='14')
    plt.title('Attitude Error', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\sigma_{B/R}$', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_sigma_BR"
    figureList[pltName] = plt.figure(5)

    plt.figure(6)
    plt.clf()
    ax = plt.axes()
    for idx in range(3):
        plt.plot(timeSecFSW, attErrorLog.omega_BR_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_' + str(idx) + '$')
    plt.legend(fontsize='14')
    plt.title('Attitude Error Rate', fontsize='22')
    plt.xlabel('time [min]', fontsize='18')
    plt.ylabel(r'$\omega_{B/R}$', fontsize='18')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    ax.yaxis.offsetText.set_fontsize(14)
    pltName = fileName + "_omega_BR_B"
    figureList[pltName] = plt.figure(6)

    if show_plots:
        plt.show()

    plt.close("all")

    return figureList


if __name__ == "__main__":
    run(
        True,
        5
    )
