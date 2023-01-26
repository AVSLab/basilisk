#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario demonstrates how to set up a spacecraft with deploying panels. The panel modules
are set up with individual profilers and motors as shown in the following diagram.

.. image:: /_images/static/test_scenario_DeployingPanel.svg
   :align: center

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioDeployingPanel.py

The simulation includes two deploying panels that start undeployed. The first panel deploys fully, 
but the second panel deploys off-nominally (to 80%), leading to a reduced power output.


Illustration of Simulation Results
----------------------------------


::

    show_plots = True

Five plots are shown. The first two show the body attitude and rate relative to the inertial frame,
demonstrating the dynamic effect of the deploying panels on the system. Because the second panel
fails to fully deploy, the body attitude does not return to the original state.

.. image:: /_images/Scenarios/scenarioDeployingPanel1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDeployingPanel2.svg
   :align: center

The next two plots show the panel angles and angle rates. Because the panels are modeled as
rigid bodies attached by a torsional spring, oscillations about the time-varying reference
angle are seen.

.. image:: /_images/Scenarios/scenarioDeployingPanel3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDeployingPanel4.svg
   :align: center

The final plot shows the power output. As the first panel deploys, power starts to be generated.
The second panel also generates power, but stops short of full deployment and thus generates less power.

.. image:: /_images/Scenarios/scenarioDeployingPanel5.svg
   :align: center


"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Scenario script and test of the hingedRigidBodyStateEffector with changes
#           made to permit deployment.
# Author:   Galen Bascom
# Creation Date:  Sept. 12, 2022
#

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation import spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)
from Basilisk.simulation import hingedRigidBodyStateEffector, simpleSolarPanel
from Basilisk.simulation import hingedBodyLinearProfiler, hingedRigidBodyMotor
import math


def run(show_plots):
    """
    Args:
        show_plots (bool): Determines if the script should display plots.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # create the spacecraft hub
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    scObject.hub.mHub = 750.0
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # setup Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()

    gravBodies = gravFactory.createBodies(['earth', 'sun'])
    gravBodies['earth'].isCentralBody = True
    mu = gravBodies['earth'].mu
    sun = 1
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    timeInitString = "2012 MAY 1 00:28:30.0"
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)
    gravFactory.spiceObject.zeroBase = 'earth'
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    rGEO = 42000. * 1000  # meters
    oe.a = (rLEO + rGEO) / 2.0
    oe.e = 1.0 - rLEO / oe.a
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)  # this stores consistent initial orbit elements

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N
    # point the body 3 axis towards the sun in the inertial n2 direction
    scObject.hub.sigma_BNInit = [[math.tan(-90. / 4. * macros.D2R)], [0.0], [0.0]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    # configure panels
    panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel1.ModelTag = "panel1"
    panel1.mass = 100.0
    panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    panel1.d = 1.5  
    panel1.k = 200.
    panel1.c = 20.  
    panel1.r_HB_B = [[-.5], [0.0], [-1.0]]
    panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    panel1.thetaInit = -np.pi
    panel1.thetaDotInit = 0

    panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel2.ModelTag = "panel2"
    panel2.mass = 100.0
    panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    panel2.d = 1.5  
    panel2.k = 200.
    panel2.c = 20.  
    panel2.r_HB_B = [[.5], [0.0], [-1.0]]
    panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    panel2.thetaInit = -np.pi
    panel2.thetaDotInit = 0

    # profilers
    profiler1 = hingedBodyLinearProfiler.HingedBodyLinearProfiler()
    profiler1.ModelTag = "deploymentProfiler"
    profiler1.startTime = macros.sec2nano(5)  # [ns] start the deployment
    profiler1.endTime = macros.sec2nano(85)  # [ns]
    profiler1.startTheta = -np.pi  # [rad] starting angle in radians
    profiler1.endTheta = -np.pi / 2  # [rad]

    profiler2 = hingedBodyLinearProfiler.HingedBodyLinearProfiler()
    profiler2.ModelTag = "deploymentProfiler2"
    profiler2.startTime = macros.sec2nano(100)  # [ns] start the deployment
    profiler2.endTime = macros.sec2nano(164)  # [ns]
    profiler2.startTheta = -np.pi  # [rad] starting angle in radians
    profiler2.endTheta = -np.pi / 1.75  # [rad] ending angle is not all the way deployed

    panel1.hingedRigidBodyRefMsg.subscribeTo(profiler1.hingedRigidBodyReferenceOutMsg)
    panel2.hingedRigidBodyRefMsg.subscribeTo(profiler2.hingedRigidBodyReferenceOutMsg)

    # motors
    motor1 = hingedRigidBodyMotor.HingedRigidBodyMotor()
    motor1.ModelTag = "hingedRigidBodyMotor"
    motor1.K = 20  # proportional gain constant
    motor1.P = 10  # derivative gain constant

    motor2 = hingedRigidBodyMotor.HingedRigidBodyMotor()
    motor2.ModelTag = "hingedRigidBodyMotor2"
    motor2.K = 20  # proportional gain constant
    motor2.P = 10  # derivative gain constant

    motor1.hingedBodyStateSensedInMsg.subscribeTo(panel1.hingedRigidBodyOutMsg)
    motor1.hingedBodyStateReferenceInMsg.subscribeTo(profiler1.hingedRigidBodyReferenceOutMsg)
    panel1.motorTorqueInMsg.subscribeTo(motor1.motorTorqueOutMsg)

    motor2.hingedBodyStateSensedInMsg.subscribeTo(panel2.hingedRigidBodyOutMsg)
    motor2.hingedBodyStateReferenceInMsg.subscribeTo(profiler2.hingedRigidBodyReferenceOutMsg)
    panel2.motorTorqueInMsg.subscribeTo(motor2.motorTorqueOutMsg)

    # add panel to spacecraft hub
    scObject.addStateEffector(panel1)  # in order to affect dynamics
    scObject.addStateEffector(panel2)  # in order to affect dynamics

    # power

    solarPanel1 = simpleSolarPanel.SimpleSolarPanel()
    solarPanel1.ModelTag = "pwr1"
    solarPanel1.nHat_B = [0, 0, 1]  
    solarPanel1.panelArea = 2.0  # m^2
    solarPanel1.panelEfficiency = 0.9  # 90% efficiency in power generation
    solarPanel1.stateInMsg.subscribeTo(panel1.hingedRigidBodyConfigLogOutMsg)
    solarPanel1.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[sun])

    solarPanel2 = simpleSolarPanel.SimpleSolarPanel()
    solarPanel2.ModelTag = "pwr2"
    solarPanel2.nHat_B = [0, 0, 1] 
    solarPanel2.panelArea = 2.0  # m^2
    solarPanel2.panelEfficiency = 0.9  # 90% efficiency in power generation
    solarPanel2.stateInMsg.subscribeTo(panel2.hingedRigidBodyConfigLogOutMsg)
    solarPanel2.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[sun])
    #
    # add modules to simulation task list
    #
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, panel1)
    scSim.AddModelToTask(simTaskName, profiler1)
    scSim.AddModelToTask(simTaskName, motor1)
    scSim.AddModelToTask(simTaskName, panel2)
    scSim.AddModelToTask(simTaskName, profiler2)
    scSim.AddModelToTask(simTaskName, motor2)
    scSim.AddModelToTask(simTaskName, solarPanel1)
    scSim.AddModelToTask(simTaskName, solarPanel2)

    # set the simulation time
    simulationTime = macros.sec2nano(240)

    # Setup data logging before the simulation is initialized
    numDataPoints = 1000
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    p1Log = panel1.hingedRigidBodyOutMsg.recorder(samplingTime)
    prof1Log = profiler1.hingedRigidBodyReferenceOutMsg.recorder(samplingTime)
    motor1Log = motor1.motorTorqueOutMsg.recorder(samplingTime)
    p2Log = panel2.hingedRigidBodyOutMsg.recorder(samplingTime)
    prof2Log = profiler2.hingedRigidBodyReferenceOutMsg.recorder(samplingTime)
    motor2Log = motor2.motorTorqueOutMsg.recorder(samplingTime)
    pwr1Log = solarPanel1.nodePowerOutMsg.recorder(samplingTime)
    pwr2Log = solarPanel2.nodePowerOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, p1Log)
    scSim.AddModelToTask(simTaskName, prof1Log)
    scSim.AddModelToTask(simTaskName, motor1Log)
    scSim.AddModelToTask(simTaskName, p2Log)
    scSim.AddModelToTask(simTaskName, prof2Log)
    scSim.AddModelToTask(simTaskName, motor2Log)
    scSim.AddModelToTask(simTaskName, pwr1Log)
    scSim.AddModelToTask(simTaskName, pwr2Log)

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName,
                                              [scObject
                                                  , [panel1.ModelTag, panel1.hingedRigidBodyConfigLogOutMsg]
                                                  , [panel2.ModelTag, panel2.hingedRigidBodyConfigLogOutMsg]
                                               ]
                                              # , saveFile=__file__
                                              )

    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=[panel1.ModelTag]
                                 , modelPath="CUBE"
                                 , scale=[3, 1, 0.1]
                                 , color=vizSupport.toRGBA255("blue"))
    vizSupport.createCustomModel(viz
                                 , simBodiesToModify=[panel2.ModelTag]
                                 , modelPath="CUBE"
                                 , scale=[3, 1, 0.1]
                                 , color=vizSupport.toRGBA255("blue"))
    viz.settings.orbitLinesOn = -1

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   retrieve the logged data
    dataSigmaBN = dataLog.sigma_BN
    dataOmegaBN_B = dataLog.omega_BN_B
    panel1thetaLog = p1Log.theta
    panel1thetaDotLog = p1Log.thetaDot
    panel2thetaLog = p2Log.theta
    panel2thetaDotLog = p2Log.thetaDot
    pwrLog1 = pwr1Log.netPower
    pwrLog2 = pwr2Log.netPower

    np.set_printoptions(precision=16)

    figureList = plotOrbits(dataLog.times(), dataSigmaBN, dataOmegaBN_B, 
                            panel1thetaLog, panel1thetaDotLog,
                            panel2thetaLog, panel2thetaDotLog, 
                            pwrLog1, pwrLog2)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


def plotOrbits(timeAxis, dataSigmaBN, dataOmegaBN,
               panel1thetaLog, panel1thetaDotLog,
               panel2thetaLog, panel2thetaDotLog,
               pwrLog1, pwrLog2):
    plt.close("all")  # clears out plots from earlier test runs

    # sigma B/N
    figCounter = 1
    plt.figure(figCounter)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    timeData = timeAxis * macros.NANO2SEC
    for idx in range(3):
        plt.plot(timeData, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [s]')
    plt.ylabel(r'MRP Attitude $\sigma_{B/N}$')
    figureList = {}
    pltName = fileName + str(figCounter)
    figureList[pltName] = plt.figure(figCounter)

    # omega B/N
    figCounter += 1
    plt.figure(figCounter)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(3):
        plt.plot(timeData, dataOmegaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [s]')
    plt.ylabel(r'Rate $\omega_{B/N}$')
    pltName = fileName + str(figCounter)
    figureList[pltName] = plt.figure(figCounter)

    # rotating panel hinge angle: panel 1
    figCounter += 1
    plt.figure(figCounter)
    ax1 = plt.figure(figCounter).add_subplot(111)
    ax1.plot(timeData, panel1thetaLog, color='royalblue')
    plt.xlabel('Time [s]')
    plt.ylabel('Panel 1 Angle [rad]', color='royalblue')
    ax2 = plt.figure(figCounter).add_subplot(111, sharex=ax1, frameon=False)
    ax2.plot(timeData, panel1thetaDotLog, color='indianred')
    ax2.yaxis.tick_right()
    ax2.yaxis.set_label_position("right")
    plt.ylabel('Panel 1 Angle Rate [rad/s]', color='indianred')
    pltName = fileName + str(figCounter)
    figureList[pltName] = plt.figure(figCounter)

    # rotating panel hinge angle: panel 2
    figCounter += 1
    plt.figure(figCounter)
    ax1 = plt.figure(figCounter).add_subplot(111)
    ax1.plot(timeData, panel2thetaLog, color='royalblue')
    plt.xlabel('Time [s]')
    plt.ylabel('Panel 2 Angle [rad]', color='royalblue')
    ax2 = plt.figure(figCounter).add_subplot(111, sharex=ax1, frameon=False)
    ax2.plot(timeData, panel2thetaDotLog, color='indianred')
    ax2.yaxis.tick_right()
    ax2.yaxis.set_label_position("right")
    plt.ylabel('Panel 2 Angle Rate [rad/s]', color='indianred')
    pltName = fileName + str(figCounter)
    figureList[pltName] = plt.figure(figCounter)

    # power: panel 1
    figCounter += 1
    plt.figure(figCounter)
    ax1 = plt.figure(figCounter).add_subplot(111)
    ax1.plot(timeData, pwrLog1, color='goldenrod', label="Panel 1")
    ax1.plot(timeData, pwrLog2, '--', color='goldenrod', label="Panel 2")
    plt.xlabel('Time [s]')
    plt.ylabel('Panel Power [W]')
    plt.legend(loc='lower right')
    pltName = fileName + str(figCounter)
    figureList[pltName] = plt.figure(figCounter)

    return figureList


if __name__ == "__main__":
    run(
        True  # show_plots
    )
