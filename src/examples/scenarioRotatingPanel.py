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

This scenario demonstrates how to set up a spacecraft spacecraft with rotating panel.  A
:ref:`coarse_sun_sensor` is then
attached onto this panel such that it's bore-sight axis rotates with the panel.  Further, the panel state
message is connected to :ref:`simpleSolarPanel`.

.. image:: /_images/static/test_scenario_RotatingPanel.svg
   :align: center

The script is found in the folder ``src/examples`` and executed by using::

    python3 scenarioRotatingPanel.py



Making a Copy of the Example Basilisk Scenario Script
-----------------------------------------------------



Illustration of Simulation Results
----------------------------------

The following images illustrate the expected simulation run returns for a range of script configurations.

::

    show_plots = True, orbitCase='LEO', useSphericalHarmonics=False, planetCase='Earth'

This scenario places the spacecraft about the Earth in a LEO orbit and without considering gravitational
spherical harmonics.

.. image:: /_images/Scenarios/scenarioBasicOrbit1LEO0Earth.svg
   :align: center

.. image:: /_images/Scenarios/scenarioBasicOrbit2LEO0Earth.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus() and gravity modules.  Illustrates
#           connected the hingedRigidBody panel state message to a CSS and solar power module
# Author:   Hanspeter Schaub
# Creation Date:  Oct. 6, 2020
#

import os
import numpy as np

import matplotlib.pyplot as plt

# To play with any scenario scripts as tutorials, you should make a copy of them into a custom folder
# outside of the Basilisk directory.
#
# To copy them, first find the location of the Basilisk installation.
# After installing, you can find the installed location of Basilisk by opening a python interpreter and
# running the commands:
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

# import simulation related support
from Basilisk.simulation import spacecraftPlus
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import simMessages
from Basilisk.simulation import simpleSolarPanel
from Basilisk.simulation import coarse_sun_sensor
import math

def run(show_plots):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # create the spacecraft hub
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # setup Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True
    mu = planet.mu
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    # create sun position message
    sunMessage = simMessages.SpicePlanetStateSimMsg()
    sunMessage.PlanetName = "Sun"
    sunMessage.PositionVector = [0, orbitalMotion.AU*1000, 0]
    sunStateMsg = "SunMsg"
    unitTestSupport.setMessage(scSim.TotalSim, simProcessName, sunStateMsg, sunMessage)

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
    scObject.hub.sigma_BNInit = [[math.tan(-90./4.*macros.D2R)], [0.0], [0.0]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    #
    # configure rotating panel module
    #
    panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel1.mass = 100.0
    panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    panel1.d = 0.0  # the S and H point are the same in this scenario
    panel1.k = 0.0
    panel1.c = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    panel1.r_HB_B = [[-2.0], [0.0], [1.0]]
    panel1.dcm_HB = rbk.euler3(90.0 * macros.D2R)
    # unique names for the state engine to track the panel angular states
    panel1.nameOfThetaState = "hingedRigidBodyTheta1"
    panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"
    # panel initial angular states
    panel1.thetaInit = 0.0
    panel1.thetaDotInit = 1.0 * macros.D2R  # rad/sec panel rotation rate

    # message containing panel angular states
    panel1.hingedRigidBodyOutMsgName = "panel1Msg"
    # message containing panel inertial position and attitude states
    panel1.hingedRigidBodyConfigLogOutMsgName = "panel1Log"

    # add panel to spacecraft hub
    scObject.addStateEffector(panel1)  # in order to affect dynamics

    #
    #  Solar Panel Power Module set-up
    #
    solarPanel = simpleSolarPanel.SimpleSolarPanel()
    solarPanel.nHat_B = [0, 0, 1]  # direction is now in the rotating panel S frame!
    solarPanel.panelArea = 2.0  # m^2
    solarPanel.panelEfficiency = 0.9  # 90% efficiency in power generation
    solarPanel.stateInMsgName = panel1.hingedRigidBodyConfigLogOutMsgName # states relative to panel states
    solarPanel.sunInMsgName = sunStateMsg
    solarPanel.nodePowerOutMsgName = "solarPanelPowerMsg"

    #
    # setup CSS sensors attached to rotating solar panel
    #
    CSS1 = coarse_sun_sensor.CoarseSunSensor()
    CSS1.ModelTag = "CSS1_sensor"
    CSS1.fov = 45. * macros.D2R
    CSS1.scaleFactor = 1.0
    CSS1.cssDataOutMsgName = "CSS1_output"
    CSS1.sunInMsgName = sunStateMsg
    CSS1.nHat_B = [1, 0, 0]
    CSS1.stateInMsgName = panel1.hingedRigidBodyConfigLogOutMsgName  # states relative to panel states

    CSS2 = coarse_sun_sensor.CoarseSunSensor()
    CSS2.ModelTag = "CSS2_sensor"
    CSS2.fov = 45. * macros.D2R
    CSS2.scaleFactor = 1.0
    CSS2.cssDataOutMsgName = "CSS2_output"
    CSS2.sunInMsgName = sunStateMsg
    CSS2.nHat_B = [0, 0, 1]
    CSS2.stateInMsgName = panel1.hingedRigidBodyConfigLogOutMsgName  # states relative to panel states

    #
    # add modules to simulation task list
    #
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, panel1)
    scSim.AddModelToTask(simTaskName, solarPanel)
    scSim.AddModelToTask(simTaskName, CSS1)
    scSim.AddModelToTask(simTaskName, CSS2)

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.05 * P)

    # Setup data logging before the simulation is initialized
    numDataPoints = 200
    samplingTime = simulationTime // (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(panel1.hingedRigidBodyOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(solarPanel.nodePowerOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(CSS1.cssDataOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(CSS2.cssDataOutMsgName, samplingTime)

    # Vizard Visualization Option
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName,
                                              # saveFile=__file__,
                                              # liveStream=True,
                                              gravBodies=gravFactory
                                              )

    scSim.InitializeSimulationAndDiscover()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   retrieve the logged data
    dataSigmaBN = scSim.pullMessageLogData(scObject.scStateOutMsgName + ".sigma_BN", list(range(3)))
    panel1thetaLog = scSim.pullMessageLogData(panel1.hingedRigidBodyOutMsgName + '.theta', list(range(1)))
    solarPowerLog = scSim.pullMessageLogData(solarPanel.nodePowerOutMsgName + ".netPower", list(range(1)))
    css1Log = scSim.pullMessageLogData(CSS1.cssDataOutMsgName + ".OutputData", list(range(1)))
    css2Log = scSim.pullMessageLogData(CSS2.cssDataOutMsgName + ".OutputData", list(range(1)))

    np.set_printoptions(precision=16)

    figureList = plotOrbits(dataSigmaBN, panel1thetaLog, solarPowerLog, css1Log, css2Log)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


def plotOrbits(dataSigmaBN, panel1thetaLog, solarPowerLog, css1Log, css2Log):
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    figCounter = 1
    plt.figure(figCounter)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    timeData = dataSigmaBN[:, 0] * macros.NANO2MIN
    for idx in range(1, 4):
        plt.plot(timeData, dataSigmaBN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'MRP Attitude $\sigma_{B/N}$')
    figureList = {}
    pltName = fileName + str(figCounter)
    figureList[pltName] = plt.figure(figCounter)

    # rotating panel hinge angle and power
    figCounter += 1
    plt.figure(figCounter)
    ax1 = plt.figure(figCounter).add_subplot(111)
    ax1.plot(timeData, panel1thetaLog[:, 1]*macros.R2D % 360, '--', color='royalblue')
    ax1.fill_between(timeData, 0, 90, facecolor='lemonchiffon')
    ax1.fill_between(timeData, 270, 360, facecolor='lemonchiffon')
    ax1.set_yticks([0, 90, 180, 270, 360])
    plt.xlabel('Time [min]')
    plt.ylabel('Panel Angle [deg]', color='royalblue')
    ax2 = plt.figure(figCounter).add_subplot(111, sharex=ax1, frameon=False)
    ax2.plot(timeData, solarPowerLog[:, 1], color='goldenrod')
    ax2.yaxis.tick_right()
    ax2.yaxis.set_label_position("right")
    plt.ylabel('Solar Panel Power [W]', color='goldenrod')
    pltName = fileName + str(figCounter) + "panel1theta"
    figureList[pltName] = plt.figure(figCounter)

    # plot CSS data
    figCounter += 1
    plt.figure(figCounter)
    ax1 = plt.figure(figCounter).add_subplot(111)
    ax1.plot(timeData, panel1thetaLog[:, 1]*macros.R2D % 360, '--', color='royalblue')
    ax1.set_yticks([0, 90, 180, 270, 360])
    plt.xlabel('Time [min]')
    plt.ylabel('Panel Angle [deg]', color='royalblue')
    ax2 = plt.figure(figCounter).add_subplot(111, sharex=ax1, frameon=False)
    ax2.plot(timeData, css1Log[:, 1],
             color='tab:pink',
             label=r'CSS$_1$')
    ax2.plot(timeData, css2Log[:, 1],
             color='tab:olive',
             label=r'CSS$_2$')
    ax1.fill_between(timeData, 225, 315, facecolor='pink')
    ax1.fill_between(timeData, 315, 360, facecolor='palegoldenrod')
    ax1.fill_between(timeData, 0, 45, facecolor='palegoldenrod')
    plt.legend(loc='lower right')
    ax2.yaxis.tick_right()
    ax2.yaxis.set_label_position("right")
    plt.ylabel(r'CSS Signals')
    pltName = fileName + str(figCounter)
    figureList[pltName] = plt.figure(figCounter)

    return figureList


if __name__ == "__main__":
    run(
        True  # show_plots
    )
