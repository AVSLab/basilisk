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
:ref:`coarseSunSensor` is then
attached onto this panel such that it's bore-sight axis rotates with the panel.  Further, the panel state
message is connected to :ref:`simpleSolarPanel`.

.. image:: /_images/static/test_scenario_RotatingPanel.svg
   :align: center

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioRotatingPanel.py

The spacecraft body frame :math:`B` is at rest and rotated such that :math:`\hat{\bf b}_3` is pointing at the sun.
The panel is oriented relative to B such that the panel normal axis :math:`\hat{\bf n}` is along :math:`\hat{\bf b}_3`
if the panel angle :math:`\theta` is zero.

.. image:: /_images/static/test_scenario_RotatingPanelFig1.svg
   :align: center

The hinge frame origin :math:`H` and solar panel frame origin :math:`S` are set to be the.  The panel then
is set to rotate at a positive rate about the :math:`\hat{\bf h}_2` axis.  Thus, the solar panel should provide electrical
power at the beginning of the simulation, but power should vanish when the panel rotates beyond 90 degrees, etc.

The coarse sun sensor of CSS devices are added to this solar panel as well.  The script connects the panel inertial
state message to each CSS module create such that their sensor signals vary with the panel orientation.  Each
CSS unit has a boresight to edge field of view of 45 degrees, and the maximum signal output is set to 1.  The CSS
boresight orientation unit 1 is set to :math:`{}^{\cal S}[1,0,0]` in solar panel frame components.
The second CSS units is pointing along :math:`{}^{\cal S}[0,0,1]`.


Illustration of Simulation Results
----------------------------------

The script will generate 3 plots.  The first plot show the spacecraft orientation which is holding steady
pointing :math:`\hat{\bf b}_3` into the sun heading.

.. image:: /_images/Scenarios/scenarioRotatingPanel1.svg
   :align: center

The second plot compares the solar panel rotation angle :math:`\theta` relative to the generated solar power.
The regions where we expect to see electrical power being generated are shaded in yellow.

.. image:: /_images/Scenarios/scenarioRotatingPanel2panel1theta.svg
   :align: center

The final plot compares the solar panel angle with respect to the two CSS signals.  The angular regions where we expect
the CSS units to get a signal are shaded with a color that matches the CSS data plot.  Again good agreement is
found with the predicted times when the rotating panel will yield CSS signals and when not.

.. image:: /_images/Scenarios/scenarioRotatingPanel3.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft() and gravity modules.  Illustrates
#           connected the hingedRigidBody panel state message to a CSS and solar power module
# Author:   Hanspeter Schaub
# Creation Date:  Oct. 6, 2020
#

import os

import matplotlib.pyplot as plt
import numpy as np
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
from Basilisk.simulation import spacecraft
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport)
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging
from Basilisk.simulation import simpleSolarPanel
from Basilisk.simulation import coarseSunSensor
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

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(1.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # create the spacecraft hub
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    scObject.hub.mHub = 750.0
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # setup Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True
    mu = planet.mu
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # create sun position message
    sunMessage = messaging.SpicePlanetStateMsgPayload()
    sunMessage.PlanetName = "Sun"
    sunMessage.PositionVector = [0, orbitalMotion.AU*1000, 0]
    sunStateMsg = messaging.SpicePlanetStateMsg().write(sunMessage)

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
    # panel initial angular states
    panel1.thetaInit = 0.0
    panel1.thetaDotInit = 1.0 * macros.D2R  # rad/sec panel rotation rate

    # add panel to spacecraft hub
    scObject.addStateEffector(panel1)  # in order to affect dynamics

    #
    #  Solar Panel Power Module set-up
    #
    solarPanel = simpleSolarPanel.SimpleSolarPanel()
    solarPanel.nHat_B = [0, 0, 1]  # direction is now in the rotating panel S frame!
    solarPanel.panelArea = 2.0  # m^2
    solarPanel.panelEfficiency = 0.9  # 90% efficiency in power generation
    solarPanel.stateInMsg.subscribeTo(panel1.hingedRigidBodyConfigLogOutMsg)
    solarPanel.sunInMsg.subscribeTo(sunStateMsg)

    #
    # setup CSS sensors attached to rotating solar panel
    #
    CSS1 = coarseSunSensor.CoarseSunSensor()
    CSS1.ModelTag = "CSS1_sensor"
    CSS1.fov = 45. * macros.D2R
    CSS1.scaleFactor = 1.0
    CSS1.sunInMsg.subscribeTo(sunStateMsg)
    CSS1.nHat_B = [1, 0, 0]
    CSS1.stateInMsg.subscribeTo(panel1.hingedRigidBodyConfigLogOutMsg)  # states relative to panel states
    CSS2 = coarseSunSensor.CoarseSunSensor()
    CSS2.ModelTag = "CSS2_sensor"
    CSS2.fov = 45. * macros.D2R
    CSS2.scaleFactor = 1.0
    CSS2.sunInMsg.subscribeTo(sunStateMsg)
    CSS2.nHat_B = [0, 0, 1]
    CSS2.stateInMsg.subscribeTo(panel1.hingedRigidBodyConfigLogOutMsg)  # states relative to panel states

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
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    pl1Log = panel1.hingedRigidBodyOutMsg.recorder(samplingTime)
    spLog = solarPanel.nodePowerOutMsg.recorder(samplingTime)
    css1Log = CSS1.cssDataOutMsg.recorder(samplingTime)
    css2Log = CSS2.cssDataOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, pl1Log)
    scSim.AddModelToTask(simTaskName, spLog)
    scSim.AddModelToTask(simTaskName, css1Log)
    scSim.AddModelToTask(simTaskName, css2Log)

    # Vizard Visualization Option
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject,
                                              # saveFile=__file__,
                                              # liveStream=True,
                                              cssList=[[CSS1, CSS2]]
                                              )

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   retrieve the logged data
    dataSigmaBN = dataLog.sigma_BN
    panel1thetaLog = pl1Log.theta
    solarPowerLog = spLog.netPower
    css1Log = css1Log.OutputData
    css2Log = css2Log.OutputData

    np.set_printoptions(precision=16)

    figureList = plotOrbits(dataLog.times(), dataSigmaBN, panel1thetaLog, solarPowerLog, css1Log, css2Log)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


def plotOrbits(timeAxis, dataSigmaBN, panel1thetaLog, solarPowerLog, css1Log, css2Log):
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    figCounter = 1
    plt.figure(figCounter)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    timeData = timeAxis * macros.NANO2MIN
    for idx in range(3):
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
    ax1.plot(timeData, panel1thetaLog*macros.R2D % 360, '--', color='royalblue')
    ax1.fill_between(timeData, 0, 90, facecolor='gold')
    ax1.fill_between(timeData, 270, 360, facecolor='gold')
    ax1.set_yticks([0, 90, 180, 270, 360])
    plt.xlabel('Time [min]')
    plt.ylabel('Panel Angle [deg]', color='royalblue')
    ax2 = plt.figure(figCounter).add_subplot(111, sharex=ax1, frameon=False)
    ax2.plot(timeData, solarPowerLog, color='goldenrod')
    ax2.yaxis.tick_right()
    ax2.yaxis.set_label_position("right")
    plt.ylabel('Solar Panel Power [W]', color='goldenrod')
    pltName = fileName + str(figCounter) + "panel1theta"
    figureList[pltName] = plt.figure(figCounter)

    # plot CSS data
    figCounter += 1
    plt.figure(figCounter)
    ax1 = plt.figure(figCounter).add_subplot(111)
    ax1.plot(timeData, panel1thetaLog*macros.R2D % 360, '--', color='royalblue')
    ax1.set_yticks([0, 90, 180, 270, 360])
    plt.xlabel('Time [min]')
    plt.ylabel('Panel Angle [deg]', color='royalblue')
    ax2 = plt.figure(figCounter).add_subplot(111, sharex=ax1, frameon=False)
    ax2.plot(timeData, css1Log,
             color='tab:pink',
             label=r'CSS$_1$')
    ax2.plot(timeData, css2Log,
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
