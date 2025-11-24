#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

This scenario simulates the response of a spacecraft with a flexible solar panel that is hit by space debris. The solar
array is divided into five segments, each with bending and torsional modes. The impact occurs at the third panel, which
imparts flexing of the panel and induces rotation of the entire spacecraft. After impact, the hinges "break", modeled by
changing their stiffness and damping coefficients.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioImpact.py

The scenario outputs three plots: one for the time history of the affected sub-panel angles, one for the sub-panel
rates, and another for the hub's angular velocity. The scenario also creates a comprehensive Vizard simulation that
shows the panels flexing after impact.

Illustration of Simulation Results
----------------------------------
The impact happens at 10 seconds, which coincides with the large deflections shown in the sub-panel's angles and angle
rates. The impact on the hub is clearly seen in the third plot, as the hub goes from not rotating to oscillating in
response to the panel's flexing motion.

.. image:: /_images/Scenarios/scenarioImpact_theta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioImpact_thetaDot.svg
   :align: center

.. image:: /_images/Scenarios/scenarioImpact_angularVelocity.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Illustrates a debris impact on a flexible solar array
# Author:   João Vaz Carneiro
# Creation Date:  November 23, 2025
#

import os
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass, vizSupport, simIncludeGravBody, orbitalMotion
from Basilisk.simulation import spacecraft, spinningBodyTwoDOFStateEffector, spinningBodyNDOFStateEffector, extForceTorque
from Basilisk.utilities import macros, unitTestSupport

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

    # Define gravity
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
    scObject.hub.omega_BN_BInit = [[0.], [0], [0.]]

    # Define the properties of the panels (rectangular cuboids)
    lengthSubPanel = diameter
    widthSubPanel = diameter
    thicknessSubPanel = 0.2
    massSubPanel = 20.0

    panel = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    panel.ModelTag = "panel"
    for idx in range(5):
        spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
        spinningBody.setMass(0.0)
        spinningBody.setISPntSc_S([[0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0]])
        spinningBody.setDCM_S0P([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        spinningBody.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])
        if idx == 0:
            spinningBody.setR_SP_P([[0.0], [diameter / 2], [height / 2 - thicknessSubPanel / 2]])
        else:
            spinningBody.setR_SP_P([[0.0], [lengthSubPanel], 0.0])
        spinningBody.setSHat_S([[1], [0], [0]])
        spinningBody.setThetaInit(0.0 * macros.D2R)
        spinningBody.setThetaDotInit(0.0 * macros.D2R)
        spinningBody.setK(2000)
        spinningBody.setC(150)
        panel.addSpinningBody(spinningBody)

        spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
        spinningBody.setMass(massSubPanel)
        spinningBody.setISPntSc_S([[massSubPanel / 12 * (lengthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0, 0.0],
                                   [0.0, massSubPanel / 12 * (widthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0],
                                   [0.0, 0.0, massSubPanel / 12 * (widthSubPanel ** 2 + lengthSubPanel ** 2)]])
        spinningBody.setDCM_S0P([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        spinningBody.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])
        spinningBody.setR_SP_P([[0.0], [0.0], [0.0]])
        spinningBody.setSHat_S([[0], [1], [0]])
        spinningBody.setThetaInit(0.0 * macros.D2R)
        spinningBody.setThetaDotInit(0.0 * macros.D2R)
        spinningBody.setK(500)
        spinningBody.setC(75)
        panel.addSpinningBody(spinningBody)

    # Add spinning body to spacecraft
    scObject.addStateEffector(panel)

    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.extForce_B = [[0.0], [0.0], [0.0]]
    extFTObject.extTorquePntB_B = [[0.0], [0.0], [0.0]]
    extFTObject.ModelTag = "externalDisturbance"
    panel.addDynamicEffector(extFTObject, 5)

    # Add modules to simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, panel)
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # Set up the message recorders and add them to the task
    datLog = scObject.scStateOutMsg.recorder()
    theta1Data = panel.spinningBodyOutMsgs[4].recorder()
    theta2Data = panel.spinningBodyOutMsgs[5].recorder()
    scStateData = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, datLog)
    scSim.AddModelToTask(simTaskName, theta1Data)
    scSim.AddModelToTask(simTaskName, theta2Data)
    scSim.AddModelToTask(simTaskName, scStateData)

    #
    # Set up Vizard visualization
    #
    scBodyList = [scObject]
    for idx in range(5):
        scBodyList.append([f"subPanel{idx + 1}", panel.spinningBodyConfigLogOutMsgs[2 * idx + 1]])

    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scBodyList
                                                  # , saveFile=fileName
                                                  )

        vizSupport.createCustomModel(viz
                                     , simBodiesToModify=[scObject.ModelTag]
                                     , modelPath="CYLINDER"
                                     , scale=[diameter, diameter, height / 2]
                                     , color=vizSupport.toRGBA255("grey"))
        for idx in range(5):
            vizSupport.createCustomModel(viz
                                         , simBodiesToModify=[f"subPanel{idx + 1}"]
                                         , modelPath="CUBE"
                                         , scale=[widthSubPanel, lengthSubPanel, thicknessSubPanel]
                                         , color=vizSupport.toRGBA255("gold"))
        viz.settings.orbitLinesOn = -1

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Configure a simulation stop time and execute the simulation run
    simulationTime = macros.sec2nano(10)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Apply a debris strike
    extFTObject.extForce_B = [[3000.0], [4000.0], [5000.0]]
    extFTObject.extTorquePntB_B = [[2000.0], [3000.0], [1000.0]]
    simulationTime += simulationTimeStep
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Reset the forces on the panels
    extFTObject.extForce_B = [[0.0], [0.0], [0.0]]
    extFTObject.extTorquePntB_B = [[0.0], [0.0], [0.0]]

    # Change the subpanel's stiffness and damping to simulate a "broken" hinge
    subpanel = panel.getSpinningBody(4)
    subpanel.setK(200.0)
    subpanel.setC(0.0)
    subpanel = panel.getSpinningBody(5)
    subpanel.setK(200.0)
    subpanel.setC(0.0)

    # Continue the simulation
    simulationTime += macros.sec2nano(20)
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve the logged data
    theta1 = theta1Data.theta
    theta1Dot = theta1Data.thetaDot
    theta2 = theta2Data.theta
    theta2Dot = theta2Data.thetaDot
    omega_BN_B = scStateData.omega_BN_B

    #
    #   Plot the results
    #
    figureList = {}

    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    plt.clf()
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta1, label=r'$\theta_1$')
    plt.plot(theta2Data.times() * macros.NANO2SEC, macros.R2D * theta2, label=r'$\theta_2$')
    plt.xlabel('time [s]')
    plt.ylabel(r'$\theta$ [deg]')
    pltName = fileName + "_theta"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    plt.clf()
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta1Dot, label=r'$\dot{\theta}_1$')
    plt.plot(theta1Data.times() * macros.NANO2SEC, macros.R2D * theta2Dot, label=r'$\dot{\theta}_2$')
    plt.xlabel('time [s]')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]')
    pltName = fileName + "_thetaDot"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    plt.clf()
    for idx in range(3):
        plt.plot(theta1Data.times() * macros.NANO2SEC, omega_BN_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BN,' + str(idx) + '}$')
    plt.xlabel('time [s]')
    plt.ylabel(r'Angular Velocity [rad/s]')
    pltName = fileName + "_angularVelocity"
    figureList[pltName] = plt.figure(3)

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
