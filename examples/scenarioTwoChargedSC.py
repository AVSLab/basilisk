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

Demonstrates the interaction between two charged spacecraft in a leader/follower configuration, the effect electrostatic forces/torques have on the separation of the two spacecrafts and how to visualize the simulation data in :ref:`Vizard <vizard>`. The scenario demonstrates how to use :ref:`msmForceTorque` to calculate the electrostatic forces using the Multi-Sphere Method. Each spacecraft is represented by multiple spheres each with a designated location and radius. The locations and radii data is stored in ``GOESR_bus_80_sphs.csv``, however any appropriate csv file can be used. Both spacecraft have a negative charged potential. The purpose of this script is to show how to set up the Multi-Sphere Method for charged spacecraft and apply the external forces/torques to the spacecrafts as well as to show how to store the Basilisk simulation data to be able to visualize both satellite's motions within the :ref:`Vizard <vizard>` application.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioTwoChargedSC.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the leader spacecraft and the follower object as well as the multisphere model for each of the spacecrafts.

.. image:: /_images/static/test_scenarioTwoChargedSC.svg
   :align: center

When the simulation completes, several plots are shown for the separation distance of the two satellites, the relative orbit of the follower spacecraft around the leader spacecraft, and the Multisphere Model representation for both spacecraft with a color bar that denotes charge of the spheres.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

.. image:: /_images/Scenarios/scenarioTwoChargedSC1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTwoChargedSC2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTwoChargedSC3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioTwoChargedSC4.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing two charged spacecraft interacting based on the Multisphere Sphere Method.
# Author:   James Walker
# Creation Date:  January 19, 2022
#

import copy
import csv
import math
import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft, extForceTorque, msmForceTorque
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                unitTestSupport, RigidBodyKinematics, vizSupport, SpherePlot)

try:
    from Basilisk.simulation import vizInterface

    vizFound = True
except ImportError:
    vizFound = False

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots):
    """
    The scenarios can be run with the following setup parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    # create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName, 1)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(25.0)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simulationTimeStep))

    #
    # setup the simulation tasks/objects
    #

    # initialize leader spacecraft object and set properties
    scObjectLeader = spacecraft.Spacecraft()
    scObjectLeader.ModelTag = "Leader"

    # initialize follower spacecraft object and set properties
    scObjectFollower = spacecraft.Spacecraft()
    scObjectFollower.ModelTag = "Follower"

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(dynTaskName, scObjectLeader)
    scSim.AddModelToTask(dynTaskName, scObjectFollower)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObjectLeader.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObjectFollower.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup MSM module
    MSMmodule = msmForceTorque.MsmForceTorque()
    MSMmodule.ModelTag = "msmForceTorqueTag"
    scSim.AddModelToTask(dynTaskName, MSMmodule)

    # define electric potentials
    voltLeaderInMsgData = messaging.VoltMsgPayload()
    voltLeaderInMsgData.voltage = -500  # [V] servicer potential
    voltLeaderInMsg = messaging.VoltMsg().write(voltLeaderInMsgData)

    voltFollowerInMsgData = messaging.VoltMsgPayload()
    voltFollowerInMsgData.voltage = 500  # [V] debris potential
    voltFollowerInMsg = messaging.VoltMsg().write(voltFollowerInMsgData)

    # Import multi-sphere model of GOESR bus and read them into an array of strings
    # For each list of 4, the first 3 values are the spacial location of an individual sphere relative to a center of
    # [0,0,0] and the forth value is the radius of the sphere
    path = os.path.dirname(os.path.abspath(__file__))
    dataFileName = os.path.join(path, 'dataForExamples', 'GOESR_bus_80_sphs.csv')
    scSphMod = open(dataFileName)
    type(scSphMod)
    csvreader = csv.reader(scSphMod)
    rows = []
    for row in csvreader:
        rows.append(row)
    scSphMod.close()
    # Convert the strings to numbers and separate the location data from the radius data
    radii = []
    spherelocation = []
    for row in rows:
        radii.append(float(row.pop(3)))
        rownum = [float(i) for i in row]
        spherelocation.append(rownum)
    spPosListLeader_H = spherelocation  # The location of each sphere for the leader spacecraft
    rListLeader = radii  # radius of each sphere in the leader spacecraft
    spPosListFollower_H = spherelocation  # The location of each sphere for the follower spacecraft
    rListFollower = radii  # radius of each sphere in the follower spacecraft
            
            
#    If you would like to simulate each spacecraft by a single sphere, uncomment this section (line186 - line189) of
    #    code and comment out the previous section lines (162-181)
#     create a list of sphere body-fixed locations and associated radii using one sphere for each spacecraft
#    spPosListLeader_H = [[0,0,0]]  # one sphere located at origin of body frame
#    rListLeader = [2]  # radius of sphere is 2m
#    spPosListFollower_H = [[0,0,0]]  # one sphere located at origin of body frame
#    rListFollower = [2]  # radius of sphere is 2m

    # add spacecraft to state
    MSMmodule.addSpacecraftToModel(scObjectLeader.scStateOutMsg, messaging.DoubleVector(rListLeader),
                                   unitTestSupport.npList2EigenXdVector(spPosListLeader_H))
    MSMmodule.addSpacecraftToModel(scObjectFollower.scStateOutMsg, messaging.DoubleVector(rListFollower),
                                   unitTestSupport.npList2EigenXdVector(spPosListFollower_H))

    # subscribe input messages to module
    MSMmodule.voltInMsgs[0].subscribeTo(voltLeaderInMsg)
    MSMmodule.voltInMsgs[1].subscribeTo(voltFollowerInMsg)

    # setup extForceTorque module for Leader
    # the electrostatic force from the MSM module is read in through the messaging system
    extFTObjectLeader = extForceTorque.ExtForceTorque()
    extFTObjectLeader.ModelTag = "eForceLeader"
    extFTObjectLeader.cmdForceInertialInMsg.subscribeTo(MSMmodule.eForceOutMsgs[0])
    scObjectLeader.addDynamicEffector(extFTObjectLeader)
    scSim.AddModelToTask(dynTaskName, extFTObjectLeader)

    # setup extForceTorque module for Follower
    # the electrostatic force from the MSM module is read in through the messaging system
    extFTObjectFollower = extForceTorque.ExtForceTorque()
    extFTObjectFollower.ModelTag = "eForceDebris"
    extFTObjectFollower.cmdForceInertialInMsg.subscribeTo(MSMmodule.eForceOutMsgs[1])
    scObjectFollower.addDynamicEffector(extFTObjectFollower)
    scSim.AddModelToTask(dynTaskName, extFTObjectFollower)

    #   set initial Spacecraft States
    #
    # set up the Leader orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 42164 * 1e3  # [m] Geosynchronous Orbit
    oe.e = 0.
    oe.i = 0.
    oe.Omega = 0.
    oe.omega = 0
    oe.f = 0.
    r_LN, v_LN = orbitalMotion.elem2rv(mu, oe)
    scObjectLeader.hub.r_CN_NInit = r_LN  # m
    scObjectLeader.hub.v_CN_NInit = v_LN  # m/s
    oe = orbitalMotion.rv2elem(mu, r_LN, v_LN)

    # setup Follower object states
    r_FS = np.array([0, -50.0, 0.0])  # relative position of follower, 10m behind servicer in along-track direction
    r_FN = r_FS + r_LN
    v_FN = v_LN
    scObjectFollower.hub.r_CN_NInit = r_FN  # m
    scObjectFollower.hub.v_CN_NInit = v_FN  # m/s
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n  # orbit period

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 1000
    simulationTime = macros.sec2nano(0.1 * P)
    samplingTime = simulationTime // (numDataPoints - 1)
    dataRecL = scObjectLeader.scStateOutMsg.recorder()
    dataRecF = scObjectFollower.scStateOutMsg.recorder()
    
    # Add recorders to the Task
    scSim.AddModelToTask(dynTaskName, dataRecL)
    scSim.AddModelToTask(dynTaskName, dataRecF)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    if vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, [scObjectLeader, scObjectFollower]
                                                  # , saveFile=fileName,
                                                  )

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # Retrieve the charge data of the spheres
    LeaderSpCharges = unitTestSupport.columnToRowList(MSMmodule.chargeMsmOutMsgs[0].read().q)
    FollowerSpCharges = unitTestSupport.columnToRowList(MSMmodule.chargeMsmOutMsgs[1].read().q)

    # retrieve the logged data from the recorders
    posDataL_N = dataRecL.r_BN_N
    velDataL_N = dataRecL.v_BN_N
    posDataF_N = dataRecF.r_BN_N

    attDataL_N = dataRecL.sigma_BN
    attDataF_N = dataRecF.sigma_BN
    
    # Calculate relative position vector and magnitude in the inertial frame
    relPosData_N = posDataL_N[:, 0:3] - posDataF_N[:, 0:3]
    relPosMagn = np.linalg.norm(relPosData_N, axis=1)

    # Project the relative position data from the inertial frame into the Hill frame of the leader spacecraft
    relPosData_H = []
    relXPosData_H = []
    relYPosData_H = []
    relZPosData_H = []
    for i in range(len(relPosData_N)):
        # Calculate the discrete cosine matrix for mapping from inertial frame to the Hill frame of the leader spacecraft
        nrn = posDataL_N[i, :]/math.sqrt(posDataL_N[i, 0]**2 + posDataL_N[i, 1]**2 + posDataL_N[i, 2]**2)
        nrh = np.cross(posDataL_N[i, 0:3], velDataL_N[i, 0:3])/np.linalg.norm(np.cross(posDataL_N[i, 0:3],
                                                                                       velDataL_N[i, 0:3]))
        nre = np.cross(nrh, nrn)
        HN = nrn, nre, nrh

        # Map the relative postion data to the Hill frame of the leader spacecraft
        relPosDatai_H = np.dot(HN, relPosData_N[i])
        relXPosData_H.append(relPosDatai_H[0])
        relYPosData_H.append(relPosDatai_H[1])
        relZPosData_H.append(relPosDatai_H[2])
        relPosData_H.append(relPosDatai_H)

    # Collect times of each recording
    timeData = dataRecL.times()

    np.set_printoptions(precision=16)

    figureList = plotOrbits(timeData, posDataL_N, posDataF_N, relPosMagn, attDataL_N, attDataF_N, P, spPosListLeader_H,
                            rListLeader, LeaderSpCharges, spPosListFollower_H, rListFollower, FollowerSpCharges,
                            relXPosData_H, relYPosData_H, relZPosData_H)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList
    

def plotOrbits(timeData, posDataL_N, posDataF_N, relPosMagn, attDataL_N, attDataF_N, P, spPosListLeader_H, rListLeader,
               LeaderSpCharges, spPosListFollower_H, rListFollower, FollowerSpCharges, relXPosData_H, relYPosData_H,
               relZPosData_H):

    #
    # draw the total separation of the spacecrafts
    #

    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    plt.plot(timeData * macros.NANO2SEC / P, relPosMagn[:])
    plt.xlabel('Time [orbits]')
    plt.ylabel('Separation [m]')
    plt.title('Total separation')
    figureList = {}
    pltName = 'scenarioTwoChargedSC1'
    figureList[pltName] = plt.figure(1)

    #
    # Plot relative separation in the Frame of the Leader spacecrafts
    #

    plt.figure(2, figsize=(5, 4))
    ax = plt.axes(projection='3d')
    # Set the Leader S/C as the center of the plot
    r_LN_N = np.array([0., 0., 0.])
    # get sphere locations
    dcm_NL = RigidBodyKinematics.MRP2C(attDataF_N[0, 0:3]).transpose()
    spPosL_N = np.dot(dcm_NL, np.array(spPosListLeader_H).transpose()).transpose()
    radiiL = copy.deepcopy(rListLeader)
    # Plot the sphere locations to model the Leader spacecraft
    u = np.linspace(0, np.pi, 10)
    v = np.linspace(0, 2 * np.pi, 10)
    x = np.outer(np.sin(u), np.sin(v))
    y = np.outer(np.sin(u), np.cos(v))
    z = np.outer(np.cos(u), np.ones_like(v))
    for ii in range(0, len(radiiL)):
        r_SpN_N = r_LN_N + spPosL_N[ii, 0:3]
        ax.plot_surface(r_SpN_N[0] + radiiL[ii] * x, r_SpN_N[1] + radiiL[ii] * y, r_SpN_N[2] + radiiL[ii] * z, color="b")

    # Plot the relative position of the Follower spacecraft
    ax.plot(relXPosData_H, relYPosData_H, relZPosData_H)
    ax.set_xlabel('Radial(m)')
    ax.set_ylabel('Along Track(m)')
    ax.set_zlabel('Orbit Normal (m)')
    pltName = 'scenarioTwoChargedSC2'
    figureList[pltName] = plt.figure(2)

    #
    # Draw the sphere representation of the satellites used by the MSM in the Hill frame of the Leader spacecraft
    #
    SpherePlotList = SpherePlot.plotSpheres(posDataL_N, posDataF_N, attDataL_N, attDataF_N, spPosListLeader_H, rListLeader,
               LeaderSpCharges, spPosListFollower_H, rListFollower, FollowerSpCharges)

    figureList['scenarioTwoChargedSC3'] = SpherePlotList['Charged_Spheres']
    figureList['scenarioTwoChargedSC4'] = SpherePlotList['Colorbar']

    return figureList


def NormalizeData(data):
    return (data - np.min(data)) / (np.max(data) - np.min(data))

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#


if __name__ == "__main__":
    run(
        True,  # show_plots
    )
