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

Demonstrates how to convert spacecraft states, stored in a text file from another program, into Basilisk
messages using :ref:`dataFileToViz`.  These messages are red by :ref:`vizInterface` to save a :ref:`Vizard <vizard>`
compatible data play for offline playback and analysis.  In this simulation a servicer is holding a relative
position with respect to an uncontrolled satellite.  Custom spacecraft models are specified for Vizard
in the folder ``dataForExamples``.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioDataToViz.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both modules.

.. image:: /_images/static/test_scenarioDataToViz.svg
   :align: center

When the simulation completes several plots are shown for the MRP norm attitude history and the
inertial relative position vector components.  A servicer spacecraft approaches a target and holds a specific
target-frame fixed location even while the target itself is slowly rotating.  The servicer and target orientations
are controlled to be the same to prepare for a final docking maneuver.  If the data is saved to a Vizard file,
then the visualization should look like:

.. image:: /_images/static/vizard-DataFile.jpg
   :align: center

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

.. image:: /_images/Scenarios/scenarioDataToViz1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioDataToViz2.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Basic simulation showing a servicer (3-axis attitude controlled) and a tumbling debris object.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 29, 2019
#

import os

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.simulation import dataFileToViz
from Basilisk.simulation import spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, simIncludeGravBody, vizSupport)
from Basilisk.utilities import unitTestSupport

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

# The path to the location of Basilisk
# Used to get the location of supporting data.
fileName = os.path.basename(os.path.splitext(__file__)[0])



def run(show_plots, attType):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        attType (int): Specify if MRP (0) or Quaternions (1) are used
    """

    path = os.path.dirname(os.path.abspath(__file__))
    if attType == 0:
        dataFileName = os.path.join(path, "dataForExamples", "scHoldTraj_rotating_MRP.csv")
    elif attType == 1:
        dataFileName = os.path.join(path, "dataForExamples", "scHoldTraj_rotating_EP.csv")
    else:
        print("unknown attType variable")
        exit()
    file1 = open(dataFileName, 'r')
    Lines = file1.readlines()
    delimiter = ","
    t0 = float(Lines[1].split(delimiter)[0])
    t1 = float(Lines[2].split(delimiter)[0])
    tN = float(Lines[-1].split(delimiter)[0])
    timeStepSeconds = t1 - t0
    simulationTimeSeconds = tN - t0

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the simulation time step information
    simulationTimeStep = macros.sec2nano(timeStepSeconds)
    simulationTime = macros.sec2nano(simulationTimeSeconds)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # setup Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body

    # create SC dummy objects to setup basic Vizard settings.  Only one has to have the Grav Bodies attached
    # to show up in Vizard
    scObject1 = spacecraft.Spacecraft()
    scObject1.ModelTag = "servicer"
    scObject1.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "target"
    scList = [scObject1, scObject2]

    #   setup the module to read in the simulation data
    dataModule = dataFileToViz.DataFileToViz()
    dataModule.ModelTag = "testModule"
    dataModule.setNumOfSatellites(2)
    # load the data path from the same folder where this python script is
    dataModule.attitudeType = attType
    dataModule.dataFileName = dataFileName

    dataModule.delimiter = delimiter
    scSim.AddModelToTask(simTaskName, dataModule)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = []
    for scCounter in range(2):
        dataLog.append(dataModule.scStateOutMsgs[scCounter].recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, dataLog[-1])

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    if vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scList
                                                  # , saveFile=fileName
                                                  )
        viz.settings.showSpacecraftLabels = 1
        viz.settings.spacecraftShadowBrightness = 0.2
        # load CAD for target spacecraft
        vizSupport.createCustomModel(viz,
                                     modelPath=os.path.join(path, "dataForExamples", "Aura_27.obj"),
                                     shader=1,
                                     simBodiesToModify=[scList[1].ModelTag],
                                     rotation=[180. * macros.D2R, 0.0 * macros.D2R, -90. * macros.D2R],
                                     scale=[1, 1, 1])
        # load CAD for servicer spacecraft
        vizSupport.createCustomModel(viz,
                                     modelPath=os.path.join(path, "dataForExamples", "Loral-1300Com-main.obj"),
                                     simBodiesToModify=[scList[0].ModelTag],
                                     rotation=[0. * macros.D2R, -90.0 * macros.D2R, 0. * macros.D2R],
                                     scale=[0.09, 0.09, 0.09])

        # over-ride the default to not read the SC states from scObjects, but set them directly
        # to read from the dataFileToFiz output message
        viz.scData.clear()
        for c in range(len(scList)):
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = scList[c].ModelTag
            scData.scStateInMsg.subscribeTo(dataModule.scStateOutMsgs[c])

            viz.scData.push_back(scData)

    #   initialize Simulation
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve logged data
    posB1N = dataLog[0].r_BN_N
    posB2N = dataLog[1].r_BN_N
    sigmaB1N = dataLog[0].sigma_BN
    sigmaB2N = dataLog[1].sigma_BN

    #
    #   plot the results
    #
    timeData = dataLog[0].times() * macros.NANO2HOUR
    plt.close("all")  # clears out plots from earlier test runs
    figureList = {}

    plt.figure(1)
    s1Data = []
    for idx in sigmaB1N:
        sNorm = np.linalg.norm(idx)
        s1Data.append(sNorm)
    plt.plot(timeData, s1Data, color=unitTestSupport.getLineColor(1, 3), label=r'$|\sigma_{B1/N}|$')
    s2Data = []
    for idx in sigmaB2N:
        sNorm = np.linalg.norm(idx)
        s2Data.append(sNorm)
    plt.plot(timeData, s2Data, color=unitTestSupport.getLineColor(2, 3), label=r'$|\sigma_{B2/N}|$')
    plt.xlabel('Time [h]')
    plt.ylabel(r'MRP Norm')
    plt.legend(loc='lower right')
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    rhoData = []
    for r1, r2 in zip(posB1N, posB2N):
        rhoData.append(r2 - r1)
    rhoData = np.array(rhoData)
    for idx in range(3):
        plt.plot(timeData, rhoData[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\rho_{' + str(idx+1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel(r'$\rho_{S/T}$ (Inertial) [m]')
    plt.legend(loc='lower right')
    pltName = fileName + "2"
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
        0       # attitude coordinate type, 0 - MRP, 1 - quaternions
    )
