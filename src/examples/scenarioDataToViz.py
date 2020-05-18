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
position with respect to an uncontrolled satellite.

The script is found in the folder ``src/examples`` and executed by using::

      python3 scenarioDataToViz.py

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both modules.

.. image:: /_images/static/test_scenarioDataToViz.svg
   :align: center

When the simulation completes several plots are shown for the MRP norm attitude history and the
inertial relative position vector components.  A servicer spacecraft approaches a target and holds a specific
target-frame fixed location even while the target itself is slowly rotating.  The servicer and target orientations
are controlled to be the same to prepare for a final docking maneuver.

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

from Basilisk.utilities import (SimulationBaseClass, macros, simIncludeGravBody, vizSupport)
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import dataFileToViz

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

    """

    path = os.path.dirname(os.path.abspath(__file__))
    if attType == 0:
        dataFileName = os.path.join(path, "data", "scHoldTraj_MRP.csv")
    elif attType == 1:
        dataFileName = os.path.join(path, "data", "scHoldTraj_quat.csv")
    else:
        print("unknown attType variable")
        exit()
    file1 = open(dataFileName, 'r')
    Lines = file1.readlines()
    t0 = float(Lines[1].split(",")[0])
    t1 = float(Lines[2].split(",")[0])
    tN = float(Lines[-1].split(",")[0])
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

    #   setup the module to read in the simulation data
    dataModule = dataFileToViz.DataFileToViz()
    dataModule.ModelTag = "testModule"
    dataModule.numSatellites = 2
    # load the data path from the same folder where this python script is
    dataModule.attitudeType = attType
    dataModule.dataFileName = dataFileName

    scNames = ["target", "servicer"]
    dataModule.scStateOutMsgNames = dataFileToViz.StringVector(scNames)
    dataModule.delimiter = ","
    scSim.AddModelToTask(simTaskName, dataModule)

    # setup Earth Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime // (numDataPoints - 1)
    for scStateMsg in scNames:
        scSim.TotalSim.logThisMessage(scStateMsg, samplingTime)


    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # to save the BSK data to a file, uncomment the saveFile line below
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory,
                                              # saveFile=fileName,
                                              scName=scNames)
    # load CAD for target spacecraft
    vizSupport.createCustomModel(viz,
                                 modelPath=os.path.join(path, "data", "Aura_27.obj"),
                                 shader=1,
                                 simBodiesToModify=[scNames[0]],
                                 rotation=[180.*macros.D2R, 0.0*macros.D2R, -90.*macros.D2R],
                                 scale=[1, 1, 1])
    # load CAD for servicer spacecraft
    vizSupport.createCustomModel(viz,
                                 modelPath=os.path.join(path, "data", "Loral-1300Com-main.obj"),
                                 simBodiesToModify=[scNames[1]],
                                 rotation=[0.*macros.D2R, -90.0*macros.D2R, 0.*macros.D2R],
                                 scale=[0.09, 0.09, 0.09])
    if vizFound:
        # delete any existing list of vizInterface spacecraft data
        viz.scData.clear()
        for item in scNames:
            # create a chief spacecraft info container
            scData = vizInterface.VizSpacecraftData()
            scData.spacecraftName = item
            scData.scPlusInMsgName = item
            viz.scData.push_back(scData)

    #   initialize Simulation
    scSim.InitializeSimulationAndDiscover()

    #   configure a simulation stop time time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # retrieve logged data
    posB1N = scSim.pullMessageLogData(scNames[0] + ".r_BN_N", list(range(3)))
    posB2N = scSim.pullMessageLogData(scNames[1] + ".r_BN_N", list(range(3)))
    sigmaB1N = scSim.pullMessageLogData(scNames[0] + ".sigma_BN", list(range(3)))
    sigmaB2N = scSim.pullMessageLogData(scNames[1] + ".sigma_BN", list(range(3)))

    #
    #   plot the results
    #
    timeData = sigmaB1N[:, 0] * macros.NANO2HOUR
    plt.close("all")  # clears out plots from earlier test runs
    figureList = {}

    plt.figure(1)
    s1Data = []
    sigmaB1N = unitTestSupport.removeTimeFromData(sigmaB1N)
    for idx in sigmaB1N:
        sNorm = np.linalg.norm(idx)
        s1Data.append(sNorm)
    plt.plot(timeData, s1Data, color=unitTestSupport.getLineColor(1, 3), label=r'$|\sigma_{B1/N}|$')
    s2Data = []
    sigmaB2N = unitTestSupport.removeTimeFromData(sigmaB2N)
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
    posB1N = unitTestSupport.removeTimeFromData(posB1N)
    posB2N = unitTestSupport.removeTimeFromData(posB2N)
    rhoData = []
    for r1, r2 in zip(posB1N, posB2N):
        rhoData.append(r2 - r1)
    rhoData = np.array(rhoData)
    for idx in range(0, 3):
        plt.plot(timeData, rhoData[:, idx],
                 color=unitTestSupport.getLineColor(idx+1, 3),
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
        1       # attitude coordinate type, 0 - MRP, 1 - quaternions
    )
