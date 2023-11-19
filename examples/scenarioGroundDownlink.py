# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""
Overview
--------

This scenario demonstrates how the on-board power system can be used to simulate data down-link that is dependent on access
to specific geographic locations (i.e., ground stations).

.. image:: /_images/static/scenarioGroundDownlink.jpg
   :align: center

This scenario is intended to provide both an overview and a concrete demonstration of the features and interface of
:ref:`GroundLocation`, which represents a specific ground location and computes visibility from that location to spacecraft,
and :ref:`spaceToGroundTransmitter`, which represents a spacecraft-based radio system that requires
visibility to a ground station.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioGroundDownlink.py

The scenario is meant to be representative of a small satellite with constant data collection attempting to
downlink data to a ground station located in Boulder, Colorado.

When the simulation completes, the following plots are shown to
demonstrate the data stored, generated, and downlinked.

.. image:: /_images/Scenarios/scenarioGroundPassECI.svg
   :align: center

.. image:: /_images/Scenarios/scenarioGroundPassPolar.svg
   :align: center
   
.. image:: /_images/Scenarios/scenarioGroundPassRange.svg
   :align: center

.. image:: /_images/Scenarios/scenarioGroundPassBaud.svg
   :align: center

.. image:: /_images/Scenarios/scenarioGroundPassStorage.svg
   :align: center
"""
import inspect
import os

import numpy as np
from matplotlib import pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import simpleInstrument, simpleStorageUnit, partitionedStorageUnit, spaceToGroundTransmitter
from Basilisk.simulation import groundLocation
from Basilisk.utilities import vizSupport
from Basilisk.utilities import unitTestSupport

from Basilisk.simulation import spacecraft
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import astroFunctions

from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))

def run(show_plots):
    taskName = "unitTask"               # arbitrary name (don't change)
    processname = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    scenarioSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(10.0)     # update process rate update time
    testProc = scenarioSim.CreateNewProcess(processname)
    testProc.addTask(scenarioSim.CreateNewTask(taskName, testProcessRate))

    # Create a spacecraft around Earth
    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True  # ensure this is the central gravitational body
    planet.useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/GGM03S-J2-only.txt', 2)
    mu = planet.mu
    # setup Spice interface for some solar system bodies
    timeInitString = '2020 MAY 21 18:28:03 (UTC)'
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/'
                                     , timeInitString
                                     )
    scenarioSim.AddModelToTask(taskName, gravFactory.spiceObject, -1)


    #   setup orbit using orbitalMotion library
    oe = orbitalMotion.ClassicElements()
    oe.a = astroFunctions.E_radius*1e3 + 418e3
    oe.e = 0.00061
    oe.i = 51.6418*macros.D2R

    oe.Omega = 119.2314*macros.D2R
    oe.omega = 	337.8329*macros.D2R
    oe.f     = 22.2753*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n

    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN

    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.000], [-0.000], [0.000]]
    scenarioSim.AddModelToTask(taskName, scObject, 1)

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Create the ground location
    groundStation = groundLocation.GroundLocation()
    groundStation.ModelTag = "BoulderGroundStation"
    groundStation.planetRadius = astroFunctions.E_radius*1e3
    groundStation.specifyLocation(np.radians(40.009971), np.radians(-105.243895), 1624)
    groundStation.planetInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[0])
    groundStation.minimumElevation = np.radians(10.)
    groundStation.maximumRange = 1e9
    groundStation.addSpacecraftToModel(scObject.scStateOutMsg)
    scenarioSim.AddModelToTask(taskName, groundStation)

    # Create an instrument
    instrument = simpleInstrument.SimpleInstrument()
    instrument.ModelTag = "instrument1"
    instrument.nodeBaudRate = 2400. # baud
    instrument.nodeDataName = "Instrument 1" # baud
    scenarioSim.AddModelToTask(taskName, instrument)

    # Create another instrument
    instrument2 = simpleInstrument.SimpleInstrument()
    instrument2.ModelTag = "instrument2"
    instrument2.nodeBaudRate = 2400. # baud
    instrument2.nodeDataName = "Instrument 2"  # baud
    scenarioSim.AddModelToTask(taskName, instrument2)

    # Create a "transmitter"
    transmitter = spaceToGroundTransmitter.SpaceToGroundTransmitter()
    transmitter.ModelTag = "transmitter"
    transmitter.nodeBaudRate = -9600.   # baud
    transmitter.packetSize = -1E6   # bits
    transmitter.numBuffers = 2
    transmitter.addAccessMsgToTransmitter(groundStation.accessOutMsgs[-1])
    scenarioSim.AddModelToTask(taskName, transmitter)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageCapacity = 8E9   # bits (1 GB)
    dataMonitor.addDataNodeToModel(instrument.nodeDataOutMsg)
    dataMonitor.addDataNodeToModel(instrument2.nodeDataOutMsg)
    dataMonitor.addDataNodeToModel(transmitter.nodeDataOutMsg)
    dataMonitor.addPartition("Instrument 1")
    dataMonitor.addPartition("Instrument 2")
    scenarioSim.AddModelToTask(taskName, dataMonitor)

    transmitter.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsg)

    # Create a simpleStorageUnit and attach the instrument to it
    dataMonitor2 = simpleStorageUnit.SimpleStorageUnit()
    dataMonitor2.ModelTag = "dataMonitor2"
    dataMonitor2.storageCapacity = 1E5  # bits
    dataMonitor2.addDataNodeToModel(instrument.nodeDataOutMsg)
    dataMonitor2.addDataNodeToModel(instrument2.nodeDataOutMsg)
    dataMonitor2.addDataNodeToModel(transmitter.nodeDataOutMsg)
    scenarioSim.AddModelToTask(taskName, dataMonitor2)

    # Setup logging on the data system
    instLog = instrument.nodeDataOutMsg.recorder()
    dataUnitLog = dataMonitor.storageUnitDataOutMsg.recorder()
    dataUnitLog2 = dataMonitor2.storageUnitDataOutMsg.recorder()
    scenarioSim.AddModelToTask(taskName, instLog)
    scenarioSim.AddModelToTask(taskName, dataUnitLog)
    scenarioSim.AddModelToTask(taskName, dataUnitLog2)

    # Also log attitude/orbit parameters
    dataLog = scObject.scStateOutMsg.recorder()
    plLog = gravFactory.spiceObject.planetStateOutMsgs[0].recorder()
    gsLog = groundStation.currentGroundStateOutMsg.recorder()
    gsAccessLog = groundStation.accessOutMsgs[-1].recorder()
    scenarioSim.AddModelToTask(taskName, dataLog)
    scenarioSim.AddModelToTask(taskName, plLog)
    scenarioSim.AddModelToTask(taskName, gsLog)
    scenarioSim.AddModelToTask(taskName, gsAccessLog)

    # setup Vizard support
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scenarioSim, taskName, scObject
                                                  # , saveFile=__file__
                                                  )
        vizSupport.addLocation(viz, stationName="Boulder Station"
                               , parentBodyName=planet.planetName
                               , r_GP_P=unitTestSupport.EigenVector3d2list(groundStation.r_LP_P_Init)
                               , fieldOfView=np.radians(160.)
                               , color='pink'
                               , range=1000.0*1000  # meters
                               )
        viz.settings.spacecraftSizeMultiplier = 1.5
        viz.settings.showLocationCommLines = 1
        viz.settings.showLocationCones = 1
        viz.settings.showLocationLabels = 1

    # Need to call the self-init and cross-init methods
    scenarioSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    scenarioSim.ConfigureStopTime(macros.hour2nano(24))        # seconds to stop simulation

    # Begin the simulation time run set above
    scenarioSim.ExecuteSimulation()

    # Grabbed logged data for plotting
    storageLevel = dataUnitLog.storageLevel
    storageNetBaud = dataUnitLog.currentNetBaud
    storedData = dataUnitLog.storedData
    scPosition = dataLog.r_BN_N
    groundPosition = gsLog.r_LP_N
    earthPosition = plLog.PositionVector
    accessData = gsAccessLog.hasAccess
    elevationData = gsAccessLog.elevation
    azimuthData = gsAccessLog.azimuth
    rangeData = gsAccessLog.slantRange

    scPosition = scPosition - earthPosition

    pass_inds = np.nonzero(accessData)
    pass_az = azimuthData[pass_inds]
    pass_el = elevationData[pass_inds]

    tvec = dataUnitLog.times()
    tvec = tvec * macros.NANO2HOUR

    #   Plot the data states
    # Stopped here. Revisiting instrument implementation first.
    figureList = {}
    plt.close("all")  # clears out plots from earlier test runs
    fig=plt.figure(1)
    plt.plot(tvec, storageLevel/(8E3), label='Data Unit Total Storage Level (KB)')
    plt.plot(tvec, storedData[:, 0]/(8E3), label='Instrument 1 Partition Level (KB)')
    plt.plot(tvec, storedData[:, 1]/(8E3), label='Instrument 2 Partition Level (KB)')
    plt.xlabel('Time (Hr)')
    plt.ylabel('Data Stored (KB)')
    plt.grid(True)
    plt.legend()
    figureList['scenarioGroundPassStorage'] = fig

    #   Plot the orbit and ground station location data
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.plot(scPosition[:,0]/1000.,scPosition[:, 1]/1000.,scPosition[:,2]/1000., label='S/C Position')
    ax.plot(groundPosition[:,0]/1000.,groundPosition[:, 0]/1000.,groundPosition[:,2]/1000., label='Ground Station Position')
    plt.legend()
    figureList['scenarioGroundPassECI'] = fig

    fig = plt.figure()
    plt.polar(pass_az, 90.-np.degrees(pass_el))
    # ax.set_yticks(range(0, 90, 10))  # Define the yticks
    # ax.set_yticklabels(map(str, range(90, 0, -10)))
    plt.title('Ground Pass Azimuth and Declination')
    figureList['scenarioGroundPassPolar'] = fig

    plt.figure()
    plt.plot(tvec, np.degrees(azimuthData),label='az')
    plt.plot(tvec, np.degrees(elevationData), label='el')
    plt.legend()
    plt.grid(True)
    plt.ylabel('Angles (deg)')
    plt.xlabel('Time (hr)')

    fig=plt.figure()
    plt.plot(tvec, rangeData/1000.)
    plt.plot(tvec, accessData*1000.)
    plt.grid(True)
    plt.title('Slant Range, Access vs. Time')
    plt.ylabel('Slant Range (km)')
    plt.xlabel('Time (hr)')
    figureList['scenarioGroundPassRange'] = fig

    fig = plt.figure()
    plt.plot(tvec,storageNetBaud / (8E3), label='Net Baud Rate (KB/s)')
    plt.xlabel('Time (Hr)')
    plt.ylabel('Data Rate (KB/s)')
    plt.grid(True)
    plt.legend()
    figureList['scenarioGroundPassBaud'] = fig

    if show_plots:
        plt.show()
    plt.close("all")

    return figureList

# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script

if __name__ == "__main__":
    run(
        True  # show_plots
    )
