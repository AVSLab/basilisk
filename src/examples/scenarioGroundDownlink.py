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

This scenario is intended to provide both an overview and a concrete demonstration of the features and interface of
:ref:`GroundLocation`, which represents a specific ground location and computes visibility from that location to spacecraft,
and :ref:`spaceToGroundTransmitter`, which represents a spacecraft-based radio system that requires
visibility to a ground station.

The script is found in the folder ``src/examples`` and executed by using::

      python3 scenarioDataDemo.py

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
import os, inspect
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import simpleInstrument, simpleStorageUnit, partitionedStorageUnit, spaceToGroundTransmitter
from Basilisk.simulation import groundLocation

from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import astroFunctions
from Basilisk.utilities import vizSupport
from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))

def run(show_plots):
    taskName = "unitTask"               # arbitrary name (don't change)
    processname = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    scenarioSim = SimulationBaseClass.SimBaseClass()
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.

    # Create test thread
    testProcessRate = macros.sec2nano(1.0)     # update process rate update time
    testProc = scenarioSim.CreateNewProcess(processname)
    testProc.addTask(scenarioSim.CreateNewTask(taskName, testProcessRate))

    # Create a spacecraft around Earth
    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True  # ensure this is the central gravitational body

    planet.useSphericalHarmParams = True
    simIncludeGravBody.loadGravFromFile(bskPath + '/supportData/LocalGravData/GGM03S-J2-only.txt',
                                        planet.spherHarm, 2)
    mu = planet.mu

    # setup Spice interface for some solar system bodies
    timeInitString = '2020 MAY 21 18:28:03 (UTC)'
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/'
                                     , timeInitString
                                     )
    scenarioSim.AddModelToTask(taskName, gravFactory.spiceObject, None, -1)


    # The value 2 indicates that the first two harmonics, exclud

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
    scenarioSim.AddModelToTask(taskName, scObject)

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Create the ground location
    groundStation = groundLocation.GroundLocation()
    groundStation.ModelTag = "BoulderGroundStation"
    groundStation.currentGroundStateOutMsgName = "BoulderGroundStation_Location"
    groundStation.planetRadius = astroFunctions.E_radius*1e3
    groundStation.specifyLocation(np.radians(40.009971), np.radians(-105.243895), 1624)
    groundStation.planetInMsgName = planet.bodyInMsgName
    groundStation.minimumElevation = np.radians(10.)
    groundStation.maximumRange = 1e9
    groundStation.addSpacecraftToModel(scObject.scStateOutMsgName)
    scenarioSim.AddModelToTask(taskName, groundStation)

    # Create an instrument
    instrument = simpleInstrument.SimpleInstrument()
    instrument.ModelTag = "instrument1"
    instrument.nodeBaudRate = 2400. # baud
    instrument.nodeDataName = "Instrument 1" # baud
    instrument.nodeDataOutMsgName = "Instrument1Msg"
    scenarioSim.AddModelToTask(taskName, instrument)

    # Create another instrument
    instrument2 = simpleInstrument.SimpleInstrument()
    instrument2.ModelTag = "instrument2"
    instrument2.nodeBaudRate = 2400. # baud
    instrument2.nodeDataName = "Instrument 2" # baud
    instrument2.nodeDataOutMsgName = "Instrument2Msg"
    scenarioSim.AddModelToTask(taskName, instrument2)

    # Create a "transmitter"
    transmitter = spaceToGroundTransmitter.SpaceToGroundTransmitter()
    transmitter.ModelTag = "transmitter"
    transmitter.nodeBaudRate = -9600.   # baud
    transmitter.packetSize = -1E6   # bits
    transmitter.numBuffers = 2
    transmitter.nodeDataOutMsgName = "TransmitterMsg"
    transmitter.addAccessMsgToTransmitter(groundStation.accessOutMsgNames[-1])
    scenarioSim.AddModelToTask(taskName, transmitter)

    # Create a partitionedStorageUnit and attach the instrument to it
    dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageUnitDataOutMsgName = "dataMonitorMsg"
    dataMonitor.storageCapacity = 8E9   # bits (1 GB)
    dataMonitor.addDataNodeToModel(instrument.nodeDataOutMsgName)
    dataMonitor.addDataNodeToModel(instrument2.nodeDataOutMsgName)
    dataMonitor.addDataNodeToModel(transmitter.nodeDataOutMsgName)
    scenarioSim.AddModelToTask(taskName, dataMonitor)

    transmitter.addStorageUnitToTransmitter(dataMonitor.storageUnitDataOutMsgName)

    # Create a simpleStorageUnit and attach the instrument to it
    dataMonitor2 = simpleStorageUnit.SimpleStorageUnit()
    dataMonitor2.ModelTag = "dataMonitor2"
    dataMonitor2.storageUnitDataOutMsgName = "dataMonitorMsg2"
    dataMonitor2.storageCapacity = 1E5 # bits
    dataMonitor2.addDataNodeToModel(instrument.nodeDataOutMsgName)
    dataMonitor2.addDataNodeToModel(instrument2.nodeDataOutMsgName)
    dataMonitor2.addDataNodeToModel(transmitter.nodeDataOutMsgName)
    scenarioSim.AddModelToTask(taskName, dataMonitor2)

    # Setup logging on the data system
    scenarioSim.TotalSim.logThisMessage(instrument.nodeDataOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(dataMonitor.storageUnitDataOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(dataMonitor2.storageUnitDataOutMsgName, testProcessRate)

    # Also log attitude/orbit parameters
    scenarioSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(planet.bodyInMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(groundStation.currentGroundStateOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(groundStation.accessOutMsgNames[-1], testProcessRate)

    # setup Vizard support
    viz = vizSupport.enableUnityVisualization(scenarioSim, taskName, processname
                                              , saveFile=__file__
                                              , gravBodies=gravFactory
                                              )
    vizSupport.addGroundLocation(viz, name="Boulder Station"
                                 , parentBodyName='earth'
                                 , r_GP_P=groundStation.r_LP_P_Init
                                 , fieldOfView=np.radians(160.)
                                 , color='pink'
                                 , sprite='STAR'
                                 )

    # Need to call the self-init and cross-init methods
    scenarioSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    scenarioSim.ConfigureStopTime(macros.sec2nano(86400))        # seconds to stop simulation

    # Begin the simulation time run set above
    scenarioSim.ExecuteSimulation()

    # Grabbed logged data for plotting
    storageLevel = scenarioSim.pullMessageLogData(dataMonitor.storageUnitDataOutMsgName + '.storageLevel')
    storageNetBaud = scenarioSim.pullMessageLogData(dataMonitor.storageUnitDataOutMsgName + '.currentNetBaud')
    storedData = scenarioSim.pullMessageLogData(dataMonitor.storageUnitDataOutMsgName + '.storedData', list(range(2)))
    scPosition = scenarioSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',list(range(3)))
    groundPosition = scenarioSim.pullMessageLogData(groundStation.currentGroundStateOutMsgName+'.r_LP_N', range(3))
    earthPosition = scenarioSim.pullMessageLogData(planet.bodyInMsgName+'.PositionVector',range(3))
    accessData = scenarioSim.pullMessageLogData(groundStation.accessOutMsgNames[-1]+'.hasAccess')
    elevationData = scenarioSim.pullMessageLogData(groundStation.accessOutMsgNames[-1]+'.elevation')
    azimuthData = scenarioSim.pullMessageLogData(groundStation.accessOutMsgNames[-1]+'.azimuth')
    rangeData = scenarioSim.pullMessageLogData(groundStation.accessOutMsgNames[-1]+'.slantRange')

    scPosition = scPosition - earthPosition

    pass_inds = np.nonzero(accessData[:,1])
    pass_az = azimuthData[pass_inds,1]
    pass_el = elevationData[pass_inds,1]



    tvec = storageLevel[:,0]
    tvec = tvec * macros.NANO2HOUR

    #   Plot the data states
    # Stopped here. Revisiting instrument implementation first.
    figureList = {}
    plt.close("all")  # clears out plots from earlier test runs
    fig=plt.figure(1)
    plt.plot(tvec, storageLevel[:, 1]/(8E3), label='Data Unit Total Storage Level (KB)')
    plt.plot(tvec, storedData[:, 1]/(8E3), label='Instrument 1 Partition Level (KB)')
    plt.plot(tvec, storedData[:, 2]/(8E3), label='Instrument 2 Partition Level (KB)')
    plt.xlabel('Time (Hr)')
    plt.ylabel('Data Stored (KB)')
    plt.grid(True)
    plt.legend()
    figureList['scenarioGroundPassStorage'] = fig

    #   Plot the orbit and ground station location data
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.plot(scPosition[:,1]/1000.,scPosition[:, 2]/1000.,scPosition[:,3]/1000., label='S/C Position')
    ax.plot(groundPosition[:,1]/1000.,groundPosition[:, 2]/1000.,groundPosition[:,3]/1000., label='Ground Station Position')
    plt.legend()
    figureList['scenarioGroundPassECI'] = fig

    fig = plt.figure()
    plt.polar(pass_az[0,:], 90.-np.degrees(pass_el[0,:]))
    # ax.set_yticks(range(0, 90, 10))  # Define the yticks
    # ax.set_yticklabels(map(str, range(90, 0, -10)))
    plt.title('Ground Pass Azimuth and Declination')
    figureList['scenarioGroundPassPolar'] = fig

    plt.figure()
    plt.plot(tvec, np.degrees(azimuthData[:, 1]),label='az')
    plt.plot(tvec, np.degrees(elevationData[:, 1]), label='el')
    plt.legend()
    plt.grid(True)
    plt.ylabel('Angles (deg)')
    plt.xlabel('Time (hr)')

    fig=plt.figure()
    plt.plot(tvec, rangeData[:, 1]/1000.)
    plt.plot(tvec, accessData[:, 1]*1000.)
    plt.grid(True)
    plt.title('Slant Range, Access vs. Time')
    plt.ylabel('Slant Range (km)')
    plt.xlabel('Time (hr)')
    figureList['scenarioGroundPassRange'] = fig

    fig = plt.figure()
    plt.plot(tvec,storageNetBaud[:, 1] / (8E3), label='Net Baud Rate (KB/s)')
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
