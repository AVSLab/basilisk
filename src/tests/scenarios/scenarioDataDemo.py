'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import os, inspect
import numpy as np
from matplotlib import pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import simpleStorageUnit
from Basilisk.simulation import simpleInstrument
from Basilisk.simulation import eclipse
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import astroFunctions
from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))

def run():
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
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

    #   setup orbit using orbitalMotion library
    oe = orbitalMotion.ClassicElements()
    oe.a = astroFunctions.E_radius*1e3 + 400e3
    oe.e = 0.0
    oe.i = 0.0*macros.D2R

    oe.Omega = 0.0*macros.D2R
    oe.omega = 0.0*macros.D2R
    oe.f     = 75.0*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n

    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)

    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.001], [0.001]]
    scenarioSim.AddModelToTask(taskName, scObject)

    # setup Spice interface for some solar system bodies
    timeInitString = '2021 MAY 04 07:47:48.965 (UTC)'
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/'
                                     , timeInitString
                                     , spicePlanetNames = ["sun", "earth"]
                                     )
    scenarioSim.AddModelToTask(taskName, gravFactory.spiceObject, None, -1)

    # Create an instrument
    simpleInstrument = simpleInstrument.SimpleInstrument()
    simpleInstrument.ModelTag = "instrument1"
    simpleInstrument.nodeBaudRate = 1. #baud
    simpleInstrument.nodeDataName = "Instrument 1". #baud
    simpleInstrument.nodeDataOutMsgName = "dataNodeMsg"
    scenarioSim.AddModelToTask(taskName, simpleInstrument)

    # Create a simpleStorageUnit and attach the instrument to it
    dataMonitor = simpleStorageUnit.SimpleStorageUnit()
    dataMonitor.ModelTag = "dataMonitor"
    dataMonitor.storageUnitDataOutMsgName = "dataMonitorMsg"
    dataMonitor.storageCapacity = 1E5 # bits
    dataMonitor.storedData_Init = 0.0 # bits
    dataMonitor.addDataNodeToModel(simpleInstrument.nodeDataOutMsgName)
    scenarioSim.AddModelToTask(taskName, dataMonitor)

    # Setup logging on the data system
    scenarioSim.TotalSim.logThisMessage(simpleInstrument.nodeDataOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(dataMonitor.storageUnitDataOutMsgName, testProcessRate)

    # Also log attitude/orbit parameters
    scenarioSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(planet.bodyInMsgName, testProcessRate)
    # Need to call the self-init and cross-init methods
    scenarioSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    scenarioSim.ConfigureStopTime(macros.sec2nano(P))        # seconds to stop simulation

    # Begin the simulation time run set above
    scenarioSim.ExecuteSimulation()

    print(dataMonitor.getStoredData())

    # # This pulls the actual data log from the simulation run.
    # # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    # supplyData = scenarioSim.pullMessageLogData(simpleInstrument.nodeDataOutMsgName + ".netData")
    # storageData = scenarioSim.pullMessageLogData(dataMonitor.storageUnitDataOutMsgName + ".storageLevel")
    #
    # scOrbit = scenarioSim.pullMessageLogData(scObject.scStateOutMsgName + ".r_BN_N", list(range(3)))
    # scAtt = scenarioSim.pullMessageLogData(scObject.scStateOutMsgName+".sigma_BN", list(range(3)))
    #
    # planetOrbit = scenarioSim.pullMessageLogData(planet.bodyInMsgName+".PositionVector", list(range(3)))
    #
    # tvec = supplyData[:,0]
    # tvec = tvec * macros.NANO2HOUR
    #
    # #   Plot the data states
    # # Stopped here. Revisiting instrument implementation first.
    # figureList = {}
    # plt.close("all")  # clears out plots from earlier test runs
    # plt.figure(1)
    # plt.plot(tvec,storageData[:,1]/3600.,label='Stored Power (W-Hr)')
    # plt.plot(tvec,netData[:,1],label='Net Power (W)')
    # plt.plot(tvec,supplyData[:,1],label='Panel Power (W)')
    # plt.plot(tvec,sinkData[:,1],label='Power Draw (W)')
    # plt.xlabel('Time (Hr)')
    # plt.ylabel('Power (W)')
    # plt.grid(True)
    # plt.legend()
    #
    # pltName = "scenario_powerDemo"
    # figureList[pltName] = plt.figure(1)
    #
    # return figureList

#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    fig = run()
    plt.show()
