''' '''
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
#
#   Unit Test Script
#   Module Name:        simpleSolarPanel
#   Author:             Andrew Harris
#   Creation Date:      July 17th 2019
#

import pytest
import os, inspect
import numpy as np
import math
from matplotlib import pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import simplePowerSink
from Basilisk.simulation import simplePowerMonitor, simpleBattery
from Basilisk.simulation import simMessages
from Basilisk.simulation import simFswInterfaceMessages
from Basilisk.simulation import simpleSolarPanel
from Basilisk.simulation import simMessages
from Basilisk.simulation import simFswInterfaceMessages
from Basilisk.simulation import eclipse
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))



# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.

def run_scenario():
    taskName = "unitTask"               # arbitrary name (don't change)
    processname = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    scenarioSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.
    scenarioSim.TotalSim.terminateSimulation()

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
    oe.a = 6371*1000.0 + 1000.*1000
    oe.e = 0.0
    oe.i = 0.0*macros.D2R

    oe.Omega = 0.0*macros.D2R
    oe.omega = 0.0*macros.D2R
    oe.f     = 0.0*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n

    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)

    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.001], [0.001]]
    scenarioSim.AddModelToTask(taskName, scObject)


    #   Create an eclipse object so the panels don't always work
    eclipseObject = eclipse.Eclipse()
    eclipseObject.addPositionMsgName(scObject.scStateOutMsgName)
    eclipseObject.addPlanetName('earth')

    scenarioSim.AddModelToTask(taskName, eclipseObject)

    # Create a power sink/source
    solarPanel = simpleSolarPanel.SimpleSolarPanel()
    solarPanel.ModelTag = "solarPanel"
    solarPanel.stateInMsgName = scObject.scStateOutMsgName
    solarPanel.sunEclipseInMsgName = "eclipse_data_0"
    solarPanel.setPanelParameters(unitTestSupport.np2EigenVectorXd(np.array([1,0,0])), 0.2*0.3, 0.20)
    solarPanel.nodePowerOutMsgName = "panelPowerMsg"
    scenarioSim.AddModelToTask(taskName, solarPanel)


    # setup Spice interface for some solar system bodies
    timeInitString = '2021 MAY 04 07:47:48.965 (UTC)'
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/'
                                     , timeInitString
                                     , spicePlanetNames = ["sun", "venus", "earth", "mars barycenter"]
                                     )

    scenarioSim.AddModelToTask(taskName, gravFactory.spiceObject, None, -1)



    powerSink = simplePowerSink.SimplePowerSink()
    powerSink.ModelTag = "powerSink2"
    powerSink.nodePowerOut = -3. # Watts
    powerSink.nodePowerOutMsgName = "powerSinkMsg"
    scenarioSim.AddModelToTask(taskName, powerSink)

    # Create a simplePowerMonitor and attach the source/sink to it
    powerMonitor = simpleBattery.SimpleBattery()
    powerMonitor.ModelTag = "powerMonitor"
    powerMonitor.batPowerOutMsgName = "powerMonitorMsg"
    powerMonitor.storageCapacity = 10.0
    powerMonitor.storedCharge = 10.0
    powerMonitor.addPowerNodeToModel(solarPanel.nodePowerOutMsgName)
    powerMonitor.addPowerNodeToModel(powerSink.nodePowerOutMsgName)
    scenarioSim.AddModelToTask(taskName, powerMonitor)


    # Setup logging on the test module output message so that we get all the writes to it
    scenarioSim.TotalSim.logThisMessage(solarPanel.nodePowerOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(powerSink.nodePowerOutMsgName, testProcessRate)
    scenarioSim.TotalSim.logThisMessage(powerMonitor.batPowerOutMsgName, testProcessRate)

    # Need to call the self-init and cross-init methods
    scenarioSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    scenarioSim.ConfigureStopTime(macros.sec2nano(P))        # seconds to stop simulation

    # Begin the simulation time run set above
    scenarioSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    supplyData = scenarioSim.pullMessageLogData(solarPanel.nodePowerOutMsgName + ".netPower_W")
    sinkData = scenarioSim.pullMessageLogData(powerSink.nodePowerOutMsgName + ".netPower_W")
    storageData = scenarioSim.pullMessageLogData(powerMonitor.batPowerOutMsgName + ".storageLevel")
    netData = scenarioSim.pullMessageLogData(powerMonitor.batPowerOutMsgName + ".currentNetPower")
    tvec = supplyData[:,0]
    tvec = tvec * macros.NANO2HOUR

    plt.figure()
    plt.style.use(['aiaa'])
    plt.plot(tvec,storageData[:,1],label='Stored Power (W-Hr)')
    plt.plot(tvec,netData[:,1],label='Net Power (W)')
    plt.plot(tvec,supplyData[:,1],label='Panel Power (W)')
    plt.plot(tvec,sinkData[:,1],label='Power Draw (W)')
    plt.xlabel('Time (Hr)')
    plt.ylabel('Power (W)')
    plt.grid(True)
    plt.legend()
    plt.show()

    return



#
# This statement below ensures that the unitTestScript can be run as a
# stand-alone python script
#
if __name__ == "__main__":
    run_scenario()
