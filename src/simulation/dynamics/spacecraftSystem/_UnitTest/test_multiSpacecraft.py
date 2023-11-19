
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


import inspect
import os

import numpy
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.simulation import spacecraftSystem
from Basilisk.utilities import macros
from Basilisk.utilities import pythonVariableLogger
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import hingedRigidBodyStateEffector


def addTimeColumn(time, data):
    return numpy.transpose(numpy.vstack([[time], numpy.transpose(data)]))


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("function", ["SCConnected"
                                      , "SCConnectedAndUnconnected"
                                      ])
def test_spacecraftSystemAllTest(show_plots, function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '(show_plots)')
    assert testResults < 1, testMessage


def SCConnected(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scSystem = spacecraftSystem.SpacecraftSystem()
    scSystem.ModelTag = "spacecraftSystem"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scSystem)

    # Define initial conditions of primary spacecraft
    scSystem.primaryCentralSpacecraft.hub.mHub = 100
    scSystem.primaryCentralSpacecraft.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scSystem.primaryCentralSpacecraft.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    scSystem.primaryCentralSpacecraft.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    scSystem.primaryCentralSpacecraft.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    scSystem.primaryCentralSpacecraft.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scSystem.primaryCentralSpacecraft.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    # Define docking information
    dock1SC1 = spacecraftSystem.DockingData()
    dock1SC1.r_DB_B = [[1.0], [0.0], [0.0]]
    dock1SC1.dcm_DB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dock1SC1.portName = "sc1port1"
    scSystem.primaryCentralSpacecraft.addDockingPort(dock1SC1)

    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 100.0
    unitTestSim.panel1.c = 0.0
    unitTestSim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.thetaInit = 5*numpy.pi/180.0
    unitTestSim.panel1.thetaDotInit = 0.0

    scSystem.primaryCentralSpacecraft.addStateEffector(unitTestSim.panel1)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.planetName = "earth_planet_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unitTestSim.earthGravBody.isCentralBody = True

    scSystem.primaryCentralSpacecraft.gravField.gravBodies = spacecraftSystem.GravBodyVector([unitTestSim.earthGravBody])

    sc2 = spacecraftSystem.SpacecraftUnit()
    sc2.hub.mHub = 100
    sc2.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc2.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc2.spacecraftName = "spacecraft2"

    # Define docking information
    dock1SC2 = spacecraftSystem.DockingData()
    dock1SC2.r_DB_B = [[-1.0], [0.0], [0.0]]
    dock1SC2.dcm_DB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dock1SC2.portName = "sc2port1"
    sc2.addDockingPort(dock1SC2)

    # Define docking information
    dock2SC2 = spacecraftSystem.DockingData()
    dock2SC2.r_DB_B = [[1.0], [0.0], [0.0]]
    dock2SC2.dcm_DB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dock2SC2.portName = "sc2port2"
    sc2.addDockingPort(dock2SC2)

    # Define gravity for sc2
    sc2.gravField.gravBodies = spacecraftSystem.GravBodyVector([unitTestSim.earthGravBody])

    sc3 = spacecraftSystem.SpacecraftUnit()
    sc3.hub.mHub = 100
    sc3.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc3.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc3.spacecraftName = "spacecraft3"

    # Define docking information
    dock1SC3 = spacecraftSystem.DockingData()
    dock1SC3.r_DB_B = [[-1.0], [0.0], [0.0]]
    dock1SC3.dcm_DB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dock1SC3.portName = "sc3port1"
    sc3.addDockingPort(dock1SC3)

    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 100.0
    unitTestSim.panel2.c = 0.0
    unitTestSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.thetaInit = 0.0
    unitTestSim.panel2.thetaDotInit = 0.0

    sc3.addStateEffector(unitTestSim.panel2)

    # Define gravity for sc2
    sc3.gravField.gravBodies = spacecraftSystem.GravBodyVector([unitTestSim.earthGravBody])

    # Attach spacecraft2 to spacecraft
    scSystem.attachSpacecraftToPrimary(sc2, dock1SC2.portName, dock1SC1.portName)

    # Attach spacecraft3 to spacecraft2
    scSystem.attachSpacecraftToPrimary(sc3, dock1SC3.portName, dock2SC2.portName)

    dataLog = scSystem.primaryCentralSpacecraft.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    scLog = pythonVariableLogger.PythonVariableLogger({
        "totOrbEnergy": lambda _: scSystem.primaryCentralSpacecraft.totOrbEnergy,
        "totOrbAngMomPntN_N": lambda _: scSystem.primaryCentralSpacecraft.totOrbAngMomPntN_N,
        "totRotAngMomPntC_N": lambda _: scSystem.primaryCentralSpacecraft.totRotAngMomPntC_N,
        "totRotEnergy": lambda _: scSystem.primaryCentralSpacecraft.totRotEnergy,
    })
    unitTestSim.AddModelToTask(unitTaskName, scLog)

    unitTestSim.InitializeSimulation()

    stopTime = 1.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbEnergy = unitTestSupport.addTimeColumn(scLog.times(), scLog.totOrbEnergy)
    orbAngMom_N = unitTestSupport.addTimeColumn(scLog.times(), scLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scLog.times(), scLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scLog.times(), scLog.totRotEnergy)

    r_BN_NOutput = dataLog.r_BN_N
    sigma_BNOutput = dataLog.sigma_BN

    truePos = [
                [-4072255.7737936215, 7456050.4649078, 5258610.029627514]
                ]

    trueSigma = [
                [3.73034285e-01,  -2.39564413e-03,   2.08570797e-01]
                ]

    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1,1], rotAngMom_N[-1,2], rotAngMom_N[-1,3]]
                 ]

    initialOrbEnergy = [
                [orbEnergy[0,1]]
                ]

    finalOrbEnergy = [
                [orbEnergy[-1,1]]
                 ]

    initialRotEnergy = [
                [rotEnergy[0,1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1,1]]
                 ]

    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInOrbitalAngularMomentumSystem"
    PlotTitle = "Change in Orbital Angular Momentum with Gravity"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInOrbitalEnergySystem"
    PlotTitle = "Change in Orbital Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInRotationalAngularMomentumSystem"
    PlotTitle = "Change In Rotational Angular Momentum with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInRotationalEnergySystem"
    PlotTitle = "Change In Rotational Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
    if show_plots:
        plt.show()
        plt.close('all')

    accuracy = 1e-8
    # for i in range(0,len(truePos)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(r_BN_NOutput[-1,:],truePos[i],3,accuracy):
    #         testFailCount += 1
    #         testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed pos unit test")
    #
    # for i in range(0,len(trueSigma)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(sigma_BNOutput[-1,:],trueSigma[i],3,accuracy):
    #         testFailCount += 1
    #         testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed attitude unit test")

    accuracy = 1e-10
    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed orbital angular momentum unit test")

    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational angular momentum unit test")

    for i in range(0,len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational energy unit test")

    for i in range(0,len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed orbital energy unit test")

    if testFailCount == 0:
        print("PASSED: " + " Spacecraft Translation and Rotation Integrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def SCConnectedAndUnconnected(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scSystem = spacecraftSystem.SpacecraftSystem()
    scSystem.ModelTag = "spacecraftSystem"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scSystem)

    # Define initial conditions of primary spacecraft
    scSystem.primaryCentralSpacecraft.hub.mHub = 100
    scSystem.primaryCentralSpacecraft.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scSystem.primaryCentralSpacecraft.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    scSystem.primaryCentralSpacecraft.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    scSystem.primaryCentralSpacecraft.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    scSystem.primaryCentralSpacecraft.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scSystem.primaryCentralSpacecraft.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    # Define docking information
    dock1SC1 = spacecraftSystem.DockingData()
    dock1SC1.r_DB_B = [[1.0], [0.0], [0.0]]
    dock1SC1.dcm_DB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dock1SC1.portName = "sc1port1"
    scSystem.primaryCentralSpacecraft.addDockingPort(dock1SC1)

    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 100.0
    unitTestSim.panel1.c = 0.0
    unitTestSim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.thetaInit = 5*numpy.pi/180.0
    unitTestSim.panel1.thetaDotInit = 0.0

    scSystem.primaryCentralSpacecraft.addStateEffector(unitTestSim.panel1)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.planetName = "earth_planet_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unitTestSim.earthGravBody.isCentralBody = True

    scSystem.primaryCentralSpacecraft.gravField.gravBodies = spacecraftSystem.GravBodyVector([unitTestSim.earthGravBody])

    sc2 = spacecraftSystem.SpacecraftUnit()
    sc2.hub.mHub = 100
    sc2.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc2.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc2.spacecraftName = "spacecraft2"

    # Define docking information
    dock1SC2 = spacecraftSystem.DockingData()
    dock1SC2.r_DB_B = [[-1.0], [0.0], [0.0]]
    dock1SC2.dcm_DB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dock1SC2.portName = "sc2port1"
    sc2.addDockingPort(dock1SC2)

    # Define docking information
    dock2SC2 = spacecraftSystem.DockingData()
    dock2SC2.r_DB_B = [[1.0], [0.0], [0.0]]
    dock2SC2.dcm_DB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dock2SC2.portName = "sc2port2"
    sc2.addDockingPort(dock2SC2)

    # Define gravity for sc2
    sc2.gravField.gravBodies = spacecraftSystem.GravBodyVector([unitTestSim.earthGravBody])

    sc3 = spacecraftSystem.SpacecraftUnit()
    sc3.hub.mHub = 100
    sc3.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc3.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc3.spacecraftName = "spacecraft3"

    # Define docking information
    dock1SC3 = spacecraftSystem.DockingData()
    dock1SC3.r_DB_B = [[-1.0], [0.0], [0.0]]
    dock1SC3.dcm_DB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dock1SC3.portName = "sc3port1"
    sc3.addDockingPort(dock1SC3)

    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 100.0
    unitTestSim.panel2.c = 0.0
    unitTestSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.thetaInit = 0.0
    unitTestSim.panel2.thetaDotInit = 0.0

    sc3.addStateEffector(unitTestSim.panel2)

    # Define gravity for sc2
    sc3.gravField.gravBodies = spacecraftSystem.GravBodyVector([unitTestSim.earthGravBody])

    # Attach spacecraft2 to spacecraft
    scSystem.attachSpacecraftToPrimary(sc2, dock1SC2.portName, dock1SC1.portName)

    # Attach spacecraft3 to spacecraft2
    scSystem.attachSpacecraftToPrimary(sc3, dock1SC3.portName, dock2SC2.portName)

    # Define two independent spacecraft
    sc4 = spacecraftSystem.SpacecraftUnit()
    sc4.hub.mHub = 100
    sc4.hub.r_BcB_B = [[0.0], [0.0], [0.1]]
    sc4.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc4.hub.r_CN_NInit = [[7490566.741852513],[-4020338.690396649],[5248299.211589362]]
    sc4.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    sc4.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc4.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]
    sc4.spacecraftName = "spacecraft4"

    # Define gravity for sc4
    sc4.gravField.gravBodies = spacecraftSystem.GravBodyVector([unitTestSim.earthGravBody])

    unitTestSim.panel3 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variables for panel 1 on sc4
    unitTestSim.panel3.mass = 100.0
    unitTestSim.panel3.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel3.d = 1.5
    unitTestSim.panel3.k = 100.0
    unitTestSim.panel3.c = 0.0
    unitTestSim.panel3.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel3.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel3.thetaInit = 0.0
    unitTestSim.panel3.thetaDotInit = 0.0

    sc4.addStateEffector(unitTestSim.panel3)

    scSystem.addSpacecraftUndocked(sc4)

    sc5 = spacecraftSystem.SpacecraftUnit()
    sc5.hub.mHub = 100
    sc5.hub.r_BcB_B = [[0.1], [0.0], [0.0]]
    sc5.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc5.hub.r_CN_NInit = [[5248299.211589362],[7490566.741852513],[-4020338.690396649]]
    sc5.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    sc5.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc5.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]
    sc5.spacecraftName = "spacecraft5"

    # Define gravity for sc4
    sc5.gravField.gravBodies = spacecraftSystem.GravBodyVector([unitTestSim.earthGravBody])

    scSystem.addSpacecraftUndocked(sc5)

    dataLog = scSystem.primaryCentralSpacecraft.scStateOutMsg.recorder()
    dataLog4 = sc4.scStateOutMsg.recorder()
    dataLog5 = sc5.scStateOutMsg.recorder()
    dataEngLog = scSystem.primaryCentralSpacecraft.scEnergyMomentumOutMsg.recorder()
    dataEngLog4 = sc4.scEnergyMomentumOutMsg.recorder()
    dataEngLog5 = sc5.scEnergyMomentumOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    unitTestSim.AddModelToTask(unitTaskName, dataLog4)
    unitTestSim.AddModelToTask(unitTaskName, dataLog5)
    unitTestSim.AddModelToTask(unitTaskName, dataEngLog)
    unitTestSim.AddModelToTask(unitTaskName, dataEngLog4)
    unitTestSim.AddModelToTask(unitTaskName, dataEngLog5)

    unitTestSim.InitializeSimulation()

    stopTime = 1.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    r_BN_NOutput = addTimeColumn(dataLog.times(), dataLog.r_BN_N)
    sigma_BNOutput = addTimeColumn(dataLog.times(), dataLog.sigma_BN)
    r_BN_NOutput1 = addTimeColumn(dataLog4.times(), dataLog4.r_BN_N)
    sigma_BNOutput1 = addTimeColumn(dataLog4.times(), dataLog4.sigma_BN)
    r_BN_NOutput2 = addTimeColumn(dataLog5.times(), dataLog5.r_BN_N)
    sigma_BNOutput2 = addTimeColumn(dataLog5.times(), dataLog5.sigma_BN)

    rotEnergy = addTimeColumn(dataEngLog.times(), dataEngLog.spacecraftRotEnergy)
    orbEnergy = addTimeColumn(dataEngLog.times(), dataEngLog.spacecraftOrbEnergy)
    rotAngMom_N = addTimeColumn(dataEngLog.times(), dataEngLog.spacecraftRotAngMomPntC_N)
    orbAngMom_N = addTimeColumn(dataEngLog.times(), dataEngLog.spacecraftOrbAngMomPntN_N)

    rotEnergy1 = addTimeColumn(dataEngLog4.times(), dataEngLog4.spacecraftRotEnergy)
    orbEnergy1 = addTimeColumn(dataEngLog4.times(), dataEngLog4.spacecraftOrbEnergy)
    rotAngMom1_N = addTimeColumn(dataEngLog4.times(), dataEngLog4.spacecraftRotAngMomPntC_N)
    orbAngMom1_N = addTimeColumn(dataEngLog4.times(), dataEngLog4.spacecraftOrbAngMomPntN_N)

    rotEnergy2 = addTimeColumn(dataEngLog5.times(), dataEngLog5.spacecraftRotEnergy)
    orbEnergy2 = addTimeColumn(dataEngLog5.times(), dataEngLog5.spacecraftOrbEnergy)
    rotAngMom2_N = addTimeColumn(dataEngLog5.times(), dataEngLog5.spacecraftRotAngMomPntC_N)
    orbAngMom2_N = addTimeColumn(dataEngLog5.times(), dataEngLog5.spacecraftOrbAngMomPntN_N)

    truePos = [
                [-4072255.7737936215, 7456050.4649078, 5258610.029627514]
                ]

    trueSigma = [
                [3.73034285e-01,  -2.39564413e-03,   2.08570797e-01]
                ]

    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1,1], rotAngMom_N[-1,2], rotAngMom_N[-1,3]]
                 ]

    initialOrbEnergy = [
                [orbEnergy[0,1]]
                ]

    finalOrbEnergy = [
                [orbEnergy[-1,1]]
                 ]

    initialRotEnergy = [
                [rotEnergy[0,1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1,1]]
                 ]

    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInOrbitalAngularMomentum"
    PlotTitle = "Change in Orbital Angular Momentum with Gravity"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInOrbitalEnergy"
    PlotTitle = "Change in Orbital Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInRotationalAngularMomentum"
    PlotTitle = "Change In Rotational Angular Momentum with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInRotationalEnergy"
    PlotTitle = "Change In Rotational Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)


    plt.figure()
    plt.clf()
    plt.plot(orbAngMom1_N[:,0]*1e-9, (orbAngMom1_N[:,1] - orbAngMom1_N[0,1])/orbAngMom1_N[0,1], orbAngMom1_N[:,0]*1e-9, (orbAngMom1_N[:,2] - orbAngMom1_N[0,2])/orbAngMom1_N[0,2], orbAngMom1_N[:,0]*1e-9, (orbAngMom1_N[:,3] - orbAngMom1_N[0,3])/orbAngMom1_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInOrbitalAngularMomentum1"
    PlotTitle = "Change in Orbital Angular Momentum with Gravity"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy1[:,0]*1e-9, (orbEnergy1[:,1] - orbEnergy1[0,1])/orbEnergy1[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInOrbitalEnergy1"
    PlotTitle = "Change in Orbital Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom1_N[:,0]*1e-9, (rotAngMom1_N[:,1] - rotAngMom1_N[0,1])/rotAngMom1_N[0,1], rotAngMom1_N[:,0]*1e-9, (rotAngMom1_N[:,2] - rotAngMom1_N[0,2])/rotAngMom1_N[0,2], rotAngMom1_N[:,0]*1e-9, (rotAngMom1_N[:,3] - rotAngMom1_N[0,3])/rotAngMom1_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInRotationalAngularMomentum1"
    PlotTitle = "Change In Rotational Angular Momentum with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy1[:,0]*1e-9, (rotEnergy1[:,1] - rotEnergy1[0,1])/rotEnergy1[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInRotationalEnergy1"
    PlotTitle = "Change In Rotational Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)


    plt.figure()
    plt.clf()
    plt.plot(orbAngMom2_N[:,0]*1e-9, (orbAngMom2_N[:,1] - orbAngMom2_N[0,1])/orbAngMom2_N[0,1], orbAngMom2_N[:,0]*1e-9, (orbAngMom2_N[:,2] - orbAngMom2_N[0,2])/orbAngMom2_N[0,2], orbAngMom2_N[:,0]*1e-9, (orbAngMom2_N[:,3] - orbAngMom2_N[0,3])/orbAngMom2_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInOrbitalAngularMomentum2"
    PlotTitle = "Change in Orbital Angular Momentum with Gravity"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy2[:,0]*1e-9, (orbEnergy2[:,1] - orbEnergy2[0,1])/orbEnergy2[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInOrbitalEnergy2"
    PlotTitle = "Change in Orbital Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom2_N[:,0]*1e-9, (rotAngMom2_N[:,1] - rotAngMom2_N[0,1])/rotAngMom2_N[0,1], rotAngMom2_N[:,0]*1e-9, (rotAngMom2_N[:,2] - rotAngMom2_N[0,2])/rotAngMom2_N[0,2], rotAngMom2_N[:,0]*1e-9, (rotAngMom2_N[:,3] - rotAngMom2_N[0,3])/rotAngMom2_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInRotationalAngularMomentum2"
    PlotTitle = "Change In Rotational Angular Momentum with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy2[:,0]*1e-9, (rotEnergy2[:,1] - rotEnergy2[0,1])/rotEnergy2[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    PlotName = "ChangeInRotationalEnergy2"
    PlotTitle = "Change In Rotational Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
    if show_plots:
        plt.show()
        plt.close('all')

    accuracy = 1e-8
    # for i in range(0,len(truePos)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(r_BN_NOutput[-1,:],truePos[i],3,accuracy):
    #         testFailCount += 1
    #         testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed pos unit test")
    #
    # for i in range(0,len(trueSigma)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(sigma_BNOutput[-1,:],trueSigma[i],3,accuracy):
    #         testFailCount += 1
    #         testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed attitude unit test")

    accuracy = 1e-10
    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed orbital angular momentum unit test")

    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational angular momentum unit test")

    for i in range(0,len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational energy unit test")

    for i in range(0,len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed orbital energy unit test")

    if testFailCount == 0:
        print("PASSED: " + " Spacecraft Translation and Rotation Integrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    # SCConnected(True)
    SCConnectedAndUnconnected(True)