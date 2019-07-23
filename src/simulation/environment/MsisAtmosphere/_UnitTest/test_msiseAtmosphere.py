''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Test the validity of a simple exponential atmosphere model.
# Author:   Andrew Harris
# Creation Date:  Jan 18, 2017
#

import os, inspect
import numpy as np
import pytest

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import msisAtmosphere

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import simIncludeGravBody


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitType", ["LPO", "LTO"])

# provide a unique test method name, starting with test_
def test_scenarioMsisAtmosphereOrbit(show_plots, orbitType):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    earthCase = "Earth"
    showVal = False

    [testResults, testMessage] = run(showVal, orbitType, earthCase)

    assert testResults < 1, testMessage

def run(show_plots, orbitCase, planetCase):
    '''Call this routine directly to run the script.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #
    #  From here on there scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    newAtmo = msisAtmosphere.MsisAtmosphere()
    atmoTaskName = "atmosphere"
    newAtmo.ModelTag = "MsisAtmo"
    newAtmo.setEpoch(1)

    dynProcess.addTask(scSim.CreateNewTask(atmoTaskName, simulationTimeStep))
    scSim.AddModelToTask(atmoTaskName, newAtmo)

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsgName)

    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #   setup orbit and simulation time
    oe = orbitalMotion.ClassicElements()
    if planetCase == "Earth":
        r_eq = planet.radEquator
        if orbitCase == "LPO":
            orbAltMin = 100.0*1000.0
            orbAltMax = orbAltMin
        elif orbitCase == "LTO":
            orbAltMin = 100.*1000.0
            orbAltMax = 100.0 * 1000.0
    else:
        return 1, "Test failed- did not initialize planets."

    rMin = r_eq + orbAltMin
    rMax = r_eq + orbAltMax
    oe.a = (rMin+rMax)/2.0
    oe.e = 1.0 - rMin/oe.a
    oe.i = 0.0*macros.D2R

    oe.Omega = 0.0*macros.D2R
    oe.omega = 0.0*macros.D2R
    oe.f     = 0.0*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
                                                # with circular or equatorial orbit, some angles are
                                                # arbitrary

    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n

    newAtmo.planetRadius = r_eq

    simulationTime = macros.sec2nano(0.001*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    sw_msg_names = [
        "ap_24_0", "ap_3_0", "ap_3_-3","ap_3_-6","ap_3_-9",
        "ap_3_-12","ap_3_-15","ap_3_-18","ap_3_-21","ap_3_-24",
        "ap_3_-27", "ap_3_-30","ap_3_-33","ap_3_-36","ap_3_-39",
        "ap_3_-42", "ap_3_-45", "ap_3_-48","ap_3_-51","ap_3_-54",
        "ap_3_-57","f107_1944_0","f107_24_-24"
    ]


    for swName in sw_msg_names:
        msgName = swName
        msgData = msisAtmosphere.SwDataSimMsg()
        msgData.dataValue=0.
        unitTestSupport.setMessage(scSim.TotalSim, simProcessName, msgName, msgData)

    numDataPoints = 2
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(newAtmo.envOutMsgNames[-1], samplingTime)

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_BN_N',range(3))
    densData = scSim.pullMessageLogData(newAtmo.envOutMsgNames[-1]+'.neutralDensity')
    tempData = scSim.pullMessageLogData(newAtmo.envOutMsgNames[-1]+'.localTemp')
    #relPosData = scSim.GetLogVariableData('MsisAtmo.relativePos')
    np.set_printoptions(precision=16)

    #   Compare to expected values

    refAtmoData = np.loadtxt('test_outputs.txt')

    accuracy = 1e-8

    unitTestSupport.writeTeXSnippet("unitTestToleranceValue", str(accuracy), path)
    #for ind in range(0, posData.shape[0]-1):
    #   Test atmospheric density calculation...
    if not unitTestSupport.isDoubleEqualRelative(densData[0,1], refAtmoData[5], accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED:  NRLMSISE-00 failed density unit test at t=" + str(densData[0, 0] * macros.NANO2SEC) + "sec with a value difference of "+str(densData[0,1]-refAtmoData[5]))

    if not unitTestSupport.isDoubleEqualRelative(tempData[0,1], refAtmoData[-1], accuracy):
        testFailCount += 1
        testMessages.append(
        "FAILED:  NRLMSISE-00 failed temperature unit test at t=" + str(densData[0, 0] * macros.NANO2SEC) + "sec with a value difference of "+str(tempData[0,1]-refAtmoData[-1]))


    snippentName = "unitTestPassFail"
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print "PASSED: " + newAtmo.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print "Failed: " + newAtmo.ModelTag
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    run(True, "LPO", "Earth")