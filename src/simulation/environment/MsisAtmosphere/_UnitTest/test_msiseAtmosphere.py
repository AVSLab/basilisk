
# ISC License
#
# Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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



#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Test the validity of a simple exponential atmosphere model.
# Author:   Andrew Harris
# Creation Date:  Jan 18, 2017
#

import inspect
import os

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import msisAtmosphere
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions

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
@pytest.mark.parametrize("setEpoch", ["Default", "Direct", "Msg"])


# provide a unique test method name, starting with test_
def test_scenarioMsisAtmosphereOrbit(show_plots, orbitType, setEpoch):
    """This function is called by the py.test environment."""
    # each test method requires a single assert method to be called
    showVal = False

    [testResults, testMessage] = run(showVal, orbitType, setEpoch)

    assert testResults < 1, testMessage


def run(show_plots, orbitCase, setEpoch):
    """Call this routine directly to run the script."""
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

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #   Initialize new atmosphere and drag model, add them to task
    newAtmo = msisAtmosphere.MsisAtmosphere()
    atmoTaskName = "atmosphere"
    newAtmo.ModelTag = "MsisAtmo"

    if setEpoch == "Msg":
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2019 Jan 01 00:00:00.00 (UTC)')
        newAtmo.epochInMsg.subscribeTo(epochMsg)

        # setting epoch day of year info deliberately to a false value.  The epoch msg info should be used
        newAtmo.epochDoy = 10

    elif setEpoch == "Direct":
        newAtmo.epochDoy = 1  # setting epoch day of year info directly

    dynProcess.addTask(scSim.CreateNewTask(atmoTaskName, simulationTimeStep))
    scSim.AddModelToTask(atmoTaskName, newAtmo)

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()
    newAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    planet = gravFactory.createEarth()
    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #   setup orbit and simulation time
    oe = orbitalMotion.ClassicElements()
    r_eq = planet.radEquator
    if orbitCase == "LPO":
        orbAltMin = 100.0*1000.0
        orbAltMax = orbAltMin
    elif orbitCase == "LTO":
        orbAltMin = 100.*1000.0
        orbAltMax = 100.0 * 1000.0

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

    simulationTime = macros.sec2nano(0.002*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    sw_msg_names = [
        "ap_24_0", "ap_3_0", "ap_3_-3", "ap_3_-6", "ap_3_-9",
        "ap_3_-12", "ap_3_-15", "ap_3_-18", "ap_3_-21", "ap_3_-24",
        "ap_3_-27", "ap_3_-30", "ap_3_-33", "ap_3_-36", "ap_3_-39",
        "ap_3_-42", "ap_3_-45", "ap_3_-48", "ap_3_-51", "ap_3_-54",
        "ap_3_-57", "f107_1944_0", "f107_24_-24"
    ]

    swMsgList = []
    for c in range(len(sw_msg_names)):
        swMsgData = messaging.SwDataMsgPayload()
        swMsgData.dataValue = 0
        swMsgList.append(messaging.SwDataMsg().write(swMsgData))
        newAtmo.swDataInMsgs[c].subscribeTo(swMsgList[-1])

    dataLog = scObject.scStateOutMsg.recorder()
    denLog = newAtmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, denLog)

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N
    densData = denLog.neutralDensity
    tempData = denLog.localTemp

    np.set_printoptions(precision=16)

    #   Compare to expected values

    refAtmoData = np.loadtxt(path + '/truthOutputs.txt')

    accuracy = 1e-8

    unitTestSupport.writeTeXSnippet("unitTestToleranceValue", str(accuracy), path)

    #   Test atmospheric density calculation; note that refAtmoData is in g/cm^3,
    #   and must be adjusted by a factor of 1e-3 to match kg/m^3
    print(densData[-1])
    print(refAtmoData[5]*1000)
    if np.testing.assert_allclose(densData[-1], refAtmoData[5]*1000., atol=accuracy):
        testFailCount += 1
        testMessages.append("FAILED:  NRLMSISE-00 failed density unit test with a value difference of "+str(densData[0]-refAtmoData[5]*1000))

    print(tempData[-1])
    print(refAtmoData[-1])
    if np.testing.assert_allclose(tempData[-1], refAtmoData[-1], atol=accuracy):
        testFailCount += 1
        testMessages.append(
        "FAILED:  NRLMSISE-00 failed temperature unit test with a value difference of "+str(tempData[-1]-refAtmoData[-1]))

    snippentName = "unitTestPassFail" + str(orbitCase) + str(setEpoch)
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + newAtmo.ModelTag)
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + newAtmo.ModelTag)
        passedText = '\\textcolor{' + colorText + '}{' + "Failed" + '}'
        print(testMessages)
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    return [testFailCount, ''.join(testMessages)]

if __name__ == '__main__':
    run(True,
        "LPO",          # orbitCase
        "Msg")          # setEpoch
