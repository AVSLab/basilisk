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
# Eclipse Condition Unit Test
#
# Purpose:  Test the proper function of the Eclipse environment module.
#           This is done by comparing computed expected shadow factors in
#           particular eclipse conditions to what is simulated
# Author:   Patrick Kenneally
# Creation Date:  May. 31, 2017
#

import pytest
import os
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import macros
from Basilisk.simulation import eclipse
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk import __path__
bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
@pytest.mark.parametrize("eclipseCondition, planet", [
("partial", "earth"), ("full", "earth"), ("none", "earth"), ("annular", "earth"),
("partial", "mars"), ("full", "mars"), ("none", "mars"), ("annular", "mars")])

def test_unitEclipse(show_plots, eclipseCondition, planet):
    [testResults, testMessage] = unitEclipse(show_plots, eclipseCondition, planet)
    assert testResults < 1, testMessage


def unitEclipse(show_plots, eclipseCondition, planet):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0
    testMessages = []
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(1)

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # Ensure simulation is empty
    unitTestSim.TotalSim.terminateSimulation()
    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    # Set up first spacecraft
    scObject_0 = spacecraftPlus.SpacecraftPlus()
    scObject_0.scStateOutMsgName = "inertial_state_output"
    unitTestSim.AddModelToTask(testTaskName, scObject_0)

    # setup Gravity Bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    if planet == "earth":
        earth = gravFactory.createEarth()
        earth.isCentralBody = True
        earth.useSphericalHarmParams = False
    elif planet == "mars":
        mars = gravFactory.createMars()
        mars.isCentralBody = True
        mars.useSphericalHarmParams = False
    scObject_0.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    # setup Spice interface for some solar system bodies
    timeInitString = '2021 MAY 04 07:47:48.965 (UTC)'
    gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/'
                                     , timeInitString
                                     , spicePlanetNames = ["sun", "venus", "earth", "mars barycenter"]
                                     )

    if planet == "earth":
        if eclipseCondition == "full":
            gravFactory.spiceObject.zeroBase = "earth"
            # set up spacecraft 0 position and velocity for full eclipse
            oe = orbitalMotion.ClassicElements()
            r_0 = (500 + orbitalMotion.REQ_EARTH)  # km
            oe.a = r_0
            oe.e = 0.00001
            oe.i = 5.0 * macros.D2R
            oe.Omega = 48.2 * macros.D2R
            oe.omega = 0 * macros.D2R
            oe.f = 173 * macros.D2R
            r_N_0, v_N_0 = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, oe)
            scObject_0.hub.r_CN_NInit = r_N_0 * 1000  # convert to meters
            scObject_0.hub.v_CN_NInit = v_N_0 * 1000  # convert to meters
        elif eclipseCondition == "partial":
            gravFactory.spiceObject.zeroBase = "earth"
            # set up spacecraft 0 position and velocity for full eclipse
            oe = orbitalMotion.ClassicElements()
            r_0 = (500 + orbitalMotion.REQ_EARTH)  # km
            oe.a = r_0
            oe.e = 0.00001
            oe.i = 5.0 * macros.D2R
            oe.Omega = 48.2 * macros.D2R
            oe.omega = 0 * macros.D2R
            oe.f = 107.5 * macros.D2R
            r_N_0, v_N_0 = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, oe)
            scObject_0.hub.r_CN_NInit = r_N_0 * 1000  # convert to meters
            scObject_0.hub.v_CN_NInit = v_N_0 * 1000  # convert to meters
        elif eclipseCondition == "none":
            oe = orbitalMotion.ClassicElements()
            r_0 = 9959991.68982  # km
            oe.a = r_0
            oe.e = 0.00001
            oe.i = 5.0 * macros.D2R
            oe.Omega = 48.2 * macros.D2R
            oe.omega = 0 * macros.D2R
            oe.f = 107.5 * macros.D2R
            r_N_0, v_N_0 = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, oe)
            scObject_0.hub.r_CN_NInit = r_N_0 * 1000  # convert to meters
            scObject_0.hub.v_CN_NInit = v_N_0 * 1000  # convert to meters
        elif eclipseCondition == "annular":
            gravFactory.spiceObject.zeroBase = "earth"
            scObject_0.hub.r_CN_NInit = [-326716535628.942, -287302983139.247, -124542549301.050]

    elif planet == "mars":
        if eclipseCondition == "full":
            gravFactory.spiceObject.zeroBase = "mars barycenter"
            scObject_0.hub.r_CN_NInit = [-2930233.55919119, 2567609.100747609, 41384.23366372246] # meters
        elif eclipseCondition == "partial":
            print "partial mars"
            gravFactory.spiceObject.zeroBase = "mars barycenter"
            scObject_0.hub.r_CN_NInit = [-6050166.454829555, 2813822.447404055, 571725.5651779658] # meters
        elif eclipseCondition == "none":
            oe = orbitalMotion.ClassicElements()
            r_0 = 9959991.68982  # km
            oe.a = r_0
            oe.e = 0.00001
            oe.i = 5.0 * macros.D2R
            oe.Omega = 48.2 * macros.D2R
            oe.omega = 0 * macros.D2R
            oe.f = 107.5 * macros.D2R
            r_N_0, v_N_0 = orbitalMotion.elem2rv(orbitalMotion.MU_MARS, oe)
            scObject_0.hub.r_CN_NInit = r_N_0 * 1000  # convert to meters
            scObject_0.hub.v_CN_NInit = v_N_0 * 1000  # convert to meters
        elif eclipseCondition == "annular":
            gravFactory.spiceObject.zeroBase = "mars barycenter"
            scObject_0.hub.r_CN_NInit = [-427424601171.464, 541312532797.400, 259820030623.064]  # meters

    unitTestSim.AddModelToTask(testTaskName, gravFactory.spiceObject, None, -1)

    eclipseObject = eclipse.Eclipse()
    eclipseObject.addPositionMsgName(scObject_0.scStateOutMsgName)
    eclipseObject.addPlanetName('earth')
    eclipseObject.addPlanetName('mars barycenter')
    eclipseObject.addPlanetName('venus')
    unitTestSim.AddModelToTask(testTaskName, eclipseObject)
    unitTestSim.TotalSim.logThisMessage("eclipse_data_0")
    unitTestSim.TotalSim.logThisMessage("mars barycenter_planet_data")
    unitTestSim.TotalSim.logThisMessage("sun_planet_data")
    unitTestSim.TotalSim.logThisMessage("earth_planet_data")

    unitTestSim.InitializeSimulation()

    # Execute the simulation for one time step
    unitTestSim.TotalSim.SingleStepProcesses()

    eclipseData_0 = unitTestSim.pullMessageLogData("eclipse_data_0.shadowFactor")
    # Obtain body position vectors to check with MATLAB

    errTol = 1E-12
    if planet == "earth":
        if eclipseCondition == "partial":
            truthShadowFactor = 0.62310760206735027
            if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
                testFailCount += 1
                testMessages.append("Shadow Factor failed for Earth partial eclipse condition")

        elif eclipseCondition == "full":
            truthShadowFactor = 0.0
            if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
                testFailCount += 1
                testMessages.append("Shadow Factor failed for Earth full eclipse condition")

        elif eclipseCondition == "none":
            truthShadowFactor = 1.0
            if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
                testFailCount += 1
                testMessages.append("Shadow Factor failed for Earth none eclipse condition")
        elif eclipseCondition == "annular":
            truthShadowFactor = 1.497253388113018e-04
            if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
                testFailCount += 1
                testMessages.append("Shadow Factor failed for Earth annular eclipse condition")

    elif planet == "mars":
        if eclipseCondition == "partial":
            truthShadowFactor = 0.18745025055615416
            if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
                testFailCount += 1
                testMessages.append("Shadow Factor failed for Mars partial eclipse condition")
        elif eclipseCondition == "full":
            truthShadowFactor = 0.0
            if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
                testFailCount += 1
                testMessages.append("Shadow Factor failed for Mars full eclipse condition")
        elif eclipseCondition == "none":
            truthShadowFactor = 1.0
            if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
                testFailCount += 1
                testMessages.append("Shadow Factor failed for Mars none eclipse condition")
        elif eclipseCondition == "annular":
            truthShadowFactor = 4.245137380531894e-05
            if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
                testFailCount += 1
                testMessages.append("Shadow Factor failed for Mars annular eclipse condition")


    if testFailCount == 0:
        print "PASSED: " + planet + "-" + eclipseCondition
        # return fail count and join into a single string all messages in the list
        # testMessage
    if testFailCount == 0:
        colorText = 'ForestGreen'
        passFailMsg = ""  # "Passed: " + name + "."
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        passFailMsg = "Failed: " + eclipseCondition + "."
        testMessages.append(passFailMsg)
        testMessages.append(" | ")
        passedText = '\\textcolor{' + colorText + '}{' + "FAILED" + '}'

    # Write some snippets for AutoTex
    snippetName = planet+eclipseCondition + "PassedText"
    snippetContent = passedText
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

    snippetName = planet+eclipseCondition + "PassFailMsg"
    snippetContent = passFailMsg
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

    snippetName = eclipseCondition + "error"
    snippetContent = str(errTol)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

    #
    #  unload the SPICE libraries that were loaded by the spiceObject earlier
    #
    gravFactory.unloadSpiceKernels()

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    unitEclipse(False, "annular", "earth")