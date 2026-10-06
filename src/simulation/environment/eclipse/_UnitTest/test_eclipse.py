
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

#
# Eclipse Condition Unit Test
#
# Purpose:  Test the proper function of the Eclipse environment module.
#           This is done by comparing computed expected illumination factors in
#           particular eclipse conditions to what is simulated
# Author:   Patrick Kenneally
# Creation Date:  May. 31, 2017
#

import os
import numpy as np

import pytest
import datetime
from Basilisk import __path__
from Basilisk.simulation import eclipse
from Basilisk.simulation import spacecraft
from Basilisk.simulation import planetEphemeris
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging
from Basilisk.architecture.messaging import EclipseMsgPayload

bskPath = __path__[0]

path = os.path.dirname(os.path.abspath(__file__))

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
@pytest.mark.parametrize("eclipseCondition, planet", [
("partial", "earth"), ("full", "earth"), ("none", "earth"), ("annular", "earth"),
("partial", "mars"), ("full", "mars"), ("none", "mars"), ("annular", "mars")])
def test_unitEclipse(show_plots, eclipseCondition, planet):
    """
**Test Description and Success Criteria**

The unit test validates the internal aspects of the Basilisk eclipse module by comparing simulated output with \
expected output. It validates the computation of a illumination factor for total eclipse, partial eclipse,annular eclipse, \
and no eclipse scenarios. The test is designed to analyze one type at a time for both Earth and Mars and is then \
repeated for all three.

Earth is set as the zero base for all eclipse types to test it as the occulting body. For full, partial, and \
no eclipse cases, orbital elements describing the spacecraft states are then converted to Cartesian vectors. \
These orbital elements vary for each eclipse type since the Sun and planet states are fixed. The conversion is \
made using the orbitalMotion elem2rv function, where the inputs are six orbital elements (a, e, i, Omega, omega, f) \
and the outputs are Cartesian position and velocity vectors. For the annular eclipse case, the conversion is \
avoided and a Cartesian position vector is initially provided instead. The vectors are then passed into \
spacecraft and, subsequently, the eclipse module through the Basilisk messaging system.

Testing the no eclipse case with Mars as the occulting body is the same as the Earth no eclipse test, except \
Mars is set as the zero base. The Mars full, partial, and annular eclipse cases, however, are like the Earth \
annular case where Cartesian vectors are, instead, the initial inputs. Since the test is performed as a single \
step process, the velocity is not necessarily needed as an input, so only a position vector is provided \
for these cases.

The illumination factor obtained through the module is compared to the expected result, which is either trivial or \
calculated, depending on the eclipse type. Full eclipse and no eclipse illumination factors are compared without the \
need for computation, since they are just 0.0 and 1.0, respectively. The partial and annular eclipse illumination \
factors, however, vary between 0.0 and 1.0, based on the cone dimensions, and are calculated using \
MATLAB and Spice data.


**Test Parameters:**

- ``eclipseCondition``: [string]
  defines if the eclipse is partial, full, none or annular
- ``planet``: [string]
  defines which planet to use.  Options include "earth" and "mars"


**Description of Variables Being Tested**

In each test scenario the illumination eclipse variable

    ``illuminationFactor``

is pulled from the log data and compared to expected truth values.

    """
    [testResults, testMessage] = unitEclipse(show_plots, eclipseCondition, planet)
    assert testResults < 1, testMessage


def test_unitEclipseCustom(show_plots):
    """
**Test Description and Success Criteria**

The unit test validates the internal aspects of the Basilisk eclipse module by comparing simulated output with \
expected output. It validates the computation of a illumination factor for total eclipse using a custom gravity body.

This unit test sets up a custom gravity body, the asteroid Bennu, using the planetEphemeris module (i.e. Spice \
is not used for this test.) An empty spice planet message is created for the sun. The spacecraft is set 500 m \
on the side of the asteroid opposite of the sun.

The illumination factor obtained through the module is compared to the expected result, which is trivial to compute.

**Description of Variables Being Tested**

In this test scenario the illumination eclipse variable

    ``illuminationFactor``

is pulled from the log data and compared to the expected truth value.

    """
    [testResults, testMessage] = unitEclipseCustom(show_plots)
    assert testResults < 1, testMessage


def unitEclipse(show_plots, eclipseCondition, planet):
    __tracebackhide__ = True

    testFailCount = 0
    testMessages = []
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(1)

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    # Set up first spacecraft
    scObject_0 = spacecraft.Spacecraft()
    scObject_0.ModelTag = "spacecraft"
    unitTestSim.AddModelToTask(testTaskName, scObject_0)

    # setup Gravity Bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    mars = gravFactory.createMarsBarycenter()
    if planet == "earth":
        earth.isCentralBody = True
    elif planet == "mars":
        mars.isCentralBody = True
    scObject_0.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # setup Spice interface for some solar system bodies
    timeInitString = "2021 MAY 04 07:47:48.965 (UTC)"
    # earth and mars must come first as with gravBodies
    gravFactory.createSpiceInterface(time=timeInitString,
                                     spicePlanetNames=["earth", "mars barycenter", "sun", "venus"])

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

    unitTestSim.AddModelToTask(testTaskName, gravFactory.spiceObject, -1)

    eclipseObject = eclipse.Eclipse()
    eclipseObject.addSpacecraftToModel(scObject_0.scStateOutMsg)
    eclipseObject.addPlanetToModel(gravFactory.spiceObject.planetStateOutMsgs[3])   # venus
    eclipseObject.addPlanetToModel(gravFactory.spiceObject.planetStateOutMsgs[1])   # mars
    eclipseObject.addPlanetToModel(gravFactory.spiceObject.planetStateOutMsgs[0])   # earth
    eclipseObject.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[2])   # sun

    unitTestSim.AddModelToTask(testTaskName, eclipseObject)

    dataLog = eclipseObject.eclipseOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(testTaskName, dataLog)

    unitTestSim.InitializeSimulation()

    # Execute the simulation for one time step
    unitTestSim.TotalSim.SingleStepProcesses()

    eclipseData_0 = dataLog.illuminationFactor
    # Obtain body position vectors to check with MATLAB

    errTol = 1E-12
    if planet == "earth":
        if eclipseCondition == "partial":
            truthilluminationFactor = 0.62310760206735027
            if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
                testFailCount += 1
                testMessages.append("Illumination Factor failed for Earth partial eclipse condition")

        elif eclipseCondition == "full":
            truthilluminationFactor = 0.0
            if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
                testFailCount += 1
                testMessages.append("Illumination Factor failed for Earth full eclipse condition")

        elif eclipseCondition == "none":
            truthilluminationFactor = 1.0
            if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
                testFailCount += 1
                testMessages.append("Illumination Factor failed for Earth none eclipse condition")
        elif eclipseCondition == "annular":
            truthilluminationFactor = 1.497253388113018e-04
            if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
                testFailCount += 1
                testMessages.append("Illumination Factor failed for Earth annular eclipse condition")

    elif planet == "mars":
        if eclipseCondition == "partial":
            truthilluminationFactor = 0.18745025055615416
            if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
                testFailCount += 1
                testMessages.append("Illumination Factor failed for Mars partial eclipse condition")
        elif eclipseCondition == "full":
            truthilluminationFactor = 0.0
            if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
                testFailCount += 1
                testMessages.append("Illumination Factor failed for Mars full eclipse condition")
        elif eclipseCondition == "none":
            truthilluminationFactor = 1.0
            if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
                testFailCount += 1
                testMessages.append("Illumination Factor failed for Mars none eclipse condition")
        elif eclipseCondition == "annular":
            truthilluminationFactor = 4.245137380531894e-05
            if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
                testFailCount += 1
                testMessages.append("Illumination Factor failed for Mars annular eclipse condition")

    if testFailCount == 0:
        print("PASSED: " + planet + "-" + eclipseCondition)
        # return fail count and join into a single string all messages in the list
        # testMessage
    else:
        print(testMessages)

    print('The error tolerance for all tests is ' + str(errTol))

    #
    #  unload the SPICE libraries that were loaded by the spiceObject earlier
    #
    gravFactory.unloadSpiceKernels()

    return [testFailCount, ''.join(testMessages)]

def unitEclipseCustom(show_plots):
    __tracebackhide__ = True

    testFailCount = 0
    testMessages = []
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(1)

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))

    # Set up first spacecraft
    scObject_0 = spacecraft.Spacecraft()
    scObject_0.ModelTag = "spacecraft"
    unitTestSim.AddModelToTask(testTaskName, scObject_0)

    # setup Gravity Bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    mu_bennu = 4.892
    custom = gravFactory.createCustomGravObject("custom", mu_bennu) # creates a custom grav object (bennu)
    scObject_0.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Create the ephemeris data for the bodies
    # setup celestial object ephemeris module
    gravBodyEphem = planetEphemeris.PlanetEphemeris()
    gravBodyEphem.ModelTag = 'planetEphemeris'
    gravBodyEphem.setPlanetNames(planetEphemeris.StringVector(["custom"]))

    # Specify bennu orbit
    oeAsteroid = planetEphemeris.ClassicElements()
    oeAsteroid.a = 1.1259 * orbitalMotion.AU * 1000. # m
    oeAsteroid.e = 0.20373
    oeAsteroid.i = 6.0343 * macros.D2R
    oeAsteroid.Omega = 2.01820 * macros.D2R
    oeAsteroid.omega = 66.304 * macros.D2R
    oeAsteroid.f = 120.0 * macros.D2R

    gravBodyEphem.planetElements = planetEphemeris.classicElementVector([oeAsteroid])
    custom.planetBodyInMsg.subscribeTo(gravBodyEphem.planetOutMsgs[0])

    # Create an empty sun spice object
    sunPlanetStateMsgData = messaging.SpicePlanetStateMsgPayload()
    sunPlanetStateMsg = messaging.SpicePlanetStateMsg()
    sunPlanetStateMsg.write(sunPlanetStateMsgData)

    r_ast_N = np.array([-177862743954.6422, -25907896415.157013, -2074871174.236055])
    r_sc_N = r_ast_N + 500 * r_ast_N / np.linalg.norm(r_ast_N)
    scObject_0.hub.r_CN_NInit = r_sc_N

    unitTestSim.AddModelToTask(testTaskName, gravBodyEphem, -1)

    eclipseObject = eclipse.Eclipse()
    eclipseObject.addSpacecraftToModel(scObject_0.scStateOutMsg)
    eclipseObject.addPlanetToModel(gravBodyEphem.planetOutMsgs[0])  # custom
    eclipseObject.sunInMsg.subscribeTo(sunPlanetStateMsg)   # sun
    eclipseObject.rEqCustom = 282. # m

    unitTestSim.AddModelToTask(testTaskName, eclipseObject)

    dataLog = eclipseObject.eclipseOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(testTaskName, dataLog)

    unitTestSim.InitializeSimulation()

    # Execute the simulation for one time step
    unitTestSim.TotalSim.SingleStepProcesses()

    eclipseData_0 = dataLog.illuminationFactor
    # Obtain body position vectors to check with MATLAB

    errTol = 1E-12
    truthilluminationFactor = 0.0
    if not unitTestSupport.isDoubleEqual(eclipseData_0[-1], truthilluminationFactor, errTol):
        testFailCount += 1
        testMessages.append("Illumination Factor failed for custom full eclipse condition")

    if testFailCount == 0:
        print("PASSED: custom-full")
        # return fail count and join into a single string all messages in the list
        # testMessage
    else:
        print(testMessages)

    print('The error tolerance for all tests is ' + str(errTol))

    return [testFailCount, ''.join(testMessages)]

def test_eclipse_uses_spacecraft_state_extrapolated_to_step_midpoint():
    """
**Test Description and Success Criteria**

The spacecraft state message read by the module was written at the end of the previous step, so the
illumination factor lags the interval it is applied to. The module therefore advances the spacecraft position
with its velocity by half of the message age.

A spacecraft is placed outside the Earth shadow, moving towards it fast enough that the position advanced
by half a step is inside the shadow. The state is given to one module with its velocity, written at the start
of the simulation and rewritten one step later, and to a second module as the already advanced position written at the current time. The two
illumination factors must match, and both must differ from a static spacecraft that does not move.

    """
    stepSeconds = 10.0  # [s]
    halfStepShift = np.array([0.0, -4.0e4, 0.0]) * stepSeconds / 2.0  # [m] v * dt / 2
    rStart = np.array([-7.0e6, 6.50e6, 0.0])  # [m] outside the penumbra, which extends ~35 km beyond the Earth radius

    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", macros.sec2nano(stepSeconds)))

    sunPayload = messaging.SpicePlanetStateMsgPayload()
    sunPayload.PositionVector = [orbitalMotion.AU * 1000.0, 0.0, 0.0]  # [m]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunPayload)
    planetPayload = messaging.SpicePlanetStateMsgPayload()
    planetPayload.PlanetName = "earth"
    planetMsg = messaging.SpicePlanetStateMsg().write(planetPayload)

    rewrites = []  # (message, payload, [ns] time of the rewrite one step later)

    def stateMsg(r, v, timeNanos):
        payload = messaging.SCStatesMsgPayload()
        payload.r_BN_N = r.tolist()
        payload.v_BN_N = v.tolist()
        msg = messaging.SCStatesMsg().write(payload, timeNanos)
        rewrites.append((msg, payload, timeNanos + macros.sec2nano(stepSeconds)))
        return msg

    withVelocity = stateMsg(rStart, np.array([0.0, -4.0e4, 0.0]), 0)
    advanced = stateMsg(rStart + halfStepShift, np.zeros(3), macros.sec2nano(stepSeconds))
    static = stateMsg(rStart, np.zeros(3), 0)

    recorders = []
    for scMsg in (withVelocity, advanced, static):
        module = eclipse.Eclipse()
        module.setExtrapolateScStateToStepMidpoint(True)
        module.addSpacecraftToModel(scMsg)
        module.addPlanetToModel(planetMsg)
        module.sunInMsg.subscribeTo(sunMsg)
        scSim.AddModelToTask("t", module)
        recorders.append(module.eclipseOutMsgs[0].recorder())
        scSim.AddModelToTask("t", recorders[-1])

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(stepSeconds))
    scSim.ExecuteSimulation()
    # the spacecraft rewrites its state one step later, so that the module observes the spacecraft task period
    for msg, payload, timeNanos in rewrites:
        msg.write(payload, timeNanos)
    scSim.ConfigureStopTime(macros.sec2nano(2 * stepSeconds))
    scSim.ExecuteSimulation()

    factorVelocity, factorAdvanced, factorStatic = (r.illuminationFactor[-1] for r in recorders)
    assert factorStatic == pytest.approx(1.0)
    assert factorAdvanced < 1.0
    assert factorVelocity == pytest.approx(factorAdvanced, abs=1e-12)


def test_shadow_vs_illumination_alias_and_deprecation_behavior():
    """
    Check aliasing and deprecation behavior for EclipseMsgPayload.

    If this test starts failing with a BSKUrgentDeprecationWarning (or the
    deprecation utility starts treating ``shadowFactor`` as an error), that is
    a signal that we have passed the deprecation deadline and should:

    * remove support for ``EclipseMsgPayload.shadowFactor``, and
    * remove this test (or update it to only check ``illuminationFactor``).
    """
    p = EclipseMsgPayload()

    # 1. Test Forward Direction (New -> Old)
    # Setting the new variable should update the old one silently (or logically).
    p.illuminationFactor = 0.8
    assert p.illuminationFactor == pytest.approx(0.8)

    # Reading the OLD variable (shadowFactor) triggers the warning.
    # We must catch it so it doesn't fail the "no warnings allowed" check.
    with pytest.warns(Warning, match=".*illuminationFactor.*"):
        val = p.shadowFactor
    assert val == pytest.approx(0.8)

    cutoff = datetime.date(2026, 12, 31)
    today = datetime.date.today()

    if today <= cutoff:
        # 2. Test Backward Direction (Old -> New)
        # Setting the deprecated name should emit a warning.
        with pytest.warns(Warning, match=r".*illuminationFactor.*"):
            p.shadowFactor = 0.5

        # Verify it updated the new variable
        assert p.illuminationFactor == pytest.approx(0.5)


if __name__ == "__main__":
    unitEclipse(False, "annular", "mars")
    unitEclipseCustom(False)
    test_shadow_vs_illumination_alias_and_deprecation_behavior()
