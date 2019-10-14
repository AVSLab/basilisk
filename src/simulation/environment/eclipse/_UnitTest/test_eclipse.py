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
    """
Test Description and Success Criteria
-------------------------------------
The unit test validates the internal aspects of the Basilisk eclipse module by comparing simulated output with \
expected output. It validates the computation of a shadow factor for total eclipse, partial eclipse,annular eclipse, \
and no eclipse scenarios. The test is designed to analyze one type at a time for both Earth and Mars and is then \
repeated for all three.

Sun States
This test begins by specifying a UTC time through the use of a Spice object. This, along with the implementation of \
Spice kernels, allows the states of the sun and any desired planets to be determined. One time is provided, fixing the \
sun and planet states so the spacecraft states may be varied. After specifying these, the spacecraft states are set \
and the three eclipse types are considered.

Spacecraft States
Earth is set as the zero base for all eclipse types to test it as the occulting body. For full, partial, and \
no eclipse cases, orbital elements describing the spacecraft states are then converted to Cartesian vectors. \
These orbital elements vary for each eclipse type since the Sun and planet states are fixed. The conversion is \
made using the orbitalMotion elem2rv function, where the inputs are six orbital elements (a, e, i, Omega, omega, f) \
and the outputs are Cartesian position and velocity vectors. For the annular eclipse case, the conversion is \
avoided and a Cartesian position vector is initially provided instead. The vectors are then passed into \
spacecraftPlus and, subsequently, the eclipse module through the Basilisk messaging system.

Testing the no eclipse case with Mars as the occulting body is the same as the Earth no eclipse test, except \
Mars is set as the zero base. The Mars full, partial, and annular eclipse cases, however, are like the Earth \
annular case where Cartesian vectors are, instead, the initial inputs. Since the test is performed as a single \
step process, the velocity is not necessarily needed as an input, so only a position vector is provided \
for these cases.

Planet States
Once the spacecraft states are defined, the planet names are provided as Spice objects. Since the module \
determines the closest planet to the spacecraft, multiple names may be input and the chosen one depends \
purely on the position of the spacecraft. For this test, only Earth and Mars were used as occulting bodies, \
since no appreciable difference in the algorithm presents itself when testing various planets.

Comparison
The shadow factor obtained through the module is compared to the expected result, which is either trivial or \
calculated, depending on the eclipse type. Full eclipse and no eclipse shadow factors are compared without the \
need for computation, since they are just 0.0 and 1.0, respectively. The partial and annular eclipse shadow \
factors, however, vary between 0.0 and 1.0, based on the cone dimensions, and are calculated using \
MATLAB and Spice data.


Test Parameters:
-----------
- eclipseCondition: [string]
    defines if the eclipse is partial, full, none or annual
- planet: [string]
    defines which planet to use.  Options include "earth" and "mars"


Description of Variables Being Tested
-------------------------------------
In each test scenario the shadow eclipse variable

    shadowFactor

is pulled from the log data and compared to expected truth values.


Test Parameters
---------------
The previous description only briefly mentions the input parameters required to perform the tests on this module, so \
this section further details the parameters set by the user and built into the unit test.

Eclipse Condition Input:
One of the two user-defined inputs is the eclipse identifier, appropriately named eclipseCondition. This specifies \
the eclipse type and can be set, via string inputs, to either full, partial, annular, or none for the full, \
partial, annular, and no eclipse cases. This input changes the spacecraft state parameters to position the \
spacecraft in the desired eclipse.

Planet Input:
The second user input is planet, which allows the user to test either Earth or Mars as the occulting body. \
Only these two planets are tested since there is no appreciable difference in the eclipse algorithm when \
varying the planet. This input changes the zero base to reference the chosen planet and the spacecraft \
state parameters to position the spacecraft in the desired eclipse.

Zero Base:
The zero base references either Earth or Mars depending on the planet input.

Orbital Elements:
Orbital elements describe the characteristics of the orbit plane and are used in this module to define \
the states of the spacecraft. They consist of the semimajor axis a in units of kilometers, eccentricity e, \
inclination i, ascending node Omega, argument of pariapses omega, and the true anomaly f. The Euler angles \
and true anomaly are all in units of degrees but are converted to radians in the test. Using orbitalMotion, \
all of the elements are converted to Cartesian position and velocity vectors that are then translated from \
kilometers to meters. The table below shows the values used when testing full, partial, and no eclipse \
cases for Earth as well as the no eclipse case for Mars.

    Table: Orbital Element Values for Each Eclipse Condition
    Element     Full Eclipse        Partial Eclipse     No Eclipse
    --------------------------------------------------------------------
    a           6878.1366km         6878.1366km         9959991.68982km
    e           0.00001$            0.00001             0.00001
    i           5deg                5deg                5deg
    Omega       48.2deg             48.2deg             48.2deg
    omega       0deg                0deg                0deg
    f           173deg              107.5deg            107.5deg
    --------------------------------------------------------------------

The orbital elements remain the same when testing the no eclipse case for Earth and Mars, but the zero base \
is changed to reference the appropriate planet. The parameters used when testing full, partial, and annular \
eclipse cases for Mars as well as the annular case for Earth are shown under Cartesian Vectors.

Cartesian Vectors:
Cartesian position vectors are used as inputs when testing the full, partial, and annular eclipse cases for \
Mars as well as the annular case for Earth. Velocity vectors are not needed since this test is performed as \
a single step process and eclipses at a single point in time depend only on the position of the celestial \
bodies at that time. The vectors are shown in the Table below. The parameters for the no eclipse case were \
given previously under Orbital Elements.

    Caption: Position Vectors for Each Eclipse Condition
    ----------------------------------------------------
    Eclipse Type                X                   Y                   Z
    ----------------------------------------------------------------------------------
    Full Eclipse            -2930233.559m       2567609.100m        41384.233$ m
    Partial Eclipse         -6050166.455m       2813822.447m        571725.565 m
    Annular Eclipse (Earth) -326716535.629km   -287302983.139km    -124542549.301$ km
    Annular Eclipse (Mars)  -427424601.171km    541312532.797km     259820030.623 km
    ----------------------------------------------------------------------------------

Standard Gravitational Parameter:
The gravitational parameter mu is necessary for converting between orbital elements and Cartesian vectors. \
It is the product of a body's gravitational constant and mass, specifying the attracting body in units of \
km^3/s^2. This test only uses the conversion when considering Earth as the occulting body, so only Earth's \
gravitational parameter is given below. The value is obtained through orbitalMotion.

    mu_{Earth} = GM_{Earth}=398600.436 km^3/s^2


Planet Names:
The eclipse module accepts and will analyze multiple planets at a time, so to indicate which planets are \
of interest, the names must be input. These are input as Spice objects of type string and can either be \
venus, earth, or mars barycenter. Spice uses this information and the time input to provide state data, \
which is then used to determine the closest planet. The closest planet is the only one further evaluated \
for potential eclipse conditions.

Time:
In order to specify the states of the planets and sun, time is a neccessary input. Only one time is used \
for every test, fixing the sun and planet states and varying the states of the spacecraft. For this test, \
the input "2021 MAY 04 07:47:49.965 (UTC)" is used, where the time is represented in UTC form.

Spice Kernels:
Spice information is gathered from a collection of kernels, specfically SPK, LSK, and PCK files. \
The binary SPK used is de430.bsp, which provides ephemeris data. The LSK file is naif0012.tls, which offers \
leapsecond information. For reference frame orientation, two PCK files are given. These are de-403-masses.tpc \
and pck00010.tpc.

    """
    [testResults, testMessage] = unitEclipse(show_plots, eclipseCondition, planet)
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
    scObject_0.gravField.gravBodies = spacecraftPlus.GravBodyVector(list(gravFactory.gravBodies.values()))

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
            print("partial mars")
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
        print("PASSED: " + planet + "-" + eclipseCondition)
        # return fail count and join into a single string all messages in the list
        # testMessage

    print('The error tolerance for all tests is ' + str(errTol))

    #
    #  unload the SPICE libraries that were loaded by the spiceObject earlier
    #
    gravFactory.unloadSpiceKernels()

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    unitEclipse(False, "annular", "earth")
