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
# Eclipse Condition Unit Test
#
# Purpose:  Test the proper function of the Eclipse environment module.
#           This is done by comparing computed expected shadow factors in
#           particular eclipse conditions to what is simulated
# Author:   Patrick Kenneally
# Creation Date:  May. 31, 2017
#

# @cond DOXYGEN_IGNORE
import sys
import os
import pytest
import inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

import SimulationBaseClass
import unitTestSupport
import spacecraftPlus
import macros
import spice_interface
import eclipse
import pyswice
import gravityEffector
import orbitalMotion as om
import coarse_sun_sensor
import numpy as np


# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)
# @pytest.mark.parametrize()
# def test_unitCoarseSunSensor(show_plots, eclipseCondition):
#     [testResults, testMessage] = unitCoarseSunSensor(show_plots, eclipseCondition)
#     assert testResults < 1, testMessage


# def unitCoarseSunSensor():
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
satellite = spacecraftPlus.SpacecraftPlus()
satellite.scStateOutMsgName = "inertial_state_output"
satellite.hub.r_CN_NInit = [[-1.11277003e9 - om.AU], [7.08965219e8], [3.28738762e8]] #place spacecraft 1AU from sun in negative first inertial direciton in standard spice inertial frame
                                                                                # now body frame can be aligned with inertial frame for simplicity.
                                                                                # this only works if no one changes UTCCalInit below.
satellite.hub.sigma_BNInit = [[0.], [0.], [0.]]
satellite.hub.omega_BN_BInit = [[0.], [0.], [np.pi*2/60.]]
unitTestSim.AddModelToTask(testTaskName, satellite)


# cssSinglePlain = coarse_sun_sensor.CoarseSunSensor()
# cssSinglePlain.ModelTag = "cssSinglePlain"
# cssSinglePlain.fov = np.pi/2.
# cssSinglePlain.OutputDataMsg = "cssSinglePlainOut"
# cssSinglePlain.nHat_B = [0,0,1] #probably move to after orbit stuff to point at sun - SJKC


# setup SPICE ephemeris support
# earth = gravityEffector.GravBodyData()
# earth.bodyInMsgName = "earth_planet_data"
# earth.outputMsgName = "earth_display_frame_data"
# earth.mu = 0.3986004415E+15  # meters^3/s^2
# earth.radEquator = 6378136.6  # meters
# earth.isCentralBody = True
# earth.useSphericalHarmParams = False

# attach gravity model to spaceCraftPlus
# satellite.gravField.gravBodies = spacecraftPlus.GravBodyVector([earth])

spiceObject = spice_interface.SpiceInterface()
spiceObject.PlanetNames = spice_interface.StringVector(["sun"]) #, "earth"])
spiceObject.ModelTag = "SpiceInterfaceData"
spiceObject.SPICEDataPath = bskPath + 'External/EphemerisData/'
spiceObject.OutputBufferCount = 100000
spiceObject.UTCCalInit = '2021 MAY 04 07:47:49.965 (UTC)'
# pull in SPICE support libraries
pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
pyswice.furnsh_c(spiceObject.SPICEDataPath + 'naif0011.tls')  # leap second file
pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
pyswice.furnsh_c(spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

# if eclipseCondition == "full":
#     spiceObject.zeroBase = "earth"
#     # set up spacecraft 0 position and velocity for full eclipse
#     oe = orbitalMotion.ClassicElements()
#     r_0 = (500 + orbitalMotion.REQ_EARTH)  # km
#     oe.a = r_0
#     oe.e = 0.00001
#     oe.i = 5.0 * macros.D2R
#     oe.Omega = 48.2 * macros.D2R
#     oe.omega = 0 * macros.D2R
#     oe.f = 173 * macros.D2R
#     r_N_0, v_N_0 = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, oe)
#     satellite.hub.r_CN_NInit = r_N_0 * 1000  # convert to meters
#     satellite.hub.v_CN_NInit = v_N_0 * 1000  # convert to meters
# elif eclipseCondition == "partial":
#     spiceObject.zeroBase = "earth"
#     # set up spacecraft 0 position and velocity for full eclipse
#     oe = orbitalMotion.ClassicElements()
#     r_0 = (500 + orbitalMotion.REQ_EARTH)  # km
#     oe.a = r_0
#     oe.e = 0.00001
#     oe.i = 5.0 * macros.D2R
#     oe.Omega = 48.2 * macros.D2R
#     oe.omega = 0 * macros.D2R
#     oe.f = 107.5 * macros.D2R
#     r_N_0, v_N_0 = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, oe)
#     satellite.hub.r_CN_NInit = r_N_0 * 1000  # convert to meters
#     satellite.hub.v_CN_NInit = v_N_0 * 1000  # convert to meters
# elif eclipseCondition == "none":
#     oe = orbitalMotion.ClassicElements()
#     r_0 = 9959991.68982  # km
#     oe.a = r_0
#     oe.e = 0.00001
#     oe.i = 5.0 * macros.D2R
#     oe.Omega = 48.2 * macros.D2R
#     oe.omega = 0 * macros.D2R
#     oe.f = 107.5 * macros.D2R
#     r_N_0, v_N_0 = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, oe)
#     satellite.hub.r_CN_NInit = r_N_0 * 1000  # convert to meters
#     satellite.hub.v_CN_NInit = v_N_0 * 1000  # convert to meters

unitTestSim.AddModelToTask(testTaskName, spiceObject)
# eclipseObject = eclipse.Eclipse()
# eclipseObject.addPositionMsgName(satellite.scStateOutMsgName)
# eclipseObject.addPlanetName('earth')
# eclipseObject.addPlanetName('mars barycenter')
# eclipseObject.addPlanetName('venus')
# unitTestSim.AddModelToTask(testTaskName, eclipseObject)
# unitTestSim.TotalSim.logThisMessage("eclipse_data_0")



unitTestSim.InitializeSimulation()
unitTestSim.TotalSim.logThisMessage("sun_planet_data", macros.sec2nano(1.))
unitTestSim.TotalSim.logThisMessage(satellite.scStateOutMsgName, macros.sec2nano(1.))

# Execute the simulation for one time step
#unitTestSim.TotalSim.SingleStepProcesses()
simulationTime = macros.sec2nano(60.)
unitTestSim.ConfigureStopTime(simulationTime)
unitTestSim.ExecuteSimulation()

r_sun_N = unitTestSim.pullMessageLogData("sun_planet_data.PositionVector", range(3))
r_sat_N = unitTestSim.pullMessageLogData(satellite.scStateOutMsgName+".r_BN_N", range(3))
print r_sun_N
print r_sat_N

#eclipseData_0 = unitTestSim.pullMessageLogData("eclipse_data_0.shadowFactor")

# errTol = 1E-12
# if eclipseCondition is "partial":
#     truthShadowFactor = 0.62310760206735027
#     if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
#         testFailCount += 1
#         testMessages.append("Shadow Factor failed for partial eclipse condition")
#
# elif eclipseCondition is "full":
#     truthShadowFactor = 0.0
#     if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
#         testFailCount += 1
#         testMessages.append("Shadow Factor failed for full eclipse condition")
#
# elif eclipseCondition is "none":
#     truthShadowFactor = 1.0
#     if not unitTestSupport.isDoubleEqual(eclipseData_0[0, :], truthShadowFactor, errTol):
#         testFailCount += 1
#         testMessages.append("Shadow Factor failed for none eclipse condition")
#
# if testFailCount == 0:
#     print "PASSED: " + eclipseCondition
# return fail count and join into a single string all messages in the list
# testMessage

# return [testFailCount, ''.join(testMessages)]

# if __name__ == "__main__":
#     unitCoarseSunSensor(False, "full")
