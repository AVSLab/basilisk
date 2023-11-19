
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
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion, RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
import matplotlib.pyplot as plt
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.pyswice_spk_utilities import spkRead
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import planetEphemeris
from Basilisk.simulation import spacecraft
from Basilisk.utilities import simIncludeGravBody
import pytest

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
@pytest.mark.parametrize("function", ["singleGravityBody"
                                      , "multiBodyGravity"
                                      , "polyGravityBody"
                                      ])
def test_gravityEffectorAllTest(show_plots, function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '(show_plots)')
    assert testResults < 1, testMessage


def singleGravityBody(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    DynUnitTestProc = unitTestSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(10.0)))


    # setup Gravity Bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'sun', 'moon', 'jupiter barycenter'])
    gravBodies['earth'].isCentralBody = True
    gravBodies['earth'].useSphericalHarmonicsGravityModel(path + '/../_UnitTest/GGM03S.txt'
                                        , 40
                                        )
    stringCurrent = "2016 MAY 1 00:32:30.0"
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/', stringCurrent)
    gravFactory.spiceObject.zeroBase = 'Earth'

    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    unitTestSim.AddModelToTask(unitTaskName, gravFactory.spiceObject, 10)

    # Use the python spice utility to load in spacecraft SPICE ephemeris data
    # Note: this following SPICE data only lives in the Python environment, and is
    #       separate from the earlier SPICE setup that was loaded to BSK.  This is why
    #       all required SPICE libraries must be included when setting up and loading
    #       SPICE kernels in Python.
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.furnsh_c(path + '/../_UnitTest/hst_edited.bsp')

    unitTestSim.AddModelToTask(unitTaskName, scObject, 9)

    stateOut = spkRead('HUBBLE SPACE TELESCOPE', stringCurrent, 'J2000', 'EARTH')

    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    scObject.hub.r_CN_NInit = (1000.0*stateOut[0:3].reshape(3,1)).tolist()
    velStart = 1000.0*stateOut[3:6]
    scObject.hub.v_CN_NInit = (velStart.reshape(3,1)).tolist()
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.002], [0.003]]

    unitTestSim.InitializeSimulation()

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")

    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    dt = 50.0
    totalTime = 20000.0
    currentTime = 0.0
    posArray = []
    velArray = []
    posError = []
    while(currentTime < totalTime):
        unitTestSim.ConfigureStopTime(macros.sec2nano(currentTime + dt))
        unitTestSim.ExecuteSimulation()
        stateOut = spkRead('HUBBLE SPACE TELESCOPE', gravFactory.spiceObject.getCurrentTimeString(), 'J2000', 'EARTH')
        posCurr = posRef.getState()
        posCurr = [y for x in posCurr for y in x]
        posArray.append(posCurr)
        velCurr = velRef.getState()
        velCurr = [y for x in velCurr for y in x]
        velArray.append(velCurr)
        posDiff = numpy.array(posCurr) - stateOut[0:3]*1000.0
        posRow = [unitTestSim.TotalSim.CurrentNanos*1.0E-9]
        posRow.extend(posDiff.tolist())
        posError.append(posRow)
        assert numpy.linalg.norm(posDiff) < 1000.0

        currentTime += dt

    stateOut = spkRead('HUBBLE SPACE TELESCOPE', gravFactory.spiceObject.getCurrentTimeString(), 'J2000', 'EARTH')
    posArray = numpy.array(posArray)
    posError = numpy.array(posError)

    gravFactory.unloadSpiceKernels()
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.unload_c(path + '/../_UnitTest/hst_edited.bsp')

    print(numpy.max(abs(posError[:,1:4])))

    if show_plots:
        plt.close("all")
        plt.figure()
        plt.plot(posError[:, 0], posError[:, 1:4])
        plt.xlabel('Time (s)')
        plt.ylabel('Position Difference (m)')
        plt.show()
        plt.close("all")


    if testFailCount == 0:
        print("PASSED: " + " Single body with spherical harmonics")
    # return fail count and join into a single string all messages in the list
    # testMessage

    return [testFailCount, ''.join(testMessages)]

def multiBodyGravity(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    DynUnitTestProc = unitTestSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(5.0)))

    # setup Gravity Bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'mars barycenter', 'sun', 'moon', 'jupiter barycenter'])
    gravBodies['sun'].isCentralBody = True

    stringCurrent = "2008 September 19, 04:00:00.0"
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/', stringCurrent)
    gravFactory.spiceObject.zeroBase = 'Earth'

    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    unitTestSim.AddModelToTask(unitTaskName, gravFactory.spiceObject, 10)

    # Use the python spice utility to load in spacecraft SPICE ephemeris data
    # Note: this following SPICE data only lives in the Python environment, and is
    #       separate from the earlier SPICE setup that was loaded to BSK.  This is why
    #       all required SPICE libraries must be included when setting up and loading
    #       SPICE kernels in Python.
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.furnsh_c(path + '/../_UnitTest/nh_pred_od077.bsp')

    unitTestSim.AddModelToTask(unitTaskName, scObject, 9)

    stateOut = spkRead('NEW HORIZONS', stringCurrent, 'J2000', 'SUN')

    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    scObject.hub.r_CN_NInit = (1000.0*stateOut[0:3].reshape(3,1)).tolist()
    velStart = 1000.0*stateOut[3:6]
    scObject.hub.v_CN_NInit = (velStart.reshape(3,1)).tolist()
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.002], [0.003]]

    unitTestSim.InitializeSimulation()

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")

    dt = 50.0
    totalTime = 20000.0
    currentTime = 0.0
    posArray = []
    velArray = []
    posError = []
    posInc = []
    while currentTime < totalTime:
        unitTestSim.ConfigureStopTime(macros.sec2nano(currentTime + dt))
        unitTestSim.ExecuteSimulation()
        timeString = pyswice.et2utc_c(gravFactory.spiceObject.J2000Current, 'C', 4, 1024, "Yo")
        stateOut = spkRead('NEW HORIZONS', timeString, 'J2000', 'SUN')
        posCurr = posRef.getState()
        posCurr = [y for x in posCurr for y in x]
        posArray.append(posCurr)
        velCurr = velRef.getState()
        velCurr = [y for x in velCurr for y in x]
        velArray.append(velCurr)
        posDiff = numpy.array(posCurr) - stateOut[0:3]*1000.0
        posRow = [unitTestSim.TotalSim.CurrentNanos*1.0E-9]
        posRow.extend(posDiff.tolist())
        posError.append(posRow)
        assert numpy.linalg.norm(posDiff) < 1000.0
        if currentTime > 0.0 + dt/2.0:
            posJump = stateOut[0:3]*1000.0 - numpy.array(posPrevious)
            posInc.append(posJump.tolist())
        posPrevious = stateOut[0:3]*1000.0
        currentTime += dt

    stateOut = spkRead('NEW HORIZONS', gravFactory.spiceObject.getCurrentTimeString(), 'J2000', 'SUN')
    posArray = numpy.array(posArray)
    posError = numpy.array(posError)
    posInc = numpy.array(posInc)

    gravFactory.unloadSpiceKernels()
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.unload_c(path + '/../_UnitTest/nh_pred_od077.bsp')

    plt.close("all")
    plt.figure()
    plt.plot(posError[:,0], posError[:,1:4])
    plt.xlabel('Time (s)')
    plt.ylabel('Position Difference (m)')

    if(show_plots):
        plt.show()
        plt.close('all')

    if testFailCount == 0:
        print("PASSED: " + " multi-point source bodies")
    # return fail count and join into a single string all messages in the list
    # testMessage

    return [testFailCount, ''.join(testMessages)]


def polyGravityBody(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Obtain validation data (simulation with tight integration tolerances in MATLAB)
    valData = numpy.genfromtxt(path + '/../_UnitTest/polyTestData.csv', delimiter=',')
    tVal = numpy.array(valData[:,0])
    posVal = numpy.array(valData[:,1:4])
    velVal = numpy.array(valData[:,4:7])

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    DynUnitTestProc = unitTestSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    intTime = 30.0
    DynUnitTestProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(intTime)))

    # specify orbit of polyhedral body
    oePolyBody = planetEphemeris.ClassicElementsMsgPayload()
    oePolyBody.a = 2.3612 * orbitalMotion.AU * 1000
    oePolyBody.e = 0
    oePolyBody.i = 0*macros.D2R
    oePolyBody.Omega = 0*macros.D2R
    oePolyBody.omega = 0*macros.D2R
    oePolyBody.f = 0*macros.D2R

    raPolyBody = 0 * macros.D2R
    decPolyBody = 90 * macros.D2R
    lst0PolyBody = 0 * macros.D2R
    rotPeriodPolyBody = 5.27 * 3600

    # setup celestial object ephemeris module
    polyBodyEphem = planetEphemeris.PlanetEphemeris()
    polyBodyEphem.ModelTag = 'erosEphemeris'
    polyBodyEphem.setPlanetNames(planetEphemeris.StringVector(["eros"]))

    # specify celestial objects orbit
    polyBodyEphem.planetElements = planetEphemeris.classicElementVector([oePolyBody])

    # specify celestial object orientation
    polyBodyEphem.rightAscension = planetEphemeris.DoubleVector([raPolyBody])
    polyBodyEphem.declination = planetEphemeris.DoubleVector([decPolyBody])
    polyBodyEphem.lst0 = planetEphemeris.DoubleVector([lst0PolyBody])
    polyBodyEphem.rotRate = planetEphemeris.DoubleVector([360 * macros.D2R / rotPeriodPolyBody])

    # setup polyhedral gravity body
    mu = 4.46275472004 * 1e5
    gravFactory = simIncludeGravBody.gravBodyFactory()
    polyBody = gravFactory.createCustomGravObject('eros', mu=mu)
    polyBody.isCentralBody = True
    polyBody.usePolyhedralGravityModel(path + '/../_UnitTest/EROS856Vert1708Fac.txt')
    polyBody.planetBodyInMsg.subscribeTo(polyBodyEphem.planetOutMsgs[0])

    # create an ephemeris converter
    polyBodyEphemConverter = ephemerisConverter.EphemerisConverter()
    polyBodyEphemConverter.ModelTag = "erosEphemConverter"
    polyBodyEphemConverter.addSpiceInputMsg(polyBodyEphem.planetOutMsgs[0])

    # create spacecraft and attach polyhedral body
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # set initial conditions for spacecraft
    angvelPolyBody = np.array([0,0,360 * macros.D2R / rotPeriodPolyBody])
    posInit = posVal[0,0:3]
    velInit = velVal[0,0:3] + np.cross(angvelPolyBody, posInit)
    scObject.hub.r_CN_NInit = posInit.tolist()
    scObject.hub.v_CN_NInit = velInit.tolist()

    # add models to task
    unitTestSim.AddModelToTask(unitTaskName, polyBodyEphem, ModelPriority=10)
    unitTestSim.AddModelToTask(unitTaskName, polyBodyEphemConverter, ModelPriority=9)
    unitTestSim.AddModelToTask(unitTaskName, scObject, ModelPriority=8)

    totalTime = 24*3600

    samplingTime = 300
    scRec = scObject.scStateOutMsg.recorder(macros.sec2nano(samplingTime))
    polyBodyRec = polyBodyEphemConverter.ephemOutMsgs[0].recorder(macros.sec2nano(samplingTime))
    unitTestSim.AddModelToTask(unitTaskName, scRec)
    unitTestSim.AddModelToTask(unitTaskName, polyBodyRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(totalTime))
    unitTestSim.ExecuteSimulation()

    # retrieve logged variables
    time = scRec.times() * macros.NANO2SEC
    N_points = len(time)
    r_BN_N = scRec.r_BN_N
    r_AN_N = polyBodyRec.r_BdyZero_N
    sigma_AN = polyBodyRec.sigma_BN

    # obtain position in small body centered fixed frame
    posArray = numpy.zeros((N_points, 3))
    for ii in range(N_points):
        # obtain rotation matrix
        R_AN = RigidBodyKinematics.MRP2C(sigma_AN[ii][0:3])

        # rotate position and velocity
        posArray[ii,0:3] = R_AN.dot(numpy.subtract(r_BN_N[ii],r_AN_N[ii]))

    # compute error in position and assert max error
    posError = numpy.linalg.norm(posArray - posVal,axis=1)
    assert max(posError) < 10
    print(max(posError))

    plt.close("all")
    plt.figure()
    plt.plot(tVal, posArray - posVal)
    plt.xlabel('Time (s)')
    plt.ylabel('Position Difference (m)')

    if(show_plots):
        plt.show()
        plt.close('all')

    if testFailCount == 0:
        print("PASSED: " + " Single body with polyhedral shape")
    # return fail count and join into a single string all messages in the list
    # testMessage

    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    # gravityEffectorAllTest(False)
    singleGravityBody(True)
    # multiBodyGravity(True)
    polyGravityBody(True)
