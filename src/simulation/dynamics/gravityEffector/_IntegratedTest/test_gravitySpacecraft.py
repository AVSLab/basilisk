''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
import numpy

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__
bskPath = __path__[0]


from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spice_interface
from Basilisk.simulation import sim_model
from Basilisk import pyswice
from Basilisk.simulation import stateArchitecture
from Basilisk.simulation import spacecraftPlus

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def gravityEffectorAllTest(show_plots):
    [testResults, testMessage] = test_singleGravityBody(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_multiBodyGravity(show_plots)
    assert testResults < 1, testMessage

def test_singleGravityBody(show_plots):
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
    unitTestSim.TotalSim.terminateSimulation()

    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    DynUnitTestProc = unitTestSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(10.0)))

    # Initialize the modules that we are using.
    unitTestSim.SpiceObject = spice_interface.SpiceInterface()

    unitTestSim.SpiceObject.ModelTag = "SpiceInterfaceData"
    unitTestSim.SpiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
    unitTestSim.SpiceObject.outputBufferCount = 10000
    unitTestSim.SpiceObject.planetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun", "moon", "jupiter barycenter"])
    unitTestSim.SpiceObject.zeroBase = 'Earth'

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
    unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
    unitTestSim.earthGravBody.isCentralBody = True
    unitTestSim.earthGravBody.useSphericalHarmParams = True
    gravityEffector.loadGravFromFile(path + '/../_UnitTest/GGM03S.txt', unitTestSim.earthGravBody.spherHarm, 40)

    unitTestSim.sunGravBody = gravityEffector.GravBodyData()
    unitTestSim.sunGravBody.bodyInMsgName = "sun_planet_data"
    unitTestSim.sunGravBody.outputMsgName = "sun_display_frame_data"
    unitTestSim.sunGravBody.mu = 1.32712440018E20  # meters!
    unitTestSim.sunGravBody.isCentralBody = False
    unitTestSim.sunGravBody.useSphericalHarmParams = False

    unitTestSim.moonGravBody = gravityEffector.GravBodyData()
    unitTestSim.moonGravBody.bodyInMsgName = "moon_planet_data"
    unitTestSim.moonGravBody.outputMsgName = "moon_display_frame_data"
    unitTestSim.moonGravBody.mu = 4.902799E12  # meters!
    unitTestSim.moonGravBody.isCentralBody = False
    unitTestSim.moonGravBody.useSphericalHarmParams = False

    unitTestSim.jupiterGravBody = gravityEffector.GravBodyData()
    unitTestSim.jupiterGravBody.bodyInMsgName = "jupiter barycenter_planet_data"
    unitTestSim.jupiterGravBody.outputMsgName = "jupiter_display_frame_data"
    unitTestSim.jupiterGravBody.mu = 1.266865349093058E17 # meters!
    unitTestSim.jupiterGravBody.isCentralBody = False
    unitTestSim.jupiterGravBody.useSphericalHarmParams = False

    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0011.tls')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.furnsh_c(path + '/../_UnitTest/hst_edited.bsp')

    #unitTestSim.SpiceObject.UTCCalInit = "2012 MAY 1 00:28:30.0"
    unitTestSim.SpiceObject.UTCCalInit = "2016 MAY 1 00:32:30.0"
    stringCurrent = unitTestSim.SpiceObject.UTCCalInit

    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody, unitTestSim.sunGravBody, unitTestSim.moonGravBody, unitTestSim.jupiterGravBody])

    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.SpiceObject, None, 10)
    unitTestSim.AddModelToTask(unitTaskName, scObject, None, 9)

    stateOut = pyswice.spkRead('HUBBLE SPACE TELESCOPE', stringCurrent, 'J2000', 'EARTH')

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
        stateOut = pyswice.spkRead('HUBBLE SPACE TELESCOPE', unitTestSim.SpiceObject.getCurrentTimeString(), 'J2000', 'EARTH')
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

    stateOut = pyswice.spkRead('HUBBLE SPACE TELESCOPE', unitTestSim.SpiceObject.getCurrentTimeString(), 'J2000', 'EARTH')
    posArray = numpy.array(posArray)
    posError = numpy.array(posError)

    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/naif0011.tls')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.unload_c(path + '/../_UnitTest/hst_edited.bsp')

    print numpy.max(abs(posError[:,1:4]))

    plt.figure()
    plt.plot(posError[:,0], posError[:,1:4])
    plt.xlabel('Time (s)')
    plt.ylabel('Position Difference (m)')


    if testFailCount == 0:
        print "PASSED: " + " Single body with spherical harmonics"
    # return fail count and join into a single string all messages in the list
    # testMessage

    return [testFailCount, ''.join(testMessages)]

def test_multiBodyGravity(show_plots):
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
    unitTestSim.TotalSim.terminateSimulation()

    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    DynUnitTestProc = unitTestSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(5.0)))

    # Initialize the modules that we are using.
    unitTestSim.SpiceObject = spice_interface.SpiceInterface()

    unitTestSim.SpiceObject.ModelTag = "SpiceInterfaceData"
    unitTestSim.SpiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
    unitTestSim.SpiceObject.outputBufferCount = 10000
    unitTestSim.SpiceObject.planetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun", "moon", "jupiter barycenter"])

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
    unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
    unitTestSim.earthGravBody.isCentralBody = False
    unitTestSim.earthGravBody.useSphericalHarmParams = False
    unitTestSim.earthGravBody.mu = 3.986004415E14

    unitTestSim.sunGravBody = gravityEffector.GravBodyData()
    unitTestSim.sunGravBody.bodyInMsgName = "sun_planet_data"
    unitTestSim.sunGravBody.outputMsgName = "sun_display_frame_data"
    unitTestSim.sunGravBody.mu = 1.32712440018E20  # meters!
    unitTestSim.sunGravBody.isCentralBody = True
    unitTestSim.sunGravBody.useSphericalHarmParams = False

    unitTestSim.moonGravBody = gravityEffector.GravBodyData()
    unitTestSim.moonGravBody.bodyInMsgName = "moon_planet_data"
    unitTestSim.moonGravBody.outputMsgName = "moon_display_frame_data"
    unitTestSim.moonGravBody.mu = 4.902799E12  # meters!
    unitTestSim.moonGravBody.isCentralBody = False
    unitTestSim.moonGravBody.useSphericalHarmParams = False

    unitTestSim.marsGravBody = gravityEffector.GravBodyData()
    unitTestSim.marsGravBody.bodyInMsgName = "moon_planet_data"
    unitTestSim.marsGravBody.outputMsgName = "moon_display_frame_data"
    unitTestSim.marsGravBody.mu = 4.2828371901284001E+13  # meters!
    unitTestSim.marsGravBody.isCentralBody = False
    unitTestSim.marsGravBody.useSphericalHarmParams = False

    unitTestSim.jupiterGravBody = gravityEffector.GravBodyData()
    unitTestSim.jupiterGravBody.bodyInMsgName = "jupiter barycenter_planet_data"
    unitTestSim.jupiterGravBody.outputMsgName = "jupiter_display_frame_data"
    unitTestSim.jupiterGravBody.mu = 1.266865349093058E17 # meters!
    unitTestSim.jupiterGravBody.isCentralBody = False
    unitTestSim.jupiterGravBody.useSphericalHarmParams = False

    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0011.tls')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.furnsh_c(path + '/../_UnitTest/nh_pred_od077.bsp')

    unitTestSim.SpiceObject.UTCCalInit = "2008 September 19, 04:00:00.0"
    stringCurrent = unitTestSim.SpiceObject.UTCCalInit

    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody, unitTestSim.sunGravBody, unitTestSim.marsGravBody, unitTestSim.jupiterGravBody])

    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.SpiceObject, None, 10)
    unitTestSim.AddModelToTask(unitTaskName, scObject, None, 9)

    stateOut = pyswice.spkRead('NEW HORIZONS', stringCurrent, 'J2000', 'SUN')

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
        timeString = pyswice.et2utc_c(unitTestSim.SpiceObject.J2000Current, 'C', 4, 1024, "Yo")
        stateOut = pyswice.spkRead('NEW HORIZONS', timeString, 'J2000', 'SUN')
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

    stateOut = pyswice.spkRead('NEW HORIZONS', unitTestSim.SpiceObject.getCurrentTimeString(), 'J2000', 'SUN')
    posArray = numpy.array(posArray)
    posError = numpy.array(posError)
    posInc = numpy.array(posInc)

    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/naif0011.tls')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.unload_c(path + '/../_UnitTest/nh_pred_od077.bsp')

    plt.figure()
    plt.plot(posError[:,0], posError[:,1:4])
    plt.xlabel('Time (s)')
    plt.ylabel('Position Difference (m)')

    if(show_plots):
        plt.show()

    if testFailCount == 0:
        print "PASSED: " + " multi-point source bodies"
    # return fail count and join into a single string all messages in the list
    # testMessage

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    gravityEffectorAllTest(False)
