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
import os, inspect
import numpy

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__
bskPath = __path__[0]


from Basilisk.utilities import SimulationBaseClass
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spice_interface
from Basilisk import pyswice
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import simIncludeGravBody

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


    # setup Gravity Bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'sun', 'moon', 'jupiter barycenter'])
    gravBodies['earth'].isCentralBody = True
    gravBodies['earth'].useSphericalHarmParams = True
    simIncludeGravBody.loadGravFromFile(path + '/../_UnitTest/GGM03S.txt'
                                        , gravBodies['earth'].spherHarm
                                        , 40
                                        )
    stringCurrent = "2016 MAY 1 00:32:30.0"
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/', stringCurrent)
    gravFactory.spiceObject.zeroBase = 'Earth'

    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    unitTestSim.AddModelToTask(unitTaskName, gravFactory.spiceObject, None, 10)

    # Use the python spice utility to load in spacecraft SPICE ephemeris data
    # Note: this following SPICE data only lives in the Python environment, and is
    #       separate from the earlier SPICE setup that was loaded to BSK.  This is why
    #       all required SPICE libraries must be included when setting up and loading
    #       SPICE kernals in Python.
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.furnsh_c(path + '/../_UnitTest/hst_edited.bsp')


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
        stateOut = pyswice.spkRead('HUBBLE SPACE TELESCOPE', gravFactory.spiceObject.getCurrentTimeString(), 'J2000', 'EARTH')
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

    stateOut = pyswice.spkRead('HUBBLE SPACE TELESCOPE', gravFactory.spiceObject.getCurrentTimeString(), 'J2000', 'EARTH')
    posArray = numpy.array(posArray)
    posError = numpy.array(posError)

    gravFactory.unloadSpiceKernels()
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
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

    # setup Gravity Bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravBodies = gravFactory.createBodies(['earth', 'mars barycenter', 'sun', 'moon', 'jupiter barycenter'])
    gravBodies['sun'].isCentralBody = True

    stringCurrent = "2008 September 19, 04:00:00.0"
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/', stringCurrent)
    gravFactory.spiceObject.zeroBase = 'Earth'

    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    unitTestSim.AddModelToTask(unitTaskName, gravFactory.spiceObject, None, 10)

    # Use the python spice utility to load in spacecraft SPICE ephemeris data
    # Note: this following SPICE data only lives in the Python environment, and is
    #       separate from the earlier SPICE setup that was loaded to BSK.  This is why
    #       all required SPICE libraries must be included when setting up and loading
    #       SPICE kernals in Python.
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.furnsh_c(path + '/../_UnitTest/nh_pred_od077.bsp')

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
        timeString = pyswice.et2utc_c(gravFactory.spiceObject.J2000Current, 'C', 4, 1024, "Yo")
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

    stateOut = pyswice.spkRead('NEW HORIZONS', gravFactory.spiceObject.getCurrentTimeString(), 'J2000', 'SUN')
    posArray = numpy.array(posArray)
    posError = numpy.array(posError)
    posInc = numpy.array(posInc)

    gravFactory.unloadSpiceKernels()
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.unload_c(path + '/../_UnitTest/nh_pred_od077.bsp')

    plt.figure()
    plt.plot(posError[:,0], posError[:,1:4])
    plt.xlabel('Time (s)')
    plt.ylabel('Position Difference (m)')

    if(show_plots):
        plt.show()
        plt.close('all')

    if testFailCount == 0:
        print "PASSED: " + " multi-point source bodies"
    # return fail count and join into a single string all messages in the list
    # testMessage

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    gravityEffectorAllTest(False)
