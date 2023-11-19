
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
import math
import os

import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import macros
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import spiceInterface
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.pyswice_spk_utilities import spkRead
from Basilisk.simulation import stateArchitecture
from Basilisk.utilities import orbitalMotion as om
from Basilisk.architecture import messaging
from Basilisk.simulation.gravityEffector import loadGravFromFileToList

#script to check spherical harmonics calcs out to 20th degree
#Uses coefficient from Vallado tables D-1

def computeGravityTo20(positionVector):
    #This code follows the formulation in Vallado, page 521, second edition and uses data from UTexas CSR for
    #gravitation harmonics parameters
    #Written 201780807 by Scott Carnahan
    #AVS Lab | CU Boulder

    #INPUTS
    #positionVector - [x,y,z] coordinates list of spacecraft in [m] in earth body frame so that lat, long can be calculated

    def legendres(degree, alpha):
        P = np.zeros((degree+1,degree+1))
        P[0,0] = 1
        P[1,0] = alpha
        cosPhi = np.sqrt(1-alpha**2)
        P[1,1] = cosPhi

        for l in range(2,degree+1):
            for m in range(0,l+1):
                if m == 0 and l >= 2:
                    P[l,m] = ((2*l-1)*alpha*P[l-1,0]-(l-1)*P[l-2,0]) / l
                elif m != 0 and m < l:
                    P[l, m] = (P[l-2, m]+(2*l-1)*cosPhi*P[l-1,m-1])
                elif m == l and l != 0:
                    P[l,m] = (2*l-1)*cosPhi*P[l-1,m-1]
                else:
                    print(l,", ", m)
        return P

    maxDegree = 20
    cList = np.zeros(maxDegree+2)
    sList = np.zeros(maxDegree+2)
    muEarth = 0.
    radEarth = 0.
    [cList, sList, muEarth, radEarth]  = loadGravFromFileToList(path + '/GGM03S.txt', maxDegree+2)

    r = np.linalg.norm(positionVector)
    rHat = positionVector / r
    gHat = rHat
    grav0 = -gHat * muEarth / r**2

    rI = positionVector[0]
    rJ = positionVector[1]
    rK = positionVector[2]

    rIJ = np.sqrt(rI**2 + rJ**2)
    if rIJ != 0.:
        phi = math.atan(rK / rIJ) #latitude in radians
    else:
        phi = math.copysign(np.pi/2., rK)
    if rI != 0.:
        lambdaSat = math.atan(rJ / rI) #longitude in radians
    else:
        lambdaSat = math.copysign(np.pi/2., rJ)

    P = legendres(maxDegree+1,np.sin(phi))

    dUdr = 0.
    dUdphi = 0.
    dUdlambda = 0.

    for l in range(0, maxDegree+1):
        for m in range(0,l+1):
            if m == 0:
                k = 1
            else:
                k = 2
            num = math.factorial(l+m)
            den = math.factorial(l-m)*k*(2*l+1)
            PI = np.sqrt(float(num)/float(den))
            cList[l][m] = cList[l][m] / PI
            sList[l][m] = sList[l][m] / PI

    for l in range(2,maxDegree+1): #can only do for max degree minus 1
        for m in range(0,l+1):
            dUdr = dUdr + (((radEarth/r)**l)*(l+1)*P[l,m]) * (cList[l][m]*np.cos(m*lambdaSat)+sList[l][m]*np.sin(m*lambdaSat))
            dUdphi = dUdphi + (((radEarth/r)**l)*P[l,m+1] - m*np.tan(phi)*P[l,m]) * (cList[l][m]*np.cos(m*lambdaSat) + sList[l][m]*np.sin(m*lambdaSat))
            dUdlambda = dUdlambda + (((radEarth/r)**l)*m*P[l,m]) * (sList[l][m]*np.cos(m*lambdaSat) - cList[l][m]*np.sin(m*lambdaSat))

    dUdr = -muEarth * dUdr / r**2
    dUdphi = muEarth * dUdphi / r
    dUdlambda = muEarth * dUdlambda / r


    if rI != 0. and rJ != 0.:
        accelerationI = (dUdr/r - rK*dUdphi/(r**2)/((rI**2+rJ**2)**0.5))*rI - (dUdlambda/(rI**2+rJ**2))*rJ + grav0[0]
        accelerationJ = (dUdr/r - rK*dUdphi/(r**2)/((rI**2+rJ**2)**0.5))*rJ + (dUdlambda/(rI**2+rJ**2))*rI + grav0[1]
    else:
        accelerationI = dUdr/r + grav0[0]
        accelerationJ = dUdr/r + grav0[1]
    accelerationK = (dUdr/r)*rK + (((rI**2+rJ**2)**0.5)*dUdphi/(r**2)) + grav0[2]

    accelerationVector = [accelerationI, accelerationJ, accelerationK]

    return accelerationVector

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_gravityEffectorAllTest(show_plots):
    """Module Unit Test"""
    [testResults, testMessage] = independentSphericalHarmonics(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = sphericalHarmonics(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = singleGravityBody(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = multiBodyGravity(show_plots)
    assert testResults < 1, testMessage

def independentSphericalHarmonics(show_plots):
    testCase = "independentCheck"
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    spherHarm = gravityEffector.SphericalHarmonics()

    gravityEffector.loadGravFromFile(path + '/GGM03S.txt', spherHarm, 20)
    gravCheck = computeGravityTo20([15000., 10000., 6378.1363E3])
    spherHarm.initializeParameters()
    gravOut = spherHarm.computeField([[15000.0], [10000.0], [(6378.1363) * 1.0E3]], 20, True)
    gravOutMag = np.linalg.norm(gravOut)
    gravCheckMag = np.linalg.norm(gravCheck)

    accuracy = 1e-12
    relative = (gravCheckMag-gravOutMag)/gravCheckMag
    if abs(relative) > accuracy:
        testFailCount += 1
        testMessages.append("Failed independent spherical harmonics check")
    snippetName = testCase + 'Accuracy'
    snippetContent = '{:1.1e}'.format(accuracy)  # write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent,
                                    path)  # write formatted LATEX string to file to be used by auto-documentation.

    if testFailCount == 0:
        passFailText = 'PASSED'
        print("PASSED: " + testCase)
        colorText = 'ForestGreen'  # color to write auto-documented "PASSED" message in in LATEX.
        snippetName = testCase + 'FailMsg'
        snippetContent = ""
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent,
                                        path)  # write formatted LATEX string to file to be used by auto-documentation.
    else:
        passFailText = 'FAILED'
        colorText = 'Red'  # color to write auto-documented "FAILED" message in in LATEX
        snippetName = testCase + 'FailMsg'
        snippetContent = passFailText
        for message in testMessages:
            snippetContent += ". " + message
        snippetContent += "."
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent,
                                        path)  # write formatted LATEX string to file to be used by auto-documentation.
    snippetName = testCase + 'PassFail'  # name of file to be written for auto-documentation which specifies if this test was passed or failed.
    snippetContent = r'\textcolor{' + colorText + '}{' + passFailText + '}'  # write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent,
                                    path)  # write formatted LATEX string to file to be used by auto-documentation.

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def sphericalHarmonics(show_plots):
    testCase = 'sphericalHarmonics'
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    spherHarm = gravityEffector.SphericalHarmonics()

    testHarm = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    spherHarm.cBar = gravityEffector.MultiArray(testHarm)

    vecCheckSuccess = True
    for i in range(len(spherHarm.cBar)):
        for j in range(len(spherHarm.cBar[i])):
            if spherHarm.cBar[i][j] != testHarm[i][j]:
                vecCheckSuccess = False


    if(vecCheckSuccess != True):
        testFailCount += 1
        testMessages.append("2D vector not input appropriately to spherical harmonics")

    gravityEffector.loadGravFromFile(path + '/GGM03S.txt', spherHarm, 20)
    spherHarm.initializeParameters()
    gravOut = spherHarm.computeField([[0.0], [0.0], [(6378.1363)*1.0E3]], 20, True)
    gravMag = np.linalg.norm(np.array(gravOut))

    accuracy = 0.1
    gravExpected = 9.8
    if gravMag > (gravExpected + accuracy) or gravMag < (gravExpected - accuracy):
        testFailCount += 1
        testMessages.append("Gravity magnitude not within allowable tolerance")
    snippetName = testCase + 'Accuracy'
    snippetContent = '{:1.1e}'.format(accuracy)  # write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.

    try:
        spherHarm.computeField([[0.0], [0.0], [(6378.1363)*1.0E3]], 100, True)
        testFailCount += 1
        testMessages.append("Gravity ceiling not enforced correctly")
    except RuntimeError:
        pass # Great! We threw an error

    if testFailCount == 0:
        passFailText = 'PASSED'
        print("PASSED: " + " Spherical Harmonics")
        colorText = 'ForestGreen'  # color to write auto-documented "PASSED" message in in LATEX.
        snippetName = testCase + 'FailMsg'
        snippetContent = ""
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    else:
        passFailText = 'FAILED'
        colorText = 'Red'  # color to write auto-documented "FAILED" message in in LATEX
        snippetName = testCase + 'FailMsg'
        snippetContent = passFailText
        for message in testMessages:
            snippetContent += ". " + message
        snippetContent += "."
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    snippetName = testCase + 'PassFail'  # name of file to be written for auto-documentation which specifies if this test was passed or failed.
    snippetContent = r'\textcolor{' + colorText + '}{' + passFailText + '}' #write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def singleGravityBody(show_plots):
    testCase = 'singleBody'
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    DateSpice = "2015 February 10, 00:00:00.0 TDB"

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()

    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))

    # Initialize the modules that we are using.
    SpiceObject = spiceInterface.SpiceInterface()

    SpiceObject.ModelTag = "SpiceInterfaceData"
    SpiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
    SpiceObject.addPlanetNames(spiceInterface.StringVector(["earth", "mars barycenter", "sun"]))
    SpiceObject.UTCCalInit = DateSpice
    TotalSim.AddModelToTask(unitTaskName, SpiceObject)
    SpiceObject.UTCCalInit = "1994 JAN 26 00:02:00.184"

    gravBody1 = gravityEffector.GravBodyData()
    gravBody1.planetName = "earth_planet_data"
    gravBody1.isCentralBody = False
    gravBody1.useSphericalHarmonicsGravityModel(path + '/GGM03S.txt', 60)
    gravBody1.planetBodyInMsg.subscribeTo(SpiceObject.planetStateOutMsgs[0])

    # Use the python spice utility to load in spacecraft SPICE ephemeris data
    # Note: this following SPICE data only lives in the Python environment, and is
    #       separate from the earlier SPICE setup that was loaded to BSK.  This is why
    #       all required SPICE libraries must be included when setting up and loading
    #       SPICE kernals in Python.
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.furnsh_c(path + '/hst_edited.bsp')

    SpiceObject.UTCCalInit = "2012 MAY 1 00:02:00.184"
    stringCurrent = SpiceObject.UTCCalInit
    et = pyswice.new_doubleArray(1)
    dt = 1.0
    pyswice.str2et_c(stringCurrent, et)
    etCurr = pyswice.doubleArray_getitem(et, 0)
    normVec = []
    gravErrNorm = []
    SpiceObject.UTCCalInit = stringCurrent
    TotalSim.InitializeSimulation()
    gravBody1.initBody(0)
    newManager = stateArchitecture.DynParamManager()
    gravBody1.registerProperties(newManager)
    SpiceObject.UpdateState(0)

    for i in range(2*3600):
        stateOut = spkRead('HUBBLE SPACE TELESCOPE', stringCurrent, 'J2000', 'EARTH')
        etPrev =etCurr - 2.0
        stringPrev = pyswice.et2utc_c(etPrev, 'C', 4, 1024, "Yo")
        statePrev = spkRead('HUBBLE SPACE TELESCOPE', stringPrev, 'J2000', 'EARTH')
        etNext =etCurr + 2.0
        stringNext = pyswice.et2utc_c(etNext, 'C', 4, 1024, "Yo")
        stateNext = spkRead('HUBBLE SPACE TELESCOPE', stringNext, 'J2000', 'EARTH')
        gravVec = (stateNext[3:6] - statePrev[3:6])/(etNext - etPrev)
        normVec.append(np.linalg.norm(stateOut[0:3]))

        stateOut*=1000.0
        SpiceObject.J2000Current = etCurr;SpiceObject.UpdateState(0)
        gravBody1.loadEphemeris()
        gravOut = gravBody1.computeGravityInertial(stateOut[0:3].reshape(3,1).tolist(), 0)
        gravErrNorm.append(np.linalg.norm(gravVec*1000.0 - np.array(gravOut).reshape(3))/
            np.linalg.norm(gravVec*1000.0))

        pyswice.str2et_c(stringCurrent, et)
        etCurr = pyswice.doubleArray_getitem(et, 0)
        etCurr += dt;
        stringCurrent = pyswice.et2utc_c(etCurr, 'C', 4, 1024, "Yo")

    accuracy = 1.0e-4
    for gravErr in gravErrNorm:
        if gravErr > accuracy:
            testFailCount += 1
            testMessages.append("Gravity numerical error too high for kernel comparison")
            break
    snippetName = testCase + 'Accuracy'
    snippetContent = '{:1.1e}'.format(accuracy)  # write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.

    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/naif0012.tls')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/de-403-masses.tpc')
    pyswice.unload_c(bskPath + '/supportData/EphemerisData/pck00010.tpc')
    pyswice.unload_c(path + '/hst_edited.bsp')


    if testFailCount == 0:
        passFailText = 'PASSED'
        print("PASSED: " + "Single-body with Spherical Harmonics")
        colorText = 'ForestGreen'  # color to write auto-documented "PASSED" message in in LATEX
        snippetName = testCase + 'FailMsg'
        snippetContent = ""
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    else:
        passFailText = 'FAILED'
        colorText = 'Red'  # color to write auto-documented "FAILED" message in in LATEX
        snippetName = testCase + 'FailMsg'
        snippetContent = passFailText
        for message in testMessages:
            snippetContent += ". " + message
        snippetContent += "."
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    snippetName = testCase + 'PassFail'  # name of file to be written for auto-documentation which specifies if this test was passed or failed.
    snippetContent = r'\textcolor{' + colorText + '}{' + passFailText + '}' #write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.


    # return fail count and join into a single string all messages in the list
    # testMessage

    return [testFailCount, ''.join(testMessages)]

def register(manager):
    """
    populates the state engines dynParamManager with nominal values

    :param manager:
    :return: posVelSig, posVelSig
    """
    positionName = "hubPosition"
    stateDim = [3, 1]
    posState = manager.registerState(stateDim[0], stateDim[1], positionName)
    posVelSig = [[0.], [0.], [0.]]
    posState.setState(posVelSig)
    velocityName = "hubVelocity"
    stateDim = [3, 1]
    velState = manager.registerState(stateDim[0], stateDim[1], velocityName)
    velState.setState(posVelSig)
    sigmaName = "hubSigma"
    stateDim = [3, 1]
    sigmaState = manager.registerState(stateDim[0], stateDim[1], sigmaName)
    sigmaState.setState(posVelSig)
    initC_B = [[0.0], [0.0], [0.0]]
    manager.createProperty("centerOfMassSC", initC_B)
    manager.createProperty("systemTime", [[0], [0.0]])

    return

def multiBodyGravity(show_plots):
    testCase = 'multiBody' #for AutoTeX stuff
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    #
    # # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    # DateSpice = "2015 February 10, 00:00:00.0 TDB"
    #
    # # Create a sim module as an empty container
    multiSim = SimulationBaseClass.SimBaseClass()
    #
    DynUnitTestProc = multiSim.CreateNewProcess(unitProcessName)
    # # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(multiSim.CreateNewTask(unitTaskName, macros.sec2nano(1000.0)))

    #Create dynParamManager to feed fake spacecraft data to so that the gravityEffector can access it.
    #This places the spacecraft at the coordinate frame origin so that planets can be placed around it.
    #velocity and attitude are just set to zero.
    #center of mass and time are set to zero.
    newManager = stateArchitecture.DynParamManager()
    register(newManager)


    #Create a message struct to place gravBody1 where it is wanted
    localPlanetEditor = messaging.SpicePlanetStateMsgPayload()
    localPlanetEditor.PositionVector = [om.AU/10., 0., 0.]
    localPlanetEditor.VelocityVector = [0., 0., 0.]
    localPlanetEditor.J20002Pfix = np.identity(3)

    #Grav Body 1 is twice the size of the other two
    gravBody1 = gravityEffector.GravBodyData()
    gravBody1.planetName = "gravBody1_planet_data"
    gravBody1.mu = 1000000.
    gravBody1.radEquator = 6500.
    gravBody1.isCentralBody = False
    gravBody1.localPlanet = localPlanetEditor

    #This is the gravityEffector which will actually compute the gravitational acceleration
    allGrav = gravityEffector.GravityEffector()
    allGrav.gravBodies = gravityEffector.GravBodyVector([gravBody1])
    allGrav.linkInStates(newManager)
    allGrav.registerProperties(newManager)
    allGrav.Reset(0)
    multiSim.AddModelToTask(unitTaskName, allGrav)
    posVelSig = [[0.], [0.], [0.]]
    allGrav.computeGravityField(posVelSig, posVelSig) #compute acceleration only considering the first body.
    step1 = newManager.getPropertyReference("g_N") #retrieve total gravitational acceleration in inertial frame

    #Create a message struct to place gravBody2&3 where they are wanted.
    localPlanetEditor.PositionVector = [-om.AU/10., 0., 0.]
    localPlanetEditor.VelocityVector = [0., 0., 0.]

    #grav Body 2 and 3 are coincident with each other, half the mass of gravBody1 and are in the opposite direction of gravBody1
    gravBody2 = gravityEffector.GravBodyData()
    gravBody2.planetName = "gravBody2_planet_data"
    gravBody2.mu = gravBody1.mu/2.
    gravBody2.radEquator = 6500.
    gravBody2.isCentralBody = False
    gravBody2.localPlanet = localPlanetEditor

    #This is the gravityEffector which will actually compute the gravitational acceleration
    newManager = stateArchitecture.DynParamManager()
    register(newManager)
    allGrav2 = gravityEffector.GravityEffector()
    allGrav2.gravBodies = gravityEffector.GravBodyVector([gravBody1, gravBody2])
    allGrav2.linkInStates(newManager)
    allGrav2.registerProperties(newManager)
    allGrav2.Reset(0)
    multiSim.AddModelToTask(unitTaskName, allGrav2)
    allGrav2.computeGravityField(posVelSig, posVelSig) #compute acceleration considering the first and second bodies.
    step2 = newManager.getPropertyReference("g_N") #retrieve total gravitational acceleration in inertial frame
    # grav Body 2 and 3 are coincident with each other, half the mass of gravBody1 and are in the opposite direction of gravBody1
    gravBody3 = gravityEffector.GravBodyData()
    gravBody3.planetName = "gravBody3_planet_data"
    gravBody3.mu = gravBody2.mu
    gravBody3.radEquator = 6500.
    gravBody3.isCentralBody = False
    gravBody3.localPlanet = localPlanetEditor

    #This is the gravityEffector which will actually compute the gravitational acceleration
    newManager = stateArchitecture.DynParamManager()
    register(newManager)
    allGrav3 = gravityEffector.GravityEffector()
    allGrav3.gravBodies = gravityEffector.GravBodyVector([gravBody1, gravBody2, gravBody3])
    allGrav3.linkInStates(newManager)
    allGrav3.registerProperties(newManager)
    allGrav3.Reset(0)
    multiSim.AddModelToTask(unitTaskName, allGrav3)
    allGrav3.computeGravityField(posVelSig, posVelSig) #comput acceleration considering all three bodies
    step3 = newManager.getPropertyReference("g_N") #retrieve total gravitational acceleration in inertial frame

    step3 = [0., step3[0][0], step3[1][0], step3[2][0]] #add a first (time) column to use isArrayZero

    #Test results for accuracy
    accuracy = 1e-12
    snippetName = testCase + 'Accuracy'
    snippetContent = '{:1.1e}'.format(accuracy)  # write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.
    if not unitTestSupport.isDoubleEqualRelative(step2[0][0]/step1[0][0], 0.5, accuracy): #if the second grav body doesn't cancel exactly half of the first body's acceleration.
        testFailCount += 1
        passFailText = "Step 2 was not half of step 1"
        testMessages.append(passFailText)
    elif not unitTestSupport.isArrayZero(step3, 3, accuracy): #if the net acceleration is not now 0.
        testFailCount += 1
        passFailText = "Step 3 did not cause gravity to return to 0"
        testMessages.append(passFailText)

    #Record test results to LaTeX
    if testFailCount == 0:
        passFailText = 'PASSED'
        print("PASSED: " + " Multi-Body")
        colorText = 'ForestGreen'  # color to write auto-documented "PASSED" message in in LATEX
        snippetName = testCase + 'FailMsg'
        snippetContent = ""
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    else:
        passFailText = 'FAILED'
        colorText = 'Red'  # color to write auto-documented "FAILED" message in in LATEX
        snippetName = testCase + 'FailMsg'
        snippetContent = passFailText
        for message in testMessages:
            snippetContent += ". " + message
        snippetContent += "."
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    snippetName = testCase + 'PassFail'  # name of file to be written for auto-documentation which specifies if this test was passed or failed.
    snippetContent = r'\textcolor{' + colorText + '}{' + passFailText + '}' #write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    # test_gravityEffectorAllTest(False)
    # independentSphericalHarmonics(False)
    # sphericalHarmonics(False)
    # singleGravityBody(True)
    multiBodyGravity(True)