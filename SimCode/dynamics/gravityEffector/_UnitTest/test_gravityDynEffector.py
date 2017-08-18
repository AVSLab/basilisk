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
import sys, os, inspect
import pytest
import math
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')
sys.path.append(splitPath[0] + '/SimCode/dynamics/gravityEffector')

import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import macros
import gravityEffector
import spice_interface
import pyswice
import stateArchitecture
from gravCoeffOps import loadGravFromFileToList
import orbitalMotion as om
import simMessages

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
                    print l,", ", m
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
    # [testResults, testMessage] = independentSphericalHarmonics(show_plots) - SJKC
    # assert testResults < 1, testMessage
    # [testResults, testMessage] = sphericalHarmonics(show_plots)
    # assert testResults < 1, testMessage
    # [testResults, testMessage] = singleGravityBody(show_plots)
    # assert testResults < 1, testMessage
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
        print "PASSED: " + testCase
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
    snippetContent = '\\textcolor{' + colorText + '}{' + passFailText + '}'  # write formatted LATEX string to file to be used by auto-documentation.
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

    gravOutMax = spherHarm.computeField([[0.0], [0.0], [(6378.1363)*1.0E3]], 100, True)
    if gravOutMax != gravOut:
        testFailCount += 1
        testMessages.append("Gravity ceiling not enforced correctly")

    if testFailCount == 0:
        passFailText = 'PASSED'
        print "PASSED: " + " Spherical Harmonics"
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
    snippetContent = '\\textcolor{' + colorText + '}{' + passFailText + '}' #write formatted LATEX string to file to be used by auto-documentation.
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
    TotalSim.TotalSim.terminateSimulation()

    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))

    # Initialize the modules that we are using.
    SpiceObject = spice_interface.SpiceInterface()

    SpiceObject.ModelTag = "SpiceInterfaceData"
    SpiceObject.SPICEDataPath = splitPath[0] + '/External/EphemerisData/'
    SpiceObject.OutputBufferCount = 10000
    SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun"])
    SpiceObject.UTCCalInit = DateSpice
    TotalSim.AddModelToTask(unitTaskName, SpiceObject)
    SpiceObject.UTCCalInit = "1994 JAN 26 00:02:00.184"
    

    gravBody1 = gravityEffector.GravBodyData()
    gravBody1.bodyInMsgName = "earth_planet_data"
    gravBody1.outputMsgName = "earth_display_frame_data"
    gravBody1.isCentralBody = False
    gravBody1.useSphericalHarmParams = True
    gravityEffector.loadGravFromFile(path + '/GGM03S.txt', gravBody1.spherHarm, 60)
    
    pyswice.furnsh_c(splitPath[0] + '/External/EphemerisData/de430.bsp')
    pyswice.furnsh_c(splitPath[0] + '/External/EphemerisData/naif0011.tls')
    pyswice.furnsh_c(splitPath[0] + '/External/EphemerisData/de-403-masses.tpc')
    pyswice.furnsh_c(splitPath[0] + '/External/EphemerisData/pck00010.tpc')
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
    SpiceObject.UpdateState(0)
    for i in range(2*3600):
        stateOut = pyswice.spkRead('HUBBLE SPACE TELESCOPE', stringCurrent, 'J2000', 'EARTH')
        etPrev =etCurr - 2.0
        stringPrev = pyswice.et2utc_c(etPrev, 'C', 4, 1024, "Yo")
        statePrev = pyswice.spkRead('HUBBLE SPACE TELESCOPE', stringPrev, 'J2000', 'EARTH')
        etNext =etCurr + 2.0
        stringNext = pyswice.et2utc_c(etNext, 'C', 4, 1024, "Yo")
        stateNext = pyswice.spkRead('HUBBLE SPACE TELESCOPE', stringNext, 'J2000', 'EARTH')
        gravVec = (stateNext[3:6] - statePrev[3:6])/(etNext - etPrev)
        normVec.append(np.linalg.norm(stateOut[0:3]))
        
        stateOut*=1000.0
        SpiceObject.J2000Current = etCurr;SpiceObject.UpdateState(0)
        gravBody1.loadEphemeris(0)
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
    
    pyswice.unload_c(splitPath[0] + '/External/EphemerisData/de430.bsp')
    pyswice.unload_c(splitPath[0] + '/External/EphemerisData/naif0011.tls')
    pyswice.unload_c(splitPath[0] + '/External/EphemerisData/de-403-masses.tpc')
    pyswice.unload_c(splitPath[0] + '/External/EphemerisData/pck00010.tpc')
    pyswice.unload_c(path + '/hst_edited.bsp')


    if testFailCount == 0:
        passFailText = 'PASSED'
        print "PASSED: " + "Single-body with Spherical Harmonics"
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
    snippetContent = '\\textcolor{' + colorText + '}{' + passFailText + '}' #write formatted LATEX string to file to be used by auto-documentation.
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.


    # return fail count and join into a single string all messages in the list
    # testMessage
    
    return [testFailCount, ''.join(testMessages)]

def multiBodyGravity(show_plots):
    testCase = 'multiBody'
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
    TotalSim = SimulationBaseClass.SimBaseClass()
    TotalSim.TotalSim.terminateSimulation()
    #
    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(1000.0)))

    localPlanetEditor = simMessages.SpicePlanetStateSimMsg()
    localPlanetEditor.PositionVector = [om.AU, 0., 0.]
    localPlanetEditor.VelocityVector = [0., 0., 0.]

    #Grav Body 1 is twice the size of the other two
    TotalSim.gravBody1 = gravityEffector.GravBodyData()
    TotalSim.gravBody1.bodyInMsgName = "gravBody1_planet_data"
    TotalSim.gravBody1.outputMsgName = "gravBody1_display_frame_data"
    TotalSim.gravBody1.mu = 1000000.
    TotalSim.gravBody1.radEquator = 6500.
    TotalSim.gravBody1.isCentralBody = False
    TotalSim.gravBody1.useSphericalHarmParams = False
    TotalSim.gravBody1.localPlanet = localPlanetEditor

    localPlanetEditor.PositionVector = [-om.AU, 0., 0.]
    localPlanetEditor.VelocityVector = [0., 0., 0.]

    #grav Body 2 and 3 are coincident with each other, half the mass of gravBody1 and are in the opposite direction of gravBody1
    TotalSim.gravBody2 = gravityEffector.GravBodyData()
    TotalSim.gravBody2.bodyInMsgName = "gravBody2_planet_data"
    TotalSim.gravBody2.outputMsgName = "gravBody2_display_frame_data"
    TotalSim.gravBody2.mu = 500000.
    TotalSim.gravBody2.radEquator = 6500.
    TotalSim.gravBody2.isCentralBody = False
    TotalSim.gravBody2.useSphericalHarmParams = False
    TotalSim.gravBody2.localPlanet = localPlanetEditor

    # grav Body 2 and 3 are coincident with each other, half the mass of gravBody1 and are in the opposite direction of gravBody1
    TotalSim.gravBody3 = gravityEffector.GravBodyData()
    TotalSim.gravBody3.bodyInMsgName = "gravBody3_planet_data"
    TotalSim.gravBody3.outputMsgName = "gravBody3_display_frame_data"
    TotalSim.gravBody3.mu = 500000.
    TotalSim.gravBody3.radEquator = 6500.
    TotalSim.gravBody3.isCentralBody = False
    TotalSim.gravBody3.useSphericalHarmParams = False
    TotalSim.gravBody3.localPlanet = localPlanetEditor

    #Create dynParamManager to feed fake spacecraft data to so that the gravityEffector can access it.
    TotalSim.newManager = stateArchitecture.DynParamManager()
    positionName = "hubPosition"
    stateDim = [3, 1]
    posState = TotalSim.newManager.registerState(stateDim[0], stateDim[1], positionName)
    posVelSig = [[0.],[0.],[0.]]
    posState.setState(posVelSig)
    # TotalSim.newManager.setPropertyValue("systemTime", [[i * 1000 * 1e9], [i * 1000.0]])
    velocityName = "hubVelocity"
    stateDim = [3, 1]
    velState = TotalSim.newManager.registerState(stateDim[0], stateDim[1], velocityName)
    velState.setState(posVelSig)
    sigmaName = "hubSigma"
    stateDim = [3, 1]
    sigmaState = TotalSim.newManager.registerState(stateDim[0], stateDim[1], sigmaName)
    sigmaState.setState(posVelSig)
    initC_B = [[0.0], [0.0], [0.0]]
    TotalSim.newManager.createProperty("centerOfMassSC", initC_B)
    TotalSim.newManager.createProperty("systemTime", [[0], [0.0]])
    #
    #

    #This is the gravityEffector which will actually compute the gravitational acceleration
    allGrav = gravityEffector.GravityEffector()
    allGrav.gravBodies = gravityEffector.GravBodyVector([TotalSim.gravBody1, TotalSim.gravBody2, TotalSim.gravBody3])#, TotalSim.sunGravBody, TotalSim.marsGravBody])
    allGrav.linkInStates(TotalSim.newManager)
    allGrav.registerProperties(TotalSim.newManager)
    TotalSim.AddModelToTask(unitTaskName, allGrav)


    TotalSim.InitializeSimulationAndDiscover()

    allGrav.computeGravityField()
    print TotalSim.newManager.getPropertyReference("g_N")
    #
    # TotalSim.gravBody1.InertialPosition
    #
    #
    # # pyswice.furnsh_c(splitPath[0] + '/External/EphemerisData/de430.bsp')
    # # pyswice.furnsh_c(splitPath[0] + '/External/EphemerisData/naif0011.tls')
    # # pyswice.furnsh_c(splitPath[0] + '/External/EphemerisData/de-403-masses.tpc')
    # # pyswice.furnsh_c(splitPath[0] + '/External/EphemerisData/pck00010.tpc')
    # # pyswice.furnsh_c(path + '/dawn_rec_081022_081109_090218_v1.bsp')
    # #
    # # SpiceObject.UTCCalInit = "2008-10-24T00:00:00"
    # TotalSim.TotalSim.logThisMessage(TotalSim.gravBody1.outputMsgName, macros.sec2nano(1000.0))
    # TotalSim.InitializeSimulationAndDiscover()
    # TotalSim.ConfigureStopTime(macros.sec2nano(10000.0))
    # TotalSim.ExecuteSimulation()
    # earthPos = TotalSim.pullMessageLogData(TotalSim.gravBody1.outputMsgName+".PositionVector",range(3))
    # print earthPos
    # gravErrVec = []
    #
    #
    #
    # accuracy = 5.0e-4
    #
    # for gravMeas in gravErrVec:
    #     if gravMeas > accuracy:
    #         testFailCount += 1
    #         testMessages.append("Gravity multi-body error too high for kernel comparison")
    #         break
    # snippetName = testCase + 'Accuracy'
    # snippetContent = '{:1.1e}'.format(accuracy)  # write formatted LATEX string to file to be used by auto-documentation.
    # unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.
    #
    # if testFailCount == 0:
    #     passFailText = 'PASSED'
    #     print "PASSED: " + "Multi-body with Spherical Harmonics"
    #     colorText = 'ForestGreen'  # color to write auto-documented "PASSED" message in in LATEX
    #     snippetName = testCase + 'FailMsg'
    #     snippetContent = ""
    #     unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    # else:
    #     passFailText = 'FAILED'
    #     colorText = 'Red'  # color to write auto-documented "FAILED" message in in LATEX
    #     snippetName = testCase + 'FailMsg'
    #     snippetContent = passFailText
    #     for message in testMessages:
    #         snippetContent += ". " + message
    #     snippetContent += "."
    #     unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)  # write formatted LATEX string to file to be used by auto-documentation.
    # snippetName = testCase + 'PassFail'  # name of file to be written for auto-documentation which specifies if this test was passed or failed.
    # snippetContent = '\\textcolor{' + colorText + '}{' + passFailText + '}' #write formatted LATEX string to file to be used by auto-documentation.
    # unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path) #write formatted LATEX string to file to be used by auto-documentation.


    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    test_gravityEffectorAllTest(False)
    
