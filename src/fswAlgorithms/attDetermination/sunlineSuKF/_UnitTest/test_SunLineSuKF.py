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
import sys, os, inspect
import numpy
import pytest
import math

from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport
from Basilisk.simulation import coarse_sun_sensor
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import sunlineSuKF, cssComm, fswMessages  # import the module that is to be tested

import SunLineSuKF_test_utilities as FilterPlots

def setupFilterData(filterObject, initialized):
    filterObject.navStateOutMsgName = "sunline_state_estimate"
    filterObject.filtDataOutMsgName = "sunline_filter_data"
    filterObject.cssDataInMsgName = "css_sensors_data"
    filterObject.cssConfigInMsgName = "css_config_data"

    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0

    if initialized:
        filterObject.stateInit = [0.0, 0.0, 1.0, 0.0, 0.0, 1.]
        filterObject.filterInitialized = 1
    else:
        filterObject.filterInitialized = 0

    filterObject.covarInit = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 1., 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 1., 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 1E-4]
    qNoiseIn = numpy.identity(6)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3]*0.001*0.001
    qNoiseIn[3:5, 3:5] = qNoiseIn[3:5, 3:5]*0.001*0.001
    qNoiseIn[5, 5] = qNoiseIn[5, 5]*0.0000002*0.0000002
    filterObject.qNoise = qNoiseIn.reshape(36).tolist()
    filterObject.qObsVal = 0.002
    filterObject.sensorUseThresh = 0.0


def test_functions_ukf(show_plots):
    [testResults, testMessage] = sunline_utilities_test(show_plots)
    assert testResults < 1, testMessage

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
@pytest.mark.parametrize("kellyOn", [
    (False),
    (True)
])


def test_all_sunline_kf(show_plots, kellyOn):
    [testResults, testMessage] = StatePropSunLine(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = StateUpdateSunLine(show_plots, kellyOn)
    assert testResults < 1, testMessage

def sunline_utilities_test(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Initialize the test module configuration data
    AMatrix = [0.488894, 0.888396, 0.325191, 0.319207,
                1.03469, -1.14707, -0.754928, 0.312859, 
                0.726885, -1.06887, 1.3703, -0.86488,
               -0.303441, -0.809499, -1.71152, -0.0300513,
                0.293871, -2.94428, -0.102242, -0.164879,
               -0.787283, 1.43838, -0.241447, 0.627707]
   
    RVector = sunlineSuKF.new_doubleArray(len(AMatrix))
    AVector = sunlineSuKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        sunlineSuKF.doubleArray_setitem(AVector, i, AMatrix[i])
        sunlineSuKF.doubleArray_setitem(RVector, i, 0.0)

        sunlineSuKF.ukfQRDJustR(AVector, 6, 4, RVector)
    RMatrix = []
    for i in range(4*4):
        RMatrix.append(sunlineSuKF.doubleArray_getitem(RVector, i))
    RBaseNumpy = numpy.array(RMatrix).reshape(4,4)
    AMatNumpy = numpy.array(AMatrix).reshape(6,4)
    q,r = numpy.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i,i] < 0.0:
            r[i,:] *= -1.0
    if numpy.linalg.norm(r - RBaseNumpy) > 1.0E-15:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")
    
    AMatrix = [1.09327, 1.10927, -0.863653, 1.32288,
     -1.21412, -1.1135, -0.00684933, -2.43508,
     -0.769666, 0.371379, -0.225584, -1.76492,
     -1.08906, 0.0325575, 0.552527, -1.6256,
     1.54421, 0.0859311, -1.49159, 1.59683]

    RVector = sunlineSuKF.new_doubleArray(len(AMatrix))
    AVector = sunlineSuKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        sunlineSuKF.doubleArray_setitem(AVector, i, AMatrix[i])
        sunlineSuKF.doubleArray_setitem(RVector, i, 0.0)

    sunlineSuKF.ukfQRDJustR(AVector, 5, 4, RVector)
    RMatrix = []
    for i in range(4*4):
        RMatrix.append(sunlineSuKF.doubleArray_getitem(RVector, i))
    RBaseNumpy = numpy.array(RMatrix).reshape(4,4)
    AMatNumpy = numpy.array(AMatrix).reshape(5,4)
    q,r = numpy.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i,i] < 0.0:
            r[i,:] *= -1.0
    if numpy.linalg.norm(r - RBaseNumpy) > 1.0E-14:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")
    
    AMatrix = [ 0.2236,         0,
               0,    0.2236,
               -0.2236,         0,
               0,   -0.2236,
               0.0170,         0,
               0,    0.0170]

    RVector = sunlineSuKF.new_doubleArray(len(AMatrix))
    AVector = sunlineSuKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        sunlineSuKF.doubleArray_setitem(AVector, i, AMatrix[i])
        sunlineSuKF.doubleArray_setitem(RVector, i, 0.0)

    sunlineSuKF.ukfQRDJustR(AVector, 6, 2, RVector)
    RMatrix = []
    for i in range(2*2):
        RMatrix.append(sunlineSuKF.doubleArray_getitem(RVector, i))
    RBaseNumpy = numpy.array(RMatrix).reshape(2,2)
    AMatNumpy = numpy.array(AMatrix).reshape(6,2)
    q,r = numpy.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i,i] < 0.0:
            r[i,:] *= -1.0

    if numpy.linalg.norm(r - RBaseNumpy) > 1.0E-15:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")


    LUSourceMat = [8,1,6,3,5,7,4,9,2]
    LUSVector = sunlineSuKF.new_doubleArray(len(LUSourceMat))
    LVector = sunlineSuKF.new_doubleArray(len(LUSourceMat))
    UVector = sunlineSuKF.new_doubleArray(len(LUSourceMat))
    intSwapVector = sunlineSuKF.new_intArray(3)
    
    for i in range(len(LUSourceMat)):
        sunlineSuKF.doubleArray_setitem(LUSVector, i, LUSourceMat[i])
        sunlineSuKF.doubleArray_setitem(UVector, i, 0.0)
        sunlineSuKF.doubleArray_setitem(LVector, i, 0.0)

    exCount = sunlineSuKF.ukfLUD(LUSVector, 3, 3, LVector, intSwapVector)
    #sunlineSuKF.ukfUInv(LUSVector, 3, 3, UVector)
    LMatrix = []
    UMatrix = []
    #UMatrix = []
    for i in range(3):
        currRow = sunlineSuKF.intArray_getitem(intSwapVector, i)
        for j in range(3):
            if(j<i):
                LMatrix.append(sunlineSuKF.doubleArray_getitem(LVector, i*3+j))
                UMatrix.append(0.0)
            elif(j>i):
                LMatrix.append(0.0)
                UMatrix.append(sunlineSuKF.doubleArray_getitem(LVector, i*3+j))
            else:
                LMatrix.append(1.0)
                UMatrix.append(sunlineSuKF.doubleArray_getitem(LVector, i*3+j))
    #    UMatrix.append(sunlineSuKF.doubleArray_getitem(UVector, i))

    LMatrix = numpy.array(LMatrix).reshape(3,3)
    UMatrix = numpy.array(UMatrix).reshape(3,3)
    outMat = numpy.dot(LMatrix, UMatrix)
    outMatSwap = numpy.zeros((3,3)) 
    for i in range(3):
        currRow = sunlineSuKF.intArray_getitem(intSwapVector, i)
        outMatSwap[i,:] = outMat[currRow, :]
        outMat[currRow,:] = outMat[i, :]
    LuSourceArray = numpy.array(LUSourceMat).reshape(3,3)

    if(numpy.linalg.norm(outMatSwap - LuSourceArray) > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Decomposition accuracy failure")

    EqnSourceMat = [2.0, 1.0, 3.0, 2.0, 6.0, 8.0, 6.0, 8.0, 18.0]
    BVector = [1.0, 3.0, 5.0]
    EqnVector = sunlineSuKF.new_doubleArray(len(EqnSourceMat))
    EqnBVector = sunlineSuKF.new_doubleArray(len(LUSourceMat)/3)
    EqnOutVector = sunlineSuKF.new_doubleArray(len(LUSourceMat)/3)

    for i in range(len(EqnSourceMat)):
        sunlineSuKF.doubleArray_setitem(EqnVector, i, EqnSourceMat[i])
        sunlineSuKF.doubleArray_setitem(EqnBVector, i/3, BVector[i/3])
        sunlineSuKF.intArray_setitem(intSwapVector, i/3, 0)
        sunlineSuKF.doubleArray_setitem(LVector, i, 0.0)
    
    exCount = sunlineSuKF.ukfLUD(EqnVector, 3, 3, LVector, intSwapVector)
    
    sunlineSuKF.ukfLUBckSlv(LVector, 3, 3, intSwapVector, EqnBVector, EqnOutVector)
    
    expectedSol = [3.0/10.0, 4.0/10.0, 0.0]
    errorVal = 0.0
    for i in range(3):
        errorVal += abs(sunlineSuKF.doubleArray_getitem(EqnOutVector, i) -expectedSol[i])

    if(errorVal > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Back-Solve accuracy failure")


    InvSourceMat = [8,1,6,3,5,7,4,9,2]
    SourceVector = sunlineSuKF.new_doubleArray(len(InvSourceMat))
    InvVector = sunlineSuKF.new_doubleArray(len(InvSourceMat))
    for i in range(len(InvSourceMat)):
        sunlineSuKF.doubleArray_setitem(SourceVector, i, InvSourceMat[i])
        sunlineSuKF.doubleArray_setitem(InvVector, i, 0.0)
    nRow = int(math.sqrt(len(InvSourceMat)))
    sunlineSuKF.ukfMatInv(SourceVector, nRow, nRow, InvVector)

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(sunlineSuKF.doubleArray_getitem(InvVector, i))

    InvOut = numpy.array(InvOut).reshape(nRow, nRow)
    expectIdent = numpy.dot(InvOut, numpy.array(InvSourceMat).reshape(3,3))
    errorNorm = numpy.linalg.norm(expectIdent - numpy.identity(3))
    if(errorNorm > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Matrix Inverse accuracy failure")

    
    cholTestMat = [1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0]
    SourceVector = sunlineSuKF.new_doubleArray(len(cholTestMat))
    CholVector = sunlineSuKF.new_doubleArray(len(cholTestMat))
    for i in range(len(cholTestMat)):
        sunlineSuKF.doubleArray_setitem(SourceVector, i, cholTestMat[i])
        sunlineSuKF.doubleArray_setitem(CholVector, i, 0.0)
    nRow = int(math.sqrt(len(cholTestMat)))
    sunlineSuKF.ukfCholDecomp(SourceVector, nRow, nRow, CholVector)
    cholOut = []
    for i in range(len(cholTestMat)):
        cholOut.append(sunlineSuKF.doubleArray_getitem(CholVector, i))

    cholOut = numpy.array(cholOut).reshape(nRow, nRow)
    cholComp = numpy.linalg.cholesky(numpy.array(cholTestMat).reshape(nRow, nRow))
    errorNorm = numpy.linalg.norm(cholOut - cholComp)
    if(errorNorm > 1.0E-14):
        testFailCount += 1
        testMessages.append("Cholesky Matrix Decomposition accuracy failure")


    InvSourceMat = [2.1950926119414667, 0.0, 0.0, 0.0,
               1.0974804773131115, 1.9010439702743847, 0.0, 0.0,
               0.0, 1.2672359635912551, 1.7923572711881284, 0.0,
               1.0974804773131113, -0.63357997864171967, 1.7920348101787789, 0.033997451205364251]
               
    SourceVector = sunlineSuKF.new_doubleArray(len(InvSourceMat))
    InvVector = sunlineSuKF.new_doubleArray(len(InvSourceMat))
    for i in range(len(InvSourceMat)):
        sunlineSuKF.doubleArray_setitem(SourceVector, i, InvSourceMat[i])
        sunlineSuKF.doubleArray_setitem(InvVector, i, 0.0)
    nRow = int(math.sqrt(len(InvSourceMat)))
    sunlineSuKF.ukfLInv(SourceVector, nRow, nRow, InvVector)

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(sunlineSuKF.doubleArray_getitem(InvVector, i))

    InvOut = numpy.array(InvOut).reshape(nRow, nRow)
    expectIdent = numpy.dot(InvOut, numpy.array(InvSourceMat).reshape(nRow,nRow))
    errorNorm = numpy.linalg.norm(expectIdent - numpy.identity(nRow))
    if(errorNorm > 1.0E-12):
        print errorNorm
        testFailCount += 1
        testMessages.append("L Matrix Inverse accuracy failure")

    InvSourceMat = numpy.transpose(numpy.array(InvSourceMat).reshape(nRow, nRow)).reshape(nRow*nRow).tolist()
    SourceVector = sunlineSuKF.new_doubleArray(len(InvSourceMat))
    InvVector = sunlineSuKF.new_doubleArray(len(InvSourceMat))
    for i in range(len(InvSourceMat)):
        sunlineSuKF.doubleArray_setitem(SourceVector, i, InvSourceMat[i])
        sunlineSuKF.doubleArray_setitem(InvVector, i, 0.0)
    nRow = int(math.sqrt(len(InvSourceMat)))
    sunlineSuKF.ukfUInv(SourceVector, nRow, nRow, InvVector)

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(sunlineSuKF.doubleArray_getitem(InvVector, i))

    InvOut = numpy.array(InvOut).reshape(nRow, nRow)
    expectIdent = numpy.dot(InvOut, numpy.array(InvSourceMat).reshape(nRow,nRow))
    errorNorm = numpy.linalg.norm(expectIdent - numpy.identity(nRow))
    if(errorNorm > 1.0E-12):
        print errorNorm
        testFailCount += 1
        testMessages.append("U Matrix Inverse accuracy failure")


    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + " UKF utilities"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def StateUpdateSunLine(show_plots, kellyOn):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True
    
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = sunlineSuKF.SunlineSuKFConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "sunlineSuKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig, False)
    cssConstelation = fswMessages.CSSConfigFswMsg()
    
    CSSOrientationList = [
       [0.70710678118654746, -0.5, 0.5],
       [0.70710678118654746, -0.5, -0.5],
       [0.70710678118654746, 0.5, -0.5],
       [0.70710678118654746, 0.5, 0.5],
       [-0.70710678118654746, 0, 0.70710678118654757],
       [-0.70710678118654746, 0.70710678118654757, 0.0],
       [-0.70710678118654746, 0, -0.70710678118654757],
       [-0.70710678118654746, -0.70710678118654757, 0.0],
    ]
    totalCSSList = []
    for CSSHat in CSSOrientationList:
        newCSS = fswMessages.CSSUnitConfigFswMsg()
        newCSS.CBias = 1.0
        newCSS.nHat_B = CSSHat
        totalCSSList.append(newCSS)
    cssConstelation.nCSS = len(CSSOrientationList)
    cssConstelation.cssVals = totalCSSList
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.cssConfigInMsgName,
                               cssConstelation)
    unitTestSim.TotalSim.logThisMessage('sunline_filter_data', testProcessRate)

    # Add the kelly curve coefficients
    if kellyOn:
        kellList = []
        for j in range(len(CSSOrientationList)):
            kellyData = sunlineSuKF.SunlineSuKFCFit()
            kellyData.cssKellFact = 0.05
            kellyData.cssKellPow = 2.
            kellyData.cssRelScale = 1.
            kellList.append(kellyData)
        moduleConfig.kellFits = kellList

    testVector = numpy.array([-0.7, 0.7, 0.0])
    testVector/=numpy.linalg.norm(testVector)
    inputData = cssComm.CSSArraySensorIntMsg()
    dotList = []
    for element in CSSOrientationList:
        dotProd = numpy.dot(numpy.array(element), testVector)/(numpy.linalg.norm(element)*numpy.linalg.norm(testVector))
        dotList.append(dotProd)

    inputData.CosValue = dotList
    inputMessageSize = inputData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                      moduleConfig.cssDataInMsgName,
                                      inputMessageSize,
                                      2)  # number of buffers (leave at 2 as default, don't make zero)

    stateTarget = testVector.tolist()
    stateTarget.extend([0.0, 0.0, 1.])
    # moduleConfig.stateInit = [0.7, 0.7, 0.0, 0.01, 0.001, 1.]

    numStates = len(moduleConfig.stateInit)
    unitTestSim.InitializeSimulation()
    if kellyOn:
        time = 1000
    else:
        time =  500
    for i in range(time):
        unitTestSim.TotalSim.WriteMessageData(moduleConfig.cssDataInMsgName,
                                  inputMessageSize,
                                  unitTestSim.TotalSim.CurrentNanos,
                                  inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+1)*0.5))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".state", range(numStates))
    postFitLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".postFitRes", range(8))
    covarLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".covar", range(numStates*numStates))

    accuracy = 1.0E-3
    if kellyOn:
        accuracy = 1.0E-2 # 1% Error test for the kelly curves given errors
    for i in range(numStates):
        if(covarLog[-1, i*numStates+1+i] > covarLog[0, i*numStates+1+i]):
            testFailCount += 1
            testMessages.append("Covariance update failure first part")
    if(numpy.arccos(numpy.dot(stateLog[-1, 1:4], stateTarget[0:3])/(numpy.linalg.norm(stateLog[-1, 1:4])*numpy.linalg.norm(stateTarget[0:3]))) > accuracy):
        print numpy.arccos(numpy.dot(stateLog[-1, 1:4], stateTarget[0:3])/(numpy.linalg.norm(stateLog[-1, 1:4])*numpy.linalg.norm(stateTarget[0:3])))
        testFailCount += 1
        testMessages.append("Pointing update failure")
    if(numpy.linalg.norm(stateLog[-1, 4:7] - stateTarget[3:6]) > accuracy):
        print numpy.linalg.norm(stateLog[-1,  4:7] - stateTarget[3:6])
        testFailCount += 1
        testMessages.append("Rate update failure")
    if(abs(stateLog[-1, 6] - stateTarget[5]) > accuracy):
        print abs(stateLog[-1, 6] - stateTarget[5])
        testFailCount += 1
        testMessages.append("Sun Intensity update failure")

    testVector = numpy.array([-0.7, 0.75, 0.0])
    testVector /= numpy.linalg.norm(testVector)
    inputData = cssComm.CSSArraySensorIntMsg()
    dotList = []
    for element in CSSOrientationList:
        dotProd = numpy.dot(numpy.array(element), testVector)
        dotList.append(dotProd)
    inputData.CosValue = dotList
        
    for i in range(time):
        if i > 20:
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.cssDataInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+time+1)*0.5))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".state", range(numStates))
    postFitLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".postFitRes", range(8))
    covarLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".covar", range(numStates*numStates))

    stateTarget = testVector.tolist()
    stateTarget.extend([0.0, 0.0, 1.0])

    for i in range(numStates):
        if(covarLog[-1, i*numStates+1+i] > covarLog[0, i*numStates+1+i]):
            print covarLog[-1, i*numStates+1+i] - covarLog[0, i*numStates+1+i]
            testFailCount += 1
            testMessages.append("Covariance update failure")
    if(numpy.arccos(numpy.dot(stateLog[-1, 1:4], stateTarget[0:3])/(numpy.linalg.norm(stateLog[-1, 1:4])*numpy.linalg.norm(stateTarget[0:3]))) > accuracy):
        print numpy.arccos(numpy.dot(stateLog[-1, 1:4], stateTarget[0:3])/(numpy.linalg.norm(stateLog[-1, 1:4])*numpy.linalg.norm(stateTarget[0:3])))
        testFailCount += 1
        testMessages.append("Pointing update failure")
    if(numpy.linalg.norm(stateLog[-1, 4:7] - stateTarget[3:6]) > accuracy):
        print numpy.linalg.norm(stateLog[-1,  4:7] - stateTarget[3:6])
        testFailCount += 1
        testMessages.append("Rate update failure")
    if(abs(stateLog[-1, 6] - stateTarget[5]) > accuracy):
        print abs(stateLog[-1, 6] - stateTarget[5])
        testFailCount += 1
        testMessages.append("Sun Intensity update failure")

    FilterPlots.StateCovarPlot(stateLog, covarLog, show_plots)
    FilterPlots.PostFitResiduals(postFitLog, moduleConfig.qObsVal, show_plots)

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state update"
    else:
        print testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def StatePropSunLine(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True
    
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = sunlineSuKF.SunlineSuKFConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "sunlineSuKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig, True)
    numStates = 6
    unitTestSim.TotalSim.logThisMessage('sunline_filter_data', testProcessRate)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(8000.0))
    unitTestSim.ExecuteSimulation()
    
    stateLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".state", range(numStates))
    postFitLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".postFitRes", range(8))
    covarLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".covar", range(numStates*numStates))

    FilterPlots.StateCovarPlot(stateLog, covarLog, show_plots)
    FilterPlots.PostFitResiduals(postFitLog, moduleConfig.qObsVal, show_plots)

    for i in range(numStates):
        if(abs(stateLog[-1, i+1] - stateLog[0, i+1]) > 1.0E-10):
            print abs(stateLog[-1, i+1] - stateLog[0, i+1])
            testFailCount += 1
            testMessages.append("State propagation failure")

    

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    # test_all_sunline_kf(True)
    StateUpdateSunLine(True, True)
