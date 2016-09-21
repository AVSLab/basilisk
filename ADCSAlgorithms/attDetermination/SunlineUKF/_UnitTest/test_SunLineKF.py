'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy
import pytest
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import alg_contain
import unitTestSupport  # general support file with common unit test functions
import sunlineUKF  # import the module that is to be tested
import cssComm
import vehicleConfigData
import macros
import sim_model
import ctypes


def setupFilterData(filterObject):
    filterObject.outputNavStateName = "sunline_state_estimate"
    filterObject.inputCSSDataName = "css_sensors_data"
    filterObject.inputPropsName = "adcs_config_data"
    filterObject.inputCSSConfigName = "css_config_data"

    filterObject.alpha = 0.2
    filterObject.beta = 0.01
    filterObject.kappa = 0.0

    filterObject.state = [0.1, 0.0]
    filterObject.covar = [0.4, 0.0, 0.0, 0.4]
    filterObject.qNoise = [0.0017*0.0017, 0.0, 0.0, 0.0017*0.0017]
    filterObject.qObsVal = 0.017*0.017

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_all_sunline_kf(show_plots):
    [testResults, testMessage] = sunline_utilities_test(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = testStatePropSunLine(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = testStateUpdateSunLine(show_plots)
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
   
    RVector = sunlineUKF.new_doubleArray(len(AMatrix))
    AVector = sunlineUKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        sunlineUKF.doubleArray_setitem(AVector, i, AMatrix[i])
        sunlineUKF.doubleArray_setitem(RVector, i, 0.0)

    sunlineUKF.ukfQRDJustR(AVector, 6, 4, RVector)
    RMatrix = []
    for i in range(4*4):
        RMatrix.append(sunlineUKF.doubleArray_getitem(RVector, i))
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

    RVector = sunlineUKF.new_doubleArray(len(AMatrix))
    AVector = sunlineUKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        sunlineUKF.doubleArray_setitem(AVector, i, AMatrix[i])
        sunlineUKF.doubleArray_setitem(RVector, i, 0.0)

    sunlineUKF.ukfQRDJustR(AVector, 5, 4, RVector)
    RMatrix = []
    for i in range(4*4):
        RMatrix.append(sunlineUKF.doubleArray_getitem(RVector, i))
    RBaseNumpy = numpy.array(RMatrix).reshape(4,4)
    AMatNumpy = numpy.array(AMatrix).reshape(5,4)
    q,r = numpy.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i,i] < 0.0:
            r[i,:] *= -1.0
    if numpy.linalg.norm(r - RBaseNumpy) > 1.0E-15:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")
    
    AMatrix = [ 0.2236,         0,
               0,    0.2236,
               -0.2236,         0,
               0,   -0.2236,
               0.0170,         0,
               0,    0.0170]

    RVector = sunlineUKF.new_doubleArray(len(AMatrix))
    AVector = sunlineUKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        sunlineUKF.doubleArray_setitem(AVector, i, AMatrix[i])
        sunlineUKF.doubleArray_setitem(RVector, i, 0.0)

    sunlineUKF.ukfQRDJustR(AVector, 6, 2, RVector)
    RMatrix = []
    for i in range(2*2):
        RMatrix.append(sunlineUKF.doubleArray_getitem(RVector, i))
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
    LUSVector = sunlineUKF.new_doubleArray(len(LUSourceMat))
    LVector = sunlineUKF.new_doubleArray(len(LUSourceMat))
    UVector = sunlineUKF.new_doubleArray(len(LUSourceMat))
    intSwapVector = sunlineUKF.new_intArray(3)
    
    for i in range(len(LUSourceMat)):
        sunlineUKF.doubleArray_setitem(LUSVector, i, LUSourceMat[i])
        sunlineUKF.doubleArray_setitem(UVector, i, 0.0)
        sunlineUKF.doubleArray_setitem(LVector, i, 0.0)

    exCount = sunlineUKF.ukfLUD(LUSVector, 3, 3, LVector, intSwapVector)
    #sunlineUKF.ukfUInv(LUSVector, 3, 3, UVector)
    LMatrix = []
    UMatrix = []
    #UMatrix = []
    for i in range(3):
        currRow = sunlineUKF.intArray_getitem(intSwapVector, i)
        for j in range(3):
            if(j<i):
                LMatrix.append(sunlineUKF.doubleArray_getitem(LVector, i*3+j))
                UMatrix.append(0.0)
            elif(j>i):
                LMatrix.append(0.0)
                UMatrix.append(sunlineUKF.doubleArray_getitem(LVector, i*3+j))
            else:
                LMatrix.append(1.0)
                UMatrix.append(sunlineUKF.doubleArray_getitem(LVector, i*3+j))
    #    UMatrix.append(sunlineUKF.doubleArray_getitem(UVector, i))

    LMatrix = numpy.array(LMatrix).reshape(3,3)
    UMatrix = numpy.array(UMatrix).reshape(3,3)
    outMat = numpy.dot(LMatrix, UMatrix)
    outMatSwap = numpy.zeros((3,3)) 
    for i in range(3):
        currRow = sunlineUKF.intArray_getitem(intSwapVector, i)
        outMatSwap[i,:] = outMat[currRow, :]
        outMat[currRow,:] = outMat[i, :]
    LuSourceArray = numpy.array(LUSourceMat).reshape(3,3)

    if(numpy.linalg.norm(outMatSwap - LuSourceArray) > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Decomposition accuracy failure")

    EqnSourceMat = [2.0, 1.0, 3.0, 2.0, 6.0, 8.0, 6.0, 8.0, 18.0]
    BVector = [1.0, 3.0, 5.0]
    EqnVector = sunlineUKF.new_doubleArray(len(EqnSourceMat))
    EqnBVector = sunlineUKF.new_doubleArray(len(LUSourceMat)/3)
    EqnOutVector = sunlineUKF.new_doubleArray(len(LUSourceMat)/3)

    for i in range(len(EqnSourceMat)):
        sunlineUKF.doubleArray_setitem(EqnVector, i, EqnSourceMat[i])
        sunlineUKF.doubleArray_setitem(EqnBVector, i/3, BVector[i/3])
        sunlineUKF.intArray_setitem(intSwapVector, i/3, 0)
        sunlineUKF.doubleArray_setitem(LVector, i, 0.0)
    
    exCount = sunlineUKF.ukfLUD(EqnVector, 3, 3, LVector, intSwapVector)
    
    sunlineUKF.ukfLUBckSlv(LVector, 3, 3, intSwapVector, EqnBVector, EqnOutVector)
    
    expectedSol = [3.0/10.0, 4.0/10.0, 0.0]
    errorVal = 0.0
    for i in range(3):
        errorVal += abs(sunlineUKF.doubleArray_getitem(EqnOutVector, i) -expectedSol[i])

    if(errorVal > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Back-Solve accuracy failure")


    InvSourceMat = [8,1,6,3,5,7,4,9,2]
    SourceVector = sunlineUKF.new_doubleArray(len(InvSourceMat))
    InvVector = sunlineUKF.new_doubleArray(len(InvSourceMat))
    for i in range(len(InvSourceMat)):
        sunlineUKF.doubleArray_setitem(SourceVector, i, InvSourceMat[i])
        sunlineUKF.doubleArray_setitem(InvVector, i, 0.0)
    nRow = int(math.sqrt(len(InvSourceMat)))
    sunlineUKF.ukfMatInv(SourceVector, nRow, nRow, InvVector)

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(sunlineUKF.doubleArray_getitem(InvVector, i))

    InvOut = numpy.array(InvOut).reshape(nRow, nRow)
    expectIdent = numpy.dot(InvOut, numpy.array(InvSourceMat).reshape(3,3))
    errorNorm = numpy.linalg.norm(expectIdent - numpy.identity(3))
    if(errorNorm > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Matrix Inverse accuracy failure")

    
    cholTestMat = [1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0]
    SourceVector = sunlineUKF.new_doubleArray(len(cholTestMat))
    CholVector = sunlineUKF.new_doubleArray(len(cholTestMat))
    for i in range(len(cholTestMat)):
        sunlineUKF.doubleArray_setitem(SourceVector, i, cholTestMat[i])
        sunlineUKF.doubleArray_setitem(CholVector, i, 0.0)
    nRow = int(math.sqrt(len(cholTestMat)))
    sunlineUKF.ukfCholDecomp(SourceVector, nRow, nRow, CholVector)
    cholOut = []
    for i in range(len(cholTestMat)):
        cholOut.append(sunlineUKF.doubleArray_getitem(CholVector, i))

    cholOut = numpy.array(cholOut).reshape(nRow, nRow)
    cholComp = numpy.linalg.cholesky(numpy.array(cholTestMat).reshape(nRow, nRow))
    errorNorm = numpy.linalg.norm(cholOut - cholComp)
    if(errorNorm > 1.0E-14):
        testFailCount += 1
        testMessages.append("Cholesky Matrix Decomposition accuracy failure")


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

def testStateUpdateSunLine(show_plots):
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
    moduleConfig = sunlineUKF.SunlineUKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        sunlineUKF.Update_sunlineUKF,
                                        sunlineUKF.SelfInit_sunlineUKF,
                                        sunlineUKF.CrossInit_sunlineUKF,
                                        sunlineUKF.Reset_sunlineUKF)
    moduleWrap.ModelTag = "SunlineUKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig)
    
    cssConstelation = vehicleConfigData.CSSConstConfig()
    
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
    i=0
    #Initializing a 2D double array is hard with SWIG.  That's why there is this
    #layer between the above list and the actual C variables.
    for CSSHat in CSSOrientationList:
        newCSS = vehicleConfigData.CSSConfigurationElement()
        newCSS.nHat_S = CSSHat
        totalCSSList.append(newCSS)
    cssConstelation.nCSS = len(CSSOrientationList)
    cssConstelation.cssVals = totalCSSList
    unitTestSim.TotalSim.CreateNewMessage("TestProcess", "css_config_data",
                                    vehicleConfigData.MAX_NUM_CSS_SENSORS*3*8+8, 2, "CSSConstellation")
    unitTestSim.TotalSim.WriteMessageData("css_config_data", vehicleConfigData.MAX_NUM_CSS_SENSORS*3*8+8, 0, cssConstelation)
    
    inputMessageSize = 18 * 8 + 8  # 18 doubles + 1 32bit integer
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputPropsName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
    I = [1000., 0., 0.,
     0., 800., 0.,
     0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    BS = [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0]
    vehicleConfigOut.BS = BS
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputPropsName,
                                                inputMessageSize,
                                                0,
                                                vehicleConfigOut)

    testVector = numpy.array([-1.0, 0.0, 0.0])
    inputData = cssComm.CSSOutputData()
    dotList = []
    for element in CSSOrientationList:
        dotProd = numpy.dot(numpy.array(element), testVector)
        dotList.append(dotProd)
    inputData.CosValue = dotList

    inputMessageSize = vehicleConfigData.MAX_NUM_CSS_SENSORS*8
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                      moduleConfig.inputCSSDataName,
                                      inputMessageSize,
                                      2)  # number of buffers (leave at 2 as default, don't make zero)


    moduleConfig.state = [0.2, 0.0]
    unitTestSim.AddVectorForLogging('SunlineUKF.covar', 'double', 0, 3, testProcessRate)
    unitTestSim.AddVectorForLogging('SunlineUKF.state', 'double', 0, 1, testProcessRate)

    unitTestSim.InitializeSimulation()

    for i in range(20000):
        if i > 20:
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputCSSDataName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+1)*0.5))
        unitTestSim.ExecuteSimulation()

    testVector = numpy.array([-1.0, 0.0, 0.0])
    inputData = cssComm.CSSOutputData()
    dotList = []
    for element in CSSOrientationList:
        dotProd = numpy.dot(numpy.array(element), testVector)
        dotList.append(dotProd)
    inputData.CosValue = dotList
        
    for i in range(20000):
        if i > 20:
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputCSSDataName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+20001)*0.5))
        unitTestSim.ExecuteSimulation()


    covarLog = unitTestSim.GetLogVariableData('SunlineUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineUKF.state')
    plt.figure()
    plt.plot(covarLog[:,0]*1.0E-9, covarLog[:,1])
    plt.show()
    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state update"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]
def testStatePropSunLine(show_plots):

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
    moduleConfig = sunlineUKF.SunlineUKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        sunlineUKF.Update_sunlineUKF,
                                        sunlineUKF.SelfInit_sunlineUKF,
                                        sunlineUKF.CrossInit_sunlineUKF,
                                        sunlineUKF.Reset_sunlineUKF)
    moduleWrap.ModelTag = "SunlineUKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig)
    unitTestSim.AddVectorForLogging('SunlineUKF.covar', 'double', 0, 3, testProcessRate)
    unitTestSim.AddVectorForLogging('SunlineUKF.state', 'double', 0, 1, testProcessRate)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(8000.0))
    unitTestSim.ExecuteSimulation()
    
    covarLog = unitTestSim.GetLogVariableData('SunlineUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineUKF.state')
    for i in range(2):
        if(covarLog[-1, i*2+1+i] <= covarLog[0, i*2+1+i]):
            testFailCount += 1
            testMessages.append("Covariance propagation failure")
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
    test_all_sunline_kf(False)
