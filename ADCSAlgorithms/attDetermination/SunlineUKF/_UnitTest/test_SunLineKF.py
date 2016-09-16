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
import matplotlib.pyplot as plt
import numpy
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import alg_contain
import unitTestSupport  # general support file with common unit test functions
import sunlineUKF  # import the module that is to be tested
import macros
import sim_model
import ctypes


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_all_sunline_kf(show_plots):
    [testResults, testMessage] = sunline_utilities_test(show_plots)
    assert testResults < 1, testMessage


def sunline_utilities_test(show_plots):
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
                                        sunlineUKF.CrossInit_sunlineUKF)
    moduleWrap.ModelTag = "SunlineUKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

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


    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    test_all_sunline_kf(False)
