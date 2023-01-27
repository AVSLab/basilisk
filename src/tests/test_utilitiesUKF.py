
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


import math

import numpy
import pytest

from Basilisk.fswAlgorithms import inertialUKF
from Basilisk.fswAlgorithms import sunlineSuKF


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("filterModule", [('inertialUKF'), ('sunlineSuKF')])
def test_all_utilities_ukf(show_plots, filterModule):
    """Test the filter utilities"""
    [testResults, testMessage] = utilities_nominal(filterModule)
    assert testResults < 1, testMessage
    [testResults, testMessage] = utilities_fault(filterModule)
    assert testResults < 1, testMessage

def utilities_nominal(filterModule):
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


    RVector = inertialUKF.new_doubleArray(len(AMatrix))
    AVector = inertialUKF.new_doubleArray(len(AMatrix))

    #RVector = eval(filterModule + '.new_doubleArray(len(AMatrix))')
    #AVector = eval(filterModule + '.new_doubleArray(len(AMatrix))')
    for i in range(len(AMatrix)):
        eval(filterModule + '.doubleArray_setitem(AVector, i, AMatrix[i])')
        eval(filterModule + '.doubleArray_setitem(RVector, i, 0.0)')

    eval(filterModule + '.ukfQRDJustR(AVector, 6, 4, RVector)')
    RMatrix = []
    for i in range(4 * 4):
        RMatrix.append(eval(filterModule + '.doubleArray_getitem(RVector, i)'))
    RBaseNumpy = numpy.array(RMatrix).reshape(4, 4)
    AMatNumpy = numpy.array(AMatrix).reshape(6, 4)
    q, r = numpy.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i, i] < 0.0:
            r[i, :] *= -1.0
    if numpy.linalg.norm(r - RBaseNumpy) > 1.0E-15:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")

    AMatrix = [1.09327, 1.10927, -0.863653, 1.32288,
               -1.21412, -1.1135, -0.00684933, -2.43508,
               -0.769666, 0.371379, -0.225584, -1.76492,
               -1.08906, 0.0325575, 0.552527, -1.6256,
               1.54421, 0.0859311, -1.49159, 1.59683]

    RVector = eval(filterModule + '.new_doubleArray(len(AMatrix))')
    AVector = eval(filterModule + '.new_doubleArray(len(AMatrix))')
    for i in range(len(AMatrix)):
        eval(filterModule + '.doubleArray_setitem(AVector, i, AMatrix[i])')
        eval(filterModule + '.doubleArray_setitem(RVector, i, 0.0)')

    eval(filterModule + '.ukfQRDJustR(AVector, 5, 4, RVector)')
    RMatrix = []
    for i in range(4 * 4):
        RMatrix.append(eval(filterModule + '.doubleArray_getitem(RVector, i)'))
    RBaseNumpy = numpy.array(RMatrix).reshape(4, 4)
    AMatNumpy = numpy.array(AMatrix).reshape(5, 4)
    q, r = numpy.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i, i] < 0.0:
            r[i, :] *= -1.0
    if numpy.linalg.norm(r - RBaseNumpy) > 1.0E-14:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")

    AMatrix = [0.2236, 0,
               0, 0.2236,
               -0.2236, 0,
               0, -0.2236,
               0.0170, 0,
               0, 0.0170]

    RVector = eval(filterModule + '.new_doubleArray(len(AMatrix))')
    AVector = eval(filterModule + '.new_doubleArray(len(AMatrix))')
    for i in range(len(AMatrix)):
        eval(filterModule + '.doubleArray_setitem(AVector, i, AMatrix[i])')
        eval(filterModule + '.doubleArray_setitem(RVector, i, 0.0)')

    eval(filterModule + '.ukfQRDJustR(AVector, 6, 2, RVector)')
    RMatrix = []
    for i in range(2 * 2):
        RMatrix.append(eval(filterModule + '.doubleArray_getitem(RVector, i)'))
    RBaseNumpy = numpy.array(RMatrix).reshape(2, 2)
    AMatNumpy = numpy.array(AMatrix).reshape(6, 2)
    q, r = numpy.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i, i] < 0.0:
            r[i, :] *= -1.0

    if numpy.linalg.norm(r - RBaseNumpy) > 1.0E-15:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")

    LUSourceMat = [8, 1, 6, 3, 5, 7, 4, 9, 2]
    LUSVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat))')
    LVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat))')
    UVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat))')
    intSwapVector = eval(filterModule + '.new_intArray(3)')

    for i in range(len(LUSourceMat)):
        eval(filterModule + '.doubleArray_setitem(LUSVector, i, LUSourceMat[i])')
        eval(filterModule + '.doubleArray_setitem(UVector, i, 0.0)')
        eval(filterModule + '.doubleArray_setitem(LVector, i, 0.0)')

    exCount = eval(filterModule + '.ukfLUD(LUSVector, 3, 3, LVector, intSwapVector)')
    # eval(filterModule + '.ukfUInv(LUSVector, 3, 3, UVector)
    LMatrix = []
    UMatrix = []
    # UMatrix = []
    for i in range(3):
        currRow = eval(filterModule + '.intArray_getitem(intSwapVector, i)')
        for j in range(3):
            if (j < i):
                LMatrix.append(eval(filterModule + '.doubleArray_getitem(LVector, i * 3 + j)'))
                UMatrix.append(0.0)
            elif (j > i):
                LMatrix.append(0.0)
                UMatrix.append(eval(filterModule + '.doubleArray_getitem(LVector, i * 3 + j)'))
            else:
                LMatrix.append(1.0)
                UMatrix.append(eval(filterModule + '.doubleArray_getitem(LVector, i * 3 + j)'))
    # UMatrix.append(eval(filterModule + '.doubleArray_getitem(UVector, i))

    LMatrix = numpy.array(LMatrix).reshape(3, 3)
    UMatrix = numpy.array(UMatrix).reshape(3, 3)
    outMat = numpy.dot(LMatrix, UMatrix)
    outMatSwap = numpy.zeros((3, 3))
    for i in range(3):
        currRow = eval(filterModule + '.intArray_getitem(intSwapVector, i)')
        outMatSwap[i, :] = outMat[currRow, :]
        outMat[currRow, :] = outMat[i, :]
    LuSourceArray = numpy.array(LUSourceMat).reshape(3, 3)

    if (numpy.linalg.norm(outMatSwap - LuSourceArray) > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Decomposition accuracy failure")

    EqnSourceMat = [2.0, 1.0, 3.0, 2.0, 6.0, 8.0, 6.0, 8.0, 18.0]
    BVector = [1.0, 3.0, 5.0]
    EqnVector = eval(filterModule + '.new_doubleArray(len(EqnSourceMat))')
    EqnBVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat) // 3)')
    EqnOutVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat) // 3)')

    for i in range(len(EqnSourceMat)):
        eval(filterModule + '.doubleArray_setitem(EqnVector, i, EqnSourceMat[i])')
        eval(filterModule + '.doubleArray_setitem(EqnBVector, i // 3, BVector[i // 3])')
        eval(filterModule + '.intArray_setitem(intSwapVector, i // 3, 0)')
        eval(filterModule + '.doubleArray_setitem(LVector, i, 0.0)')

    exCount = eval(filterModule + '.ukfLUD(EqnVector, 3, 3, LVector, intSwapVector)')
    eval(filterModule + '.ukfLUBckSlv(LVector, 3, 3, intSwapVector, EqnBVector, EqnOutVector)')

    expectedSol = [3.0 / 10.0, 4.0 / 10.0, 0.0]
    errorVal = 0.0
    for i in range(3):
        errorVal += abs(eval(filterModule + '.doubleArray_getitem(EqnOutVector, i) - expectedSol[i]'))

    if (errorVal > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Back-Solve accuracy failure")

    InvSourceMat = [8, 1, 6, 3, 5, 7, 4, 9, 2]
    SourceVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat))')
    InvVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat))')
    for i in range(len(InvSourceMat)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, InvSourceMat[i])')
        eval(filterModule + '.doubleArray_setitem(InvVector, i, 0.0)')
    nRow = int(math.sqrt(len(InvSourceMat)))
    eval(filterModule + '.ukfMatInv(SourceVector, nRow, nRow, InvVector)')

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(eval(filterModule + '.doubleArray_getitem(InvVector, i)'))

    InvOut = numpy.array(InvOut).reshape(nRow, nRow)
    expectIdent = numpy.dot(InvOut, numpy.array(InvSourceMat).reshape(3, 3))
    errorNorm = numpy.linalg.norm(expectIdent - numpy.identity(3))
    if (errorNorm > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Matrix Inverse accuracy failure")

    cholTestMat = [1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0]
    SourceVector = eval(filterModule + '.new_doubleArray(len(cholTestMat))')
    CholVector = eval(filterModule + '.new_doubleArray(len(cholTestMat))')
    for i in range(len(cholTestMat)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, cholTestMat[i])')
        eval(filterModule + '.doubleArray_setitem(CholVector, i, 0.0)')
    nRow = int(math.sqrt(len(cholTestMat)))
    eval(filterModule + '.ukfCholDecomp(SourceVector, nRow, nRow, CholVector)')
    cholOut = []
    for i in range(len(cholTestMat)):
        cholOut.append(eval(filterModule + '.doubleArray_getitem(CholVector, i)'))

    cholOut = numpy.array(cholOut).reshape(nRow, nRow)
    cholComp = numpy.linalg.cholesky(numpy.array(cholTestMat).reshape(nRow, nRow))
    errorNorm = numpy.linalg.norm(cholOut - cholComp)
    if (errorNorm > 1.0E-14):
        testFailCount += 1
        testMessages.append("Cholesky Matrix Decomposition accuracy failure")

    InvSourceMat = [2.1950926119414667, 0.0, 0.0, 0.0,
                    1.0974804773131115, 1.9010439702743847, 0.0, 0.0,
                    0.0, 1.2672359635912551, 1.7923572711881284, 0.0,
                    1.0974804773131113, -0.63357997864171967, 1.7920348101787789, 0.033997451205364251]

    SourceVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat))')
    InvVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat))')
    for i in range(len(InvSourceMat)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, InvSourceMat[i])')
        eval(filterModule + '.doubleArray_setitem(InvVector, i, 0.0)')
    nRow = int(math.sqrt(len(InvSourceMat)))
    eval(filterModule + '.ukfLInv(SourceVector, nRow, nRow, InvVector)')

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(eval(filterModule + '.doubleArray_getitem(InvVector, i)'))

    InvOut = numpy.array(InvOut).reshape(nRow, nRow)
    expectIdent = numpy.dot(InvOut, numpy.array(InvSourceMat).reshape(nRow, nRow))
    errorNorm = numpy.linalg.norm(expectIdent - numpy.identity(nRow))
    if (errorNorm > 1.0E-12):
        print(errorNorm)
        testFailCount += 1
        testMessages.append("L Matrix Inverse accuracy failure")

    InvSourceMat = numpy.transpose(numpy.array(InvSourceMat).reshape(nRow, nRow)).reshape(nRow * nRow).tolist()
    SourceVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat))')
    InvVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat))')
    for i in range(len(InvSourceMat)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, InvSourceMat[i])')
        eval(filterModule + '.doubleArray_setitem(InvVector, i, 0.0)')
    nRow = int(math.sqrt(len(InvSourceMat)))
    eval(filterModule + '.ukfUInv(SourceVector, nRow, nRow, InvVector)')

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(eval(filterModule + '.doubleArray_getitem(InvVector, i)'))

    InvOut = numpy.array(InvOut).reshape(nRow, nRow)
    expectIdent = numpy.dot(InvOut, numpy.array(InvSourceMat).reshape(nRow, nRow))
    errorNorm = numpy.linalg.norm(expectIdent - numpy.identity(nRow))
    if (errorNorm > 1.0E-12):
        print(errorNorm)
        testFailCount += 1
        testMessages.append("U Matrix Inverse accuracy failure")

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + filterModule +" UKF utilities")

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


def utilities_fault(filterModule):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages


    LUSourceMat = [8, 1, 6, 3, 5, 7, 4, 9, 2, 1, 9, 3]
    EqnSourceMat = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    BVector = [0.0, 0.0, 0.0]

    LUSVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat))')
    LVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat))')
    UVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat))')
    intSwapVector = eval(filterModule + '.new_intArray(3)')
    EqnVector = eval(filterModule + '.new_doubleArray(len(EqnSourceMat))')
    EqnBVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat) // 3)')
    EqnOutVector = eval(filterModule + '.new_doubleArray(len(LUSourceMat) // 3)')

    for i in range(len(LUSourceMat)):
        eval(filterModule + '.doubleArray_setitem(LUSVector, i, LUSourceMat[i])')
        eval(filterModule + '.doubleArray_setitem(UVector, i, 0.0)')
        eval(filterModule + '.doubleArray_setitem(LVector, i, 0.0)')

    exCount = eval(filterModule + '.ukfLUD(LUSVector, 3, 4, LVector, intSwapVector)')
    returnBackSolve = eval(filterModule + '.ukfLUBckSlv(LVector, 3, 4, intSwapVector, EqnBVector, EqnOutVector)')

    if (exCount >= 0 or exCount is None):
        testFailCount += 1
        testMessages.append("LU Decomposition not square")

    if (returnBackSolve >= 0 or returnBackSolve is None):
        testFailCount += 1
        testMessages.append("LU Back-Solve accuracy failure")


    for i in range(len(EqnSourceMat)):
        eval(filterModule + '.doubleArray_setitem(EqnVector, i, EqnSourceMat[i])')
        eval(filterModule + '.doubleArray_setitem(EqnBVector, i // 3, BVector[i // 3])')
        eval(filterModule + '.intArray_setitem(intSwapVector, i // 3, 0)')
        eval(filterModule + '.doubleArray_setitem(LVector, i, 0.0)')

    exCount = eval(filterModule + '.ukfLUD(EqnVector, 3, 3, LVector, intSwapVector)')
    returnBackSolve = eval(filterModule + '.ukfLUBckSlv(LVector, 3, 3, intSwapVector, EqnBVector, EqnOutVector)')

    if (exCount >= 0 or exCount is None):
        testFailCount += 1
        testMessages.append("LU Back-Solve accuracy failure")

    if (returnBackSolve >= 0 or returnBackSolve is None):
        testFailCount += 1
        testMessages.append("LU Back-Solve accuracy failure")


    InvSourceMat = [8, 1, 6, 3, 5, 7, 4, 9, 2, 4, 4, 5]
    SourceVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat))')
    InvVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat))')
    for i in range(len(InvSourceMat)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, InvSourceMat[i])')
        eval(filterModule + '.doubleArray_setitem(InvVector, i, 0.0)')
    returnInv = eval(filterModule + '.ukfMatInv(SourceVector, 3, 4, InvVector)')

    if (returnInv >=0 or returnInv is None):
        testFailCount += 1
        testMessages.append("LU Matrix Inverse accuracy failure")

    cholTestMatLong = [1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0, 10.0, 5.0, 10.0]
    SourceVector = eval(filterModule + '.new_doubleArray(len(cholTestMatLong))')
    CholVector = eval(filterModule + '.new_doubleArray(len(cholTestMatLong))')
    for i in range(len(cholTestMatLong)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, cholTestMatLong[i])')
        eval(filterModule + '.doubleArray_setitem(CholVector, i, 0.0)')
    returnChol = eval(filterModule + '.ukfCholDecomp(SourceVector, 3, 4, CholVector)')

    if (returnChol >=0 or returnChol is None):
        testFailCount += 1
        testMessages.append("Cholesky Matrix Decomposition Size failure")

    cholTestMat = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    SourceVector = eval(filterModule + '.new_doubleArray(len(cholTestMat))')
    CholVector = eval(filterModule + '.new_doubleArray(len(cholTestMat))')
    for i in range(len(cholTestMat)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, cholTestMat[i])')
        eval(filterModule + '.doubleArray_setitem(CholVector, i, 0.0)')
    nRow = int(math.sqrt(len(cholTestMat)))
    returnChol = eval(filterModule + '.ukfCholDecomp(SourceVector, nRow, nRow, CholVector)')

    if (returnChol >=0 or returnChol is None):
        testFailCount += 1
        testMessages.append("Cholesky Matrix Decomposition Zero Matrix")

    rMat = [1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0]
    xVec = [100.0, 0.0, 0.0]
    inRMat = eval(filterModule + '.new_doubleArray(len(rMat))')
    inXVec = eval(filterModule + '.new_doubleArray(len(xVec))')
    for i in range(len(rMat)):
        eval(filterModule + '.doubleArray_setitem(inRMat, i, rMat[i])')
    for i in range(len(xVec)):
        eval(filterModule + '.doubleArray_setitem(inXVec, i, xVec[i])')
    nRow = int(math.sqrt(len(rMat)))
    returnDown = eval(filterModule + '.ukfCholDownDate(inRMat, inXVec, -100, 3, CholVector)')

    if (returnChol >=0 or returnChol is None):
        testFailCount += 1
        testMessages.append("Cholesky Matrix Decomposition Size")

    InvSourceMatRect = [2.1950926119414667, 0.0, 0.0, 0.0, 1.,
                    1.0974804773131115, 1.9010439702743847, 0.0, 0.0, 1.,
                    0.0, 1.2672359635912551, 1.7923572711881284, 0.0, 1.,
                    1.0974804773131113, -0.63357997864171967, 1.7920348101787789, 0.033997451205364251, 1.]

    SourceVector = eval(filterModule + '.new_doubleArray(len(InvSourceMatRect))')
    InvVector = eval(filterModule + '.new_doubleArray(len(InvSourceMatRect))')
    for i in range(len(InvSourceMatRect)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, InvSourceMatRect[i])')
        eval(filterModule + '.doubleArray_setitem(InvVector, i, 0.0)')
    nRow = int(math.sqrt(len(InvSourceMatRect)))
    returnLinv = eval(filterModule + '.ukfLInv(SourceVector, nRow, nRow, InvVector)')

    if (returnLinv >= 0 or returnLinv is None):
        testFailCount += 1
        testMessages.append("L Matrix Inverse didn't catch size error")

    InvSourceMat0 = [0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0,]

    SourceVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat0))')
    InvVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat0))')
    for i in range(len(InvSourceMat0)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, InvSourceMat0[i])')
        eval(filterModule + '.doubleArray_setitem(InvVector, i, 0.0)')
    nRow = int(math.sqrt(len(InvSourceMat0)))
    returnLinv = eval(filterModule + '.ukfLInv(SourceVector, nRow, nRow, InvVector)')

    if (returnLinv >= 0 or returnLinv is None):
        testFailCount += 1
        testMessages.append("L Matrix Inverse didn't catch sign error")

    SourceVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat0))')
    InvVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat0))')
    for i in range(len(InvSourceMat0)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, InvSourceMat0[i])')
        eval(filterModule + '.doubleArray_setitem(InvVector, i, 0.0)')
    nRow = int(math.sqrt(len(InvSourceMat0)))
    returnU = eval(filterModule + '.ukfUInv(SourceVector, nRow, nRow, InvVector)')

    if (returnU >= 0 or returnU is None):
        testFailCount += 1
        testMessages.append("U Matrix Inverse didn't catch non square matrix")

    SourceVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat0))')
    InvVector = eval(filterModule + '.new_doubleArray(len(InvSourceMat0))')
    for i in range(len(InvSourceMat0)):
        eval(filterModule + '.doubleArray_setitem(SourceVector, i, InvSourceMat0[i])')
        eval(filterModule + '.doubleArray_setitem(InvVector, i, 0.0)')
    nRow = int(math.sqrt(len(InvSourceMat0)))
    returnU = eval(filterModule + '.ukfUInv(SourceVector, nRow, nRow, InvVector)')

    if (returnU >= 0 or returnU is None):
        testFailCount += 1
        testMessages.append("U Matrix Inverse didn't catch zero matrix")

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + filterModule + " UKF utilities")

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    utilities_fault('inertialUKF')
