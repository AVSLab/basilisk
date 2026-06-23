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

import numpy as np
import pytest

from Basilisk import __path__
from Basilisk.utilities import macros
from Basilisk.utilities.simHelpers import (
    EigenVector3d2list,
    EigenVector3d2np,
    addTimeColumn,
    checkMethodKeyword,
    columnToRowList,
    decimalYearToDateTime,
    flattenList,
    getLineColor,
    getScenarioFigureFileName,
    np2EigenMatrix3d,
    np2EigenVectorXd,
    npList2EigenXdVector,
    pullVectorSetFromData,
    removeTimeFromData,
    samplingTime,
    saveFigurePDF,
    saveScenarioFigure,
    saveScenarioGraphvizFigure,
    timeStringToGregorianUTCMsg,
    writeFigureLaTeX,
    writeTableLaTeX,
    writeTeXSnippet,
)

bskPath = __path__[0]


def isVectorEqual(result, truth, accuracy):
    """function to check if a 3D vector is the same as the truth values"""
    if foundNAN(result):
        return 0

    if np.linalg.norm(result - truth) > accuracy:
        return 0  # return 0 to indicate the array's are not equal
    return 1  # return 1 to indicate the two array's are equal


def isArrayEqual(result, truth, dim, accuracy):
    """function to check if an array of values is the same as the truth values"""
    # the result array is of dimension dim, no time stamp
    # the truth array is of dimesion dim, no time stamp
    if dim < 1:
        print("Incorrect array dimension " + dim + " sent to isArrayEqual")
        return 0

    if len(result) == 0:
        print("Result array was empty")
        return 0

    if len(truth) == 0:
        print("Truth array was empty")
        return 0

    if foundNAN(result):
        return 0

    for i in range(0, dim):
        if math.fabs(result[i] - truth[i]) > accuracy:
            return 0  # return 0 to indicate the array's are not equal
    return 1  # return 1 to indicate the two array's are equal


def isArrayEqualRelative(result, truth, dim, accuracy):
    """Compare relative accuracy of two arrays"""
    # the result array is of dimension dim, no time stamp
    # the truth array is of dimesion dim, no time stamp
    if dim < 1:
        print("Incorrect array dimension " + dim + " sent to isArrayEqual")
        return 0

    if len(result) == 0:
        print("Result array was empty")
        return 0

    if len(truth) == 0:
        print("Truth array was empty")
        return 0

    if foundNAN(result):
        return 0

    for i in range(0, dim):
        if truth[i] == 0:
            if result[i] == 0:
                continue
            else:
                print("Truth array contains zero")
                return 0
        if math.fabs((result[i] - truth[i]) / truth[i]) > accuracy:
            return 0  # return 0 to indicate the array's are not equal
    return 1  # return 1 to indicate the two array's are equal


def isArrayZero(result, dim, accuracy):
    """function to check if an array of values are zero"""
    # the result array is of dimension dim
    if dim < 1:
        print("Incorrect array dimension " + dim + " sent to isArrayEqual")
        return 0

    if len(result) == 0:
        print("Result array was empty")
        return 0

    if foundNAN(result):
        return 0

    for i in range(0, dim):
        if math.fabs(result[i]) > accuracy:
            return 0  # return 0 to indicate the array's are not equal

    return 1  # return 1 to indicate the two array's are equal


def compareVector(
    trueStates, dataStates, accuracy, msg, testFailCount, testMessages, ExpectedResult=1
):
    """Compare two vector size and values and check absolute accuracy"""
    if len(trueStates) != len(dataStates):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" unequal data array sizes\n")
    else:
        if isVectorEqual(dataStates, trueStates, accuracy) != ExpectedResult:
            testFailCount += 1
            testMessages.append("FAILED: " + msg + r"\n")
    return testFailCount, testMessages


def compareArray(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    """Compare two arrays size and values and check absolute accuracy"""
    if len(trueStates) != len(dataStates):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" unequal data array sizes\n")
    elif len(trueStates) == 0 or len(dataStates) == 0:
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqual(dataStates[i], trueStates[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + msg + "\n")
    return testFailCount, testMessages


def compareArrayND(
    trueStates, dataStates, accuracy, msg, size, testFailCount, testMessages
):
    """Compare two arrays of size N for size and values and check absolute accuracy"""
    if len(trueStates) != len(dataStates):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" unequal data array sizes\n")
    elif len(trueStates) == 0 or len(dataStates) == 0:
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            try:
                data = dataStates[i].flatten()
            except:
                data = dataStates[i]
            try:
                trueValue = trueStates[i].flatten()
            except:
                trueValue = trueStates[i]
            if not isArrayEqual(data, trueValue, size, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + msg)
    return testFailCount, testMessages


def compareArrayRelative(
    trueStates, dataStates, accuracy, msg, testFailCount, testMessages
):
    """
    Checks whether the relative distance between elements of a pullMessageLogData-derived array and a
    truth array is below a provided accuracy, and return an error if not.

    Args:
        trueStates: iterable of size (m,n);
        dataStates: iterable of size (m,n)
        accuracy: Relative accuracy boundary
        msg:
        testFailCount:
        testMessages:

    """
    if len(trueStates) != len(dataStates):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" unequal data array sizes\n")
    elif len(trueStates) == 0 or len(dataStates) == 0:
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqualRelative(dataStates[i], trueStates[i], 3, accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED: "
                    + msg
                    + " at t="
                    + str(dataStates[i, 0] * macros.NANO2SEC)
                    + r"sec\n"
                )
    return testFailCount, testMessages


def isDoubleEqual(result, truth, accuracy):
    """function to check if a double equals a truth value"""
    if foundNAN(result):
        return 0

    if math.fabs(result - truth) > accuracy:
        return 0  # return 0 to indicate the doubles are not equal

    return 1  # return 1 to indicate the doubles are equal


def isDoubleEqualRelative(result, truth, accuracy):
    """function to check if a double equals a truth value with relative tolerance"""
    if foundNAN(result):
        return 0
    if foundNAN(truth):
        return 0
    if foundNAN(accuracy):
        return 0
    if truth == 0:
        print("truth is zero, cannot compare")
        return 0

    if math.fabs((truth - result) / truth) > accuracy:
        return 0  # return 0 to indicate the doubles are not equal

    return 1  # return 1 to indicate the doubles are equal


def compareDoubleArrayRelative(
    trueStates, dataStates, accuracy, msg, testFailCount, testMessages
):
    """Compare two arrays of doubles for size and values and check relative accuracy"""
    if len(trueStates) != len(dataStates):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" unequal data array sizes\n")
    elif len(trueStates) == 0 or len(dataStates) == 0:
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isDoubleEqualRelative(dataStates[i], trueStates[i], accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + msg + "\n")
    return testFailCount, testMessages


def compareDoubleArray(
    trueStates, dataStates, accuracy, msg, testFailCount, testMessages
):
    """Compare two arrays of doubles for size and values and check absolute accuracy"""
    if len(trueStates) != len(dataStates):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" unequal data array sizes\n")
    elif len(trueStates) == 0 or len(dataStates) == 0:
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isDoubleEqual(dataStates[i], trueStates[i], accuracy):
                testFailCount += 1
                testMessages.append("FAILED: " + msg + "\n")
    return testFailCount, testMessages


def compareListRelative(
    trueStates, dataStates, accuracy, msg, testFailCount, testMessages
):
    """Compare two row lists of values and check relative accuracy"""
    if len(trueStates) != len(dataStates):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" unequal data array sizes\n")
    elif len(trueStates) == 0 or len(dataStates) == 0:
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        if not trueStates == pytest.approx(dataStates, rel=accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + msg + "\n")
    return testFailCount, testMessages


def compareList(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    """Compare two row lists of values and check relative accuracy"""
    if len(trueStates) != len(dataStates):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" unequal data array sizes\n")
    elif len(trueStates) == 0 or len(dataStates) == 0:
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        if not trueStates == pytest.approx(dataStates, abs=accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + msg + "\n")
    return testFailCount, testMessages


def foundNAN(array):
    """check if an array contains NAN values"""
    if np.isnan(np.sum(array)):
        print("Warning: found NaN value.")
        return 1  # return 1 to indicate a NaN value was found
    return 0
