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


import errno
import math
import os
from datetime import datetime, timedelta

import matplotlib as mpl
import numpy as np
import pytest
from Basilisk.architecture import bskUtilities
from Basilisk.architecture import messaging
from Basilisk.topLevelModules import pyswice

mpl.rc("figure", facecolor="white")
mpl.rc('xtick', labelsize=9)
mpl.rc('ytick', labelsize=9)
mpl.rc("figure", figsize=(5.75, 2.5))
mpl.rc('axes', labelsize=10)
mpl.rc('legend', fontsize=9)
mpl.rc('figure', autolayout=True)
mpl.rc('figure', max_open_warning=30)
mpl.rc('legend', loc='lower right')

import matplotlib.colors as colors
import matplotlib.cm as cmx

from Basilisk.utilities import macros

from Basilisk import __path__

bskPath = __path__[0]

try:
    from Basilisk.utilities import tabulate as T

    # '''
    # del(T.LATEX_ESCAPE_RULES['$'])
    # del(T.LATEX_ESCAPE_RULES['\\'])
    # del(T.LATEX_ESCAPE_RULES['_'])
    # del(T.LATEX_ESCAPE_RULES['{'])
    # del(T.LATEX_ESCAPE_RULES['}'])
    # '''

    del (T.LATEX_ESCAPE_RULES[u'$'])
    del (T.LATEX_ESCAPE_RULES[u'\\'])
    del (T.LATEX_ESCAPE_RULES[u'_'])
    del (T.LATEX_ESCAPE_RULES[u'{'])
    del (T.LATEX_ESCAPE_RULES[u'}'])
    from Basilisk.utilities.tabulate import *
except:
    pass

def isVectorEqual(result, truth, accuracy):
    """function to check if a 3D vector is the same as the truth values"""
    if foundNAN(result): return 0

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

    if foundNAN(result): return 0

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

    if foundNAN(result): return 0

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

    if foundNAN(result): return 0

    for i in range(0, dim):
        if (math.fabs(result[i]) > accuracy):
            return 0  # return 0 to indicate the array's are not equal

    return 1  # return 1 to indicate the two array's are equal


def compareVector(trueStates, dataStates, accuracy, msg, testFailCount, testMessages, ExpectedResult=1):
    """Compare two vector size and values and check absolute accuracy"""
    if (len(trueStates) != len(dataStates)):
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


def compareArrayND(trueStates, dataStates, accuracy, msg, size, testFailCount, testMessages):
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


def compareArrayRelative(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
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
    if (len(trueStates) != len(dataStates)):
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
                testMessages.append("FAILED: " + msg + " at t=" + str(dataStates[i, 0] * macros.NANO2SEC) + r"sec\n")
    return testFailCount, testMessages


def isDoubleEqual(result, truth, accuracy):
    """function to check if a double equals a truth value"""
    if foundNAN(result): return 0

    if math.fabs(result - truth) > accuracy:
        return 0  # return 0 to indicate the doubles are not equal

    return 1  # return 1 to indicate the doubles are equal


def isDoubleEqualRelative(result, truth, accuracy):
    """function to check if a double equals a truth value with relative tolerance"""
    if foundNAN(result): return 0
    if foundNAN(truth): return 0
    if foundNAN(accuracy): return 0
    if truth == 0:
        print("truth is zero, cannot compare")
        return 0

    if math.fabs((truth - result) / truth) > accuracy:
        return 0  # return 0 to indicate the doubles are not equal

    return 1  # return 1 to indicate the doubles are equal


def compareDoubleArrayRelative(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
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


def compareDoubleArray(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
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


def compareListRelative(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
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


def writeTableLaTeX(tableName, tableHeaders, caption, array, path):
    """Take a list and return equivalent LaTeX table code"""

    texFileName = path + "/../_Documentation/AutoTeX/" + tableName + ".tex"

    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(texFileName, "w") as texTable:
        table = tabulate(array,
                         tableHeaders,
                         tablefmt="latex",
                         numalign="center"
                         )

        texTable.write(r'\begin{table}[htbp]')
        texTable.write(r'\caption{' + caption + '}')
        texTable.write(r'\label{tbl:' + tableName + '}')
        texTable.write(r'\centering')
        texTable.write(table)
        texTable.write(r'\end{table}')
        texTable.close()

    return


def writeTeXSnippet(snippetName, texSnippet, path):
    """Write a LaTeX snippet to a file"""

    texFileName = path + "/../_Documentation/AutoTeX/" + snippetName + ".tex"

    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(texFileName, "w") as fileHandler:
        fileHandler.write(texSnippet)
        fileHandler.close()

    return


def saveScenarioFigure(figureName, plt, path, extension=".svg"):
    """save a python scenario result into the documentation image folder"""
    imgFileName = os.path.join(path, "..", "..", "docs", "source", "_images", "Scenarios", figureName + extension)
    if not os.path.exists(os.path.dirname(imgFileName)):
        try:
            os.makedirs(os.path.dirname(imgFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    plt.savefig(imgFileName, transparent=True)


def saveFigurePDF(figureName, plt, path):
    """Save a Figure as a PDF"""
    figFileName = os.path.join(path, figureName + ".pdf")
    if not os.path.exists(os.path.dirname(figFileName)):
        try:
            os.makedirs(os.path.dirname(figFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    plt.savefig(figFileName, transparent=True, pad_inches=0.05)


def writeFigureLaTeX(figureName, caption, plt, format, path):
    """Save a figure and associated TeX code snippet"""
    texFileName = os.path.join(path, "..", "_Documentation", "AutoTeX", figureName + ".tex")
    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(texFileName, "w") as texFigure:
        texFigure.write(r'\begin{figure}[htbp]')
        texFigure.write(r'\centerline{')
        texFigure.write(r'\includegraphics[' + format + ']{AutoTeX/' + figureName + r'}}')
        texFigure.write(r'\caption{' + caption + r'}')
        texFigure.write(r'\label{fig:' + figureName + r'}')
        texFigure.write(r'\end{figure}')
        texFigure.close()

        texFileName = path + "/../_Documentation/AutoTeX/" + figureName + ".pdf"
        plt.savefig(texFileName, transparent=True)

    return


def foundNAN(array):
    """check if an array contains NAN values"""
    if (np.isnan(np.sum(array))):
        print("Warning: found NaN value.")
        return 1  # return 1 to indicate a NaN value was found
    return 0


def getLineColor(idx, maxNum):
    """pick a nicer color pattern to plot 3 vector components"""
    values = list(range(0, maxNum + 2))
    colorMap = mpl.pyplot.get_cmap('gist_earth')
    cNorm = colors.Normalize(vmin=0, vmax=values[-1])
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=colorMap)
    return scalarMap.to_rgba(values[idx + 1])


def np2EigenMatrix3d(mat):
    """convert 3D numpy matrix to Eigen matrix"""
    return [
        [mat[0], mat[1], mat[2]]
        , [mat[3], mat[4], mat[5]]
        , [mat[6], mat[7], mat[8]]
    ]


def np2EigenVectorXd(vec):
    """Convert numpy to Eigen vector"""
    npVec = []
    for item in vec:
        npVec.extend([[item]])

    return npVec

def npList2EigenXdVector(list):
    """Conver a list of arrays to a list of eigen values"""
    eigenList = bskUtilities.Eigen3dVector()
    for pos in list:
        eigenList.push_back(pos)
    return eigenList

def EigenVector3d2np(eig):
    """convert Eigen vector3d to numpy"""
    return np.array([eig[0][0], eig[1][0], eig[2][0]])

def flattenList(matrix):
    """
    returns a flattened list
    Args:
        matrix: list of list

    Returns: flattened list

    """
    flat_list = []
    for row in matrix:
        flat_list.extend(row)
    return flat_list

def EigenVector3d2list(eig):
    """convert Eigen vector3d to list"""
    return EigenVector3d2np(eig).tolist()



def pullVectorSetFromData(inpMat):
    """extract the vector data set from a data matrix where the 1st column is the time information"""
    outMat = np.array(inpMat).transpose()
    return outMat[1:].transpose()


def addTimeColumn(time, data):
    """Add a time column to the data set"""
    return np.transpose(np.vstack([[time], np.transpose(data)]))


def decimalYearToDateTime(start):
    """convert a decimal Year format to a regular dataTime object"""
    year = int(start)
    rem = start - year

    base = datetime(year, 1, 1)
    return base + timedelta(seconds=(base.replace(year=base.year + 1) - base).total_seconds() * rem)


def timeStringToGregorianUTCMsg(DateSpice, **kwargs):
    """convert a general time/date string to a gregoarian UTC msg object"""
    # set the data path
    if 'dataPath' in kwargs:
        dataPath = kwargs['dataPath']
        if not isinstance(dataPath, str):
            print('ERROR: dataPath must be a string argument')
            exit(1)
    else:
        dataPath = bskPath + '/supportData/EphemerisData/'  # default value

    # load spice kernel and convert the string into a UTC date/time string
    pyswice.furnsh_c(dataPath + 'naif0012.tls')
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(DateSpice, et)
    etEpoch = pyswice.doubleArray_getitem(et, 0)
    ep1 = pyswice.et2utc_c(etEpoch, 'C', 6, 255, "Yo")
    pyswice.unload_c(dataPath + 'naif0012.tls')  # leap second file

    try:
        # convert UTC string to datetime object
        datetime_object = datetime.strptime(ep1, '%Y %b %d %H:%M:%S.%f')

        # Validate month is in range 1-12
        if datetime_object.month < 1 or datetime_object.month > 12:
            raise ValueError(f"Invalid month value: {datetime_object.month}")

        # populate the epochMsg with the gregorian UTC date/time information
        epochMsgStructure = messaging.EpochMsgPayload()
        epochMsgStructure.year = datetime_object.year
        epochMsgStructure.month = datetime_object.month
        epochMsgStructure.day = datetime_object.day
        epochMsgStructure.hours = datetime_object.hour
        epochMsgStructure.minutes = datetime_object.minute
        epochMsgStructure.seconds = datetime_object.second + datetime_object.microsecond / 1e6

        epochMsg = messaging.EpochMsg().write(epochMsgStructure)

        # Store the message in a global registry to prevent garbage collection
        if not hasattr(timeStringToGregorianUTCMsg, '_msg_registry'):
            timeStringToGregorianUTCMsg._msg_registry = []
        timeStringToGregorianUTCMsg._msg_registry.append(epochMsg)

        return epochMsg

    except Exception as e:
        print(f"Error processing date string '{ep1}': {str(e)}")
        print(f"Original input string was: {DateSpice}")
        raise

def columnToRowList(set):
    """Loop through a column list and return a row list"""
    ans = []
    for item in set:
        ans.append(item[0])
    return ans

def checkMethodKeyword(karglist, kwargs):
    """loop through list of method keyword arguments and make sure that an approved keyword is used."""
    for key in kwargs:
        if key not in karglist:
            print('ERROR: you tried to use an incorrect keyword ' + key + '. Options include:')
            print(karglist)
            exit(1)


def removeTimeFromData(dataList):
    """pull out the time column out of a 4xN data list"""
    return (dataList.transpose()[1:len(dataList[0])]).transpose()


def samplingTime(simTime, baseTimeStep, numDataPoints):
    """
    Given a simulation duration, this routine returns
    a sampling time that yields the closest integer match to a desired number of sampling points

    Args:
        simTime: [ns] total simulation duration
        baseTimeStep: [ns] baseline sampling period
        numDataPoints: nominal desired number of data points over the simulation duration

    """
    deltaTime = math.floor(simTime / baseTimeStep / (numDataPoints - 1)) * baseTimeStep
    if deltaTime < 1:
        deltaTime = 1
    return deltaTime
