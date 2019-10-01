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


#
#   Unit Test Support Script
#
import math
import os,errno
import numpy as np
import matplotlib as mpl
from datetime import datetime, timedelta
from Basilisk.simulation.simMessages import simMessages
from Basilisk.pyswice import pyswice

mpl.rc("figure", facecolor="white")
mpl.rc('xtick', labelsize=9)
mpl.rc('ytick', labelsize=9)
mpl.rc("figure", figsize=(5.75,2.5))
mpl.rc('axes', labelsize=10)
mpl.rc('legend', fontsize=9)
mpl.rc('figure', autolayout=True)
mpl.rc('figure', max_open_warning=30)



import matplotlib.colors as colors
import matplotlib.cm as cmx

from . import macros


from Basilisk import __path__
bskPath = __path__[0]

from . import tabulate as T
'''
del(T.LATEX_ESCAPE_RULES['$'])
del(T.LATEX_ESCAPE_RULES['\\'])
del(T.LATEX_ESCAPE_RULES['_'])
del(T.LATEX_ESCAPE_RULES['{'])
del(T.LATEX_ESCAPE_RULES['}'])
'''

del(T.LATEX_ESCAPE_RULES[u'$'])
del(T.LATEX_ESCAPE_RULES[u'\\'])
del(T.LATEX_ESCAPE_RULES[u'_'])
del(T.LATEX_ESCAPE_RULES[u'{'])
del(T.LATEX_ESCAPE_RULES[u'}'])
from .tabulate import *



#
#   function to check if a 3D vector is the same as the truth values
#
def isVectorEqual(result, truth, accuracy):

    if foundNAN(result): return 0

    if np.linalg.norm(result - truth) > accuracy:
        return 0        # return 0 to indicate the array's are not equal
    return 1            # return 1 to indicate the two array's are equal



#
#   function to check if an array of values is the same as the truth values
#
def isArrayEqual(result, truth, dim, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    # the truth array is of dimesion dim, no time stamp
    if dim < 1:
        print("Incorrect array dimension " + dim + " sent to isArrayEqual")
        return 0

    if len(result)==0:
        print("Result array was empty")
        return 0

    if len(truth)==0:
        print("Truth array was empty")
        return 0

    if foundNAN(result): return 0

    for i in range(0,dim):
        if math.fabs(result[i+1] - truth[i]) > accuracy:
            return 0    # return 0 to indicate the array's are not equal
    return 1            # return 1 to indicate the two array's are equal

def isArrayEqualRelative(result, truth, dim, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    # the truth array is of dimesion dim, no time stamp
    if dim < 1:
        print("Incorrect array dimension " + dim + " sent to isArrayEqual")
        return 0

    if len(result)==0:
        print("Result array was empty")
        return 0

    if len(truth)==0:
        print("Truth array was empty")
        return 0

    if foundNAN(result): return 0

    for i in range(0,dim):
        if truth[i] == 0:
            if result[i+1] == 0:
                continue
            else:
                print("Truth array contains zero")
                return 0
        if math.fabs((result[i+1] - truth[i])/truth[i]) > accuracy:
            return 0    # return 0 to indicate the array's are not equal
    return 1            # return 1 to indicate the two array's are equal
#
#   function to check if an array of values are zero
#
def isArrayZero(result, dim, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    if dim < 1:
        print("Incorrect array dimension " + dim + " sent to isArrayEqual")
        return 0

    if len(result)==0:
        print("Result array was empty")
        return 0

    if foundNAN(result): return 0

    for i in range(0,dim):
        if (math.fabs(result[i+1]) > accuracy):
            return 0    # return 0 to indicate the array's are not equal

    return 1            # return 1 to indicate the two array's are equal


#
#   Compare two vector size and values and check absolute accuracy
#
def compareVector(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+ r" unequal data array sizes\n")
    else:
        if not isVectorEqual(dataStates, trueStates, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: "+msg+ r"\n")
    return testFailCount, testMessages


#
#   Compare two arrays size and values and check absolute accuracy
#
def compareArray(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+ r" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqual(dataStates[i], trueStates[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+r"sec\n")
    return testFailCount, testMessages

#
#   Compare two arrays of size N for size and values and check absolute accuracy
#
def compareArrayND(trueStates, dataStates, accuracy, msg, size, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+r" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqual(dataStates[i], trueStates[i], size, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+r"sec\n")
    return testFailCount, testMessages


#
#   Compare two arrays size and values and check relative accuracy
#
def compareArrayRelative(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    """
    Checks whether the relative distance between elements of a pullMessageLogData-derived array and a truth array is below a provided
    accuracy, and return an error if not.
    :param trueStates: iterable of size (m,n);
    :param dataStates: iterable of size (m+1,n), where the first column can be ignored
    :param accuracy: Relative accuracy boundary
    :param msg:
    :param testFailCount:
    :param testMessages:
    :return:
    """
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+r" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqualRelative(dataStates[i], trueStates[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+r"sec\n")
    return testFailCount, testMessages

#
#   function to check if a double equals a truth value
#
def isDoubleEqual(result, truth, accuracy):
    if foundNAN(result): return 0

    # the result array is of dimension dim+1, as the first entry is the time stamp
    if (math.fabs(result[1] - truth) > accuracy):
        return 0    # return 0 to indicate the doubles are not equal

    return 1        # return 1 to indicate the doubles are equal

#
#   function to check if a double equals a truth value with relative tolerance
#
def isDoubleEqualRelative(result, truth, accuracy):
    if foundNAN(result): return 0
    if foundNAN(truth): return 0
    if foundNAN(accuracy): return 0
    if truth == 0:
        print("truth is zero, cannot compare")
        return 0

    # the result array is of dimension dim+1, as the first entry is the time stamp
    if (math.fabs((truth - result)/truth) > accuracy):
        return 0    # return 0 to indicate the doubles are not equal

    return 1        # return 1 to indicate the doubles are equal

#
#   Compare two arrays of doubles for size and values and check relative accuracy
#
def compareDoubleArrayRelative(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+r" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isDoubleEqualRelative(dataStates[i,1], trueStates[i], accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+r"sec\n")
    return testFailCount, testMessages

#
#   Compare two arrays of doubles for size and values and check absolute accuracy
#
def compareDoubleArray(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+r" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + r" data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isDoubleEqual(dataStates[i], trueStates[i], accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+r"sec\n")
    return testFailCount, testMessages


def writeTableLaTeX(tableName, tableHeaders, caption, array, path):

    texFileName = path+"/../_Documentation/AutoTeX/"+tableName+".tex"

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

    texFileName = path+"/../_Documentation/AutoTeX/"+snippetName+".tex"

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

#
#   save a python scenario result into the Doxygen image folder
#
def saveScenarioFigure(figureName, plt, path, extension = ".svg"):
    texFileName = path + "/../../../docs/Images/Scenarios/"+figureName+extension
    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    plt.savefig(texFileName, transparent=True)

def saveFigurePDF(figureName, plt, path):
    figFileName = path+figureName+".pdf"
    if not os.path.exists(os.path.dirname(figFileName)):
        try:
            os.makedirs(os.path.dirname(figFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    plt.savefig(figFileName, transparent=True, bbox_inches='tight', pad_inches=0.05)


def writeFigureLaTeX(figureName, caption, plt, format, path):

    texFileName = path + "/../_Documentation/AutoTeX/" + figureName + ".tex"
    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(texFileName, "w") as texFigure:
        texFigure.write(r'\begin{figure}[htbp]')
        texFigure.write(r'\centerline{')
        texFigure.write(r'\includegraphics[' + format +']{AutoTeX/' + figureName + r'}}')
        texFigure.write(r'\caption{' + caption + r'}')
        texFigure.write(r'\label{fig:' + figureName + r'}')
        texFigure.write(r'\end{figure}')
        texFigure.close()

        texFileName = path + "/../_Documentation/AutoTeX/" + figureName + ".pdf"
        plt.savefig(texFileName, transparent=True)

    return

def foundNAN(array):
    if (np.isnan(np.sum(array))):
        print("Warning: found NaN value.")
        return 1        # return 1 to indicate a NaN value was found
    return 0

#
#   macro to create and write a general message
#
def setMessage(simObject, processName, msgName, inputMessageData, msgStrName = ""):
    """
    Creates a message defined by msgName that appears in the process defined by processName.
    :param simObject: the TotalSim attribute of a simulationBaseClass instance.
    :param processName: str : the name of a process in which to write the message.
    :param msgName: str : the name of the message, used for publish/subscribe
    :param inputMessageData: the structure containing the message information
    :param msgStrName: ???
    :return:
    """
    inputMessageSize = inputMessageData.getStructSize()
    simObject.CreateNewMessage(processName,
                               msgName,
                               inputMessageSize,
                               2,
                               msgStrName)
    simObject.WriteMessageData(msgName,
                               inputMessageSize,
                               0,
                               inputMessageData)


#
#   pick a nicer color pattern to plot 3 vector components
#

def getLineColor(idx,maxNum):
    values = list(range(0, maxNum+2))
    colorMap = mpl.pyplot.get_cmap('gist_earth')
    cNorm = colors.Normalize(vmin=0, vmax=values[-1])
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=colorMap)
    return scalarMap.to_rgba(values[idx])

def np2EigenMatrix3d(mat):
    return [
        [mat[0], mat[1], mat[2]]
        ,[mat[3], mat[4], mat[5]]
        ,[mat[6], mat[7], mat[8]]
    ]

def np2EigenVectorXd(vec):
    npVec = []
    for item in vec:
        npVec.extend([[item]])

    return npVec

def EigenVector3d2np(eig):
    return np.array([eig[0][0], eig[1][0], eig[2][0]])

def pullVectorSetFromData(inpMat):
    outMat = np.array(inpMat).transpose()
    return outMat[1:].transpose()



def decimalYearToDateTime(start):
    year = int(start)
    rem = start - year

    base = datetime(year, 1, 1)
    return base + timedelta(seconds=(base.replace(year=base.year + 1) - base).total_seconds() * rem)


def timeStringToGregorianUTCMsg(DateSpice, **kwargs):

    # set the data path
    if 'dataPath' in kwargs:
        dataPath = kwargs['dataPath']
        if not isinstance(dataPath, str):
            print('ERROR: dataPath must be a string argument')
            exit(1)
    else:
        dataPath = bskPath +'/supportData/EphemerisData/'  # default value

    # load spice kernal and convert the string into a UTC date/time string
    pyswice.furnsh_c(dataPath + 'naif0012.tls')
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(DateSpice, et)
    etEpoch = pyswice.doubleArray_getitem(et, 0)
    ep1 = pyswice.et2utc_c(etEpoch, 'C', 6, 255, "Yo")
    pyswice.unload_c(dataPath + 'naif0012.tls')  # leap second file

    # convert UTC string to datetime object
    datetime_object = datetime.strptime(ep1, '%Y %b %d %H:%M:%S.%f')

    # populate the epochMsg with the gregorian UTC date/time information
    epochMsg = simMessages.EpochSimMsg()
    epochMsg.year = datetime_object.year
    epochMsg.month = datetime_object.month
    epochMsg.day = datetime_object.day
    epochMsg.hours = datetime_object.hour
    epochMsg.minutes = datetime_object.minute
    epochMsg.seconds = datetime_object.second + datetime_object.microsecond / 1e6

    return epochMsg
