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


#
#   Unit Test Support Script
#

import math
import os,errno
import platform
import numpy as np
import matplotlib as mpl
mpl.rc("figure", facecolor="white")
mpl.rc('xtick', labelsize=9)
mpl.rc('ytick', labelsize=9)
mpl.rc("figure", figsize=(5.75,2.5))
mpl.rc('axes', labelsize=10)
mpl.rc('legend', fontsize=9)
mpl.rc('figure', autolayout=True)

if platform.system() == 'Darwin':
    mpl.use('MacOSX')



import matplotlib.colors as colors
import matplotlib.cm as cmx

import macros

# import Viz messaging related modules
from Basilisk.simulation import vis_message_interface
from Basilisk.simulation import vis_clock_synch
from Basilisk.simulation import spice_interface

import tabulate as T
del(T.LATEX_ESCAPE_RULES[u'$'])
del(T.LATEX_ESCAPE_RULES[u'\\'])
del(T.LATEX_ESCAPE_RULES[u'_'])
del(T.LATEX_ESCAPE_RULES[u'{'])
del(T.LATEX_ESCAPE_RULES[u'}'])
from tabulate import *



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
        print "Incorrect array dimension " + dim + " sent to isArrayEqual"
        return 0

    if len(result)==0:
        print "Result array was empty"
        return 0

    if len(truth)==0:
        print "Truth array was empty"
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
        print "Incorrect array dimension " + dim + " sent to isArrayEqual"
        return 0

    if len(result)==0:
        print "Result array was empty"
        return 0

    if len(truth)==0:
        print "Truth array was empty"
        return 0

    if foundNAN(result): return 0

    for i in range(0,dim):
        if truth[i] == 0:
            if result[i+1] == 0:
                continue
            else:
                print "Truth array contains zero"
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
        print "Incorrect array dimension " + dim + " sent to isArrayEqual"
        return 0

    if len(result)==0:
        print "Result array was empty"
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
        testMessages.append("FAILED: "+msg+" unequal data array sizes\n")
    else:
        if not isVectorEqual(dataStates, trueStates, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: "+msg+"\n")
    return testFailCount, testMessages


#
#   Compare two arrays size and values and check absolute accuracy
#
def compareArray(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + " data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqual(dataStates[i], trueStates[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+"sec\n")
    return testFailCount, testMessages

#
#   Compare two arrays of size N for size and values and check absolute accuracy
#
def compareArrayND(trueStates, dataStates, accuracy, msg, size, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + " data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqual(dataStates[i], trueStates[i], size, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+"sec\n")
    return testFailCount, testMessages


#
#   Compare two arrays size and values and check relative accuracy
#
def compareArrayRelative(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + " data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqualRelative(dataStates[i], trueStates[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+"sec\n")
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
        print "truth is zero, cannot compare"
        return 0

    # the result array is of dimension dim+1, as the first entry is the time stamp
    if (math.fabs((truth - result)/truth) > accuracy):
        return 0    # return 0 to indicate the doubles are not equal

    return 1        # return 1 to indicate the doubles are equal

#
#   Compare two arrays of doubles for size and values and check absolute accuracy
#
def compareDoubleArray(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+" unequal data array sizes\n")
    elif (len(trueStates) == 0 or len(dataStates) == 0):
        testFailCount += 1
        testMessages.append("FAILED: " + msg + " data had empty arrays\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isDoubleEqual(dataStates[i], trueStates[i], accuracy):
                testFailCount += 1
                testMessages.append("FAILED: "+msg+" at t="+str(dataStates[i, 0]*macros.NANO2SEC)+"sec\n")
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

        texTable.write('\\begin{table}[htbp]\n')
        texTable.write('\caption{' + caption + '}\n')
        texTable.write('\label{tbl:' + tableName + '}\n')
        texTable.write('\centering\n')
        texTable.write(table)
        texTable.write('\end{table}')
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
        texFigure.write('\\begin{figure}[htbp]\n')
        texFigure.write('\centerline{\n')
        texFigure.write('\includegraphics['+ format +']{AutoTeX/' + figureName + '}}\n')
        texFigure.write('\caption{' + caption + '}\n')
        texFigure.write('\label{fig:'+ figureName +'}\n')
        texFigure.write('\end{figure}')
        texFigure.close()

        texFileName = path + "/../_Documentation/AutoTeX/" + figureName + ".pdf"
        plt.savefig(texFileName, transparent=True)

    return

def foundNAN(array):
    if (np.isnan(np.sum(array))):
        print "Warning: found NaN value."
        return 1        # return 1 to indicate a NaN value was found
    return 0

#
#   macro to create and write a general message
#
def setMessage(simObject, processName, msgName, inputMessageData):
    inputMessageSize = inputMessageData.getStructSize()
    simObject.CreateNewMessage(processName,
                               msgName,
                               inputMessageSize,
                               2)
    simObject.WriteMessageData(msgName,
                               inputMessageSize,
                               0,
                               inputMessageData)


#
#   pick a nicer color pattern to plot 3 vector components
#

def getLineColor(idx,maxNum):
    values = range(0, maxNum+2)
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

def enableVisualization(scSim, dynProcess, processName, bodyName = 'earth'):
    VizTaskName = "VizTask"
    dynProcess.addTask(scSim.CreateNewTask(VizTaskName, macros.sec2nano(0.5)))
    viz = vis_message_interface.VisMessageInterface()
    scSim.AddModelToTask(VizTaskName, viz)
    clockSynch = vis_clock_synch.VisClockSynch()
    scSim.AddModelToTask(VizTaskName, clockSynch, None, 101)

    ephemData = spice_interface.SpicePlanetStateSimMsg()
    ephemData.J2000Current = 0.0
    ephemData.PositionVector = [0.0, 0.0, 0.0]
    ephemData.VelocityVector = [0.0, 0.0, 0.0]
    ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    ephemData.PlanetName = bodyName
    msgName = "central_body_spice"
    messageSize = ephemData.getStructSize()
    scSim.TotalSim.CreateNewMessage(processName, msgName, messageSize, 2, "SpicePlanetStateSimMsg")
    scSim.TotalSim.WriteMessageData(msgName, messageSize, 0, ephemData)

    return
