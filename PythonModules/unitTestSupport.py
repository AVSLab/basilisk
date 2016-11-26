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
#
#   Unit Test Support Script
#

import math
import os,errno
import numpy as np
import matplotlib as mpl
mpl.rc("figure", facecolor="white")
mpl.rc('xtick', labelsize=9)
mpl.rc('ytick', labelsize=9)
mpl.rc("figure", figsize=(5.75,2.5))
mpl.rc('axes', labelsize=10)
mpl.rc('legend', fontsize=9)
mpl.rc('figure', autolayout=True)

import matplotlib.colors as colors
import matplotlib.cm as cmx

import macros

import tabulate as T
del(T.LATEX_ESCAPE_RULES[u'$'])
del(T.LATEX_ESCAPE_RULES[u'\\'])
from tabulate import *



#
#   function to check if an array of values is the same as the truth values
#
def isArrayEqual(result, truth, dim, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    # the truth array is of dimesion dim, no time stamp
    if dim < 1:
        print "Incorrect array dimension " + dim + " sent to isArrayEqual"
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

    if foundNAN(result): return 0

    for i in range(0,dim):
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

    if foundNAN(result): return 0

    for i in range(0,dim):
        if (math.fabs(result[i+1]) > accuracy):
            return 0    # return 0 to indicate the array's are not equal

    return 1            # return 1 to indicate the two array's are equal

#
#   Compare two arrays size and values and check absolute accuracy
#
def compareArray(trueStates, dataStates, accuracy, msg, testFailCount, testMessages):
    if (len(trueStates) != len(dataStates)):
        testFailCount += 1
        testMessages.append("FAILED: "+msg+" unequal data array sizes\n")
    else:
        for i in range(0, len(trueStates)):
            # check a vector values
            if not isArrayEqual(dataStates[i], trueStates[i], 3, accuracy):
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

#
#   save a python scenario result into the Doxygen image folder
#
def saveScenarioFigure(figureName, plt, path):
    texFileName = path+"/../Images/Scenarios/"+figureName+".svg"
    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    plt.savefig(texFileName, transparent=True)

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
#   pick a nicer color pattern to plot 3 vector components
#

def getLineColor(idx,maxNum):
    values = range(0, maxNum+2)
    colorMap = mpl.pyplot.get_cmap('gist_earth')
    cNorm = colors.Normalize(vmin=0, vmax=values[-1])
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=colorMap)
    return scalarMap.to_rgba(values[idx])

def np2EigenVector3d(vec):
    return [[vec[0]],[vec[1]],[vec[2]]]

def np2EigenMatrixXd(mat):
    return [
        [mat[0], mat[1], mat[2]]
        ,[mat[3], mat[4], mat[5]]
        ,[mat[6], mat[7], mat[8]]
    ]
