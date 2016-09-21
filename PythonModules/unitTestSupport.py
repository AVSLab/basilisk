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
#
#   Unit Test Support Script
#

import math
import os,errno

import tabulate as T
del(T.LATEX_ESCAPE_RULES[u'$'])
del(T.LATEX_ESCAPE_RULES[u'\\'])
from tabulate import *

import matplotlib.pyplot as plt


#
#   function to check if an array of values is the same as the truth values
#
def isArrayEqual(result, truth, dim, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    # the truth array is of dimesion dim, no time stamp
    if dim < 1:
        print "Incorrect array dimension " + dim + " sent to isArrayEqual"
        return 0

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

    for i in range(0,dim):
        if (math.fabs(result[i+1]) > accuracy):
            return 0    # return 0 to indicate the array's are not equal

    return 1            # return 1 to indicate the two array's are equal


#
#   function to check if a double equals a truth value
#
def isDoubleEqual(result, truth, accuracy):
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
        plt.savefig(texFileName)

    return
