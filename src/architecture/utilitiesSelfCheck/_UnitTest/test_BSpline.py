
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


#
# BSpline Unit Test
#
# Purpose:  Tests the BSpline interpolating function
# Author:   Riccardo Calaon
# Creation Date:  Oct 10 2021
#

import pytest
import os, inspect
from Basilisk.architecture import BSpline
from Basilisk.utilities import unitTestSupport
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.
@pytest.mark.parametrize("P", [3, 4, 5, 6])
@pytest.mark.parametrize("XDot_0_flag", [False, True])
@pytest.mark.parametrize("XDot_N_flag", [False, True])
@pytest.mark.parametrize("XDDot_0_flag", [False, True])
@pytest.mark.parametrize("XDDot_N_flag", [False, True])
@pytest.mark.parametrize("accuracy", [1e-6])

def test_BSpline(show_plots, P, XDot_0_flag, XDot_N_flag, XDDot_0_flag, XDDot_N_flag, accuracy):
    
    # each test method requires a single assert method to be called
    [testResults, testMessage] = BSplineTestFunction(P, XDot_0_flag, XDot_N_flag, XDDot_0_flag, XDDot_N_flag, accuracy)
    assert testResults < 1, testMessage


def BSplineTestFunction(P, XDot_0_flag, XDot_N_flag, XDDot_0_flag, XDDot_N_flag, accuracy):

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    X1 = np.array([0, 1, 2, 3, 4, 5, 6])
    X2 = np.array([5, 4, 3, 2, 1, 0, 1])
    X3 = np.array([3, 2, 1, 2, 3, 4, 5])

    Input = BSpline.InputDataSet(X1, X2, X3)
    Input.setT([0, 2, 3, 5, 7, 8, 10])
    if XDot_0_flag:
        Input.setXDot_0([0, 0, 0])
    if XDot_N_flag:
        Input.setXDot_N([0, 0, 0])
    if XDDot_0_flag:
        Input.setXDDot_0([0, 0, 0])
    if XDDot_N_flag:
        Input.setXDDot_N([0.2, 0, 0])

    Output = BSpline.OutputDataSet()
    BSpline.interpolate(Input, 101, 1, P, Output)

    for i in range(len(Output.T)):
        for j in range(len(Input.T)):
            if abs(Output.T[i][0] - Input.T[j][0]) < accuracy:
                if not abs(Output.X1[i][0] - X1[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X2[i][0] - X2[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X3[i][0] - X3[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Input.T[j][0]))
    if XDot_0_flag:
        if not ((abs(Output.XD1[0][0]-Input.XDot_0[0][0]) < accuracy) and 
                (abs(Output.XD2[0][0]-Input.XDot_0[1][0]) < accuracy) and 
                (abs(Output.XD3[0][0]-Input.XDot_0[2][0]) < accuracy)):
            testFailCount += 1
            testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at starting point".format(P))
    if XDDot_0_flag:
        if not ((abs(Output.XDD1[0][0]-Input.XDDot_0[0][0]) < accuracy) and 
                (abs(Output.XDD2[0][0]-Input.XDDot_0[1][0]) < accuracy) and 
                (abs(Output.XDD3[0][0]-Input.XDDot_0[2][0]) < accuracy)):
            testFailCount += 1
            testMessages.append("FAILED: BSpline." + " Function of order {} failed second derivative at starting point".format(P))

    print(testFailCount, testMessages)


    return


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    BSplineTestFunction(
        1,       # polynomial order 
        False,    # XDot_0_flag
        False,    # XDot_N_flag
        False,    # XDDot_0_flag
        False,    # XDDot_N_flag
        1e-6)     

# X1 = []
# X2 = []
# X3 = []
# XD1 = []
# XD2 = []
# XD3 = []
# XDD1 = []
# XDD2 = []
# XDD3 = []
# I = len(Output.X1)
# for i in range(I):
#     X1.append(Output.X1[i][0])
#     X2.append(Output.X2[i][0])
#     X3.append(Output.X3[i][0])
#     XD1.append(Output.XD1[i][0])
#     XD2.append(Output.XD2[i][0])
#     XD3.append(Output.XD3[i][0])
#     XDD1.append(Output.XDD1[i][0])
#     XDD2.append(Output.XDD2[i][0])
#     XDD3.append(Output.XDD3[i][0])

# plt.figure(1)
# plt.plot(X1,X2,X3)
# plt.figure(2)
# plt.plot(Output.T,X1)
# plt.plot(Output.T,X2)
# plt.figure(3)
# plt.plot(Output.T,XD1)
# plt.plot(Output.T,XD2)
# plt.figure(4)
# plt.plot(Output.T,XDD1)
# plt.plot(Output.T,XDD2)
# plt.grid()
# plt.show()

# print(Input.X1)
# print(Output.X1)
# print(Output.X2)
# print(Input.XDot_0_flag)
# print(Input.XDot_N_flag)
# print(Input.T)