
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
# Purpose:  Tests an object-oriented Keplerian Orbit Object
# Author:   Scott Carnahan
# Creation Date:  Sept 10 2019
#

import pytest
import os, inspect
from Basilisk.architecture import BSpline
import numpy as np
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# X1 = np.array([0, 1, 2, 3, 4, 5, 6])
# X2 = np.array([5, 4, 3, 2, 1, 0, 1])
# X3 = np.array([0, 0, 0, 0, 0, 0, 0])

X1 = np.array([0, 1, 2, 3, 4, 5, 6])
X2 = np.array([5, 4, 3, 2, 1, 0, 1])
X3 = np.array([0, 0, 0, 0, 0, 0, 0])

for i in range(len(X1)):
    plt.plot(X1[i],X2[i],'r*')

Input = BSpline.InputDataSet(X1, X2, X3)
# Input.setXDot_0([0, 0, 0])
# Input.setXDot_N([0, 0, 0])
# Input.setXDDot_0([0, 0, 0])
# Input.setXDDot_N([0, 0, 0])
# Input.setT([0, 1, 2, 3])

Output = BSpline.OutputDataSet()
BSpline.interpolate(Input, 200, 1, 4, Output)

X1 = []
X2 = []
X3 = []
XD1 = []
XD2 = []
XD3 = []
XDD1 = []
XDD2 = []
XDD3 = []
I = len(Output.X1)
for i in range(I):
    X1.append(Output.X1[i][0])
    X2.append(Output.X2[i][0])
    X3.append(Output.X3[i][0])
    XD1.append(Output.XD1[i][0])
    XD2.append(Output.XD2[i][0])
    XD3.append(Output.XD3[i][0])
    XDD1.append(Output.XDD1[i][0])
    XDD2.append(Output.XDD2[i][0])
    XDD3.append(Output.XDD3[i][0])

plt.figure(1)
plt.plot(X1,X2)
plt.figure(2)
plt.plot(XD1)
plt.plot(XD2)
plt.figure(3)
plt.plot(XDD1)
plt.plot(XDD2)
plt.show()

# print(Input.X1)
# print(Output.X1)
# print(Output.X2)
# print(Input.XDot_0_flag)
# print(Input.XDot_N_flag)
# print(Input.T)