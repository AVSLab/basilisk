
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

import pytest
import matplotlib.pyplot as plt
import os, inspect
from Basilisk.architecture import BSpline
import numpy as np


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.
@pytest.mark.parametrize("P", [5, 6])
@pytest.mark.parameterize("n",[3]) # Add parameter variation for n
@pytest.mark.parametrize("XDot_flag", [False, True])
@pytest.mark.parametrize("XDDot_flag", [False, True])
@pytest.mark.parametrize("accuracy", [1e-6])

def test_BSpline(show_plots,P,n,XDot_flag,XDDot_flag,accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the capability of the BSpline function to correctly interpolate 
    a series of points in 3 dimensions.
    The coordinates of these 7 points are stored in 3 numpy arrays:

    X1 = np.array([0, 1, 2, 3, 4, 5, 6])

    X2 = np.array([5, 4, 3, 2, 1, 0, 1])

    X3 = np.array([3, 2, 1, 2, 3, 4, 5]).

    The input arrays are initialized through ``Input = BSpline.InputDataSet(X1, X2, X3)``. 
    The time tags at which each waypoint is to be hit are provided through ``Input.setT([0, 2, 3, 5, 7, 8, 10])``. 
    Alternatively, it is possible to specify the average velocity norm through ``Input.setAvgXDot()``.
    The endpoint derivatives are specified through the methods:

    - ``Input.setXDot_0()`` for starting point first-order derivative;
    - ``Input.setXDot_N()`` for last point first-order derivative;
    - ``Input.setXDDot_0()`` for starting point second-order derivative;
    - ``Input.setXDDot_N()`` for last point second-order derivative.

    Each method to specify the derivatives takes in a 3-dimensional numpy array.
    The output data structure is created with ``Output = BSpline.OutputDataSet()``.
    The interpolation happens calling the method ``BSpline.interpolate(Input, N, P, Output)`` where:

    - N is the desired number of equally spaced data points in the interpolated function;
    
    - P is the polynomial order of the B-Spline function. The order should be at least 3 when first-order derivatives are specified, 
      and 5 when second-order derivatives are specified. The maximum oder is P = n + k - 1, with n being the number of waypoints and k
      being the number of endpoint derivatives that are being specified.

    **Test Parameters**

    As this is a parameterized unit test, note that the test case parameters values are shown automatically in the
    pytest HTML report.  This sample script has the parameters param1 and param 2.  Provide a description of what
    each parameter controls.  This is a convenient location to include the accuracy variable used in the
    validation test.

    Args:
        P (int): polynomial order of the B-Spline curve;
        XDot_flag (bool) : whether the first-order end point derivatives should be specified;
        XDDot_flag (bool) : whether the second-order end point derivatives should be specified;
        n: Number of control points;
        accuracy (float): absolute accuracy value used in the validation tests;

        
    **Description of Variables Being Tested**

    This unit test checks the correctness of the interpolated function: 
    - a check is performed on whether or not each waypoint is hit at the specified time;
    - when the derivatives are specified, it checks whether the starting point derivative actually matches the input derivative.
    """
    
    # each test method requires a single assert method to be called
    [testResults, testMessage] = BSplineTestFunction(P,n,XDot_flag,XDDot_flag,accuracy)
    assert testResults < 1, testMessage



def BSplineTestFunction(P,n,XDot_flag, XDDot_flag,accuracy):
    
    
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    
    # Set Attitudes
    X1 = np.array([0, 1, 2, 3, 4, 5, 6])
    X2 = np.array([5, 4, 3, 2, 1, 0, 1])
    X3 = np.array([3, 2, 1, 2, 3, 4, 5])
    
    Input = BSpline.InputDataSet(X1, X2, X3)
    Output = BSpline.OutputDataSet()

    if XDot_flag:
        Input.setXDot_0([0.2, 0, 0])
        Input.setXDot_N([0, 0, 0])
    if XDDot_flag:
        Input.setXDDot_0([0, 0, 0])
        Input.setXDDot_N([0.2, 0, 0])
    
    print("Enter which case you would like to test.")
    
    print("Case A: input time provided, No LSpline xDot approximation (red)")
    
    print("Case B: No input time provided, No LSpline xDot approximation (yellow)")
    
    print("Case C: input time provided, LSpline xDot approximation (black)")
    
    print("Case D: No input time provided, LSpline xDot approximation (green)")
    
    case_input = input("Enter Case ")
    
    match case_input:
    
        case "A":
            print("Case A")
            Input.setT([0, 2, 3, 5, 7, 8, 10])
            BSpline.approximate(Input,101,n,P,Output)
            
            # Obtain End Indices of Input and Output Structure Time Stamp
            i = len(Output.T)-1
            j = len(Input.T)-1
            
            
            # Check the accuracy of Start and End Attitudes
            if abs(Output.T[0][0] - Input.T[0][0]) < accuracy:
            
                # Start Attitude
                if not abs(Output.X1[0][0] - X1[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Input.T[0][0]))
                if not abs(Output.X2[0][0] - X2[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Input.T[0][0]))
                if not abs(Output.X3[0][0] - X3[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Input.T[0][0]))
                    
                # End Attitude
                if not abs(Output.X1[i][0] - X1[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X2[i][0] - X2[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X3[i][0] - X3[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Input.T[j][0]))

            # Check the accuracy of Start and End First Derivatives
            if XDot_flag:
                if not ((abs(Output.XD1[0][0]-Input.XDot_0[0][0]) < accuracy) and
                        (abs(Output.XD2[0][0]-Input.XDot_0[1][0]) < accuracy) and
                        (abs(Output.XD3[0][0]-Input.XDot_0[2][0]) < accuracy)):
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at starting point".format(P))
                    
            # Check the accuracy of End First Derivatives
                if not ((abs(Output.XD1[i][0]-Input.XDot_N[0][0]) < accuracy) and
                        (abs(Output.XD2[i][0]-Input.XDot_N[1][0]) < accuracy) and
                        (abs(Output.XD3[i][0]-Input.XDot_N[2][0]) < accuracy)):
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at end point".format(P))

            
            # Plotting Attitudes Code:
            fig, axs = plt.subplots(3)
            axs[0].scatter(Input.T,X1,c = 'b')
            axs[0].plot(Output.T,Output.X1,c = 'r')
            fig.suptitle("Attitudes vs Time")
            axs[0].set_xlabel("Time [s]")
            axs[0].set_ylabel("X1 Attitude")
            axs[0].legend(["Way Points","LS Approximation"])
            
            axs[1].scatter(Input.T,X2,c = 'b')
            axs[1].plot(Output.T,Output.X2,c = 'r')
            axs[1].set_xlabel("Time [s]")
            axs[1].set_ylabel("X2 Attitude")
            axs[1].legend(["Way Points","LS Approximation"])
            
            axs[2].scatter(Input.T,X3,c = 'b')
            axs[2].plot(Output.T,Output.X3,c = 'r')
            axs[2].set_xlabel("Time [s]")
            axs[2].set_ylabel("X3 Attitude")
            axs[2].legend(["Way Points","LS Approximation"])
            fig.tight_layout()
            plt.show()
        case "B":
            print("Case B")
            BSpline.approximate(Input,101,n,P,Output)
            Input.setAvgXDot(0.3)
            # Obtain End Indices of Input and Output Structure Time Stamp
            i = len(Output.T)-1
            j = len(Output.T_way_calc)-1
        # Check the accuracy of Start and End Attitudes
            if abs(Output.T_way_calc[0][0] - Output.T[0][0]) < accuracy:
            
                # Start Attitude
                if not abs(Output.X1[0][0] - X1[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Output.T_way_calc[0][0]))
                if not abs(Output.X2[0][0] - X2[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Output.T_way_calc[0][0]))
                if not abs(Output.X3[0][0] - X3[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Output.T_way_calc[0][0]))
                
                # End Attitude
                if not abs(Output.X1[i][0] - X1[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Output.T_way_calc[j][0]))
                if not abs(Output.X2[i][0] - X2[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Output.T_way_calc[j][0]))
                if not abs(Output.X3[i][0] - X3[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Output.T_way_calc[j][0]))

            # Check the accuracy of Start and End First Derivatives
            if XDot_flag:
                if not ((abs(Output.XD1[0][0]-Input.XDot_0[0][0]) < accuracy) and
                        (abs(Output.XD2[0][0]-Input.XDot_0[1][0]) < accuracy) and
                        (abs(Output.XD3[0][0]-Input.XDot_0[2][0]) < accuracy)):
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at starting point".format(P))
                    
            # Check the accuracy of End First Derivatives
                if not ((abs(Output.XD1[i][0]-Input.XDot_N[0][0]) < accuracy) and
                        (abs(Output.XD2[i][0]-Input.XDot_N[1][0]) < accuracy) and
                        (abs(Output.XD3[i][0]-Input.XDot_N[2][0]) < accuracy)):
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at end point".format(P))
            
            
            # The outputs are successful, need to figure out how to plot the answer.
            #print(len(Output.T_way_calc))
            
            # Plotting Attitudes Code:
            fig, axs = plt.subplots(3)
            axs[0].scatter(Output.T_way_calc,X1,c = 'b')
            axs[0].plot(Output.T,Output.X1,c = 'y')
            fig.suptitle("Attitudes vs Time")
            axs[0].set_xlabel("Time [s]")
            axs[0].set_ylabel("X1 Attitude")
            axs[0].legend(["Way Points","LS Approximation"])
            
            axs[1].scatter(Output.T_way_calc,X2,c = 'b')
            axs[1].plot(Output.T,Output.X2,c = 'y')
            axs[1].set_xlabel("Time [s]")
            axs[1].set_ylabel("X2 Attitude")
            axs[1].legend(["Way Points","LS Approximation"])
            
            axs[2].scatter(Output.T_way_calc,X3,c = 'b')
            axs[2].plot(Output.T,Output.X3,c = 'y')
            axs[2].set_xlabel("Time [s]")
            axs[2].set_ylabel("X3 Attitude")
            axs[2].legend(["Way Points","LS Approximation"])
            fig.tight_layout()
            plt.show()
        case "C":
            print("Case C")
            Input.setLS_Dot()
            Input.setAvgXDot(0.3)
            print(Input.AvgXDot)
            X1Dot_des = np.array([1, 3, 0, 2, 1, 3, 1])/Input.AvgXDot
            X2Dot_des = np.array([2, 0, 3, 1, 2, 0, 2])/Input.AvgXDot
            X3Dot_des = np.array([2, 0, 0, 2, 2, 0, 2])/Input.AvgXDot
            Input.setXDot_des(X1Dot_des,X2Dot_des,X3Dot_des)
            Input.setT([0, 2, 3, 5, 7, 8, 10])
            BSpline.approximate(Input,101,n,P,Output)
            
            # Obtain End Indices of Input and Output Structure Time Stamp
            i = len(Output.T)-1
            j = len(Input.T)-1
            # Check the accuracy of Start and End Attitudes
            if abs(Output.T[0][0] - Input.T[0][0]) < accuracy:
            
                # Start Attitude
                if not abs(Output.X1[0][0] - X1[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Input.T[0][0]))
                if not abs(Output.X2[0][0] - X2[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Input.T[0][0]))
                if not abs(Output.X3[0][0] - X3[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Input.T[0][0]))
                    
                # End Attitude
                if not abs(Output.X1[i][0] - X1[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X2[i][0] - X2[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X3[i][0] - X3[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Input.T[j][0]))

            # Check the accuracy of Start and End First Derivatives
            if XDot_flag:
                if not ((abs(Output.XD1[0][0]-Input.XDot_0[0][0]) < accuracy) and
                        (abs(Output.XD2[0][0]-Input.XDot_0[1][0]) < accuracy) and
                        (abs(Output.XD3[0][0]-Input.XDot_0[2][0]) < accuracy)):
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at starting point".format(P))
                    
            # Check the accuracy of End First Derivatives
                if not ((abs(Output.XD1[i][0]-Input.XDot_N[0][0]) < accuracy) and
                        (abs(Output.XD2[i][0]-Input.XDot_N[1][0]) < accuracy) and
                        (abs(Output.XD3[i][0]-Input.XDot_N[2][0]) < accuracy)):
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at end point".format(P))

            
            # Plotting Attitudes Code:
            fig, axs = plt.subplots(3)
            axs[0].scatter(Input.T,X1,c = 'b')
            axs[0].plot(Output.T,Output.X1,c = 'r')
            fig.suptitle("Attitudes vs Time")
            axs[0].set_xlabel("Time [s]")
            axs[0].set_ylabel("X1 Attitude")
            axs[0].legend(["Way Points","LS Approximation"])
            
            axs[1].scatter(Input.T,X2,c = 'b')
            axs[1].plot(Output.T,Output.X2,c = 'r')
            axs[1].set_xlabel("Time [s]")
            axs[1].set_ylabel("X2 Attitude")
            axs[1].legend(["Way Points","LS Approximation"])
            
            axs[2].scatter(Input.T,X3,c = 'b')
            axs[2].plot(Output.T,Output.X3,c = 'r')
            axs[2].set_xlabel("Time [s]")
            axs[2].set_ylabel("X3 Attitude")
            axs[2].legend(["Way Points","LS Approximation"])
            fig.tight_layout()
            plt.show()
        case "D":
            print("Case D")
            Input.setLS_Dot()
            Input.setAvgXDot(0.3)
            print(Input.AvgXDot)
            X1Dot_des = np.array([1, 3, 0, 2, 1, 3, 1])/Input.AvgXDot
            X2Dot_des = np.array([2, 0, 3, 1, 2, 0, 2])/Input.AvgXDot
            X3Dot_des = np.array([2, 0, 0, 2, 2, 0, 2])/Input.AvgXDot
            Input.setXDot_des(X1Dot_des,X2Dot_des,X3Dot_des)
            
            BSpline.approximate(Input,101,n,P,Output) # Change 1: Test approximate function first
        
            # Obtain End Indices of Input and Output Structure Time Stamp
            i = len(Output.T)-1
            j = len(Input.T)-1
            
            
            # Check the accuracy of Start and End Attitudes
            if abs(Output.T[0][0] - Output.T_way_calc[0][0]) < accuracy:
            
                # Start Attitude
                if not abs(Output.X1[0][0] - X1[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Input.T[0][0]))
                if not abs(Output.X2[0][0] - X2[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Input.T[0][0]))
                if not abs(Output.X3[0][0] - X3[0]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Input.T[0][0]))
                    
                # End Attitude
                if not abs(Output.X1[i][0] - X1[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #1 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X2[i][0] - X2[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #2 check at time t = {}".format(P,Input.T[j][0]))
                if not abs(Output.X3[i][0] - X3[j]) < accuracy:
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed coordinate #3 check at time t = {}".format(P,Input.T[j][0]))

            # Check the accuracy of Start and End First Derivatives
            if XDot_flag:
                if not ((abs(Output.XD1[0][0]-Input.XDot_0[0][0]) < accuracy) and
                        (abs(Output.XD2[0][0]-Input.XDot_0[1][0]) < accuracy) and
                        (abs(Output.XD3[0][0]-Input.XDot_0[2][0]) < accuracy)):
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at starting point".format(P))
                    
            # Check the accuracy of End First Derivatives
                if not ((abs(Output.XD1[i][0]-Input.XDot_N[0][0]) < accuracy) and
                        (abs(Output.XD2[i][0]-Input.XDot_N[1][0]) < accuracy) and
                        (abs(Output.XD3[i][0]-Input.XDot_N[2][0]) < accuracy)):
                    testFailCount += 1
                    testMessages.append("FAILED: BSpline." + " Function of order {} failed first derivative at end point".format(P))
            
            #print("The length of the attitude output vector is",len(Output.X1))
            #print("The length of the attitude output vector is",len(Output.T))
            
            # Plotting Attitudes Code:
            fig, axs = plt.subplots(3)
            axs[0].scatter(Output.T_way_calc,X1,c = 'b')
            axs[0].plot(Output.T,Output.X1,c = 'g')
            fig.suptitle("Attitudes vs Time")
            axs[0].set_xlabel("Time [s]")
            axs[0].set_ylabel("X1 Attitude")
            axs[0].legend(["Way Points","LS Approximation"])
            
            axs[1].scatter(Output.T_way_calc,X2,c = 'b')
            axs[1].plot(Output.T,Output.X2,c = 'g')
            axs[1].set_xlabel("Time [s]")
            axs[1].set_ylabel("X2 Attitude")
            axs[1].legend(["Way Points","LS Approximation"])
            
            axs[2].scatter(Output.T_way_calc,X3,c = 'b')
            axs[2].plot(Output.T,Output.X3,c = 'g')
            axs[2].set_xlabel("Time [s]")
            axs[2].set_ylabel("X3 Attitude")
            axs[2].legend(["Way Points","LS Approximation"])
            fig.tight_layout()
            plt.show()
    return


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    BSplineTestFunction(
        4,        # polynomial order
        6,       # control points
        True,    # XDot_flag
        False,    # XDDot_flag
        1e-6)     
