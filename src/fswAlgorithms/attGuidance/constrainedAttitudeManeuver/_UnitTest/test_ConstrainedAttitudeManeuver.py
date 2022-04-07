#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
#   Unit Test Script
#   Module Name:        constrainedAttitudeManeuver
#   Author:             Riccardo Calaon
#   Creation Date:      March 14, 2021
#


import os
import pytest
import numpy as np

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.fswAlgorithms import constrainedAttitudeManeuver
from Basilisk.utilities import macros
from Basilisk.architecture import bskLogging
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging

import matplotlib.pyplot as plt

path = os.path.dirname(os.path.abspath(__file__))
dataFileName = None

def shadowSetMap(sigma, switch):
    sigma = np.array(sigma)
    if switch:
        s2 = (sigma[0]**2 + sigma[1]**2 + sigma[2]**2)
        if not s2 == 0:
            return -sigma / s2
        else:
            return sigma
    else:
        return sigma

class constraint:
    def __init__(self, axis, color):
        self.axis =  axis / np.linalg.norm(axis)
        self.color = color
        
class node:
    def __init__(self, sigma_BN, constraints, **kwargs):
        self.sigma_BN = np.array(sigma_BN)
        s = np.linalg.norm(self.sigma_BN)
        if s > 1:
            self.sigma_BN = shadowSetMap(sigma_BN, True)  # mapping to shadow set if |sigma| > 1
        if np.abs(s-1) < 1e-5:
            s = 1.
        self.isBoundary = False
        self.s = s
        if s == 1:
            self.isBoundary = True
        self.isFree = True
        self.color = ''
        self.neighbors = {}
        self.heuristic = 0
        self.priority = 0
        self.path = []
        # check cosntraint compliance
        sigma_tilde = np.array([ [0,           -sigma_BN[2], sigma_BN[1]],
                                 [sigma_BN[2],      0,      -sigma_BN[0]],
                                 [-sigma_BN[1], sigma_BN[0],     0      ] ])
        BN = np.identity(3) + ( 8*np.matmul(sigma_tilde, sigma_tilde) -4*(1-s**2)*sigma_tilde ) / (1 + s**2)**2
        NB = BN.transpose()
        # checking for keepOut constraint violation
        if 'keepOut_b' in kwargs:
            for i in range(len(kwargs['keepOut_b'])):
                b_B = np.array(kwargs['keepOut_b'][i])
                b_N = np.matmul(NB, b_B)
                for c in constraints['keepOut']:
                    if np.dot(b_N, c.axis) >= np.cos(kwargs['keepOut_fov'][i]):
                        self.isFree = False
                        self.color = c.color
                        return
        # checking for keepIn constraint violation (at least one SS must see the Sun)
        if 'keepIn_b' in kwargs:
            b_N = []
            for i in range(len(kwargs['keepIn_b'])):
                b_B = np.array(kwargs['keepIn_b'][i])
                b_N.append(np.matmul(NB, b_B))
            isIn = False
            for c in constraints['keepIn']:
                for i in range(len(b_N)):
                    if np.dot(b_N[i], c.axis) >= np.cos(kwargs['keepIn_fov'][i]):
                        isIn = True
                        self.color = c.color
            if not isIn:
                self.isFree = False

def distanceCart(n1, n2):
    d1 = np.linalg.norm(n1.sigma_BN - n2.sigma_BN)
    sigma1norm2 = n1.sigma_BN[0]**2 + n1.sigma_BN[1]**2 + n1.sigma_BN[2]**2
    sigma2norm2 = n2.sigma_BN[0]**2 + n2.sigma_BN[1]**2 + n2.sigma_BN[2]**2
    if sigma2norm2 > 1e-8:
        sigma2_SS = shadowSetMap(n2.sigma_BN, True)
        d2 = np.linalg.norm(n1.sigma_BN - sigma2_SS)
    else:
        d2 = d1
    if sigma1norm2 > 1e-8:
        sigma1_SS = shadowSetMap(n1.sigma_BN, True)
        d3 = np.linalg.norm(sigma1_SS - n2.sigma_BN)
    else:
        d3 = d1
    if sigma1norm2 > 1e-8 and sigma2norm2 > 1e-8:
        d4 = np.linalg.norm(sigma1_SS - sigma2_SS)
    else:
        d4 = d1

    return min(d1, d2, d3, d4)

# PRA between two nodes
def distanceMRP(n1, n2):    
    s1 = n1.sigma_BN
    s2 = n2.sigma_BN
    sigma1norm2 = n1.sigma_BN[0]**2 + n1.sigma_BN[1]**2 + n1.sigma_BN[2]**2
    sigma2norm2 = n2.sigma_BN[0]**2 + n2.sigma_BN[1]**2 + n2.sigma_BN[2]**2

    D = 1 + (sigma1norm2*sigma2norm2)**2 + 2*np.dot(s1, s2)

    if abs(D) < 1e-5:
        s2 = shadowSetMap(s2, True)
        sigma2norm2 = 1 / sigma2norm2
        D = 1 + (sigma1norm2*sigma2norm2)**2 + 2*np.dot(s1, s2)

    s12 = ( (1-sigma2norm2)*s1 - (1-sigma1norm2)*s2 + 2*np.cross(s1, s2) ) / D
    sigma12norm2 = np.linalg.norm(s12)**2

    if sigma12norm2 > 1:
        s12 = shadowSetMap(s12, True)

    return 4*np.arctan(np.linalg.norm(s12))

def mirrorFunction(i, j, k):
    return [ [i, j, k], [-i, j, k], [i, -j, k], [i, j, -k], [-i, -j, k], [-i, j, -k], [i, -j, -k], [-i, -j, -k] ]

def neighboringNodes(i, j, k):
    return [ [i-1,  j,  k], [i+1,  j,  k], [i,  j-1,  k], [i,  j+1,  k], [i,  j,  k-1], [i,  j,  k+1],
             [i-1, j-1, k], [i+1, j-1, k], [i-1, j+1, k], [i+1, j+1, k], [i-1, j, k-1], [i+1, j, k-1],
             [i-1, j, k+1], [i+1, j, k+1], [i, j-1, k-1], [i, j+1, k-1], [i, j-1, k+1], [i, j+1, k+1],
             [i-1,j-1,k-1], [i+1,j-1,k-1], [i-1,j+1,k-1], [i-1,j-1,k+1], [i+1,j+1,k-1], [i+1,j-1,k+1], [i-1,j+1,k+1], [i+1,j+1,k+1] ]

def generateGrid(n_start, n_goal, N, constraints, data):
    
    u = np.linspace(0, 1, N, endpoint = True)
    nodes = {}

    # add internal nodes (|sigma| <= 1)
    for i in range(N):
        for j in range(N):
            for k in range(N):
                if (u[i]**2+u[j]**2+u[k]**2) <= 1:
                    for m in mirrorFunction(i, j, k):
                        if (m[0], m[1], m[2]) not in nodes:
                            nodes[(m[0], m[1], m[2])] = node([np.sign(m[0])*u[i], np.sign(m[1])*u[j], np.sign(m[2])*u[k]], constraints, **data)
    # add missing boundary nodes (|sigma| = 1)
    for i in range(N-1):
        for j in range(N-1):
            for k in range(N-1):
                if (i, j, k) in nodes:
                    if not nodes[(i, j, k)].isBoundary:
                        if (i+1, j, k) not in nodes:
                            for m in mirrorFunction(i+1, j, k):
                                if (m[0], m[1], m[2]) not in nodes:
                                    nodes[(m[0], m[1], m[2])] = node([np.sign(m[0])*(1-u[j]**2-u[k]**2)**0.5, np.sign(m[1])*u[j], np.sign(m[2])*u[k]], constraints, **data)
                        if (i, j+1, k) not in nodes:
                            for m in mirrorFunction(i, j+1, k):
                                if (m[0], m[1], m[2]) not in nodes:
                                    nodes[(m[0], m[1], m[2])] = node([np.sign(m[0])*u[i], np.sign(m[1])*(1-u[i]**2-u[k]**2)**0.5, np.sign(m[2])*u[k]], constraints, **data)
                        if (i, j, k+1) not in nodes:
                            for m in mirrorFunction(i, j, k+1):
                                if (m[0], m[1], m[2]) not in nodes:
                                    nodes[(m[0], m[1], m[2])] = node([np.sign(m[0])*u[i], np.sign(m[1])*u[j], np.sign(m[2])*(1-u[i]**2-u[j]**2)**0.5], constraints, **data)
    
    # # linking nodes
    for key1 in nodes:
        i = key1[0]
        j = key1[1]
        k = key1[2]
        # linking nodes to immediate neighbors
        for n in neighboringNodes(i, j, k):
            key2 = (n[0], n[1], n[2])
            if key2 in nodes:
                if nodes[key1].isFree and nodes[key2].isFree:
                    nodes[key1].neighbors[key2] = nodes[key2]
        # linking boundary nodes to neighbors of respective shadow sets
        if nodes[key1].isBoundary:
            if (-i, -j, -k) in nodes:
                for key2 in nodes[(-i, -j, -k)].neighbors:
                    if nodes[key1].isFree and nodes[key2].isFree and key2 not in nodes[key1].neighbors:
                        nodes[key1].neighbors[key2] = nodes[key2]

    # add start and goal nodes
    # looking for closest nodes to start and goal
    ds = 10
    dg = 10
    for key in nodes:
        if nodes[key].isFree:
            d1 = distanceCart(nodes[key], n_start)
            if abs(d1-ds) < 1e-6:
                if np.linalg.norm(nodes[key].sigma_BN) < np.linalg.norm(nodes[key_s].sigma_BN):
                    ds = d1
                    n_s = nodes[key]
                    key_s = key
            else:
                if d1 < ds:
                    ds = d1
                    n_s = nodes[key]
                    key_s = key
            d2 = distanceCart(nodes[key], n_goal)
            if abs(d2-dg) < 1e-6:
                if np.linalg.norm(nodes[key].sigma_BN) < np.linalg.norm(nodes[key_g].sigma_BN):
                    dg = d2
                    n_g = nodes[key]
                    key_g = key
            else:
                if d2 < dg:
                    dg = d2
                    n_g = nodes[key]
                    key_g = key
    for key in n_s.neighbors:
        n_start.neighbors[key] = n_s.neighbors[key]
    for key in n_g.neighbors:
        nodes[key].neighbors[key_g] = n_goal
    nodes[key_s] = n_start 
    nodes[key_g] = n_goal

    return nodes

# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.
@pytest.mark.parametrize("N", [6,7,8,9,10,11,12])
@pytest.mark.parametrize("keepOutFov", [20])
@pytest.mark.parametrize("keepInFov", [70])
@pytest.mark.parametrize("accuracy", [1e-12])

def test_constrainedAttitudeManeuver(show_plots, N, keepOutFov, keepInFov, accuracy):

    r"""Add documentation here"""

    # each test method requires a single assert method to be called
    [testResults, testMessage] = CAMTestFunction(N, keepOutFov, keepInFov, accuracy)

    assert testResults < 1, testMessage

def CAMTestFunction(N, keepOutFov, keepInFov, accuracy):

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    InertiaTensor = [0.02 / 3,  0.,         0.,
                     0.,        0.1256 / 3, 0.,
                     0.,        0.,         0.1256 / 3]
    PlanetInertialPosition = np.array([10, 0, 0])
    SCInertialPosition = np.array([1, 0, 0])
    SCInitialAttitude = np.array([0, 0, -0.5])
    SCTargetAttitude = np.array([0, 0.5, 0])
    SCInitialAngRate = np.array([0, 0, 0])
    SCTargetAngRate = np.array([0, 0, 0])
    keepOutBoresight_B = [[1, 0, 0]]
    keepInBoresight_B = [[0, 1, 0], [0, 0, 1]]
    # convert Fov angles to radiants
    keepOutFov = keepOutFov * macros.D2R
    keepInFov = keepInFov * macros.D2R

    constraints = {'keepOut' : [], 'keepIn' : []}
    constraints['keepOut'].append( constraint(PlanetInertialPosition-SCInertialPosition, 'r') )
    constraints['keepIn'].append( constraint(PlanetInertialPosition-SCInertialPosition, 'g') )
    data =  {'keepOut_b' : keepOutBoresight_B, 'keepOut_fov' : [keepOutFov], 
             'keepIn_b' : keepInBoresight_B, 'keepIn_fov' : [keepInFov, keepInFov]}
    n_start = node(SCInitialAttitude, constraints, **data)
    n_goal  = node(SCTargetAttitude, constraints, **data)
    Grid = generateGrid(n_start, n_goal, N, constraints, data)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    simulationTime = macros.min2nano(2)
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))
    
    testModule = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver(N)
    testModule.sigma_BN_goal = SCTargetAttitude
    testModule.omega_BN_B_goal = SCTargetAngRate
    testModule.avgOmega = 0.03
    testModule.BSplineType = 0
    testModule.appendKeepOutDirection(keepOutBoresight_B[0], keepOutFov)
    testModule.appendKeepInDirection(keepInBoresight_B[0], keepInFov)
    testModule.appendKeepInDirection(keepInBoresight_B[1], keepInFov)
    testModule.ModelTag = "testModule"
    unitTestSim.AddModelToTask(unitTaskName, testModule)
	
    # connect messages
    SCStatesMsgData = messaging.SCStatesMsgPayload()
    SCStatesMsgData.r_BN_N = SCInertialPosition
    SCStatesMsgData.sigma_BN = SCInitialAttitude
    SCStatesMsgData.omega_BN_B = SCInitialAngRate
    SCStatesMsg = messaging.SCStatesMsg().write(SCStatesMsgData)
    VehicleConfigMsgData = messaging.VehicleConfigMsgPayload()
    VehicleConfigMsgData.ISCPntB_B = InertiaTensor
    VehicleConfigMsg = messaging.VehicleConfigMsg().write(VehicleConfigMsgData)
    PlanetStateMsgData = messaging.SpicePlanetStateMsgPayload()
    PlanetStateMsgData.PositionVector = PlanetInertialPosition
    PlanetStateMsg = messaging.SpicePlanetStateMsg().write(PlanetStateMsgData)
    testModule.scStateInMsg.subscribeTo(SCStatesMsg)
    testModule.vehicleConfigInMsg.subscribeTo(VehicleConfigMsg)
    testModule.keepOutCelBodyInMsg.subscribeTo(PlanetStateMsg)
    testModule.keepInCelBodyInMsg.subscribeTo(PlanetStateMsg)

    numDataPoints = 200
    samplingTime = unitTestSupport.samplingTime(simulationTime, testProcessRate, numDataPoints)

    CAMLog = testModule.attRefOutMsg.recorder(samplingTime)
    unitTestSim.AddModelToTask(unitTaskName, CAMLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(simulationTime)        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # checking correctness of grid points:
    for i in range(-N,N+1):
        for j in range(-N,N+1):
            for k in range(-N,N+1):
                if (i, j, k) in Grid:
                    sigma_BN = Grid[(i, j, k)].sigma_BN
                    sigma_BN_BSK = []
                    for p in range(3):
                        sigma_BN_BSK.append( testModule.returnNodeCoord([i, j, k], p) )
                    if not unitTestSupport.isVectorEqual(sigma_BN, sigma_BN_BSK, accuracy):
                        testFailCount += 1
                        testMessages.append("FAILED: " + testModule.ModelTag + " Error in the coordinates of node ({},{},{}) \n".format(i, j, k))
                    if not Grid[(i, j, k)].isFree == testModule.returnNodeState([i, j, k]):
                        testFailCount += 1
                        testMessages.append("FAILED: " + testModule.ModelTag + " Error in the state of node ({},{},{}) \n".format(i, j, k))

    # timeData = CAMLog.times() * macros.NANO2SEC
    # dataSigmaRN = CAMLog.sigma_RN
    # dataOmegaRN = CAMLog.omega_RN_N
    # dataOmegaDotRN = CAMLog.domega_RN_N

    # plot_attitude_reference(timeData, dataSigmaRN)
    # plot_rate_reference(timeData, dataOmegaRN)
    # plot_acc_reference(timeData, dataOmegaDotRN)

    # plt.show()

    return [testFailCount, ''.join(testMessages)]
	   

def plot_attitude_reference(timeData, dataSigmaRN):
    """Plot the reference attitude."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataSigmaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Reference Attitude $\sigma_{R/N}$')

def plot_rate_reference(timeData, dataOmegaRN):
    """Plot the reference angular rate."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataOmegaRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{RN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('Reference Angular Rate (rad/s) ')

def plot_acc_reference(timeData, dataOmegaDotRN):
    """Plot the reference angular acceleration."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, dataOmegaDotRN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\dot{\omega}_{RN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel('Reference Angular Acceleration (rad/s^2) ')



#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    CAMTestFunction(
        7,       # grid coarsness N
        20,      # keepOutFov
        70,      # keepInFov
        1e-12    # accuracy
        )