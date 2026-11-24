#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Creation Date:      April 16, 2022
#


import gc
import os
import sys
import weakref
from types import SimpleNamespace

import numpy as np
import pytest
from Basilisk.architecture import BSpline
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import constrainedAttitudeManeuver
from Basilisk.utilities import RigidBodyKinematics as rbk
# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simHelpers

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
        self.backpointer = self
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

    # link nodes
    for key1 in nodes:
        i = key1[0]
        j = key1[1]
        k = key1[2]
        # link nodes to immediate neighbors
        for n in neighboringNodes(i, j, k):
            key2 = (n[0], n[1], n[2])
            if key2 in nodes:
                if nodes[key1].isFree and nodes[key2].isFree:
                    nodes[key1].neighbors[key2] = nodes[key2]
        # link boundary nodes to neighbors of respective shadow sets
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


def backtrack(n, n_start):
    if n == n_start:
        path = [n]
        return path
    else:
        path = backtrack(n.backpointer, n_start)
        path.append(n)
        return path


def pathHandle(path, avgOmega):

    T = [0]
    S = 0
    for n in range(len(path)-1):
        T.append(T[n] + distanceCart(path[n], path[n+1]))
        S += T[n+1] - T[n]

    raw_waypoints = np.array([point.sigma_BN for point in path])
    quaternions = np.array([rbk.MRP2EP(sigma) for sigma in raw_waypoints])
    # Score the original MRP norms so quaternion projection cannot break a peak tie.
    norms = np.linalg.norm(raw_waypoints, axis=1)
    shadow_norms = np.divide(1.0, norms, where=norms != 0, out=np.full_like(norms, np.inf))
    tolerance = 4 * np.finfo(float).eps
    def arc_peak(start, end):
        # Parameterize the great circle as start*cos(t) + tangent*sin(t).
        dot = np.dot(start, end)
        tangent = end - dot * start
        sine = np.linalg.norm(tangent)
        if sine > tolerance:
            tangent /= sine
            minimum = (np.arctan2(tangent[0], start[0]) + np.pi) % (2 * np.pi)
            if 0 < minimum < np.arctan2(sine, dot):
                extreme = np.cos(minimum) * start + np.sin(minimum) * tangent
                radius = np.linalg.norm(extreme[1:])
                return (1 - extreme[0]) / radius if radius > tolerance else np.inf
        return 0.0

    # Enumerate the two directions only at a half-turn. Visit the principal start first on a tie.
    first = quaternions[0]
    start_sign = first[0] if abs(first[0]) > tolerance else first[np.argmax(np.abs(first))]
    candidates = [([sign], norms[0] if sign == 1 else shadow_norms[0])
                  for sign in (np.sign(start_sign), -np.sign(start_sign))]
    for i in range(1, len(path)):
        candidates = [(signs + [sign], max(peak, norm, arc_peak(signs[-1] * quaternions[i-1], sign * quaternions[i])))
                      for signs, peak in candidates
                      for sign, norm in ((1, norms[i]), (-1, shadow_norms[i]))
                      if np.dot(signs[-1] * quaternions[i-1], sign * quaternions[i]) >= -tolerance]
    best = min(peak for _, peak in candidates)
    if np.isinf(best):
        return None
    signs = next(signs for signs, peak in candidates if peak <= best * (1 + tolerance))
    quaternions *= np.array(signs)[:, None]

    # For unit quaternions, ||q + identity||² = 2(1 + q0), even near the shadow pole.
    denominator = np.sum((quaternions + [1, 0, 0, 0])**2, axis=1, keepdims=True)
    waypoints = np.divide(2 * quaternions[:, 1:], denominator, where=denominator != 0,
                          out=np.full(raw_waypoints.shape, np.inf))

    Input = BSpline.InputDataSet(*waypoints.T)
    Input.setT( np.array(T) * 4 * S / (T[-1] * avgOmega) )

    return Input


def spline(Input, omegaS, omegaG):

    sigmaS = [Input.X1[0][0],  Input.X2[0][0],  Input.X3[0][0]]
    sigmaG = [Input.X1[-1][0], Input.X2[-1][0], Input.X3[-1][0]]
    sigmaDotS = rbk.dMRP(sigmaS, omegaS)
    sigmaDotG = rbk.dMRP(sigmaG, omegaG)

    Input.setXDot_0(sigmaDotS)
    Input.setXDot_N(sigmaDotG)

    Output = BSpline.OutputDataSet()
    BSpline.interpolate(Input, 100, 4, Output)

    return Output


def computeTorque(sigma, sigmaDot, sigmaDDot, I):

    omega = rbk.dMRP2Omega(sigma, sigmaDot)
    omegaDot = rbk.ddMRP2dOmega(sigma, sigmaDot, sigmaDDot)

    return np.matmul(I, omegaDot) + np.cross(omega, np.matmul(I, omega))


def effortEvaluation(Output, I):

    effort = 0
    sigma     = [Output.X1[0][0], Output.X2[0][0], Output.X3[0][0]]
    sigmaDot  = [Output.XD1[0][0], Output.XD2[0][0], Output.XD3[0][0]]
    sigmaDDot = [Output.XDD1[0][0], Output.XDD2[0][0], Output.XDD3[0][0]]
    L_a = computeTorque(sigma, sigmaDot, sigmaDDot, I)

    for n in range(len(Output.T)-1):
        sigma     = [Output.X1[n+1][0], Output.X2[n+1][0], Output.X3[n+1][0]]
        sigmaDot  = [Output.XD1[n+1][0], Output.XD2[n+1][0], Output.XD3[n+1][0]]
        sigmaDDot = [Output.XDD1[n+1][0], Output.XDD2[n+1][0], Output.XDD3[n+1][0]]
        L_b = computeTorque(sigma, sigmaDot, sigmaDDot, I)
        effort += (np.linalg.norm(L_a) + np.linalg.norm(L_b)) * (Output.T[n+1][0] - Output.T[n][0]) / 2

        L_a = L_b

    return effort


def AStar(nodes, n_start, n_goal):

    for key in nodes:
        nodes[key].heuristic = distanceCart(nodes[key], n_goal)

    O = [n_start]
    C = []
    n = 0

    while O[0] != n_goal and n < 10000:
        n += 1
        C.append(O[0])
        for key in O[0].neighbors:
            if nodes[key] not in C:
                p = nodes[key].heuristic + distanceCart(O[0], nodes[key]) + O[0].priority - O[0].heuristic
                if nodes[key] in O:
                    if p < nodes[key].priority:
                        nodes[key].priority = p
                        nodes[key].backpointer = O[0]
                else:
                    nodes[key].priority = p
                    nodes[key].backpointer = O[0]
                    O.append(nodes[key])
        O.pop(0)

        if not O:
            print("Dead end")
        else:
            O.sort(key = lambda x: x.priority)

    path = backtrack(O[0], n_start)

    return path


def effortBasedAStar(nodes, n_start, n_goal, omegaS, omegaG, avgOmega, I):

    O = [n_start]
    C = []
    n = 0

    while O[0] != n_goal and n < 10000:
        n += 1
        print(n)
        C.append(O[0])
        for key in O[0].neighbors:
            if nodes[key] not in C:
                path = backtrack(O[0], n_start)
                path.append(nodes[key])
                if nodes[key] != n_goal:
                    path.append(n_goal)
                Input = pathHandle(path, avgOmega)
                if Input is None:
                    continue
                Output = spline(Input, omegaS, omegaG)
                p = effortEvaluation(Output, I)
                if nodes[key] in O:
                    if p < nodes[key].priority:
                        nodes[key].priority = p
                        nodes[key].backpointer = O[0]
                else:
                    nodes[key].priority = p
                    nodes[key].backpointer = O[0]
                    O.append(nodes[key])
        O.pop(0)

        if not O:
            print("Dead end")
        else:
            O.sort(key = lambda x: x.priority)

    path = backtrack(O[0], n_start)

    return path

def _configured_grid_module(refinement, cost_type=0, spline_type=0):
    """Return a configured planner and its input-message owners for grid tests."""
    model = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver(refinement)
    model.sigma_BN_goal = [0.0, 0.5, 0.0]  # MRPs [-]
    model.omega_BN_B_goal = [0.0, 0.0, 0.0]  # [rad/s]
    model.avgOmega = 0.03  # [rad/s]
    if spline_type is not None:
        model.BSplineType = spline_type
    if cost_type is not None:
        model.costFcnType = cost_type
    model.appendKeepOutDirection([1.0, 0.0, 0.0], 20.0*macros.D2R)  # [-], [rad]
    model.appendKeepInDirection([0.0, 1.0, 0.0], 70.0*macros.D2R)  # [-], [rad]
    model.appendKeepInDirection([0.0, 0.0, 1.0], 70.0*macros.D2R)  # [-], [rad]

    state = messaging.SCStatesMsgPayload()
    state.r_BN_N = [1.0, 0.0, 0.0]  # [m]
    state.sigma_BN = [0.0, 0.0, -0.5]  # MRPs [-]
    vehicle = messaging.VehicleConfigMsgPayload()
    vehicle.ISCPntB_B = [0.02/3, 0.0, 0.0, 0.0, 0.1256/3, 0.0, 0.0, 0.0, 0.1256/3]  # [kg*m^2]
    planet = messaging.SpicePlanetStateMsgPayload()
    planet.PositionVector = [10.0, 0.0, 0.0]  # [m]
    messages = (messaging.SCStatesMsg().write(state), messaging.VehicleConfigMsg().write(vehicle),
                messaging.SpicePlanetStateMsg().write(planet))
    model.scStateInMsg.subscribeTo(messages[0])
    model.vehicleConfigInMsg.subscribeTo(messages[1])
    model.keepOutCelBodyInMsg.subscribeTo(messages[2])
    model.keepInCelBodyInMsg.subscribeTo(messages[2])
    return model, messages


def _generate_test_grid(model):
    """Generate only the grid, without running the graph search or spline."""
    model.ReadInputs()
    nodes = []
    for attitude in (model.scStateInMsg().sigma_BN, model.sigma_BN_goal):  # MRPs [-]
        endpoint = constrainedAttitudeManeuver.Node()
        endpoint.sigma_BN = attitude
        nodes.append(endpoint)
    model.GenerateGrid(*nodes)
    return nodes


@pytest.mark.parametrize("refinement", [-1, 0, 1, 21, 2**31 - 1])
@pytest.mark.parametrize("change_after_construction", [False, True])
@pytest.mark.parametrize("entry_point", ["reset", "grid"])
def test_invalid_grid_refinement(refinement, change_after_construction, entry_point):
    """Reject invalid refinement through both entry points, including public N changes."""
    model, messages = _configured_grid_module(5 if change_after_construction else refinement)
    if change_after_construction:
        model.N = refinement
    with pytest.raises(bskLogging.BasiliskError, match="grid refinement N must be between 2 and 20"):
        if entry_point == "reset":
            model.Reset(0)
        else:
            _generate_test_grid(model)


def test_default_constructor_requires_grid_refinement():
    """Require an explicit refinement level when using the no-argument constructor."""
    model = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver()
    start, goal = constrainedAttitudeManeuver.Node(), constrainedAttitudeManeuver.Node()
    goal.sigma_BN = [0.0, 0.5, 0.0]  # MRPs [-]
    assert model.N == 0
    with pytest.raises(bskLogging.BasiliskError, match="grid refinement N must be between 2 and 20"):
        model.GenerateGrid(start, goal)
    model.N = 5
    model.GenerateGrid(start, goal)
    assert model.keyS != model.keyG


@pytest.mark.parametrize("args", [(), (5,)], ids=["default", "with-refinement"])
def test_goal_defaults(args):
    """Require a finite goal, accept an explicit identity, and default to rest at the goal."""
    model = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver(*args)
    configured, messages = _configured_grid_module(5)
    model.N = configured.N
    model.avgOmega = configured.avgOmega
    model.costFcnType = model.BSplineType = 0
    model.scStateInMsg.subscribeTo(messages[0])
    model.vehicleConfigInMsg.subscribeTo(messages[1])

    assert np.isnan(model.sigma_BN_goal).all()
    np.testing.assert_array_equal(model.omega_BN_B_goal, np.zeros(3))  # [rad/s]
    with pytest.raises(bskLogging.BasiliskError, match="finite goal attitude"):
        model.Reset(0)
    model.sigma_BN_goal = [0.0, np.inf, 0.0]  # MRPs [-]
    with pytest.raises(bskLogging.BasiliskError, match="finite goal attitude"):
        model.Reset(0)
    model.sigma_BN_goal = [0.0, 0.0, 0.0]  # MRPs [-]
    model.Reset(0)
    for component in ("X1", "X2", "X3", "XD1", "XD2", "XD3"):
        assert np.ravel(getattr(model.Output, component))[-1] == pytest.approx(0.0, abs=1e-12)


def test_python_goal_requires_three_components():
    """Reject incomplete or oversized Python goals and preserve the previous attitude."""
    model = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver()
    goal = np.array([0.0, 0.25, -0.5])  # MRPs [-]
    model.sigma_BN_goal = goal
    for size in (0, 1, 2, 4):
        with pytest.raises(ValueError):
            model.sigma_BN_goal = [0.0] * size
        np.testing.assert_array_equal(model.sigma_BN_goal, goal)
    model.omega_BN_B_goal = [0.001]  # [rad/s]
    np.testing.assert_array_equal(model.omega_BN_B_goal, [0.001, 0.0, 0.0])  # [rad/s]


@pytest.mark.parametrize("refinement", [2, 20])
def test_grid_refinement_boundaries(refinement):
    """Build grids at both supported limits and select valid endpoint keys."""
    model, messages = _configured_grid_module(refinement)
    model.boresights.keepInFov = [np.pi]*10  # [rad] Include nodes on the coarsest grid.
    _generate_test_grid(model)
    for key in (model.keyS, model.keyG):
        assert all(abs(index) < refinement for index in key)
        assert model.returnNodeState(key)
        assert all(np.isfinite(model.returnNodeCoord(key, axis)) for axis in range(3))


@pytest.mark.parametrize("cost_type", [0, 1])
@pytest.mark.parametrize("refinement", [5, 20])
def test_grid_without_free_nodes(cost_type, refinement):
    """Reject fully blocked grids, including nodes affected by rotation-matrix roundoff."""
    model, messages = _configured_grid_module(refinement, cost_type)
    unset_key = [model.invalidGridIndex] * 3
    assert abs(model.invalidGridIndex) >= refinement
    assert model.keyS == model.keyG == unset_key
    model.boresights.keepOutFov = [np.pi]*10  # [rad]
    model.boresights.keepInFov = [np.pi]*10  # [rad] Avoid masking keep-out classification errors.
    with pytest.raises(bskLogging.BasiliskError, match="no constraint-free grid node"):
        model.Reset(0)
    assert model.path.N == 0
    assert model.keyS == model.keyG == unset_key


@pytest.mark.parametrize("cost_type", [0, 1])
def test_blocked_goal(cost_type):
    """Reject a forbidden goal even when nearby free nodes support a maneuver."""
    model, messages = _configured_grid_module(5, cost_type)
    model.Reset(0)
    free_goal = model.sigma_BN_goal
    blocked = [0.0, 0.0, 0.0]  # MRPs [-], points the keep-out boresight at the source
    model.sigma_BN_goal = blocked
    for plan in (lambda: model.Reset(0), lambda: _generate_test_grid(model)):
        with pytest.raises(bskLogging.BasiliskError, match="target attitude violates"):
            plan()
        assert model.path.N == 0
        assert model.keyS == model.keyG == [model.invalidGridIndex] * 3
        with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
            model.UpdateState(0)

    state = messages[0].read()
    state.sigma_BN = blocked
    messages[0].write(state)
    model.sigma_BN_goal = free_goal
    model.Reset(0)
    assert not model.returnNodeState(model.keyS)
    assert model.returnNodeState(model.keyG)


@pytest.mark.parametrize("cost_type", [0, 1], ids=["distance", "effort"])
@pytest.mark.parametrize("at_start", [False, True], ids=["goal", "start"])
@pytest.mark.parametrize("sign", [-1, 1])
def test_half_turn_endpoint_with_free_interior_sample(cost_type, at_start, sign):
    """Reach either representation of a half-turn whose closest free sample is an interior node."""
    model, messages = _configured_grid_module(3, cost_type)
    model.boresights.keepInBoresightCount = 0
    model.boresights.keepOutFov = [np.pi / 4] * 10  # [rad], +x body boresight
    state = messages[0].read()
    body = messages[2].read()
    body.PositionVector = np.array(state.r_BN_N) + [0.0, 10.0, 0.0]  # [m], source along inertial +y
    messages[2].write(body)

    half_turn = sign * np.array([1.0, 2.0, 1.0]) / np.sqrt(6.0)  # MRPs [-]
    start, goal = (half_turn, np.zeros(3)) if at_start else (np.zeros(3), half_turn)
    state.sigma_BN = start
    messages[0].write(state)
    model.sigma_BN_goal = goal
    model.Reset(0)

    for index, expected in ((0, start), (model.path.N - 1, goal)):
        np.testing.assert_allclose([model.returnPathCoord(index, axis) for axis in range(3)], expected)


def test_half_turn_goal_with_both_interior_cells_free():
    """Keep the same interior sample for either goal sign even when its mirror is free."""
    model, messages = _configured_grid_module(3)
    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    half_turn = np.ones(3) / np.sqrt(3.0)  # MRPs [-]
    keys = []
    for goal in (half_turn, -half_turn):
        model.sigma_BN_goal = goal
        model.Reset(0)
        keys.append(tuple(model.keyG))

    assert keys[0] == keys[1]
    mirror = [-index for index in keys[0]]
    assert model.returnNodeState(mirror)
    assert 0 < np.linalg.norm([model.returnNodeCoord(mirror, axis) for axis in range(3)]) < 1


@pytest.mark.parametrize("cost_type", [0, 1], ids=["distance", "effort"])
@pytest.mark.parametrize("goal_z", [-1.0, 1.0])  # MRPs [-]
def test_antipodal_boundary_node_collision(cost_type, goal_z):
    """Reject distinct endpoints mapped to the same sampled half-turn for either goal sign."""
    model, messages = _configured_grid_module(5, cost_type)
    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    state = messages[0].read()
    state.sigma_BN = [0.0, 0.0, 0.95]  # MRPs [-]
    messages[0].write(state)
    model.sigma_BN_goal = [0.0, 0.0, goal_z]  # MRPs [-]
    with pytest.raises(bskLogging.BasiliskError, match="start and goal attitudes map to the same grid node"):
        model.Reset(0)
    assert model.path.N == 0
    assert model.keyS == model.keyG == [model.invalidGridIndex] * 3


@pytest.mark.parametrize("goal_z", [-0.49, -0.5], ids=["nearby", "identical"])
def test_coincident_endpoint_keys(goal_z):
    """Reject a maneuver whose start and goal occupy the same grid node."""
    model, messages = _configured_grid_module(5)
    model.sigma_BN_goal = [0.0, 0.0, goal_z]  # MRPs [-], starting at [0, 0, -0.5]
    with pytest.raises(bskLogging.BasiliskError, match="start and goal attitudes map to the same grid node"):
        model.Reset(0)
    assert model.path.N == 0
    assert model.keyS == model.keyG == [model.invalidGridIndex] * 3


@pytest.mark.parametrize("cost_type", [0, 1])
@pytest.mark.parametrize("start", [[0.0, 0.0, 1.0], [0.6, 0.8, 0.0]])  # MRPs [-]
def test_equivalent_shadow_endpoints(cost_type, start):
    """Reject opposite boundary nodes that describe the same physical attitude."""
    model, messages = _configured_grid_module(5, cost_type)
    model.boresights.keepOutBoresightCount = 0
    model.boresights.keepInFov = [np.pi] * 10  # [rad]
    state = messages[0].read()
    state.sigma_BN = start
    messages[0].write(state)
    model.sigma_BN_goal = -np.array(start)
    with pytest.raises(bskLogging.BasiliskError, match="start and goal represent the same attitude"):
        model.Reset(0)
    assert model.path.N == 0
    assert model.keyS == model.keyG == [model.invalidGridIndex] * 3


@pytest.mark.parametrize("cost_type", [0, 1], ids=["distance", "effort"])
@pytest.mark.parametrize("refinement, reason", [(5, "open list exhausted"), (16, "iteration limit reached")])
@pytest.mark.parametrize("entry_point", ["reset", "direct"])
def test_disconnected_grid(refinement, reason, cost_type, entry_point):
    """Reject a disconnected grid and resume publishing only after a successful reset."""
    model, messages = _configured_grid_module(5, cost_type)
    model.SelfInit()
    cpp_output = model.attRefOutMsg.addSubscriber()
    c_output = messaging.AttRefMsgReader()
    c_output.subscribeTo(model.attRefOutMsgC)
    state = messages[0].read()
    state.sigma_BN = [0.0, 0.0, 0.0]  # MRPs [-]
    messages[0].write(state)
    model.sigma_BN_goal = [0.0, 0.0, 1.0]  # MRPs [-], half-turn about z

    model.boresights.keepOutBoresightCount = 0
    model.boresights.keepInBoresightCount = 0
    for boresight in ([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]):
        model.appendKeepInDirection(boresight, np.pi)  # [rad]
    model.Reset(0)
    step = macros.sec2nano(1)  # [ns]
    model.UpdateState(step)
    assert cpp_output.timeWritten() == c_output.timeWritten() == step

    # The start satisfies the +x cone and the goal satisfies the disjoint -x cone.
    model.N = refinement
    model.boresights.keepInFov = [40.0 * macros.D2R] * 10  # [rad]
    with pytest.raises(bskLogging.BasiliskError, match=reason):
        if entry_point == "reset":
            model.Reset(step)
        else:
            _generate_test_grid(model)
            if cost_type == 0:
                model.AStar()
            else:
                model.effortBasedAStar()
    assert model.keyS != model.keyG
    assert model.returnNodeState(model.keyS) and model.returnNodeState(model.keyG)
    assert model.path.N == 0
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(2 * step)
    assert cpp_output.timeWritten() == c_output.timeWritten() == step

    model.N = 5
    model.boresights.keepInFov = [np.pi] * 10  # [rad]
    model.Reset(2 * step)
    model.UpdateState(3 * step)
    assert cpp_output.timeWritten() == c_output.timeWritten() == 3 * step


@pytest.mark.parametrize("spline_type", [0, 1], ids=["interpolation", "approximation"])
@pytest.mark.parametrize("start_z, goal_z, fitted_start_z, fitted_goal_z",
                         [(-0.5, -0.55, -0.5, -0.55), (1.0, -0.8, -1.0, -0.8), (0.8, -1.0, 0.8, 1.0)],
                         ids=["principal", "from-shadow-boundary", "to-shadow-boundary"])
def test_two_waypoint_reference(spline_type, start_z, goal_z, fitted_start_z, fitted_goal_z):
    """Fit both endpoint attitudes and rates and publish finite references."""
    model, messages = _configured_grid_module(6)
    model.SelfInit()
    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    model.appendKeepInDirection([1.0, 0.0, 0.0], np.pi)  # [rad]
    model.BSplineType = spline_type
    state = messages[0].read()
    state.sigma_BN = [0.0, 0.0, start_z]  # MRPs [-]
    state.omega_BN_B = [0.001, 0.0, 0.0]  # [rad/s]
    messages[0].write(state)
    model.sigma_BN_goal = [0.0, 0.0, goal_z]  # MRPs [-]
    model.omega_BN_B_goal = [0.0, 0.002, 0.003]  # [rad/s]
    model.Reset(0)
    assert model.path.N == 2
    assert np.isfinite(model.pathCost)

    start = np.array([0.0, 0.0, fitted_start_z])  # MRPs [-]
    goal = np.array([0.0, 0.0, fitted_goal_z])  # MRPs [-]
    start_dot = rbk.dMRP(start, state.omega_BN_B)
    goal_dot = rbk.dMRP(goal, model.omega_BN_B_goal)
    duration = np.ravel(model.Output.T)[-1]  # [s]
    for u in (0.0, 0.25, 1.0):
        # Round down so the final sample stays inside the maneuver.
        model.UpdateState(int(u * duration * 1e9))  # [ns]
        expected = (start + u**2 * (3 - 2 * u) * (goal - start)
                    + duration * (u * (1 - u)**2 * start_dot + u**2 * (u - 1) * goal_dot))
        for output in (model.attRefOutMsg, model.attRefOutMsgC):
            ref = output.read()
            np.testing.assert_allclose(ref.sigma_RN, expected, rtol=0, atol=1e-10)
            assert np.isfinite(ref.omega_RN_N).all() and np.isfinite(ref.domega_RN_N).all()
            if u in (0.0, 1.0):
                omega_body = state.omega_BN_B if u == 0 else model.omega_BN_B_goal
                np.testing.assert_allclose(ref.omega_RN_N, rbk.MRP2C(expected).T @ omega_body,
                                           rtol=0, atol=1e-10)  # [rad/s]


@pytest.mark.parametrize("spline_type", [0, 1], ids=["interpolation", "approximation"])
@pytest.mark.parametrize("at_start, at_goal", [(True, False), (False, True), (True, True)],
                         ids=["start", "goal", "both"])
def test_two_waypoint_endpoint_accelerations(spline_type, at_start, at_goal):
    """Refit a two-waypoint path with the prescribed MRP derivatives at either or both endpoints."""
    model, messages = _configured_grid_module(6, spline_type=spline_type)
    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    state = messages[0].read()
    state.omega_BN_B = [0.001, 0.002, 0.0]  # [rad/s]
    messages[0].write(state)
    model.sigma_BN_goal = [0.0, 0.0, -0.55]  # MRPs [-]
    model.omega_BN_B_goal = [0.0, -0.002, 0.003]  # [rad/s]
    model.Reset(0)
    assert model.path.N == 2

    start_accel = [0.001, -0.002, 0.003]  # [1/s^2]
    goal_accel = [-0.004, 0.005, 0.002]  # [1/s^2]
    if at_start:
        model.Input.setXDDot_0(start_accel)
    if at_goal:
        model.Input.setXDDot_N(goal_accel)
    model.spline()

    output = model.Output
    positions = np.column_stack((output.X1, output.X2, output.X3))
    rates = np.column_stack((output.XD1, output.XD2, output.XD3))
    accelerations = np.column_stack((output.XDD1, output.XDD2, output.XDD3))
    np.testing.assert_allclose(positions[[0, -1]], [state.sigma_BN, model.sigma_BN_goal], rtol=0, atol=1e-12)
    expected_rates = [rbk.dMRP(state.sigma_BN, state.omega_BN_B),
                      rbk.dMRP(model.sigma_BN_goal, model.omega_BN_B_goal)]
    np.testing.assert_allclose(rates[[0, -1]], expected_rates, rtol=0, atol=1e-12)
    if at_start:
        np.testing.assert_allclose(accelerations[0], start_accel, rtol=0, atol=1e-12)
    if at_goal:
        np.testing.assert_allclose(accelerations[-1], goal_accel, rtol=0, atol=1e-12)
    assert output.P == 3 + at_start + at_goal
    assert np.isfinite(model.effortEvaluation())


@pytest.mark.parametrize("refinement, cost_type, goal_z", [
    (2, 0, 0.0), (5, 0, 0.0), (5, 1, 0.0), (8, 0, -0.15),
], ids=["direct", "multi-waypoint", "effort", "interior-identity"])  # goal MRPs [-]
def test_shadow_start_with_identity_waypoint(refinement, cost_type, goal_z):
    """Equivalent starts must yield the same finite reference at and around identity."""
    model, messages = _configured_grid_module(refinement, cost_type)
    model.boresights.keepOutBoresightCount = model.boresights.keepInBoresightCount = 0
    model.sigma_BN_goal = [0.0, 0.0, goal_z]  # MRPs [-]
    state = messages[0].read()
    state.omega_BN_B = [0.001, 0.0, 0.0]  # [rad/s]

    def plan(start_z):
        state.sigma_BN = [0.0, 0.0, start_z]  # MRPs [-]
        messages[0].write(state)
        model.Reset(0)
        return np.column_stack((model.Output.T, model.Output.X1, model.Output.X2, model.Output.X3))

    principal = plan(0.8)  # MRPs [-]
    shadow = plan(-1.25)  # MRPs [-], shadow of 0.8 about z
    waypoints = [[model.returnPathCoord(n, axis) for axis in range(3)] for n in range(model.path.N)]
    assert [0.0, 0.0, 0.0] in waypoints
    assert np.isfinite(model.pathCost)
    np.testing.assert_allclose(shadow, principal, rtol=1e-11, atol=1e-12)
    np.testing.assert_allclose(shadow[0, 1:], [0.0, 0.0, 0.8], atol=1e-12)
    np.testing.assert_allclose(shadow[-1, 1:], model.sigma_BN_goal, atol=1e-12)


@pytest.mark.parametrize("refinement, start_z, shadow_z, goal", [
    (5, 0.8, -1.25, [0.0, 0.0, 0.01]),
    (5, 1.0, -1.0, [0.0, 0.0, 0.01]),
    (2, 1.0, -1.0, [0.05, 0.0, 0.001]),
], ids=["shadow-start", "half-turn", "off-axis"])  # MRPs [-]
@pytest.mark.parametrize("cost_type", [0, 1], ids=["distance", "effort"])
def test_near_identity_reference_from_equivalent_starts(refinement, start_z, shadow_z, goal, cost_type):
    """Fit the smaller-norm waypoint sequence and publish the same maneuver from either MRP set."""
    model, messages = _configured_grid_module(refinement, cost_type)
    model.SelfInit()
    model.boresights.keepOutBoresightCount = model.boresights.keepInBoresightCount = 0
    model.sigma_BN_goal = goal
    state = messages[0].read()

    def sample(z):
        state.sigma_BN = [0.0, 0.0, z]  # MRPs [-]
        messages[0].write(state)
        model.Reset(0)
        waypoints = np.column_stack((model.Input.X1, model.Input.X2, model.Input.X3))
        assert np.max(np.linalg.norm(waypoints, axis=1)) <= 1.0 + 1e-12
        quaternions = np.array([rbk.MRP2EP(sigma) for sigma in waypoints])
        adjacent_dots = np.sum(quaternions[:-1] * quaternions[1:], axis=1)
        assert np.all(adjacent_dots >= -4 * np.finfo(float).eps)
        duration = np.ravel(model.Input.T)[-1]  # [s]
        samples = []
        for t in np.linspace(0.0, duration, 9):
            model.UpdateState(macros.sec2nano(t))
            ref = model.attRefOutMsg.read()
            samples.append(np.r_[rbk.MRP2C(ref.sigma_RN).ravel(), ref.omega_RN_N, ref.domega_RN_N])
        return duration, samples

    duration, reference = sample(start_z)
    shadow_duration, shadow_reference = sample(shadow_z)
    assert shadow_duration == pytest.approx(duration, abs=1e-10, rel=0)
    np.testing.assert_allclose(shadow_reference, reference, atol=1e-10, rtol=0)


@pytest.mark.parametrize("sign", [1, -1])
def test_tied_peak_waypoint_norms(sign):
    """Keep the same fitted waypoints when equivalent starts round tied peak norms differently."""
    model, messages = _configured_grid_module(5)
    model.boresights.keepOutBoresightCount = model.boresights.keepInBoresightCount = 0
    start = sign * np.array([0.0, 0.0, 0.58])  # MRPs [-]
    model.sigma_BN_goal = -start
    state = messages[0].read()
    # The two continuous sequences have the same peak norm at opposite endpoints.
    expected_z = sign * np.array([0.58, 0.75, 1.0, 1.0 / 0.75, 1.0 / 0.58])  # MRPs [-]
    for attitude in (start, rbk.MRPshadow(start)):
        state.sigma_BN = attitude
        messages[0].write(state)
        model.Reset(0)
        np.testing.assert_allclose(np.ravel(model.Input.X3), expected_z, rtol=0, atol=1e-14)


@pytest.mark.parametrize("raw_z, expected_z", [
    ([-1.25, 0.5, 0.25, 0.01], [0.8, 0.5, 0.25, 0.01]),
    ([0.8, 1.0, -0.5, -0.25], [-1.25, -1.0, -0.5, -0.25]),
], ids=["principal-sequence-start", "shadow-sequence-start"])  # MRPs [-]
def test_reference_path_waypoints(raw_z, expected_z):
    """Check both choices of the continuous MRP sequence against known waypoints."""
    path = [SimpleNamespace(sigma_BN=np.array([0.0, 0.0, z])) for z in raw_z]
    reference = pathHandle(path, 0.03)  # [rad/s]
    np.testing.assert_allclose(np.ravel(reference.X3), expected_z, rtol=0, atol=1e-14)


def _set_test_path(model, waypoints):
    """Replace the planner path with nodes at the supplied MRP attitudes."""
    model.path.clear()
    for sigma in waypoints:
        waypoint = constrainedAttitudeManeuver.Node()
        waypoint.sigma_BN = sigma
        model.path.append(waypoint)


def test_off_axis_peak_tie():
    """Keep the principal start for a symmetric off-axis path in either goal representation."""
    # With this attitude, projecting the shadow goal through a quaternion used to break the tie.
    start = np.array([0.2090301766069771, -0.5572751623347386, 0.1653204732428118])  # MRPs [-]
    half_turn = start / np.linalg.norm(start)
    fitted_goal = rbk.MRPshadow(-start)
    model, messages = _configured_grid_module(5)
    for goal in (-start, fitted_goal):
        waypoints = [start, half_turn, goal]
        _set_test_path(model, waypoints)
        model.pathHandle()
        reference = pathHandle([SimpleNamespace(sigma_BN=sigma) for sigma in waypoints], model.avgOmega)
        for data in (model.Input, reference):
            fitted = np.column_stack((data.X1, data.X2, data.X3))
            np.testing.assert_allclose(fitted, [start, half_turn, fitted_goal], rtol=0, atol=1e-14)


def _roll_waypoints(rolls, tilt):
    """Return principal MRPs, tilting the final rotation axis toward x by the given degrees."""
    rolls = np.asarray(rolls)  # [deg]
    z = np.tan(np.deg2rad((rolls + 180) % 360 - 180) / 4)
    waypoints = np.column_stack((np.zeros_like(z), np.zeros_like(z), z))
    angle = np.deg2rad(tilt)  # [rad]
    waypoints[-1] = z[-1] * np.array([np.sin(angle), 0, np.cos(angle)])
    return waypoints


@pytest.mark.parametrize("tilt", [0.0, 1.0])  # [deg]
@pytest.mark.parametrize("reverse", [False, True], ids=["forward", "reverse"])
def test_signed_roll_through_mrp_pole(reverse, tilt):
    """Verify signed and cumulative rotation on every fitted leg of the split maneuver."""
    waypoints = _roll_waypoints([100, 190, 280, 375], tilt)
    if reverse:
        waypoints = waypoints[::-1]
    model, messages = _configured_grid_module(5)
    model.ReadInputs()
    _set_test_path(model, waypoints)
    model.pathHandle()
    model.spline()
    times = np.ravel(model.Input.T)  # [s]
    for i, (a, b) in enumerate(zip(times, times[1:])):
        fitted = [[model.Output.getStates(t, 0, axis) for axis in range(3)] for t in np.linspace(a, b, 257)]
        body_x = np.array([rbk.MRP2C(s)[0] for s in fitted])
        headings = np.unwrap(np.arctan2(body_x[:, 1], body_x[:, 0]))
        expected_x = np.array([rbk.MRP2C(s)[0] for s in waypoints[i:i+2]])
        expected = np.diff(np.unwrap(np.arctan2(expected_x[:, 1], expected_x[:, 0])))[0]
        assert headings[-1] - headings[0] == pytest.approx(expected, abs=1e-10)  # [rad]
        q = np.array([rbk.MRP2EP(s) for s in fitted])
        travel = 4 * np.arctan2(np.linalg.norm(q[1:] - q[:-1], axis=1), np.linalg.norm(q[1:] + q[:-1], axis=1)).sum()
        assert travel < abs(expected) + np.deg2rad(5)  # [rad] Allow minor off-axis curvature.


@pytest.mark.parametrize("tilt", [0.0, 1.0])  # [deg]
@pytest.mark.parametrize("spline_type", [0, 1], ids=["interpolation", "approximation"])
def test_extra_revolutions_reject_fit(tilt, spline_type):
    """A finite spline that adds a full turn inside a leg must be discarded."""
    model, messages = _configured_grid_module(5, spline_type=spline_type)
    model.ReadInputs()
    _set_test_path(model, _roll_waypoints([10, 100, 190, 280, 375], tilt))
    model.pathHandle()
    with pytest.raises(bskLogging.BasiliskError, match="full turn.*Split"):
        model.spline()
    assert np.size(model.Output.T) == 0
    with pytest.raises(bskLogging.BasiliskError, match="successful reset"):
        model.UpdateState(0)


@pytest.mark.parametrize("cost_type, reason", [(0, "full turn"), (1, "open list exhausted")])
def test_reset_rejects_extra_turn_and_recovers(cost_type, reason):
    """Both searches discard excessive fits and a subsequent feasible reset publishes again."""
    model, messages = _configured_grid_module(5, cost_type=cost_type)
    model.SelfInit()
    reader = model.attRefOutMsg.addSubscriber()
    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    model.sigma_BN_goal = [0, 0, -0.25]  # MRPs [-]
    model.Reset(0)
    model.UpdateState(0)
    model.omega_BN_B_goal = [0, 0, 2.0]  # [rad/s]
    with pytest.raises(bskLogging.BasiliskError, match=reason):
        model.Reset(1)
    assert np.size(model.Output.T) == 0
    with pytest.raises(bskLogging.BasiliskError, match="successful reset"):
        model.UpdateState(1)
    assert reader.timeWritten() == 0  # [ns]
    model.omega_BN_B_goal = [0, 0, 0]  # [rad/s]
    model.Reset(2)
    model.UpdateState(2)
    assert reader.timeWritten() == 2  # [ns]


@pytest.mark.parametrize("tilt", [0.0, 1.0])  # [deg]
def test_chart_selection_near_mrp_pole(tilt):
    """Score the closest approach between waypoints even when the final axis is tilted."""
    waypoints = _roll_waypoints([10, 100, 190, 280, 375], tilt)
    model, messages = _configured_grid_module(5)
    _set_test_path(model, waypoints)
    model.pathHandle()
    reference = pathHandle([SimpleNamespace(sigma_BN=s) for s in waypoints], model.avgOmega)
    for data in (model.Input, reference):
        # The continuous set that passes through the identity on the axial final leg.
        expected = [rbk.MRPshadow(waypoints[0]), rbk.MRPshadow(waypoints[1]), *waypoints[2:]]
        np.testing.assert_allclose(np.column_stack((data.X1, data.X2, data.X3)), expected, atol=1e-12)


@pytest.mark.parametrize("reverse", [False, True], ids=["forward", "reverse"])
def test_feasible_half_turn_direction(reverse):
    """Take the finite short arc at a half-turn when the other choice runs into a pole."""
    rolls = np.array([0, 50, -130, 15])  # [deg]
    if reverse:
        rolls = rolls[::-1]
    z = np.tan(np.deg2rad(rolls) / 4)
    waypoints = np.column_stack((np.zeros_like(z), np.zeros_like(z), z))
    model, messages = _configured_grid_module(5)
    model.ReadInputs()
    _set_test_path(model, waypoints)
    model.pathHandle()
    reference = pathHandle([SimpleNamespace(sigma_BN=s) for s in waypoints], model.avgOmega)
    for data in (model.Input, reference):
        np.testing.assert_allclose(np.column_stack((data.X1, data.X2, data.X3)), waypoints, atol=1e-14)
    model.spline()
    assert np.isfinite(model.effortEvaluation())


def test_path_crossing_both_mrp_poles():
    """Reject a path that crosses both poles between successive waypoints."""
    rolls = np.array([10, 100, 190, 280, 375, 460, 550, 640, 735])  # [deg]
    z = np.tan(np.deg2rad((rolls + 180) % 360 - 180) / 4)  # principal MRPs [-]
    model, messages = _configured_grid_module(5)
    _set_test_path(model, ([0, 0, value] for value in z))
    with pytest.raises(bskLogging.BasiliskError, match="MRP"):
        model.pathHandle()
    assert np.size(model.Input.T) == 0


def test_reset_restarts_reference_clock():
    """Sample the same trajectory relative to two reset epochs on both outputs."""
    model, messages = _configured_grid_module(5)
    model.SelfInit()
    readers = [model.attRefOutMsg.addSubscriber(), messaging.AttRefMsgReader()]
    readers[1].subscribeTo(model.attRefOutMsgC)
    step = macros.sec2nano(5)  # [ns]
    epoch = macros.sec2nano(600)  # [ns], later than the duration of the first maneuver
    model.Reset(0)
    model.UpdateState(step)
    expected = readers[0]().sigma_RN
    start = messages[0].read().sigma_BN
    assert not np.allclose(expected, start)
    model.Reset(epoch)
    for now, attitude in ((epoch - 1, start), (epoch, start), (epoch + step, expected)):
        model.UpdateState(now)
        for reader in readers:
            assert reader.timeWritten() == now
            np.testing.assert_allclose(reader().sigma_RN, attitude, rtol=0, atol=1e-12)


def test_three_waypoint_approximation():
    """Use a valid quartic approximation when there is one interior waypoint."""
    model, messages = _configured_grid_module(6)
    model.BSplineType = 1
    model.sigma_BN_goal = [0.0, 0.0, -0.8]  # MRPs [-]
    model.Reset(0)
    assert model.path.N == 3
    assert model.Output.P == 4
    assert np.isfinite(model.pathCost)


def test_three_waypoint_approximation_control_point_limit():
    """Reject six endpoint conditions on five control points and accept exactly five."""
    model, messages = _configured_grid_module(6, spline_type=1)
    model.sigma_BN_goal = [0.0, 0.0, -0.8]  # MRPs [-]
    model.Reset(0)
    assert model.path.N == 3
    model.Input.setXDDot_0([0.0, 0.0, 0.0])  # [1/s^2]
    model.Input.setXDDot_N([0.0, 0.0, 0.0])  # [1/s^2]
    with pytest.raises(bskLogging.BasiliskError, match="at least 6 control points"):
        model.spline()
    assert np.size(model.Output.T) == 0
    model.Input.XDDot_N_flag = False
    model.spline()
    assert np.isfinite(model.effortEvaluation())
    np.testing.assert_allclose(np.ravel(model.Output.X3)[[0, -1]], np.ravel(model.Input.X3)[[0, -1]])


def test_repeated_path_waypoint():
    """A repeated node cannot supply a time interval. Removing it restores the fit."""
    model, messages = _configured_grid_module(5)
    model.ReadInputs()
    start, goal = constrainedAttitudeManeuver.Node(), constrainedAttitudeManeuver.Node()
    goal.sigma_BN = [0.0, 0.0, 0.25]  # MRPs [-]
    for node in (start, start, goal):
        model.path.append(node)
    with pytest.raises(bskLogging.BasiliskError, match="strictly increasing"):
        model.pathHandle()
    assert np.size(model.Input.T) == 0
    model.path.pop(1)
    model.pathHandle()
    model.spline()
    assert np.isfinite(model.effortEvaluation())


@pytest.mark.parametrize("component", ["X1", "X2", "X3", "T"])
@pytest.mark.parametrize("spline_type", [0, 1])
def test_spline_waypoint_array_lengths(spline_type, component):
    """Reject short and long waypoint arrays and recover after the original values are restored."""
    model, messages = _configured_grid_module(6, spline_type=spline_type)
    model.sigma_BN_goal = [0.0, 0.0, -0.8]  # MRPs [-]
    model.Reset(0)
    original = np.ravel(getattr(model.Input, component))
    assert original.size == 3
    expected = model.pathCost
    for values in ([], original[:-1], np.append(original, original[-1])):
        setattr(model.Input, component, values)
        with pytest.raises(bskLogging.BasiliskError, match="waypoint"):
            model.spline()
        assert np.size(model.Output.T) == 0
    setattr(model.Input, component, original)
    model.spline()
    assert model.effortEvaluation() == pytest.approx(expected)


def test_two_waypoint_approximation_weight_lengths():
    """Require two supplied weights even though both endpoints are fixed."""
    model, messages = _configured_grid_module(6, spline_type=1)
    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    model.sigma_BN_goal = [0.0, 0.0, -0.55]  # MRPs [-]
    model.Reset(0)
    assert model.path.N == 2
    expected = np.ravel(model.Output.X3)
    for weights in ([], [1], [1, 1, 1]):
        model.Input.setW(weights)
        with pytest.raises(bskLogging.BasiliskError, match="weights must have one entry per waypoint"):
            model.spline()
        assert np.size(model.Output.T) == 0
    model.Input.setW([0, 0])
    model.spline()
    np.testing.assert_allclose(np.ravel(model.Output.X3), expected, atol=1e-12, rtol=0)


def test_spline_optional_waypoint_arrays():
    """Use generated times when requested and require one weight per waypoint for approximation."""
    model, messages = _configured_grid_module(6, spline_type=1)
    model.sigma_BN_goal = [0.0, 0.0, -0.8]  # MRPs [-]
    model.Reset(0)
    count = model.path.N
    model.Input.setT([])
    model.Input.setAvgXDot(0.1)  # [1/s] Average MRP coordinate speed
    for length in (count - 1, count + 1):
        model.Input.setW(np.ones(length))
        with pytest.raises(bskLogging.BasiliskError, match="weights must have one entry per waypoint"):
            model.spline()
    assert count == 3
    model.Input.setW([1, 0, 1])
    with pytest.raises(bskLogging.BasiliskError, match="rank deficient"):
        model.spline()
    assert np.size(model.Output.T) == 0
    model.Input.setW(np.ones(count))
    model.spline()
    assert np.isfinite(model.effortEvaluation())


def test_effort_requires_fitted_samples():
    """Reject effort evaluation before a fit and accept samples from a completed fit."""
    model, messages = _configured_grid_module(5)
    with pytest.raises(bskLogging.BasiliskError, match="at least two fitted samples"):
        model.effortEvaluation()
    model.Output.T = [0.0]  # [s], one sample provides no integration interval.
    with pytest.raises(bskLogging.BasiliskError, match="at least two fitted samples"):
        model.effortEvaluation()
    model.Reset(0)
    assert model.effortEvaluation() == pytest.approx(model.pathCost)


@pytest.mark.parametrize("component", ["X1", "X2", "X3", "XD1", "XD2", "XD3", "XDD1", "XDD2", "XDD3"])
def test_torque_and_effort_reject_mismatched_sample_lengths(component):
    """Require every fitted component to have exactly one value per time sample."""
    model, messages = _configured_grid_module(5)
    model.Reset(0)
    inertia = messages[1].read().ISCPntB_B
    samples = np.ravel(getattr(model.Output, component))
    for mismatched in (samples[:-1], np.append(samples, samples[-1])):
        setattr(model.Output, component, mismatched)
        with pytest.raises(bskLogging.BasiliskError, match="sample array lengths must match the time array"):
            model.computeTorqueNorm(len(samples) - 1, inertia)
        with pytest.raises(bskLogging.BasiliskError, match="sample array lengths must match the time array"):
            model.effortEvaluation()
    setattr(model.Output, component, samples)
    assert model.effortEvaluation() == pytest.approx(model.pathCost)


@pytest.mark.parametrize("search", ["AStar", "effortBasedAStar"])
def test_spline_requires_current_waypoints(search):
    """Fit the new goal only after the waypoints for the rebuilt path have been prepared."""
    model, messages = _configured_grid_module(6)
    model.SelfInit()
    cpp_output = model.attRefOutMsg.addSubscriber()
    c_output = messaging.AttRefMsgReader()
    c_output.subscribeTo(model.attRefOutMsgC)
    model.Reset(0)
    step = macros.sec2nano(1)  # [ns]
    model.UpdateState(step)
    start, goal = constrainedAttitudeManeuver.Node(), constrainedAttitudeManeuver.Node()
    start.sigma_BN = messages[0].read().sigma_BN
    goal.sigma_BN = [0.0, 0.0, -0.8]  # MRPs [-]
    model.GenerateGrid(start, goal)
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(2 * step)
    assert cpp_output.timeWritten() == c_output.timeWritten() == step
    assert np.isnan(model.pathCost)
    assert np.size(model.Output.T) == 0
    with pytest.raises(bskLogging.BasiliskError, match="at least two fitted samples"):
        model.effortEvaluation()
    with pytest.raises(bskLogging.BasiliskError, match="prepared waypoints"):
        model.spline()
    getattr(model, search)()
    assert np.size(model.Output.T) == 0
    with pytest.raises(bskLogging.BasiliskError, match="at least two fitted samples"):
        model.effortEvaluation()
    with pytest.raises(bskLogging.BasiliskError, match="prepared waypoints"):
        model.spline()
    model.pathHandle()
    model.spline()
    fitted_goal = [np.ravel(axis)[-1] for axis in (model.Output.X1, model.Output.X2, model.Output.X3)]
    np.testing.assert_allclose(fitted_goal, goal.sigma_BN, rtol=0, atol=1e-12)
    assert np.isfinite(model.effortEvaluation())


def test_failed_refit_discards_previous_output():
    """A failed refit clears the old spline while preserving the waypoints for a retry."""
    model, messages = _configured_grid_module(5)
    model.Reset(0)
    inertia = messages[1].read().ISCPntB_B
    expected_cost = model.pathCost
    expected_torque = model.computeTorqueNorm(0, inertia)
    expected_start_z = model.Output.getStates(0.0, 0, 2)
    model.BSplineType = 2
    with pytest.raises(bskLogging.BasiliskError, match="BSplineType"):
        model.spline()
    # The planner's getData wrapper accepts Python lists for the three output buffers.
    with pytest.raises(bskLogging.BasiliskError, match="spline query requires fitted samples"):
        constrainedAttitudeManeuver.OutputDataSet.getData(model.Output, 0.0, [0.0] * 3, [0.0] * 3, [0.0] * 3)
    with pytest.raises(bskLogging.BasiliskError, match="spline query requires fitted samples"):
        model.Output.getStates(0.0, 0, 2)
    with pytest.raises(bskLogging.BasiliskError, match="torque sample index"):
        model.computeTorque(0, inertia, [0.0] * 3)
    with pytest.raises(bskLogging.BasiliskError, match="torque sample index"):
        model.computeTorqueNorm(0, inertia)
    with pytest.raises(bskLogging.BasiliskError, match="at least two fitted samples"):
        model.effortEvaluation()
    model.BSplineType = 0
    model.spline()
    assert model.Output.getStates(0.0, 0, 2) == pytest.approx(expected_start_z)
    assert model.computeTorqueNorm(0, inertia) == pytest.approx(expected_torque)
    assert model.effortEvaluation() == pytest.approx(expected_cost)


def test_torque_sample_index_bounds():
    """Accept the last fitted torque sample and reject indices outside the time array."""
    model, messages = _configured_grid_module(5)
    model.Reset(0)
    inertia = messages[1].read().ISCPntB_B
    count = len(model.Output.T)
    assert np.isfinite(model.computeTorqueNorm(count - 1, inertia))
    for index in (-1, count):
        with pytest.raises(bskLogging.BasiliskError, match="torque sample index"):
            model.computeTorqueNorm(index, inertia)


def test_torque_norm_uses_supplied_inertia():
    """Check torque scaling at the midpoint, where the gyroscopic term is nonzero."""
    model, messages = _configured_grid_module(5)
    model.Reset(0)
    inertia = np.asarray(messages[1].read().ISCPntB_B)  # [kg*m^2]
    midpoint = len(model.Output.T) // 2
    torque = model.computeTorqueNorm(midpoint, inertia)
    assert torque > 0.0  # [N*m]
    assert model.computeTorqueNorm(midpoint, 2 * inertia) == pytest.approx(2 * torque, rel=1e-12, abs=0.0)


@pytest.mark.parametrize("method", ["ReadInputs", "AStar", "effortBasedAStar", "pathHandle", "spline"])
def test_direct_replanning_invalidates_reference(method):
    """Each public replanning step blocks publication of the previously accepted reference."""
    model, messages = _configured_grid_module(5)
    model.Reset(0)
    original_path_length = model.path.N
    getattr(model, method)()
    if method == "AStar":
        assert model.path.N == original_path_length
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)
    if method in ("AStar", "effortBasedAStar"):
        with pytest.raises(bskLogging.BasiliskError, match="prepared waypoints"):
            model.spline()


@pytest.mark.parametrize("method", ["AStar", "effortBasedAStar"])
@pytest.mark.parametrize("failed_rebuild", [False, True], ids=["fresh", "failed-rebuild"])
def test_search_requires_generated_grid(method, failed_rebuild):
    """A direct search requires a successful grid build and can recover after a failed build."""
    model, messages = _configured_grid_module(5)
    if failed_rebuild:
        model.Reset(0)
        model.N = 1
        with pytest.raises(bskLogging.BasiliskError, match="grid refinement"):
            _generate_test_grid(model)
    search = getattr(model, method)
    with pytest.raises(bskLogging.BasiliskError, match="requires a generated grid"):
        search()
    assert model.path.N == 0
    model.N = 5
    _generate_test_grid(model)
    search()
    goal = [model.returnPathCoord(model.path.N - 1, axis) for axis in range(3)]
    np.testing.assert_array_equal(goal, model.sigma_BN_goal)


@pytest.mark.parametrize("avg_omega", [-0.03, 0.0, np.inf, -np.inf, np.nan])  # [rad/s]
@pytest.mark.parametrize("entry_point", ["distance", "effort", "effortBasedAStar", "pathHandle"])
def test_invalid_maneuver_rate(avg_omega, entry_point):
    """Reject invalid timing without exposing a trial path or publishing a reference."""
    effort_search = entry_point in ("effort", "effortBasedAStar")
    model, messages = _configured_grid_module(5, cost_type=int(effort_search))
    model.Reset(0)
    model.avgOmega = avg_omega
    with pytest.raises(bskLogging.BasiliskError, match="avgOmega must be finite and positive"):
        if entry_point in ("distance", "effort"):
            model.Reset(0)
        else:
            getattr(model, entry_point)()
    if effort_search:
        assert model.path.N == 0
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)
    if entry_point == "pathHandle":
        with pytest.raises(bskLogging.BasiliskError, match="prepared waypoints"):
            model.spline()


def test_unset_maneuver_rate():
    """Both constructors leave the maneuver rate at the invalid zero default."""
    for args in ((), (5,)):
        assert constrainedAttitudeManeuver.ConstrainedAttitudeManeuver(*args).avgOmega == 0.0  # [rad/s]


def test_nonfinite_fit_rejects_publication():
    """Keep a fit with a nonfinite endpoint rate from becoming publishable."""
    model, messages = _configured_grid_module(5)
    model.SelfInit()
    model.Reset(0)
    model.omega_BN_B_goal = [0.0, 0.0, np.nan]  # [rad/s]
    with pytest.raises(bskLogging.BasiliskError, match="nonfinite"):
        model.Reset(0)
    with pytest.raises(bskLogging.BasiliskError, match="at least two fitted samples"):
        model.effortEvaluation()
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)


@pytest.mark.parametrize("method, args", [("Reset", (0,)), ("effortBasedAStar", ())], ids=["reset", "direct"])
def test_spline_error_clears_trial_path(method, args):
    """Discard the effort-search candidate on a spline error and allow a retry."""
    model, messages = _configured_grid_module(5, cost_type=1)
    model.SelfInit()
    model.Reset(0)
    expected_length = model.path.N
    plan = getattr(model, method)

    model.BSplineType = 2
    with pytest.raises(bskLogging.BasiliskError, match="BSplineType has not been specified"):
        plan(*args)
    assert model.path.N == 0
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)

    model.BSplineType = 0
    with pytest.raises(bskLogging.BasiliskError, match="prepared waypoints"):
        model.spline()
    plan(*args)
    assert model.path.N == expected_length
    goal = [model.returnPathCoord(model.path.N - 1, axis) for axis in range(3)]
    np.testing.assert_array_equal(goal, model.sigma_BN_goal)


@pytest.mark.parametrize("input_name, message_type, index", [
    ("scStateInMsg", messaging.SCStatesMsg, 0),
    ("vehicleConfigInMsg", messaging.VehicleConfigMsg, 1),
])
@pytest.mark.parametrize("previous_plan", [False, True], ids=["first-reset", "replan"])
def test_required_input_must_be_written(input_name, message_type, index, previous_plan):
    """Reject a missing state or inertia before using any previously buffered value."""
    model, messages = _configured_grid_module(5)
    if previous_plan:
        model.Reset(0)
    reader = getattr(model, input_name)
    replacement = message_type()
    reader.subscribeTo(replacement)
    with pytest.raises(bskLogging.BasiliskError, match=input_name):
        model.Reset(0)
    assert model.path.N == 0
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)

    reader.unsubscribe()
    with pytest.raises(bskLogging.BasiliskError, match=input_name):
        model.Reset(0)
    reader.subscribeTo(replacement)
    replacement.write(messages[index].read())
    model.Reset(0)
    assert model.path.N >= 2


@pytest.mark.parametrize("failed_source", [None, "vehicleConfigInMsg", "keepOutCelBodyInMsg"],
                         ids=["before-first-read", "missing-inertia", "failed-cone-read"])
def test_direct_planning_requires_successful_input_read(failed_source):
    """Only a complete input read makes cached rates and inertia available to direct calls."""
    model, messages = _configured_grid_module(5, cost_type=1)
    if failed_source:
        model.ReadInputs()
        reader = getattr(model, failed_source)
        original = messages[1] if failed_source == "vehicleConfigInMsg" else messages[2]
        unwritten = type(original)()
        reader.subscribeTo(unwritten)
        with pytest.raises(bskLogging.BasiliskError, match=failed_source):
            model.ReadInputs()
        reader.subscribeTo(original)

    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    start, goal = constrainedAttitudeManeuver.Node(), constrainedAttitudeManeuver.Node()
    start.sigma_BN = messages[0].read().sigma_BN
    goal.sigma_BN = model.sigma_BN_goal
    model.GenerateGrid(start, goal)
    with pytest.raises(bskLogging.BasiliskError, match="successful ReadInputs"):
        model.effortBasedAStar()
    model.AStar()
    model.pathHandle()
    with pytest.raises(bskLogging.BasiliskError, match="successful ReadInputs"):
        model.spline()

    # Supply a complete fit so the effort query must check its own cached inertia.
    fitted, owners = _configured_grid_module(5)
    fitted.Reset(0)
    model.Output = fitted.Output
    with pytest.raises(bskLogging.BasiliskError, match="successful ReadInputs"):
        model.effortEvaluation()
    model.Reset(0)
    assert np.isfinite(model.effortEvaluation())


def test_recovery_uses_replacement_inertia():
    """Doubling the inertia after a failed read doubles effort on the same distance-based path."""
    model, messages = _configured_grid_module(5, cost_type=0)
    model.Reset(0)
    original_inertia = np.array(messages[1].read().ISCPntB_B)  # [kg*m^2]
    original_effort = model.pathCost  # [N*m*s]
    assert original_effort > 0.0

    replacement = messaging.VehicleConfigMsg()
    model.vehicleConfigInMsg.subscribeTo(replacement)
    with pytest.raises(bskLogging.BasiliskError, match="vehicleConfigInMsg"):
        model.ReadInputs()
    vehicle = messaging.VehicleConfigMsgPayload()
    vehicle.ISCPntB_B = 2.0 * original_inertia  # [kg*m^2]
    replacement.write(vehicle)
    model.Reset(0)
    np.testing.assert_array_equal(messages[1].read().ISCPntB_B, original_inertia)
    assert model.pathCost == pytest.approx(2.0 * original_effort, rel=1e-12, abs=0.0)


@pytest.mark.parametrize("kind", ["In", "Out"])
@pytest.mark.parametrize("offset", [0.0, np.nan, np.inf], ids=["coincident", "nan", "infinite"])  # [m]
def test_undefined_constraint_direction(kind, offset):
    """Require a finite line of sight for each active celestial-body constraint."""
    model, messages = _configured_grid_module(5)
    model.Reset(0)
    planet = messaging.SpicePlanetStateMsgPayload()
    position = np.array(messages[0].read().r_BN_N)  # [m]
    position[0] += offset
    planet.PositionVector = position
    source = messaging.SpicePlanetStateMsg().write(planet)
    getattr(model, f"keep{kind}CelBodyInMsg").subscribeTo(source)
    with pytest.raises(bskLogging.BasiliskError, match=f"keep{kind}CelBodyInMsg.*line of sight"):
        model.Reset(0)
    assert not model.constraints.keepOut and not model.constraints.keepIn
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)
    source.write(messages[2].read())
    model.Reset(0)
    assert model.constraints.keepOut and model.constraints.keepIn


@pytest.mark.parametrize("kind, other", [("In", "Out"), ("Out", "In")])
def test_optional_constraint_source(kind, other):
    """Recompute active cones after disconnecting, replacing, and restoring a source."""
    model, messages = _configured_grid_module(5)
    reader = getattr(model, f"keep{kind}CelBodyInMsg")
    model.Reset(0)
    reader.unsubscribe()
    model.Reset(0)
    assert not getattr(model.constraints, f"keep{kind}")
    assert getattr(model.constraints, f"keep{other}")

    unwritten = messaging.SpicePlanetStateMsg()
    reader.subscribeTo(unwritten)
    with pytest.raises(bskLogging.BasiliskError, match=f"keep{kind}CelBodyInMsg has not been written"):
        model.Reset(0)
    assert not model.constraints.keepIn and not model.constraints.keepOut
    assert model.path.N == 0
    assert model.keyS == model.keyG == [model.invalidGridIndex] * 3
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)
    reader.subscribeTo(messages[2])
    model.Reset(0)
    assert model.constraints.keepIn and model.constraints.keepOut

    setattr(model.boresights, f"keep{kind}BoresightCount", 0)
    reader.subscribeTo(unwritten)
    getattr(model, f"keep{other}CelBodyInMsg").unsubscribe()
    model.Reset(0)
    assert not model.constraints.keepIn and not model.constraints.keepOut


@pytest.mark.parametrize("cost_type", [0, 1])
def test_unset_spline_type(cost_type):
    """Require an explicit spline selection for either search method."""
    assert constrainedAttitudeManeuver.ConstrainedAttitudeManeuver().BSplineType == -1
    model, messages = _configured_grid_module(5, cost_type, spline_type=None)
    assert model.BSplineType == -1
    with pytest.raises(bskLogging.BasiliskError, match="BSplineType has not been specified"):
        model.Reset(0)
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)


def test_unset_cost_function():
    """Require an explicit cost function before searching a valid grid."""
    model, messages = _configured_grid_module(5, cost_type=None)
    assert model.costFcnType == -1
    with pytest.raises(bskLogging.BasiliskError, match="costFcnType has not been specified"):
        model.Reset(0)


def test_full_sphere_keep_in_roundoff():
    """Include antipodal nodes whose computed unit-vector dot products fall below -1."""
    model, messages = _configured_grid_module(20)
    model.boresights.keepInBoresightCount = 0
    model.appendKeepInDirection([1.0, 0.0, 0.0], np.pi)  # [-], [rad]
    _generate_test_grid(model)
    for key in ([0, -18, -8], [0, 18, 8]):
        assert model.returnNodeState(key)


@pytest.mark.parametrize("cost_type", [0, 1])
@pytest.mark.parametrize("refinement", [5, 6])
def test_grid_reset_and_recovery(cost_type, refinement):
    """Rebuild without stale nodes, reject changed constraints, and recover after failure."""
    model, messages = _configured_grid_module(5, cost_type)
    model.Reset(0)
    assert model.N == 5
    model.N = refinement
    model.Reset(0)
    assert model.N == refinement
    expected = np.array([[model.returnPathCoord(index, axis) for axis in range(3)]
                         for index in range(model.path.N)])
    expected_cost = model.pathCost
    expected_keys = (model.keyS, model.keyG)
    model.Reset(0)
    assert model.N == refinement
    actual = np.array([[model.returnPathCoord(index, axis) for axis in range(3)]
                       for index in range(model.path.N)])
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
    assert model.pathCost == pytest.approx(expected_cost, rel=1e-12)

    model.boresights.keepOutFov = [np.pi]*10  # [rad]
    with pytest.raises(bskLogging.BasiliskError, match="no constraint-free grid node"):
        model.Reset(0)
    assert model.path.N == 0
    assert model.keyS == model.keyG == [model.invalidGridIndex] * 3
    model.boresights.keepOutFov = [20.0*macros.D2R]*10  # [rad]
    model.Reset(0)
    assert model.N == refinement
    assert (model.keyS, model.keyG) == expected_keys
    recovered = np.array([[model.returnPathCoord(index, axis) for axis in range(3)]
                          for index in range(model.path.N)])
    np.testing.assert_allclose(recovered, expected, rtol=0, atol=1e-12)
    assert model.pathCost == pytest.approx(expected_cost, rel=1e-12)


def test_neighbor_capacity():
    """Allow the last available slot and reject overflow without changing the count."""
    node = constrainedAttitudeManeuver.Node()
    capacity = 52
    for _ in range(capacity):
        node.appendNeighbor(node)
    with pytest.raises(bskLogging.BasiliskError, match="capacity exceeded"):
        node.appendNeighbor(node)
    assert node.neighborCount == capacity


def test_node_list_growth():
    """Grow beyond the former capacity and preserve ownership through a path copy."""
    node = constrainedAttitudeManeuver.Node()
    node.sigma_BN = [0.125, 0.0, 0.0]  # MRPs [-]
    nodes = constrainedAttitudeManeuver.NodeList()
    for _ in range(10001):
        nodes.append(node)
    model = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver()
    model.path = nodes
    nodes.clear()
    del node, nodes
    gc.collect()
    assert model.path.N == 10001
    assert model.returnPathCoord(10000, 0) == 0.125
    model.path.pop(10000)
    assert model.path.N == 10000
    model.path.clear()
    assert model.path.N == 0


def test_node_list_index_bounds():
    """Reject invalid removals and swaps without changing the count or node order."""
    queue = constrainedAttitudeManeuver.NodeList()
    with pytest.raises(AttributeError):
        queue.N = 1
    with pytest.raises(TypeError):
        queue.append(None)
    with pytest.raises(bskLogging.BasiliskError, match="out of range"):
        queue.pop(0)
    assert queue.N == 0
    first, second = constrainedAttitudeManeuver.Node(), constrainedAttitudeManeuver.Node()
    queue.append(first)
    queue.append(second)
    for invalid in (-1, queue.N):
        with pytest.raises(bskLogging.BasiliskError, match="out of range"):
            queue.pop(invalid)
        with pytest.raises(bskLogging.BasiliskError, match="out of range"):
            queue.swap(invalid, 0)
        with pytest.raises(bskLogging.BasiliskError, match="out of range"):
            queue.swap(1, invalid)
    assert queue.N == 2
    queue.swap(0, 1)
    queue.pop(0)
    assert queue.N == 1 and queue.contains(first) and not queue.contains(second)


def test_node_list_retains_python_nodes():
    """Retain duplicate entries across a path copy and release each node after its final removal."""
    queue = constrainedAttitudeManeuver.NodeList()
    model = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver()
    first, second = constrainedAttitudeManeuver.Node(), constrainedAttitudeManeuver.Node()
    first.sigma_BN = [0.125, 0.0, 0.0]  # MRPs [-]
    first.priority = 2
    second.priority = 1
    first_handle, second_handle = first.this, second.this
    queue.append(node=first)
    queue.append(second)
    queue.append(first)
    model.path = queue
    with pytest.raises(AttributeError):
        model.path.list = queue.list
    queue.clear()
    del first, second, queue
    gc.collect()
    first_count, second_count = sys.getrefcount(first_handle), sys.getrefcount(second_handle)
    model.path.sort()
    model.path.pop(0)
    assert sys.getrefcount(second_handle) == second_count - 1
    model.path.pop(0)
    assert sys.getrefcount(first_handle) == first_count
    del first_handle
    assert model.returnPathCoord(0, 0) == 0.125
    model.path.pop(0)
    assert model.path.N == 0


def test_backtrack_is_private():
    """Only the planner's searches can follow predecessor pointers."""
    assert not hasattr(constrainedAttitudeManeuver.ConstrainedAttitudeManeuver, "backtrack")


def test_node_back_pointer_is_read_only():
    """A failed predecessor assignment preserves ownership and leaves both nodes usable."""
    node, predecessor = constrainedAttitudeManeuver.Node(), constrainedAttitudeManeuver.Node()
    with pytest.raises(AttributeError):
        node.backPointer = predecessor
    assert node.backPointer is None
    assert predecessor.thisown
    path = constrainedAttitudeManeuver.NodeList()
    path.append(predecessor)
    assert path.contains(predecessor)


def test_node_list_rejects_borrowed_node(monkeypatch):
    """Reject a non-owning proxy before changing the path or its fitted cost."""
    model, messages = _configured_grid_module(5)
    model.Reset(0)
    count, cost = model.path.N, model.pathCost
    node = constrainedAttitudeManeuver.Node()
    monkeypatch.setattr(node, "thisown", False)  # Present a live, non-owning SWIG handle for this test.
    with pytest.raises(TypeError, match="Python-owned Node"):
        model.path.append(node)
    assert model.path.N == count and model.pathCost == cost


def test_grid_backed_path_copy():
    """Reject borrowed and mixed paths before replacing the destination, while allowing self-assignment."""
    source, messages = _configured_grid_module(5)
    source.Reset(0)
    count = source.path.N
    source.path = source.path
    assert source.path.N == count > 0
    destination = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver()
    marker = constrainedAttitudeManeuver.Node()
    destination.path.append(marker)
    for mixed in (False, True):
        if mixed:
            source.path.append(marker)
            source.path.swap(0, count)
        with pytest.raises(bskLogging.BasiliskError, match="grid-backed or borrowed"):
            destination.path = source.path
        with pytest.raises(bskLogging.BasiliskError, match="grid-backed or borrowed"):
            constrainedAttitudeManeuver.NodeList(source.path)
        assert destination.path.N == 1 and destination.path.contains(marker)
    _generate_test_grid(source)
    assert destination.path.N == 1 and destination.path.contains(marker)
    destination.path = source.path
    assert destination.path.N == 0


@pytest.mark.parametrize("edit", ["replace", "replace_same_length", "append", "pop", "swap", "clear"])
def test_path_edit_invalidates_reference(edit):
    """Editing an accepted path discards its waypoints, fit, and publishable reference."""
    model, messages = _configured_grid_module(5)
    model.SelfInit()
    model.Reset(0)
    output = model.attRefOutMsg.addSubscriber()
    step = macros.sec2nano(1)  # [ns]
    model.UpdateState(step)
    path = model.path
    if edit == "replace":
        model.path = constrainedAttitudeManeuver.NodeList()
    elif edit == "replace_same_length":
        replacement = constrainedAttitudeManeuver.NodeList()
        for index in reversed(range(path.N)):
            node = constrainedAttitudeManeuver.Node()
            node.sigma_BN = [model.returnPathCoord(index, axis) for axis in range(3)]
            replacement.append(node)
        assert replacement.N == path.N
        model.path = replacement
        np.testing.assert_array_equal([model.returnPathCoord(0, axis) for axis in range(3)], model.sigma_BN_goal)
    elif edit == "append":
        path.append(constrainedAttitudeManeuver.Node())
    elif edit == "pop":
        path.pop(0)
    elif edit == "swap":
        path.swap(0, 1)
    else:
        path.clear()
    assert np.size(model.Input.X1) == np.size(model.Output.T) == 0
    assert np.isnan(model.pathCost)
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(2 * step)
    assert output.timeWritten() == step
    model.Reset(2 * step)
    model.UpdateState(2 * step)
    assert output.timeWritten() == 2 * step


def test_unchanged_path_preserves_reference():
    """Self-assignment, an unchanged ordering, and rejected edits preserve an accepted fit."""
    model, messages = _configured_grid_module(5)
    model.SelfInit()
    model.Reset(0)
    cost = model.pathCost
    model.path = model.path
    model.path.swap(0, 0)
    with pytest.raises(bskLogging.BasiliskError, match="out of range"):
        model.path.pop(model.path.N)
    other, other_messages = _configured_grid_module(5)
    other.Reset(0)
    with pytest.raises(bskLogging.BasiliskError, match="grid-backed or borrowed"):
        model.path = other.path
    assert model.effortEvaluation() == pytest.approx(cost)
    assert model.pathCost == cost
    model.UpdateState(0)


def test_sort_only_invalidates_reordered_planner_path():
    """Sorting a copy preserves the fit, while reordering the planner's own path clears it."""
    model, messages = _configured_grid_module(5)
    model.ReadInputs()
    nodes = constrainedAttitudeManeuver.NodeList()
    for index in range(2):
        node = constrainedAttitudeManeuver.Node()
        node.sigma_BN = [0.0, 0.25 * index, 0.0]  # MRPs [-]
        node.priority = 1 - index
        nodes.append(node)
    model.path = nodes
    model.pathHandle()
    model.spline()
    copied = constrainedAttitudeManeuver.NodeList(model.path)
    copied.sort()
    model.path = nodes
    assert np.isfinite(model.effortEvaluation())
    model.path.sort()
    assert model.returnPathCoord(0, 1) == 0.25
    assert np.size(model.Input.X1) == np.size(model.Output.T) == 0
    model.pathHandle()
    model.spline()
    model.path.sort()
    assert np.isfinite(model.effortEvaluation())


@pytest.mark.parametrize("args", [(), ([0.0, 1.0], [0.0, 0.0], [0.0, 0.0])])  # MRPs [-]
def test_optional_spline_values_survive_input_copy(args):
    """Both constructors zero optional values and copying a partial input preserves its settings."""
    data = BSpline.InputDataSet(*args)
    derivatives = ("XDot_0", "XDot_N", "XDDot_0", "XDDot_N")
    for field in derivatives:
        np.testing.assert_array_equal(np.ravel(getattr(data, field)), np.zeros(3))
    rate = [0.1, 0.2, 0.3]  # MRP rate [1/s]
    data.setXDot_0(rate)
    model = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver()
    model.Input = data
    data.setXDot_0(np.zeros(3))
    copied = model.Input
    assert copied.AvgXDot == 0.0 and not copied.AvgXDot_flag
    np.testing.assert_array_equal(np.ravel(copied.XDot_0), rate)
    assert copied.XDot_0_flag
    for field in derivatives[1:]:
        np.testing.assert_array_equal(np.ravel(getattr(copied, field)), np.zeros(3))
        assert not getattr(copied, field + "_flag")


def test_model_path_node_lifetime():
    """A path view keeps its model alive and native grid cleanup releases the appended node."""
    model, messages = _configured_grid_module(5)
    path_view = model.path
    model_ref = weakref.ref(model)
    node = constrainedAttitudeManeuver.Node()
    node.sigma_BN = [0.25, 0.0, 0.0]  # MRPs [-]
    handle = node.this
    count = sys.getrefcount(handle)
    model.path.append(weakref.proxy(node))
    del node, model
    gc.collect()
    assert model_ref() is not None and sys.getrefcount(handle) == count
    assert model_ref().returnPathCoord(0, 0) == 0.25
    _generate_test_grid(model_ref())
    assert path_view.N == 0 and sys.getrefcount(handle) == count - 1
    del path_view
    gc.collect()
    assert model_ref() is None


def test_path_coordinate_index_bounds():
    """Return the existing missing-coordinate sentinel for either invalid index."""
    model, messages = _configured_grid_module(5)
    model.Reset(0)
    for waypoint, axis in ((-1, 0), (model.path.N, 0), (0, -1), (0, 3)):
        assert model.returnPathCoord(waypoint, axis) == 1000
    np.testing.assert_array_equal([model.returnPathCoord(0, i) for i in range(3)], messages[0].read().sigma_BN)
    np.testing.assert_array_equal([model.returnPathCoord(model.path.N - 1, i) for i in range(3)], model.sigma_BN_goal)


@pytest.mark.parametrize("kind", ["In", "Out"])
@pytest.mark.parametrize("axis, angle, error", [  # [-], [rad]
    ([0.0, 0.0, 0.0], 0.4, "boresight"),
    ([1.0, np.nan, 0.0], 0.4, "boresight"),
    ([np.inf, 0.0, 0.0], 0.4, "boresight"),
    ([1.0, 0.0, 0.0], np.nan, "half-angle"),
    ([1.0, 0.0, 0.0], np.inf, "half-angle"),
    ([1.0, 0.0, 0.0], -0.1, "half-angle"),
    ([1.0, 0.0, 0.0], np.pi + 0.1, "half-angle"),
])
def test_invalid_boresight_cone(kind, axis, angle, error):
    """Reject an invalid append and detect the same cone when set through the public arrays."""
    model, messages = _configured_grid_module(5)
    count = f"keep{kind}BoresightCount"
    slot = getattr(model.boresights, count)
    diagnostic = f"keep-{kind.lower()}.*{error}"
    with pytest.raises(bskLogging.BasiliskError, match=diagnostic):
        getattr(model, f"appendKeep{kind}Direction")(axis, angle)
    assert getattr(model.boresights, count) == slot

    model.Reset(0)
    axes = [[1.0, 0.0, 0.0] for _ in range(10)]  # [-]
    angles = [0.4] * 10  # [rad]
    axes[slot], angles[slot] = axis, angle
    setattr(model.boresights, f"keep{kind}Boresight_B", axes)
    setattr(model.boresights, f"keep{kind}Fov", angles)
    setattr(model.boresights, count, slot + 1)
    with pytest.raises(bskLogging.BasiliskError, match=diagnostic):
        model.Reset(0)
    assert not model.constraints.keepIn and not model.constraints.keepOut
    with pytest.raises(bskLogging.BasiliskError, match="requires a successful reset"):
        model.UpdateState(0)


@pytest.mark.parametrize("kind", ["In", "Out"])
@pytest.mark.parametrize("public_arrays", [False, True], ids=["append", "arrays"])
@pytest.mark.parametrize("offset", [-1e-11, 0.0, 1e-11], ids=["outside", "rim", "inside"])  # [rad]
def test_cone_boundary(kind, public_arrays, offset):
    """Include the rim in either cone and distinguish goals just to either side."""
    model, messages = _configured_grid_module(5)
    model.sigma_BN_goal = [0.0, 0.0, 0.0]
    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    half_angle = np.pi / 4 + offset  # [rad]
    if public_arrays:
        # Rotating [4, 9, 0] by 45 degrees gives a direction proportional to [-5, 13, 0].
        planet = messages[2].read()
        planet.PositionVector = np.array(messages[0].read().r_BN_N) + [4.0, 9.0, 0.0]  # [m]
        messages[2].write(planet)
        axis = np.array([-5.0, 13.0, 0.0])  # [-]
        axis *= (1.0 - 4e-13) / np.linalg.norm(axis)  # Within the stored unit-vector tolerance.
        setattr(model.boresights, f"keep{kind}Boresight_B", [axis.tolist()] * 10)
        setattr(model.boresights, f"keep{kind}Fov", [half_angle] * 10)
        setattr(model.boresights, f"keep{kind}BoresightCount", 1)
    else:
        getattr(model, f"appendKeep{kind}Direction")([3.0, 3.0, 0.0], half_angle)

    inside = offset >= 0.0
    if inside == (kind == "In"):
        model.Reset(0)
        assert model.returnNodeState(model.keyG)
    else:
        with pytest.raises(bskLogging.BasiliskError, match="target attitude violates"):
            model.Reset(0)


@pytest.mark.parametrize("separation, half_angle, blocked", [  # [rad], [rad]
    (0.0, 5e-7, True), (0.0, 0.0, True), (1e-9, 0.0, False),
])
def test_narrow_keep_out_cone(separation, half_angle, blocked):
    """An accepted public boresight length must not change the angular separation."""
    model, messages = _configured_grid_module(5)
    model.boresights.keepInBoresightCount = 0
    axis = (1.0 - 4e-13) * np.array([np.cos(separation), np.sin(separation), 0.0])  # [-]
    model.boresights.keepOutBoresight_B = [axis.tolist()] * 10
    model.boresights.keepOutFov = [half_angle] * 10
    _generate_test_grid(model)
    assert model.returnNodeState([0, 0, 0]) == (not blocked)


@pytest.mark.parametrize("kind", ["In", "Out"])
def test_stale_cone_activation_on_grid_rebuild(kind):
    """Require fresh inputs after enabling a cone by count or disabling it by subscription."""
    model, messages = _configured_grid_module(5)
    model.sigma_BN_goal = [0.0, 0.0, 0.0]
    model.boresights.keepInBoresightCount = model.boresights.keepOutBoresightCount = 0
    endpoints = _generate_test_grid(model)
    setattr(model.boresights, f"keep{kind}BoresightCount", 1)
    with pytest.raises(bskLogging.BasiliskError, match="ReadInputs"):
        model.GenerateGrid(*endpoints)
    assert model.keyS == model.keyG == [model.invalidGridIndex] * 3
    model.ReadInputs()
    with pytest.raises(bskLogging.BasiliskError, match="target attitude violates"):
        model.GenerateGrid(*endpoints)

    getattr(model, f"keep{kind}CelBodyInMsg").unsubscribe()
    with pytest.raises(bskLogging.BasiliskError, match="ReadInputs"):
        model.GenerateGrid(*endpoints)
    model.ReadInputs()
    model.GenerateGrid(*endpoints)
    assert model.returnNodeState(model.keyG)


@pytest.mark.parametrize("kind", ["In", "Out"])
@pytest.mark.parametrize("axis", [[0.0, 0.0, 0.0], [1.0, np.nan, 0.0], [np.inf, 0.0, 0.0]])
def test_undefined_inertial_axis_on_rebuild(kind, axis):
    """Reject an undefined active inertial axis supplied directly to grid generation."""
    model, messages = _configured_grid_module(5)
    endpoints = _generate_test_grid(model)
    setattr(model.constraints, f"keep{kind}Dir_N", axis)
    with pytest.raises(bskLogging.BasiliskError, match=f"keep-{kind.lower()}.*inertial"):
        model.GenerateGrid(*endpoints)
    assert model.keyS == model.keyG == [model.invalidGridIndex] * 3


def test_rebuild_reclassifies_endpoints(capfd):
    """Reuse a formerly free start and a formerly blocked goal after changing the boresight."""
    model, messages = _configured_grid_module(5)
    model.bskLogger.setLogLevel(bskLogging.BSK_WARNING)
    model.boresights.keepInBoresightCount = 0
    state = messages[0].read()
    state.sigma_BN = [0.0, 0.0, np.tan(np.pi / 8)]  # MRPs [-], 90 degrees about +z
    messages[0].write(state)
    blocked_start_warning = "the initial attitude of the S/C is not constraint-compliant"
    start, goal = _generate_test_grid(model)
    assert blocked_start_warning not in capfd.readouterr().out
    start.isFree = model.returnNodeState(model.keyS)
    goal.sigma_BN = [0.05, 0.0, 0.0]  # MRPs [-], off-grid rotation about the original keep-out boresight.
    goal.isFree = model.returnNodeState([0, 0, 0])
    assert start.isFree and not goal.isFree

    model.boresights.keepOutBoresight_B = [[0.0, -1.0, 0.0]] * 10  # [-] Points at the source at the start.
    model.GenerateGrid(start, goal)
    assert capfd.readouterr().out.count(blocked_start_warning) == 1
    assert not model.returnNodeState(model.keyS)
    assert model.returnNodeState(model.keyG)
    np.testing.assert_array_equal([model.returnNodeCoord(model.keyG, axis) for axis in range(3)], goal.sigma_BN)


@pytest.mark.parametrize("kind", ["In", "Out"])
def test_rebuild_after_source_replacement(kind):
    """Use the cached direction until ReadInputs samples the replacement source."""
    model, messages = _configured_grid_module(5)
    model.sigma_BN_goal = [0.0, 0.0, 0.0]
    model.boresights.keepInBoresightCount = int(kind == "In")
    model.boresights.keepOutBoresightCount = int(kind == "Out")
    planet = messages[2].read()
    planet.PositionVector = [1.0, 10.0, 0.0]  # [m] The initial line of sight is +y.
    messages[2].write(planet)
    endpoints = _generate_test_grid(model)

    planet.PositionVector = [11.0, 0.0, 0.0]  # [m] The replacement line of sight is +x.
    replacement = messaging.SpicePlanetStateMsg().write(planet)
    getattr(model, f"keep{kind}CelBodyInMsg").subscribeTo(replacement)
    model.GenerateGrid(*endpoints)
    assert model.returnNodeState(model.keyG)
    model.ReadInputs()
    with pytest.raises(bskLogging.BasiliskError, match="target attitude violates"):
        model.GenerateGrid(*endpoints)


@pytest.mark.parametrize("kind", ["In", "Out"])
def test_nonunit_boresight_on_grid_rebuild(kind):
    """A direct rebuild must recheck stored unit vectors after inputs were read."""
    model, messages = _configured_grid_module(5)
    endpoints = _generate_test_grid(model)
    setattr(model.boresights, f"keep{kind}Boresight_B", [[0.5, 0.0, 0.0]] * 10)  # [-]
    with pytest.raises(bskLogging.BasiliskError, match=f"keep-{kind.lower()}.*boresight"):
        model.GenerateGrid(*endpoints)
    getattr(model, f"keep{kind}CelBodyInMsg").unsubscribe()
    model.ReadInputs()
    assert not getattr(model.constraints, f"keep{kind}")


@pytest.mark.parametrize("kind", ["In", "Out"])
def test_boresight_capacity(kind):
    """Accept the final boresight slot and reject invalid counts before array access."""
    model, messages = _configured_grid_module(5)
    count = f"keep{kind}BoresightCount"
    fov = f"keep{kind}Fov"
    append = getattr(model, f"appendKeep{kind}Direction")
    setattr(model.boresights, count, 0)
    angles = np.linspace(0.0, np.pi, 10)  # [rad], including both valid endpoints
    for angle in angles:
        append([3.0, 4.0, 0.0], angle)  # [-]
    with pytest.raises(bskLogging.BasiliskError, match="cannot append"):
        append([0.0, 1.0, 0.0], np.pi)
    assert getattr(model.boresights, count) == len(angles)
    np.testing.assert_array_equal(getattr(model.boresights, fov), angles)
    np.testing.assert_allclose(getattr(model.boresights, f"keep{kind}Boresight_B"), [[0.6, 0.8, 0.0]] * 10)

    for invalid in (-1, len(angles) + 1):
        setattr(model.boresights, count, invalid)
        with pytest.raises(bskLogging.BasiliskError, match="cannot append"):
            append([0.0, 1.0, 0.0], np.pi)
        with pytest.raises(bskLogging.BasiliskError, match="invalid keep-.* boresight count"):
            model.Reset(0)


def test_node_list_sorts_large_priorities():
    """Sort nodes on both sides of the former 100,000 priority cutoff."""
    queue = constrainedAttitudeManeuver.NodeList()
    nodes = [constrainedAttitudeManeuver.Node() for _ in range(3)]
    for node, priority in zip(nodes, (2e5, 1e5, 1.0)):
        node.priority = priority
        queue.append(node)
    queue.sort()
    for node in reversed(nodes):
        assert queue.contains(node)
        queue.pop(0)
        assert not queue.contains(node)


# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.
@pytest.mark.parametrize("N", [5,6])
@pytest.mark.parametrize("keepOutFov", [20])
@pytest.mark.parametrize("keepInFov", [70])
@pytest.mark.parametrize("costFcnType", [0,1])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_constrainedAttitudeManeuver(show_plots, N, keepOutFov, keepInFov, costFcnType, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the path computed by the ConstrainedAttitudeManeuver module.
    The module is tested against Python scripts that mirror the same functions contained in the module. Tests are run for
    different grid coarseness levels, different keep-out fields of view, and one keep-in field of view.

    **Test Parameters**

    Args:
        N (int) : grid coarseness;
        keepOutFov (float) : Field of View (in radiants) of the keep-out boresight;
        keepInFov (float) : Field of View (in radiants) of the keep-in boresight;
        costFcnType (int) : 0 for the MRP distance-based graph search, 1 for the effort-based graph search.
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    The tests to show the correctness of the module are the following:

    - First of all, an equivalent grid is built in python. The first test consists in comparing the nodes generated
      in Python versus the nodes generated in C++. The check consists in verifying whether the same key indices
      :math:`(i,j,k)` generate the same node coordinates :math:`\sigma_{BN}`. Secondly, it is checked whether
      the same node is constraint-compliant or -incompliant both in Python and in C++.

    - After running the graph-search algorithm, a check is conduced to ensure the equivalence of the computed paths.
      Note that this unit test does not run the effort-based version of A*, due to the slow nature of the Python
      implementation. If the user wishes, it is possible to uncomment line 429 to also test the effort-based
      graph-search algorithm.

    - The interpolated trajectory obtained in Python is checked versus the interpolated trajectory
      obtained in C++. The Python code uses the BSK-native :ref:`BSpline` library, which has its own unit test.

    - Lastly, a check is run on the norm of the required control torque for each time step of the
      interpolated trajectory.  The correctness of this check should imply the correctness of the
      functions used in the effort-based graph-search algorithm as well.
    """

    # each test method requires a single assert method to be called
    [testResults, testMessage] = CAMTestFunction(N, keepOutFov, keepInFov, costFcnType, accuracy)

    assert testResults < 1, testMessage

def CAMTestFunction(N, keepOutFov, keepInFov, costFcnType, accuracy):

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    Inertia = [0.02 / 3,  0.,         0.,
               0.,        0.1256 / 3, 0.,
               0.,        0.,         0.1256 / 3]
    InertiaTensor = simHelpers.np2EigenMatrix3d(Inertia)
    PlanetInertialPosition = np.array([10, 0, 0])
    SCInertialPosition = np.array([1, 0, 0])
    SCInitialAttitude = np.array([0, 0, -0.5])
    SCTargetAttitude = np.array([0, 0.5, 0])
    SCInitialAngRate = np.array([0, 0, 0])
    SCTargetAngRate = np.array([0, 0, 0])
    SCAvgAngRate = 0.03
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
    nodes = generateGrid(n_start, n_goal, N, constraints, data)
    if costFcnType == 0:
        path = AStar(nodes, n_start, n_goal)
    else:
        path = effortBasedAStar(nodes, n_start, n_goal, SCInitialAngRate, SCTargetAngRate, SCAvgAngRate, InertiaTensor)
    Input = pathHandle(path, SCAvgAngRate)
    Output = spline(Input, SCInitialAngRate, SCTargetAngRate)
    pathCost = effortEvaluation(Output, InertiaTensor)

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
    testModule.avgOmega = SCAvgAngRate
    testModule.BSplineType = 0
    testModule.costFcnType = costFcnType
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
    VehicleConfigMsgData.ISCPntB_B = Inertia
    VehicleConfigMsg = messaging.VehicleConfigMsg().write(VehicleConfigMsgData)
    PlanetStateMsgData = messaging.SpicePlanetStateMsgPayload()
    PlanetStateMsgData.PositionVector = PlanetInertialPosition
    PlanetStateMsg = messaging.SpicePlanetStateMsg().write(PlanetStateMsgData)
    testModule.scStateInMsg.subscribeTo(SCStatesMsg)
    testModule.vehicleConfigInMsg.subscribeTo(VehicleConfigMsg)
    testModule.keepOutCelBodyInMsg.subscribeTo(PlanetStateMsg)
    testModule.keepInCelBodyInMsg.subscribeTo(PlanetStateMsg)

    numDataPoints = 200
    samplingTime = simHelpers.samplingTime(simulationTime, testProcessRate, numDataPoints)

    CAMLog = testModule.attRefOutMsg.recorder(samplingTime)
    unitTestSim.AddModelToTask(unitTaskName, CAMLog)
    CAMLogC = testModule.attRefOutMsgC.recorder(samplingTime)
    unitTestSim.AddModelToTask(unitTaskName, CAMLogC)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(simulationTime)        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    timeData = CAMLog.times() * macros.NANO2SEC
    print(timeData)
    # print(len(CAMLog.sigma_RN))

    # check correctness of grid points:
    for i in range(-N,N+1):
        for j in range(-N,N+1):
            for k in range(-N,N+1):
                if (i, j, k) in nodes:
                    sigma_BN = nodes[(i, j, k)].sigma_BN
                    sigma_BN_BSK = []
                    for p in range(3):
                        sigma_BN_BSK.append( testModule.returnNodeCoord([i, j, k], p) )
                    if not unitTestSupport.isVectorEqual(sigma_BN, sigma_BN_BSK, accuracy):
                        testFailCount += 1
                        testMessages.append("FAILED: " + testModule.ModelTag + " Error in the coordinates of node ({},{},{}) \n".format(i, j, k))
                    if not nodes[(i, j, k)].isFree == testModule.returnNodeState([i, j, k]):
                        testFailCount += 1
                        testMessages.append("FAILED: " + testModule.ModelTag + " Error in the state of node ({},{},{}) \n".format(i, j, k))

    # check that the same path is produced
    for p in range(len(path)):
        sigma_BN = path[p].sigma_BN
        sigma_BN_BSK = []
        for j in range(3):
            sigma_BN_BSK.append(testModule.returnPathCoord(p,j))
        if not unitTestSupport.isVectorEqual(sigma_BN, sigma_BN_BSK, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in waypoint number {} in path \n".format(p))

    # check interpolated path compliance
    for n in range(len(Output.T)):
        T_BSK = testModule.Output.T[n][0]
        sigma_BSK = np.array([testModule.Output.X1[n][0], testModule.Output.X2[n][0], testModule.Output.X3[n][0]])
        sigmaDot_BSK = np.array([testModule.Output.XD1[n][0], testModule.Output.XD2[n][0], testModule.Output.XD3[n][0]])
        sigmaDDot_BSK = np.array([testModule.Output.XDD1[n][0], testModule.Output.XDD2[n][0], testModule.Output.XDD3[n][0]])

        T         = Output.T[n][0]
        sigma     = np.array([Output.X1[n][0], Output.X2[n][0], Output.X3[n][0]])
        sigmaDot  = np.array([Output.XD1[n][0], Output.XD2[n][0], Output.XD3[n][0]])
        sigmaDDot = np.array([Output.XDD1[n][0], Output.XDD2[n][0], Output.XDD3[n][0]])

        if not unitTestSupport.isDoubleEqual(T, T_BSK, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in time at index #{} in trajectory \n".format(n))
        if not unitTestSupport.isVectorEqual(sigma, sigma_BSK, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in sigma at index #{} in trajectory \n".format(n))
        if not unitTestSupport.isVectorEqual(sigmaDot, sigmaDot_BSK, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in sigmaDot at index #{} in trajectory \n".format(n))
        if not unitTestSupport.isVectorEqual(sigmaDDot, sigmaDDot_BSK, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in sigmaDDot at index #{} in trajectory \n".format(n))

    # check same path cost for every spline point
    for n in range(len(Output.T)):
        c1 = testModule.computeTorqueNorm(n, Inertia)

        sigma     = [Output.X1[n][0], Output.X2[n][0], Output.X3[n][0]]
        sigmaDot  = [Output.XD1[n][0], Output.XD2[n][0], Output.XD3[n][0]]
        sigmaDDot = [Output.XDD1[n][0], Output.XDD2[n][0], Output.XDD3[n][0]]
        c2 = np.linalg.norm(computeTorque(sigma, sigmaDot, sigmaDDot, InertiaTensor))

        if not unitTestSupport.isDoubleEqual(c1, c2, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error torque norm in point {} in trajectory \n".format(n))

    if not unitTestSupport.isDoubleEqual(pathCost, testModule.pathCost, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + testModule.ModelTag + " Error in path cost \n")

    # check correctness of output message
    for n in range(len(timeData)):
        t = timeData[n]
        sigma_RN     = [Output.getStates(t,0,0), Output.getStates(t,0,1), Output.getStates(t,0,2)]
        sigmaDot_RN  = [Output.getStates(t,1,0), Output.getStates(t,1,1), Output.getStates(t,1,2)]
        sigmaDDot_RN = [Output.getStates(t,2,0), Output.getStates(t,2,1), Output.getStates(t,2,2)]
        RN = rbk.MRP2C(sigma_RN)
        omega_RN_R = rbk.dMRP2Omega(sigma_RN, sigmaDot_RN)
        omega_RN_N = np.matmul(omega_RN_R, RN)
        omegaDot_RN_R = rbk.ddMRP2dOmega(sigma_RN, sigmaDot_RN, sigmaDDot_RN)
        omegaDot_RN_N = np.matmul(omegaDot_RN_R, RN)

        if not unitTestSupport.isVectorEqual(sigma_RN, CAMLog.sigma_RN[n], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in attitude reference message at t = {} sec \n".format(t))
        if not unitTestSupport.isVectorEqual(omega_RN_N, CAMLog.omega_RN_N[n], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in attitude reference message at t = {} sec \n".format(t))
        if not unitTestSupport.isVectorEqual(omegaDot_RN_N, CAMLog.domega_RN_N[n], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in attitude reference message at t = {} sec \n".format(t))

        if not unitTestSupport.isVectorEqual(sigma_RN, CAMLogC.sigma_RN[n], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in C attitude reference message at t = {} sec \n".format(t))
        if not unitTestSupport.isVectorEqual(omega_RN_N, CAMLogC.omega_RN_N[n], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in C attitude reference message at t = {} sec \n".format(t))
        if not unitTestSupport.isVectorEqual(omegaDot_RN_N, CAMLogC.domega_RN_N[n], accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + testModule.ModelTag + " Error in C attitude reference message at t = {} sec \n".format(t))


    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    CAMTestFunction(
        5,      # grid coarsness N
        20,      # keepOutFov
        70,      # keepInFov
        1,       # costFcnType
        1e-12    # accuracy
        )
