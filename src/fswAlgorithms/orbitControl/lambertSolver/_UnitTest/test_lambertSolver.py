#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
import inspect
import os
import sys
import copy
import itertools

import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertSolver
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + "/Support")
from IzzoLambert import *

# parameters
solver = ["Gooding", "Izzo"]
revs = [0, 1, 4]
times = [1e2, 1e6]
eccentricities = [0.0, 0.05, 1.0, 1.2]
transferAngle = [30., 90., 180., 210., -60., 500.]
alignmentThreshold = [1.]

paramArray = [solver, revs, times, eccentricities, transferAngle, alignmentThreshold]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))
# transfer angles >= 180 or < 0 not applicable for parabolic and hyperbolic transfer. Delete from list.
paramList = [item for item in paramList if not (item[3] >= 1.0 and (item[4] >= 180. or item[4] < 0.))]
# add cases to check alignmentThreshold module parameter
paramList.append((solver[0], revs[0], times[0], eccentricities[0], 0.5, 1.))
paramList.append((solver[0], revs[0], times[0], eccentricities[0], 1.5, 1.))
paramList.append((solver[0], revs[0], times[0], eccentricities[0], 1.5, 2.))
paramList.append((solver[0], revs[0], times[0], eccentricities[0], 2.5, 2.))

@pytest.mark.parametrize("accuracy", [1e-2])
@pytest.mark.parametrize("p1_solver, p2_revs, p3_times, p4_eccs, p5_angles, p6_align", paramList)

def test_lambertSolver(show_plots, p1_solver, p2_revs, p3_times, p4_eccs, p5_angles, p6_align, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the Lambert solver module works correctly for different Lambert solver algorithms,
    number of revolutions, time of flight, transfer orbit eccentricities, transfer angles, and position vector
    alignment threshold. The Lambert solver module is tested using the orbitalMotion libraries
    (for zero revolution scenarios), and using an external python implementation of the Izzo algorithm
    (for multi-revolution scenarios).

    **Test Parameters**

    Args:
        show_plots: flag if plots should be shown
        p1_solver: Lambert solver algorithm
        p2_revs: number of revolutions to be completed
        p3_times: time-of-flight (transfer time)
        p4_eccs: eccentricity of transfer orbit
        p5_angles: transfer angle
        p6_align: threshold for transfer angle being considered too small

    **Description of Variables Being Tested**

    The computed velocity vectors at position 1 and position 2 are compared to the true velocity vectors.
    For the zero-revolution case, the eccentricity and transfer angle parameters are used to determine the
    time-of-flight input for the Lambert solver, and the corresponding orbit elements are used to determine the true
    velocity vectors for the given position vectors. For the multi-revolution case, the time-of-flight parameter is
    used as the input for the Lambert solver, and the true velocity vectors are computed using an external
    Python script that uses Izzo's algorithm to solve Lambert's problem.

    For the multi-revolution case, the solution for the free variable x is also tested.
    """
    lambertSolverTestFunction(show_plots, p1_solver, p2_revs, p3_times, p4_eccs, p5_angles, p6_align, accuracy)


def lambertSolverTestFunction(show_plots, p1_solver, p2_revs, p3_times, p4_eccs, p5_angles, p6_align, accuracy):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = lambertSolver.LambertSolver()
    module.ModelTag = "lambertSolver"
    if p6_align != 1.0:
        # alignmentThreshold equals 1.0 by default so does not have to be specified if equal to 1.0
        module.setAlignmentThreshold(p6_align)
    unitTestSim.AddModelToTask(unitTaskName, module)

    # set up the transfer orbit using classical orbit elements
    mu = 3.986004418e14
    oe1 = orbitalMotion.ClassicElements()
    r = 10000. * 1000  # meters
    if p4_eccs < 1.0:
        # elliptic case
        oe1.a = r
    else:
        # parabolic and hyperbolic case
        oe1.a = -r
    oe1.e = p4_eccs
    oe1.i = 5. * macros.D2R
    oe1.Omega = 25. * macros.D2R
    oe1.omega = 30. * macros.D2R
    oe1.f = 10. * macros.D2R
    r1_N, v1_N = orbitalMotion.elem2rv_parab(mu, oe1)

    oe2 = copy.deepcopy(oe1)
    oe2.f = oe1.f + p5_angles * macros.D2R
    # Izzo and Gooding Lambert algorithms only consider positive transfer angles. Convert to 0 < angle < 2pi
    oe2.f = (oe2.f*macros.R2D % 360) * macros.D2R
    r2_N, v2_N = orbitalMotion.elem2rv_parab(mu, oe2)

    # determine time-of-flight for given transfer orbit and position vectors (true anomalies)
    if p2_revs > 0:
        # for multi-revolution case use time-of-flight from parameterization
        t_transfer = p3_times
    elif p4_eccs < 1.0:
        # elliptic case
        M1 = orbitalMotion.E2M(orbitalMotion.f2E(oe1.f, p4_eccs), p4_eccs)
        M2 = orbitalMotion.E2M(orbitalMotion.f2E(oe2.f, p4_eccs), p4_eccs)
        n = np.sqrt(mu/(oe1.a)**3)
        t_transfer = np.abs(M2-M1)/n
    elif p4_eccs == 1.0:
        # parabolic case
        D1 = np.tan(oe1.f/2)
        D2 = np.tan(oe2.f/2)
        M1 = D1 + 1/3*D1**3
        M2 = D2 + 1/3*D2**3
        n = np.sqrt(mu/(2*(-oe1.a)**3))
        t_transfer = np.abs(M2-M1)/n
    else:
        # hyperbolic case
        N1 = orbitalMotion.H2N(orbitalMotion.f2H(oe1.f, p4_eccs), p4_eccs)
        N2 = orbitalMotion.H2N(orbitalMotion.f2H(oe2.f, p4_eccs), p4_eccs)
        n = np.sqrt(mu/(-oe1.a)**3)
        t_transfer = np.abs(N2-N1)/n

    solverName = p1_solver
    time = t_transfer
    revs = p2_revs

    if solverName == "Gooding":
        solverMethod = messaging.GOODING
    elif solverName == "Izzo":
        solverMethod = messaging.IZZO

    # Configure input messages
    lambertProblemInMsgData = messaging.LambertProblemMsgPayload()
    lambertProblemInMsgData.solverMethod = solverMethod
    lambertProblemInMsgData.r1_N = r1_N
    lambertProblemInMsgData.r2_N = r2_N
    lambertProblemInMsgData.transferTime = time
    lambertProblemInMsgData.mu = mu
    lambertProblemInMsgData.numRevolutions = revs
    lambertProblemInMsg = messaging.LambertProblemMsg().write(lambertProblemInMsgData)

    # subscribe input messages to module
    module.lambertProblemInMsg.subscribeTo(lambertProblemInMsg)

    # setup output message recorder objects
    lambertSolutionOutMsgRec = module.lambertSolutionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, lambertSolutionOutMsgRec)
    lambertPerformanceOutMsgRec = module.lambertPerformanceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, lambertPerformanceOutMsgRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    v1_N = lambertSolutionOutMsgRec.v1_N[0]
    v2_N = lambertSolutionOutMsgRec.v2_N[0]
    validFlag = lambertSolutionOutMsgRec.valid[0]
    v1Sol2_N = lambertSolutionOutMsgRec.v1Sol2_N[0]
    v2Sol2_N = lambertSolutionOutMsgRec.v2Sol2_N[0]
    validFlagSol2 = lambertSolutionOutMsgRec.validSol2[0]

    # for multi-revolution case, use external Lambert solver
    if revs > 0 and p5_angles != 180.:
        Izzo = IzzoSolve(np.array(r1_N), np.array(r2_N), time, mu, revs)
        Izzo.solve()
        numSolutions = len(Izzo.x)

    idx = 2 * revs - 1
    idxSol2 = idx + 1

    # if the transfer angle is smaller than alignmentThreshold, the two position vectors are too aligned.
    # They might not define a plane, so no solution should be returned.
    if abs(np.sin(p5_angles*macros.D2R)) < abs(np.sin(p6_align*macros.D2R)):
        alignmentFlag = True
    else:
        alignmentFlag = False

    if alignmentFlag or (revs > 0 and idx+1 > numSolutions):
        # 1. if transfer angle is smaller than alignmentThreshold, no solution should be returned
        # 2. external Lambert solver does not compute solution if requested transfer time is less than minimum
        # time-of-flight for multi-revolution solution. In this case set all true outputs to zero
        # (so does the lambert module as well, since no solution exists for the requested time-of-flight)
        v1True_N = np.array([0., 0., 0.])
        v2True_N = np.array([0., 0., 0.])
        validFlagTrue = 0
        v1TrueSol2_N = np.array([0., 0., 0.])
        v2TrueSol2_N = np.array([0., 0., 0.])
        validFlagTrueSol2 = 0
    elif revs == 0:
        # for zero-revolution case, obtain true velocity vectors from computed transfer orbit
        v1True_N = v1_N
        v2True_N = v2_N
        validFlagTrue = 1
        v1TrueSol2_N = np.array([0., 0., 0.])
        v2TrueSol2_N = np.array([0., 0., 0.])
        validFlagTrueSol2 = 0
    else:
        # for multi-revolution case, obtain true velocity vectors from external Lambert solver script if solution exist
        v1True_N = Izzo.v1[idx]
        v2True_N = Izzo.v2[idx]
        validFlagTrue = 1
        v1TrueSol2_N = Izzo.v1[idxSol2]
        v2TrueSol2_N = Izzo.v2[idxSol2]
        validFlagTrueSol2 = 1

    # only compare solution for free variable x for multi-revolution case (for 0 revolution case,
    # external lambert solver is not called so no true value is available)
    if (alignmentFlag == False and revs > 0 and idx < numSolutions):
        x = lambertPerformanceOutMsgRec.x[0]
        xSol2 = lambertPerformanceOutMsgRec.xSol2[0]
        xTrue = Izzo.x[idx]
        xTrueSol2 = Izzo.x[idxSol2]
    else:
        x = 0.
        xSol2 = 0.
        xTrue = 0.
        xTrueSol2 = 0.

    # make sure module output data is correct
    paramsString = ' for solver={}, rev={}, time={}, eccentricity={}, angle={}, alignmentThreshold={}, ' \
                   'accuracy={}'.format(
                    str(p1_solver),
                    str(p2_revs),
                    str(p3_times),
                    str(p4_eccs),
                    str(p5_angles),
                    str(p6_align),
                    str(accuracy))

    np.testing.assert_allclose(v1_N,
                               v1True_N,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: v1_N,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(v2_N,
                               v2True_N,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: v2_N,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(validFlag,
                               validFlagTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: validFlag,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(v1Sol2_N,
                               v1TrueSol2_N,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: v1Sol2_N,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(v2Sol2_N,
                               v2TrueSol2_N,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: v2Sol2_N,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(validFlagSol2,
                               validFlagTrueSol2,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: validFlagSol2,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(x,
                               xTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: x,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(xSol2,
                               xTrueSol2,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: xSol2,' + paramsString),
                               verbose=True)


if __name__ == "__main__":
    test_lambertSolver(False, solver[1], revs[0],
                       times[1], eccentricities[1], transferAngle[0], alignmentThreshold[0], 1e-2)
