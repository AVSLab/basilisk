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
import copy
import itertools
import math

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertValidator
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# parameters
DVs = np.array([[1., 3., 4.], [500., -100., 200.]])
time_maneuver = [1.e3, 2.e3]
time_final = [2.1e3, 4.e3]
iterations = [3]
errorsX = [1e-9]

paramArray = [DVs, time_maneuver, time_final, iterations, errorsX]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))
# add cases to check Lambert solver iterations and error limits, plus final time occurring before maneuver time
paramList.append((DVs[0], 1.e3, 2.e3, 7, 1.e-9))
paramList.append((DVs[0], 1.e3, 2.e3, 3, 1.e-5))
paramList.append((DVs[0], 2.e3, 1.e3, 3, 1.e-9))
paramList.append((DVs[0], 2.e3, 2.e3, 3, 1.e-9))

@pytest.mark.parametrize("accuracy", [1e-4])
@pytest.mark.parametrize("p1_dv, p2_tm, p3_tf, p4_iter, p5_errX", paramList)

def test_lambertValidator(show_plots, p1_dv, p2_tm, p3_tf, p4_iter, p5_errX, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the Lambert validator module works correctly for different Delta-V vectors, maneuver times and
    final times. Different values for the Lambert solver root finder iteration number and error are also used.
    If the iteration number or error is too large, the commanded Delta-V should be zero.

    **Test Parameters**

    Args:
        show_plots: flag if plots should be shown
        p1_dv: Delta-V vector
        p2_tm: maneuver time
        p3_tf: final time
        p4_iter: number of iterations of the Lambert solver root finder
        p5_errX: error of the Lambert solver root finder solution
        accuracy: accuracy of the test

    **Description of Variables Being Tested**

    The content of the DvBurnCmdMsg output message (DV vector and burn start time) is compared with the true values.
    """
    lambertValidatorTestFunction(show_plots, p1_dv, p2_tm, p3_tf, p4_iter, p5_errX, accuracy)


def getInitialStates(r1_BN_N, v1_BN_N, dv_N, errStates, errDV):
    # state and DV vector expressed in Hill frame
    dcm_HN = orbitalMotion.hillFrame(r1_BN_N, v1_BN_N)
    r1_BN_H = dcm_HN.dot(r1_BN_N)
    v1_BN_H = dcm_HN.dot(v1_BN_N)
    X1nom_H = np.hstack((r1_BN_H, v1_BN_H))
    dv_H = dcm_HN.dot(dv_N)
    dvHat_H = dv_H / np.linalg.norm(dv_H)

    # 6x6 dcm that can be used for 6x1 state vector
    dcm_HN_states = np.kron(np.eye(2), dcm_HN)

    # state uncertainty matrix
    Psqrt = errStates

    numInitialStates = 27
    N = 6

    initialStates = np.zeros((N, numInitialStates))
    for ii in range(N):
        for jj in range(2):
            if jj == 0:
                multiplier = 1.
            else:
                multiplier = -1.

            # add state uncertainty
            X1_H = X1nom_H + multiplier*Psqrt[:, ii]
            X1minDV_H = copy.deepcopy(X1_H)
            X1maxDV_H = copy.deepcopy(X1_H)

            # add DV uncertainty
            X1minDV_H[3:6] += dv_H - errDV * dvHat_H
            X1maxDV_H[3:6] += dv_H + errDV * dvHat_H

            initialStates[:, jj*N + ii] = dcm_HN_states.transpose().dot(X1minDV_H)
            initialStates[:, 2*N + jj*N + ii] = dcm_HN_states.transpose().dot(X1maxDV_H)

    X1_H = copy.deepcopy(X1nom_H)
    X1minDV_H = copy.deepcopy(X1_H)
    X1maxDV_H = copy.deepcopy(X1_H)

    # no state uncertainty, only DV uncertainty
    X1_H[3:6] += dv_H
    X1minDV_H[3:6] += dv_H - errDV * dvHat_H
    X1maxDV_H[3:6] += dv_H + errDV * dvHat_H

    initialStates[:, 4*N] = dcm_HN_states.transpose().dot(X1_H)
    initialStates[:, 4*N + 1] = dcm_HN_states.transpose().dot(X1minDV_H)
    initialStates[:, 4*N + 2] = dcm_HN_states.transpose().dot(X1maxDV_H)

    return initialStates

def countViolations(initialStates, muBody, tm, tf, r_TN_N, maxDistanceTarget, minOrbitRadius):
    violationsDistanceTarget = 0
    violationsOrbitRadius = 0

    numInitialStates = 27

    for ii in range(0, numInitialStates):
        Xm_N = initialStates[:, ii]
        rm_BN_N = Xm_N[0:3]
        vm_BN_N = Xm_N[3:6]
        oem = orbitalMotion.rv2elem_parab(muBody, rm_BN_N, vm_BN_N)

        oef = copy.deepcopy(oem)
        eccf = oem.e

        if eccf < 1.0:
            # elliptic case
            M2 = orbitalMotion.E2M(orbitalMotion.f2E(oem.f, eccf), eccf)
            n = np.sqrt(muBody / (oem.a) ** 3)
            M3 = M2 + n*(tf-tm)
            oef.f = orbitalMotion.E2f(orbitalMotion.M2E(M3, eccf), eccf)
        elif eccf == 1.0:
            # parabolic case
            D2 = np.tan(oem.f / 2)
            M2 = D2 + 1/3*D2**3
            n = np.sqrt(muBody / (2 * (-oem.a) ** 3))
            M3 = M2 + n*(tf-tm)
            A = 3./2.*M3
            B = (A + np.sqrt(A**2 + 1))**(1./3.)
            oef.f = 2. * np.arctan(B - 1. / B)
        else:
            # hyperbolic case
            N2 = orbitalMotion.H2N(orbitalMotion.f2H(oem.f, eccf), eccf)
            n = np.sqrt(muBody / (-oem.a) ** 3)
            N3 = N2 + n*(tf-tm)
            oef.f = orbitalMotion.H2f(orbitalMotion.N2H(N3, eccf), eccf)

        # spacecraft state at final time
        rf_BN_N, vf_BN_N = orbitalMotion.elem2rv_parab(muBody, oef)

        # difference of final location and target location
        rf_BT_N = rf_BN_N - r_TN_N

        # find the smallest orbit radius along transfer arc
        f1 = math.remainder(oem.f, 2*np.pi)
        f2 = math.remainder(oef.f, 2*np.pi)
        if f1*f2 < 0.:
            f_rmin = 0.
        else:
            f_rmin = min([abs(f1), abs(f2)])
        oe_rmin = copy.deepcopy(oem)
        oe_rmin.f = f_rmin
        rmin = oe_rmin.rmag

        if np.linalg.norm(rf_BT_N) > maxDistanceTarget:
            violationsDistanceTarget += 1

        if rmin < minOrbitRadius:
            violationsOrbitRadius += 1

    return violationsDistanceTarget, violationsOrbitRadius


def lambertValidatorTestFunction(show_plots, p1_dv, p2_tm, p3_tf, p4_iter, p5_errX, accuracy):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(100)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    solverMethod = messaging.IZZO
    muBody = 3.986004418e14
    t0 = 0.
    tm = p2_tm
    tf = p3_tf
    numIter = p4_iter
    errX = p5_errX
    ecc = 0.1
    dv_N = p1_dv
    errStates = np.diag([5., 5., 5., 0.01, 0.01, 0.001])  # 6x6 state uncertainty matrix
    errDV = 0.1  # [m/s] Delta-V magnitude uncertainty
    maxDistanceTarget = 3000.
    minOrbitRadius = 6378 * 1000.

    # set up orbit using classical orbit elements
    oe0 = orbitalMotion.ClassicElements()
    r = 10000. * 1000  # meters
    if ecc < 1.0:
        # elliptic case
        oe0.a = r
    else:
        # parabolic and hyperbolic case
        oe0.a = -r
    oe0.e = ecc
    oe0.i = 10. * macros.D2R
    oe0.Omega = 20. * macros.D2R
    oe0.omega = 30. * macros.D2R
    oe0.f = 40. * macros.D2R
    # spacecraft state at initial time
    r0_BN_N, v0_BN_N = orbitalMotion.elem2rv_parab(muBody, oe0)
    oe0 = orbitalMotion.rv2elem_parab(muBody, r0_BN_N, v0_BN_N)

    oe1 = copy.deepcopy(oe0)

    if ecc < 1.0:
        # elliptic case
        M0 = orbitalMotion.E2M(orbitalMotion.f2E(oe0.f, ecc), ecc)
        n = np.sqrt(muBody / (oe0.a) ** 3)
        M1 = M0 + n*(tm-t0)
        oe1.f = orbitalMotion.E2f(orbitalMotion.M2E(M1, ecc), ecc)
    elif ecc == 1.0:
        # parabolic case
        D0 = np.tan(oe0.f / 2)
        M0 = D0 + 1/3*D0**3
        n = np.sqrt(muBody / (2 * (-oe0.a) ** 3))
        M1 = M0 + n*(tm-t0)
        A = 3./2.*M1
        B = (A + np.sqrt(A**2 + 1))**(1./3.)
        oe1.f = 2. * np.arctan(B - 1. / B)
    else:
        # hyperbolic case
        N0 = orbitalMotion.H2N(orbitalMotion.f2H(oe0.f, ecc), ecc)
        n = np.sqrt(muBody / (-oe0.a) ** 3)
        N1 = N0 + n*(tm-t0)
        oe1.f = orbitalMotion.H2f(orbitalMotion.N2H(N1, ecc), ecc)

    # spacecraft state at maneuver time, just before maneuver
    r1_BN_N, v1_BN_N = orbitalMotion.elem2rv_parab(muBody, oe1)

    # spacecraft state at maneuver time, just after maneuver
    r2_BN_N = r1_BN_N
    v2_BN_N = v1_BN_N + dv_N
    oe2 = orbitalMotion.rv2elem_parab(muBody, r2_BN_N, v2_BN_N)

    oe3 = copy.deepcopy(oe2)
    ecc3 = oe3.e

    if ecc3 < 1.0:
        # elliptic case
        M2 = orbitalMotion.E2M(orbitalMotion.f2E(oe2.f, ecc3), ecc3)
        n = np.sqrt(muBody / (oe2.a) ** 3)
        M3 = M2 + n*(tf-tm)
        oe3.f = orbitalMotion.E2f(orbitalMotion.M2E(M3, ecc3), ecc3)
    elif ecc3 == 1.0:
        # parabolic case
        D2 = np.tan(oe2.f / 2)
        M2 = D2 + 1/3*D2**3
        n = np.sqrt(muBody / (2 * (-oe2.a) ** 3))
        M3 = M2 + n*(tf-tm)
        A = 3./2.*M3
        B = (A + np.sqrt(A**2 + 1))**(1./3.)
        oe3.f = 2. * np.arctan(B - 1. / B)
    else:
        # hyperbolic case
        N2 = orbitalMotion.H2N(orbitalMotion.f2H(oe2.f, ecc3), ecc3)
        n = np.sqrt(muBody / (-oe2.a) ** 3)
        N3 = N2 + n*(tf-tm)
        oe3.f = orbitalMotion.H2f(orbitalMotion.N2H(N3, ecc3), ecc3)

    # spacecraft state at final time
    r3_BN_N, v3_BN_N = orbitalMotion.elem2rv_parab(muBody, oe3)

    # setup module to be tested
    module = lambertValidator.LambertValidator()
    module.ModelTag = "lambertValidator"
    module.setFinalTime(tf)
    module.setManeuverTime(tm)
    module.setMaxDistanceTarget(maxDistanceTarget)
    module.setMinOrbitRadius(minOrbitRadius)
    module.setUncertaintyStates(errStates)
    module.setUncertaintyDV(errDV)
    module.setDvConvergenceTolerance(np.linalg.norm(dv_N)/1000.)
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure input messages
    navTransInMsgData = messaging.NavTransMsgPayload()
    navTransInMsgData.timeTag = t0
    navTransInMsgData.r_BN_N = r0_BN_N
    navTransInMsgData.v_BN_N = v0_BN_N
    navTransInMsg = messaging.NavTransMsg().write(navTransInMsgData)

    lambertProblemInMsgData = messaging.LambertProblemMsgPayload()
    lambertProblemInMsgData.solverMethod = solverMethod
    lambertProblemInMsgData.r1_N = r1_BN_N
    lambertProblemInMsgData.r2_N = r3_BN_N
    lambertProblemInMsgData.transferTime = tf-tm
    lambertProblemInMsgData.mu = muBody
    lambertProblemInMsgData.numRevolutions = 0
    lambertProblemInMsg = messaging.LambertProblemMsg().write(lambertProblemInMsgData)

    lambertSolutionInMsgData = messaging.LambertSolutionMsgPayload()
    lambertSolutionInMsgData.v1_N = v1_BN_N + dv_N
    lambertSolutionInMsgData.v2_N = v3_BN_N
    lambertSolutionInMsgData.valid = 1
    lambertSolutionInMsgData.v1Sol2_N = np.array([0., 0., 0.])
    lambertSolutionInMsgData.v2Sol2_N = np.array([0., 0., 0.])
    lambertSolutionInMsgData.validSol2 = 0
    lambertSolutionInMsg = messaging.LambertSolutionMsg().write(lambertSolutionInMsgData)

    lambertPerformanceInMsgData = messaging.LambertPerformanceMsgPayload()
    lambertPerformanceInMsgData.x = 0.
    lambertPerformanceInMsgData.numIter = numIter
    lambertPerformanceInMsgData.errX = errX
    lambertPerformanceInMsgData.xSol2 = 0.
    lambertPerformanceInMsgData.numIterSol2 = 0
    lambertPerformanceInMsgData.errXSol2 = 0.
    lambertPerformanceInMsg = messaging.LambertPerformanceMsg().write(lambertPerformanceInMsgData)

    # subscribe input messages to module
    module.navTransInMsg.subscribeTo(navTransInMsg)
    module.lambertProblemInMsg.subscribeTo(lambertProblemInMsg)
    module.lambertPerformanceInMsg.subscribeTo(lambertPerformanceInMsg)
    module.lambertSolutionInMsg.subscribeTo(lambertSolutionInMsg)

    # setup output message recorder objects
    dvBurnCmdOutMsgRec = module.dvBurnCmdOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dvBurnCmdOutMsgRec)
    lambertValidatorOutMsgRec = module.lambertValidatorOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, lambertValidatorOutMsgRec)

    unitTestSim.InitializeSimulation()

    # run simulation for 3 time steps (excluding initial time step at 0 ns), scale DV vector each time step
    # returned dV vector should only be non-zero if dV solution has converged
    # (simulated by using scaler of 1.0 twice in a row)
    scaler = np.array([1.0, 1.5, 1.0, 1.0])
    dvTrue = np.zeros([len(scaler), 3])
    burnStartTimeTrue = np.zeros([len(scaler)])
    failedNumIterationsLambertTrue = np.zeros([len(scaler)])
    failedXToleranceLambertTrue = np.zeros([len(scaler)])
    failedDistanceTargetConstraintTrue = np.zeros([len(scaler)])
    failedOrbitRadiusConstraintTrue = np.zeros([len(scaler)])
    failedDvSolutionConvergenceTrue = np.array([1, 1, 1, 0])
    dvTheoryTrue = np.zeros([len(scaler), 3])
    for i in range(0, len(scaler)):
        lambertSolutionInMsgData.v1_N = v1_BN_N + dv_N * scaler[i]
        lambertSolutionInMsgData.v2_N = v3_BN_N
        lambertSolutionInMsgData.valid = 1
        lambertSolutionInMsgData.v1Sol2_N = np.array([0., 0., 0.])
        lambertSolutionInMsgData.v2Sol2_N = np.array([0., 0., 0.])
        lambertSolutionInMsgData.validSol2 = 0
        lambertSolutionInMsg.write(lambertSolutionInMsgData, unitTestSim.TotalSim.CurrentNanos)

        unitTestSim.ConfigureStopTime(i * testProcessRate)
        unitTestSim.ExecuteSimulation()

        initialStates = getInitialStates(r1_BN_N, v1_BN_N, dv_N * scaler[i], errStates, errDV)
        violationsDistanceTarget, violationsOrbitRadius = countViolations(initialStates, muBody, tm, tf, r3_BN_N,
                                                                                  maxDistanceTarget, minOrbitRadius)

        # true values
        if violationsDistanceTarget == 0 and \
                violationsOrbitRadius == 0 and \
                failedDvSolutionConvergenceTrue[i] == 0 and \
                numIter < 6 and \
                errX < 1.e-8:
            dvTrue[i, :] = dv_N * scaler[i]
            burnStartTimeTrue[i] = macros.sec2nano(tm)
        else:
            dvTrue[i, :] = np.array([0., 0., 0.])
            burnStartTimeTrue[i] = macros.sec2nano(0.)
        if numIter >= 6:
            failedNumIterationsLambertTrue[i] = 1
        if errX >= 1.e-8:
            failedXToleranceLambertTrue[i] = 1
        if violationsDistanceTarget != 0:
            failedDistanceTargetConstraintTrue[i] = 1
        if violationsOrbitRadius != 0:
            failedOrbitRadiusConstraintTrue[i] = 1
        dvTheoryTrue[i, :] = dv_N * scaler[i]

    # pull module data
    dv = dvBurnCmdOutMsgRec.dvInrtlCmd  # commanded Delta-V
    burnStartTime = dvBurnCmdOutMsgRec.burnStartTime
    failedNumIterationsLambert = lambertValidatorOutMsgRec.failedNumIterationsLambert
    failedXToleranceLambert = lambertValidatorOutMsgRec.failedXToleranceLambert
    failedDvSolutionConvergence = lambertValidatorOutMsgRec.failedDvSolutionConvergence
    failedDistanceTargetConstraint = lambertValidatorOutMsgRec.failedDistanceTargetConstraint
    failedOrbitRadiusConstraint = lambertValidatorOutMsgRec.failedOrbitRadiusConstraint
    dvTheory = lambertValidatorOutMsgRec.dv_N  # Delta-V that would be returned if all tests were passed

    # make sure module output data is correct
    paramsString = ' for DV={}, maneuver time={}, final time={}, iterations={}, errorsX={}, accuracy={}'.format(
        str(p1_dv),
        str(p2_tm),
        str(p3_tf),
        str(p4_iter),
        str(p5_errX),
        str(accuracy))

    np.testing.assert_allclose(dv,
                               dvTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: dv_N,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(burnStartTime,
                               burnStartTimeTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: burnStartTime,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(failedNumIterationsLambert,
                               failedNumIterationsLambertTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: failedNumIterationsLambert,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(failedXToleranceLambert,
                               failedXToleranceLambertTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: failedXToleranceLambert,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(failedDvSolutionConvergence,
                               failedDvSolutionConvergenceTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: failedDvSolutionConvergence,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(failedDistanceTargetConstraint,
                               failedDistanceTargetConstraintTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: failedDistanceTargetConstraint,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(failedOrbitRadiusConstraint,
                               failedOrbitRadiusConstraintTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: failedOrbitRadiusConstraint,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(dvTheory,
                               dvTheoryTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: dvTheory,' + paramsString),
                               verbose=True)


if __name__ == "__main__":
    test_lambertValidator(False, DVs[1], time_maneuver[0], time_final[1], iterations[0], errorsX[0], 1e-4)
