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

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertPlanner
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# parameters
revs = [0, 1, 4]
time_maneuver = [1e3, 2e3]
time_final = [1e3, 4e3]
eccentricities = [0.0, 0.05, 1.0, 1.2]

paramArray = [revs, time_maneuver, time_final, eccentricities]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))

@pytest.mark.parametrize("accuracy", [1e-2])
@pytest.mark.parametrize("p1_revs, p2_tm, p3_tf, p4_eccs", paramList)

def test_lambertPlanner(show_plots, p1_revs, p2_tm, p3_tf, p4_eccs, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the Lambert planner module works correctly for different number of revolutions, maneuver times,
    final times, and orbit eccentricities.

    **Test Parameters**

    Args:
        show_plots: flag if plots should be shown
        p1_revs: number of revolutions to be completed
        p2_tm: maneuver time
        p3_tf: final time
        p4_eccs: eccentricity of orbit

    **Description of Variables Being Tested**

    The content of the LambertProblemMsg output message is compared with the true values. Most of the message content
    corresponds to the module input variables. The position vector r1_N of the message is obtained by integrating two
    body point mass equations of motion inside the module from the current time to the maneuver time maneuverTime. The
    true value for r1_N is obtained by solving Kepler's equation using from current time to maneuver time and the given
    orbit elements.
    """
    lambertPlannerTestFunction(show_plots, p1_revs, p2_tm, p3_tf, p4_eccs, accuracy)


def lambertPlannerTestFunction(show_plots, p1_revs, p2_tm, p3_tf, p4_eccs, accuracy):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    solver = messaging.IZZO
    muBody = 3.986004418e14
    t0 = 100.
    revs = p1_revs
    tm = p2_tm
    tf = p3_tf
    ecc = p4_eccs
    targetPosition = np.array([7000*1000., -1000*1000., 2000*1000.])

    # set up orbit using classical orbit elements
    oe1 = orbitalMotion.ClassicElements()
    r = 10000. * 1000  # meters
    if ecc < 1.0:
        # elliptic case
        oe1.a = r
    else:
        # parabolic and hyperbolic case
        oe1.a = -r
    oe1.e = ecc
    oe1.i = 10. * macros.D2R
    oe1.Omega = 20. * macros.D2R
    oe1.omega = 30. * macros.D2R
    oe1.f = 40. * macros.D2R
    # spacecraft state at initial time
    r0_BN_N, v0_BN_N = orbitalMotion.elem2rv_parab(muBody, oe1)

    oe2 = copy.deepcopy(oe1)

    if ecc < 1.0:
        # elliptic case
        M1 = orbitalMotion.E2M(orbitalMotion.f2E(oe1.f, ecc), ecc)
        n = np.sqrt(muBody/(oe1.a)**3)
        M2 = M1 + n*(tm-t0)
        oe2.f = orbitalMotion.E2f(orbitalMotion.M2E(M2, ecc), ecc)
    elif ecc == 1.0:
        # parabolic case
        D1 = np.tan(oe1.f/2)
        M1 = D1 + 1/3*D1**3
        n = np.sqrt(muBody/(2*(-oe1.a)**3))
        M2 = M1 + n*(tm-t0)
        A = 3./2.*M2
        B = (A + np.sqrt(A**2 + 1))**(1./3.)
        oe2.f = 2.*np.arctan(B-1./B)
    else:
        # hyperbolic case
        N1 = orbitalMotion.H2N(orbitalMotion.f2H(oe1.f, ecc), ecc)
        n = np.sqrt(muBody/(-oe1.a)**3)
        N2 = N1 + n*(tm-t0)
        oe2.f = orbitalMotion.H2f(orbitalMotion.N2H(N2, ecc), ecc)

    # spacecraft state at maneuver time
    rm_BN_N, vm_BN_N = orbitalMotion.elem2rv_parab(muBody, oe2)

    # setup module to be tested
    module = lambertPlanner.LambertPlanner()
    module.ModelTag = "lambertPlanner"
    module.setR_TN_N(targetPosition)
    module.setFinalTime(tf)
    module.setManeuverTime(tm)
    module.setMu(muBody)
    module.setNumRevolutions(p1_revs)
    module.useSolverIzzoMethod()
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure input messages
    navTransInMsgData = messaging.NavTransMsgPayload()
    navTransInMsgData.timeTag = t0
    navTransInMsgData.r_BN_N = r0_BN_N
    navTransInMsgData.v_BN_N = v0_BN_N
    navTransInMsg = messaging.NavTransMsg().write(navTransInMsgData)

    # subscribe input messages to module
    module.navTransInMsg.subscribeTo(navTransInMsg)

    # setup output message recorder objects
    lambertProblemOutMsgRec = module.lambertProblemOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, lambertProblemOutMsgRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    solverMethod = lambertProblemOutMsgRec.solverMethod[0]
    r1_N = lambertProblemOutMsgRec.r1_N[0]
    r2_N = lambertProblemOutMsgRec.r2_N[0]
    transferTime = lambertProblemOutMsgRec.transferTime[0]
    mu = lambertProblemOutMsgRec.mu[0]
    numRevolutions = lambertProblemOutMsgRec.numRevolutions[0]

    if solverMethod == messaging.GOODING:
        solverName = "Gooding"
    elif solverMethod == messaging.IZZO:
        solverName = "Izzo"

    # true values
    solverMethodTrue = solver
    r1vecTrue = rm_BN_N
    r2vecTrue = targetPosition
    transferTimeTrue = tf - tm
    muTrue = muBody
    numRevolutionsTrue = revs

    if solverMethodTrue == messaging.GOODING:
        solverNameTrue = "Gooding"
    elif solverMethodTrue == messaging.IZZO:
        solverNameTrue = "Izzo"

    # make sure module output data is correct
    paramsString = ' for rev={}, maneuver time={}, final time={}, eccentricity={}, accuracy={}'.format(
        str(p1_revs),
        str(p2_tm),
        str(p3_tf),
        str(p4_eccs),
        str(accuracy))

    np.testing.assert_string_equal(solverName, solverNameTrue)

    np.testing.assert_allclose(r1_N,
                               r1vecTrue,
                               rtol=accuracy,
                               atol=0,
                               err_msg=('Variable: r1_N,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(r2_N,
                               r2vecTrue,
                               rtol=accuracy,
                               atol=0,
                               err_msg=('Variable: r2_N,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(transferTime,
                               transferTimeTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: transferTime,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(mu,
                               muTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: mu,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(numRevolutions,
                               numRevolutionsTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: numRevolutions,' + paramsString),
                               verbose=True)


if __name__ == "__main__":
    test_lambertPlanner(False, revs[0], time_maneuver[0], time_final[1], eccentricities[0], 1e-2)
