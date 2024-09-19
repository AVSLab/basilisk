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
import itertools

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import lambertSurfaceRelativeVelocity
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# parameters
relativeVelocities = np.array([[0., 0., 0.], [100., -300., 200.]])
time_maneuver = [1.e3, 2.e3]
trueAnomalies = [0., 70.]
rotAngles = [0., 230.]
angularVelocities = np.array([[0., 0., 0.], [1., -3., 2.]])

paramArray = [relativeVelocities, time_maneuver, trueAnomalies, rotAngles, angularVelocities]
# create list with all combinations of parameters
paramList = list(itertools.product(*paramArray))

@pytest.mark.parametrize("accuracy", [1e-4])
@pytest.mark.parametrize("p1_vr, p2_tm, p3_f, p4_rot, p5_omega", paramList)

def test_lambertSurfaceRelativeVelocity(show_plots, p1_vr, p2_tm, p3_f, p4_rot, p5_omega, accuracy):
    r"""
    **Validation Test Description**

    This test checks if the Lambert Surface Relative Velocity module works correctly for different relative velocity
    vectors, maneuver times, true anomalies, planet rotation angles, and planet angular velocity vectors.

    **Test Parameters**

    Args:
        show_plots: flag if plots should be shown
        p1_vr: desired relative velocity
        p2_tm: maneuver time
        p3_f: true anomaly
        p4_rot: rotation angle of planet
        p5_omega: angular velocity vector of planet
        accuracy: accuracy of the test

    **Description of Variables Being Tested**

    The content of the DesiredVelocityMsg output message (velocity vector and maneuver time) is compared with the true
    values.
    """
    lambertSurfaceRelativeVelocityTestFunction(show_plots, p1_vr, p2_tm, p3_f, p4_rot, p5_omega, accuracy)


def lambertSurfaceRelativeVelocityTestFunction(show_plots, p1_vr, p2_tm, p3_f, p4_rot, p5_omega, accuracy):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(100)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    muBody = 3.986004418e14
    ecc = 0.1
    vRelativeDesired_S = p1_vr
    tm = p2_tm

    omega_PN_N = p5_omega
    if np.linalg.norm(omega_PN_N) > 1e-6:
        omegaHat_PN_N = omega_PN_N/np.linalg.norm(omega_PN_N)
    else:
        omegaHat_PN_N = np.array([0.0, 0.0, 1.0])

    prv_PN = p4_rot * macros.D2R * omegaHat_PN_N
    sigma_PN = RigidBodyKinematics.PRV2MRP(prv_PN)
    dcm_PN = RigidBodyKinematics.MRP2C(sigma_PN)
    omega_PN_P = np.dot(dcm_PN, omega_PN_N)

    # set up orbit using classical orbit elements
    oe0 = orbitalMotion.ClassicElements()
    r = 10000. * 1000  # meters
    oe0.a = r
    oe0.e = ecc
    oe0.i = 10. * macros.D2R
    oe0.Omega = 20. * macros.D2R
    oe0.omega = 30. * macros.D2R
    oe0.f = p3_f * macros.D2R
    # spacecraft state at initial time
    r0_BN_N, v0_BN_N = orbitalMotion.elem2rv_parab(muBody, oe0)

    # setup module to be tested
    module = lambertSurfaceRelativeVelocity.LambertSurfaceRelativeVelocity()
    module.ModelTag = "lambertSurfaceRelativeVelocity"
    module.setVRelativeDesired_S(vRelativeDesired_S)
    module.setTime(tm)
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure input messages
    ephemerisInMsgData = messaging.EphemerisMsgPayload()
    ephemerisInMsgData.sigma_BN = sigma_PN
    ephemerisInMsgData.omega_BN_B = omega_PN_P
    ephemerisInMsg = messaging.EphemerisMsg().write(ephemerisInMsgData)

    lambertProblemInMsgData = messaging.LambertProblemMsgPayload()
    lambertProblemInMsgData.r2_N = r0_BN_N  # only info of LambertProblemMsg needed by module
    lambertProblemInMsg = messaging.LambertProblemMsg().write(lambertProblemInMsgData)

    # subscribe input messages to module
    module.lambertProblemInMsg.subscribeTo(lambertProblemInMsg)
    module.ephemerisInMsg.subscribeTo(ephemerisInMsg)

    # setup output message recorder objects
    desiredVelocityOutMsgRec = module.desiredVelocityOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, desiredVelocityOutMsgRec)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    # pull module data
    vDesired = desiredVelocityOutMsgRec.vDesired_N[0]
    time = desiredVelocityOutMsgRec.maneuverTime[0]

    # true values
    if np.linalg.norm(omega_PN_N) > 1e-6:
        s1Hat = np.cross(omega_PN_N, r0_BN_N)/np.linalg.norm(np.cross(omega_PN_N, r0_BN_N))
    else:
        s1Hat = (np.cross(np.array([0.0, 0.0, 1.0]), r0_BN_N)/
                 np.linalg.norm(np.cross(np.array([0.0, 0.0, 1.0]), r0_BN_N)))
    s3Hat = r0_BN_N/np.linalg.norm(r0_BN_N)
    s2Hat = np.cross(s3Hat, s1Hat)/np.linalg.norm(np.cross(s3Hat, s1Hat))
    dcm_SN = np.array([s1Hat, s2Hat, s3Hat])
    vDesiredTrue = np.cross(omega_PN_N, r0_BN_N) + np.dot(dcm_SN.T, vRelativeDesired_S)
    timeTrue = tm

    # make sure module output data is correct
    paramsString = ' for desired relative velocity={}, maneuver time={}, true anomaly={}, rotation angle={}, ' \
                   'angular velocity={}, accuracy={}'.format(
        str(p1_vr),
        str(p2_tm),
        str(p3_f),
        str(p4_rot),
        str(p5_omega),
        str(accuracy))

    np.testing.assert_allclose(vDesired,
                               vDesiredTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: desired velocity,' + paramsString),
                               verbose=True)

    np.testing.assert_allclose(time,
                               timeTrue,
                               rtol=0,
                               atol=accuracy,
                               err_msg=('Variable: maneuver time,' + paramsString),
                               verbose=True)


if __name__ == "__main__":
    test_lambertSurfaceRelativeVelocity(False, relativeVelocities[1], time_maneuver[0], trueAnomalies[0], rotAngles[0],
                                        angularVelocities[0], 1e-4)
