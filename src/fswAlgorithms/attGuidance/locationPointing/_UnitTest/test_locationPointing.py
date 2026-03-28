#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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


import math

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import locationPointing
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


@pytest.mark.parametrize("accuracy", [1e-12])
@pytest.mark.parametrize("r_LS_N", [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [0, -1, 0.], [1, 1, 1]])
@pytest.mark.parametrize("locationType, v_LP_N", [
    (0, [0., 0., 0.]),
    (1, [0., 0., 0.]),
    (2, [0., 0., 0.]),
    (3, [0., 1., 0.]),
    (3, [0., 0., 1.]),
    (3, [1., 0., 0.]),
])
@pytest.mark.parametrize("use3DRate", [True, False])
def test_locationPointing(show_plots, r_LS_N, locationType, v_LP_N, use3DRate, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that the Attitude Guidance and Attitude Reference messages content are properly computed
    for a series of desired inertial target locations.
    For strip imaging (locationType=3), the test also verifies that the extra rotation aligns ``cHat_B``
    perpendicular to the scanning direction.

    **Test Parameters**

    Args:
        r_LS_N (float): position vector of location relative to spacecraft
        locationType (int): choose whether to use ``locationInMsg``, ``celBodyInMsg``,
            ``scTargetInMsg``, or ``locationstripInMsg``
        v_LP_N (float): strip velocity vector relative to the planet (inertial frame),
            only used when locationType=3
        use3DRate (bool): choose between 2D or 3D rate control
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    The script checks the attitude and rate outputs.
    For strip imaging, it additionally checks that the reference attitude correctly
    constrains the roll degree of freedom so that ``cHat_B`` is perpendicular to the
    strip scanning direction.

    """
    [testResults, testMessage] = locationPointingTestFunction(show_plots, r_LS_N, v_LP_N,
                                                              locationType, use3DRate, accuracy)
    assert testResults < 1, testMessage


def locationPointingTestFunction(show_plots, r_LS_NIn,v_LP_NIn,locationType, use3DRate, accuracy):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    timeStep = 0.1
    testProcessRate = macros.sec2nano(timeStep)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup
    pHat_B = np.array([1, 0, 0])
    cHat_B= np.array([0, 1, 0])
    r_SN_N = np.array([10, 11, 12])
    r_LS_N = np.array(r_LS_NIn)
    v_LP_N = np.array(v_LP_NIn)
    omega_BN_B = np.array([0.001, 0.002, 0.003])
    sigma_BN = np.array([0., 0., 0.])
    r_LN_N = r_LS_N + r_SN_N
    r_PN_N = np.array([5., 6., 7.])   # planet center in inertial frame
    r_LP_N_pos = r_LN_N - r_PN_N     # target position relative to planet center

    # setup module to be tested
    module = locationPointing.locationPointing()
    module.ModelTag = "locationPointingTag"
    unitTestSim.AddModelToTask(unitTaskName, module)
    module.pHat_B = pHat_B
    module.cHat_B = cHat_B
    eps = 0.1 * macros.D2R
    module.smallAngle = eps
    if use3DRate:
        module.useBoresightRateDamping = 1

    # Configure input messages
    scTransInMsgData = messaging.NavTransMsgPayload()
    scTransInMsgData.r_BN_N = r_SN_N
    scTransInMsg = messaging.NavTransMsg().write(scTransInMsgData)
    scAttInMsgData = messaging.NavAttMsgPayload()
    scAttInMsgData.omega_BN_B = omega_BN_B
    scAttInMsgData.sigma_BN = sigma_BN
    scAttInMsg = messaging.NavAttMsg().write(scAttInMsgData)

    if locationType == 0:
        locationInMsgData = messaging.GroundStateMsgPayload()
        locationInMsgData.r_LN_N = r_LN_N
        locationInMsg = messaging.GroundStateMsg().write(locationInMsgData)
        module.locationInMsg.subscribeTo(locationInMsg)
    elif locationType == 1:
        locationInMsgData = messaging.EphemerisMsgPayload()
        locationInMsgData.r_BdyZero_N = r_LN_N
        locationInMsg = messaging.EphemerisMsg().write(locationInMsgData)
        module.celBodyInMsg.subscribeTo(locationInMsg)
    elif locationType == 2:
        locationInMsgData = messaging.NavTransMsgPayload()
        locationInMsgData.r_BN_N = r_LN_N
        locationInMsg = messaging.NavTransMsg().write(locationInMsgData)
        module.scTargetInMsg.subscribeTo(locationInMsg)
    elif locationType == 3:
        locationInMsgData = messaging.StripStateMsgPayload()
        locationInMsgData.r_LN_N = r_LN_N
        locationInMsgData.r_LP_N = r_LP_N_pos
        locationInMsgData.v_LP_N = v_LP_N
        locationInMsg = messaging.StripStateMsg().write(locationInMsgData)
        module.locationstripInMsg.subscribeTo(locationInMsg)

    # subscribe input messages to module
    module.scTransInMsg.subscribeTo(scTransInMsg)
    module.scAttInMsg.subscribeTo(scAttInMsg)

    # setup output message recorder objects
    attGuidOutMsgRec = module.attGuidOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, attGuidOutMsgRec)
    attRefOutMsgRec = module.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, attRefOutMsgRec)
    scTransRec = scTransInMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scTransRec)
    scAttRec = scAttInMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scAttRec)

    # setup and execute simulation
    unitTestSim.InitializeSimulation()
    counter = 0
    while counter < 3:
        scAttInMsgData.sigma_BN = sigma_BN + omega_BN_B * timeStep * counter
        scAttInMsg.write(scAttInMsgData)
        unitTestSim.ConfigureStopTime(macros.sec2nano(counter * timeStep))
        unitTestSim.ExecuteSimulation()
        counter += 1

    truthSigmaBR, truthOmegaBR, truthSigmaRN, truthOmegaRN_B, truthOmegaRN_N = \
        truthValues(pHat_B, cHat_B, r_LN_N, r_SN_N, scAttRec.sigma_BN, scAttRec.omega_BN_B, eps,
                    timeStep, use3DRate, v_LP_N=v_LP_N, r_LP_N_pos=r_LP_N_pos, isStrip=(locationType == 3))

    # compare the module results to the truth values
    for i in range(0, len(truthSigmaBR)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(attGuidOutMsgRec.sigma_BR[i], truthSigmaBR[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed sigma_BR unit test at t=" +
                                str(attGuidOutMsgRec.times()[i] * macros.NANO2SEC) +
                                "sec\n")

    for i in range(0, len(truthOmegaBR)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(attGuidOutMsgRec.omega_BR_B[i], truthOmegaBR[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed omega_BR_B unit test at t=" +
                                str(attGuidOutMsgRec.times()[i] * macros.NANO2SEC) +
                                "sec\n")

    for i in range(0, len(truthSigmaRN)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(attRefOutMsgRec.sigma_RN[i], truthSigmaRN[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed sigma_RN unit test at t=" +
                                str(attRefOutMsgRec.times()[i] * macros.NANO2SEC) +
                                "sec\n")

    for i in range(0, len(truthOmegaRN_B)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(attGuidOutMsgRec.omega_RN_B[i], truthOmegaRN_B[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed omega_RN_B unit test at t=" +
                                str(attRefOutMsgRec.times()[i] * macros.NANO2SEC) +
                                "sec\n")

    for i in range(0, len(truthOmegaRN_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(attRefOutMsgRec.omega_RN_N[i], truthOmegaRN_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + module.ModelTag + " Module failed omega_RN_N unit test at t=" +
                                str(attRefOutMsgRec.times()[i] * macros.NANO2SEC) +
                                "sec\n")

    # Geometric property checks on the reference attitude
    # Use eps (the module's smallAngle threshold) as tolerance because the module
    # intentionally zeroes sigma_BR when the pointing error is below smallAngle,
    # so the reference attitude can be off by up to ~eps in pointing direction.
    geomFailCount, geomMessages = checkReferenceGeometry(
        attRefOutMsgRec.sigma_RN, scAttRec.sigma_BN, r_LN_N, r_SN_N,
        pHat_B, cHat_B, v_LP_N, isStrip=(locationType == 3), accuracy=eps)
    testFailCount += geomFailCount
    testMessages.extend(geomMessages)

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


def checkReferenceGeometry(sigma_RN_list, sigma_BN_list, r_LN_N, r_SN_N,
                           pHat_B, cHat_B, v_LP_N, isStrip=False, accuracy=1e-10):
    """Verify geometric properties of the reference attitude.

    For every recorded time step this function checks:
        1. **Pointing constraint** – the body-fixed pointing axis ``pHat_B``,
           when expressed in the inertial frame through the reference DCM,
           must be aligned with the target direction ``r_LS_N``.
        2. **Scan-line constraint** (strip imaging only) – the body-fixed
           cross-track axis ``cHat_B``, when expressed in the inertial frame,
           must be perpendicular to the strip scanning direction ``v_LP_N``.

    Args:
        sigma_RN_list: recorded reference MRP history (N×3).
        sigma_BN_list: recorded body MRP history (N×3), used only for indexing.
        r_LN_N: inertial target position [m].
        r_SN_N: inertial spacecraft position [m].
        pHat_B: body-fixed pointing axis.
        cHat_B: body-fixed cross-track axis.
        v_LP_N: strip velocity in inertial frame.
        isStrip: True when strip-imaging mode is active.
        accuracy: absolute tolerance for the checks.

    Returns:
        (failCount, messages): number of failures and list of error strings.
    """
    failCount = 0
    messages = []

    r_LS_N = np.array(r_LN_N) - np.array(r_SN_N)
    rHat_LS_N = r_LS_N / np.linalg.norm(r_LS_N)

    for i in range(len(sigma_RN_list)):
        sigma_RN = np.array(sigma_RN_list[i])
        dcm_RN = RigidBodyKinematics.MRP2C(sigma_RN)  # maps N -> R (body)

        # --- 1. Pointing check: pHat_B expressed in N must align with rHat_LS_N ---
        # dcm_RN maps inertial vectors to body frame, so
        # pHat_N = dcm_RN^T . pHat_B
        pHat_N = np.transpose(dcm_RN).dot(pHat_B)
        crossProd = np.cross(pHat_N, rHat_LS_N)
        if np.linalg.norm(crossProd) > accuracy:
            failCount += 1
            messages.append(
                f"FAILED: Pointing check at index {i}: "
                f"|pHat_N x rHat_LS_N| = {np.linalg.norm(crossProd):.2e} "
                f"(should be < {accuracy})\n")

        # --- 2. Scan-line check (strip imaging only) ---
        if isStrip and np.linalg.norm(v_LP_N) > 1e-12:
            v_hat_N = np.array(v_LP_N) / np.linalg.norm(v_LP_N)
            # Reproduce the alignmentThreshold check from C code:
            # if v_perp = pHat_B x v_TP_B is too small, the C code skips
            # the extra strip rotation, so perpendicularity is not guaranteed.
            alignmentThreshold = 0.1  # default value in C code
            v_TP_B_check = dcm_RN.dot(v_hat_N)
            v_TP_B_check = v_TP_B_check / np.linalg.norm(v_TP_B_check)
            v_perp_check = np.cross(pHat_B, v_TP_B_check)
            if np.linalg.norm(v_perp_check) >= alignmentThreshold:
                cHat_N = np.transpose(dcm_RN).dot(cHat_B)
                dotProduct = abs(np.dot(cHat_N, v_hat_N))
                if dotProduct > accuracy:
                    failCount += 1
                    messages.append(
                        f"FAILED: Scan-line perpendicularity check at index {i}: "
                        f"|cHat_N . vHat| = {dotProduct:.2e} "
                        f"(should be < {accuracy})\n")

    return failCount, messages


def truthValues(pHat_B, cHat_B, r_LN_N, r_SN_N, sigma_BNList, omega_BNList, smallAngle, dt,
                use3DRate, v_LP_N=None, r_LP_N_pos=None, isStrip=False):
    # setup eHat180_B
    eHat180_B = np.cross(pHat_B, np.array([1., 0., 0.]))
    if np.linalg.norm(eHat180_B) < 0.1:
        eHat180_B = np.cross(pHat_B, np.array([0., 1., 0.]))
    eHat180_B = eHat180_B / np.linalg.norm(eHat180_B)

    r_LS_N = r_LN_N - r_SN_N

    counter = 0
    omega_BR_B = np.array([0., 0., 0.])
    sigma_BR_Out = []
    omega_BR_B_Out = []
    sigma_RN_Out = []
    omega_RN_B_Out = []
    omega_RN_N_Out = []
    while counter <= 2:
        sigma_BN = sigma_BNList[counter]
        dcmBN = RigidBodyKinematics.MRP2C(sigma_BN)
        r_LS_B = dcmBN.dot(r_LS_N)
        dum1 = pHat_B.dot(r_LS_B) / np.linalg.norm(r_LS_B)
        if abs(dum1) > 1.0:
            dum1 = dum1 / abs(dum1)
        phi = math.acos(dum1)
        if phi < smallAngle:
            sigma_BR = np.array([0., 0., 0.])
        else:
            if math.pi - phi < smallAngle:
                eHat_B = eHat180_B
            else:
                eHat_B = np.cross(pHat_B, r_LS_B)
            eHat_B = eHat_B / np.linalg.norm(eHat_B)
            sigma_BR = - math.tan(phi / 4.) * eHat_B

        # Compute pointing-only reference attitude sigma_RN
        sigma_RN = RigidBodyKinematics.addMRP(sigma_BN, -sigma_BR)

        if isStrip and v_LP_N is not None:
            # --- Extra rotation for strip imaging ---
            alignmentThreshold = 0.1  # default value in C code

            # Step 1: Transform strip velocity to reference body frame
            dcmRN = RigidBodyKinematics.MRP2C(sigma_RN)
            v_hat = v_LP_N / np.linalg.norm(v_LP_N)
            v_TP_B = dcmRN.dot(v_hat)
            v_TP_B = v_TP_B / np.linalg.norm(v_TP_B)

            # Step 2: Compute v_perp = pHat_B x v_TP_B
            v_perp = np.cross(pHat_B, v_TP_B)
            if np.linalg.norm(v_perp) < alignmentThreshold:
                # Special case: v_TP_B and pHat_B are colinear or nearly colinear.
                # v_perp is either not defined or unstable (90-degree shifts).
                # No extra rotation performed (matches C code goto skipStripRotation).
                pass
            else:
                v_perp = v_perp / np.linalg.norm(v_perp)

                # Step 3: Choose sign of v_perp to minimize rotation from cHat_B
                if np.dot(cHat_B, v_perp) < 0.0:
                    v_perp = -v_perp

                # Step 4: Compute rotation angle from cHat_B to v_perp
                dotProd2 = np.dot(cHat_B, v_perp)
                if abs(dotProd2) > 1.0:
                    dotProd2 = dotProd2 / abs(dotProd2)
                angle = math.acos(dotProd2)

                # Step 5: Compute sigma_R2R
                if angle < smallAngle:
                    sigma_R2R = np.array([0., 0., 0.])
                else:
                    eHat_R2R = np.cross(cHat_B, v_perp)
                    eHat_R2R = eHat_R2R / np.linalg.norm(eHat_R2R)
                    sigma_R2R = math.tan(angle / 4.) * eHat_R2R

                # Step 6: Compose sigma_R2N = addMRP(sigma_RN, sigma_R2R)
                sigma_R2N = RigidBodyKinematics.addMRP(sigma_RN, sigma_R2R)

                # Step 7: Replace sigma_BR with strip-corrected value
                sigma_BR = RigidBodyKinematics.subMRP(sigma_BN, sigma_R2N)

        if counter >= 1:
            dsigma = (sigma_BR - sigma_BR_Out[counter - 1]) / dt
            Binv = RigidBodyKinematics.BinvMRP(sigma_BR)
            omega_BR_B = Binv.dot(dsigma) * 4

        if use3DRate and not isStrip:
            rHat_LS_B = r_LS_B / np.linalg.norm(r_LS_B)
            omega_BR_B = omega_BR_B + (omega_BNList[counter].dot(rHat_LS_B))*rHat_LS_B

        # store truth results
        sigma_BR_Out.append(sigma_BR)
        omega_BR_B_Out.append(omega_BR_B)
        sigma_RN_Out.append(RigidBodyKinematics.addMRP(sigma_BNList[counter], -sigma_BR))
        omega_RN_B_Out.append(omega_BNList[counter] - omega_BR_B_Out[counter])
        omega_RN_N_Out.append(np.transpose(dcmBN).dot(omega_BNList[counter] - omega_BR_B_Out[counter]))

        counter += 1

    return sigma_BR_Out, omega_BR_B_Out, sigma_RN_Out, omega_RN_B_Out, omega_RN_N_Out


if __name__ == "__main__":
    locationPointingTestFunction(False, [1, 0, 0], [0., 1., 0.], 3, False, 1e-12)
