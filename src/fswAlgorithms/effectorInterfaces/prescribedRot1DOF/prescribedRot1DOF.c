/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "prescribedRot1DOF.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <math.h>
#include <stdbool.h>

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, int64_t moduleID) {
    PrescribedMotionMsg_C_init(&configData->prescribedMotionOutMsg);
    HingedRigidBodyMsg_C_init(&configData->spinningBodyOutMsg);
}

/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Reset_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, uint64_t callTime, int64_t moduleID) {
    // Check if the required input message is linked
    if (!HingedRigidBodyMsg_C_isLinked(&configData->spinningBodyInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedRot1DOF.spinningBodyInMsg wasn't connected.");
    }

    // Set the initial time to zero
    configData->tInit = 0.0;

    // Set the module variables to the initial states set by the user
    configData->theta = configData->thetaInit;
    configData->thetaDot = configData->thetaDotInit;

    // Set the initial convergence to true to enter the if statement in the Update method on the first pass
    configData->convergence = true;
}

/*! This method profiles the spinning body rotation and updates the prescribed states as a function of time.
The spinning body prescribed states are then written to the output message.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Update_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, uint64_t callTime, int64_t moduleID) {
    // Create the buffer messages
    HingedRigidBodyMsgPayload spinningBodyIn;
    HingedRigidBodyMsgPayload spinningBodyOut;
    PrescribedMotionMsgPayload prescribedMotionOut;

    // Zero the output messages
    spinningBodyOut = HingedRigidBodyMsg_C_zeroMsgPayload();
    prescribedMotionOut = PrescribedMotionMsg_C_zeroMsgPayload();

    // Read the input message
    spinningBodyIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&configData->spinningBodyInMsg)) {
        spinningBodyIn = HingedRigidBodyMsg_C_read(&configData->spinningBodyInMsg);
    }

    /* This loop is entered (a) initially and (b) when each rotation is complete. The parameters used to profile the
    spinning body rotation are updated in this statement. */
    if (HingedRigidBodyMsg_C_timeWritten(&configData->spinningBodyInMsg) <= callTime && configData->convergence) {
        // Update the initial time as the current simulation time
        configData->tInit = callTime * NANO2SEC;

        // Store the reference angle
        configData->thetaRef = spinningBodyIn.theta;

        // Define temporal information for the maneuver
        double convTime = sqrt(((0.5 * fabs(configData->thetaRef - configData->thetaInit)) * 8) / configData->thetaDDotMax);
        configData->tf = configData->tInit + convTime;
        configData->ts = configData->tInit + convTime / 2;
        // Update the initial spinning body angle
        configData->thetaInit = configData->theta;

        // Define the parabolic constants for the first and second half of the maneuver
        configData->a = 0.5 * (configData->thetaRef - configData->thetaInit) / ((configData->ts - configData->tInit) * (configData->ts - configData->tInit));
        configData->b = -0.5 * (configData->thetaRef - configData->thetaInit) / ((configData->ts - configData->tf) * (configData->ts - configData->tf));

        // Set the convergence to false until the rotation is complete
        configData->convergence = false;
    }

    // Store the current simulation time
    double t = callTime * NANO2SEC;

    // Define the scalar prescribed states
    double thetaDDot;
    double thetaDot;
    double theta;

    // Compute the prescribed scalar states at the current simulation time
    if ((t < configData->ts || t == configData->ts) && configData->tf - configData->tInit != 0) // Entered during the first half of the maneuver
    {
        thetaDDot = configData->thetaDDotMax;
        thetaDot = thetaDDot * (t - configData->tInit) + configData->thetaDotInit;
        theta = configData->a * (t - configData->tInit) * (t - configData->tInit) + configData->thetaInit;
    }
    else if ( t > configData->ts && t <= configData->tf && configData->tf - configData->tInit != 0) // Entered during the second half of the maneuver
    {
        thetaDDot = -1 * configData->thetaDDotMax;
        thetaDot = thetaDDot * (t - configData->tInit) + configData->thetaDotInit - thetaDDot * (configData->tf - configData->tInit);
        theta = configData->b * (t - configData->tf) * (t - configData->tf) + configData->thetaRef;
    }
    else // Entered when the maneuver is complete
    {
        thetaDDot = 0.0;
        thetaDot = configData->thetaDotRef;
        theta = configData->thetaRef;
        configData->convergence = true;
    }

    // Determine the prescribed parameters: omega_FM_F and omegaPrime_FM_F
    v3Normalize(configData->rotAxis_M, configData->rotAxis_M);
    v3Scale(configData->thetaDot, configData->rotAxis_M, configData->omega_FM_F);
    v3Scale(configData->thetaDDot, configData->rotAxis_M, configData->omegaPrime_FM_F);

    // Determine the DCM dcm_F0M that describes the initial spinning body attitude relative to the hub-fixed mount frame
    double dcm_F0M[3][3];
    double prv_F0M_array[3];
    v3Scale(configData->thetaInit, configData->rotAxis_M, prv_F0M_array);
    PRV2C(prv_F0M_array, dcm_F0M);

    // Determine the DCM dcm_FF0 that describes the current spinning body attitude relative to the initial attitude
    double dcm_FF0[3][3];
    double prv_FF0_array[3];
    double theta_FF0 = configData->theta - configData->thetaInit;
    v3Scale(theta_FF0, configData->rotAxis_M, prv_FF0_array);
    PRV2C(prv_FF0_array, dcm_FF0);

    // Determine the DCM dcm_FM that describes the current spinning body attitude relative to the mount frame
    double dcm_FM[3][3];
    m33MultM33(dcm_FF0, dcm_F0M, dcm_FM);

    // Determine the spinning body MRP attitude sigma_FM from the computed dcm_FM
    C2MRP(dcm_FM, configData->sigma_FM);

    // Copy the required module variables to the prescribedMotionOut output message
    v3Copy(configData->r_FM_M, prescribedMotionOut.r_FM_M);
    v3Copy(configData->rPrime_FM_M, prescribedMotionOut.rPrime_FM_M);
    v3Copy(configData->rPrimePrime_FM_M, prescribedMotionOut.rPrimePrime_FM_M);
    v3Copy(configData->omega_FM_F, prescribedMotionOut.omega_FM_F);
    v3Copy(configData->omegaPrime_FM_F, prescribedMotionOut.omegaPrime_FM_F);
    v3Copy(configData->sigma_FM, prescribedMotionOut.sigma_FM);

    // Copy the required scalar variables to the spinningBodyOut output message
    spinningBodyOut.theta = configData->theta;
    spinningBodyOut.thetaDot = configData->thetaDot;

    // Write the output messages
    HingedRigidBodyMsg_C_write(&spinningBodyOut, &configData->spinningBodyOutMsg, moduleID, callTime);
    PrescribedMotionMsg_C_write(&prescribedMotionOut, &configData->prescribedMotionOutMsg, moduleID, callTime);
}
