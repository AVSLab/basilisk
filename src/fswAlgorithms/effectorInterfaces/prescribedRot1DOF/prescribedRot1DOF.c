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
The method also checks that the coastOption and tRamp variables are correctly configured.
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

    // Check for nonzero tRamp if the coast option is chosen
    if (configData->coastOption && configData->tRamp == 0.0) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedRot1DOF.tRamp was not configured for the coast "
                                                  "option.");
        // Set the coastOption boolean parameter to false if the user does not specify a ramp time
        configData->coastOption = 0;
    }

    // Check for zero tRamp if the option with no coast period is chosen
    if (!configData->coastOption && configData->tRamp != 0.0) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedRot1DOF.tRamp cannot be set when "
                                                  "prescribedRot1DOF.coastOption is false.");
    }

    // Set the initial time to zero
    configData->tInit = 0.0;

    // Set the module variables to the initial states set by the user
    configData->theta = configData->thetaInit;
    configData->thetaDotInit = 0.0;
    configData->thetaDot = 0.0;

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

        // Update the initial spinning body angle
        configData->thetaInit = configData->theta;

        // Set the convergence to false until the rotation is complete
        configData->convergence = false;

        // Set the parameters required to profile the rotation
        if (configData->coastOption) { // Set parameters for the coast option
            if (configData->thetaInit != configData->thetaRef) {
                // Determine the time at the end of the first ramp segment
                configData->tr = configData->tInit + configData->tRamp;

                // Determine the angle and angle rate at the end of the ramp segment/start of the coast segment
                if (configData->thetaInit < configData->thetaRef) {
                    configData->theta_tr = (0.5 * configData->thetaDDotMax * configData->tRamp * configData->tRamp)
                                       + (configData->thetaDotInit * configData->tRamp) + configData->thetaInit;
                    configData->thetaDot_tr = configData->thetaDDotMax * configData->tRamp + configData->thetaDotInit;
                } else {
                    configData->theta_tr = - ((0.5 * configData->thetaDDotMax * configData->tRamp * configData->tRamp)
                                       + (configData->thetaDotInit * configData->tRamp)) + configData->thetaInit;
                    configData->thetaDot_tr = - configData->thetaDDotMax * configData->tRamp + configData->thetaDotInit;
                }

                // Determine the angle traveled during the coast period
                double deltaThetaCoast = configData->thetaRef - configData->thetaInit
                                         - 2 * (configData->theta_tr - configData->thetaInit);

                // Determine the time duration of the coast segment
                double tCoast = fabs(deltaThetaCoast) / fabs(configData->thetaDot_tr);

                // Determine the time at the end of the coast segment
                configData->tc = configData->tr + tCoast;

                // Determine the angle at the end of the coast segment
                configData->theta_tc = configData->theta_tr + deltaThetaCoast;

                // Determine the time at the end of the rotation
                configData->tf = configData->tc + configData->tRamp;

                // Define the parabolic constants for the first and second ramp segments of the rotation
                configData->a = (configData->theta_tr - configData->thetaInit) /
                                ((configData->tr - configData->tInit) * (configData->tr - configData->tInit));
                configData->b = - (configData->thetaRef - configData->theta_tc) /
                                 ((configData->tc - configData->tf) * (configData->tc - configData->tf));
            } else { // If the initial angle equals the reference angle, no rotation is required. Setting the final time
                // equal to the initial time ensures the correct statement is entered when the rotational states are
                // profiled below
                configData->tf = configData->tInit;
            }
        } else { // Set parameters for the no coast option
            // Determine the total time required for the rotation
            double totalRotTime = sqrt(((0.5 * fabs(configData->thetaRef - configData->thetaInit)) * 8) /
                                         configData->thetaDDotMax);

            // Determine the time at the end of the rotation
            configData->tf = configData->tInit + totalRotTime;

            // Determine the time halfway through the rotation
            configData->ts = configData->tInit + (totalRotTime / 2);

            // Define the parabolic constants for the first and second half of the rotation
            configData->a = 0.5 * (configData->thetaRef - configData->thetaInit) /
                            ((configData->ts - configData->tInit) * (configData->ts - configData->tInit));
            configData->b = -0.5 * (configData->thetaRef - configData->thetaInit) /
                            ((configData->ts - configData->tf) * (configData->ts - configData->tf));
            }
    }

    // Store the current simulation time
    double t = callTime * NANO2SEC;

    // Compute the scalar rotational states at the current simulation time
    if (configData->coastOption) {
        if (t <= configData->tr && configData->tf - configData->tInit != 0) { // Entered during the first ramp segment
            // The acceleration during the first ramp segment is positive if the reference angle is greater than
            // the initial angle. The acceleration is negative during the first ramp segment if the reference angle
            // is less than the initial angle
            if (configData->thetaInit < configData->thetaRef) {
                configData->thetaDDot = configData->thetaDDotMax;
            } else {
                configData->thetaDDot = - configData->thetaDDotMax;
            }
            configData->thetaDot = configData->thetaDDot * (t - configData->tInit) + configData->thetaDotInit;
            configData->theta = configData->a * (t - configData->tInit) * (t - configData->tInit) + configData->thetaInit;
        } else if (t > configData->tr && t <= configData->tc && configData->tf - configData->tInit != 0) { // Entered during the coast segment
            configData->thetaDDot = 0.0;
            configData->thetaDot = configData->thetaDot_tr;
            configData->theta = configData->thetaDot_tr * (t - configData->tr) + configData->theta_tr;
        } else if (t > configData->tc && t <= configData->tf && configData->tf - configData->tInit != 0) { // Entered during the second ramp segment
            // The acceleration during the second ramp segment is negative if the reference angle is greater than
            // the initial angle. The acceleration is positive during the second ramp segment if the reference angle
            // is less than the initial angle
            if (configData->thetaInit < configData->thetaRef) {
                configData->thetaDDot = - configData->thetaDDotMax;
            } else {
                configData->thetaDDot = configData->thetaDDotMax;
            }
            configData->thetaDot = configData->thetaDDot * (t - configData->tInit) + configData->thetaDotInit
                                 - configData->thetaDDot * (configData->tf - configData->tInit);
            configData->theta = configData->b * (t - configData->tf) * (t - configData->tf) + configData->thetaRef;
        } else { // Entered when the rotation is complete
            configData->thetaDDot = 0.0;
            configData->thetaDot = 0.0;
            configData->theta = configData->thetaRef;
            configData->convergence = true;
        }
    } else {
        if (t <= configData->ts && configData->tf - configData->tInit != 0) { // Entered during the first half of the rotation
            if (configData->thetaInit < configData->thetaRef) {
                configData->thetaDDot = configData->thetaDDotMax;
            } else {
                configData->thetaDDot = - configData->thetaDDotMax;
            }
            configData->thetaDot = configData->thetaDDot * (t - configData->tInit) + configData->thetaDotInit;
            configData->theta = configData->a * (t - configData->tInit) * (t - configData->tInit)
                              + configData->thetaInit;
        } else if (t > configData->ts && t <= configData->tf && configData->tf - configData->tInit != 0) { // Entered during the second half of the rotation
            if (configData->thetaInit < configData->thetaRef) {
                configData->thetaDDot = - configData->thetaDDotMax;
            } else {
                configData->thetaDDot = configData->thetaDDotMax;
            }
            configData->thetaDot = configData->thetaDDot * (t - configData->tInit) + configData->thetaDotInit
                                 - configData->thetaDDot * (configData->tf - configData->tInit);
            configData->theta = configData->b * (t - configData->tf) * (t - configData->tf) + configData->thetaRef;
        } else { // Entered when the rotation is complete
            configData->thetaDDot = 0.0;
            configData->thetaDot = 0.0;
            configData->theta = configData->thetaRef;
            configData->convergence = true;
        }
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
