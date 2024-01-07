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

#include "prescribedTrans.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

/*! This method initializes the output message for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_prescribedTrans(PrescribedTransConfig *configData, int64_t moduleID) {
    PrescribedMotionMsg_C_init(&configData->prescribedMotionOutMsg);
}

/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
The method also checks that the coastOption and tRamp variables are correctly configured.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Reset_prescribedTrans(PrescribedTransConfig *configData, uint64_t callTime, int64_t moduleID) {
    // Check if the required input message is linked
    if (!PrescribedTransMsg_C_isLinked(&configData->prescribedTransInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedTrans.prescribedTransInMsg wasn't connected.");
    }

    // Check for nonzero tRamp if the coast option is chosen
    if (configData->coastOption && configData->tRamp == 0.0) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedTrans.tRamp was not configured for the coast "
                                                  "option.");
        // Set the coastOption boolean parameter to false if the user does not specify a ramp time
        configData->coastOption = 0;
    }

    // Check for zero tRamp if the option with no coast period is chosen
    if (!configData->coastOption && configData->tRamp != 0.0) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedTrans.tRamp cannot be set when "
                                                  "prescribedTrans.coastOption is false.");
    }

    // Set the initial time to zero
    configData->tInit = 0.0;

    // Set the module variables to the initial states set by the user
    configData->transPos = configData->transPosInit;
    configData->transVelInit = 0.0;
    configData->transVel = 0.0;

    // Set the initial convergence to true to enter the if statement in the Update method on the first pass
    configData->convergence = true;
}

/*! This method profiles the required translation and updates the prescribed states as a function of time.
The prescribed states are then written to the output message.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Update_prescribedTrans(PrescribedTransConfig *configData, uint64_t callTime, int64_t moduleID) {
    // Create the buffer messages
    PrescribedTransMsgPayload prescribedTransIn;
    PrescribedMotionMsgPayload prescribedMotionOut;

    // Zero the output message
    prescribedMotionOut = PrescribedMotionMsg_C_zeroMsgPayload();

    // Read the input message
    prescribedTransIn = PrescribedTransMsg_C_zeroMsgPayload();
    if (PrescribedTransMsg_C_isWritten(&configData->prescribedTransInMsg)) {
        prescribedTransIn = PrescribedTransMsg_C_read(&configData->prescribedTransInMsg);
    }

    /* This loop is entered (a) initially and (b) when each translation is complete. The parameters used to profile the
    translation are updated in this statement. */
    if (PrescribedTransMsg_C_timeWritten(&configData->prescribedTransInMsg) <= callTime && configData->convergence) {
        // Update the initial time as the current simulation time
        configData->tInit = callTime * NANO2SEC;

        // Store the reference scalar position
        configData->transPosRef = prescribedTransIn.scalarPos;

        // Update the initial scalar position
        configData->transPosInit = configData->transPos;

        // Set the convergence to false until the translation is complete
        configData->convergence = false;

        // Set the parameters required to profile the translation
        if (configData->coastOption) { // Set parameters for the coast option
            if (configData->transPosInit != configData->transPosRef) {
                // Determine the time at the end of the first ramp segment
                configData->tr = configData->tInit + configData->tRamp;

                // Determine the position and velocity at the end of the ramp segment/start of the coast segment
                if (configData->transPosInit < configData->transPosRef) {
                    configData->transPos_tr = (0.5 * configData->transAccelMax * configData->tRamp * configData->tRamp)
                                              + (configData->transVelInit * configData->tRamp) +
                                              configData->transPosInit;
                    configData->transVel_tr = configData->transAccelMax * configData->tRamp + configData->transVelInit;
                } else {
                    configData->transPos_tr =
                            -((0.5 * configData->transAccelMax * configData->tRamp * configData->tRamp)
                              + (configData->transVelInit * configData->tRamp)) + configData->transPosInit;
                    configData->transVel_tr = -configData->transAccelMax * configData->tRamp + configData->transVelInit;
                }

                // Determine the distance traveled during the coast period
                double deltaPosCoast = configData->transPosRef - configData->transPosInit
                                       - 2 * (configData->transPos_tr - configData->transPosInit);

                // Determine the time duration of the coast segment
                double tCoast = fabs(deltaPosCoast) / fabs(configData->transVel_tr);

                // Determine the time at the end of the coast segment
                configData->tc = configData->tr + tCoast;

                // Determine the position at the end of the coast segment
                configData->transPos_tc = configData->transPos_tr + deltaPosCoast;

                // Determine the time at the end of the translation
                configData->tf = configData->tc + configData->tRamp;

                // Define the parabolic constants for the first and second ramp segments of the translation
                configData->a = (configData->transPos_tr - configData->transPosInit) /
                                ((configData->tr - configData->tInit) * (configData->tr - configData->tInit));
                configData->b = -(configData->transPosRef - configData->transPos_tc) /
                                ((configData->tc - configData->tf) * (configData->tc - configData->tf));
            } else { // If the initial position equals the reference position, no translation is required. Setting the 
                // final time equal to the initial time ensures the correct statement is entered when the translational 
                // states are profiled below
                configData->tf = configData->tInit;
            }
        } else { // Set parameters for the no coast option
            // Determine the total time required for the translation
            double totalTransTime = sqrt(((0.5 * fabs(configData->transPosRef - configData->transPosInit)) * 8) /
                                         configData->transAccelMax);

            // Determine the time at the end of the translation
            configData->tf = configData->tInit + totalTransTime;

            // Determine the time halfway through the translation
            configData->ts = configData->tInit + (totalTransTime / 2);

            // Define the parabolic constants for the first and second half of the translation
            configData->a = 0.5 * (configData->transPosRef - configData->transPosInit) /
                            ((configData->ts - configData->tInit) * (configData->ts - configData->tInit));
            configData->b = -0.5 * (configData->transPosRef - configData->transPosInit) /
                            ((configData->ts - configData->tf) * (configData->ts - configData->tf));
        }
    }

    // Store the current simulation time
    double t = callTime * NANO2SEC;

    // Compute the scalar translational states at the current simulation time
    if (configData->coastOption) {
        if (t <= configData->tr && configData->tf - configData->tInit != 0) { // Entered during the first ramp segment
            // The acceleration during the first ramp segment is positive if the reference position is greater than
            // the initial position. The acceleration is negative during the first ramp segment if the reference position
            // is less than the initial position
            if (configData->transPosInit < configData->transPosRef) {
                configData->transAccel = configData->transAccelMax;
            } else {
                configData->transAccel = - configData->transAccelMax;
            }
            configData->transVel = configData->transAccel * (t - configData->tInit) + configData->transVelInit;
            configData->transPos = configData->a * (t - configData->tInit) * (t - configData->tInit) 
                                   + configData->transPosInit;
        } else if (t > configData->tr && t <= configData->tc && configData->tf - configData->tInit != 0) { // Entered during the coast segment
            configData->transAccel = 0.0;
            configData->transVel = configData->transVel_tr;
            configData->transPos = configData->transVel_tr * (t - configData->tr) + configData->transPos_tr;
        } else if (t > configData->tc && t <= configData->tf && configData->tf - configData->tInit != 0) { // Entered during the second ramp segment
            // The acceleration during the second ramp segment is negative if the reference position is greater than
            // the initial position. The acceleration is positive during the second ramp segment if the reference
            // position is less than the initial position
            if (configData->transPosInit < configData->transPosRef) {
                configData->transAccel = - configData->transAccelMax;
            } else {
                configData->transAccel = configData->transAccelMax;
            }
            configData->transVel = configData->transAccel * (t - configData->tInit) + configData->transVelInit
                                   - configData->transAccel * (configData->tf - configData->tInit);
            configData->transPos = configData->b * (t - configData->tf) * (t - configData->tf) + configData->transPosRef;
        } else { // Entered when the translation is complete
            configData->transAccel = 0.0;
            configData->transVel = 0.0;
            configData->transPos = configData->transPosRef;
            configData->convergence = true;
        }
    } else {
        if (t <= configData->ts &&
            configData->tf - configData->tInit != 0) { // Entered during the first half of the translation
            if (configData->transPosInit < configData->transPosRef) {
                configData->transAccel = configData->transAccelMax;
            } else {
                configData->transAccel = -configData->transAccelMax;
            }
            configData->transVel = configData->transAccel * (t - configData->tInit) + configData->transVelInit;
            configData->transPos = configData->a * (t - configData->tInit) * (t - configData->tInit)
                                   + configData->transPosInit;
        } else if (t > configData->ts && t <= configData->tf &&
                   configData->tf - configData->tInit != 0) { // Entered during the second half of the translation
            if (configData->transPosInit < configData->transPosRef) {
                configData->transAccel = -configData->transAccelMax;
            } else {
                configData->transAccel = configData->transAccelMax;
            }
            configData->transVel = configData->transAccel * (t - configData->tInit) + configData->transVelInit
                                   - configData->transAccel * (configData->tf - configData->tInit);
            configData->transPos = configData->b * (t - configData->tf) * (t - configData->tf) + configData->transPosRef;
        } else { // Entered when the translation is complete
            configData->transAccel = 0.0;
            configData->transVel = 0.0;
            configData->transPos = configData->transPosRef;
            configData->convergence = true;
        }
    }

    // Determine the prescribed parameters: r_FM_M, rPrime_FM_M and rPrimePrime_FM_M
    v3Scale(configData->transPos, configData->transAxis_M, configData->r_FM_M);
    v3Scale(configData->transVel, configData->transAxis_M, configData->rPrime_FM_M);
    v3Scale(configData->transAccel, configData->transAxis_M, configData->rPrimePrime_FM_M);

    // Copy the required module variables to the prescribedMotionOut output message
    v3Copy(configData->r_FM_M, prescribedMotionOut.r_FM_M);
    v3Copy(configData->rPrime_FM_M, prescribedMotionOut.rPrime_FM_M);
    v3Copy(configData->rPrimePrime_FM_M, prescribedMotionOut.rPrimePrime_FM_M);
    v3Copy(configData->omega_FM_F, prescribedMotionOut.omega_FM_F);
    v3Copy(configData->omegaPrime_FM_F, prescribedMotionOut.omegaPrime_FM_F);
    v3Copy(configData->sigma_FM, prescribedMotionOut.sigma_FM);

    // Write the prescribed motion output message
    PrescribedMotionMsg_C_write(&prescribedMotionOut, &configData->prescribedMotionOutMsg, moduleID, callTime);
}
