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

/* Import the module header file */
#include "prescribedRot1DOF.h"

/* Other required files to import */
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, int64_t moduleID)
{
    // Initialize the output messages
    PrescribedMotionMsg_C_init(&configData->prescribedMotionOutMsg);
    StepperMotorMsg_C_init(&configData->stepperMotorOutMsg);
}


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Reset_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // Check if the required input message is linked
    if (!MotorStepCountMsg_C_isLinked(&configData->motorStepCountInMsg))
    {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedRot1DOF.motorStepCountInMsg wasn't connected.");
    }

    // Set the initial time
    configData->tInit = 0.0;

    // Set the initial step count to zero
    configData->stepCount = 0;

    configData->numSteps = 0;

    // Calculate max angular acceleration for a single step
    configData->thetaDDotMax = configData->stepAngle / (0.25 * configData->stepTime *  configData->stepTime);

    // Set the initial completion to true
    configData->completion = true;
    configData->stepComplete = true;
    configData->newMsg = false;

    configData->previousWrittenTime = -1;

    // Calculate the current ange and angle rate
    double prv_FM_array[3];
    MRP2PRV(configData->sigma_FM, prv_FM_array);
    configData->thetaInit = v3Dot(prv_FM_array, configData->rotAxis_M);
    configData->thetaDotInit = v3Norm(configData->omega_FM_F);

    configData->maneuverThetaInit = configData->thetaInit;

    configData->theta = configData->thetaInit;
    configData->thetaDot = configData->thetaDotInit;
    configData->thetaDDot = 0.0;
}


/*! This method profiles the prescribed trajectory and updates the prescribed states as a function of time.
The prescribed states are then written to the output message.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Update_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // Create the buffer messages
    MotorStepCountMsgPayload motorStepCountIn;
    StepperMotorMsgPayload stepperMotorOut;
    PrescribedMotionMsgPayload prescribedMotionOut;

    // Zero the buffer messages
    stepperMotorOut = StepperMotorMsg_C_zeroMsgPayload();
    prescribedMotionOut = PrescribedMotionMsg_C_zeroMsgPayload();
    motorStepCountIn = MotorStepCountMsg_C_zeroMsgPayload();

    // Read the input message
    if (MotorStepCountMsg_C_isWritten(&configData->motorStepCountInMsg))
    {
        motorStepCountIn = MotorStepCountMsg_C_read(&configData->motorStepCountInMsg);
        // Store the number of commanded motor steps
        if ( (motorStepCountIn.numSteps != 0) && (configData->previousWrittenTime <  MotorStepCountMsg_C_timeWritten(&configData->motorStepCountInMsg)) ) {
            configData->previousWrittenTime = MotorStepCountMsg_C_timeWritten(&configData->motorStepCountInMsg);
            configData->numSteps = motorStepCountIn.numSteps;
            configData->completion = false;
            configData->newMsg = true;
        }
    }

    if (!(configData->completion)) {
        if (configData->newMsg && configData->stepComplete) {

            // Update the step count to zero
            configData->stepCount = 0;

            // Calculate the current ange and angle rate
            configData->maneuverThetaInit = configData->theta;
            configData->thetaDotInit = configData->thetaDot;

            // Store the initial time as the current simulation time
            configData->tInit = callTime * NANO2SEC;

            configData->newMsg = false;
        }

        // Define temporal information for the maneuver
        configData->tf = configData->tInit + configData->stepTime;
        configData->ts = configData->tInit + configData->stepTime / 2;

        // Find intermediate initial and reference angles
        if (configData->stepComplete) {
            if (configData->numSteps > 0) {
                configData->intermediateThetaInit = configData->maneuverThetaInit + (configData->stepCount * configData->stepAngle);
                configData->intermediateThetaRef = configData->maneuverThetaInit + ((configData->stepCount + 1) * configData->stepAngle);
                // Define the parabolic constants for the first and second half of the maneuver
                configData->a = 0.5 * (configData->stepAngle) / ((configData->ts - configData->tInit) * (configData->ts - configData->tInit));
                configData->b = -0.5 * (configData->stepAngle) / ((configData->ts - configData->tf) * (configData->ts - configData->tf));
            }
            else {
                configData->intermediateThetaInit = configData->maneuverThetaInit + (configData->stepCount * configData->stepAngle);
                configData->intermediateThetaRef = configData->maneuverThetaInit + ((configData->stepCount - 1) * configData->stepAngle);
                // Define the parabolic constants for the first and second half of the maneuver
                configData->a = 0.5 * (-configData->stepAngle) / ((configData->ts - configData->tInit) * (configData->ts - configData->tInit));
                configData->b = -0.5 * (-configData->stepAngle) / ((configData->ts - configData->tf) * (configData->ts - configData->tf));
            }
        }

        // Store the current simulation time
        double t = callTime * NANO2SEC;

        // Compute the prescribed scalar states at the current simulation time
        if ((t < configData->ts || t == configData->ts) && configData->tf - configData->tInit != 0) { // Entered during the first half of the maneuver
            if (configData->numSteps > 0 && !configData->newMsg){
                configData->thetaDDot = configData->thetaDDotMax;
            }
            else if (!configData->newMsg){
                configData->thetaDDot = -configData->thetaDDotMax;
            }
            configData->thetaDot = configData->thetaDDot * (t - configData->tInit) + configData->thetaDotInit;
            configData->theta = configData->a * (t - configData->tInit) * (t - configData->tInit) + configData->intermediateThetaInit;
            configData->stepComplete = false;
        } else if (t > configData->ts && t <= configData->tf && configData->tf - configData->tInit != 0) { // Entered during the second half of the maneuver
            if (configData->numSteps > 0 && !configData->newMsg){
                configData->thetaDDot = -configData->thetaDDotMax;
            }
            else if (!configData->newMsg){
                configData->thetaDDot = configData->thetaDDotMax;
            }
            configData->thetaDot = configData->thetaDDot * (t - configData->tInit) + configData->thetaDotInit -
                    configData->thetaDDot * (configData->tf - configData->tInit);
            configData->theta = configData->b * (t - configData->tf) * (t - configData->tf) + configData->intermediateThetaRef;
            configData->stepComplete = false;
        }
        else { // Entered when a step is complete
            configData->stepComplete = true;

            configData->thetaDDot = 0.0;
            configData->thetaDot = configData->thetaDotRef;
            configData->theta = configData->intermediateThetaRef;

            // Increment the step count
            if (!configData->newMsg){
                if (configData->numSteps > 0){
                    configData->stepCount++;
                }
                else{
                    configData->stepCount--;
                }
            }

            // Store the initial time as the current simulation time
            configData->tInit = callTime * NANO2SEC;

            if ( (configData->stepCount == configData->numSteps) && !configData->newMsg ){
                configData->completion = true;
            }
        }
    }

    // Determine the prescribed parameters: omega_FM_F and omegaPrime_FM_F
    v3Normalize(configData->rotAxis_M, configData->rotAxis_M);
    v3Scale(configData->thetaDot, configData->rotAxis_M, configData->omega_FM_F);
    v3Scale(configData->thetaDDot, configData->rotAxis_M, configData->omegaPrime_FM_F);

    // Determine dcm_FF0
    double dcm_FF0[3][3];
    double prv_FF0_array[3];
    double theta_FF0 = 3.1415 / 180 * (configData->theta - configData->thetaInit);
    v3Scale(theta_FF0, configData->rotAxis_M, prv_FF0_array);
    PRV2C(prv_FF0_array, dcm_FF0);

    // Determine dcm_F0M
    double dcm_F0M[3][3];
    double prv_F0M_array[3];
    v3Scale(3.1415 / 180 * configData->thetaInit, configData->rotAxis_M, prv_F0M_array);
    PRV2C(prv_F0M_array, dcm_F0M);

    // Determine dcm_FM
    double dcm_FM[3][3];
    m33MultM33(dcm_FF0, dcm_F0M, dcm_FM);

    // Determine the prescribed parameter: sigma_FM
    C2MRP(dcm_FM, configData->sigma_FM);

    // Copy the module variables to the prescribedMotionOut output message
    v3Copy(configData->r_FM_M, prescribedMotionOut.r_FM_M);
    v3Copy(configData->rPrime_FM_M, prescribedMotionOut.rPrime_FM_M);
    v3Copy(configData->rPrimePrime_FM_M, prescribedMotionOut.rPrimePrime_FM_M);
    v3Copy(configData->omega_FM_F, prescribedMotionOut.omega_FM_F);
    v3Copy(configData->omegaPrime_FM_F, prescribedMotionOut.omegaPrime_FM_F);
    v3Copy(configData->sigma_FM, prescribedMotionOut.sigma_FM);

    // Copy motor information to the stepper motor message
    stepperMotorOut.theta = configData->theta;
    stepperMotorOut.thetaDot = configData->thetaDot;
    stepperMotorOut.thetaDDot = configData->thetaDDot;
    stepperMotorOut.numSteps = configData->numSteps;
    stepperMotorOut.stepCount = configData->stepCount;

    // Write the output messages
    PrescribedMotionMsg_C_write(&prescribedMotionOut, &configData->prescribedMotionOutMsg, moduleID, callTime);
    StepperMotorMsg_C_write(&stepperMotorOut, &configData->stepperMotorOutMsg, moduleID, callTime);
}
