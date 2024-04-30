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

#include "stepperMotorProfiler.h"
#include <stdbool.h>
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_stepperMotorProfiler(StepperMotorProfilerConfig *configData, int64_t moduleID) {
    StepperMotorMsg_C_init(&configData->stepperMotorOutMsg);
}

/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Reset_stepperMotorProfiler(StepperMotorProfilerConfig *configData, uint64_t callTime, int64_t moduleID) {
    // Check if the input message is linked
    if (!MotorStepCommandMsg_C_isLinked(&configData->motorStepCommandInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedRot1DOF.motorStepCommandInMsg wasn't connected.");
    }

    // Initialize the module parameters to zero
    configData->theta = configData->thetaInit;
    configData->maneuverThetaInit = configData->thetaInit;
    configData->thetaDotInit = 0.0;
    configData->thetaDot = 0.0;
    configData->thetaDotRef = 0.0;
    configData->thetaDDot = 0.0;
    configData->tInit = 0.0;
    configData->stepCount = 0;

    // Set the previous written time to a negative value to capture a message written at time zero
    configData->previousWrittenTime = -1;

    // Initialize the module boolean parameters
    configData->completion = true;
    configData->stepComplete = true;
    configData->newMsg = false;
}

/*! This method profiles the stepper motor trajectory and updates the prescribed motor states as a function of time.
The motor states are then written to the output messages.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Update_stepperMotorProfiler(StepperMotorProfilerConfig *configData, uint64_t callTime, int64_t moduleID) {
    // Create the buffer messages
    MotorStepCommandMsgPayload motorStepCommandIn;
    StepperMotorMsgPayload stepperMotorOut;

    // Zero the buffer messages
    motorStepCommandIn = MotorStepCommandMsg_C_zeroMsgPayload();
    stepperMotorOut = StepperMotorMsg_C_zeroMsgPayload();

    // Read the input message
    if (MotorStepCommandMsg_C_isWritten(&configData->motorStepCommandInMsg)) {
        motorStepCommandIn = MotorStepCommandMsg_C_read(&configData->motorStepCommandInMsg);
        // Store the number of commanded motor steps when a new message is written
        if (configData->previousWrittenTime <  MotorStepCommandMsg_C_timeWritten(&configData->motorStepCommandInMsg)) {
            configData->previousWrittenTime = MotorStepCommandMsg_C_timeWritten(&configData->motorStepCommandInMsg);
            configData->stepsCommanded = motorStepCommandIn.stepsCommanded;
            if (configData->stepsCommanded != 0) {
                configData->completion = false;
            } else {
                configData->completion = true;
            }
            configData->newMsg = true;
        }
    }

    // Reset the motor states for the next maneuver ONLY when the current step is completed
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

        // Update the intermediate initial and reference motor angles and the parabolic constants when a step is completed
        if (configData->stepComplete) {
            if (configData->stepsCommanded > 0) {
                configData->intermediateThetaInit = configData->maneuverThetaInit + (configData->stepCount * configData->stepAngle);
                configData->intermediateThetaRef = configData->maneuverThetaInit + ((configData->stepCount + 1) * configData->stepAngle);
                configData->a = 0.5 * (configData->stepAngle) / ((configData->ts - configData->tInit) * (configData->ts - configData->tInit));
                configData->b = -0.5 * (configData->stepAngle) / ((configData->ts - configData->tf) * (configData->ts - configData->tf));
            } else {
                configData->intermediateThetaInit = configData->maneuverThetaInit + (configData->stepCount * configData->stepAngle);
                configData->intermediateThetaRef = configData->maneuverThetaInit + ((configData->stepCount - 1) * configData->stepAngle);
                configData->a = 0.5 * (-configData->stepAngle) / ((configData->ts - configData->tInit) * (configData->ts - configData->tInit));
                configData->b = -0.5 * (-configData->stepAngle) / ((configData->ts - configData->tf) * (configData->ts - configData->tf));
            }
        }

        // Store the current simulation time
        double t = callTime * NANO2SEC;

        // Update the scalar motor states during each step
        if ((t < configData->ts || t == configData->ts) && configData->tf - configData->tInit != 0) { // Entered during the first half of the maneuver
            if (configData->stepsCommanded > 0 && !configData->newMsg) {
                configData->thetaDDot = configData->thetaDDotMax;
            } else if (!configData->newMsg) {
                configData->thetaDDot = -configData->thetaDDotMax;
            }
            configData->thetaDot = configData->thetaDDot * (t - configData->tInit) + configData->thetaDotInit;
            configData->theta = configData->a * (t - configData->tInit) * (t - configData->tInit) + configData->intermediateThetaInit;
            configData->stepComplete = false;
        } else if (t > configData->ts && t < configData->tf && configData->tf - configData->tInit != 0) { // Entered during the second half of the maneuver
            if (configData->stepsCommanded > 0 && !configData->newMsg){
                configData->thetaDDot = -configData->thetaDDotMax;
            } else if (!configData->newMsg) {
                configData->thetaDDot = configData->thetaDDotMax;
            }
            configData->thetaDot = configData->thetaDDot * (t - configData->tInit) + configData->thetaDotInit - configData->thetaDDot * (configData->tf - configData->tInit);
            configData->theta = configData->b * (t - configData->tf) * (t - configData->tf) + configData->intermediateThetaRef;
            configData->stepComplete = false;
        } else { // Entered when a step is complete
            configData->stepComplete = true;
            configData->thetaDDot = 0.0;
            configData->thetaDot = configData->thetaDotRef;
            configData->theta = configData->intermediateThetaRef;

            // Update the motor step count
            if (!configData->newMsg) {
                if (configData->stepsCommanded > 0) {
                    configData->stepCount++;
                } else {
                    configData->stepCount--;
                }
            } else {
                if (configData->intermediateThetaRef > configData->intermediateThetaInit) {
                    configData->stepCount++;
                } else {
                    configData->stepCount--;
                }
            }

            // Update the initial time
            configData->tInit = callTime * NANO2SEC;

            // Update the completion boolean variable only when motor actuation is complete
            if ((configData->stepCount == configData->stepsCommanded) && !configData->newMsg) {
                configData->completion = true;
            }
        }
    }

    // Copy motor information to the stepper motor message
    stepperMotorOut.theta = configData->theta;
    stepperMotorOut.thetaDot = configData->thetaDot;
    stepperMotorOut.thetaDDot = configData->thetaDDot;
    stepperMotorOut.stepsCommanded = configData->stepsCommanded;
    stepperMotorOut.stepCount = configData->stepCount;

    // Write the output messages
    StepperMotorMsg_C_write(&stepperMotorOut, &configData->stepperMotorOutMsg, moduleID, callTime);
}
