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

#include "stepperMotor.h"
#include <stdbool.h>
#include <math.h>
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_stepperMotor(StepperMotorConfig* configData, int64_t moduleID) {
    MotorStepCountMsg_C_init(&configData->motorStepCountOutMsg);
}


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Reset_stepperMotor(StepperMotorConfig* configData, uint64_t callTime, int64_t moduleID) {
    // Check if the required input message is linked
    if (!HingedRigidBodyMsg_C_isLinked(&configData->spinningBodyInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: stepperMotor.spinningBodyInMsg wasn't connected.");
    }

    // Set the initial module variables to zero
    configData->stepCount = 0;
    configData->stepsCommanded = 0;
    configData->initAngle = 0.0;
    configData->deltaAngle = 0.0;
    configData->deltaSimTime = 0.0;
    configData->previousWrittenTime = 0.0;

    // Set firstCall to true to capture a message written at time zero
    configData->firstCall = true;
}

/*! This method computes the required number of motor steps given a desired motor angle message and follows the
 * motor actuation in time.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Update_stepperMotor(StepperMotorConfig *configData, uint64_t callTime, int64_t moduleID) {
    // Create the buffer messages
    MotorStepCountMsgPayload motorStepCountOut;
    HingedRigidBodyMsgPayload spinningBodyIn;

    // Zero the output messages
    motorStepCountOut = MotorStepCountMsg_C_zeroMsgPayload();

    // Read the input message
    spinningBodyIn = HingedRigidBodyMsg_C_zeroMsgPayload();
    if (HingedRigidBodyMsg_C_isWritten(&configData->spinningBodyInMsg)) {
        spinningBodyIn = HingedRigidBodyMsg_C_read(&configData->spinningBodyInMsg);
    }

    // Store the time the input message was written
    double hingedRigidBodyMsgTimeWritten = NANO2SEC * HingedRigidBodyMsg_C_timeWritten(&configData->spinningBodyInMsg);

    // The steps commanded are calculated and updated in this statement when a new message is written
    if (configData->previousWrittenTime <  hingedRigidBodyMsgTimeWritten || configData->firstCall) {
        configData->firstCall = false;

        // Update the previous written time
        configData->previousWrittenTime = hingedRigidBodyMsgTimeWritten;

        // Read in the desired angle
        configData->desiredAngle = spinningBodyIn.theta;

        // Calculate the difference between the desired angle and the current motor angle, ensuring that the current
        // motor angle is updated to the next multiple of the motor step angle if actuation is interrupted
        if (configData->currentAngle > 0) {
            configData->deltaAngle = configData->desiredAngle - (ceil(configData->currentAngle / configData->stepAngle) * configData->stepAngle);
        } else {
            configData->deltaAngle = configData->desiredAngle - (floor(configData->currentAngle / configData->stepAngle) * configData->stepAngle);
        }

        // Calculate the integer number of steps commanded, ensuring to rounding to the nearest integer step
        double tempStepsCommanded = configData->deltaAngle / configData->stepAngle;
        if ((ceil(tempStepsCommanded) - tempStepsCommanded) > (tempStepsCommanded - floor(tempStepsCommanded))) {
            configData->stepsCommanded = floor(tempStepsCommanded);
        } else {
            configData->stepsCommanded = ceil(tempStepsCommanded);
        }

        // Update the output message buffer
        motorStepCountOut.stepsCommanded = configData->stepsCommanded;

        // Reset the steps taken to zero
        configData->stepCount = 0; 
    }

    // Calculate the time elapsed since the last message was written
    configData->deltaSimTime = (NANO2SEC * callTime) - configData->previousWrittenTime;

    // Update the motor information
    if (configData->stepsCommanded > 0) {
        configData->stepCount = floor(configData->deltaSimTime / configData->stepTime);
        configData->currentAngle = configData->initAngle + configData->stepAngle * (configData->deltaSimTime / configData->stepTime);
        if (configData->currentAngle >= configData->desiredAngle) {
            configData->stepCount = configData->stepsCommanded;
            configData->currentAngle = configData->desiredAngle;
            configData->initAngle = configData->desiredAngle;
        }
    } else {
        configData->stepCount = -floor(configData->deltaSimTime / configData->stepTime);
        configData->currentAngle =  configData->initAngle - configData->stepAngle * (configData->deltaSimTime / configData->stepTime);
        if (configData->currentAngle <= configData->desiredAngle) {
            configData->stepCount = configData->stepsCommanded;
            configData->currentAngle = configData->desiredAngle;
            configData->initAngle = configData->desiredAngle;
        }
    }

    // Write the output message
    MotorStepCountMsg_C_write(&motorStepCountOut, &configData->motorStepCountOutMsg, moduleID, callTime);
}
