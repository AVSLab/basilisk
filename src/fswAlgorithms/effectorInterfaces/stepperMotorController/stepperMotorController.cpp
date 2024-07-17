/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "stepperMotorController.h"
#include <math.h>
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This method performs a complete reset of the module. The input message is checked to ensure it is linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotorController::Reset(uint64_t callTime) {
    if (!this->motorRefAngleInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR, "stepperMotorController.motorRefAngleInMsg wasn't connected.");
    }

    // Set module parameter values for module reset
    this->stepCount = 0;
    this->stepsCommanded = 0;
    this->deltaTheta = 0.0;
    this->deltaSimTime = 0.0;
    this->previousWrittenTime = -1.0;
}

/*! This method computes the required number of motor steps given a reference motor angle message and tracks the
motor actuation in time.
 @return void
 @param callTime [ns] Time the method is called
*/
void StepperMotorController::UpdateState(uint64_t callTime) {
    // Create the buffer messages
    HingedRigidBodyMsgPayload motorRefAngleIn;
    MotorStepCommandMsgPayload motorStepCommandOut;

    // Zero the buffer messages
    motorRefAngleIn = HingedRigidBodyMsgPayload();
    motorStepCommandOut = MotorStepCommandMsgPayload();

    // Read the input message
    if (this->motorRefAngleInMsg.isWritten()) {
        motorRefAngleIn = this->motorRefAngleInMsg();
    }

    // Store the time the input message was written
    double hingedRigidBodyMsgTimeWritten = NANO2SEC * this->motorRefAngleInMsg.timeWritten();

    // The steps commanded are calculated and updated in this statement when a new message is written
    if (this->previousWrittenTime <  hingedRigidBodyMsgTimeWritten) {

        // Update the previous written time
        this->previousWrittenTime = hingedRigidBodyMsgTimeWritten;

        // Read in the desired angle
        this->thetaRef = motorRefAngleIn.theta;

        // Calculate the difference between the desired angle and the current motor angle, ensuring that the current
        // motor angle is updated to the next multiple of the motor step angle if actuation is interrupted
        if (this->theta > 0) {
            this->deltaTheta = this->thetaRef - (ceil(this->theta / this->stepAngle) * this->stepAngle);
        } else {
            this->deltaTheta = this->thetaRef - (floor(this->theta / this->stepAngle) * this->stepAngle);
        }

        // Calculate the integer number of steps commanded, ensuring to rounding to the nearest integer step
        double tempStepsCommanded = this->deltaTheta / this->stepAngle;
        if ((ceil(tempStepsCommanded) - tempStepsCommanded) > (tempStepsCommanded - floor(tempStepsCommanded))) {
            this->stepsCommanded = floor(tempStepsCommanded);
        } else {
            this->stepsCommanded = ceil(tempStepsCommanded);
        }

        // Update the desired motor angle
        this->thetaRef = this->theta + (this->stepsCommanded * this->stepAngle);

        // Update the output message buffer
        motorStepCommandOut.stepsCommanded = this->stepsCommanded;

        // Reset the steps taken to zero
        this->stepCount = 0;

        // Write the output message
        this->motorStepCommandOutMsg.write(&motorStepCommandOut, moduleID, callTime);
    }

    // Calculate the time elapsed since the last message was written
    this->deltaSimTime = (NANO2SEC * callTime) - this->previousWrittenTime;

    // Update the motor information
    if (this->stepsCommanded > 0) {
        this->stepCount = floor(this->deltaSimTime / this->stepTime);
        this->theta = this->thetaInit + this->stepAngle * (this->deltaSimTime / this->stepTime);
        if (this->theta >= this->thetaRef) {
            this->stepCount = this->stepsCommanded;
            this->theta = this->thetaRef;
            this->thetaInit = this->thetaRef;
        }
    } else {
        this->stepCount = -floor(this->deltaSimTime / this->stepTime);
        this->theta =  this->thetaInit - this->stepAngle * (this->deltaSimTime / this->stepTime);
        if (this->theta <= this->thetaRef) {
            this->stepCount = this->stepsCommanded;
            this->theta = this->thetaRef;
            this->thetaInit = this->thetaRef;
        }
    }
}

/*! Getter method for the initial motor angle.
 @return double
*/
double StepperMotorController::getThetaInit() const {
    return this->thetaInit;
}

/*! Getter method for the motor step angle.
 @return double
*/
double StepperMotorController::getStepAngle() const {
    return this->stepAngle;
}

/*! Getter method for the motor step time.
 @return double
*/
double StepperMotorController::getStepTime() const {
    return this->stepTime;
}

/*! Setter method for the initial motor angle.
 @return void
 @param thetaInit [rad] Initial motor angle
*/
void StepperMotorController::setThetaInit(const double thetaInit) {
    this->thetaInit = thetaInit;
    this->theta = thetaInit;
}

/*! Setter method for the motor step angle.
 @return void
 @param stepAngle [rad] Motor step angle
*/
void StepperMotorController::setStepAngle(const double stepAngle) {
    this->stepAngle = stepAngle;
}

/*! Setter method for the motor step time.
 @return void
 @param stepTime [s] Motor step time
*/
void StepperMotorController::setStepTime(const double stepTime) {
    this->stepTime = stepTime;
}
