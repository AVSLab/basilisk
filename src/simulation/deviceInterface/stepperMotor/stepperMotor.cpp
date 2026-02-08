/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "architecture/utilities/macroDefinitions.h"
#include <cassert>
#include <cmath>

/*! Module reset method.
 @param callTime [ns] Time the method is called
*/
void StepperMotor::Reset(uint64_t callTime) {
    assert(this->motorStepCommandInMsg.isLinked());

    // Reset required module parameters
    this->theta = this->thetaInit;
    this->thetaDot = 0.0;
    this->thetaDDot = 0.0;
    this->tInit = 0.0;
    this->stepCount = 0;
    this->previousWrittenTime = -1;
    this->actuationComplete = true;
    this->stepComplete = true;
    this->newMsg = false;
    this->interruptMsg = false;

    // Set motor maximum angular acceleration
    this->thetaDDotMax = this->stepAngle / (0.25 * this->stepTime * this->stepTime);  // [rad/s^2]
}

/*! Module update method. This method profiles the stepper motor actuation as a function of time. The motor states
 are then written to the output message.
 @param callTime [ns] Time the method is called
*/
void StepperMotor::UpdateState(uint64_t callTime) {
    MotorStepCommandMsgPayload motorStepCommandIn{};

    // Read the input message
    if (this->motorStepCommandInMsg.isWritten()) {
        motorStepCommandIn = this->motorStepCommandInMsg();
        // Store the number of commanded motor steps when a new message is written
        if (this->previousWrittenTime < this->motorStepCommandInMsg.timeWritten()) {
            this->stepsCommanded = motorStepCommandIn.stepsCommanded;
            this->previousWrittenTime = this->motorStepCommandInMsg.timeWritten();

            // Update booleans
            this->newMsg = true;
            if (this->actuationComplete) {
                this->interruptMsg = false;
            } else {
                this->interruptMsg = true;
            }
            if (this->stepsCommanded == 0) {
                this->actuationComplete = true;
                this->stepCount = 0;
            } else {
                this->actuationComplete = false;
            }
        }
    }

    // Actuate the motor only if a current actuation segment is not complete
    double t = callTime * NANO2SEC;
    if (!(this->actuationComplete)) {
        // Reset the motor immediately after a new non-interrupting request is received
        if ((this->newMsg && !this->interruptMsg) || (this->interruptMsg && this->stepComplete)) {
            this->resetMotor();
        }
        this->actuateMotor(t);
    } else {
        this->tInit = t;
        this->thetaDDot = 0.0;
    }

    // Write the output message
    StepperMotorMsgPayload stepperMotorOut{};
    stepperMotorOut.theta = this->theta;
    stepperMotorOut.thetaDot = this->thetaDot;
    stepperMotorOut.thetaDDot = this->thetaDDot;
    stepperMotorOut.stepsCommanded = this->stepsCommanded;
    stepperMotorOut.stepCount = this->stepCount;
    this->stepperMotorOutMsg.write(&stepperMotorOut, moduleID, callTime);
}

/*! This method is used to simulate the stepper motor actuation in time.
 @param t [s] Time the method is called
*/
void StepperMotor::actuateMotor(double t) {
    // Update the motor step parameters when a step is completed
    if (this->stepComplete) {
        this->updateStepParameters();
    }

    // Update the motor states during each step
    if (this->isInStepFirstHalf(t)) {
        this->computeStepFirstHalf(t);
    } else if (this->isInStepSecondHalf(t)) {
        this->computeStepSecondHalf(t);
    } else {
        this->computeStepComplete(t);
    }
}

/*! This method resets the motor states when the current request is complete and a new request is received.
*/
void StepperMotor::resetMotor() {
    this->stepCount = 0;
    this->thetaInit = this->theta;
    this->newMsg = false;
    this->interruptMsg = false;
}

/*! This method updates the step parameters after a step is completed.
*/
void StepperMotor::updateStepParameters() {
    this->stepComplete = false;
    this->tf = this->tInit + this->stepTime;
    this->ts = this->tInit + this->stepTime / 2;
    this->intermediateThetaInit = this->thetaInit + (this->stepCount * this->stepAngle);

    if (this->stepsCommanded > 0) {
        this->intermediateThetaRef = this->thetaInit + ((this->stepCount + 1) * this->stepAngle);
        this->a = 0.5 * (this->stepAngle) / ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (this->stepAngle) / ((this->ts - this->tf) * (this->ts - this->tf));
    } else {
        this->intermediateThetaRef = this->thetaInit + ((this->stepCount - 1) * this->stepAngle);
        this->a = 0.5 * (-this->stepAngle) / ((this->ts - this->tInit) * (this->ts - this->tInit));
        this->b = -0.5 * (-this->stepAngle) / ((this->ts - this->tf) * (this->ts - this->tf));
    }
}

/*! This method determines if the motor is in the first half of a step.
 @return bool
 @param t [s] Time the method is called
*/
bool StepperMotor::isInStepFirstHalf(double t) { return (t < this->ts && std::abs(this->ts - t) > 1e-5); }

/*! This method computes the motor states during the first half of each step.
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepFirstHalf(double t) {
    if (this->intermediateThetaRef > this->intermediateThetaInit) {
        this->thetaDDot = this->thetaDDotMax;
    } else {
        this->thetaDDot = -this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit);
    this->theta = this->a * (t - this->tInit) * (t - this->tInit) + this->intermediateThetaInit;
}

/*! This method determines if the motor is in the second half of a step.
 @return bool
 @param t [s] Time the method is called
*/
bool StepperMotor::isInStepSecondHalf(double t) {
    return ((t >= this->ts || std::abs(this->ts - t) < 1e-5) && std::abs(this->tf - t) > 1e-5);
}

/*! This method computes the motor states during the second half of each step.
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepSecondHalf(double t) {
    if (this->intermediateThetaRef > this->intermediateThetaInit) {
        this->thetaDDot = -this->thetaDDotMax;
    } else {
        this->thetaDDot = this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit) - this->thetaDDot * (this->tf - this->tInit);
    this->theta = this->b * (t - this->tf) * (t - this->tf) + this->intermediateThetaRef;
}

/*! This method computes the motor states when a step is complete.
 @param t [s] Time the method is called
*/
void StepperMotor::computeStepComplete(double t) {
    this->stepComplete = true;
    this->thetaDot = 0.0;
    this->theta = this->intermediateThetaRef;
    this->tInit = t;

    // Update the motor step count
    if (this->intermediateThetaRef > this->intermediateThetaInit) {
        this->stepCount++;
    } else {
        this->stepCount--;
    }

    // Update the actuationComplete boolean variable only when motor actuation is complete
    if ((this->stepCount == this->stepsCommanded) && !this->interruptMsg) {
        this->actuationComplete = true;
    }
}

/*! Getter method for the initial motor angle called `thetaInit`.
 @return double
*/
double StepperMotor::getThetaInit() const { return this->thetaInit; }

/*! Getter method for the motor step angle called `stepAngle`.
 @return double
*/
double StepperMotor::getStepAngle() const { return this->stepAngle; }

/*! Getter method for the motor step time called `stepTime`.
 @return double
*/
double StepperMotor::getStepTime() const { return this->stepTime; }

/*! Getter method for the maximum motor angular acceleration called `thetaDDotMax`.
 @return double
*/
double StepperMotor::getThetaDDotMax() const { return this->thetaDDotMax; }

/*! Setter method for the initial motor angle.
 @param thetaInit [rad] Initial motor angle
*/
void StepperMotor::setThetaInit(const double thetaInit) { this->thetaInit = thetaInit; }

/*! Setter method for the motor step angle.
 @param stepAngle [rad] Motor step angle
*/
void StepperMotor::setStepAngle(const double stepAngle) {
    assert(stepAngle > 0.0);
    this->stepAngle = std::abs(stepAngle);
}

/*! Setter method for the motor step time.
 @param stepTime [s] Motor step time
*/
void StepperMotor::setStepTime(const double stepTime) {
    assert(stepTime > 0.0);
    this->stepTime = std::abs(stepTime);
}
