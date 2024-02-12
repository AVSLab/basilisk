/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "prescribedLinearTranslation.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <cmath>


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param callTime [ns] Simulation time the method is called
*/
void PrescribedLinearTranslation::Reset(uint64_t callTime)
{
    if (!this->linearTranslationRigidBodyInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: prescribedLinearTranslation.linearTranslationRigidBodyInMsg wasn't connected.");
    }

    // Set the initial time
    this->tInit = 0.0;

    this->transPos = this->transPosInit;
    this->transVel = 0.0;

    // Set the initial convergence to true to enter the correct loop in the Update() method on the first pass
    this->convergence = true;
}

/*! This method profiles the prescribed trajectory and updates the prescribed states as a function of time.
The prescribed states are then written to the output message.
 @return void
 @param callTime [ns] Simulation time the method is called
*/
void PrescribedLinearTranslation::UpdateState(uint64_t callTime)
{
    LinearTranslationRigidBodyMsgPayload linearTranslationRigidBodyIn;
    PrescribedTranslationMsgPayload prescribedTranslationMsgOut;

    prescribedTranslationMsgOut = PrescribedTranslationMsgPayload();

    linearTranslationRigidBodyIn = LinearTranslationRigidBodyMsgPayload();
    if (this->linearTranslationRigidBodyInMsg.isWritten()) {
        linearTranslationRigidBodyIn = this->linearTranslationRigidBodyInMsg();
    }

    // This loop is entered (a) initially and (b) when each translation is complete.
    // The parameters used to profile the translation are updated in this statement.
    if (this->linearTranslationRigidBodyInMsg.timeWritten() <= callTime && this->convergence) {
        // Update the initial time as the current simulation time
        this->tInit = callTime * NANO2SEC;

        // Store the reference scalar position
        this->transPosRef = linearTranslationRigidBodyIn.rho;

        this->transPosInit = this->transPos;

        // Set the convergence to false until the translation is complete
        this->convergence = false;

        // Set the parameters required to profile the translation
        if (this->coastOptionRampDuration > 0.0) {
            this->computeCoastParameters();
        } else {
            this->computeParametersNoCoast();
        }
    }

    double t = callTime * NANO2SEC;

    // Compute the scalar translational states at the current simulation time
    if (this->coastOptionRampDuration > 0.0) {
        if (this->isInFirstRampSegment(t)) {
            this->computeFirstRampSegment(t);
        } else if (this->isInCoastSegment(t)) {
            this->computeCoastSegment(t);
        } else if (this->isInSecondRampSegment(t)) {
            this->computeSecondRampSegment(t);
        } else {
            this->computeTranslationComplete();
        }
    } else {
        if (this->isInFirstRampSegmentNoCoast(t)) {
            this->computeFirstRampSegment(t);
        } else if (this->isInSecondRampSegmentNoCoast(t)) {
            this->computeSecondRampSegment(t);
        } else {
            this->computeTranslationComplete();
        }
    }

    // [m] Translational body position relative to the Mount frame expressed in M frame components
    Eigen::Vector3d r_FM_M = this->transPos*this->transHat_M;

    // [m/s] B frame time derivative of r_FM_M expressed in M frame components
    Eigen::Vector3d rPrime_FM_M = this->transVel*this->transHat_M;

    // [m/s^2] B frame time derivative of rPrime_FM_M expressed in M frame components
    Eigen::Vector3d rPrimePrime_FM_M = this->transAccel*this->transHat_M;

    // Write the output message
    eigenVector3d2CArray(r_FM_M, prescribedTranslationMsgOut.r_FM_M);
    eigenVector3d2CArray(rPrime_FM_M, prescribedTranslationMsgOut.rPrime_FM_M);
    eigenVector3d2CArray(rPrimePrime_FM_M, prescribedTranslationMsgOut.rPrimePrime_FM_M);

    this->prescribedTranslationOutMsg.write(&prescribedTranslationMsgOut, this->moduleID, callTime);
}

/*! This method determines if the current time is within the first ramp segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInFirstRampSegment(double t) const {
    return (t <= this->tr && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the coast segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInCoastSegment(double t) const {
    return (t > this->tr && t <= this->tc && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the second ramp segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInSecondRampSegment(double t) const {
    return (t > this->tc && t <= this->tf && this->tf - this->tInit != 0);
}

/*! This method computes the required parameters for the translation with a coast period.
 @return void
*/
void PrescribedLinearTranslation::computeCoastParameters() {
    if (this->transPosInit != this->transPosRef) {
        // Determine the time at the end of the first ramp segment
        this->tr = this->tInit + this->coastOptionRampDuration;

        // Determine the position and velocity at the end of the ramp segment/start of the coast segment
        if (this->transPosInit < this->transPosRef) {
            this->transPos_tr = (0.5 * this->transAccelMax * this->coastOptionRampDuration * this->coastOptionRampDuration)
                                 + this->transPosInit;
            this->transVel_tr = this->transAccelMax * this->coastOptionRampDuration;
        } else {
            this->transPos_tr =
                    -((0.5 * this->transAccelMax * this->coastOptionRampDuration * this->coastOptionRampDuration))
                    + this->transPosInit;
            this->transVel_tr = -this->transAccelMax * this->coastOptionRampDuration;
        }

        // Determine the distance traveled during the coast period
        double deltaPosCoast = this->transPosRef - this->transPosInit - 2 * (this->transPos_tr - this->transPosInit);

        // Determine the time duration of the coast segment
        double tCoast = fabs(deltaPosCoast) / fabs(this->transVel_tr);

        // Determine the time at the end of the coast segment
        this->tc = this->tr + tCoast;

        // Determine the position [m] at the end of the coast segment
        double transPos_tc = this->transPos_tr + deltaPosCoast;

        // Determine the time at the end of the translation
        this->tf = this->tc + this->coastOptionRampDuration;

        // Define the parabolic constants for the first and second ramp segments of the translation
        this->a = (this->transPos_tr - this->transPosInit) / ((this->tr - this->tInit) * (this->tr - this->tInit));
        this->b = -(this->transPosRef - transPos_tc) / ((this->tc - this->tf) * (this->tc - this->tf));
    } else {
        // If the initial position equals the reference position, no translation is required.
        this->tf = this->tInit;
    }
}

/*! This method computes the scalar translational states for the coast option coast period.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeCoastSegment(double t) {
    this->transAccel = 0.0;
    this->transVel = this->transVel_tr;
    this->transPos = this->transVel_tr * (t - this->tr) + this->transPos_tr;
}

/*! This method determines if the current time is within the first ramp segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInFirstRampSegmentNoCoast(double t) const {
    return (t <= this->ts && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the second ramp segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInSecondRampSegmentNoCoast(double t) const {
    return (t > this->ts && t <= this->tf && this->tf - this->tInit != 0);
}

/*! This method computes the required parameters for the translation with no coast period.
 @return void
*/
void PrescribedLinearTranslation::computeParametersNoCoast() {
    // Determine the total time required for the translation
    double totalTransTime = sqrt(((0.5 * fabs(this->transPosRef - this->transPosInit)) * 8) / this->transAccelMax);

    // Determine the time at the end of the translation
    this->tf = this->tInit + totalTransTime;

    // Determine the time halfway through the translation
    this->ts = this->tInit + (totalTransTime / 2);

    // Define the parabolic constants for the first and second half of the translation
    this->a = 0.5 * (this->transPosRef - this->transPosInit) / ((this->ts - this->tInit) * (this->ts - this->tInit));
    this->b = -0.5 * (this->transPosRef - this->transPosInit) / ((this->ts - this->tf) * (this->ts - this->tf));
}

/*! This method computes the scalar translational states for the first ramp segment. The acceleration during the first
 * ramp segment is positive if the reference position is greater than the initial position. The acceleration is
 * negative during the first ramp segment if the reference position is less than the initial position.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeFirstRampSegment(double t) {
    if (this->transPosInit < this->transPosRef) {
        this->transAccel = this->transAccelMax;
    } else {
        this->transAccel = -this->transAccelMax;
    }
    this->transVel = this->transAccel * (t - this->tInit);
    this->transPos = this->a * (t - this->tInit) * (t - this->tInit) + this->transPosInit;
}

/*! This method computes the scalar translational states for the second ramp segment. The acceleration during the
 * second ramp segment is negative if the reference position is greater than the initial position. The acceleration
 * is positive during the second ramp segment if the reference position is less than the initial position.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeSecondRampSegment(double t) {
    if (this->transPosInit < this->transPosRef) {
        this->transAccel = -this->transAccelMax;
    } else {
        this->transAccel = this->transAccelMax;
    }
    this->transVel = this->transAccel * (t - this->tInit) - this->transAccel * (this->tf - this->tInit);
    this->transPos = this->b * (t - this->tf) * (t - this->tf) + this->transPosRef;
}

/*! This method computes the scalar translational states when the translation is complete.
 @return void
*/
void PrescribedLinearTranslation::computeTranslationComplete() {
    this->transAccel = 0.0;
    this->transVel = 0.0;
    this->transPos = this->transPosRef;
    this->convergence = true;
}

/*! Setter method for the coast option ramp duration.
 @return void
 @param rampDuration [s] Ramp segment time duration
*/
void PrescribedLinearTranslation::setCoastOptionRampDuration(double rampDuration) {
    this->coastOptionRampDuration = rampDuration;
}

/*! Setter method for the ramp segment scalar linear acceleration.
 @return void
 @param transAccelMax [m/s^2] Ramp segment linear angular acceleration
*/
void PrescribedLinearTranslation::setTransAccelMax(double transAccelMax) {
    this->transAccelMax = transAccelMax;
}

/*! Setter method for the translating body axis of translation.
 @return void
 @param transHat_M Translating body axis of translation (unit vector)
*/
void PrescribedLinearTranslation::setTransHat_M(const Eigen::Vector3d &transHat_M) {
    this->transHat_M = transHat_M;
}

/*! Setter method for the initial translating body hub-relative position.
 @return void
 @param transPosInit [m] Initial translating body position relative to the hub
*/
void PrescribedLinearTranslation::setTransPosInit(double transPosInit) {
    this->transPosInit = transPosInit;
}

/*! Getter method for the coast option ramp duration.
 @return double
*/
double PrescribedLinearTranslation::getCoastOptionRampDuration() const {
    return this->coastOptionRampDuration;
}

/*! Getter method for the ramp segment scalar linear acceleration.
 @return double
*/
double PrescribedLinearTranslation::getTransAccelMax() const {
    return this->transAccelMax;
}

/*! Getter method for the translating body axis of translation.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &PrescribedLinearTranslation::getTransHat_M() const {
    return this->transHat_M;
}

/*! Getter method for the initial translating body position.
 @return double
*/
double PrescribedLinearTranslation::getTransPosInit() const {
    return this->transPosInit;
}
