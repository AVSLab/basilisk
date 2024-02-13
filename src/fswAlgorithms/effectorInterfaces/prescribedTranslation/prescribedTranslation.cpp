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

#include "prescribedTranslation.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <cmath>


/*! This method performs a complete reset of the module. The input messages are checked to ensure they are linked.
 @return void
 @param callTime [ns] Simulation time the method is called
*/
void PrescribedTranslation::Reset(uint64_t callTime)
{
    if (!this->linearTranslationRigidBodyInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: prescribedTrans.linearTranslationRigidBodyInMsg wasn't connected.");
    }

    // Set the initial time
    this->tInit = 0.0;

    this->transPos = this->transPosInit;
    this->transVelInit = 0.0;
    this->transVel = 0.0;

    // Set the initial convergence to true to enter the correct loop in the Update() method on the first pass
    this->convergence = true;
}

/*! This method profiles the prescribed trajectory and updates the prescribed states as a function of time.
The prescribed states are then written to the output message.
 @return void
 @param callTime [ns] Simulation time the method is called
*/
void PrescribedTranslation::UpdateState(uint64_t callTime)
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
        if (this->coastRampDuration > 0.0) {
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

    // Determine the prescribed parameters
    this->r_FM_M = this->transPos*this->transAxis_M;
    this->rPrime_FM_M = this->transVel*this->transAxis_M;
    this->rPrimePrime_FM_M = this->transAccel*this->transAxis_M;

    eigenVector3d2CArray(this->r_FM_M, prescribedTranslationMsgOut.r_FM_M);
    eigenVector3d2CArray(this->rPrime_FM_M, prescribedTranslationMsgOut.rPrime_FM_M);
    eigenVector3d2CArray(this->rPrimePrime_FM_M, prescribedTranslationMsgOut.rPrimePrime_FM_M);

    // Write the output message
    this->prescribedTranslationOutMsg.write(&prescribedTranslationMsgOut, this->moduleID, callTime);
}

/*! This method determines if the current time is within the first ramp segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedTranslation::isInFirstRampSegment(double t) const {
    return (t <= this->tr && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the coast segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedTranslation::isInCoastSegment(double t) const {
    return (t > this->tr && t <= this->tc && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the second ramp segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedTranslation::isInSecondRampSegment(double t) const {
    return (t > this->tc && t <= this->tf && this->tf - this->tInit != 0);
}

/*! This method computes the required parameters for the translation with a coast period.
 @return void
*/
void PrescribedTranslation::computeCoastParameters() {
    if (this->transPosInit != this->transPosRef) {
        // Determine the time at the end of the first ramp segment
        this->tr = this->tInit + this->coastOptionRampDuration;

        // Determine the position and velocity at the end of the ramp segment/start of the coast segment
        if (this->transPosInit < this->transPosRef) {
            this->transPos_tr = (0.5 * this->transAccelMax * this->coastOptionRampDuration * this->coastOptionRampDuration)
                                + (this->transVelInit * this->coastOptionRampDuration) + this->transPosInit;
            this->transVel_tr = this->transAccelMax * this->coastOptionRampDuration + this->transVelInit;
        } else {
            this->transPos_tr =
                    -((0.5 * this->transAccelMax * this->coastOptionRampDuration * this->coastOptionRampDuration)
                      + (this->transVelInit * this->coastOptionRampDuration)) + this->transPosInit;
            this->transVel_tr = -this->transAccelMax * this->coastOptionRampDuration + this->transVelInit;
        }

        // Determine the distance traveled during the coast period
        double deltaPosCoast = this->transPosRef - this->transPosInit - 2 * (this->transPos_tr - this->transPosInit);

        // Determine the time duration of the coast segment
        double tCoast = fabs(deltaPosCoast) / fabs(this->transVel_tr);

        // Determine the time at the end of the coast segment
        this->tc = this->tr + tCoast;

        // Determine the position at the end of the coast segment
        this->transPos_tc = this->transPos_tr + deltaPosCoast;

        // Determine the time at the end of the translation
        this->tf = this->tc + this->coastOptionRampDuration;

        // Define the parabolic constants for the first and second ramp segments of the translation
        this->a = (this->transPos_tr - this->transPosInit) / ((this->tr - this->tInit) * (this->tr - this->tInit));
        this->b = -(this->transPosRef - this->transPos_tc) / ((this->tc - this->tf) * (this->tc - this->tf));
    } else {
        // If the initial position equals the reference position, no translation is required.
        this->tf = this->tInit;
    }
}

/*! This method computes the scalar translational states for the coast option coast period.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedTranslation::computeCoastSegment(double t) {
    this->transAccel = 0.0;
    this->transVel = this->transVel_tr;
    this->transPos = this->transVel_tr * (t - this->tr) + this->transPos_tr;
}

/*! This method determines if the current time is within the first ramp segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedTranslation::isInFirstRampSegmentNoCoast(double t) const {
    return (t <= this->ts && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the second ramp segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedTranslation::isInSecondRampSegmentNoCoast(double t) const {
    return (t > this->ts && t <= this->tf && this->tf - this->tInit != 0);
}

/*! This method computes the required parameters for the translation with no coast period.
 @return void
*/
void PrescribedTranslation::computeParametersNoCoast() {
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
void PrescribedTranslation::computeFirstRampSegment(double t) {
    if (this->transPosInit < this->transPosRef) {
        this->transAccel = this->transAccelMax;
    } else {
        this->transAccel = -this->transAccelMax;
    }
    this->transVel = this->transAccel * (t - this->tInit) + this->transVelInit;
    this->transPos = this->a * (t - this->tInit) * (t - this->tInit) + this->transPosInit;
}

/*! This method computes the scalar translational states for the second ramp segment. The acceleration during the
 * second ramp segment is negative if the reference position is greater than the initial position. The acceleration
 * is positive during the second ramp segment if the reference position is less than the initial position.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedTranslation::computeSecondRampSegment(double t) {
    if (this->transPosInit < this->transPosRef) {
        this->transAccel = -this->transAccelMax;
    } else {
        this->transAccel = this->transAccelMax;
    }
    this->transVel = this->transAccel * (t - this->tInit) + this->transVelInit
                   - this->transAccel * (this->tf - this->tInit);
    this->transPos = this->b * (t - this->tf) * (t - this->tf) + this->transPosRef;
}

/*! This method computes the scalar translational states when the translation is complete.
 @return void
*/
void PrescribedTranslation::computeTranslationComplete() {
    this->transAccel = 0.0;
    this->transVel = 0.0;
    this->transPos = this->transPosRef;
    this->convergence = true;
}

/*! Setter method for the coast option ramp duration.
 @return void
 @param rampDuration [s] Ramp segment time duration
*/
void PrescribedTranslation::setCoastOptionRampDuration(double rampDuration) {
    this->coastOptionRampDuration = rampDuration;
}

/*! Setter method for the translating body hub-relative position vector.
 @return void
 @param r_FM_M Translating body hub-relative position vector
*/
void PrescribedTranslation::setR_FM_M(const Eigen::Vector3d &r_FM_M) {
    this->r_FM_M = r_FM_M;
}

/*! Setter method for the translating body hub-relative velocity vector.
 @return void
 @param rPrime_FM_M Translating body hub-relative velocity vector
*/
void PrescribedTranslation::setRPrime_FM_M(const Eigen::Vector3d &rPrime_FM_M) {
    this->rPrime_FM_M = rPrime_FM_M;
}

/*! Setter method for the translating body hub-relative acceleration vector.
 @return void
 @param rPrimePrime_FM_M Translating body hub-relative acceleration vector
*/
void PrescribedTranslation::setRPrimePrime_FM_M(const Eigen::Vector3d &rPrimePrime_FM_M) {
    this->rPrimePrime_FM_M = rPrimePrime_FM_M;
}

/*! Setter method for the ramp segment scalar linear acceleration.
 @return void
 @param transAccelMax [m/s^2] Ramp segment linear angular acceleration
*/
void PrescribedTranslation::setTransAccelMax(double transAccelMax) {
    this->transAccelMax = transAccelMax;
}

/*! Setter method for the translating body axis of translation.
 @return void
 @param transAxis_M Translating body axis of translation (unit vector)
*/
void PrescribedTranslation::setTransAxis_M(const Eigen::Vector3d &transAxis_M) {
    this->transAxis_M = transAxis_M;
}

/*! Setter method for the initial translating body hub-relative position.
 @return void
 @param transPosInit [m] Initial translating body position relative to the hub
*/
void PrescribedTranslation::setTransPosInit(double transPosInit) {
    this->transPosInit = transPosInit;
}

/*! Getter method for the coast option ramp duration.
 @return double
*/
double PrescribedTranslation::getCoastOptionRampDuration() const {
    return this->coastOptionRampDuration;
}

/*! Getter method for the translating body's hub-relative position vector.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &PrescribedTranslation::getR_FM_M() const {
    return this->r_FM_M;
}

/*! Getter method for the translating body's hub-relative linear velocity vector.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &PrescribedTranslation::getRPrime_FM_M() const {
    return this->rPrime_FM_M;
}

/*! Getter method for the translating body's hub-relative linear acceleration vector.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &PrescribedTranslation::getRPrimePrime_FM_M() const {
    return this->rPrimePrime_FM_M;
}

/*! Getter method for the ramp segment scalar linear acceleration.
 @return double
*/
double PrescribedTranslation::getTransAccelMax() const {
    return this->transAccelMax;
}

/*! Getter method for the translating body axis of translation.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &PrescribedTranslation::getTransAxis_M() const {
    return this->transAxis_M;
}

/*! Getter method for the initial translating body position.
 @return double
*/
double PrescribedTranslation::getTransPosInit() const {
    return this->transPosInit;
}