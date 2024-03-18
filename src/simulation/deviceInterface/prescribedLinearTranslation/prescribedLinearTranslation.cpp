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

/*! This method self initializes the C-wrapped output message.
 @return void
*/
void PrescribedLinearTranslation::SelfInit() {
    PrescribedTranslationMsg_C_init(&this->prescribedTranslationOutMsgC);
}

/*! This method resets required module variables and checks the input messages to ensure they are linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedLinearTranslation::Reset(uint64_t callTime) {
    if (!this->linearTranslationRigidBodyInMsg.isLinked()) {
        _bskLog(this->bskLogger,
                BSK_ERROR,
                "prescribedLinearTranslation.linearTranslationRigidBodyInMsg wasn't connected.");
    }

    this->tInit = 0.0;
    this->transPos = this->transPosInit;
    this->transVel = 0.0;

    // Set the initial convergence to true to enter the required loop in Update() method on the first pass
    this->convergence = true;
}

/*! This method profiles the translation and updates the prescribed translational states as a function of time.
The prescribed translational states are then written to the output message.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedLinearTranslation::UpdateState(uint64_t callTime) {
    // Read the input message
    LinearTranslationRigidBodyMsgPayload linearTranslationRigidBodyIn;
    if (this->linearTranslationRigidBodyInMsg.isWritten()) {
        linearTranslationRigidBodyIn = LinearTranslationRigidBodyMsgPayload();
        linearTranslationRigidBodyIn = this->linearTranslationRigidBodyInMsg();
    }

    /* This loop is entered (a) initially and (b) when each rotation is complete.
    The parameters used to profile the translation are updated in this statement. */
    if (this->linearTranslationRigidBodyInMsg.timeWritten() <= callTime && this->convergence) {
        // Update the initial time as the current simulation time
        this->tInit = callTime * NANO2SEC;

        // Update the initial hub-relative position
        this->transPosInit = this->transPos;

        // Store the reference position
        this->transPosRef = linearTranslationRigidBodyIn.rho;

        // Set the parameters required to profile the translation
        if (this->coastOptionBangDuration > 0.0) {
            this->computeBangCoastBangParametersNoSmoothing();
        } else {
            this->computeBangBangParametersNoSmoothing();
        }

        // Set the convergence to false until the translation is complete
        this->convergence = false;
    }

    // Compute the scalar translational states at the current simulation time
    this->computeCurrentState(callTime * NANO2SEC);

    // Write the module output messages
    this->writeOutputMessages(callTime);
}

/*! This method computes the required parameters for the translation with no coast period.
 @return void
*/
void PrescribedLinearTranslation::computeBangBangParametersNoSmoothing() {
    // Determine the total time required for the translation
    double totalTransTime = sqrt(((0.5 * fabs(this->transPosRef - this->transPosInit)) * 8) / this->transAccelMax);

    // Determine the time at the end of the translation
    this->t_f = this->tInit + totalTransTime;

    // Determine the time halfway through the translation
    this->t_b1 = this->tInit + (totalTransTime / 2);

    // Define the parabolic constants for the first and second half of the translation
    this->a = 0.5 * (this->transPosRef - this->transPosInit) / ((this->t_b1 - this->tInit) * (this->t_b1 - this->tInit));
    this->b = -0.5 * (this->transPosRef - this->transPosInit) / ((this->t_b1 - this->t_f) * (this->t_b1 - this->t_f));
}

/*! This method computes the required parameters for the translation with a coast period.
 @return void
*/
void PrescribedLinearTranslation::computeBangCoastBangParametersNoSmoothing() {
    if (this->transPosInit != this->transPosRef) {
        // Determine the time at the end of the first bang segment
        this->t_b1 = this->tInit + this->coastOptionBangDuration;

        // Determine the position and velocity at the end of the bang segment/start of the coast segment
        if (this->transPosInit < this->transPosRef) {
            this->transPos_tr = (0.5 * this->transAccelMax * this->coastOptionBangDuration * this->coastOptionBangDuration)
                                 + this->transPosInit;
            this->transVel_tr = this->transAccelMax * this->coastOptionBangDuration;
        } else {
            this->transPos_tr =
                    -((0.5 * this->transAccelMax * this->coastOptionBangDuration * this->coastOptionBangDuration))
                    + this->transPosInit;
            this->transVel_tr = -this->transAccelMax * this->coastOptionBangDuration;
        }

        // Determine the distance traveled during the coast period
        double deltaPosCoast = this->transPosRef - this->transPosInit - 2 * (this->transPos_tr - this->transPosInit);

        // Determine the time duration of the coast segment
        double tCoast = fabs(deltaPosCoast) / fabs(this->transVel_tr);

        // Determine the time at the end of the coast segment
        this->t_c = this->t_b1 + tCoast;

        // Determine the position [m] at the end of the coast segment
        double transPos_tc = this->transPos_tr + deltaPosCoast;

        // Determine the time at the end of the translation
        this->t_f = this->t_c + this->coastOptionBangDuration;

        // Define the parabolic constants for the first and second bang segments of the translation
        this->a = (this->transPos_tr - this->transPosInit) / ((this->t_b1 - this->tInit) * (this->t_b1 - this->tInit));
        this->b = -(this->transPosRef - transPos_tc) / ((this->t_c - this->t_f) * (this->t_c - this->t_f));
    } else {
        // If the initial position equals the reference position, no translation is required.
        this->t_f = this->tInit;
    }
}

/*! This intermediate method groups the calculation of the current translational states into a single method.
 @return void
*/
void PrescribedLinearTranslation::computeCurrentState(double t) {
    if (this->coastOptionBangDuration > 0.0) {
        if (this->isInFirstBangSegment(t)) {
            this->computeFirstBangSegment(t);
        } else if (this->isInCoastSegment(t)) {
            this->computeCoastSegment(t);
        } else if (this->isInSecondBangSegment(t)) {
            this->computeSecondBangSegment(t);
        } else {
            this->computeTranslationComplete();
        }
    } else {
        if (this->isInFirstBangSegmentNoCoast(t)) {
            this->computeFirstBangSegment(t);
        } else if (this->isInSecondBangSegmentNoCoast(t)) {
            this->computeSecondBangSegment(t);
        } else {
            this->computeTranslationComplete();
        }
    }
}

/*! This method determines if the current time is within the first bang segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInFirstBangSegmentNoCoast(double t) const {
    return (t <= this->t_b1 && this->t_f - this->tInit != 0);
}

/*! This method determines if the current time is within the first bang segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInFirstBangSegment(double t) const {
    return (t <= this->t_b1 && this->t_f - this->tInit != 0);
}

/*! This method determines if the current time is within the second bang segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInSecondBangSegmentNoCoast(double t) const {
    return (t > this->t_b1 && t <= this->t_f && this->t_f - this->tInit != 0);
}

/*! This method determines if the current time is within the second bang segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInSecondBangSegment(double t) const {
    return (t > this->t_c && t <= this->t_f && this->t_f - this->tInit != 0);
}

/*! This method determines if the current time is within the coast segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInCoastSegment(double t) const {
    return (t > this->t_b1 && t <= this->t_c && this->t_f - this->tInit != 0);
}

/*! This method computes the scalar translational states for the first bang segment. The acceleration during the first
 * bang segment is positive if the reference position is greater than the initial position. The acceleration is
 * negative during the first bang segment if the reference position is less than the initial position.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeFirstBangSegment(double t) {
    if (this->transPosInit < this->transPosRef) {
        this->transAccel = this->transAccelMax;
    } else {
        this->transAccel = -this->transAccelMax;
    }
    this->transVel = this->transAccel * (t - this->tInit);
    this->transPos = this->a * (t - this->tInit) * (t - this->tInit) + this->transPosInit;
}

/*! This method computes the scalar translational states for the second bang segment. The acceleration during the
 * second bang segment is negative if the reference position is greater than the initial position. The acceleration
 * is positive during the second bang segment if the reference position is less than the initial position.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeSecondBangSegment(double t) {
    if (this->transPosInit < this->transPosRef) {
        this->transAccel = -this->transAccelMax;
    } else {
        this->transAccel = this->transAccelMax;
    }
    this->transVel = this->transAccel * (t - this->tInit) - this->transAccel * (this->t_f - this->tInit);
    this->transPos = this->b * (t - this->t_f) * (t - this->t_f) + this->transPosRef;
}

/*! This method computes the scalar translational states for the coast option coast period.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeCoastSegment(double t) {
    this->transAccel = 0.0;
    this->transVel = this->transVel_tr;
    this->transPos = this->transVel_tr * (t - this->t_b1) + this->transPos_tr;
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

/*! This method writes the module output messages and computes the output message data.
 @return void
*/
void PrescribedLinearTranslation::writeOutputMessages(uint64_t callTime) {
    // Create the output buffer message
    PrescribedTranslationMsgPayload prescribedTranslationMsgOut;

    // Zero the output messages
    prescribedTranslationMsgOut = PrescribedTranslationMsgPayload();

    // Compute the translational body position relative to the mount frame M expressed in M frame components
    Eigen::Vector3d r_FM_M = this->transPos * this->transHat_M;  // [m]

    // Compute the translational body velocity relative to the mount frame M expressed in M frame components
    Eigen::Vector3d rPrime_FM_M = this->transVel * this->transHat_M;  // [m/s]

    // Compute the translational body acceleration relative to the mount frame M expressed in M frame components
    Eigen::Vector3d rPrimePrime_FM_M = this->transAccel * this->transHat_M;  // [m/s^2]

    // Copy the module variables to the output buffer message
    eigenVector3d2CArray(r_FM_M, prescribedTranslationMsgOut.r_FM_M);
    eigenVector3d2CArray(rPrime_FM_M, prescribedTranslationMsgOut.rPrime_FM_M);
    eigenVector3d2CArray(rPrimePrime_FM_M, prescribedTranslationMsgOut.rPrimePrime_FM_M);

    // Write the output messages
    this->prescribedTranslationOutMsg.write(&prescribedTranslationMsgOut, this->moduleID, callTime);
    PrescribedTranslationMsg_C_write(&prescribedTranslationMsgOut,
                                     &prescribedTranslationOutMsgC, this->moduleID, callTime);
}

/*! Setter method for the coast option bang duration.
 @return void
 @param bangDuration [s] Bang segment time duration
*/
void PrescribedLinearTranslation::setCoastOptionBangDuration(double bangDuration) {
    this->coastOptionBangDuration = bangDuration;
}

/*! Setter method for the bang segment scalar linear acceleration.
 @return void
 @param transAccelMax [m/s^2] Bang segment linear angular acceleration
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

/*! Getter method for the coast option bang duration.
 @return double
*/
double PrescribedLinearTranslation::getCoastOptionBangDuration() const {
    return this->coastOptionBangDuration;
}

/*! Getter method for the bang segment scalar linear acceleration.
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
