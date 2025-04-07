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

*/
void PrescribedLinearTranslation::SelfInit() {
    PrescribedTranslationMsg_C_init(&this->prescribedTranslationOutMsgC);
}

/*! This method resets required module variables and checks the input messages to ensure they are linked.

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

 @param callTime [ns] Time the method is called
*/
void PrescribedLinearTranslation::UpdateState(uint64_t callTime) {
    // Read the input message
    LinearTranslationRigidBodyMsgPayload linearTranslationRigidBodyIn;
    if (this->linearTranslationRigidBodyInMsg.isWritten()) {
        linearTranslationRigidBodyIn = LinearTranslationRigidBodyMsgPayload();
        linearTranslationRigidBodyIn = this->linearTranslationRigidBodyInMsg();
    }

    /* This loop is entered (a) initially and (b) when the translation is complete.
    The parameters used to profile the translation are updated in this statement. */
    if (this->linearTranslationRigidBodyInMsg.timeWritten() <= callTime && this->convergence) {
        // Update the initial time as the current simulation time
        this->tInit = callTime * NANO2SEC;

        // Update the initial hub-relative position
        this->transPosInit = this->transPos;

        // Store the reference position
        this->transPosRef = linearTranslationRigidBodyIn.rho;

        // Set the parameters required to profile the translation
        if (this->transPosRef != this->transPosInit) {
            this->computeTranslationParameters();
        } else {
            this->t_f = this->tInit;
        }

        // Set the convergence to false until the translation is complete
        this->convergence = false;
    }

    // Compute the scalar translational states at the current simulation time
    this->computeCurrentState(callTime * NANO2SEC);

    // Write the module output messages
    this->writeOutputMessages(callTime);
}

/*! This intermediate method groups the calculation of translation parameters into a single method.

*/
void PrescribedLinearTranslation::computeTranslationParameters() {
    if (this->coastOptionBangDuration > 0.0) {
        if (this->smoothingDuration > 0.0) {
            this->computeSmoothedBangCoastBangParameters();
        } else {
            this->computeBangCoastBangParametersNoSmoothing();
        }
    } else {
        if (this->smoothingDuration > 0.0) {
            this->computeSmoothedBangBangParameters();
        } else {
            this->computeBangBangParametersNoSmoothing();
        }
    }
}

/*! This method computes the required parameters for the translation with a non-smoothed bang-bang acceleration profile.

*/
void PrescribedLinearTranslation::computeBangBangParametersNoSmoothing() {
    // Determine the total time required for the translation
    double totalTransTime = sqrt(((0.5 * fabs(this->transPosRef - this->transPosInit)) * 8.0) / this->transAccelMax);

    // Determine the time when the translation is complete t_f
    this->t_f = this->tInit + totalTransTime;

    // Determine the time halfway through the translation
    this->t_b1 = this->tInit + (totalTransTime / 2.0);

    // Define the parabolic constants for the first and second half of the translation
    this->a = 0.5 * (this->transPosRef - this->transPosInit)
              / ((this->t_b1 - this->tInit) * (this->t_b1 - this->tInit));
    this->b = -0.5 * (this->transPosRef - this->transPosInit)
              / ((this->t_b1 - this->t_f) * (this->t_b1 - this->t_f));
}

/*! This method computes the required parameters for the translation with a non-smoothed bang-coast-bang acceleration profile.

*/
void PrescribedLinearTranslation::computeBangCoastBangParametersNoSmoothing() {
    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);

    // Determine the time at the end of the first bang segment t_b1
    this->t_b1 = this->tInit + this->coastOptionBangDuration;

    // Determine the hub-relative position at time t_b1
    this->transPos_tb1 = sign * 0.5 * this->transAccelMax * this->coastOptionBangDuration
                         * this->coastOptionBangDuration + this->transPosInit;
    this->transVel_tb1 = sign * this->transAccelMax * this->coastOptionBangDuration;

    // Determine the distance traveled during the coast period
    double deltaPosCoast = this->transPosRef - this->transPosInit - 2.0 * (this->transPos_tb1 - this->transPosInit);

    // Determine the duration of the coast segment coastDuration
    double coastDuration = fabs(deltaPosCoast / this->transVel_tb1);

    // Determine the time at the end of the coast segment t_c
    this->t_c = this->t_b1 + coastDuration;

    // Determine the hub-relative position at time t_c
    double transPos_tc = this->transPos_tb1 + deltaPosCoast;

    // Determine the time when the translation is complete t_f
    this->t_f = this->t_c + this->coastOptionBangDuration;

    // Define the parabolic constants for the first and second bang segments of the translation
    this->a = (this->transPos_tb1 - this->transPosInit) / ((this->t_b1 - this->tInit) * (this->t_b1 - this->tInit));
    this->b = -(this->transPosRef - transPos_tc) / ((this->t_c - this->t_f) * (this->t_c - this->t_f));
}

/*! This method computes the required parameters for the translation with a smoothed bang-bang acceleration profile.

*/
void PrescribedLinearTranslation::computeSmoothedBangBangParameters() {
    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);

    // Determine the time at the end of the first smoothing segment t_s1
    this->t_s1 = this->tInit + this->smoothingDuration;

    // Determine the hub-relative position and velocity at time t_s1
    this->transVel_ts1 = sign * 0.5 * this->transAccelMax * this->smoothingDuration;
    this->transPos_ts1 = sign * (3.0 / 20.0) * this->transAccelMax * this->smoothingDuration * this->smoothingDuration
                         + this->transPosInit;

    // Determine the duration of the bang segment bangDuration
    double aTerm = sign * 0.5 * this->transAccelMax;
    double bTerm = (sign * this->transAccelMax * this->smoothingDuration + this->transVel_ts1) / aTerm;
    double cTerm = (sign * (2.0 / 5.0) * this->transAccelMax * this->smoothingDuration * this->smoothingDuration
                   + this->transVel_ts1 * this->smoothingDuration + this->transPos_ts1
                   - 0.5 * (this->transPosRef + this->transPosInit)) / aTerm;
    double bangDuration = (- bTerm + sqrt(bTerm * bTerm - 4.0 * cTerm)) / 2.0;

    // Determine the time at the end of the first bang segment t_b1
    this->t_b1 = this->t_s1 + bangDuration;

    // Determine the hub-relative position and velocity at time t_b1
    this->transVel_tb1 = sign * this->transAccelMax * bangDuration + this->transVel_ts1;
    this->transPos_tb1 = sign * 0.5 * this->transAccelMax * bangDuration * bangDuration
                         + this->transVel_ts1 * bangDuration + this->transPos_ts1;

    // Determine the time at the end of the second smoothing segment t_s2
    this->t_s2 = this->t_b1 + 2.0 * this->smoothingDuration;

    // Determine the hub-relative position and velocity at time t_s2
    this->transVel_ts2 = this->transVel_tb1;
    this->transPos_ts2 = sign * (4.0 / 5.0) * this->transAccelMax * this->smoothingDuration * this->smoothingDuration
                         + this->transVel_tb1 * 2.0 * this->smoothingDuration + this->transPos_tb1;

    // Determine the time at the end of the second bang segment t_b2
    this->t_b2 = this->t_s2 + bangDuration;

    // Determine the hub-relative position and velocity at time t_b2
    this->transVel_tb2 = - sign * this->transAccelMax * bangDuration + this->transVel_ts2;
    this->transPos_tb2 = - sign * 0.5 * this->transAccelMax * bangDuration * bangDuration
                         + this->transVel_ts2 * bangDuration + this->transPos_ts2;

    // Determine the time when the translation is complete t_f
    this->t_f = this->t_b2 + this->smoothingDuration;
}

/*! This method computes the required parameters for the translation with a smoothed bang-coast-bang acceleration profile.

*/
void PrescribedLinearTranslation::computeSmoothedBangCoastBangParameters() {
    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);

    // Determine the time at the end of the first smoothing segment t_s1
    this->t_s1 = this->tInit + this->smoothingDuration;

    // Determine the hub-relative position and velocity at time t_s1
    this->transVel_ts1 = sign * 0.5 * this->transAccelMax * this->smoothingDuration;
    this->transPos_ts1 = sign * (3.0 / 20.0) * this->transAccelMax * this->smoothingDuration * this->smoothingDuration
                         + this->transPosInit;

    // Determine the time at the end of the first bang segment t_b1
    this->t_b1 = this->t_s1 + this->coastOptionBangDuration;

    // Determine the hub-relative position and velocity at time t_b1
    this->transVel_tb1 = sign * this->transAccelMax * this->coastOptionBangDuration + this->transVel_ts1;
    this->transPos_tb1 = sign * 0.5 * this->transAccelMax * this->coastOptionBangDuration
                         * this->coastOptionBangDuration + this->transVel_ts1 * this->coastOptionBangDuration
                         + this->transPos_ts1;

    // Determine the time at the end of the second smoothing segment t_s2
    this->t_s2 = this->t_b1 + this->smoothingDuration;

    // Determine the hub-relative position and velocity at time t_s2
    this->transVel_ts2 = sign * 0.5 * this->transAccelMax * this->smoothingDuration + this->transVel_tb1;
    this->transPos_ts2 = sign * (7.0 / 20.0) * this->transAccelMax * this->smoothingDuration * this->smoothingDuration
                         + this->transVel_tb1 * this->smoothingDuration + this->transPos_tb1;

    // Compute the time at the end of the coast segment t_c
    double deltaPosCoast = (this->transPosRef - this->transPosInit) - 2 * (this->transPos_ts2 - this->transPosInit);
    this->t_c = (deltaPosCoast / this->transVel_ts2) + this->t_s2;

    // Determine the hub-relative position and velocity at time t_c
    this->transVel_tc = this->transVel_ts2;
    this->transPos_tc = this->transVel_ts2 * (this->t_c - this->t_s2) + this->transPos_ts2;

    // Determine the time at the end of the third smoothing segment t_s3
    this->t_s3 = this->t_c + this->smoothingDuration;

    // Determine the hub-relative position and velocity at time t_s3
    this->transVel_ts3 = - sign * 0.5 * this->transAccelMax * this->smoothingDuration + this->transVel_tc;
    this->transPos_ts3 = - sign * (3.0 / 20.0) * this->transAccelMax * this->smoothingDuration * this->smoothingDuration
                         + this->transVel_tc * this->smoothingDuration + this->transPos_tc;

    // Determine the time at the end of the second bang segment t_b2
    this->t_b2 = this->t_s3 + this->coastOptionBangDuration;

    // Determine the hub-relative position and velocity at time t_b2
    this->transVel_tb2 = - sign * this->transAccelMax * this->coastOptionBangDuration + this->transVel_ts3;
    this->transPos_tb2 = - sign * 0.5 * this->transAccelMax * this->coastOptionBangDuration
                         * this->coastOptionBangDuration + this->transVel_ts3 * this->coastOptionBangDuration
                         + this->transPos_ts3;

    // Determine the time when the translation is complete t_f
    this->t_f = this->t_b2 + this->smoothingDuration;
}

/*! This intermediate method groups the calculation of the current translational states into a single method.

*/
void PrescribedLinearTranslation::computeCurrentState(double t) {
    if (this->coastOptionBangDuration > 0.0) {
        if(this->smoothingDuration > 0.0) {
            if (this->isInFirstSmoothedSegment(t)) {
                this->computeFirstSmoothedSegment(t);
            } else if (this->isInFirstBangSegment(t)) {
                this->computeFirstBangSegment(t);
            } else if (this->isInSecondSmoothedSegment(t)) {
                this->computeSecondSmoothedSegment(t);
            } else if (this->isInCoastSegment(t)) {
                this->computeCoastSegment(t);
            } else if (this->isInThirdSmoothedSegment(t)) {
                this->computeThirdSmoothedSegment(t);
            } else if (this->isInSecondBangSegment(t)) {
                this->computeSecondBangSegment(t);
            } else if (this->isInFourthSmoothedSegment(t)) {
                this->computeFourthSmoothedSegment(t);
            } else {
                this->computeTranslationComplete();
            }
        } else {
            if (this->isInFirstBangSegment(t)) {
                this->computeFirstBangSegment(t);
            } else if (this->isInCoastSegment(t)) {
                this->computeCoastSegment(t);
            } else if (this->isInSecondBangSegment(t)) {
                this->computeSecondBangSegment(t);
            } else {
                this->computeTranslationComplete();
            }
        }
    } else {
        if (this->smoothingDuration > 0.0) {
            if (this->isInFirstSmoothedSegment(t)) {
                this->computeFirstSmoothedSegment(t);
            } else if (this->isInFirstBangSegment(t)) {
                this->computeFirstBangSegment(t);
            } else if (this->isInSecondSmoothedSegment(t)) {
                this->computeSecondSmoothedSegment(t);
            } else if (this->isInSecondBangSegment(t)) {
                this->computeSecondBangSegment(t);
            } else if (this->isInThirdSmoothedSegment(t)) {
                this->computeThirdSmoothedSegment(t);
            } else {
                this->computeTranslationComplete();
            }
        } else {
            if (this->isInFirstBangSegment(t)) {
                this->computeFirstBangSegment(t);
            } else if (this->isInSecondBangSegment(t)) {
                this->computeSecondBangSegment(t);
            } else {
                this->computeTranslationComplete();
            }
        }
    }
}

/*! This method determines if the current time is within the first bang segment.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInFirstBangSegment(double t) const {
    if (this->smoothingDuration > 0.0) {
        return (t > this->t_s1 && t <= this->t_b1 && this->t_f - this->tInit != 0.0);
    } else {
        return (t <= this->t_b1 && this->t_f - this->tInit != 0.0);
    }
}

/*! This method determines if the current time is within the second bang segment.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInSecondBangSegment(double t) const {
    if (this->coastOptionBangDuration > 0.0) {
        if (this->smoothingDuration > 0.0) {
            return (t > this->t_s3 && t <= this->t_b2 && this->t_f - this->tInit != 0.0);
        } else {
            return (t > this->t_c && t <= this->t_f && this->t_f - this->tInit != 0.0);
        }
    } else {
        if (this->smoothingDuration > 0.0) {
            return (t > this->t_s2 && t <= this->t_b2 && this->t_f - this->tInit != 0.0);
        } else {
            return (t > this->t_b1 && t <= this->t_f && this->t_f - this->tInit != 0.0);
        }
    }
}

/*! This method determines if the current time is within the first smoothing segment for the smoothed profiler options.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInFirstSmoothedSegment(double t) const {
    return (t <= this->t_s1 && this->t_f - this->tInit != 0.0);
}

/*! This method determines if the current time is within the second smoothing segment for the smoothed profiler options..
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInSecondSmoothedSegment(double t) const {
    return (t > this->t_b1 && t <= this->t_s2 && this->t_f - this->tInit != 0.0);
}

/*! This method determines if the current time is within the third smoothing segment for the smoothed profiler options.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInThirdSmoothedSegment(double t) const {
    if (this->coastOptionBangDuration > 0.0) {
        return (t > this->t_c && t <= this->t_s3 && this->t_f - this->tInit != 0.0);
    } else {
        return (t > this->t_b2 && t <= this->t_f && this->t_f - this->tInit != 0.0);
    }
}

/*! This method determines if the current time is within the fourth smoothing segment for the smoothed bang-coast-bang option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInFourthSmoothedSegment(double t) const {
    return (t > this->t_b2 && t <= this->t_f && this->t_f - this->tInit != 0.0);
}

/*! This method determines if the current time is within the coast segment.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedLinearTranslation::isInCoastSegment(double t) const {
    if (this->smoothingDuration > 0.0) {
        return (t > this->t_s2 && t <= this->t_c && this->t_f - this->tInit != 0.0);
    } else{
        return (t > this->t_b1 && t <= this->t_c && this->t_f - this->tInit != 0.0);
    }
}

/*! This method computes the first bang segment scalar translational states.

 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeFirstBangSegment(double t) {
    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);
    this->transAccel = sign * this->transAccelMax;

    if (this->smoothingDuration > 0.0) {
        this->transVel = this->transAccel * (t - this->t_s1) + this->transVel_ts1;
        this->transPos = 0.5 * this->transAccel * (t - this->t_s1) * (t - this->t_s1)
                         + this->transVel_ts1 * (t - this->t_s1) + this->transPos_ts1;
    } else {
        this->transVel = this->transAccel * (t - this->tInit);
        this->transPos = this->a * (t - this->tInit) * (t - this->tInit) + this->transPosInit;
    }
}

/*! This method computes the second bang segment scalar translational states.

 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeSecondBangSegment(double t) {
    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);
    this->transAccel = - sign * this->transAccelMax;

    if (this->smoothingDuration > 0.0) {
        if (this->coastOptionBangDuration > 0.0) {
            this->transVel = this->transAccel * (t - this->t_s3) + this->transVel_ts3;
            this->transPos = 0.5 * this->transAccel * (t - this->t_s3) * (t - this->t_s3)
                             + this->transVel_ts3 * (t - this->t_s3) + this->transPos_ts3;
        } else {
            this->transVel = this->transAccel * (t - this->t_s2) + this->transVel_ts2;
            this->transPos = 0.5 * this->transAccel * (t - this->t_s2) * (t - this->t_s2)
                             + this->transVel_ts2 * (t - this->t_s2) + this->transPos_ts2;
        }
    } else {
        this->transVel = this->transAccel * (t - this->t_f);
        this->transPos = this->b * (t - this->t_f) * (t - this->t_f) + this->transPosRef;
    }
}

/*! This method computes the first smoothing segment scalar translational states for the smoothed profiler options.

 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeFirstSmoothedSegment(double t) {
    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);

    double term1 = (3.0 * (t - this->tInit) * (t - this->tInit)) / (this->smoothingDuration * this->smoothingDuration);
    double term2 = (2.0 * (t - this->tInit) * (t - this->tInit) * (t - this->tInit))
                   / (this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
    double term3 = ((t - this->tInit) * (t - this->tInit) * (t - this->tInit))
                   / (this->smoothingDuration * this->smoothingDuration);
    double term4 = ((t - this->tInit) * (t - this->tInit) * (t - this->tInit) * (t - this->tInit))
                   / (2.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
    double term5 = ((t - this->tInit) * (t - this->tInit) * (t - this->tInit) * (t - this->tInit))
                   / (4.0 * this->smoothingDuration * this->smoothingDuration);
    double term6 = ((t - this->tInit) * (t - this->tInit) * (t - this->tInit) * (t - this->tInit) * (t - this->tInit))
                   / (10.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);

    this->transAccel = sign * this->transAccelMax * (term1 - term2);
    this->transVel = sign * this->transAccelMax * (term3 - term4);
    this->transPos = sign * this->transAccelMax * (term5 - term6) + this->transPosInit;
}

/*! This method computes the second smoothing segment scalar translational states for the smoothed profiler options.

 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeSecondSmoothedSegment(double t) {
    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);

    double term1;
    double term2;
    double term3;
    double term4;
    double term5;
    double term6;
    double term7;

    if (this->coastOptionBangDuration > 0.0) {
        term1 = (3.0 * (t - this->t_b1) * (t - this->t_b1)) / (this->smoothingDuration * this->smoothingDuration);
        term2 = (2.0 * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
        term3 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (this->smoothingDuration * this->smoothingDuration);
        term4 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (2.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
        term5 = 0.5 * (t - this->t_b1) * (t - this->t_b1);
        term6 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (4.0 * this->smoothingDuration * this->smoothingDuration);
        term7 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (10.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
    } else {
        term1 = (3.0 * (t - this->t_b1) * (t - this->t_b1)) / (2.0 * this->smoothingDuration * this->smoothingDuration);
        term2 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (2.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
        term3 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (2.0 * this->smoothingDuration * this->smoothingDuration);
        term4 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (8.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
        term5 = 0.5 * (t - this->t_b1) * (t - this->t_b1);
        term6 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (8.0 * this->smoothingDuration * this->smoothingDuration);
        term7 = ((t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1) * (t - this->t_b1))
                / (40.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
    }

    this->transAccel = sign * this->transAccelMax * (1.0 - term1 + term2);
    this->transVel = sign * this->transAccelMax * ((t - this->t_b1) - term3 + term4) + this->transVel_tb1;
    this->transPos = sign * this->transAccelMax * (term5 - term6 + term7)
                     + this->transVel_tb1 * (t - this->t_b1) + this->transPos_tb1;
}

/*! This method computes the third smoothing segment scalar translational states for the smoothed profiler options.

 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeThirdSmoothedSegment(double t) {
    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);

    double term1;
    double term2;
    double term3;
    double term4;
    double term5;
    double term6;
    double term7;

    if (this->coastOptionBangDuration > 0.0) {
        term1 = (3.0 * (t - this->t_c) * (t - this->t_c)) / (this->smoothingDuration * this->smoothingDuration);
        term2 = (2.0 * (t - this->t_c) * (t - this->t_c) * (t - this->t_c))
                / (this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
        term3 = ((t - this->t_c) * (t - this->t_c) * (t - this->t_c))
                / (this->smoothingDuration * this->smoothingDuration);
        term4 = ((t - this->t_c) * (t - this->t_c) * (t - this->t_c) * (t - this->t_c))
                / (2.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
        term5 = ((t - this->t_c) * (t - this->t_c) * (t - this->t_c) * (t - this->t_c))
                / (4.0 * this->smoothingDuration * this->smoothingDuration);
        term6 = ((t - this->t_c) * (t - this->t_c) * (t - this->t_c) * (t - this->t_c) * (t - this->t_c))
                / (10.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);

        this->transAccel = - sign * this->transAccelMax * (term1 - term2);
        this->transVel = - sign * this->transAccelMax * (term3 - term4) + this->transVel_tc;
        this->transPos = - sign * this->transAccelMax * (term5 - term6) + this->transVel_tc * (t - this->t_c) + this->transPos_tc;
    } else {
        term1 = (3.0 * (t - this->t_b2) * (t - this->t_b2)) / (this->smoothingDuration * this->smoothingDuration);
        term2 = (2.0 * (t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2))
                / (this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
        term3 = ((t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2))
                / (this->smoothingDuration * this->smoothingDuration);
        term4 = ((t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2))
                / (2.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
        term5 = - 0.5 * (t - this->t_b2) * (t - this->t_b2);
        term6 = ((t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2))
                / (4.0 * this->smoothingDuration * this->smoothingDuration);
        term7 = ((t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2) * (t - this->t_b2))
                / (10.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);

        this->transAccel = sign * this->transAccelMax * ( - 1.0 + term1 - term2);
        this->transVel = sign * this->transAccelMax * ( - (t - this->t_b2) + term3 - term4) + this->transVel_tb2;
        this->transPos = sign * this->transAccelMax * (term5 + term6 - term7) + this->transVel_tb2 * (t - this->t_b2)
                         + this->transPos_tb2;
    }
}

/*! This method computes the fourth smoothing segment scalar translational states for the smoothed bang-coast-bang option.

 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeFourthSmoothedSegment(double t) {
    double term1 = (3.0 * (this->t_f - t) * (this->t_f - t)) / (this->smoothingDuration * this->smoothingDuration);
    double term2 = (2.0 * (this->t_f - t) * (this->t_f - t) * (this->t_f - t))
                   / (this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
    double term3 = ((this->t_f - t) * (this->t_f - t) * (this->t_f - t))
                   / (this->smoothingDuration * this->smoothingDuration);
    double term4 = ((this->t_f - t) * (this->t_f - t) * (this->t_f - t) * (this->t_f - t))
                   / (2.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);
    double term5 = ((this->t_f - t) * (this->t_f - t) * (this->t_f - t) * (this->t_f - t))
                   / (4.0 * this->smoothingDuration * this->smoothingDuration);
    double term6 = ((this->t_f - t) * (this->t_f - t) * (this->t_f - t) * (this->t_f - t) * (this->t_f - t))
                   / (10.0 * this->smoothingDuration * this->smoothingDuration * this->smoothingDuration);

    double sign = (this->transPosRef - this->transPosInit) / abs(this->transPosRef - this->transPosInit);

    this->transAccel = - sign * this->transAccelMax * (term1 - term2);
    this->transVel = sign * this->transAccelMax * (term3 - term4);
    this->transPos = - sign * this->transAccelMax * (term5 - term6) + this->transPosRef;
}

/*! This method computes the coast segment scalar translational states

 @param t [s] Current simulation time
*/
void PrescribedLinearTranslation::computeCoastSegment(double t) {
    this->transAccel = 0.0;

    if (this->smoothingDuration > 0.0) {
        this->transVel = this->transVel_ts2;
        this->transPos = this->transVel_ts2 * (t - this->t_s2) + this->transPos_ts2;
    } else {
        this->transVel = this->transVel_tb1;
        this->transPos = this->transVel_tb1 * (t - this->t_b1) + this->transPos_tb1;
    }
}

/*! This method computes the scalar translational states when the translation is complete.

*/
void PrescribedLinearTranslation::computeTranslationComplete() {
    this->transAccel = 0.0;
    this->transVel = 0.0;
    this->transPos = this->transPosRef;
    this->convergence = true;
}

/*! This method writes the module output messages and computes the output message data.

*/
void PrescribedLinearTranslation::writeOutputMessages(uint64_t callTime) {
    // Create the output buffer message
    PrescribedTranslationMsgPayload prescribedTranslationMsgOut;
    LinearTranslationRigidBodyMsgPayload linearTranslationRigidBodyMsgOut;

    // Zero the output messages
    prescribedTranslationMsgOut = PrescribedTranslationMsgPayload();
    linearTranslationRigidBodyMsgOut = LinearTranslationRigidBodyMsgPayload();

    // Compute the translational body position relative to the mount frame M expressed in M frame components
    Eigen::Vector3d r_PM_M = this->transPos * this->transHat_M;  // [m]

    // Compute the translational body velocity relative to the mount frame M expressed in M frame components
    Eigen::Vector3d rPrime_PM_M = this->transVel * this->transHat_M;  // [m/s]

    // Compute the translational body acceleration relative to the mount frame M expressed in M frame components
    Eigen::Vector3d rPrimePrime_PM_M = this->transAccel * this->transHat_M;  // [m/s^2]

    // Copy the module variables to the output buffer message
    eigenVector3d2CArray(r_PM_M, prescribedTranslationMsgOut.r_PM_M);
    eigenVector3d2CArray(rPrime_PM_M, prescribedTranslationMsgOut.rPrime_PM_M);
    eigenVector3d2CArray(rPrimePrime_PM_M, prescribedTranslationMsgOut.rPrimePrime_PM_M);
    linearTranslationRigidBodyMsgOut.rho = this->transPos;
    linearTranslationRigidBodyMsgOut.rhoDot = this->transVel;

    // Write the output messages
    this->prescribedTranslationOutMsg.write(&prescribedTranslationMsgOut, this->moduleID, callTime);
    PrescribedTranslationMsg_C_write(&prescribedTranslationMsgOut,
                                     &prescribedTranslationOutMsgC, this->moduleID, callTime);
    this->linearTranslationRigidBodyOutMsg.write(&linearTranslationRigidBodyMsgOut, this->moduleID, callTime);
}

/*! Setter method for the coast option bang duration.

 @param coastOptionBangDuration [s] Bang segment time duration
*/
void PrescribedLinearTranslation::setCoastOptionBangDuration(const double coastOptionBangDuration) {
    this->coastOptionBangDuration = coastOptionBangDuration;
}

/*! Setter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value.

 @param smoothingDuration [s] Duration the acceleration is smoothed until reaching the given maximum acceleration value
*/
void PrescribedLinearTranslation::setSmoothingDuration(const double smoothingDuration) {
    this->smoothingDuration = smoothingDuration;
}

/*! Setter method for the bang segment scalar linear acceleration.

 @param transAccelMax [m/s^2] Bang segment linear angular acceleration
*/
void PrescribedLinearTranslation::setTransAccelMax(const double transAccelMax) {
    this->transAccelMax = transAccelMax;
}

/*! Setter method for the translating body axis of translation.

 @param transHat_M Translating body axis of translation (unit vector)
*/
void PrescribedLinearTranslation::setTransHat_M(const Eigen::Vector3d &transHat_M) {
    this->transHat_M = transHat_M;
}

/*! Setter method for the initial translating body hub-relative position.

 @param transPosInit [m] Initial translating body position relative to the hub
*/
void PrescribedLinearTranslation::setTransPosInit(const double transPosInit) {
    this->transPosInit = transPosInit;
}

/*! Getter method for the coast option bang duration.
 @return double
*/
double PrescribedLinearTranslation::getCoastOptionBangDuration() const {
    return this->coastOptionBangDuration;
}

/*! Getter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value.
 @return double
*/
double PrescribedLinearTranslation::getSmoothingDuration() const {
    return this->smoothingDuration;
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
