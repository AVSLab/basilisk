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

#include "prescribedRotation1DOF.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <cmath>

/*! This method self initializes the C-wrapped output messages.

*/
void PrescribedRotation1DOF::SelfInit() {
    HingedRigidBodyMsg_C_init(&this->spinningBodyOutMsgC);
    PrescribedRotationMsg_C_init(&this->prescribedRotationOutMsgC);
}

/*! This method resets required module variables and checks the input messages to ensure they are linked.

 @param callTime [ns] Time the method is called
*/
void PrescribedRotation1DOF::Reset(uint64_t callTime) {
    if (!this->spinningBodyInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "prescribedRotation1DOF.spinningBodyInMsg wasn't connected.");
    }

    this->tInit = 0.0;
    this->theta = this->thetaInit;
    this->thetaDot = 0.0;

    // Set the initial convergence to true to enter the required loop in Update() method on the first pass
    this->convergence = true;
}

/*! This method profiles the spinning body rotation and updates the prescribed rotational states as a function of time.
 The spinning body rotational states are then written to the output message.

 @param callTime [ns] Time the method is called
*/
void PrescribedRotation1DOF::UpdateState(uint64_t callTime) {
    // Read the input message
    HingedRigidBodyMsgPayload spinningBodyIn = HingedRigidBodyMsgPayload();
    if (this->spinningBodyInMsg.isWritten()) {
        spinningBodyIn = this->spinningBodyInMsg();
    }

    /* This loop is entered (a) initially and (b) when each rotation is complete. The parameters used to profile the
    spinning body rotation are updated in this statement. */
    if (this->spinningBodyInMsg.timeWritten() <= callTime && this->convergence) {
        // Update the initial time as the current simulation time
        this->tInit = callTime * NANO2SEC;

        // Update the initial angle
        this->thetaInit = this->theta;

        // Store the reference angle
        this->thetaRef = spinningBodyIn.theta;

        // Set the parameters required to profile the rotation
        if (this->thetaInit != this->thetaRef) {
            this->computeRotationParameters();
        } else {
            this->t_f = this->tInit;
        }

        // Set the convergence to false until the rotation is complete
        this->convergence = false;
    }

    // Compute the scalar rotational states at the current simulation time
    this->computeCurrentState(callTime * NANO2SEC);

    // Write the module output messages
    this->writeOutputMessages(callTime);
}

/*! This intermediate method groups the calculation of rotation parameters into a single method.

*/
void PrescribedRotation1DOF::computeRotationParameters() {
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

/*! This method computes the required parameters for the rotation with a non-smoothed bang-bang acceleration profile.

*/
void PrescribedRotation1DOF::computeBangBangParametersNoSmoothing() {
    // Determine the total time required for the rotation
    double totalRotTime = sqrt(((0.5 * fabs(this->thetaRef - this->thetaInit)) * 8.0) / this->thetaDDotMax);

    // Determine the time when the rotation is complete t_f
    this->t_f = this->tInit + totalRotTime;

    // Determine the time halfway through the rotation
    this->t_b1 = this->tInit + (totalRotTime / 2.0);

    // Define the parabolic constants for the first and second half of the rotation
    this->a = 0.5 * (this->thetaRef - this->thetaInit) / ((this->t_b1 - this->tInit) * (this->t_b1 - this->tInit));
    this->b = -0.5 * (this->thetaRef - this->thetaInit) / ((this->t_b1 - this->t_f) * (this->t_b1 - this->t_f));
}

/*! This method computes the required parameters for the rotation with a non-smoothed bang-coast-bang acceleration profile.

*/
void PrescribedRotation1DOF::computeBangCoastBangParametersNoSmoothing() {
    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);

    // Determine the time at the end of the first bang segment t_b1
    this->t_b1 = this->tInit + this->coastOptionBangDuration;

    // Determine the hub-relative angle and rate at time t_b1
    this->theta_tb1 = sign * 0.5 * this->thetaDDotMax * this->coastOptionBangDuration
                         * this->coastOptionBangDuration + this->thetaInit;
    this->thetaDot_tb1 = sign * this->thetaDDotMax * this->coastOptionBangDuration;

    // Determine the angle traveled during the coast period
    double deltaThetaCoast = this->thetaRef - this->thetaInit - 2.0 * (this->theta_tb1 - this->thetaInit);

    // Determine the duration of the coast segment coastDuration
    double coastDuration = fabs(deltaThetaCoast / this->thetaDot_tb1);

    // Determine the time at the end of the coast segment t_c
    this->t_c = this->t_b1 + coastDuration;

    // Determine the hub-relative angle at time t_c
    double theta_tc = this->theta_tb1 + deltaThetaCoast;

    // Determine the time when the rotation is complete t_f
    this->t_f = this->t_c + this->coastOptionBangDuration;

    // Define the parabolic constants for the first and second bang segments of the rotation
    this->a = (this->theta_tb1 - this->thetaInit) / ((this->t_b1 - this->tInit) * (this->t_b1 - this->tInit));
    this->b = -(this->thetaRef - theta_tc) / ((this->t_c - this->t_f) * (this->t_c - this->t_f));
}

/*! This method computes the required parameters for the rotation with a smoothed bang-bang acceleration profile.

*/
void PrescribedRotation1DOF::computeSmoothedBangBangParameters() {
    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);

    // Determine the time at the end of the first smoothing segment t_s1
    this->t_s1 = this->tInit + this->smoothingDuration;

    // Determine the hub-relative angle and rate at time t_s1
    this->thetaDot_ts1 = sign * 0.5 * this->thetaDDotMax * this->smoothingDuration;
    this->theta_ts1 = sign * (3.0 / 20.0) * this->thetaDDotMax * this->smoothingDuration * this->smoothingDuration
                         + this->thetaInit;

    // Determine the duration of the bang segment bangDuration
    double aTerm = sign * 0.5 * this->thetaDDotMax;
    double bTerm = (sign * this->thetaDDotMax * this->smoothingDuration + this->thetaDot_ts1) / aTerm;
    double cTerm = (sign * (2.0 / 5.0) * this->thetaDDotMax * this->smoothingDuration * this->smoothingDuration
                   + this->thetaDot_ts1 * this->smoothingDuration + this->theta_ts1
                   - 0.5 * (this->thetaRef + this->thetaInit)) / aTerm;
    double bangDuration = (- bTerm + sqrt(bTerm * bTerm - 4.0 * cTerm)) / 2.0;

    // Determine the time at the end of the first bang segment t_b1
    this->t_b1 = this->t_s1 + bangDuration;

    // Determine the hub-relative angle and rate at time t_b1
    this->thetaDot_tb1 = sign * this->thetaDDotMax * bangDuration + this->thetaDot_ts1;
    this->theta_tb1 = sign * 0.5 * this->thetaDDotMax * bangDuration * bangDuration
                         + this->thetaDot_ts1 * bangDuration + this->theta_ts1;

    // Determine the time at the end of the second smoothing segment t_s2
    this->t_s2 = this->t_b1 + 2.0 * this->smoothingDuration;

    // Determine the hub-relative angle and rate at time t_s2
    this->thetaDot_ts2 = this->thetaDot_tb1;
    this->theta_ts2 = sign * (4.0 / 5.0) * this->thetaDDotMax * this->smoothingDuration * this->smoothingDuration
                         + this->thetaDot_tb1 * 2.0 * this->smoothingDuration + this->theta_tb1;

    // Determine the time at the end of the second bang segment t_b2
    this->t_b2 = this->t_s2 + bangDuration;

    // Determine the hub-relative angle and rate at time t_b2
    this->thetaDot_tb2 = - sign * this->thetaDDotMax * bangDuration + this->thetaDot_ts2;
    this->theta_tb2 = - sign * 0.5 * this->thetaDDotMax * bangDuration * bangDuration
                         + this->thetaDot_ts2 * bangDuration + this->theta_ts2;

    // Determine the time when the rotation is complete t_f
    this->t_f = this->t_b2 + this->smoothingDuration;
}

/*! This method computes the required parameters for the rotation with a smoothed bang-coast-bang acceleration profile.

*/
void PrescribedRotation1DOF::computeSmoothedBangCoastBangParameters() {
    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);

    // Determine the time at the end of the first smoothing segment t_s1
    this->t_s1 = this->tInit + this->smoothingDuration;

    // Determine the hub-relative angle and rate at time t_s1
    this->thetaDot_ts1 = sign * 0.5 * this->thetaDDotMax * this->smoothingDuration;
    this->theta_ts1 = sign * (3.0 / 20.0) * this->thetaDDotMax * this->smoothingDuration * this->smoothingDuration
                         + this->thetaInit;

    // Determine the time at the end of the first bang segment t_b1
    this->t_b1 = this->t_s1 + this->coastOptionBangDuration;

    // Determine the hub-relative angle and rate at time t_b1
    this->thetaDot_tb1 = sign * this->thetaDDotMax * this->coastOptionBangDuration + this->thetaDot_ts1;
    this->theta_tb1 = sign * 0.5 * this->thetaDDotMax * this->coastOptionBangDuration
                         * this->coastOptionBangDuration + this->thetaDot_ts1 * this->coastOptionBangDuration
                         + this->theta_ts1;

    // Determine the time at the end of the second smoothing segment t_s2
    this->t_s2 = this->t_b1 + this->smoothingDuration;

    // Determine the hub-relative angle and rate at time t_s2
    this->thetaDot_ts2 = sign * 0.5 * this->thetaDDotMax * this->smoothingDuration + this->thetaDot_tb1;
    this->theta_ts2 = sign * (7.0 / 20.0) * this->thetaDDotMax * this->smoothingDuration * this->smoothingDuration
                         + this->thetaDot_tb1 * this->smoothingDuration + this->theta_tb1;

    // Compute the time at the end of the coast segment t_c
    double deltaThetaCoast = (this->thetaRef - this->thetaInit) - 2 * (this->theta_ts2 - this->thetaInit);
    this->t_c = (deltaThetaCoast / this->thetaDot_ts2) + this->t_s2;

    // Determine the hub-relative angle and rate at time t_c
    this->thetaDot_tc = this->thetaDot_ts2;
    this->theta_tc = this->thetaDot_ts2 * (this->t_c - this->t_s2) + this->theta_ts2;

    // Determine the time at the end of the third smoothing segment t_s3
    this->t_s3 = this->t_c + this->smoothingDuration;

    // Determine the hub-relative angle and rate at time t_s3
    this->thetaDot_ts3 = - sign * 0.5 * this->thetaDDotMax * this->smoothingDuration + this->thetaDot_tc;
    this->theta_ts3 = - sign * (3.0 / 20.0) * this->thetaDDotMax * this->smoothingDuration * this->smoothingDuration
                         + this->thetaDot_tc * this->smoothingDuration + this->theta_tc;

    // Determine the time at the end of the second bang segment t_b2
    this->t_b2 = this->t_s3 + this->coastOptionBangDuration;

    // Determine the hub-relative angle and rate at time t_b2
    this->thetaDot_tb2 = - sign * this->thetaDDotMax * this->coastOptionBangDuration + this->thetaDot_ts3;
    this->theta_tb2 = - sign * 0.5 * this->thetaDDotMax * this->coastOptionBangDuration
                         * this->coastOptionBangDuration + this->thetaDot_ts3 * this->coastOptionBangDuration
                         + this->theta_ts3;

    // Determine the time when the rotation is complete t_f
    this->t_f = this->t_b2 + this->smoothingDuration;
}

/*! This intermediate method groups the calculation of the current rotational states into a single method.

*/
void PrescribedRotation1DOF::computeCurrentState(double t) {
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
                this->computeRotationComplete();
            }
        } else {
            if (this->isInFirstBangSegment(t)) {
                this->computeFirstBangSegment(t);
            } else if (this->isInCoastSegment(t)) {
                this->computeCoastSegment(t);
            } else if (this->isInSecondBangSegment(t)) {
                this->computeSecondBangSegment(t);
            } else {
                this->computeRotationComplete();
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
                this->computeRotationComplete();
            }
        } else {
            if (this->isInFirstBangSegment(t)) {
                this->computeFirstBangSegment(t);
            } else if (this->isInSecondBangSegment(t)) {
                this->computeSecondBangSegment(t);
            } else {
                this->computeRotationComplete();
            }
        }
    }
}

/*! This method determines if the current time is within the first bang segment.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInFirstBangSegment(double t) const {
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
bool PrescribedRotation1DOF::isInSecondBangSegment(double t) const {
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
bool PrescribedRotation1DOF::isInFirstSmoothedSegment(double t) const {
    return (t <= this->t_s1 && this->t_f - this->tInit != 0.0);
}

/*! This method determines if the current time is within the second smoothing segment for the smoothed profiler options..
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInSecondSmoothedSegment(double t) const {
    return (t > this->t_b1 && t <= this->t_s2 && this->t_f - this->tInit != 0.0);
}

/*! This method determines if the current time is within the third smoothing segment for the smoothed profiler options.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInThirdSmoothedSegment(double t) const {
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
bool PrescribedRotation1DOF::isInFourthSmoothedSegment(double t) const {
    return (t > this->t_b2 && t <= this->t_f && this->t_f - this->tInit != 0.0);
}

/*! This method determines if the current time is within the coast segment.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInCoastSegment(double t) const {
    if (this->smoothingDuration > 0.0) {
        return (t > this->t_s2 && t <= this->t_c && this->t_f - this->tInit != 0.0);
    } else{
        return (t > this->t_b1 && t <= this->t_c && this->t_f - this->tInit != 0.0);
    }
}

/*! This method computes the scalar rotational states for the first bang segment.

 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeFirstBangSegment(double t) {
    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);
    this->thetaDDot = sign * this->thetaDDotMax;

    if (this->smoothingDuration > 0.0) {
        this->thetaDot = this->thetaDDot * (t - this->t_s1) + this->thetaDot_ts1;
        this->theta = 0.5 * this->thetaDDot * (t - this->t_s1) * (t - this->t_s1)
                         + this->thetaDot_ts1 * (t - this->t_s1) + this->theta_ts1;
    } else {
        this->thetaDot = this->thetaDDot * (t - this->tInit);
        this->theta = this->a * (t - this->tInit) * (t - this->tInit) + this->thetaInit;
    }
}

/*! This method computes the scalar rotational states for the second bang segment.

 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeSecondBangSegment(double t) {
    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);
    this->thetaDDot = - sign * this->thetaDDotMax;

    if (this->smoothingDuration > 0.0) {
        if (this->coastOptionBangDuration > 0.0) {
            this->thetaDot = this->thetaDDot * (t - this->t_s3) + this->thetaDot_ts3;
            this->theta = 0.5 * this->thetaDDot * (t - this->t_s3) * (t - this->t_s3)
                             + this->thetaDot_ts3 * (t - this->t_s3) + this->theta_ts3;
        } else {
            this->thetaDot = this->thetaDDot * (t - this->t_s2) + this->thetaDot_ts2;
            this->theta = 0.5 * this->thetaDDot * (t - this->t_s2) * (t - this->t_s2)
                             + this->thetaDot_ts2 * (t - this->t_s2) + this->theta_ts2;
        }
    } else {
        this->thetaDot = this->thetaDDot * (t - this->t_f);
        this->theta = this->b * (t - this->t_f) * (t - this->t_f) + this->thetaRef;
    }
}

/*! This method computes the first smoothing segment scalar rotational states for the smoothed profiler options.

 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeFirstSmoothedSegment(double t) {
    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);

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

    this->thetaDDot = sign * this->thetaDDotMax * (term1 - term2);
    this->thetaDot = sign * this->thetaDDotMax * (term3 - term4);
    this->theta = sign * this->thetaDDotMax * (term5 - term6) + this->thetaInit;
}

/*! This method computes the second smoothing segment scalar rotational states for the smoothed profiler options.

 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeSecondSmoothedSegment(double t) {
    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);

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

    this->thetaDDot = sign * this->thetaDDotMax * (1.0 - term1 + term2);
    this->thetaDot = sign * this->thetaDDotMax * ((t - this->t_b1) - term3 + term4) + this->thetaDot_tb1;
    this->theta = sign * this->thetaDDotMax * (term5 - term6 + term7)
                     + this->thetaDot_tb1 * (t - this->t_b1) + this->theta_tb1;
}

/*! This method computes the third smoothing segment scalar rotational states for the smoothed profiler options.

 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeThirdSmoothedSegment(double t) {
    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);

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

        this->thetaDDot = - sign * this->thetaDDotMax * (term1 - term2);
        this->thetaDot = - sign * this->thetaDDotMax * (term3 - term4) + this->thetaDot_tc;
        this->theta = - sign * this->thetaDDotMax * (term5 - term6) + this->thetaDot_tc * (t - this->t_c) + this->theta_tc;
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

        this->thetaDDot = sign * this->thetaDDotMax * ( - 1.0 + term1 - term2);
        this->thetaDot = sign * this->thetaDDotMax * ( - (t - this->t_b2) + term3 - term4) + this->thetaDot_tb2;
        this->theta = sign * this->thetaDDotMax * (term5 + term6 - term7) + this->thetaDot_tb2 * (t - this->t_b2)
                         + this->theta_tb2;
    }
}

/*! This method computes the fourth smoothing segment scalar rotational states for the smoothed bang-coast-bang option.

 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeFourthSmoothedSegment(double t) {
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

    double sign = (this->thetaRef - this->thetaInit) / abs(this->thetaRef - this->thetaInit);

    this->thetaDDot = - sign * this->thetaDDotMax * (term1 - term2);
    this->thetaDot = sign * this->thetaDDotMax * (term3 - term4);
    this->theta = - sign * this->thetaDDotMax * (term5 - term6) + this->thetaRef;
}

/*! This method computes the coast segment scalar rotational states

 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeCoastSegment(double t) {
    this->thetaDDot = 0.0;

    if (this->smoothingDuration > 0.0) {
        this->thetaDot = this->thetaDot_ts2;
        this->theta = this->thetaDot_ts2 * (t - this->t_s2) + this->theta_ts2;
    } else {
        this->thetaDot = this->thetaDot_tb1;
        this->theta = this->thetaDot_tb1 * (t - this->t_b1) + this->theta_tb1;
    }
}

/*! This method computes the scalar rotational states when the rotation is complete.

*/
void PrescribedRotation1DOF::computeRotationComplete() {
    this->thetaDDot = 0.0;
    this->thetaDot = 0.0;
    this->theta = this->thetaRef;
    this->convergence = true;
}

/*! This method writes the module output messages and computes the output message data.

*/
void PrescribedRotation1DOF::writeOutputMessages(uint64_t callTime) {
    // Create the output buffer messages
    HingedRigidBodyMsgPayload spinningBodyOut;
    PrescribedRotationMsgPayload prescribedRotationOut;

    // Zero the output messages
    spinningBodyOut = HingedRigidBodyMsgPayload();
    prescribedRotationOut = PrescribedRotationMsgPayload();

    // Compute the angular velocity of frame P wrt frame M in P frame components
    Eigen::Vector3d omega_PM_P = this->thetaDot * this->rotHat_M;  // [rad/s]

    // Compute the B frame time derivative of omega_PM_P in P frame components
    Eigen::Vector3d omegaPrime_PM_P = this->thetaDDot * this->rotHat_M;  // [rad/s^2]

    // Compute the MRP attitude of spinning body frame P with respect to frame M
    Eigen::Vector3d sigma_PM = this->computeSigma_PM();

    // Copy the module variables to the output buffer messages
    spinningBodyOut.theta = this->theta;
    spinningBodyOut.thetaDot = this->thetaDot;
    eigenVector3d2CArray(omega_PM_P, prescribedRotationOut.omega_PM_P);
    eigenVector3d2CArray(omegaPrime_PM_P, prescribedRotationOut.omegaPrime_PM_P);
    eigenVector3d2CArray(sigma_PM, prescribedRotationOut.sigma_PM);

    // Write the output messages
    this->spinningBodyOutMsg.write(&spinningBodyOut, moduleID, callTime);
    this->prescribedRotationOutMsg.write(&prescribedRotationOut, moduleID, callTime);
    HingedRigidBodyMsg_C_write(&spinningBodyOut, &spinningBodyOutMsgC, this->moduleID, callTime);
    PrescribedRotationMsg_C_write(&prescribedRotationOut, &prescribedRotationOutMsgC, this->moduleID, callTime);
}

/*! This method computes the current spinning body MRP attitude relative to the mount frame: sigma_PM

*/
Eigen::Vector3d PrescribedRotation1DOF::computeSigma_PM() {
    // Determine dcm_PP0 for the current spinning body attitude relative to the initial attitude
    double dcm_PP0[3][3];
    double prv_PP0_array[3];
    double theta_PP0 = this->theta - this->thetaInit;
    Eigen::Vector3d prv_PP0 = theta_PP0 * this->rotHat_M;
    eigenVector3d2CArray(prv_PP0, prv_PP0_array);
    PRV2C(prv_PP0_array, dcm_PP0);

    // Determine dcm_P0M for the initial spinning body attitude relative to the mount frame
    double dcm_P0M[3][3];
    double prv_P0M_array[3];
    Eigen::Vector3d prv_P0M = this->thetaInit * this->rotHat_M;
    eigenVector3d2CArray(prv_P0M, prv_P0M_array);
    PRV2C(prv_P0M_array, dcm_P0M);

    // Determine dcm_PM for the current spinning body attitude relative to the mount frame
    double dcm_PM[3][3];
    m33MultM33(dcm_PP0, dcm_P0M, dcm_PM);

    // Compute the MRP sigma_PM representing the current spinning body attitude relative to the mount frame
    double sigma_PM_array[3];
    C2MRP(dcm_PM, sigma_PM_array);
    return cArray2EigenVector3d(sigma_PM_array);
}

/*! Setter method for the coast option bang duration.

 @param bangDuration [s] Bang segment time duration
*/
void PrescribedRotation1DOF::setCoastOptionBangDuration(const double bangDuration) {
    this->coastOptionBangDuration = bangDuration;
}

/*! Setter method for the spinning body rotation axis.

 @param rotHat_M Spinning body rotation axis (unit vector)
*/
void PrescribedRotation1DOF::setRotHat_M(const Eigen::Vector3d &rotHat_M) {
    this->rotHat_M = rotHat_M / rotHat_M.norm();
}

/*! Setter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value.

 @param smoothingDuration [s] Duration the acceleration is smoothed until reaching the given maximum acceleration value
*/
void PrescribedRotation1DOF::setSmoothingDuration(const double smoothingDuration) {
    this->smoothingDuration = smoothingDuration;
}

/*! Setter method for the bang segment scalar angular acceleration.

 @param thetaDDotMax [rad/s^2] Bang segment scalar angular acceleration
*/
void PrescribedRotation1DOF::setThetaDDotMax(const double thetaDDotMax) {
    this->thetaDDotMax = thetaDDotMax;
}

/*! Setter method for the initial spinning body angle.

 @param thetaInit [rad] Initial spinning body angle
*/
void PrescribedRotation1DOF::setThetaInit(const double thetaInit) {
    this->thetaInit = thetaInit;
}

/*! Getter method for the coast option bang duration.
 @return double
*/
double PrescribedRotation1DOF::getCoastOptionBangDuration() const {
    return this->coastOptionBangDuration;
}

/*! Getter method for the spinning body rotation axis.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &PrescribedRotation1DOF::getRotHat_M() const {
    return this->rotHat_M;
}

/*! Getter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value.
 @return double
*/
double PrescribedRotation1DOF::getSmoothingDuration() const {
    return this->smoothingDuration;
}

/*! Getter method for the ramp segment scalar angular acceleration.
 @return double
*/
double PrescribedRotation1DOF::getThetaDDotMax() const {
    return this->thetaDDotMax;
}

/*! Getter method for the initial spinning body angle.
 @return double
*/
double PrescribedRotation1DOF::getThetaInit() const {
    return this->thetaInit;
}
