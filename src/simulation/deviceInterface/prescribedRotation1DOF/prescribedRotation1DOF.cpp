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
 @return void
*/
void PrescribedRotation1DOF::SelfInit() {
    HingedRigidBodyMsg_C_init(&this->spinningBodyOutMsgC);
    PrescribedRotationMsg_C_init(&this->prescribedRotationOutMsgC);
}

/*! This method resets required module variables and checks the input messages to ensure they are linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedRotation1DOF::Reset(uint64_t callTime) {
    if (!this->spinningBodyInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: prescribedRotation1DOF.spinningBodyInMsg wasn't connected.");
    }

    this->tInit = 0.0;
    this->theta = this->thetaInit;
    this->thetaDot = 0.0;

    // Set the initial convergence to true to enter the required loop in Update() method on the first pass
    this->convergence = true;
}

/*! This method profiles the spinning body rotation and updates the prescribed rotational states as a function of time.
 The spinning body rotational states are then written to the output message.
 @return void
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
        if (this->coastOptionBangDuration > 0.0) {
            this->computeCoastParameters();
        } else {
            this->computeParametersNoCoast();
        }

        // Set the convergence to false until the rotation is complete
        this->convergence = false;
    }

    // Compute the scalar rotational states at the current simulation time
    this->computeCurrentState(callTime * NANO2SEC);

    // Write the module output messages
    this->writeOutputMessages(callTime);
}

/*! This method computes the required parameters for the rotation with no coast period.
 @return void
*/
void PrescribedRotation1DOF::computeParametersNoCoast() {
    // Determine the total time required for the rotation
    double totalRotTime = sqrt(((0.5 * fabs(this->thetaRef - this->thetaInit)) * 8) / this->thetaDDotMax);

    // Determine the time at the end of the rotation
    this->t_f = this->tInit + totalRotTime;

    // Determine the time halfway through the rotation
    this->t_s = this->tInit + (totalRotTime / 2);

    // Define the parabolic constants for the first and second half of the rotation
    this->a = 0.5 * (this->thetaRef - this->thetaInit) / ((this->t_s - this->tInit) * (this->t_s - this->tInit));
    this->b = -0.5 * (this->thetaRef - this->thetaInit) / ((this->t_s - this->t_f) * (this->t_s - this->t_f));
}

/*! This method computes the required parameters for the rotation with a coast period.
 @return void
*/
void PrescribedRotation1DOF::computeCoastParameters() {
    if (this->thetaInit != this->thetaRef) {
        // Determine the time at the end of the first bang segment
        this->t_r = this->tInit + this->coastOptionBangDuration;

        // Determine the angle and angle rate at the end of the bang segment/start of the coast segment
        if (this->thetaInit < this->thetaRef) {
            this->theta_tr = (0.5 * this->thetaDDotMax * this->coastOptionBangDuration * this->coastOptionBangDuration)
                             + this->thetaInit;
            this->thetaDot_tr = this->thetaDDotMax * this->coastOptionBangDuration;
        } else {
            this->theta_tr = - (0.5 * this->thetaDDotMax * this->coastOptionBangDuration
                             * this->coastOptionBangDuration) + this->thetaInit;
            this->thetaDot_tr = - this->thetaDDotMax * this->coastOptionBangDuration;
        }

        // Determine the angle traveled during the coast period
        double deltaThetaCoast = this->thetaRef - this->thetaInit - 2 * (this->theta_tr - this->thetaInit);

        // Determine the time duration of the coast segment
        double tCoast = fabs(deltaThetaCoast) / fabs(this->thetaDot_tr);

        // Determine the time at the end of the coast segment
        this->t_c = this->t_r + tCoast;

        // Determine the angle at the end of the coast segment
        this->theta_tc = this->theta_tr + deltaThetaCoast;

        // Determine the time at the end of the rotation
        this->t_f = this->t_c + this->coastOptionBangDuration;

        // Define the parabolic constants for the first and second bang segments of the rotation
        this->a = (this->theta_tr - this->thetaInit) / ((this->t_r - this->tInit) * (this->t_r - this->tInit));
        this->b = - (this->thetaRef - this->theta_tc) / ((this->t_c - this->t_f) * (this->t_c - this->t_f));
    } else { // If the initial angle equals the reference angle, no rotation is required. Setting the final time
        // equal to the initial time ensures the correct statement is entered when the rotational states are
        // profiled below
        this->t_f = this->tInit;
    }
}

/*! This intermediate method groups the calculation of the current rotational states into a single method.
 @return void
*/
void PrescribedRotation1DOF::computeCurrentState(double t) {
    if (this->coastOptionBangDuration > 0.0) {
        if (this->isInFirstBangSegment(t)) {
            this->computeFirstBangSegment(t);
        } else if (this->isInCoastSegment(t)) {
            this->computeCoastSegment(t);
        } else if (this->isInSecondBangSegment(t)) {
            this->computeSecondBangSegment(t);
        } else {
            this->computeRotationComplete();
        }
    } else {
        if (this->isInFirstBangSegmentNoCoast(t)) {
            this->computeFirstBangSegment(t);
        } else if (this->isInSecondBangSegmentNoCoast(t)) {
            this->computeSecondBangSegment(t);
        } else {
            this->computeRotationComplete();
        }
    }
}

/*! This method determines if the current time is within the first bang segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInFirstBangSegmentNoCoast(double t) const {
    return (t <= this->t_s && this->t_f - this->tInit != 0);
}

/*! This method determines if the current time is within the first bang segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInFirstBangSegment(double t) const {
    return (t <= this->t_r && this->t_f - this->tInit != 0);
}

/*! This method determines if the current time is within the second bang segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInSecondBangSegmentNoCoast(double t) const {
    return (t > this->t_s && t <= this->t_f && this->t_f - this->tInit != 0);
}

/*! This method determines if the current time is within the second bang segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInSecondBangSegment(double t) const {
    return (t > this->t_c && t <= this->t_f && this->t_f - this->tInit != 0);
}

/*! This method determines if the current time is within the coast segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInCoastSegment(double t) const {
    return (t > this->t_r && t <= this->t_c && this->t_f - this->tInit != 0);
}

/*! This method computes the scalar rotational states for the first bang segment.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeFirstBangSegment(double t) {
    // The acceleration during the first bang segment is positive if the reference angle is greater than
    // the initial angle. The acceleration is negative during the first bang segment if the reference angle
    // is less than the initial angle
    if (this->thetaInit < this->thetaRef) {
        this->thetaDDot = this->thetaDDotMax;
    } else {
        this->thetaDDot = - this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit);
    this->theta = this->a * (t - this->tInit) * (t - this->tInit) + this->thetaInit;
}

/*! This method computes the scalar rotational states for the second bang segment.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeSecondBangSegment(double t) {
    // The acceleration during the second bang segment is negative if the reference angle is greater than
    // the initial angle. The acceleration is positive during the second bang segment if the reference angle
    // is less than the initial angle
    if (this->thetaInit < this->thetaRef) {
        this->thetaDDot = - this->thetaDDotMax;
    } else {
        this->thetaDDot = this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit) - this->thetaDDot * (this->t_f - this->tInit);
    this->theta = this->b * (t - this->t_f) * (t - this->t_f) + this->thetaRef;
}

/*! This method computes the scalar rotational states for the coast option coast period.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeCoastSegment(double t) {
    this->thetaDDot = 0.0;
    this->thetaDot = this->thetaDot_tr;
    this->theta = this->thetaDot_tr * (t - this->t_r) + this->theta_tr;
}

/*! This method computes the scalar rotational states when the rotation is complete.
 @return void
*/
void PrescribedRotation1DOF::computeRotationComplete() {
    this->thetaDDot = 0.0;
    this->thetaDot = 0.0;
    this->theta = this->thetaRef;
    this->convergence = true;
}

/*! This method writes the module output messages and computes the output message data.
 @return void
*/
void PrescribedRotation1DOF::writeOutputMessages(uint64_t callTime) {
    // Create the output buffer messages
    HingedRigidBodyMsgPayload spinningBodyOut;
    PrescribedRotationMsgPayload prescribedRotationOut;

    // Zero the output messages
    spinningBodyOut = HingedRigidBodyMsgPayload();
    prescribedRotationOut = PrescribedRotationMsgPayload();

    // Compute the angular velocity of frame F wrt frame M in F frame components
    Eigen::Vector3d omega_FM_F = this->thetaDot * this->rotHat_M;  // [rad/s]

    // Compute the B frame time derivative of omega_FM_F in F frame components
    Eigen::Vector3d omegaPrime_FM_F = this->thetaDDot * this->rotHat_M;  // [rad/s^2]

    // Compute the MRP attitude of spinning body frame F with respect to frame M
    Eigen::Vector3d sigma_FM = this->computeSigma_FM();

    // Copy the module variables to the output buffer messages
    spinningBodyOut.theta = this->theta;
    spinningBodyOut.thetaDot = this->thetaDot;
    eigenVector3d2CArray(omega_FM_F, prescribedRotationOut.omega_FM_F);
    eigenVector3d2CArray(omegaPrime_FM_F, prescribedRotationOut.omegaPrime_FM_F);
    eigenVector3d2CArray(sigma_FM, prescribedRotationOut.sigma_FM);

    // Write the output messages
    this->spinningBodyOutMsg.write(&spinningBodyOut, moduleID, callTime);
    this->prescribedRotationOutMsg.write(&prescribedRotationOut, moduleID, callTime);
    HingedRigidBodyMsg_C_write(&spinningBodyOut, &spinningBodyOutMsgC, this->moduleID, callTime);
    PrescribedRotationMsg_C_write(&prescribedRotationOut, &prescribedRotationOutMsgC, this->moduleID, callTime);
}

/*! This method computes the current spinning body MRP attitude relative to the mount frame: sigma_FM
 @return void
*/
Eigen::Vector3d PrescribedRotation1DOF::computeSigma_FM() {
    // Determine dcm_FF0 for the current spinning body attitude relative to the initial attitude
    double dcm_FF0[3][3];
    double prv_FF0_array[3];
    double theta_FF0 = this->theta - this->thetaInit;
    Eigen::Vector3d prv_FF0 = theta_FF0 * this->rotHat_M;
    eigenVector3d2CArray(prv_FF0, prv_FF0_array);
    PRV2C(prv_FF0_array, dcm_FF0);

    // Determine dcm_F0M for the initial spinning body attitude relative to the mount frame
    double dcm_F0M[3][3];
    double prv_F0M_array[3];
    Eigen::Vector3d prv_F0M = this->thetaInit * this->rotHat_M;
    eigenVector3d2CArray(prv_F0M, prv_F0M_array);
    PRV2C(prv_F0M_array, dcm_F0M);

    // Determine dcm_FM for the current spinning body attitude relative to the mount frame
    double dcm_FM[3][3];
    m33MultM33(dcm_FF0, dcm_F0M, dcm_FM);

    // Compute the MRP sigma_FM representing the current spinning body attitude relative to the mount frame
    double sigma_FM_array[3];
    C2MRP(dcm_FM, sigma_FM_array);
    return cArray2EigenVector3d(sigma_FM_array);
}

/*! Setter method for the coast option bang duration.
 @return void
 @param bangDuration [s] Bang segment time duration
*/
void PrescribedRotation1DOF::setCoastOptionBangDuration(double bangDuration) {
    this->coastOptionBangDuration = bangDuration;
}

/*! Setter method for the spinning body rotation axis.
 @return void
 @param rotHat_M Spinning body rotation axis (unit vector)
*/
void PrescribedRotation1DOF::setRotHat_M(const Eigen::Vector3d &rotHat_M) {
    this->rotHat_M = rotHat_M / rotHat_M.norm();
}

/*! Setter method for the bang segment scalar angular acceleration.
 @return void
 @param thetaDDotMax [rad/s^2] Bang segment scalar angular acceleration
*/
void PrescribedRotation1DOF::setThetaDDotMax(double thetaDDotMax) {
    this->thetaDDotMax = thetaDDotMax;
}

/*! Setter method for the initial spinning body angle.
 @return void
 @param thetaInit [rad] Initial spinning body angle
*/
void PrescribedRotation1DOF::setThetaInit(double thetaInit) {
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

/*! Getter method for the bang segment scalar angular acceleration.
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
