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


/*! This method resets required module variables and checks the input messages to ensure they are linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void PrescribedRotation1DOF::Reset(uint64_t callTime) {
    // Check if the required input message is linked
    if (!this->spinningBodyInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "Error: prescribedRot.spinningBodyInMsg wasn't connected.");
    }

    this->tInit = 0.0;
    this->theta = this->thetaInit;
    this->thetaDotInit = 0.0;
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
    // Create the buffer messages
    HingedRigidBodyMsgPayload spinningBodyIn;
    HingedRigidBodyMsgPayload spinningBodyOut;
    PrescribedRotationMsgPayload prescribedRotationOut;

    // Zero the output messages
    spinningBodyOut = HingedRigidBodyMsgPayload();
    prescribedRotationOut = PrescribedRotationMsgPayload();

    // Read the input message
    spinningBodyIn = HingedRigidBodyMsgPayload();
    if (this->spinningBodyInMsg.isWritten()) {
        spinningBodyIn = this->spinningBodyInMsg();
    }

    /* This loop is entered (a) initially and (b) when each rotation is complete. The parameters used to profile the
    spinning body rotation are updated in this statement. */
    if (this->spinningBodyInMsg.timeWritten() <= callTime && this->convergence)
    {
        // Update the initial time as the current simulation time
        this->tInit = callTime * NANO2SEC;

        // Update the initial angle
        this->thetaInit = this->theta;

        // Store the reference angle
        this->thetaRef = spinningBodyIn.theta;

        // Set the convergence to false until the rotation is complete
        this->convergence = false;

        // Set the parameters required to profile the rotation
        if (this->coastOptionRampDuration > 0.0) {
            this->computeCoastParameters();
        } else {
            this->computeParametersNoCoast();
        }
    }

    // Store the current simulation time
    double t = callTime * NANO2SEC;

    // Compute the scalar rotational states at the current simulation time
    if (this->coastOptionRampDuration > 0.0) {
        if (this->isInFirstRampSegment(t)) {
            this->computeFirstRampSegment(t);
        } else if (this->isInCoastSegment(t)) {
            this->computeCoastSegment(t);
        } else if (this->isInSecondRampSegment(t)) {
            this->computeSecondRampSegment(t);
        } else {
            this->computeRotationComplete();
        }
    } else {
        if (this->isInFirstRampSegmentNoCoast(t)) {
            this->computeFirstRampSegment(t);
        } else if (this->isInSecondRampSegmentNoCoast(t)) {
            this->computeSecondRampSegment(t);
        } else {
            this->computeRotationComplete();
        }
    }

    // Determine the prescribed parameters omega_FM_F, omegaPrime_FM_F, and sigma_FM
    this->omega_FM_F = this->thetaDot * this->rotHat_M;
    this->omegaPrime_FM_F = this->thetaDDot * this->rotHat_M;
    this->computeSigma_FM();

    // Copy the module variables to the prescribedRotationOut output message
    eigenVector3d2CArray(this->omega_FM_F, prescribedRotationOut.omega_FM_F);
    eigenVector3d2CArray(this->omegaPrime_FM_F, prescribedRotationOut.omegaPrime_FM_F);
    eigenVector3d2CArray(this->sigma_FM, prescribedRotationOut.sigma_FM);

    // Copy the scalar variables to the spinningBodyOut output message
    spinningBodyOut.theta = this->theta;
    spinningBodyOut.thetaDot = this->thetaDot;

    // Write the output messages
    this->spinningBodyOutMsg.write(&spinningBodyOut, moduleID, callTime);
    this->prescribedRotationOutMsg.write(&prescribedRotationOut, moduleID, callTime);
}

/*! This method determines if the current time is within the first ramp segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInFirstRampSegment(double t) const {
    return (t <= this->tr && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the coast segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInCoastSegment(double t) const {
    return (t > this->tr && t <= this->tc && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the second ramp segment for the coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInSecondRampSegment(double t) const {
    return (t > this->tc && t <= this->tf && this->tf - this->tInit != 0);
}

/*! This method computes the required parameters for the rotation with a coast period.
 @return void
*/
void PrescribedRotation1DOF::computeCoastParameters() {
    if (this->thetaInit != this->thetaRef) {
        // Determine the time at the end of the first ramp segment
        this->tr = this->tInit + this->coastOptionRampDuration;

        // Determine the angle and angle rate at the end of the ramp segment/start of the coast segment
        if (this->thetaInit < this->thetaRef) {
            this->theta_tr = (0.5 * this->thetaDDotMax * this->coastOptionRampDuration * this->coastOptionRampDuration)
                             + (this->thetaDotInit * this->coastOptionRampDuration) + this->thetaInit;
            this->thetaDot_tr = this->thetaDDotMax * this->coastOptionRampDuration + this->thetaDotInit;
        } else {
            this->theta_tr = - ((0.5 * this->thetaDDotMax * this->coastOptionRampDuration * this->coastOptionRampDuration)
                                + (this->thetaDotInit * this->coastOptionRampDuration)) + this->thetaInit;
            this->thetaDot_tr = - this->thetaDDotMax * this->coastOptionRampDuration + this->thetaDotInit;
        }

        // Determine the angle traveled during the coast period
        double deltaThetaCoast = this->thetaRef - this->thetaInit - 2 * (this->theta_tr - this->thetaInit);

        // Determine the time duration of the coast segment
        double tCoast = fabs(deltaThetaCoast) / fabs(this->thetaDot_tr);

        // Determine the time at the end of the coast segment
        this->tc = this->tr + tCoast;

        // Determine the angle at the end of the coast segment
        this->theta_tc = this->theta_tr + deltaThetaCoast;

        // Determine the time at the end of the rotation
        this->tf = this->tc + this->coastOptionRampDuration;

        // Define the parabolic constants for the first and second ramp segments of the rotation
        this->a = (this->theta_tr - this->thetaInit) / ((this->tr - this->tInit) * (this->tr - this->tInit));
        this->b = - (this->thetaRef - this->theta_tc) / ((this->tc - this->tf) * (this->tc - this->tf));
    } else { // If the initial angle equals the reference angle, no rotation is required. Setting the final time
        // equal to the initial time ensures the correct statement is entered when the rotational states are
        // profiled below
        this->tf = this->tInit;
    }
}

/*! This method computes the scalar rotational states for the coast option coast period.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeCoastSegment(double t) {
    this->thetaDDot = 0.0;
    this->thetaDot = this->thetaDot_tr;
    this->theta = this->thetaDot_tr * (t - this->tr) + this->theta_tr;
}

/*! This method determines if the current time is within the first ramp segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInFirstRampSegmentNoCoast(double t) const {
    return (t <= this->ts && this->tf - this->tInit != 0);
}

/*! This method determines if the current time is within the second ramp segment for the no coast option.
 @return bool
 @param t [s] Current simulation time
*/
bool PrescribedRotation1DOF::isInSecondRampSegmentNoCoast(double t) const {
    return (t > this->ts && t <= this->tf && this->tf - this->tInit != 0);
}

/*! This method computes the required parameters for the rotation with no coast period.
 @return void
*/
void PrescribedRotation1DOF::computeParametersNoCoast() {
    // Determine the total time required for the rotation
    double totalRotTime = sqrt(((0.5 * fabs(this->thetaRef - this->thetaInit)) * 8) / this->thetaDDotMax);

    // Determine the time at the end of the rotation
    this->tf = this->tInit + totalRotTime;

    // Determine the time halfway through the rotation
    this->ts = this->tInit + (totalRotTime / 2);

    // Define the parabolic constants for the first and second half of the rotation
    this->a = 0.5 * (this->thetaRef - this->thetaInit) / ((this->ts - this->tInit) * (this->ts - this->tInit));
    this->b = -0.5 * (this->thetaRef - this->thetaInit) / ((this->ts - this->tf) * (this->ts - this->tf));
}

/*! This method computes the scalar rotational states for the first ramp segment.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeFirstRampSegment(double t) {
    // The acceleration during the first ramp segment is positive if the reference angle is greater than
    // the initial angle. The acceleration is negative during the first ramp segment if the reference angle
    // is less than the initial angle
    if (this->thetaInit < this->thetaRef) {
        this->thetaDDot = this->thetaDDotMax;
    } else {
        this->thetaDDot = - this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit) + this->thetaDotInit;
    this->theta = this->a * (t - this->tInit) * (t - this->tInit) + this->thetaInit;
}

/*! This method computes the scalar rotational states for the second ramp segment.
 @return void
 @param t [s] Current simulation time
*/
void PrescribedRotation1DOF::computeSecondRampSegment(double t) {
    // The acceleration during the second ramp segment is negative if the reference angle is greater than
    // the initial angle. The acceleration is positive during the second ramp segment if the reference angle
    // is less than the initial angle
    if (this->thetaInit < this->thetaRef) {
        this->thetaDDot = - this->thetaDDotMax;
    } else {
        this->thetaDDot = this->thetaDDotMax;
    }
    this->thetaDot = this->thetaDDot * (t - this->tInit) + this->thetaDotInit
                     - this->thetaDDot * (this->tf - this->tInit);
    this->theta = this->b * (t - this->tf) * (t - this->tf) + this->thetaRef;
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

/*! This method computes the current spinning body MRP attitude relative to the mount frame: sigma_FM
 @return void
*/
void PrescribedRotation1DOF::computeSigma_FM() {
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
    this->sigma_FM = cArray2EigenVector3d(sigma_FM_array);
}

/*! Setter method for the coast option ramp duration.
 @return void
 @param rampDuration [s] Ramp segment time duration
*/
void PrescribedRotation1DOF::setCoastOptionRampDuration(double rampDuration) {
    this->coastOptionRampDuration = rampDuration;
}

/*! Setter method for the spinning body rotation axis.
 @return void
 @param rotHat_M Spinning body rotation axis (unit vector)
*/
void PrescribedRotation1DOF::setRotHat_M(const Eigen::Vector3d &rotHat_M) {
    this->rotHat_M = rotHat_M / rotHat_M.norm();
}

/*! Setter method for the ramp segment scalar angular acceleration.
 @return void
 @param thetaDDotMax [rad/s^2] Ramp segment scalar angular acceleration
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

/*! Getter method for the coast option ramp duration.
 @return double
*/
double PrescribedRotation1DOF::getCoastOptionRampDuration() const {
    return this->coastOptionRampDuration;
}

/*! Getter method for the spinning body rotation axis.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &PrescribedRotation1DOF::getRotHat_M() const {
    return this->rotHat_M;
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
