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

#include "singleAxisProfiler.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <cmath>

/*! This method checks the input message to ensure it is linked.
 @return void
 @param callTime [ns] Time the method is called
*/
void SingleAxisProfiler::Reset(uint64_t callTime) {
    if (!this->stepperMotorInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "singleAxisProfiler.stepperMotorInMsg wasn't connected.");
    }
}

/*! This method converts the stepper motor scalar rotational states to the prescribed motion hub-relative rotational
states. The prescribed rotational states are then written to the output message.
 @return void
 @param callTime [ns] Time the method is called
*/
void SingleAxisProfiler::UpdateState(uint64_t callTime) {
    // Read the input message
    StepperMotorMsgPayload stepperMotorIn;
    if (this->stepperMotorInMsg.isWritten()) {
        stepperMotorIn = StepperMotorMsgPayload();
        stepperMotorIn = this->stepperMotorInMsg();
    }

    // Create the output buffer message
    PrescribedRotationMsgPayload prescribedRotationOut;
    prescribedRotationOut = PrescribedRotationMsgPayload();

    // Compute the angular velocity of frame F wrt frame M in F frame components
    Eigen::Vector3d omega_FM_F = stepperMotorIn.thetaDot * this->rotHat_M;  // [rad/s]

    // Compute the B frame time derivative of omega_FM_F in F frame components
    Eigen::Vector3d omegaPrime_FM_F = stepperMotorIn.thetaDDot * this->rotHat_M;  // [rad/s^2]

    // Compute the MRP attitude of spinning body frame F with respect to frame M
    Eigen::Vector3d sigma_FM = this->computeSigma_FM(stepperMotorIn.theta);

    // Copy the prescribed rotational states to the output buffer message
    eigenVector3d2CArray(omega_FM_F, prescribedRotationOut.omega_FM_F);
    eigenVector3d2CArray(omegaPrime_FM_F, prescribedRotationOut.omegaPrime_FM_F);
    eigenVector3d2CArray(sigma_FM, prescribedRotationOut.sigma_FM);

    // Write the output message
    this->prescribedRotationOutMsg.write(&prescribedRotationOut, moduleID, callTime);
}

/*! This method computes the current spinning body MRP attitude relative to the mount frame: sigma_FM
 @return void
*/
Eigen::Vector3d SingleAxisProfiler::computeSigma_FM(double theta) {
    // Determine dcm_FM for the current spinning body attitude relative to the mount frame
    double dcm_FM[3][3];
    double prv_FM_array[3];
    Eigen::Vector3d prv_FM = theta * this->rotHat_M;
    eigenVector3d2CArray(prv_FM, prv_FM_array);
    PRV2C(prv_FM_array, dcm_FM);

    // Compute the MRP sigma_FM representing the current spinning body attitude relative to the mount frame
    double sigma_FM_array[3];
    C2MRP(dcm_FM, sigma_FM_array);
    return cArray2EigenVector3d(sigma_FM_array);
}

/*! Setter method for the spinning body rotation axis.
 @return void
 @param rotHat_M Spinning body rotation axis (unit vector)
*/
void SingleAxisProfiler::setRotHat_M(const Eigen::Vector3d &rotHat_M) {
    this->rotHat_M = rotHat_M / rotHat_M.norm();
}

/*! Getter method for the spinning body rotation axis.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &SingleAxisProfiler::getRotHat_M() const {
    return this->rotHat_M;
}
