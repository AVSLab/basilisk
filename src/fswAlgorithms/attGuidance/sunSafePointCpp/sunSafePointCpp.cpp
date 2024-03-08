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

#include "sunSafePointCpp.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <math.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] Time the method is called
*/
void SunSafePointCpp::Reset(uint64_t callTime)
{
    // Check if the required input messages are linked
    if (!this->sunDirectionInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "sunSafePointCpp.sunDirectionInMsg wasn't connected.");
    }
    if (!this->imuInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "sunSafePointCpp.imuInMsg wasn't connected.");
    }

    // Compute an Eigen axis orthogonal to sHatBdyCmd
    if (this->sHatBdyCmd.norm()  < 0.1) {
      char info[MAX_LOGGING_LENGTH];
      sprintf(info, "The module vector sHatBdyCmd is not setup as a unit vector [%f, %f %f]",
                this->sHatBdyCmd[0], this->sHatBdyCmd[1], this->sHatBdyCmd[2]);
      _bskLog(this->bskLogger, BSK_ERROR, info);
    } else {
        Eigen::Vector3d v1 = {1.0, 0.0, 0.0};
        this->sHatBdyCmd = this->sHatBdyCmd / this->sHatBdyCmd.norm();  // Ensure that this vector is a unit vector
        this->eHat180_B = this->sHatBdyCmd.cross(v1);
        if (this->eHat180_B.norm() < 0.1) {
            v1 = {0.0, 1.0, 0.0};
            this->eHat180_B = this->sHatBdyCmd.cross(v1);
        }
        this->eHat180_B = this->eHat180_B / this->eHat180_B.norm();
    }
}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param callTime [ns] Time the method is called
*/
void SunSafePointCpp::UpdateState(uint64_t callTime)
{
    // Create the buffer messages
    NavAttMsgPayload sunDirectionInBuffer;
    NavAttMsgPayload localImuDataInBuffer;
    AttGuidMsgPayload attGuidanceOutBuffer;

    // Zero the attitude guidance output buffer message
    attGuidanceOutBuffer = AttGuidMsgPayload();

    // Read the current sun body vector estimate input message
    sunDirectionInBuffer = NavAttMsgPayload();
    if (this->sunDirectionInMsg.isWritten()) {
        sunDirectionInBuffer = this->sunDirectionInMsg();
    }

    // Read the imu guidance input message
    localImuDataInBuffer = NavAttMsgPayload();
    if (this->imuInMsg.isWritten()) {
        localImuDataInBuffer = this->imuInMsg();
    }

    // Create local copy of hub inertial angular velocity vector in B frame components
    Eigen::Vector3d omega_BN_B = cArray2EigenVector3d(localImuDataInBuffer.omega_BN_B);  // [rad/s]

    // Create local copy of hub angular velocity error in B frame components
    Eigen::Vector3d omega_BR_B;  // [rad/s]

    // Determine norm of measured Sun-direction vector
    double sHatNorm = cArray2EigenVector3d(sunDirectionInBuffer.vehSunPntBdy).norm();

    if(sHatNorm > this->minUnitMag) {  // A good sun direction vector is available
        double dotProductNormalized = (this->sHatBdyCmd.dot(cArray2EigenVector3d(sunDirectionInBuffer.vehSunPntBdy)))
                                      / sHatNorm;
        dotProductNormalized = fabs(dotProductNormalized) > 1.0 ?
        dotProductNormalized/fabs(dotProductNormalized) : dotProductNormalized;
        this->sunAngleErr = safeAcos(dotProductNormalized);

        // Compute the heading error relative to the sun direction vector
        Eigen::Vector3d e_hat;  // Eigen Axis
        if (this->sunAngleErr < this->smallAngle) {  // Sun heading and desired body axis are essentially aligned. Set attitude error to zero.
            v3SetZero(attGuidanceOutBuffer.sigma_BR);
        } else {
            if (M_PI - this->sunAngleErr < this->smallAngle) {  // The commanded body vector nearly is opposite the sun heading
                e_hat = this->eHat180_B;
            } else {  // Normal case where sun and commanded body vectors are not aligned
                e_hat = cArray2EigenVector3d(sunDirectionInBuffer.vehSunPntBdy).cross(this->sHatBdyCmd);
            }
            this->sunMnvrVec = e_hat / e_hat.norm();
            Eigen::Vector3d v2 = tan(this->sunAngleErr * 0.25) * this->sunMnvrVec;
            eigenVector3d2CArray(v2, attGuidanceOutBuffer.sigma_BR);
            MRPswitch(attGuidanceOutBuffer.sigma_BR, 1.0, attGuidanceOutBuffer.sigma_BR);
        }

        // Rate tracking error are the body rates to bring spacecraft to rest
        this->omega_RN_B = (this->sunAxisSpinRate / sHatNorm) * cArray2EigenVector3d(sunDirectionInBuffer.vehSunPntBdy);
        omega_BR_B = omega_BN_B - this->omega_RN_B;
        eigenVector3d2CArray(omega_BR_B, attGuidanceOutBuffer.omega_BR_B);
        eigenVector3d2CArray(this->omega_RN_B, attGuidanceOutBuffer.omega_RN_B);

    } else {  // No proper sun direction vector is available
        v3SetZero(attGuidanceOutBuffer.sigma_BR);

        // Specify a body-fixed constant search rotation rate
        omega_BR_B = omega_BN_B - this->omega_RN_B;
        eigenVector3d2CArray(omega_BR_B, attGuidanceOutBuffer.omega_BR_B);
        eigenVector3d2CArray(this->omega_RN_B, attGuidanceOutBuffer.omega_RN_B);
    }

    // Write the guidance output message
    this->attGuidanceOutMsg.write(&attGuidanceOutBuffer, moduleID, callTime);
}
