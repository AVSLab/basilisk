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
void SunSafePointCpp::Reset(uint64_t callTime) {
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
void SunSafePointCpp::UpdateState(uint64_t callTime) {
    // Zero the attitude guidance output buffer message
    this->attGuidanceOutBuffer = AttGuidMsgPayload();

    // Read the current sun body vector estimate input message
    this->sunDirectionInBuffer = NavAttMsgPayload();
    if (this->sunDirectionInMsg.isWritten()) {
        this->sunDirectionInBuffer = this->sunDirectionInMsg();
    }

    // Determine norm of measured Sun-direction vector
    double sHatNorm = cArray2EigenVector3d(this->sunDirectionInBuffer.vehSunPntBdy).norm();

    // Computing the attitude guidance states sigma_BR and omega_RN_B
    if (this->sunDirectionIsAvailable(sHatNorm)) {
        this->computeAttGuidanceStates(sHatNorm);
    } else {
        v3SetZero(this->attGuidanceOutBuffer.sigma_BR);
    }

    // Compute the hub angular rate error omega_BR_B
    this->computeHubAngularRateError();

    // Write the guidance output message
    eigenVector3d2CArray(this->omega_RN_B, this->attGuidanceOutBuffer.omega_RN_B);
    this->attGuidanceOutMsg.write(&this->attGuidanceOutBuffer, moduleID, callTime);
}

/*! Method for computing the attitude guidance states sigma_BR and omega_RN_B if a valid sun direction vector is available.
 @return void
 @param sHatNorm Norm of measured Sun-direction vector
*/
void SunSafePointCpp::computeAttGuidanceStates(double sHatNorm) {
    double dotProductNormalized = (this->sHatBdyCmd.dot(cArray2EigenVector3d(this->sunDirectionInBuffer.vehSunPntBdy)))
                                   / sHatNorm;
    dotProductNormalized = fabs(dotProductNormalized) > 1.0 ?
    dotProductNormalized/fabs(dotProductNormalized) : dotProductNormalized;
    this->sunAngleErr = safeAcos(dotProductNormalized);

    // Compute the heading error relative to the sun direction vector
    if (this->sunAngleErr < this->smallAngle) {  // Sun heading and desired body axis are essentially aligned. Set attitude error to zero.
        v3SetZero(this->attGuidanceOutBuffer.sigma_BR);
    } else {
         Eigen::Vector3d e_hat;  // Eigen Axis
        if (M_PI - this->sunAngleErr < this->smallAngle) {  // The commanded body vector nearly is opposite the sun heading
            e_hat = this->eHat180_B;
        } else {  // Normal case where sun and commanded body vectors are not aligned
            e_hat = cArray2EigenVector3d(this->sunDirectionInBuffer.vehSunPntBdy).cross(this->sHatBdyCmd);
        }
        this->sunMnvrVec = e_hat / e_hat.norm();
        Eigen::Vector3d v2 = tan(this->sunAngleErr * 0.25) * this->sunMnvrVec;
        eigenVector3d2CArray(v2, this->attGuidanceOutBuffer.sigma_BR);
        MRPswitch(this->attGuidanceOutBuffer.sigma_BR, 1.0, this->attGuidanceOutBuffer.sigma_BR);
    }

    // Rate tracking error is the body rate to bring spacecraft to rest
    this->omega_RN_B = (this->sunAxisSpinRate / sHatNorm) * cArray2EigenVector3d(this->sunDirectionInBuffer.vehSunPntBdy);
}

/*! Method for computing the hub angular rate error omega_BR_B.
 @return void
*/
void SunSafePointCpp::computeHubAngularRateError() {
    // Read the imu guidance input message
    NavAttMsgPayload localImuDataInBuffer = NavAttMsgPayload();
    if (this->imuInMsg.isWritten()) {
        localImuDataInBuffer = this->imuInMsg();
    }

    // Create local copy of hub inertial angular velocity vector in B frame components
    Eigen::Vector3d omega_BN_B = cArray2EigenVector3d(localImuDataInBuffer.omega_BN_B);  // [rad/s]

    // Create local copy of hub angular velocity error in B frame components
    Eigen::Vector3d omega_BR_B = omega_BN_B - this->omega_RN_B;  // [rad/s]

    eigenVector3d2CArray(omega_BR_B, this->attGuidanceOutBuffer.omega_BR_B);
}

/*! Method for determining if a valid sun direction vector is available.
 @return bool
 @param sHatNorm Norm of measured Sun-direction vector
*/
bool SunSafePointCpp::sunDirectionIsAvailable(double sHatNorm) const {
    return sHatNorm > this->minUnitMag;
}

/*! Getter method for the minimally accepted sun body vector norm.
 @return double
*/
double SunSafePointCpp::getMinUnitMag() const {
    return this->minUnitMag;
}

/*! Getter method for the small alignment tolerance angle near 0 or 180 degrees.
 @return double
*/
double SunSafePointCpp::getSmallAngle() const {
    return this->smallAngle;
}

/*! Getter method for the desired constant spin rate about sun heading vector.
 @return double
*/
double SunSafePointCpp::getSunAxisSpinRate() const {
    return this->sunAxisSpinRate;
}

/*! Getter method for the desired body rate vector if no sun direction is available.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &SunSafePointCpp::getOmega_RN_B() const {
    return this->omega_RN_B;
}

/*! Getter method for the desired body vector to point at the sun.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &SunSafePointCpp::getSHatBdyCmd() const {
    return this->sHatBdyCmd;
}

/*! Setter method for the minimally accepted sun body vector norm.
 @return void
 @param minUnitMag The minimally acceptable norm of sun body vector
*/
void SunSafePointCpp::setMinUnitMag(const double minUnitMag) {
    this->minUnitMag = minUnitMag;
}

/*! Setter method for the small alignment tolerance angle near 0 or 180 degrees.
 @return void
 @param smallAngle [rad] An angle value that specifies what is near 0 or 180 degrees
*/
void SunSafePointCpp::setSmallAngle(const double smallAngle) {
    this->smallAngle = smallAngle;
}

/*! Setter method for the desired constant spin rate about sun heading vector.
 @return void
 @param sunAxisSpinRate [rad/s] Desired constant spin rate about sun heading vector
*/
void SunSafePointCpp::setSunAxisSpinRate(const double sunAxisSpinRate) {
    this->sunAxisSpinRate = sunAxisSpinRate;
}

/*! Setter method for the desired body rate vector if no sun direction is available.
 @return void
 @param omega_RN_B [rad/s] Desired body rate vector if no sun direction is available
*/
void SunSafePointCpp::setOmega_RN_B(const Eigen::Vector3d &omega_RN_B) {
    this->omega_RN_B = omega_RN_B;
}

/*! Setter method for the desired body vector to point at the sun.
 @return void
 @param sHatBdyCmd Desired body vector to point at the sun
*/
void SunSafePointCpp::setSHatBdyCmd(const Eigen::Vector3d &sHatBdyCmd) {
    this->sHatBdyCmd = sHatBdyCmd;
}
