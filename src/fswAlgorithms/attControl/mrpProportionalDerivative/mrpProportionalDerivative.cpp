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

#include "mrpProportionalDerivative.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include <string.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpProportionalDerivative::Reset(uint64_t callTime)
{
    // Check if the required input messages are linked
    if (!this->guidInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "mrpProportionalDerivative.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        _bskLog(this->bskLogger, BSK_ERROR, "mrpProportionalDerivative.vehConfigInMsg wasn't connected.");
    }

    // Create VehicleConfigMsgPayload input buffer message
    VehicleConfigMsgPayload vcInMsg;

    // Read the VehicleConfigMsgPayload input message
    vcInMsg = VehicleConfigMsgPayload();
    if (this->vehConfigInMsg.isWritten()) {
        vcInMsg = this->vehConfigInMsg();
        this->ISCPntB_B = cArray2EigenMatrixXd(vcInMsg.ISCPntB_B, 3, 3);
    }
}

/*! This method takes the attitude and rate errors relative to the reference frame, as well as
the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param callTime [ns] Time the method is called
*/
void MrpProportionalDerivative::UpdateState(uint64_t callTime)
{
    // Create the buffer messages
    CmdTorqueBodyMsgPayload controlOutMsg;  // Control output request msg
    AttGuidMsgPayload guidInMsg;  // Guidance input message

    // Zero the output message
    controlOutMsg = CmdTorqueBodyMsgPayload();

    // Read the guidance input message
    guidInMsg = AttGuidMsgPayload();
    if (this->guidInMsg.isWritten()) {
        guidInMsg = this->guidInMsg();
    }

    // Compute hub inertial angular velocity in B-frame components
    Eigen::Vector3d omega_BR_B = cArray2EigenVector3d(guidInMsg.omega_BR_B);
    Eigen::Vector3d omega_RN_B = cArray2EigenVector3d(guidInMsg.omega_RN_B);
    Eigen::Vector3d omega_BN_B = omega_BR_B + omega_RN_B;

    // Compute K*sigma_BR
    Eigen::Vector3d sigma_BR = cArray2EigenVector3d(guidInMsg.sigma_BR);
    Eigen::Vector3d v3_temp1 = this->K * sigma_BR;

    // Compute P*delta_omega
    Eigen::Vector3d v3_temp2 = this->P * omega_BR_B;
    
    // Compute omega_r x [I]omega
    Eigen::Vector3d v3_temp3 = omega_RN_B.cross(this->ISCPntB_B * omega_BN_B);
    
    // Compute [I](d(omega_r)/dt - omega x omega_r)
    Eigen::Vector3d domega_RN_B = cArray2EigenVector3d(guidInMsg.domega_RN_B);
    Eigen::Vector3d v3_temp4 = this->ISCPntB_B * (domega_RN_B - omega_BN_B.cross(omega_RN_B));

    // Compute required attitude control torque vector
    // Lr =  K*sigma_BR + P*delta_omega  - omega_r x [I]omega - [I](d(omega_r)/dt - omega x omega_r) + L
    Eigen::Vector3d Lr = - v3_temp1 - v3_temp2 + v3_temp3 + v3_temp4 - this->knownTorquePntB_B;  // [Nm] Required control torque vector

    // Write the output message
    eigenVector3d2CArray(Lr, controlOutMsg.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&controlOutMsg, moduleID, callTime);
}

/*! Getter method for the derivative gain P.
 @return const double
*/
double MrpProportionalDerivative::getDerivativeGainP() {
    return this->P;
}

/*! Getter method for the known torque about point B.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d &MrpProportionalDerivative::getKnownTorquePntB_B() const {
    return this->knownTorquePntB_B;
}

/*! Getter method for the proportional gain K.
 @return const double
*/
double MrpProportionalDerivative::getProportionalGainK() {
    return this->K;
}

/*! Setter method for the derivative gain P.
 @return void
 @param P [N*m*s] Rate error feedback gain applied
*/
void MrpProportionalDerivative::setDerivativeGainP(double P) {
    this->P = P;
}

/*! Setter method for the known external torque about point B.
 @return void
 @param knownTorquePntB_B [N*m] Known external torque expressed in body frame components
*/
void MrpProportionalDerivative::setKnownTorquePntB_B(Eigen::Vector3d &knownTorquePntB_B) {
    this->knownTorquePntB_B = knownTorquePntB_B;
}

/*! Setter method for the proportional gain K.
 @return void
 @param K [rad/s] Proportional gain applied to MRP errors
*/
void MrpProportionalDerivative::setProportionalGainK(double K) {
    this->K = K;
}
