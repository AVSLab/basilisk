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
    }

    mCopy(vcInMsg.ISCPntB_B, 1, 9, this->ISCPntB_B);
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
    double omega_BN_B[3];
    v3Add(guidInMsg.omega_BR_B, guidInMsg.omega_RN_B, omega_BN_B);

    // Compute K*sigma_BR
    double v3_temp1[3];
    v3Scale(this->K, guidInMsg.sigma_BR, v3_temp1);

    // Compute P*delta_omega
    double v3_temp2[3];
    v3Scale(this->P, guidInMsg.omega_BR_B, v3_temp2);
    
    // Compute omega_r x [I]omega
    double v3_temp3[3];
    m33MultV3(RECAST3X3 this->ISCPntB_B, omega_BN_B, v3_temp3);
    v3Cross(guidInMsg.omega_RN_B, v3_temp3, v3_temp3);
    
    // Compute [I](d(omega_r)/dt - omega x omega_r)
    double v3_temp4[3];
    v3Cross(omega_BN_B, guidInMsg.omega_RN_B, v3_temp4);
    v3Subtract(guidInMsg.domega_RN_B, v3_temp4, v3_temp4);
    m33MultV3(RECAST3X3 this->ISCPntB_B, v3_temp4, v3_temp4);

    // Compute required attitude control torque vector
    // Lr =  K*sigma_BR + P*delta_omega  - omega_r x [I]omega - [I](d(omega_r)/dt - omega x omega_r) + L
    double Lr[3];  // [Nm] Required control torque vector
    v3Add(v3_temp1, v3_temp2, Lr);
    v3Subtract(Lr, v3_temp3, Lr);
    v3Subtract(Lr, v3_temp4, Lr);
    v3Add(this->knownTorquePntB_B, Lr, Lr);
    v3Scale(-1.0, Lr, Lr);

    // Write the output message
    v3Copy(Lr, controlOutMsg.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&controlOutMsg, moduleID, callTime);
}
