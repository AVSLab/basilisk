/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "fswAlgorithms/effectorInterfaces/vscmgGimbalRateServo/vscmgGimbalRateServo.h"
#include <cstring>
#include <iostream>

/*! Initialize C-wrapped output messages */
void
VscmgGimbalRateServo::SelfInit(){
    VSCMGArrayTorqueMsg_C_init(&this->cmdsOutMsgC);
}

VscmgGimbalRateServo::VscmgGimbalRateServo()
{
    this->K_gammaDot = 0;

    return;
}

/*! Reset the module to original configuration values.

 */
void
VscmgGimbalRateServo::Reset(uint64_t CurrentSimNanos)
{
    if (!this->vsmcgParamsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgGimbalRateServo.vsmcgParamsInMsg was not linked.");
    }
    if (!this->vscmgRefStatesInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgGimbalRateServo.vscmgRefStatesInMsg was not linked.");
    }
    if (!this->attInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgGimbalRateServo.attInMsg was not linked.");
    }
    if (!this->speedsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgGimbalRateServo.speedsInMsg was not linked.");
    }

    this->vscmgConfigParams = this->vsmcgParamsInMsg();
}

void
VscmgGimbalRateServo::UpdateState(uint64_t CurrentSimNanos)
{
    Eigen::Matrix3d BG0;
    Eigen::Matrix3d GG0;
    Eigen::Matrix3d BG;
    Eigen::Vector3d omega_BN_B;
    Eigen::Vector3d gsHat0_B;
    Eigen::Vector3d gtHat0_B;
    Eigen::Vector3d ggHat_B;
    VSCMGRefStatesMsgPayload VSCMGRefStates;
    NavAttMsgPayload hubAttState;
    VSCMGSpeedMsgPayload VSCMGSpeeds;
    double omegas;
    double omegat;
    double gammaDotDot;

    this->outputTorques = this->cmdsOutMsg.zeroMsgPayload;

    VSCMGRefStates = this->vscmgRefStatesInMsg();
    hubAttState = this->attInMsg();

    if (!this->speedsInMsg.isWritten()) {
        for (int i = 0; i < vscmgConfigParams.numVSCMG; i++) {
        VSCMGSpeeds.wheelSpeeds[i] = this->vscmgConfigParams.Omega0List[i];
        VSCMGSpeeds.gimbalAngles[i] = this->vscmgConfigParams.gamma0List[i];
        VSCMGSpeeds.gimbalRates[i] = this->vscmgConfigParams.gammaDot0List[i];
        }
    } else {
        VSCMGSpeeds = this->speedsInMsg();
    }



    omega_BN_B = Eigen::Vector3d(hubAttState.omega_BN_B[0], hubAttState.omega_BN_B[1], hubAttState.omega_BN_B[2]);
    //! - Compute torques for each VSCMG
    for (int VSCMGi = 0; VSCMGi < vscmgConfigParams.numVSCMG; VSCMGi++) {
        gsHat0_B = Eigen::Map<const Eigen::Vector3d>(&vscmgConfigParams.Gs0Matrix_B[3 * VSCMGi]);
        gtHat0_B = Eigen::Map<const Eigen::Vector3d>(&vscmgConfigParams.Gt0Matrix_B[3 * VSCMGi]);
        ggHat_B = Eigen::Map<const Eigen::Vector3d>(&vscmgConfigParams.GgMatrix_B[3 * VSCMGi]);
        BG0.col(0) = gsHat0_B;
        BG0.col(1) = gtHat0_B;
        BG0.col(2) = ggHat_B;
        GG0 = eigenM3(VSCMGSpeeds.gimbalAngles[VSCMGi]);
        BG = BG0 * GG0.transpose();

        gammaDotDot = -this->K_gammaDot * (VSCMGSpeeds.gimbalRates[VSCMGi] - VSCMGRefStates.gimbalRates[VSCMGi]);
        omegas = BG.col(0).transpose() * omega_BN_B;
        omegat = BG.col(1).transpose() * omega_BN_B;

        this->outputTorques.wheelTorque[VSCMGi] =
          vscmgConfigParams.IwsList[VSCMGi] *
          (VSCMGRefStates.wheelAccels[VSCMGi] + VSCMGSpeeds.gimbalRates[VSCMGi] * omegat);
        this->outputTorques.gimbalTorque[VSCMGi] =
          vscmgConfigParams.JgList[VSCMGi] * gammaDotDot -
          (vscmgConfigParams.JsList[VSCMGi] - vscmgConfigParams.JtList[VSCMGi]) * omegas * omegat -
          vscmgConfigParams.IwsList[VSCMGi] * VSCMGSpeeds.wheelSpeeds[VSCMGi] * omegat;
    }

    // Write the output message for the VSCMG torques
    this->cmdsOutMsg.write(&this->outputTorques, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message for the VSCMG torques
    VSCMGArrayTorqueMsg_C_write(&this->outputTorques, &this->cmdsOutMsgC, this->moduleID, CurrentSimNanos);
}

void
VscmgGimbalRateServo::setK_gammaDot(double var)
{
    this->K_gammaDot = var;
}
