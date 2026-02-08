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

#include "fswAlgorithms/effectorInterfaces/vscmgVelocitySteering/vscmgVelocitySteering.h"
#include <cstring>
#include <iostream>

/*! Initialize C-wrapped output messages */
void
VscmgVelocitySteering::SelfInit(){
    VSCMGRefStatesMsg_C_init(&this->vscmgRefStatesOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
VscmgVelocitySteering::VscmgVelocitySteering()
{
    // initialize module variables
    this->mu = 0.0;
    this->W0_s = std::vector<double>(MAX_EFF_CNT, 0.0);
    this->W_g = std::vector<double>(MAX_EFF_CNT, 0.0);
    this->h_bar = std::vector<double>(MAX_EFF_CNT, 0.0);
}

/*! This method is used to reset the module, checks that required input messages are connected, and initializes internal
 * states.
 */
void
VscmgVelocitySteering::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->vscmgParamsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgVelocitySteering.vscmgParamsInMsg was not linked.");
    }
    if (!this->vehControlInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgVelocitySteering.vehControlInMsg was not linked.");
    }
    if (!this->attNavInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgVelocitySteering.attNavInMsg was not linked.");
    }
    if (!this->attGuideInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgVelocitySteering.attGuideInMsg was not linked.");
    }
    if (!this->speedsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "VscmgVelocitySteering.speedsInMsg was not linked.");
    }

    this->vscmgConfigParams = this->vscmgParamsInMsg();

    for (int i = 0; i < vscmgConfigParams.numVSCMG; i++) {
        this->h_bar[i] = vscmgConfigParams.JsList[i] * vscmgConfigParams.Omega0List[i];
    }
    Eigen::VectorXd h_bar_vec = Eigen::Map<Eigen::VectorXd>(this->h_bar.data(), this->h_bar.size());
    double mean_h_bar = 0.0;
    for (int i = 0; i < vscmgConfigParams.numVSCMG; i++) {
        mean_h_bar += h_bar_vec[i];
    }
    mean_h_bar /= vscmgConfigParams.numVSCMG;
    this->h_bar_squared = mean_h_bar * mean_h_bar;
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
 */
void
VscmgVelocitySteering::UpdateState(uint64_t CurrentSimNanos)
{
    double omegas;                                          /*![rad/s] projection of body angular velocity onto gsHat axis*/
    double omegat;                                          /*![rad/s] projection of body angular velocity onto gtHat axis*/
    double delta;                                           /*![-] proximity to CMG singularity*/
    double W_si;                                            /*![-] RW mode weight*/
    Eigen::Vector3d omega_BN_B;                             /*![rad/s] angular velocity of the body frame relative to the inertial frame*/
    Eigen::Vector3d omega_RN_B;                             /*![rad/s] angular velocity of the reference frame relative to the inertial frame*/
    Eigen::Vector3d gsHat0_B;                               /*![-] first gimbal unit axis when rotation angle is 0*/
    Eigen::Vector3d gtHat0_B;                               /*![-] second gimbal unit axis when rotation angle is 0*/
    Eigen::Vector3d ggHat_B;                                /*![-] third gimbal unit axis*/
    Eigen::VectorXd etaDot(2 * vscmgConfigParams.numVSCMG); /*![rad/s^2] and [rad/s] desired RW accelerations and gimbal rates*/
    Eigen::Matrix3d BG0;                                    /*![-] DCM between body frame and gimbal frame when rotation angle is zero */
    Eigen::Matrix3d GG0;                                    /*![-] DCM between gimbal frame and gimbal frame when rotation angle is zero */
    Eigen::Matrix3d BG;                                     /*![-] DCM between body frame and gimbal frame */
    Eigen::Matrix3d QWQT;
    Eigen::MatrixXd D0(3, vscmgConfigParams.numVSCMG);
    Eigen::MatrixXd D1(3, vscmgConfigParams.numVSCMG);
    Eigen::MatrixXd D2(3, vscmgConfigParams.numVSCMG);
    Eigen::MatrixXd D3(3, vscmgConfigParams.numVSCMG);
    Eigen::MatrixXd D4(3, vscmgConfigParams.numVSCMG);
    Eigen::MatrixXd D(3, vscmgConfigParams.numVSCMG);
    Eigen::MatrixXd Q(3, 2 * vscmgConfigParams.numVSCMG);
    Eigen::MatrixXd W(2 * vscmgConfigParams.numVSCMG, 2 * vscmgConfigParams.numVSCMG);
    CmdTorqueBodyMsgPayload Lr;
    NavAttMsgPayload hubAttState;
    AttGuidMsgPayload hubGuideState;
    VSCMGSpeedMsgPayload VSCMGSpeeds;

    D0.setZero();
    D1.setZero();
    D2.setZero();
    D3.setZero();
    D4.setZero();
    D.setZero();
    Q.setZero();
    W.setZero();

    this->outputRefStates = this->vscmgRefStatesOutMsg.zeroMsgPayload;
    Lr = this->vehControlInMsg();
    hubAttState = this->attNavInMsg();
    hubGuideState = this->attGuideInMsg();

    if (!this->speedsInMsg.isWritten()) {
        for (int i = 0; i < vscmgConfigParams.numVSCMG; i++) {
            VSCMGSpeeds.wheelSpeeds[i] = vscmgConfigParams.Omega0List[i];
            VSCMGSpeeds.gimbalAngles[i] = vscmgConfigParams.gamma0List[i];
            VSCMGSpeeds.gimbalRates[i] = vscmgConfigParams.gammaDot0List[i];
        }
    } else {
        VSCMGSpeeds = this->speedsInMsg();
    }

    omega_BN_B = Eigen::Vector3d(hubAttState.omega_BN_B[0], hubAttState.omega_BN_B[1], hubAttState.omega_BN_B[2]);
    omega_RN_B = Eigen::Vector3d(hubGuideState.omega_RN_B[0], hubGuideState.omega_RN_B[1], hubGuideState.omega_RN_B[2]);

    for (int i = 0; i < vscmgConfigParams.numVSCMG; i++) {
        gsHat0_B = Eigen::Map<const Eigen::Vector3d>(&vscmgConfigParams.Gs0Matrix_B[3 * i]);
        gtHat0_B = Eigen::Map<const Eigen::Vector3d>(&vscmgConfigParams.Gt0Matrix_B[3 * i]);
        ggHat_B = Eigen::Map<const Eigen::Vector3d>(&vscmgConfigParams.GgMatrix_B[3 * i]);

        BG0.col(0) = gsHat0_B;
        BG0.col(1) = gtHat0_B;
        BG0.col(2) = ggHat_B;

        GG0 = eigenM3(VSCMGSpeeds.gimbalAngles[i]);
        BG = BG0 * GG0.transpose();

        omegas = BG.col(0).transpose() * omega_BN_B;
        omegat = BG.col(1).transpose() * omega_BN_B;

        D0.col(i) = BG.col(0) * vscmgConfigParams.IwsList[i];
        D1.col(i) =
          (vscmgConfigParams.IwsList[i] * VSCMGSpeeds.wheelSpeeds[i] + 0.5 * vscmgConfigParams.JsList[i] * omegas) *
            BG.col(1) +
          0.5 * vscmgConfigParams.JsList[i] * omegat * BG.col(0);
        D2.col(i) = 0.5 * vscmgConfigParams.JtList[i] * (omegat * BG.col(0) + omegas * BG.col(1));
        D3.col(i) = vscmgConfigParams.JgList[i] * (omegat * BG.col(0) - omegas * BG.col(1));
        D4.col(i) = 0.5 * (vscmgConfigParams.JsList[i] - vscmgConfigParams.JtList[i]) *
                    (BG.col(0) * BG.col(1).transpose() * omega_RN_B + BG.col(1) * BG.col(0).transpose() * omega_RN_B);
    }

    D = D1 - D2 + D3 + D4;

    Q.block(0, 0, 3, vscmgConfigParams.numVSCMG) = D0;
    Q.block(0, vscmgConfigParams.numVSCMG, 3, vscmgConfigParams.numVSCMG) = D;

    delta = ((1.0 / this->h_bar_squared) * (D1 * D1.transpose())).determinant();

    for (int i = 0; i < vscmgConfigParams.numVSCMG; i++) {
        W_si = this->W0_s[i] * exp(-this->mu * delta);
        W(i, i) = W_si;
        W(i + vscmgConfigParams.numVSCMG, i + vscmgConfigParams.numVSCMG) = this->W_g[i];
    }

    Eigen::Vector3d torqueVec = Eigen::Map<const Eigen::Vector3d>(Lr.torqueRequestBody);
    QWQT = Q * W * Q.transpose();
    etaDot = W * Q.transpose() * QWQT.ldlt().solve(-torqueVec);

    for (int i = 0; i < vscmgConfigParams.numVSCMG; i++) {
        this->outputRefStates.wheelAccels[i] = etaDot(i);
        this->outputRefStates.gimbalRates[i] = etaDot(i + vscmgConfigParams.numVSCMG);
    }

    // Write the output message for the VSCMG reference states
    this->vscmgRefStatesOutMsg.write(&outputRefStates, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message for the VSCMG torques
    VSCMGRefStatesMsg_C_write(&this->outputRefStates, &this->vscmgRefStatesOutMsgC, this->moduleID, CurrentSimNanos);
}

void
VscmgVelocitySteering::setMu(double var)
{
    this->mu = var;
}

void
VscmgVelocitySteering::setW0_s(std::vector<double> var)
{
    for (size_t i = 0; i < var.size() && i < MAX_EFF_CNT; ++i) {
        this->W0_s[i] = var[i];
    }
}

void
VscmgVelocitySteering::setW_g(std::vector<double> var)
{
    for (size_t i = 0; i < var.size() && i < MAX_EFF_CNT; ++i) {
        this->W_g[i] = var[i];
    }
}
