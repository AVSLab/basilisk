/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "rotTransTwoDOFStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>

RotTransTwoDOFStateEffector::RotTransTwoDOFStateEffector() {
    this->nameOfThetaState = "rigidBodyTheta" + std::to_string(RotTransTwoDOFStateEffector::effectorID);
    this->nameOfThetaDotState = "rigidBodyThetaDot" + std::to_string(RotTransTwoDOFStateEffector::effectorID);
    this->nameOfRhoState = "rigidBodyRho" + std::to_string(RotTransTwoDOFStateEffector::effectorID);
    this->nameOfRhoDotState = "rigidBodyRhoDot" + std::to_string(RotTransTwoDOFStateEffector::effectorID);
    RotTransTwoDOFStateEffector::effectorID++;
}

uint64_t RotTransTwoDOFStateEffector::effectorID = 1;

void RotTransTwoDOFStateEffector::Reset(uint64_t CurrentClock [[maybe_unused]]) {
}

void RotTransTwoDOFStateEffector::linkInStates(DynParamManager& states) {
    this->inertialPositionProperty = states.getPropertyReference(this->propName_inertialPosition);
    this->inertialVelocityProperty = states.getPropertyReference(this->propName_inertialVelocity);
}

void RotTransTwoDOFStateEffector::registerStates(DynParamManager& statesIn) {
    this->thetaState = statesIn.registerState(1, 1, this->nameOfThetaState);
    this->rhoState = statesIn.registerState(1, 1, this->nameOfRhoState);
    Eigen::MatrixXd betaInitMatrix(2,1);
    betaInitMatrix(0,0) = this->thetaInit;
    betaInitMatrix(1,0) = this->rhoInit;
    this->thetaState->setState(betaInitMatrix.row(0));
    this->rhoState->setState(betaInitMatrix.row(1));

    this->thetaDotState = statesIn.registerState(1, 1, this->nameOfThetaDotState);
    this->rhoDotState = statesIn.registerState(1, 1, this->nameOfRhoDotState);
    Eigen::MatrixXd betaDotInitMatrix(2,1);
    betaDotInitMatrix(0,0) = this->thetaDotInit;
    betaDotInitMatrix(1,0) = this->rhoDotInit;
    this->thetaDotState->setState(betaDotInitMatrix.row(0));
    this->rhoDotState->setState(betaDotInitMatrix.row(1));
}

void RotTransTwoDOFStateEffector::UpdateState(uint64_t CurrentSimNanos) {
    if (this->motorTorqueInMsg.isLinked() && this->motorTorqueInMsg.isWritten()) {
        ArrayMotorTorqueMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorTorqueInMsg();
        this->u = incomingCmdBuffer.motorTorque[0];
    }
    if (this->motorForceInMsg.isLinked() && this->motorForceInMsg.isWritten()) {
        ArrayMotorForceMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorForceInMsg();
        this->f = incomingCmdBuffer.motorForce[0];
    }
    if (this->spinningBodyRefInMsg.isLinked() && this->spinningBodyRefInMsg.isWritten()) {
        HingedRigidBodyMsgPayload incomingRefBuffer;
        incomingRefBuffer = this->spinningBodyRefInMsg();
        this->thetaRef = incomingRefBuffer.theta;
        this->thetaDotRef = incomingRefBuffer.thetaDot;
    }
    if (this->translatingBodyRefInMsg.isLinked() && this->translatingBodyRefInMsg.isWritten()) {
        LinearTranslationRigidBodyMsgPayload incomingRefBuffer;
        incomingRefBuffer = this->translatingBodyRefInMsg();
        this->rhoRef = incomingRefBuffer.rho;
        this->rhoDotRef = incomingRefBuffer.rhoDot;
    }
    this->writeOutputStateMessages(CurrentSimNanos);
}

void RotTransTwoDOFStateEffector::updateEffectorMassProps(double integTime [[maybe_unused]]) {
    this->effProps.mEff = this->mass1 + this->mass2;

    this->theta = this->thetaState->getStateReference()(0, 0);
    this->thetaDot = this->thetaDotState->getStateReference()(0, 0);
    this->rho = this->rhoState->getStateReference()(0, 0);
    this->rhoDot = this->rhoDotState->getStateReference()(0, 0);

    // Compute dcm_BS
    Eigen::Vector3d prv_SS0 = this->theta * this->sHat_S;
    double prv_SS0_array[3];
    eigenVector3d2CArray(prv_SS0, prv_SS0_array);
    double dcm_SS0[3][3];
    PRV2C(prv_SS0_array, dcm_SS0);
    this->dcm_BS = this->dcm_S0B.transpose() * c2DArray2EigenMatrix3d(dcm_SS0).transpose();

    // Compute effProps.rEff_CB_B
    Eigen::Vector3d r_ScS_B = this->dcm_BS * this->r_ScS_S;
    Eigen::Vector3d r_ScB_B = r_ScS_B + this->r_SB_B;
    Eigen::Vector3d r_TcT_S = this->dcm_TS.transpose() * this->r_TcT_T;
    Eigen::Vector3d r_TT0_T = this->rho * this->tHat_T;
    Eigen::Vector3d r_TS_S =  this->dcm_TS.transpose() * r_TT0_T + this->r_T0S_S;
    Eigen::Vector3d r_TcS_S = r_TcT_S + r_TS_S;
    Eigen::Vector3d r_TcS_B = this->dcm_BS * r_TcS_S;
    Eigen::Vector3d r_TcB_B = r_TcS_B + this->r_SB_B;
    Eigen::Vector3d r_GcB_B = (this->mass1 * r_ScB_B + this->mass2 * r_TcB_B) / (this->mass1 + this->mass2);
    this->effProps.rEff_CB_B = r_GcB_B;

    // Compute effProps.rEffPrime_CB_B
    Eigen::Vector3d omega_SB_S = this->thetaDot * this->sHat_S;
    Eigen::Vector3d omega_SB_B = this->dcm_BS * omega_SB_S;
    Eigen::Matrix3d omegaTilde_SB_B = eigenTilde(omega_SB_B);
    Eigen::Vector3d rPrime_ScB_B = omegaTilde_SB_B * r_ScS_B;
    Eigen::Vector3d rSPrime_TS_S = this->rhoDot * this->dcm_TS.transpose() * this->tHat_T;
    Eigen::Vector3d rSPrime_TS_B = this->dcm_BS * rSPrime_TS_S;
    Eigen::Vector3d rPrime_TcB_B = rSPrime_TS_B + omegaTilde_SB_B * r_TcS_B;
    Eigen::Vector3d rPrime_GcB_B = (this->mass1 * rPrime_ScB_B + this->mass2 * rPrime_TcB_B) / (this->mass1 + this->mass2);
    this->effProps.rEffPrime_CB_B = rPrime_GcB_B;

    // Compute effProps.IEffPntB_B
    Eigen::Matrix3d ISPntSc_B = this->dcm_BS * this->ISPntSc_S * this->dcm_BS.transpose();
    Eigen::Matrix3d dcm_BT = this->dcm_BS * this->dcm_TS.transpose();
    Eigen::Matrix3d ITPntTc_B = dcm_BT * this->ITPntTc_T * dcm_BT.transpose();
    Eigen::Matrix3d rTilde_ScB_B = eigenTilde(r_ScB_B);
    Eigen::Matrix3d rTilde_TcB_B = eigenTilde(r_TcB_B);
    Eigen::Matrix3d ISPntB_B = ISPntSc_B - this->mass1 * rTilde_ScB_B * rTilde_ScB_B;
    Eigen::Matrix3d ITPntB_B = ITPntTc_B - this->mass2 * rTilde_TcB_B * rTilde_TcB_B;
    this->effProps.IEffPntB_B =  ISPntB_B + ITPntB_B;

    // Compute effProps.IEffPrimePntB_B
    Eigen::Matrix3d IPrimeSPntSc_B = omegaTilde_SB_B * ISPntSc_B - ISPntSc_B * omegaTilde_SB_B;
    Eigen::Matrix3d IPrimeTPntTc_B = omegaTilde_SB_B * ITPntTc_B - ITPntTc_B * omegaTilde_SB_B;
    Eigen::Matrix3d rPrimeTilde_ScB_B = eigenTilde(rPrime_ScB_B);
    Eigen::Matrix3d rPrimeTilde_TcB_B = eigenTilde(rPrime_TcB_B);
    Eigen::Matrix3d IPrimeSPntB_B = IPrimeSPntSc_B - this->mass1 * (rPrimeTilde_ScB_B * rTilde_ScB_B + rTilde_ScB_B * rPrimeTilde_ScB_B);
    Eigen::Matrix3d IPrimeTPntB_B = IPrimeTPntTc_B - this->mass2 * (rPrimeTilde_TcB_B * rTilde_TcB_B + rTilde_TcB_B * rPrimeTilde_TcB_B);
    this->effProps.IEffPrimePntB_B = IPrimeSPntB_B + IPrimeTPntB_B;
}

void RotTransTwoDOFStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N) {
    this->dcm_BN = sigma_BN.toRotationMatrix().transpose();
    Eigen::Vector3d g_B = this->dcm_BN * g_N;

    Eigen::Vector3d sHat_B = this->dcm_BS * this->sHat_S;
    Eigen::Vector3d tHat_B = this->dcm_BS * (this->dcm_TS.transpose() * this->tHat_T);

    Eigen::Matrix3d ISPntSc_B = this->dcm_BS * this->ISPntSc_S * this->dcm_BS.transpose();
    Eigen::Vector3d r_ScS_B = this->dcm_BS * this->r_ScS_S;
    Eigen::Matrix3d rTilde_ScS_B = eigenTilde(r_ScS_B);
    Eigen::Matrix3d ISPntS_B = ISPntSc_B + this->mass1 * rTilde_ScS_B * rTilde_ScS_B.transpose();

    Eigen::Matrix3d ITPntTc_B = this->dcm_BS * this->dcm_TS.transpose() * this->ITPntTc_T * this->dcm_TS * this->dcm_BS.transpose();
    Eigen::Vector3d r_TcT_B = this->dcm_BS * this->dcm_TS.transpose() * this->r_TcT_T;
    Eigen::Vector3d r_TS_B = this->dcm_BS * this->r_T0S_S + this->rho * tHat_B;
    Eigen::Vector3d r_TcS_B = r_TcT_B + r_TS_B;
    Eigen::Matrix3d rTilde_TcS_B = eigenTilde(r_TcS_B);
    Eigen::Matrix3d ITPntS_B = ITPntTc_B + this->mass2 * rTilde_TcS_B * rTilde_TcS_B.transpose();

    Eigen::Matrix2d MBeta;
    MBeta << sHat_B.transpose() * (ISPntS_B + ITPntS_B) * sHat_B, this->mass2 * sHat_B.transpose() * rTilde_TcS_B * tHat_B,
             - this->mass2 * tHat_B.transpose() * rTilde_TcS_B * sHat_B, this->mass2 * tHat_B.transpose() * tHat_B;

   // Define ABetaStar matrix
    Eigen::Matrix<double, 2, 3> ABetaStar;
    ABetaStar.row(0) = - sHat_B.transpose() * eigenTilde(this->mass1 * r_ScS_B + this->mass2 * r_TcS_B);
    ABetaStar.row(1) = - tHat_B.transpose() * this->mass2 * Eigen::Matrix3d::Identity();

    // Define BBetaStar matrix
    Eigen::Matrix3d rTilde_SB_B = eigenTilde(this->r_SB_B);
    Eigen::Matrix<double, 2, 3> BBetaStar;
    BBetaStar.row(0) = sHat_B.transpose() * (- (ISPntS_B + ITPntS_B) + eigenTilde(this->mass1 * r_ScS_B + this->mass2 * r_TcS_B) * rTilde_SB_B);
    Eigen::Matrix3d rTilde_TcB_B = rTilde_TcS_B + rTilde_SB_B;
    BBetaStar.row(1) = this->mass2 * tHat_B.transpose() * rTilde_TcB_B;

    // Define CBetaStar row 1
    this->omega_BN_B = omega_BN_B;
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    Eigen::Vector3d omega_SB_B = this->thetaDot * sHat_B;
    Eigen::Vector3d cBetaRow1Term1_B = - (ISPntS_B + ITPntS_B) * omegaTilde_BN_B * omega_SB_B;
    Eigen::Vector3d omega_SN_B = omega_SB_B + this->omega_BN_B;
    Eigen::Matrix3d omegaTilde_SN_B = eigenTilde(omega_SN_B);
    Eigen::Vector3d rSPrime_TS_B = this->rhoDot * tHat_B;
    Eigen::Matrix3d rSPrimeTilde_TS_B = eigenTilde(rSPrime_TS_B);
    Eigen::Vector3d cBetaRow1Term2_B = - (omegaTilde_SN_B * (ISPntS_B + ITPntS_B) - this->mass2 * (rSPrimeTilde_TS_B * rTilde_TcS_B + rTilde_TcS_B * rSPrimeTilde_TS_B) ) * omega_SN_B;
    Eigen::Vector3d cBetaRow1Term3_B = - this->mass2 * eigenTilde(omegaTilde_SN_B * r_TcS_B + rSPrime_TS_B) * rSPrime_TS_B;
    Eigen::Vector3d cBetaRow1Term4_B = - this->mass2 * rTilde_TcS_B * omegaTilde_SN_B * rSPrime_TS_B;
    Eigen::Vector3d cBetaRow1Term5_B = - eigenTilde(this->mass1 * r_ScS_B + this->mass2 * r_TcS_B) * omegaTilde_BN_B * omegaTilde_BN_B * this->r_SB_B;
    Eigen::Vector3d cBetaRow1_B = cBetaRow1Term1_B + cBetaRow1Term2_B + cBetaRow1Term3_B + cBetaRow1Term4_B + cBetaRow1Term5_B;

    // Define CBetaStar row 2
    Eigen::Vector3d cBetaRow2Term1_B = - this->mass2 * (eigenTilde(omegaTilde_BN_B * omega_SB_B) + omegaTilde_SN_B * omegaTilde_SN_B) * r_TcS_B;
    Eigen::Vector3d cBetaRow2Term2_B = -2 * this->mass2 * omegaTilde_SN_B * rSPrime_TS_B - this->mass2 * omegaTilde_BN_B * omegaTilde_BN_B * this->r_SB_B;
    Eigen::Vector3d cBetaRow2_B = cBetaRow2Term1_B + cBetaRow2Term2_B;

    // Define CBetaStar vector
    Eigen::Vector3d r_GcS_B = (this->mass1 * r_ScS_B + this->mass2 * r_TcS_B) / (this->mass1 + this->mass2);
    Eigen::Vector3d gravityTorquePntS_B = r_GcS_B.cross((this->mass1 + this->mass2) * g_B);
    Eigen::Vector3d gravityForce_B = this->mass2 * g_B;  // [N]

    Eigen::Vector2d CBetaStar;
    CBetaStar(0,0) = this->u - this->k1 * (this->theta - this->thetaRef)
        - this->c1 * (this->thetaDot - this->thetaDotRef) + sHat_B.dot(gravityTorquePntS_B)
        + sHat_B.dot(cBetaRow1_B);
    CBetaStar(1, 0) = this->f - this->k2 * (this->rho - this->rhoRef)
        - this->c2 * (this->rhoDot - this->rhoDotRef) + tHat_B.dot(gravityForce_B)
        + tHat_B.dot(cBetaRow2_B);

    // Define the ABeta, BBeta and CBeta matrices
    this->ABeta = MBeta.inverse() * ABetaStar;
    this->BBeta = MBeta.inverse() * BBetaStar;
    this->CBeta = MBeta.inverse() * CBetaStar;

    // Backsubstitution contributions
    backSubContr.matrixA = this->mass2 * tHat_B * this->ABeta.row(1) - ( this->mass1 * rTilde_ScS_B + this->mass2 * rTilde_TcS_B) * sHat_B * this->ABeta.row(0);
    backSubContr.matrixB = this->mass2 * tHat_B * this->BBeta.row(1) - ( this->mass1 * rTilde_ScS_B + this->mass2 * rTilde_TcS_B) * sHat_B * this->BBeta.row(0);
    Eigen::Matrix3d omegaTilde_SB_B = eigenTilde(omega_SB_B);
    backSubContr.vecTrans = - this->mass1 * omegaTilde_SB_B * omegaTilde_SB_B * r_ScS_B
        - this->mass2 * (2.0 * omegaTilde_SB_B * rSPrime_TS_B + omegaTilde_SB_B * omegaTilde_SB_B * r_TcS_B)
        - this->mass2 * this->CBeta(1, 0) * tHat_B
        + (this->mass1 * rTilde_ScS_B + this->mass2 * rTilde_TcS_B) * this->CBeta(0, 0) * sHat_B;
    Eigen::Vector3d r_ScB_B = r_ScS_B + this->r_SB_B;
    Eigen::Matrix3d rTilde_ScB_B = eigenTilde(r_ScB_B);
    Eigen::Matrix3d rotTerm = ISPntSc_B + ITPntTc_B - this->mass1 * rTilde_ScB_B * rTilde_ScS_B - this->mass2 * rTilde_TcB_B * rTilde_TcS_B;
    backSubContr.matrixC = this->mass2 * rTilde_TcB_B * tHat_B * this->ABeta.row(1) + rotTerm * sHat_B * this->ABeta.row(0);
    backSubContr.matrixD = this->mass2 * rTilde_TcB_B * tHat_B * this->BBeta.row(1) + rotTerm * sHat_B * this->BBeta.row(0);
    backSubContr.vecRot = - omegaTilde_SN_B * (ISPntSc_B + ITPntTc_B) * omega_SB_B
        - this->mass1 * omegaTilde_BN_B * rTilde_ScB_B * omegaTilde_SB_B * r_ScS_B
        - this->mass2 * omegaTilde_BN_B * rTilde_TcB_B * (omegaTilde_SB_B * r_TcS_B + rSPrime_TS_B)
        - this->mass1 * rTilde_ScB_B * omegaTilde_SB_B * omegaTilde_SB_B * r_ScS_B
        - this->mass2 * rTilde_TcB_B * omegaTilde_SB_B * omegaTilde_SB_B * r_TcS_B
        - 2.0 * this->mass2 * rTilde_TcB_B * omegaTilde_SB_B * rSPrime_TS_B
        - this->mass2 * rTilde_TcB_B * this->CBeta(1, 0) * tHat_B
        - rotTerm * this->CBeta(0, 0) * sHat_B;
}

void RotTransTwoDOFStateEffector::computeDerivatives(double integTime [[maybe_unused]], Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN [[maybe_unused]]) {
    this->dcm_BN = sigma_BN.toRotationMatrix().transpose();

    this->thetaState->setDerivative(this->thetaDotState->getStateReference());
    this->rhoState->setDerivative(this->rhoDotState->getStateReference());

    Eigen::Vector3d rDDot_BN_B = this->dcm_BN * rDDot_BN_N;
    Eigen::Vector2d betaDDot = this->ABeta * rDDot_BN_B + this->BBeta * omegaDot_BN_B + this->CBeta;
    this->thetaDotState->setDerivative(betaDDot.row(0));
    this->rhoDotState->setDerivative(betaDDot.row(1));
}

void RotTransTwoDOFStateEffector::updateEnergyMomContributions(double integTime [[maybe_unused]], Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B) {
    // Update omega_BN_B and omega_SN_B
    Eigen::Vector3d sHat_B = this->dcm_BS * this->sHat_S;
    Eigen::Vector3d omega_SB_B = this->thetaDot * sHat_B;
    this->omega_BN_B = omega_BN_B;
    Eigen::Vector3d omega_SN_B = omega_SB_B + this->omega_BN_B;

    // Compute the rDot terms
    Eigen::Vector3d r_ScS_B = this->dcm_BS * this->r_ScS_S;
    Eigen::Vector3d r_ScB_B = r_ScS_B + this->r_SB_B;
    Eigen::Matrix3d omegaTilde_SB_B = eigenTilde(omega_SB_B);
    Eigen::Vector3d rPrime_ScB_B = omegaTilde_SB_B * r_ScS_B;
    this->rDot_ScB_B = rPrime_ScB_B + this->omega_BN_B.cross(r_ScB_B);

    Eigen::Vector3d r_TcT_S = this->dcm_TS.transpose() * this->r_TcT_T;
    Eigen::Vector3d r_TT0_T = this->rho * this->tHat_T;
    Eigen::Vector3d r_TS_S = this->dcm_TS.transpose() * r_TT0_T + this->r_T0S_S;
    Eigen::Vector3d r_TcS_S = r_TcT_S + r_TS_S;
    Eigen::Vector3d r_TcS_B = this->dcm_BS * r_TcS_S;
    Eigen::Vector3d r_TcB_B = r_TcS_B + this->r_SB_B;

    Eigen::Vector3d tHat_B = this->dcm_BS * this->dcm_TS.transpose() * this->tHat_T;
    Eigen::Vector3d rSPrime_TS_B = this->rhoDot * tHat_B;
    Eigen::Vector3d rPrime_TcB_B = rSPrime_TS_B + omegaTilde_SB_B * r_TcS_B;
    this->rDot_TcB_B = rPrime_TcB_B + this->omega_BN_B.cross(r_TcB_B);

    // Find rotational angular momentum contribution from hub
    Eigen::Matrix3d ISPntSc_B = this->dcm_BS * this->ISPntSc_S * this->dcm_BS.transpose();
    Eigen::Matrix3d ITPntTc_B = this->dcm_BS * this->dcm_TS.transpose() * this->ITPntTc_T * this->dcm_TS * this->dcm_BS.transpose();

    Eigen::Matrix3d rTilde_ScB_B = eigenTilde(r_ScB_B);
    Eigen::Matrix3d rTilde_TcB_B = eigenTilde(r_TcB_B);
    rotAngMomPntCContr_B = ISPntSc_B * omega_SN_B + this->mass1 * rTilde_ScB_B * this->rDot_ScB_B
                         + ITPntTc_B * omega_SN_B + this->mass2 * rTilde_TcB_B * this->rDot_TcB_B;

    // Find rotational energy contribution from the hub
    rotEnergyContr = 1.0 / 2.0 * omega_SN_B.dot(ISPntSc_B * omega_SN_B) + 1.0 / 2.0 * this->mass1 * this->rDot_ScB_B.dot(this->rDot_ScB_B) + 1.0 / 2.0 * this->k1 * (this->theta - this->thetaRef) * (this->theta - this->thetaRef)
                   + 1.0 / 2.0 * omega_SN_B.dot(ITPntTc_B * omega_SN_B) + 1.0 / 2.0 * this->mass2 * this->rDot_TcB_B.dot(this->rDot_TcB_B) + 1.0 / 2.0 * this->k2 * (this->rho - this->rhoRef) * (this->rho - this->rhoRef);
}

void RotTransTwoDOFStateEffector::computeBodyInertialStates() {
    // Compute the inertial attitude
    Eigen::Matrix3d dcm_SN = this->dcm_BS.transpose() * this->dcm_BN;
    Eigen::Matrix3d dcm_BT = this->dcm_BS * this->dcm_TS.transpose();
    Eigen::Matrix3d dcm_TN = dcm_BT.transpose() * this->dcm_BN;
    const Eigen::MRPd sigma_SN = eigenC2MRP(dcm_SN);
    const Eigen::MRPd sigma_TN = eigenC2MRP(dcm_TN);
    this->sigma_SN = sigma_SN.coeffs();
    this->sigma_TN = sigma_TN.coeffs();

    // Convert the angular velocity to the corresponding frame
    Eigen::Vector3d sHat_B = this->dcm_BS * this->sHat_S;
    Eigen::Vector3d omega_SB_B = this->thetaDot * sHat_B;
    Eigen::Vector3d omega_SN_B = omega_SB_B + this->omega_BN_B;

    this->omega_SN_S = this->dcm_BS.transpose() * omega_SN_B;
    this->omega_TN_T = dcm_BT.transpose() * omega_SN_B;

    // Compute the inertial position vectors
    Eigen::Vector3d r_ScS_B = this->dcm_BS * this->r_ScS_S;
    Eigen::Vector3d r_ScB_B = r_ScS_B + this->r_SB_B;
    this->r_ScN_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * r_ScB_B;

    Eigen::Vector3d r_TcT_S = this->dcm_TS.transpose() * this->r_TcT_T;
    Eigen::Vector3d r_TT0_T = this->rho * this->tHat_T;
    Eigen::Vector3d r_TS_S = this->dcm_TS.transpose() * r_TT0_T + this->r_T0S_S;
    Eigen::Vector3d r_TcS_S = r_TcT_S + r_TS_S;
    Eigen::Vector3d r_TcS_B = this->dcm_BS * r_TcS_S;
    Eigen::Vector3d r_TcB_B = r_TcS_B + this->r_SB_B;
    this->r_TcN_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * r_TcB_B;

    // Compute the inertial velocity vectors
    this->v_ScN_N = (Eigen::Vector3d)(*this->inertialVelocityProperty) + this->dcm_BN.transpose() * this->rDot_ScB_B;
    this->v_TcN_N = (Eigen::Vector3d)(*this->inertialVelocityProperty) + this->dcm_BN.transpose() * this->rDot_TcB_B;
}

void RotTransTwoDOFStateEffector::writeOutputStateMessages(uint64_t CurrentClock) {
    this->computeBodyInertialStates();

    HingedRigidBodyMsgPayload spinningBodyBuffer;
    if (this->spinningBodyOutMsg.isLinked()) {
        spinningBodyBuffer = this->spinningBodyOutMsg.zeroMsgPayload;
        spinningBodyBuffer.theta = this->theta;
        spinningBodyBuffer.thetaDot = this->thetaDot;
        this->spinningBodyOutMsg.write(&spinningBodyBuffer, this->moduleID, CurrentClock);
    }
    LinearTranslationRigidBodyMsgPayload translatingBodyBuffer;
    if (this->translatingBodyOutMsg.isLinked()) {
        translatingBodyBuffer = this->translatingBodyOutMsg.zeroMsgPayload;
        translatingBodyBuffer.rho = this->rho;
        translatingBodyBuffer.rhoDot = this->rhoDot;
        this->translatingBodyOutMsg.write(&translatingBodyBuffer, this->moduleID, CurrentClock);
    }
    if (this->bodyConfigLogOutMsgs[0]->isLinked()) {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->bodyConfigLogOutMsgs[0]->zeroMsgPayload;
        eigenVector3d2CArray((this->r_ScN_N), configLogMsg.r_BN_N);
        eigenVector3d2CArray((this->v_ScN_N), configLogMsg.v_BN_N);
        eigenVector3d2CArray((this->sigma_SN), configLogMsg.sigma_BN);
        eigenVector3d2CArray((this->omega_SN_S), configLogMsg.omega_BN_B);
        this->bodyConfigLogOutMsgs[0]->write(&configLogMsg, this->moduleID, CurrentClock);
    }
    if (this->bodyConfigLogOutMsgs[1]->isLinked()) {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->bodyConfigLogOutMsgs[1]->zeroMsgPayload;
        eigenVector3d2CArray(this->r_TcN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_TcN_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_TN, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_TN_T, configLogMsg.omega_BN_B);
        this->bodyConfigLogOutMsgs[1]->write(&configLogMsg, this->moduleID, CurrentClock);
    }
}
