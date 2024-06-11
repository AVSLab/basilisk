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

#include "linearTranslationOneDOFStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"

linearTranslationOneDOFStateEffector::linearTranslationOneDOFStateEffector()
{
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.setZero();
	this->effProps.rEff_CB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();

	this->nameOfRhoState = "linearTranslationRho" + std::to_string(linearTranslationOneDOFStateEffector::effectorID);
	this->nameOfRhoDotState = "linearTranslationRhoDot" + std::to_string(linearTranslationOneDOFStateEffector::effectorID);
    linearTranslationOneDOFStateEffector::effectorID++;
}

uint64_t linearTranslationOneDOFStateEffector::effectorID = 1;

linearTranslationOneDOFStateEffector::~linearTranslationOneDOFStateEffector()
{
    linearTranslationOneDOFStateEffector::effectorID = 1;
}

void linearTranslationOneDOFStateEffector::Reset(uint64_t CurrentClock) {
}

void linearTranslationOneDOFStateEffector::setMass(double mass) {
    if (mass > 0.0)
        this->mass = mass;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "Mass must be greater than 0.");
    }
}

void linearTranslationOneDOFStateEffector::setFHat_B(Eigen::Vector3d fHat_B) {
    if (fHat_B.norm() > 0.01) {
        this->fHat_B = fHat_B.normalized();
    }
    else {
        this->bskLogger.bskLog(BSK_ERROR, "Norm of fHat must be greater than 0.");
    }
}

void linearTranslationOneDOFStateEffector::setK(double k) {
    if (k >= 0.0)
        this->k = k;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "k must be greater than or equal to 0.");
    }
}

void linearTranslationOneDOFStateEffector::setC(double c) {
    if (c >= 0.0)
        this->c = c;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "c must be greater than or equal to 0.");
    }
}

void linearTranslationOneDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialVelocity);
    this->g_N = statesIn.getPropertyReference("g_N");
}

void linearTranslationOneDOFStateEffector::registerStates(DynParamManager& states)
{
	this->rhoState = states.registerState(1, 1, nameOfRhoState);
    Eigen::MatrixXd rhoInitMatrix(1,1);
    rhoInitMatrix(0,0) = this->rhoInit;
    this->rhoState->setState(rhoInitMatrix);

	this->rhoDotState = states.registerState(1, 1, nameOfRhoDotState);
    Eigen::MatrixXd rhoDotInitMatrix(1,1);
    rhoDotInitMatrix(0,0) = this->rhoDotInit;
    this->rhoDotState->setState(rhoDotInitMatrix);
}

void linearTranslationOneDOFStateEffector::readInputMessages()
{
    if (this->motorForceInMsg.isLinked() && this->motorForceInMsg.isWritten()) {
        ArrayMotorForceMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorForceInMsg();
        this->motorForce = incomingCmdBuffer.motorForce[0];
    }

    if (this->motorLockInMsg.isLinked() && this->motorLockInMsg.isWritten()) {
        ArrayEffectorLockMsgPayload incomingLockBuffer;
        incomingLockBuffer = this->motorLockInMsg();
        this->isAxisLocked = incomingLockBuffer.effectorLockFlag[0];
    }

    if (this->translatingBodyRefInMsg.isLinked() && this->translatingBodyRefInMsg.isWritten()) {
        LinearTranslationRigidBodyMsgPayload incomingRefBuffer;
        incomingRefBuffer = this->translatingBodyRefInMsg();
        this->rhoRef = incomingRefBuffer.rho;
        this->rhoDotRef = incomingRefBuffer.rhoDot;
    }
}

void linearTranslationOneDOFStateEffector::writeOutputStateMessages(uint64_t currentSimNanos)
{
    if (this->translatingBodyOutMsg.isLinked()) {
        LinearTranslationRigidBodyMsgPayload translatingBodyBuffer;
        translatingBodyBuffer = this->translatingBodyOutMsg.zeroMsgPayload;
        translatingBodyBuffer.rho = this->rho;
        translatingBodyBuffer.rhoDot = this->rhoDot;
        this->translatingBodyOutMsg.write(&translatingBodyBuffer, this->moduleID, currentSimNanos);
    }

    if (this->translatingBodyConfigLogOutMsg.isLinked()) {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->translatingBodyConfigLogOutMsg.zeroMsgPayload;

        // Logging the P frame is the body frame B of that object
        eigenVector3d2CArray(this->r_FcN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_FcN_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_FN, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_FN_F, configLogMsg.omega_BN_B);
        this->translatingBodyConfigLogOutMsg.write(&configLogMsg, this->moduleID, currentSimNanos);
    }
}

void linearTranslationOneDOFStateEffector::updateEffectorMassProps(double integTime)
{
	this->rho = this->rhoState->getState()(0,0);
    this->rhoDot = this->rhoDotState->getState()(0, 0);

    if (this->isAxisLocked)
    {
        Eigen::MatrixXd zeroMatrix = Eigen::MatrixXd::Constant(1, 1, 0.0);
        this->rhoDotState->setState(zeroMatrix);
    }

    this->r_FcF0_B = this->dcm_FB.transpose() * this->r_FcF_F + this->rho * this->fHat_B;
	this->r_FcB_B = this->r_F0B_B + this->r_FcF0_B;
    this->rTilde_FcB_B = eigenTilde(this->r_FcB_B);
    this->rPrime_FcB_B = this->rhoDot * this->fHat_B;
    this->rPrimeTilde_FcB_B = eigenTilde(this->rPrime_FcB_B);
    this->IPntFc_B = this->dcm_FB.transpose() * this->IPntFc_F * this->dcm_FB;

	this->effProps.mEff = this->mass;
	this->effProps.rEff_CB_B = this->r_FcB_B;
    this->effProps.rEffPrime_CB_B = this->rPrime_FcB_B;
	this->effProps.IEffPntB_B = this->IPntFc_B + this->mass * this->rTilde_FcB_B * this->rTilde_FcB_B.transpose();
	this->effProps.IEffPrimePntB_B = -this->mass * (this->rPrimeTilde_FcB_B * this->rTilde_FcB_B
            + this->rTilde_FcB_B * this->rPrimeTilde_FcB_B);
}

void linearTranslationOneDOFStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr,
                                                               Eigen::Vector3d sigma_BN,
                                                               Eigen::Vector3d omega_BN_B,
                                                               Eigen::Vector3d g_N)
{
    Eigen::MRPd sigmaLocal_BN;
    sigmaLocal_BN = sigma_BN;
    this->dcm_BN = sigmaLocal_BN.toRotationMatrix().transpose();
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(omega_BN_B);

    Eigen::Vector3d gLocal_N = *this->g_N;
    Eigen::Vector3d g_B = dcm_BN * gLocal_N;
    Eigen::Vector3d F_g = this->mass * g_B;

    computeBackSubContributions(backSubContr, F_g);
}

void linearTranslationOneDOFStateEffector::computeBackSubContributions(BackSubMatrices & backSubContr,
                                                                       const Eigen::Vector3d& F_g)
{
    // There are no contributions if the effector is locked
    if (this->isAxisLocked)
    {
        this->aRho.setZero();
        this->bRho.setZero();
        this->cRho = 0.0;

        return;
    }

    this->aRho = - this->fHat_B.transpose();
    this->bRho = this->fHat_B.transpose() * this->rTilde_FcB_B;
    this->cRho = 1.0 / this->mass * (this->motorForce - this->k * (this->rho - this->rhoRef)
            - this->c * (this->rhoDot - this->rhoDotRef)
                 + this->fHat_B.transpose() * (F_g - this->mass * (2 * this->omegaTilde_BN_B * this->rPrime_FcB_B
                 + this->omegaTilde_BN_B*this->omegaTilde_BN_B*this->r_FcB_B)));

	backSubContr.matrixA = this->mass * this->fHat_B * this->aRho.transpose();
    backSubContr.matrixB = this->mass * this->fHat_B * this->bRho.transpose();
    backSubContr.matrixC = this->mass * this->rTilde_FcB_B * this->fHat_B * this->aRho.transpose();
	backSubContr.matrixD = this->mass * this->rTilde_FcB_B * this->fHat_B * this->bRho.transpose();
	backSubContr.vecTrans = - this->mass * this->cRho * this->fHat_B;
	backSubContr.vecRot = - this->mass * this->omegaTilde_BN_B * this->rTilde_FcB_B * this->rPrime_FcB_B
            - this->mass * this->cRho * this->rTilde_FcB_B * this->fHat_B;
}

void linearTranslationOneDOFStateEffector::computeDerivatives(double integTime,
                                                              Eigen::Vector3d rDDot_BN_N,
                                                              Eigen::Vector3d omegaDot_BN_B,
                                                              Eigen::Vector3d sigma_BN)
{
	Eigen::MRPd sigmaLocal_BN;
	sigmaLocal_BN = sigma_BN;
	this->dcm_BN = sigmaLocal_BN.toRotationMatrix().transpose();

	Eigen::MatrixXd rhoDDot(1,1);
	Eigen::Vector3d rDDot_BN_B = dcm_BN * rDDot_BN_N;
    rhoDDot(0,0) = this->aRho.dot(rDDot_BN_B) + this->bRho.dot(omegaDot_BN_B) + this->cRho;
	this->rhoDotState->setDerivative(rhoDDot);
    this->rhoState->setDerivative(this->rhoDotState->getState());
}

void linearTranslationOneDOFStateEffector::updateEnergyMomContributions(double integTime,
                                                                        Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                        double & rotEnergyContr,
                                                                        Eigen::Vector3d omega_BN_B)
{
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    Eigen::Vector3d omega_FN_B = this->omega_BN_B;

    Eigen::Vector3d rDotFcB_B = this->rPrime_FcB_B + this->omegaTilde_BN_B * this->r_FcB_B;
    rotAngMomPntCContr_B = this->IPntFc_B * omega_FN_B + this->mass * this->r_FcB_B.cross(rDotFcB_B);
    rotEnergyContr = 1.0 / 2.0 * omega_FN_B.dot(this->IPntFc_B * omega_FN_B)
            + 1.0 / 2.0 * this->mass * rDotFcB_B.dot(rDotFcB_B)
            + 1.0 / 2.0 * this->k * (this->rho - this->rhoRef) * (this->rho - this->rhoRef);
}

void linearTranslationOneDOFStateEffector::computeTranslatingBodyInertialStates()
{
    Eigen::Matrix3d dcm_FN = this->dcm_FB * this->dcm_BN;
    this->sigma_FN = eigenMRPd2Vector3d(eigenC2MRP(dcm_FN));
    this->omega_FN_F = this->dcm_FB.transpose() * this->omega_BN_B;

    this->r_FcN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_FcB_B;
    Eigen::Vector3d rDot_FcB_B = this->rPrime_FcB_B + this->omegaTilde_BN_B * this->r_FcB_B;
    this->v_FcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * rDot_FcB_B;
}

void linearTranslationOneDOFStateEffector::UpdateState(uint64_t currentSimNanos)
{
    this->readInputMessages();
    this->computeTranslatingBodyInertialStates();
    this->writeOutputStateMessages(currentSimNanos);
}
