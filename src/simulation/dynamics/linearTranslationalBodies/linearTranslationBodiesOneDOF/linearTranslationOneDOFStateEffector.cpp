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
    // Get access to the hub's states needed for dynamic coupling
    this->hubSigma = statesIn.getStateObject("hubSigma");
    this->hubOmega = statesIn.getStateObject("hubOmega");
    this->hubPosition = statesIn.getStateObject("hubPosition");
    this->hubVelocity = statesIn.getStateObject("hubVelocity");

    this->inertialPositionProperty = statesIn.getPropertyReference(this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->propName_inertialVelocity);
    this->inertialAttitudeProperty = statesIn.getPropertyReference(this->propName_inertialAttitude);
    this->inertialAngVelocityProperty = statesIn.getPropertyReference(this->propName_inertialAngVelocity);
    this->g_N = statesIn.getPropertyReference("g_N");

    if (this->nameOfSpacecraftAttachedTo == "prescribedObject") {
        this->prescribedPositionProperty = statesIn.getPropertyReference(this->propName_prescribedPosition);
        this->prescribedVelocityProperty = statesIn.getPropertyReference(this->propName_prescribedVelocity);
        this->prescribedAccelerationProperty = statesIn.getPropertyReference(this->propName_prescribedAcceleration);
        this->prescribedAttitudeProperty = statesIn.getPropertyReference(this->propName_prescribedAttitude);
        this->prescribedAngVelocityProperty = statesIn.getPropertyReference(this->propName_prescribedAngVelocity);
        this->prescribedAngAccelerationProperty = statesIn.getPropertyReference(this->propName_prescribedAngAcceleration);
    }
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

    if (this->nameOfSpacecraftAttachedTo == "prescribedObject") {
        // Collect hub states
        Eigen::Vector3d omega_bN_b = this->hubOmega->getState();

        // Access prescribed motion properties
        Eigen::Vector3d r_PB_B = (Eigen::Vector3d)*this->prescribedPositionProperty;
        Eigen::Vector3d rPrime_PB_B = (Eigen::Vector3d)*this->prescribedVelocityProperty;
        Eigen::Vector3d rPrimePrime_PB_B = (Eigen::Vector3d)*this->prescribedAccelerationProperty;
        Eigen::MRPd sigma_PB;
        sigma_PB = (Eigen::Vector3d)*this->prescribedAttitudeProperty;
        Eigen::Vector3d omega_PB_P = (Eigen::Vector3d)*this->prescribedAngVelocityProperty;
        Eigen::Vector3d omegaPrime_PB_P = (Eigen::Vector3d)*this->prescribedAngAccelerationProperty;
        Eigen::Matrix3d dcm_PB = sigma_PB.toRotationMatrix().transpose();

        Eigen::Vector3d fHat_b = dcm_PB.transpose() * this->fHat_B;
        Eigen::Vector3d r_FcB_b = dcm_PB.transpose() * this->r_FcB_B;
        Eigen::Matrix3d rTilde_FcB_b = eigenTilde(r_FcB_b);
        Eigen::Vector3d omega_BN_b = dcm_PB.transpose() * this->omega_BN_B;
        Eigen::Matrix3d omegaTilde_BN_b = eigenTilde(omega_BN_b);
        Eigen::Vector3d F_g_b = dcm_PB.transpose() * F_g;
        Eigen::Vector3d rPrime_FcB_b = dcm_PB.transpose() * this->rPrime_FcB_B;

        this->aRho = - fHat_b.transpose();
        this->bRho = fHat_b.transpose() * rTilde_FcB_b;
        this->cRho = 1.0 / this->mass * (this->motorForce - this->k * (this->rho - this->rhoRef)
                                         - this->c * (this->rhoDot - this->rhoDotRef)
                                         + fHat_b.transpose() * (F_g_b
                                         - this->mass * (2 * omegaTilde_BN_b * rPrime_FcB_b + omegaTilde_BN_b * omegaTilde_BN_b * r_FcB_b)));

        backSubContr.matrixA = this->mass * fHat_b * this->aRho.transpose();
        backSubContr.matrixB = this->mass * fHat_b * this->bRho.transpose();
        backSubContr.matrixC = this->mass * rTilde_FcB_b * fHat_b * this->aRho.transpose();
        backSubContr.matrixD = this->mass * rTilde_FcB_b * fHat_b * this->bRho.transpose();
        backSubContr.vecTrans = - this->mass * this->cRho * fHat_b;
        backSubContr.vecRot = - this->mass * omegaTilde_BN_b * rTilde_FcB_b * rPrime_FcB_b
                              - this->mass * this->cRho * rTilde_FcB_b * fHat_b;

        // Prescribed motion coupling contributions
        Eigen::Matrix3d rTilde_PB_B = eigenTilde(r_PB_B);
        Eigen::Vector3d omega_PB_B = dcm_PB.transpose() * omega_PB_P;
        Eigen::Matrix3d omegaTilde_PB_B = eigenTilde(omega_PB_B);
        Eigen::Vector3d rPrime_FcP_B = rPrime_FcB_b;
        Eigen::Vector3d omegaPrime_PB_B = dcm_PB.transpose() * omegaPrime_PB_P;
        Eigen::Matrix3d omegaPrimeTilde_PB_B = eigenTilde(omegaPrime_PB_B);
        Eigen::Matrix3d omegaTilde_bN_b = eigenTilde(omega_bN_b);

        Eigen::Matrix3d IPntFc_b = dcm_PB.transpose() * this->IPntFc_B * dcm_PB;
        Eigen::Vector3d r_Fcb_b = r_FcB_b + r_PB_B;
        Eigen::Matrix3d rTilde_Fcb_b = eigenTilde(r_Fcb_b);

        // Prescribed motion translation coupling contributions
        backSubContr.matrixB += - this->mass * rTilde_PB_B * fHat_b * this->aRho.transpose();
        Eigen::Vector3d term1 = - 2.0 * this->mass * omegaTilde_PB_B * rPrime_FcP_B
                                - this->mass * omegaPrimeTilde_PB_B * r_FcB_b
                                - this->mass * omegaTilde_PB_B * omegaTilde_PB_B * r_FcB_b
                                - this->mass * rPrimePrime_PB_B;
        double term2 = - this->aRho.transpose() * (rPrimePrime_PB_B + 2.0 * omegaTilde_bN_b * rPrime_PB_B + omegaTilde_bN_b * omegaTilde_bN_b * r_PB_B);
        double term3 = this->bRho.transpose() * (omegaPrime_PB_B + omegaTilde_bN_b * omega_PB_B);
        backSubContr.vecTrans += term1 + this->mass * (term2 + term3) * fHat_b;

        // Prescribed motion rotation coupling contributions
        backSubContr.matrixC += this->mass * rTilde_PB_B * fHat_b * this->aRho.transpose();
        backSubContr.matrixD += - this->mass * rTilde_PB_B * fHat_b * this->bRho.transpose()
                                - (this->mass * rTilde_FcB_b * fHat_b * this->aRho.transpose()
                                  - this->mass * rTilde_PB_B * fHat_b * this->aRho.transpose()) * rTilde_PB_B;
        backSubContr.vecRot += - IPntFc_b * omegaPrime_PB_B
                               - omegaTilde_BN_b * IPntFc_b * omega_PB_B
                               - this->mass * rTilde_Fcb_b * (2.0 * omegaTilde_PB_B * rPrime_FcP_B
                                                                + omegaPrimeTilde_PB_B * r_FcB_b + omegaTilde_PB_B * omegaTilde_PB_B * r_FcB_b)
                               + this->mass * omegaTilde_PB_B * rTilde_FcB_b * rPrime_FcP_B
                               - this->mass * omegaTilde_BN_b * rTilde_PB_B * rPrime_FcP_B
                               + this->mass * omegaTilde_PB_B * rTilde_PB_B * rPrime_FcP_B
                               - this->mass * omegaTilde_BN_b * rTilde_Fcb_b * (omegaTilde_PB_B * r_FcB_b + rPrime_PB_B)
                               + this->mass * omegaTilde_PB_B * rTilde_Fcb_b * (omegaTilde_PB_B * r_FcB_b + rPrime_PB_B)
                               - this->mass * this->cRho * rTilde_PB_B * fHat_b
                               - (this->mass * rTilde_FcB_b * fHat_b * this->aRho.transpose() - this->mass * rTilde_PB_B * fHat_b * this->aRho.transpose())
                               * (rPrimePrime_PB_B + 2.0 * omegaTilde_bN_b * rPrime_PB_B + omegaTilde_bN_b * omegaTilde_bN_b * r_PB_B)
                               + (this->mass * rTilde_FcB_b * fHat_b * this->bRho.transpose() + this->mass * rTilde_PB_B * fHat_b * this->bRho.transpose())
                               * (omegaPrime_PB_B + omegaTilde_bN_b * omega_PB_B);
    } else {
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
    if (this->nameOfSpacecraftAttachedTo == "prescribedObject") {
        // Access prescribed motion properties
        Eigen::Vector3d r_PB_B = (Eigen::Vector3d)*this->prescribedPositionProperty;
        Eigen::Vector3d rPrime_PB_B = (Eigen::Vector3d)*this->prescribedVelocityProperty;
        Eigen::MRPd sigma_PB;
        sigma_PB = (Eigen::Vector3d)*this->prescribedAttitudeProperty;
        Eigen::Vector3d omega_PB_P = (Eigen::Vector3d)*this->prescribedAngVelocityProperty;
        Eigen::Matrix3d dcm_PB = sigma_PB.toRotationMatrix().transpose();

        this->omega_BN_B = omega_BN_B; // omega_PN_P
        this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
        Eigen::Vector3d omega_PN_b = dcm_PB.transpose() * this->omega_BN_B;
        Eigen::Vector3d omega_FN_b = omega_PN_b;
        Eigen::Vector3d omega_bn_b = omega_PN_b - dcm_PB.transpose() * omega_PB_P;
        Eigen::Matrix3d omegaTilde_bn_b = eigenTilde(omega_bn_b);
        Eigen::Matrix3d IPntFc_b = dcm_PB.transpose() * this->IPntFc_B * dcm_PB;
        Eigen::Vector3d r_Fcb_b = dcm_PB.transpose() * this->r_FcB_B + r_PB_B;
        Eigen::Matrix3d rTilde_Fcb_b = eigenTilde(r_Fcb_b);

        // Compute rDot_Fcb_b
        Eigen::Vector3d rDot_FcB_B = this->rPrime_FcB_B + this->omegaTilde_BN_B * this->r_FcB_B;
        Eigen::Vector3d rDot_Pb_b = rPrime_PB_B + omegaTilde_bn_b * r_PB_B;
        Eigen::Vector3d rDot_Fcb_b = dcm_PB.transpose() * rDot_FcB_B + rDot_Pb_b;

        rotAngMomPntCContr_B = IPntFc_b * omega_FN_b + this->mass * rTilde_Fcb_b * rDot_Fcb_b;
        rotEnergyContr = 1.0 / 2.0 * omega_FN_b.dot(IPntFc_b * omega_FN_b)
                         + 1.0 / 2.0 * this->mass * rDot_Fcb_b.dot(rDot_Fcb_b)
                         + 1.0 / 2.0 * this->k * (this->rho - this->rhoRef) * (this->rho - this->rhoRef);
    } else {
        this->omega_BN_B = omega_BN_B;
        this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
        Eigen::Vector3d omega_FN_B = this->omega_BN_B;

        Eigen::Vector3d rDotFcB_B = this->rPrime_FcB_B + this->omegaTilde_BN_B * this->r_FcB_B;
        rotAngMomPntCContr_B = this->IPntFc_B * omega_FN_B + this->mass * this->r_FcB_B.cross(rDotFcB_B);
        rotEnergyContr = 1.0 / 2.0 * omega_FN_B.dot(this->IPntFc_B * omega_FN_B)
                         + 1.0 / 2.0 * this->mass * rDotFcB_B.dot(rDotFcB_B)
                         + 1.0 / 2.0 * this->k * (this->rho - this->rhoRef) * (this->rho - this->rhoRef);
    }
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
