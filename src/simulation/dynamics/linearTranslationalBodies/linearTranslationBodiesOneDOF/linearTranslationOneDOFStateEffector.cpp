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

LinearTranslationOneDOFStateEffector::LinearTranslationOneDOFStateEffector()
{
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.setZero();
	this->effProps.rEff_CB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();

	this->nameOfRhoState = "linearTranslationRho" + std::to_string(LinearTranslationOneDOFStateEffector::effectorID);
	this->nameOfRhoDotState = "linearTranslationRhoDot" + std::to_string(LinearTranslationOneDOFStateEffector::effectorID);
    this->nameOfInertialPositionProperty = "linearTranslationInertialPosition" + std::to_string(LinearTranslationOneDOFStateEffector::effectorID);
    this->nameOfInertialVelocityProperty = "linearTranslationInertialVelocity" + std::to_string(LinearTranslationOneDOFStateEffector::effectorID);
    this->nameOfInertialAttitudeProperty = "linearTranslationInertialAttitude" + std::to_string(LinearTranslationOneDOFStateEffector::effectorID);
    this->nameOfInertialAngVelocityProperty = "linearTranslationInertialAngVelocity" + std::to_string(LinearTranslationOneDOFStateEffector::effectorID);
    LinearTranslationOneDOFStateEffector::effectorID++;
}

uint64_t LinearTranslationOneDOFStateEffector::effectorID = 1;

LinearTranslationOneDOFStateEffector::~LinearTranslationOneDOFStateEffector()
{
    LinearTranslationOneDOFStateEffector::effectorID = 1;
}

void LinearTranslationOneDOFStateEffector::Reset(uint64_t CurrentClock) {
}

void LinearTranslationOneDOFStateEffector::setMass(double mass) {
    if (mass > 0.0)
        this->mass = mass;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "Mass must be greater than 0.");
    }
}

void LinearTranslationOneDOFStateEffector::setFHat_B(Eigen::Vector3d fHat_B) {
    if (fHat_B.norm() > 0.01) {
        this->fHat_B = fHat_B.normalized();
    }
    else {
        this->bskLogger.bskLog(BSK_ERROR, "Norm of fHat must be greater than 0.");
    }
}

void LinearTranslationOneDOFStateEffector::setK(double k) {
    if (k >= 0.0)
        this->k = k;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "k must be greater than or equal to 0.");
    }
}

void LinearTranslationOneDOFStateEffector::setC(double c) {
    if (c >= 0.0)
        this->c = c;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "c must be greater than or equal to 0.");
    }
}

void LinearTranslationOneDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    // Get access to the hub's states needed for dynamic coupling
    this->hubOmega = statesIn.getStateObject("hubOmega");

    // Get access to properties needed for dynamic coupling (Hub or prescribed)
    this->inertialPositionProperty = statesIn.getPropertyReference(this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->propName_inertialVelocity);
    this->g_N = statesIn.getPropertyReference("g_N");
}

void LinearTranslationOneDOFStateEffector::linkInPrescribedMotionProperties(DynParamManager& properties)
{
    this->prescribedPositionProperty = properties.getPropertyReference(this->propName_prescribedPosition);
    this->prescribedVelocityProperty = properties.getPropertyReference(this->propName_prescribedVelocity);
    this->prescribedAccelerationProperty = properties.getPropertyReference(this->propName_prescribedAcceleration);
    this->prescribedAttitudeProperty = properties.getPropertyReference(this->propName_prescribedAttitude);
    this->prescribedAngVelocityProperty = properties.getPropertyReference(this->propName_prescribedAngVelocity);
    this->prescribedAngAccelerationProperty = properties.getPropertyReference(this->propName_prescribedAngAcceleration);
}

void LinearTranslationOneDOFStateEffector::registerStates(DynParamManager& states)
{
	this->rhoState = states.registerState(1, 1, nameOfRhoState);
    Eigen::MatrixXd rhoInitMatrix(1,1);
    rhoInitMatrix(0,0) = this->rhoInit;
    this->rhoState->setState(rhoInitMatrix);

	this->rhoDotState = states.registerState(1, 1, nameOfRhoDotState);
    Eigen::MatrixXd rhoDotInitMatrix(1,1);
    rhoDotInitMatrix(0,0) = this->rhoDotInit;
    this->rhoDotState->setState(rhoDotInitMatrix);

    registerProperties(states);
}

/*! This method attaches a dynamicEffector
 @param newDynamicEffector the dynamic effector to be attached to the translating body
 @param segment defaults to the only segment for 1DOF (base segment 1) */
void LinearTranslationOneDOFStateEffector::addDynamicEffector(DynamicEffector *newDynamicEffector, int segment)
{
    if (segment != 1) {
        bskLogger.bskLog(BSK_ERROR, "Specifying attachment to a non-existent translating bodies linkage.");
    }

    this->assignStateParamNames<DynamicEffector *>(newDynamicEffector);

    this->dynEffectors.push_back(newDynamicEffector);
}

/*! This method registers the SB inertial properties with the dynamic parameter manager and links
 them into dependent dynamic effectors  */
void LinearTranslationOneDOFStateEffector::registerProperties(DynParamManager& states)
{
    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    this->r_FN_N = states.createProperty(this->nameOfInertialPositionProperty, stateInit);
    this->v_FN_N = states.createProperty(this->nameOfInertialVelocityProperty, stateInit);
    this->sigma_FN = states.createProperty(this->nameOfInertialAttitudeProperty, stateInit);
    this->omega_FN_F = states.createProperty(this->nameOfInertialAngVelocityProperty, stateInit);

    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInProperties(states);
    }
}

void LinearTranslationOneDOFStateEffector::readInputMessages()
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

void LinearTranslationOneDOFStateEffector::writeOutputStateMessages(uint64_t currentSimNanos)
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
        eigenMatrixXd2CArray(*this->sigma_FN, configLogMsg.sigma_BN);
        eigenMatrixXd2CArray(*this->omega_FN_F, configLogMsg.omega_BN_B);
        this->translatingBodyConfigLogOutMsg.write(&configLogMsg, this->moduleID, currentSimNanos);
    }
}

void LinearTranslationOneDOFStateEffector::updateEffectorMassProps(double integTime)
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

void LinearTranslationOneDOFStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr,
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

    computeBackSubContributions(backSubContr, F_g, integTime);
}

void LinearTranslationOneDOFStateEffector::computeBackSubContributions(BackSubMatrices & backSubContr,
                                                                       const Eigen::Vector3d& F_g,
                                                                       double integTime)
{
    // Loop through to collect forces and torques from any connected dynamic effectors
    Eigen::Vector3d attBodyForce_F = Eigen::Vector3d::Zero();
    Eigen::Vector3d attBodyTorquePntF_F = Eigen::Vector3d::Zero();
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        // - Compute the force and torque contributions from the dynamicEffectors
        (*dynIt)->computeForceTorque(integTime, double(0.0));
        attBodyForce_F += (*dynIt)->forceExternal_B;
        attBodyTorquePntF_F += (*dynIt)->torqueExternalPntB_B;
    }

    // There are no contributions if the effector is locked
    if (this->isAxisLocked)
    {
        this->aRho.setZero();
        this->bRho.setZero();
        this->cRho = 0.0;

        backSubContr.vecTrans = this->dcm_FB.transpose() * attBodyForce_F;
        backSubContr.vecRot = this->dcm_FB.transpose() * attBodyTorquePntF_F +
                              eigenTilde(this->r_F0B_B + this->rho * this->fHat_B) *
                              (this->dcm_FB.transpose() * attBodyForce_F);
        return;
    }

    this->aRho = -this->fHat_B.transpose();
    this->bRho = this->fHat_B.transpose() * this->rTilde_FcB_B;
    this->cRho = 1.0 / this->mass * (this->motorForce - this->k * (this->rho - this->rhoRef)
                 - this->c * (this->rhoDot - this->rhoDotRef)
                 + this->fHat_B.transpose() * (F_g + this->dcm_FB.transpose() * attBodyForce_F
                 - this->mass * (2 * this->omegaTilde_BN_B * this->rPrime_FcB_B
                 + this->omegaTilde_BN_B*this->omegaTilde_BN_B*this->r_FcB_B)));

    backSubContr.matrixA = this->mass * this->fHat_B * this->aRho.transpose();
    backSubContr.matrixB = this->mass * this->fHat_B * this->bRho.transpose();
    backSubContr.matrixC = this->mass * this->rTilde_FcB_B * this->fHat_B * this->aRho.transpose();
	backSubContr.matrixD = this->mass * this->rTilde_FcB_B * this->fHat_B * this->bRho.transpose();
	backSubContr.vecTrans = - this->mass * this->cRho * this->fHat_B
                            + this->dcm_FB.transpose() * attBodyForce_F;
	backSubContr.vecRot = - this->mass * this->omegaTilde_BN_B * this->rTilde_FcB_B * this->rPrime_FcB_B
                          - this->mass * this->cRho * this->rTilde_FcB_B * this->fHat_B
                          + this->dcm_FB.transpose() * attBodyTorquePntF_F + eigenTilde(this->r_F0B_B +
                          this->rho * this->fHat_B) * (this->dcm_FB.transpose() * attBodyForce_F);
}

void LinearTranslationOneDOFStateEffector::addPrescribedMotionCouplingContributions(BackSubMatrices & backSubContr) {

    // Access prescribed motion properties
    Eigen::Vector3d r_PB_B = (Eigen::Vector3d)*this->prescribedPositionProperty;
    Eigen::Vector3d rPrime_PB_B = (Eigen::Vector3d)*this->prescribedVelocityProperty;
    Eigen::Vector3d rPrimePrime_PB_B = (Eigen::Vector3d)*this->prescribedAccelerationProperty;
    Eigen::MRPd sigma_PB;
    sigma_PB = (Eigen::Vector3d)*this->prescribedAttitudeProperty;
    Eigen::Vector3d omega_PB_P = (Eigen::Vector3d)*this->prescribedAngVelocityProperty;
    Eigen::Vector3d omegaPrime_PB_P = (Eigen::Vector3d)*this->prescribedAngAccelerationProperty;
    Eigen::Matrix3d dcm_PB = sigma_PB.toRotationMatrix().transpose();

    // Collect hub states
    Eigen::Vector3d omega_BN_B = this->hubOmega->getState();
    Eigen::Vector3d omega_BN_P = dcm_PB * omega_BN_B;

    // Prescribed motion translation coupling contributions
    Eigen::Vector3d fHat_P = this->fHat_B;
    Eigen::Vector3d r_PB_P = dcm_PB * r_PB_B;
    Eigen::Matrix3d rTilde_PB_P = eigenTilde(r_PB_P);
    backSubContr.matrixB += - this->mass * fHat_P * this->aRho.transpose() * rTilde_PB_P;

    Eigen::Matrix3d omegaTilde_PB_P = eigenTilde(omega_PB_P);
    Eigen::Vector3d rPPrime_FcP_P = this->rPrime_FcB_B;
    Eigen::Matrix3d omegaPrimeTilde_PB_P = eigenTilde(omegaPrime_PB_P);
    Eigen::Vector3d r_FcP_P = this->r_FcB_B;
    Eigen::Vector3d rPrimePrime_PB_P = dcm_PB * rPrimePrime_PB_B;
    Eigen::Matrix3d omegaTilde_BN_P = eigenTilde(omega_BN_P);
    Eigen::Vector3d rPrime_PB_P = dcm_PB * rPrime_PB_B;
    Eigen::Vector3d term1 = 2.0 * omegaTilde_PB_P * rPPrime_FcP_P
                            + omegaPrimeTilde_PB_P * r_FcP_P
                            + omegaTilde_PB_P * omegaTilde_PB_P * r_FcP_P
                            + rPrimePrime_PB_P;
    Eigen::Vector3d term2 = rPrimePrime_PB_P + 2.0 * omegaTilde_BN_P * rPrime_PB_P
                            + omegaTilde_BN_P * omegaTilde_BN_P * r_PB_P;
    Eigen::Vector3d term3 = omegaPrime_PB_P + omegaTilde_BN_P * omega_PB_P;
    backSubContr.vecTrans += - this->mass * term1
                             - this->mass * this->aRho.transpose() * term2 * fHat_P
                             - this->mass * this->bRho.transpose() * term3 * fHat_P;

    // Prescribed motion rotation coupling contributions
    backSubContr.matrixC += this->mass * rTilde_PB_P * fHat_P * this->aRho.transpose();

    Eigen::Vector3d r_FcB_P = r_FcP_P + r_PB_P;
    Eigen::Matrix3d rTilde_FcB_P = eigenTilde(r_FcB_P);
    backSubContr.matrixD += + this->mass * rTilde_PB_P * fHat_P * this->bRho.transpose()
                            - this->mass * rTilde_FcB_P * fHat_P * this->aRho.transpose() * rTilde_PB_P;

    Eigen::Matrix3d IPntFc_P = this->IPntFc_B;
    Eigen::Vector3d omega_PN_P = omega_PB_P + omega_BN_P;
    Eigen::Matrix3d omegaTilde_PN_P = eigenTilde(omega_PN_P);
    Eigen::Matrix3d rTilde_FcP_P = eigenTilde(r_FcP_P);

    Eigen::Vector3d vecRotTerm1 = - IPntFc_P * omegaPrime_PB_P - omegaTilde_PN_P * IPntFc_P * omega_PB_P;
    Eigen::Vector3d vecRotTerm2 = - this->mass * rTilde_FcB_P * term1;
    Eigen::Vector3d vecRotTerm3 = - this->mass * (omegaTilde_BN_P * rTilde_PB_P - omegaTilde_PB_P * rTilde_FcP_P) * rPPrime_FcP_P;
                                  - this->mass * omegaTilde_BN_P * rTilde_FcB_P * (omegaTilde_PB_P * r_FcP_P + rPrime_PB_P);
    Eigen::Vector3d vecRotTerm4 = - this->mass * this->cRho * rTilde_PB_P * fHat_P;
    Eigen::Vector3d vecRotTerm5 = - this->mass * rTilde_FcB_P * fHat_P * (this->aRho.transpose() * term2)
            - this->mass * rTilde_FcB_P * fHat_P * (this->bRho.transpose() * term3);
    backSubContr.vecRot += vecRotTerm1
            + vecRotTerm2
            + vecRotTerm3
            + vecRotTerm4
            + vecRotTerm5;
}

void LinearTranslationOneDOFStateEffector::computeDerivatives(double integTime,
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

void LinearTranslationOneDOFStateEffector::updateEnergyMomContributions(double integTime,
                                                                        Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                        double & rotEnergyContr,
                                                                        Eigen::Vector3d omega_BN_B)
{
    // Update omega_BN_B and omega_FN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    Eigen::Vector3d omega_FN_B = this->omega_BN_B;

    // Compute rDot_FcB_B
    Eigen::Vector3d rDot_FcB_B = this->rPrime_FcB_B + this->omegaTilde_BN_B * this->r_FcB_B;

    // Find rotational angular momentum contribution
    rotAngMomPntCContr_B = this->IPntFc_B * omega_FN_B + this->mass * this->r_FcB_B.cross(rDot_FcB_B);

    // Find rotational energy contribution
    rotEnergyContr = 1.0 / 2.0 * omega_FN_B.dot(this->IPntFc_B * omega_FN_B)
                     + 1.0 / 2.0 * this->mass * rDot_FcB_B.dot(rDot_FcB_B)
                     + 1.0 / 2.0 * this->k * (this->rho - this->rhoRef) * (this->rho - this->rhoRef);
}

void LinearTranslationOneDOFStateEffector::computeTranslatingBodyInertialStates()
{
    Eigen::Matrix3d dcm_FN = this->dcm_FB * this->dcm_BN;
    *this->sigma_FN = eigenMRPd2Vector3d(eigenC2MRP(dcm_FN));
    *this->omega_FN_F = this->dcm_FB.transpose() * this->omega_BN_B;

    this->r_FcN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_FcB_B;
    *this->r_FN_N = this->r_FcN_N - dcm_FN.transpose() * this->r_FcF_F;
    Eigen::Vector3d rDot_FcB_B = this->rPrime_FcB_B + this->omegaTilde_BN_B * this->r_FcB_B;
    this->v_FcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * rDot_FcB_B;
    *this->v_FN_N = this->dcm_BN.transpose() * (this->rPrime_FcB_B + this->omegaTilde_BN_B *
                    (this->r_FcB_B - this->dcm_FB.transpose() * this->r_FcF_F));
}

void LinearTranslationOneDOFStateEffector::UpdateState(uint64_t currentSimNanos)
{
    this->readInputMessages();
    this->computeTranslatingBodyInertialStates();
    this->writeOutputStateMessages(currentSimNanos);
}
