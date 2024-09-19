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

#include "linearTranslationNDOFStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>

/*! This is the constructor, setting variables to default values */
linearTranslationNDOFStateEffector::linearTranslationNDOFStateEffector()
{
    // Zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    this->nameOfRhoState = "translatingBodyRho" + std::to_string(linearTranslationNDOFStateEffector::effectorID);
    this->nameOfRhoDotState = "translatingBodyRhoDot" + std::to_string(linearTranslationNDOFStateEffector::effectorID);
    linearTranslationNDOFStateEffector::effectorID++;
}

uint64_t linearTranslationNDOFStateEffector::effectorID = 1;

/*! This is the destructor, nothing to report here */
linearTranslationNDOFStateEffector::~linearTranslationNDOFStateEffector()
{
    linearTranslationNDOFStateEffector::effectorID --;    /* reset the panel ID*/
}

void translatingBody::setMass(double mass) {
    if (mass > 0.0)
        this->mass = mass;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "Mass must be greater than 0.");
    }
}

void translatingBody::setFHat_P(Eigen::Vector3d fHat_P) {
    if (fHat_P.norm() > 0.01) {
        this->fHat_P = fHat_P.normalized();
    }
    else {
        this->bskLogger.bskLog(BSK_ERROR, "Norm of fHat must be greater than 0.");
    }
}

void translatingBody::setK(double k) {
    if (k >= 0.0)
        this->k = k;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "k must be greater than or equal to 0.");
    }
}

void translatingBody::setC(double c) {
    if (c >= 0.0)
        this->c = c;
    else {
        this->bskLogger.bskLog(BSK_ERROR, "c must be greater than or equal to 0.");
    }
}

/*! This method is used to reset the module. */
void linearTranslationNDOFStateEffector::Reset(uint64_t CurrentClock)
{
    for(auto& translatingBody: this->translatingBodyVec) {
        if (translatingBody.fHat_P.norm() > 0.0) {
            translatingBody.fHat_P.normalize();
        }
        else {
            bskLogger.bskLog(BSK_ERROR, "Norm of fHat must be greater than 0. sHat may not have been set by the user.");
        }
    }
}

/*! This method is used to add a translating body. */
void linearTranslationNDOFStateEffector::addTranslatingBody(const translatingBody& newBody) {
    // Pushback new body
    translatingBodyVec.push_back(newBody);
    this->N++;

    // Create the output vectors
    this->translatingBodyConfigLogOutMsgs.push_back(new Message<SCStatesMsgPayload>);
    this->translatingBodyOutMsgs.push_back(new Message<LinearTranslationRigidBodyMsgPayload>);
    this->translatingBodyRefInMsgs.push_back(ReadFunctor<LinearTranslationRigidBodyMsgPayload>());

    // resize A B and C
    this->ARho.conservativeResize(this->ARho.rows()+1, 3);
    this->BRho.conservativeResize(this->BRho.rows()+1, 3);
    this->CRho.conservativeResize(this->CRho.rows()+1);
}

/*! This method reads motor force, lock flag, and reference state messages. */
void linearTranslationNDOFStateEffector::readInputMessages()
{
    //! - Read the incoming command array
    if (this->motorForceInMsg.isLinked() && this->motorForceInMsg.isWritten()) {
        ArrayMotorForceMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorForceInMsg();
        int i = 0;
        for(auto& translatingBody: this->translatingBodyVec) {
            translatingBody.u = incomingCmdBuffer.motorForce[i];
            i++;
        }
    }

    //! - Zero the command buffer and read the incoming command array
    if (this->motorLockInMsg.isLinked() && this->motorLockInMsg.isWritten()) {
        ArrayEffectorLockMsgPayload incomingLockBuffer;
        incomingLockBuffer = this->motorLockInMsg();
        int i = 0;
        for(auto& translatingBody: this->translatingBodyVec) {
            translatingBody.isAxisLocked = incomingLockBuffer.effectorLockFlag[i];
            i++;
        }
    }

    int translatingBodyIndex = 0;
    for(auto& translatingBody: this->translatingBodyVec) {
        if (this->translatingBodyRefInMsgs[translatingBodyIndex].isLinked() && this->translatingBodyRefInMsgs[translatingBodyIndex].isWritten()) {
            LinearTranslationRigidBodyMsgPayload incomingRefBuffer;
            incomingRefBuffer = this->translatingBodyRefInMsgs[translatingBodyIndex]();
            translatingBody.rhoRef = incomingRefBuffer.rho;
            translatingBody.rhoDotRef = incomingRefBuffer.rhoDot;
        }
        translatingBodyIndex++;
    }
}

/*! This method takes the computed rho states and outputs them to the messaging system. */
void linearTranslationNDOFStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    // Write out the translating body output messages
    int i = 0;
    LinearTranslationRigidBodyMsgPayload translatingBodyBuffer;
    SCStatesMsgPayload configLogMsg;
    for(auto& translatingBody: this->translatingBodyVec) {
        if (this->translatingBodyOutMsgs[i]->isLinked()) {
            translatingBodyBuffer = this->translatingBodyOutMsgs[i]->zeroMsgPayload;
            translatingBodyBuffer.rho = translatingBody.rho;
            translatingBodyBuffer.rhoDot = translatingBody.rhoDot;
            this->translatingBodyOutMsgs[i]->write(&translatingBodyBuffer, this->moduleID, CurrentClock);
        }

        if (this->translatingBodyConfigLogOutMsgs[i]->isLinked()) {
            configLogMsg = this->translatingBodyConfigLogOutMsgs[i]->zeroMsgPayload;

            // Logging the F frame is the body frame B of that object
            eigenVector3d2CArray(translatingBody.r_FcN_N, configLogMsg.r_BN_N);
            eigenVector3d2CArray(translatingBody.v_FcN_N, configLogMsg.v_BN_N);
            eigenVector3d2CArray(translatingBody.sigma_FN, configLogMsg.sigma_BN);
            eigenVector3d2CArray(translatingBody.omega_FN_F, configLogMsg.omega_BN_B);
            this->translatingBodyConfigLogOutMsgs[i]->write(&configLogMsg, this->moduleID, CurrentClock);
        }

        i++;
    }
}

/*! This method prepends the name of the spacecraft for multi-spacecraft simulations.*/
void linearTranslationNDOFStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfRhoState = this->nameOfSpacecraftAttachedTo + this->nameOfRhoState;
    this->nameOfRhoDotState = this->nameOfSpacecraftAttachedTo + this->nameOfRhoDotState;
}

/*! This method allows the TB state effector to have access to the hub states and gravity*/
void linearTranslationNDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "v_BN_N");
}

/*! This method allows the TB state effector to register its states: rho and rhoDot with the dynamic parameter manager */
void linearTranslationNDOFStateEffector::registerStates(DynParamManager& states)
{
    // Register the rho states
    this->rhoState = states.registerState(N, 1, this->nameOfRhoState);
    this->rhoDotState = states.registerState(N, 1, this->nameOfRhoDotState);
    Eigen::MatrixXd RhoInitMatrix(N,1);
    Eigen::MatrixXd RhoDotInitMatrix(N,1);
    int i = 0;
    for(const auto& translatingBody: this->translatingBodyVec) {
        RhoInitMatrix(i,0) = translatingBody.rhoInit;
        RhoDotInitMatrix(i,0) = translatingBody.rhoDotInit;
        i++;
    }
    this->rhoState->setState(RhoInitMatrix);
    this->rhoDotState->setState(RhoDotInitMatrix);
}

/*! This method allows the TB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void linearTranslationNDOFStateEffector::updateEffectorMassProps(double integTime)
{
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B = Eigen::Vector3d::Zero();
    this->effProps.rEffPrime_CB_B = Eigen::Vector3d::Zero();
    this->effProps.IEffPntB_B = Eigen::Matrix3d::Zero();
    this->effProps.IEffPrimePntB_B = Eigen::Matrix3d::Zero();

    int i = 0;
    for(auto& translatingBody: this->translatingBodyVec) {
        if (translatingBody.isAxisLocked) {
            auto rhoDotVector = this->rhoDotState->getState();
            rhoDotVector(i) = 0.0;
            this->rhoDotState->setState(rhoDotVector);
        }
        // Give the mass of the translating body to the effProps mass
        this->effProps.mEff += translatingBody.mass;

        // Grab current states
        translatingBody.rho = this->rhoState->getState()(i, 0);
        translatingBody.rhoDot = this->rhoDotState->getState()(i, 0);

        // Write the translating axis in B frame
        if (i == 0) {
            translatingBody.dcm_FB = translatingBody.dcm_FP;
            translatingBody.fHat_B = translatingBody.fHat_P;
        } else {
            translatingBody.dcm_FB = translatingBody.dcm_FP * this->translatingBodyVec[i-1].dcm_FB;
            translatingBody.fHat_B = this->translatingBodyVec[i-1].dcm_FB.transpose() * translatingBody.fHat_P;
        }
        translatingBody.r_FF0_B = translatingBody.rho * translatingBody.fHat_B;

        // Compute the effector's CoM with respect to point B
        translatingBody.r_FcF_B = translatingBody.dcm_FB.transpose() * translatingBody.r_FcF_F;
        if (i == 0) {
            // The parent frame of first body is the B frame
            translatingBody.r_F0P_B = translatingBody.r_F0P_P;
            translatingBody.r_FP_B = translatingBody.r_F0P_B + translatingBody.r_FF0_B;
            translatingBody.r_FB_B = translatingBody.r_FP_B;
        } else {
            translatingBody.r_F0P_B = this->translatingBodyVec[i-1].dcm_FB.transpose() * translatingBody.r_F0P_P;
            translatingBody.r_FP_B = translatingBody.r_F0P_B + translatingBody.r_FF0_B;
            translatingBody.r_FB_B = translatingBody.r_FP_B + this->translatingBodyVec[i-1].r_FB_B;
        }
        translatingBody.r_FcB_B = translatingBody.r_FcF_B + translatingBody.r_FB_B;
        this->effProps.rEff_CB_B += translatingBody.mass * translatingBody.r_FcB_B;

        // Find the inertia of the bodies about point B
        translatingBody.rTilde_FcB_B = eigenTilde(translatingBody.r_FcB_B);
        translatingBody.IPntFc_B = translatingBody.dcm_FB.transpose() * translatingBody.IPntFc_F * translatingBody.dcm_FB;
        this->effProps.IEffPntB_B += translatingBody.IPntFc_B - translatingBody.mass * translatingBody.rTilde_FcB_B * translatingBody.rTilde_FcB_B;

        // Find rPrime_FcB_B
        translatingBody.rPrime_FcF_B = Eigen::Vector3d::Zero();
        translatingBody.rPrime_FF0_B = translatingBody.rhoDot * translatingBody.fHat_B;
        translatingBody.rPrime_FP_B = translatingBody.rPrime_FF0_B;
        if (i == 0) {
            translatingBody.rPrime_FB_B = translatingBody.rPrime_FP_B;
        } else {
            translatingBody.rPrime_FB_B = translatingBody.rPrime_FP_B + this->translatingBodyVec[i-1].rPrime_FB_B;
        }
        translatingBody.rPrime_FcB_B = translatingBody.rPrime_FcF_B + translatingBody.rPrime_FB_B;
        this->effProps.rEffPrime_CB_B += translatingBody.mass * translatingBody.rPrime_FcB_B;

        // Find the body-frame time derivative of the inertia of each arm and the entire spacecraft
        translatingBody.IPrimePntFc_B = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d rPrimeTilde_FcB_B = eigenTilde(translatingBody.rPrime_FcB_B);
        this->effProps.IEffPrimePntB_B += translatingBody.IPrimePntFc_B - translatingBody.mass * (rPrimeTilde_FcB_B * translatingBody.rTilde_FcB_B + translatingBody.rTilde_FcB_B * rPrimeTilde_FcB_B);

        i++;
    }
    this->effProps.rEff_CB_B /= this->effProps.mEff;
    this->effProps.rEffPrime_CB_B /= this->effProps.mEff;
}

/*! This method allows the TB state effector to give its contributions to the matrices needed for the back-sub
 method */
void linearTranslationNDOFStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // Find the DCM from N to B frames
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
    this->omega_BN_B = omega_BN_B;

    Eigen::MatrixXd MRho = Eigen::MatrixXd::Zero(this->N, this->N);
    Eigen::MatrixXd ARhoStar = Eigen::MatrixXd::Zero(this->N, 3);
    Eigen::MatrixXd BRhoStar = Eigen::MatrixXd::Zero(this->N, 3);
    Eigen::VectorXd CRhoStar = Eigen::VectorXd::Zero(this->N);

    this->computeMRho(MRho);
    this->computeARhoStar(ARhoStar);
    this->computeBRhoStar(BRhoStar);
    this->computeCRhoStar(CRhoStar, g_N);

    this->ARho = MRho.inverse() * ARhoStar;
    this->BRho = MRho.inverse() * BRhoStar;
    this->CRho = MRho.inverse() * CRhoStar;

    this->computeBackSubContributions(backSubContr);
}

/*! This method compute MRho for back-sub */
void linearTranslationNDOFStateEffector::computeMRho(Eigen::MatrixXd& MRho)
{
    for (int n = 0; n<this->N; n++) {
        for (int i = 0; i<this->N; i++) {
            MRho(n,i) = 0.0;
            if ((this->translatingBodyVec[n].isAxisLocked || this->translatingBodyVec[i].isAxisLocked) && n != i)
                continue;
            for (int j = (i<=n) ? n : i; j<this->N; j++) {
                MRho(n,i) += this->translatingBodyVec[n].fHat_B.transpose() * this->translatingBodyVec[j].mass
                    * this->translatingBodyVec[i].fHat_B;
            }
        }
    }
}

/*! This method compute ARhoStar for back-sub */
void linearTranslationNDOFStateEffector::computeARhoStar(Eigen::MatrixXd& ARhoStar)
{
    for (int n = 0; n<this->N; n++) {
        if (this->translatingBodyVec[n].isAxisLocked)
            continue;
        for (int i = n; i<this->N; i++) {
            ARhoStar.row(n) -= this->translatingBodyVec[n].fHat_B.transpose() * this->translatingBodyVec[i].mass;
        }
    }
}

/*! This method compute BRhoStar for back-sub */
void linearTranslationNDOFStateEffector::computeBRhoStar(Eigen::MatrixXd& BRhoStar)
{
    for (int n = 0; n<this->N; n++) {
        if (this->translatingBodyVec[n].isAxisLocked)
            continue;
        for (int i = n; i<this->N; i++) {
            Eigen::Vector3d r_FciB_B = this->translatingBodyVec[i].r_FcB_B;
            Eigen::Matrix3d rTilde_FciB_B = eigenTilde(r_FciB_B);

            BRhoStar.row(n) += this->translatingBodyVec[n].fHat_B.transpose() * this->translatingBodyVec[i].mass * rTilde_FciB_B;
        }
    }
}

/*! This method compute CRhoStar for back-sub */
void linearTranslationNDOFStateEffector::computeCRhoStar(Eigen::VectorXd& CRhoStar,
                                                      const Eigen::Vector3d& g_N)
{
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    // Map gravity to body frame
    Eigen::Vector3d g_B;
    g_B = this->dcm_BN * g_N;
    Eigen::Vector3d F_g = Eigen::Vector3d::Zero().transpose();

    for (int n = 0; n<this->N; n++) {
        if (this->translatingBodyVec[n].isAxisLocked)
            continue;
        CRhoStar(n, 0) = this->translatingBodyVec[n].u
                           - this->translatingBodyVec[n].k * (this->translatingBodyVec[n].rho -
                           this->translatingBodyVec[n].rhoRef) - this->translatingBodyVec[n].c *
                           (this->translatingBodyVec[n].rhoDot - this->translatingBodyVec[n].rhoDotRef);
        for (int i = n; i<this->N; i++) {
            Eigen::Vector3d r_FciB_B = this->translatingBodyVec[i].r_FcB_B;
            Eigen::Vector3d rPrime_FciB_B = this->translatingBodyVec[i].rPrime_FcB_B;

            F_g = this->translatingBodyVec[i].mass * g_B;
            CRhoStar(n, 0) += this->translatingBodyVec[n].fHat_B.transpose() * (F_g - this->translatingBodyVec[i].mass *
                              (omegaTilde_BN_B * omegaTilde_BN_B * r_FciB_B + 2 * omegaTilde_BN_B * rPrime_FciB_B));
        }
    }
}

/*! This method computes the back-sub contributions of the system */
void linearTranslationNDOFStateEffector::computeBackSubContributions(BackSubMatrices& backSubContr) const
{
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    for (int i = 0; i<this->N; i++) {
    Eigen::Matrix3d rTilde_FciB_B = eigenTilde(this->translatingBodyVec[i].r_FcB_B);
    Eigen::Vector3d rPrime_FciB_B = this->translatingBodyVec[i].rPrime_FcB_B;
    backSubContr.vecRot -= this->translatingBodyVec[i].mass * omegaTilde_BN_B * rTilde_FciB_B * rPrime_FciB_B;
        for (int j = i; j < this->N; j++) {
            Eigen::Matrix3d rTilde_FcjB_B = eigenTilde(this->translatingBodyVec[j].r_FcB_B);

            // Translation contributions
            backSubContr.matrixA += this->translatingBodyVec[j].mass *  this->translatingBodyVec[i].fHat_B * this->ARho.row(i);
            backSubContr.matrixB += this->translatingBodyVec[j].mass *  this->translatingBodyVec[i].fHat_B * this->BRho.row(i);
            backSubContr.vecTrans -= this->translatingBodyVec[j].mass * this->translatingBodyVec[i].fHat_B * this->CRho.row(i);

            // Rotation contributions
            backSubContr.matrixC += this->translatingBodyVec[j].mass * rTilde_FcjB_B * this->translatingBodyVec[i].fHat_B * this->ARho.row(i);
            backSubContr.matrixD += this->translatingBodyVec[j].mass * rTilde_FcjB_B * this->translatingBodyVec[i].fHat_B * this->BRho.row(i);
            backSubContr.vecRot -= this->translatingBodyVec[j].mass * rTilde_FcjB_B * this->translatingBodyVec[i].fHat_B * this->CRho.row(i);
        }
    }
}

/*! This method is used to find the derivatives for the TB stateEffector: rhoDDot and the kinematic derivative */
void linearTranslationNDOFStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    // Find rDDotLoc_BN_B
    const Eigen::Vector3d& rDDotLocal_BN_N = rDDot_BN_N;
    Eigen::Vector3d rDDotLocal_BN_B = this->dcm_BN * rDDotLocal_BN_N;

    // Compute rho and rhoDot derivatives
    Eigen::VectorXd rhoDDot = this->ARho * rDDotLocal_BN_B + this->BRho * omegaDot_BN_B + this->CRho;
    this->rhoState->setDerivative(this->rhoDotState->getState());
    this->rhoDotState->setDerivative(rhoDDot);
}

/*! This method is for calculating the contributions of the TB state effector to the energy and momentum of the spacecraft */
void linearTranslationNDOFStateEffector::updateEnergyMomContributions(double integTime,
                                                                      Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                      double & rotEnergyContr,
                                                                      Eigen::Vector3d omega_BN_B)
{
    this->omega_BN_B = omega_BN_B;
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    rotAngMomPntCContr_B = Eigen::Vector3d::Zero();
    rotEnergyContr = 0.0;

    for(auto& translatingBody: this->translatingBodyVec) {
        // Update omega_FN_B
        translatingBody.omega_FN_B = this->omega_BN_B;

        // Compute rDot_FcB_B
        translatingBody.rDot_FcB_B = translatingBody.rPrime_FcB_B + omegaTilde_BN_B * translatingBody.r_FcB_B;

        // Find rotational angular momentum contribution from hub
        rotAngMomPntCContr_B += translatingBody.IPntFc_B * translatingBody.omega_FN_B + translatingBody.mass * translatingBody.rTilde_FcB_B * translatingBody.rDot_FcB_B;

        // Find rotational energy contribution from the hub
        rotEnergyContr += 1.0 / 2.0 * translatingBody.omega_FN_B.dot(translatingBody.IPntFc_B * translatingBody.omega_FN_B)
                        + 1.0 / 2.0 * translatingBody.mass * translatingBody.rDot_FcB_B.dot(translatingBody.rDot_FcB_B)
                        + 1.0 / 2.0 * translatingBody.k * (translatingBody.rho - translatingBody.rhoRef) *
                        (translatingBody.rho - translatingBody.rhoRef);
    }
}

/*! This method computes the translating body states relative to the inertial frame */
void linearTranslationNDOFStateEffector::computeTranslatingBodyInertialStates()
{
    for(auto& translatingBody: this->translatingBodyVec) {
        // Compute the rotational properties
        Eigen::Matrix3d dcm_FN;
        dcm_FN = translatingBody.dcm_FB * this->dcm_BN;
        translatingBody.sigma_FN = eigenMRPd2Vector3d(eigenC2MRP(dcm_FN));
        translatingBody.omega_FN_F = translatingBody.dcm_FB.transpose().transpose() * translatingBody.omega_FN_B;

        // Compute the translation properties
        translatingBody.r_FcN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * translatingBody.r_FcB_B;
        translatingBody.v_FcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * translatingBody.rDot_FcB_B;
    }
}

/*! This method is used so that the simulation will ask TB to update messages */
void linearTranslationNDOFStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->computeTranslatingBodyInertialStates();
    this->writeOutputStateMessages(CurrentSimNanos);
}
