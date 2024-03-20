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
//YES
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
//YES
linearTranslationNDOFStateEffector::~linearTranslationNDOFStateEffector()
{
    linearTranslationNDOFStateEffector::effectorID --;    /* reset the panel ID*/
}

/*! This method is used to reset the module. */
//YES
void linearTranslationNDOFStateEffector::Reset(uint64_t CurrentClock)
{
    for(auto& translatingBody: this->translatingBodyVec) {
        if (translatingBody.fHat_F.norm() > 0.01) {
            translatingBody.fHat_F.normalize();
        }
        else {
            bskLogger.bskLog(BSK_ERROR, "Norm of fHat must be greater than 0. sHat may not have been set by the user.");
        }
    }
}

//YES
void linearTranslationNDOFStateEffector::addTranslatingBody(const translatingBody& newBody) {
    // Pushback new body
    translatingBodyVec.push_back(newBody);
    this->N++;
    
    // Create the output vectors
    this->translatingBodyConfigLogOutMsgs.push_back(new Message<SCStatesMsgPayload>);
    this->translatingBodyOutMsgs.push_back(new Message<LinearTranslationRigidBodyMsgPayload>);

    // resize A B and C
    this->ARho.conservativeResize(this->ARho.rows()+1, 3);
    this->BRho.conservativeResize(this->BRho.rows()+1, 3);
    this->CRho.conservativeResize(this->CRho.rows()+1);
}


/*! This method takes the computed theta states and outputs them to the messaging system. */
//YES
void linearTranslationNDOFStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    // Write out the spinning body output messages
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

            // Logging the S frame is the body frame B of that object
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
//YES
void linearTranslationNDOFStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfRhoState = this->nameOfSpacecraftAttachedTo + this->nameOfRhoState;
    this->nameOfRhoDotState = this->nameOfSpacecraftAttachedTo + this->nameOfRhoDotState;
}

/*! This method allows the SB state effector to have access to the hub states and gravity*/
//YES
void linearTranslationNDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "v_BN_N");
}

/*! This method allows the SB state effector to register its states: rho and rhoDot with the dynamic parameter manager */
//YES
void linearTranslationNDOFStateEffector::registerStates(DynParamManager& states)
{
    // Register the theta states
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

/*! This method allows the SB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
//YES
void linearTranslationNDOFStateEffector::updateEffectorMassProps(double integTime)
{
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B = Eigen::Vector3d::Zero();
    this->effProps.rEffPrime_CB_B = Eigen::Vector3d::Zero();
    this->effProps.IEffPntB_B = Eigen::Matrix3d::Zero();
    this->effProps.IEffPrimePntB_B = Eigen::Matrix3d::Zero();

    int i = 0;
    for(auto& translatingBody: this->translatingBodyVec) {
        // Give the mass of the spinning body to the effProps mass
        this->effProps.mEff += translatingBody.mass;

        // Grab current states
        translatingBody.rho = this->rhoState->getState()(i, 0);
        translatingBody.rhoDot = this->rhoDotState->getState()(i, 0);

        // Write the spinning axis in B frame
        translatingBody.fHat_B = translatingBody.dcm_BF * translatingBody.fHat_F;

        // Compute the effector's CoM with respect to point B
        translatingBody.r_FcF_B = translatingBody.dcm_BF * translatingBody.r_FcF_F;
        if (i == 0) {
        // parent frame of first body is b frame
            translatingBody.r_FP_B = translatingBody.r_FP_P;
            translatingBody.r_FB_B = translatingBody.r_FP_P;
            translatingBody.r_FcB_B = translatingBody.r_FcF_B + translatingBody.r_FB_B;
        } else {
            translatingBody.r_FP_B = this->translatingBodyVec[i-1].dcm_BF * translatingBody.r_FP_P;
            translatingBody.r_FB_B = translatingBody.r_FP_B + this->translatingBodyVec[i-1].r_FB_B;
            translatingBody.r_FcB_B = translatingBody.r_FcF_B + translatingBody.r_FB_B;
        }
        this->effProps.rEff_CB_B += translatingBody.mass * translatingBody.r_FcB_B;

        // Find the inertia of the bodies about point B
        translatingBody.rTilde_FcB_B = eigenTilde(translatingBody.r_FcB_B);
        translatingBody.IPntFc_B = translatingBody.dcm_BF * translatingBody.IPntFc_F * translatingBody.dcm_BF.transpose();
        this->effProps.IEffPntB_B += translatingBody.IPntFc_B - translatingBody.mass * translatingBody.rTilde_FcB_B * translatingBody.rTilde_FcB_B;

        // Find rPrime_Sc1B_B and rPrime_Sc2B_B
                // BN_B or FN_B check
        Eigen::Matrix3d omegaTilde_FN_B = eigenTilde(translatingBody.omega_FN_B);
        translatingBody.rPrime_FcF_B = omegaTilde_FN_B * translatingBody.r_FcF_B;
        if (i == 0) {
            translatingBody.rPrime_FP_B = Eigen::Vector3d::Zero();
            translatingBody.rPrime_FB_B = translatingBody.rPrime_FP_B;
        } else {
        // todo if else could be an issue
            translatingBody.rPrime_FP_B = Eigen::Vector3d::Zero();
            // translatingBody.rPrime_FP_B = this->translatingBodyVec[i-1].omegaTilde_FB_B * translatingBody.r_FP_B;
            translatingBody.rPrime_FB_B = translatingBody.rPrime_FP_B + this->translatingBodyVec[i-1].rPrime_FB_B;
        }
        translatingBody.rPrime_FcB_B = translatingBody.rPrime_FcF_B + translatingBody.rPrime_FB_B;
        this->effProps.rEffPrime_CB_B += translatingBody.mass * translatingBody.rPrime_FcB_B;

        // Find the body-frame time derivative of the inertias of each arm and the entire spacecraft
        // todo BN_B or FN_B check
        translatingBody.IPrimePntFc_B = omegaTilde_FN_B * translatingBody.IPntFc_B - translatingBody.IPntFc_B * omegaTilde_FN_B;
        Eigen::Matrix3d rPrimeTilde_PcB_B = eigenTilde(translatingBody.rPrime_FcB_B);
        this->effProps.IEffPrimePntB_B += translatingBody.IPrimePntFc_B - translatingBody.mass * (rPrimeTilde_PcB_B * translatingBody.rTilde_FcB_B + translatingBody.rTilde_FcB_B * rPrimeTilde_PcB_B);

        i++;
    }
    this->effProps.rEff_CB_B /= this->effProps.mEff;
    this->effProps.rEffPrime_CB_B /= this->effProps.mEff;
}

/*! This method allows the SB state effector to give its contributions to the matrices needed for the back-sub 
 method */
//YES
void linearTranslationNDOFStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // Find the DCM from N to B frames
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

    this->omega_BN_B = omega_BN_B;
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    // update omega_Pn_B

    // Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_B;
    gLocal_N = g_N;
    g_B = this->dcm_BN * gLocal_N;

    // Compute MRho
    Eigen::MatrixXd MRho(this->N, this->N);
    for (int n = 0; n<this->N; n++) {
        for (int i = 0; i<this->N; i++) {
            MRho(n,i) = 0.0;
            // CHECK
            for (int j = n; j<this->N; j++) {
                MRho(n,i) += this->translatingBodyVec[n].fHat_B.transpose() * this->translatingBodyVec[j].mass
                    * this->translatingBodyVec[i].fHat_B;
            }
        }
    }

    // Compute ARhoStar, BRhoStar and CRhoStar
    Eigen::MatrixXd ARhoStar(this->N, 3);
    Eigen::MatrixXd BRhoStar(this->N, 3);
    Eigen::VectorXd CRhoStar(this->N);
    for (int n = 0; n<this->N; n++) {
        ARhoStar.row(n) = Eigen::Vector3d::Zero().transpose();
        BRhoStar.row(n) = Eigen::Vector3d::Zero().transpose();
        // WHAT IS IS INITIALIZATION
        CRhoStar(n, 0) = this->translatingBodyVec[n].u
                           - this->translatingBodyVec[n].k * this->translatingBodyVec[n].rho
                           - this->translatingBodyVec[n].c * this->translatingBodyVec[n].rhoDot;
        for (int i = n; i<this->N; i++) {
            Eigen::Vector3d r_FciB_B = this->translatingBodyVec[i].r_FcB_B;
            Eigen::Matrix3d rTilde_FciB_B = eigenTilde(r_FciB_B);
            Eigen::Vector3d rPrime_FciB_B = this->translatingBodyVec[i].rPrime_FcB_B;

            ARhoStar.row(n) -= this->translatingBodyVec[n].fHat_B.transpose() * this->translatingBodyVec[i].mass;
            BRhoStar.row(n) -= this->translatingBodyVec[n].fHat_B.transpose() * this->translatingBodyVec[i].mass * rTilde_FciB_B;
            CRhoStar(n, 0) -= this->translatingBodyVec[n].fHat_B.transpose() * this->translatingBodyVec[i].mass *
                              (omegaTilde_BN_B*omegaTilde_BN_B*r_FciB_B + 2*omegaTilde_BN_B * rPrime_FciB_B);
        }
    }

    // Define the ARho, BRho and CRho matrices
    this->ARho = MRho.inverse() * ARhoStar;
    this->BRho = MRho.inverse() * BRhoStar;
    this->CRho = MRho.inverse() * CRhoStar;

    //
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

/*! This method is used to find the derivatives for the SB stateEffector: rhoDDot and the kinematic derivative */
//YES
void linearTranslationNDOFStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{   
    // Grab omegaDot_BN_B 
    Eigen::Vector3d omegaDotLocal_BN_B;
    omegaDotLocal_BN_B = omegaDot_BN_B;

    // Find rDDotLoc_BN_B
    const Eigen::Vector3d& rDDotLocal_BN_N = rDDot_BN_N;
    Eigen::Vector3d rDDotLocal_BN_B = this->dcm_BN * rDDotLocal_BN_N;

    // Compute rho and rhoDot derivatives
    Eigen::VectorXd rhoDDot = this->ARho * rDDotLocal_BN_B + this->BRho * omegaDotLocal_BN_B + this->CRho;
    this->rhoState->setDerivative(this->rhoDotState->getState());
    this->rhoDotState->setDerivative(rhoDDot);
}

/*! This method is for calculating the contributions of the SB state effector to the energy and momentum of the spacecraft */
//YES
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
        // Update omega_BN_B and omega_FN_B
        // translatingBody.omega_FN_B = translatingBody.omega_FB_B + this->omega_BN_B; // possible replacement line
        translatingBody.omega_FN_B = this->omega_BN_B;

        // Compute rDot_FcB_B
        translatingBody.rDot_FcB_B = translatingBody.rPrime_FcB_B + omegaTilde_BN_B * translatingBody.r_FcB_B;

        // Find rotational angular momentum contribution from hub
        rotAngMomPntCContr_B += translatingBody.IPntFc_B * translatingBody.omega_FN_B + translatingBody.mass * translatingBody.rTilde_FcB_B * translatingBody.rDot_FcB_B;

        // Find rotational energy contribution from the hub
        rotEnergyContr += 1.0 / 2.0 * translatingBody.omega_FN_B.dot(translatingBody.IPntFc_B * translatingBody.omega_FN_B)
                        + 1.0 / 2.0 * translatingBody.mass * translatingBody.rDot_FcB_B.dot(translatingBody.rDot_FcB_B)
                        + 1.0 / 2.0 * translatingBody.k * translatingBody.rho * translatingBody.rho;
    }
}

/*! This method computes the spinning body states relative to the inertial frame */
//YES
void linearTranslationNDOFStateEffector::computeTranslatingBodyInertialStates()
{
    for(auto& translatingBody: this->translatingBodyVec) {
        // Compute the rotational properties
        Eigen::Matrix3d dcm_FN;
        dcm_FN = translatingBody.dcm_BF.transpose() * this->dcm_BN;
        translatingBody.sigma_FN = eigenMRPd2Vector3d(eigenC2MRP(dcm_FN));
        // maybe should be pnb and set pnb = bnb earlier somewhere (forgot where)
        translatingBody.omega_FN_F = translatingBody.dcm_BF.transpose() * translatingBody.omega_FN_B;

        // Compute the translation properties
        translatingBody.r_FcN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * translatingBody.r_FcB_B;
        translatingBody.v_FcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * translatingBody.rDot_FcB_B;
    }
}

/*! This method is used so that the simulation will ask SB to update messages */
//YES
void linearTranslationNDOFStateEffector::UpdateState(uint64_t CurrentSimNanos)
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
            translatingBody.lockFlag = incomingLockBuffer.effectorLockFlag[i];
            i++;
        }
    }

    /* Compute translating body inertial states */
    this->computeTranslatingBodyInertialStates();
    
    /* Write output messages*/
    this->writeOutputStateMessages(CurrentSimNanos);
}
