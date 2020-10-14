/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "dualHingedRigidBodyStateEffector.h"
#include "simFswInterfaceMessages/arrayMotorTorqueIntMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "architecture/messaging/system_messaging.h"
#include "../../utilities/rigidBodyKinematics.h"
#include "../../utilities/avsEigenSupport.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <iostream>

DualHingedRigidBodyStateEffector::DualHingedRigidBodyStateEffector()
{
    // - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.mEffDot = 0.0;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();
    this->matrixFDHRB.resize(2, 3);
    this->matrixGDHRB.resize(2, 3);

    // - Initialize the variables to working values
    this->mass1 = 0.0;
    this->d1 = 1.0;
    this->k1 = 1.0;
    this->c1 = 0.0;
    this->l1 = 0.0;
    this->u1 = 0.0;
    this->mass2 = 0.0;
    this->d2 = 1.0;
    this->k2 = 1.0;
    this->c2 = 0.0;
    this->u2 = 0.0;
    this->theta1Init = 0.0;
    this->theta1DotInit = 0.0;
    this->theta2Init = 0.0;
    this->theta2DotInit = 0.0;
    this->IPntS1_S1.setIdentity();
    this->IPntS2_S2.setIdentity();
    this->r_H1B_B.setZero();
    this->dcm_H1B.setIdentity();
    this->thetaH2S1 = 0.0;
    this->nameOfTheta1State = "hingedRigidBodyTheta1";
    this->nameOfTheta1DotState = "hingedRigidBodyTheta1Dot";
    this->nameOfTheta2State = "hingedRigidBodyTheta2";
    this->nameOfTheta2DotState = "hingedRigidBodyTheta2Dot";

    this->motorTorqueInMsgName = "";
    this->motorTorqueInMsgId = -1;

    this->dualHingedRigidBodyOutMsgName = "";
    this->dualHingedRigidBodyConfigLogOutMsgName = "";
    for (int i=0; i<2; i++) {
        this->dualHingedRigidBodyOutMsgId[i] = -1;
        this->dualHingedRigidBodyConfigLogOutMsgId[i] = -1;
    }

    this->ModelTag = "";

    return;
}


DualHingedRigidBodyStateEffector::~DualHingedRigidBodyStateEffector()
{
    return;
}

/*! This method initializes the object. It creates the module's output
 messages.
 @return void*/
void DualHingedRigidBodyStateEffector::SelfInit()
{
    SystemMessaging *messageSys = SystemMessaging::GetInstance();

    /* create panel angular state output messages */
    if (this->dualHingedRigidBodyOutMsgName.length() == 0) {
        if (this->ModelTag.length() > 0) {
            this->dualHingedRigidBodyOutMsgName = this->ModelTag;
        } else {
            /* make sure this has a unique output name in case the user didn't set ModelTag */
            this->dualHingedRigidBodyOutMsgName = "panel" + std::to_string(rand());
        }
    }
    for (int i=0; i<2;i++) {
        this->dualHingedRigidBodyOutMsgId[i] =  messageSys->CreateNewMessage(this->dualHingedRigidBodyOutMsgName + "_OutputStates" + std::to_string(i),
                                                 sizeof(HingedRigidBodySimMsg), 2, "HingedRigidBodySimMsg", this->moduleID);
    }

    /* create panel inertial position/attitude output message */
    if (this->dualHingedRigidBodyConfigLogOutMsgName.length() == 0) {
        this->dualHingedRigidBodyConfigLogOutMsgName = this->ModelTag;
    }
    for (int i=0; i<2;i++) {
        this->dualHingedRigidBodyConfigLogOutMsgId[i] =  messageSys->CreateNewMessage(this->dualHingedRigidBodyConfigLogOutMsgName
                                                                                      + "_InertialStates" + std::to_string(i),
                                                                                      sizeof(SCPlusStatesSimMsg), 2, "SCPlusStatesSimMsg", this->moduleID);
    }

    return;
}

/*! This method subscribes to messages the HRB needs.
 @return void*/
void DualHingedRigidBodyStateEffector::CrossInit()
{
    /* check if the optional motor torque input message name has been set */
    if (this->motorTorqueInMsgName.length() > 0) {
        this->motorTorqueInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->motorTorqueInMsgName,
                                                                                     sizeof(ArrayMotorTorqueIntMsg),
                                                                                     moduleID);
    }

    return;
}

void DualHingedRigidBodyStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfTheta1State = this->nameOfSpacecraftAttachedTo + this->nameOfTheta1State;
    this->nameOfTheta1DotState = this->nameOfSpacecraftAttachedTo + this->nameOfTheta1DotState;
    this->nameOfTheta2State = this->nameOfSpacecraftAttachedTo + this->nameOfTheta2State;
    this->nameOfTheta2DotState = this->nameOfSpacecraftAttachedTo + this->nameOfTheta2DotState;

    return;
}


void DualHingedRigidBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    // - Get access to the hubs sigma, omegaBN_B and velocity needed for dynamic coupling
    this->g_N = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "g_N");

    this->sigma_BNState = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + "hubSigma");
    this->omega_BN_BState = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + "hubOmega");
    this->r_BN_NState = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + "hubPosition");
    this->v_BN_NState = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + "hubVelocity");

    return;
}

void DualHingedRigidBodyStateEffector::registerStates(DynParamManager& states)
{
    // - Register the states associated with hinged rigid bodies - theta and thetaDot
    this->theta1State = states.registerState(1, 1, this->nameOfTheta1State);
    this->theta1DotState = states.registerState(1, 1, this->nameOfTheta1DotState);
    this->theta2State = states.registerState(1, 1, this->nameOfTheta2State);
    this->theta2DotState = states.registerState(1, 1, this->nameOfTheta2DotState);

    // - Add this code to allow for non-zero initial conditions, as well hingedRigidBody
    Eigen::MatrixXd theta1InitMatrix(1,1);
    theta1InitMatrix(0,0) = this->theta1Init;
    this->theta1State->setState(theta1InitMatrix);
    Eigen::MatrixXd theta1DotInitMatrix(1,1);
    theta1DotInitMatrix(0,0) = this->theta1DotInit;
    this->theta1DotState->setState(theta1DotInitMatrix);
    Eigen::MatrixXd theta2InitMatrix(1,1);
    theta2InitMatrix(0,0) = this->theta2Init;
    this->theta2State->setState(theta2InitMatrix);
    Eigen::MatrixXd theta2DotInitMatrix(1,1);
    theta2DotInitMatrix(0,0) = this->theta2DotInit;
    this->theta2DotState->setState(theta2DotInitMatrix);

    return;
}

void DualHingedRigidBodyStateEffector::updateEffectorMassProps(double integTime)
{
    // - Give the mass of the hinged rigid body to the effProps mass
    this->effProps.mEff = this->mass1 + this->mass2;

    // - find hinged rigid bodies' position with respect to point B
    // - First need to grab current states
    this->theta1 = this->theta1State->getState()(0, 0);
    this->theta1Dot = this->theta1DotState->getState()(0, 0);
    this->theta2 = this->theta2State->getState()(0, 0);
    this->theta2Dot = this->theta2DotState->getState()(0, 0);
    // - Next find the sHat unit vectors
    Eigen::Matrix3d dcmS1H1;
    dcmS1H1 = eigenM2(this->theta1);
    this->dcm_S1B = dcmS1H1*this->dcm_H1B;
    Eigen::Matrix3d dcmH2S1;
    dcmH2S1 = eigenM2(this->thetaH2S1);
    Eigen::Matrix3d dcmH2B;
    dcmH2B = dcmH2S1*this->dcm_S1B;
    Eigen::Matrix3d dcmS2H2;
    dcmS2H2 = eigenM2(this->theta2);
    this->dcm_S2B = dcmS2H2 * dcmH2B;
    this->sHat11_B = this->dcm_S1B.row(0);
    this->sHat12_B = this->dcm_S1B.row(1);
    this->sHat13_B = this->dcm_S1B.row(2);
    this->sHat21_B = this->dcm_S2B.row(0);
    this->sHat22_B = this->dcm_S2B.row(1);
    this->sHat23_B = this->dcm_S2B.row(2);
    this->r_S1B_B = this->r_H1B_B - this->d1*this->sHat11_B;
    this->r_S2B_B = this->r_H1B_B - this->l1*this->sHat11_B - this->d2*this->sHat21_B;
    this->effProps.rEff_CB_B = 1.0/this->effProps.mEff*(this->mass1*this->r_S1B_B + this->mass2*this->r_S2B_B);

    // - Find the inertia of the hinged rigid body about point B
    // - Define rTildeSB_B
    this->rTildeS1B_B = eigenTilde(this->r_S1B_B);
    this->rTildeS2B_B = eigenTilde(this->r_S2B_B);
    this->effProps.IEffPntB_B = this->dcm_S1B.transpose()*this->IPntS1_S1*this->dcm_S1B + this->mass1*this->rTildeS1B_B*this->rTildeS1B_B.transpose() + this->dcm_S2B.transpose()*this->IPntS2_S2*this->dcm_S2B + this->mass2*this->rTildeS2B_B*this->rTildeS2B_B.transpose();

    // First, find the rPrimeSB_B
    this->rPrimeS1B_B = this->d1*this->theta1Dot*this->sHat13_B;
    this->rPrimeS2B_B = this->l1*this->theta1Dot*this->sHat13_B + this->d2*(this->theta1Dot + this->theta2Dot)*this->sHat23_B;
    this->effProps.rEffPrime_CB_B = 1.0/this->effProps.mEff*(this->mass1*this->rPrimeS1B_B + this->mass2*this->rPrimeS2B_B);

    // - Next find the body time derivative of the inertia about point B
    // - Define tilde matrix of rPrimeSB_B
    this->rPrimeTildeS1B_B = eigenTilde(this->rPrimeS1B_B);
    this->rPrimeTildeS2B_B = eigenTilde(this->rPrimeS2B_B);
    // - Find body time derivative of IPntS_B
    this->IS1PrimePntS1_B = this->theta1Dot*(this->IPntS1_S1(2,2) - this->IPntS1_S1(0,0))*(this->sHat11_B*this->sHat13_B.transpose() + this->sHat13_B*this->sHat11_B.transpose());
    this->IS2PrimePntS2_B = (this->theta1Dot+this->theta2Dot)*(this->IPntS2_S2(2,2) - this->IPntS2_S2(0,0))*(this->sHat21_B*this->sHat23_B.transpose() + this->sHat23_B*this->sHat21_B.transpose());
    // - Find body time derivative of IPntB_B
    this->effProps.IEffPrimePntB_B = this->IS1PrimePntS1_B - this->mass1*(this->rPrimeTildeS1B_B*this->rTildeS1B_B + this->rTildeS1B_B*this->rPrimeTildeS1B_B) + this->IS2PrimePntS2_B - this->mass2*(this->rPrimeTildeS2B_B*this->rTildeS2B_B + this->rTildeS2B_B*this->rPrimeTildeS2B_B);

    return;
}

void DualHingedRigidBodyStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcmBN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcmNB;                        /* direction cosine matrix from B to N */
    Eigen::Vector3d gravityTorquePntH1_B;          /* torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gravityTorquePntH2_B;          /* torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_B;                          /* gravitational acceleration in B frame */
    gLocal_N = *this->g_N;

    // - Find dcmBN
    sigmaBNLocal = (Eigen::Vector3d )this->sigma_BNState->getState();
    dcmNB = sigmaBNLocal.toRotationMatrix();
    dcmBN = dcmNB.transpose();
    // - Map gravity to body frame
    g_B = dcmBN*gLocal_N;

    // - Define gravity terms
    Eigen::Vector3d gravTorquePan1PntH1 = -this->d1*this->sHat11_B.cross(this->mass1*g_B);
    Eigen::Vector3d gravForcePan2 = this->mass2*g_B;
    Eigen::Vector3d gravTorquePan2PntH2 = -this->d2*this->sHat21_B.cross(this->mass2*g_B);

    // - Define omegaBN_S
    this->omega_BNLoc_B = this->omega_BN_BState->getState();
    this->omega_BN_S1 = this->dcm_S1B*this->omega_BNLoc_B;
    this->omega_BN_S2 = this->dcm_S2B*this->omega_BNLoc_B;
    // - Define omegaTildeBNLoc_B
    this->omegaTildeBNLoc_B = eigenTilde(this->omega_BNLoc_B);
    // - Define matrices needed for back substitution
    //gravityTorquePntH1_B = -this->d1*this->sHat11_B.cross(this->mass1*g_B); //Need to review these equations and implement them - SJKC
    //gravityTorquePntH2_B = -this->d2*this->sHat21_B.cross(this->mass2*g_B); //Need to review these equations and implement them - SJKC
    this->matrixADHRB(0,0) = this->IPntS1_S1(1,1) + this->mass1*this->d1*this->d1 + this->mass2*this->l1*this->l1 + this->mass2*this->l1*this->d2*this->sHat13_B.transpose()*(this->sHat23_B);
    this->matrixADHRB(0,1) = this->mass2*this->l1*this->d2*this->sHat13_B.transpose()*(this->sHat23_B);
    this->matrixADHRB(1,0) = IPntS2_S2(1,1) + this->mass2*this->d2*this->d2 + this->mass2*this->l1*this->d2*this->sHat23_B.transpose()*this->sHat13_B;
    this->matrixADHRB(1,1) = this->IPntS2_S2(1,1) + this->mass2*this->d2*this->d2;
    this->matrixEDHRB = this->matrixADHRB.inverse();
    this->matrixFDHRB.row(0) = -(this->mass2*this->l1 + this->mass1*this->d1)*this->sHat13_B.transpose();
    this->matrixFDHRB.row(1) = -this->mass2*this->d2*this->sHat23_B.transpose();

    this->matrixGDHRB.row(0) = -(this->IPntS1_S1(1,1)*this->sHat12_B.transpose() - this->mass1*this->d1*this->sHat13_B.transpose()*this->rTildeS1B_B - this->mass2*this->l1*this->sHat13_B.transpose()*this->rTildeS2B_B);
    this->matrixGDHRB.row(1) = -(this->IPntS2_S2(1,1)*this->sHat22_B.transpose() - this->mass2*this->d2*this->sHat23_B.transpose()*this->rTildeS2B_B);

    this->vectorVDHRB(0) =  -(this->IPntS1_S1(0,0) - this->IPntS1_S1(2,2))*this->omega_BN_S1(2)*this->omega_BN_S1(0)
                            + this->u1 - this->k1*this->theta1 - this->c1*this->theta1Dot + this->k2*this->theta2 + this->c2*this->theta2Dot + this->sHat12_B.dot(gravTorquePan1PntH1) + this->l1*this->sHat13_B.dot(gravForcePan2)
                            - this->mass1*this->d1*this->sHat13_B.transpose()*(2*this->omegaTildeBNLoc_B*this->rPrimeS1B_B + this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->r_S1B_B)
                            - this->mass2*this->l1*this->sHat13_B.transpose()*(2*this->omegaTildeBNLoc_B*this->rPrimeS2B_B + this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->r_S2B_B + this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_B + this->d2*(this->theta1Dot + this->theta2Dot)*(this->theta1Dot + this->theta2Dot)*this->sHat21_B); //still missing torque and force terms - SJKC

    this->vectorVDHRB(1) =  -(this->IPntS2_S2(0,0) - this->IPntS2_S2(2,2))*this->omega_BN_S2(2)*this->omega_BN_S2(0)
                            + this->u2 - this->k2*this->theta2 - this->c2*this->theta2Dot + this->sHat22_B.dot(gravTorquePan2PntH2) - this->mass2*this->d2*this->sHat23_B.transpose()*(2*this->omegaTildeBNLoc_B*this->rPrimeS2B_B + this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->r_S2B_B + this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_B); // still missing torque term. - SJKC

    // - Start defining them good old contributions - start with translation
    // - For documentation on contributions see Allard, Diaz, Schaub flex/slosh paper
    backSubContr.matrixA = (this->mass1*this->d1*this->sHat13_B + this->mass2*this->l1*this->sHat13_B + this->mass2*this->d2*this->sHat23_B)*matrixEDHRB.row(0)*this->matrixFDHRB + this->mass2*this->d2*this->sHat23_B*this->matrixEDHRB.row(1)*this->matrixFDHRB;
    backSubContr.matrixB = (this->mass1*this->d1*this->sHat13_B + this->mass2*this->l1*this->sHat13_B + this->mass2*this->d2*this->sHat23_B)*this->matrixEDHRB.row(0)*(matrixGDHRB) + this->mass2*this->d2*this->sHat23_B*this->matrixEDHRB.row(1)*(matrixGDHRB);
    backSubContr.vecTrans = -(this->mass1*this->d1*this->theta1Dot*this->theta1Dot*this->sHat11_B + this->mass2*(this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_B + this->d2*(this->theta1Dot+this->theta2Dot)*(this->theta1Dot+this->theta2Dot)*this->sHat21_B)
                    + (this->mass1*this->d1*this->sHat13_B + this->mass2*this->l1*this->sHat13_B + this->mass2*this->d2*this->sHat23_B)*this->matrixEDHRB.row(0)*this->vectorVDHRB + this->mass2*this->d2*this->sHat23_B*this->matrixEDHRB.row(1)*this->vectorVDHRB);

    // - Define rotational matrice contributions (Eq 96 in paper)
    
    backSubContr.matrixC = (this->IPntS1_S1(1,1)*this->sHat12_B + this->mass1*this->d1*this->rTildeS1B_B*this->sHat13_B + this->IPntS2_S2(1,1)*this->sHat22_B + this->mass2*this->l1*this->rTildeS2B_B*this->sHat13_B + this->mass2*this->d2*this->rTildeS2B_B*this->sHat23_B)*this->matrixEDHRB.row(0)*this->matrixFDHRB
                    + (this->IPntS2_S2(1,1)*this->sHat22_B + this->mass2*this->d2*this->rTildeS2B_B*this->sHat23_B)*this->matrixEDHRB.row(1)*this->matrixFDHRB;
    
    backSubContr.matrixD = (this->IPntS1_S1(1,1)*this->sHat12_B + this->mass1*this->d1*this->rTildeS1B_B*this->sHat13_B + this->IPntS2_S2(1,1)*this->sHat22_B + this->mass2*this->l1*this->rTildeS2B_B*this->sHat13_B + this->mass2*this->d2*this->rTildeS2B_B*this->sHat23_B)*this->matrixEDHRB.row(0)*this->matrixGDHRB
                    +(this->IPntS2_S2(1,1)*this->sHat22_B + this->mass2*this->d2*this->rTildeS2B_B*this->sHat23_B)*this->matrixEDHRB.row(1)*this->matrixGDHRB;
    
    backSubContr.vecRot = -(this->theta1Dot*this->IPntS1_S1(1,1)*this->omegaTildeBNLoc_B*this->sHat12_B
                    + this->mass1*this->omegaTildeBNLoc_B*this->rTildeS1B_B*this->rPrimeS1B_B + this->mass1*this->d1*this->theta1Dot*this->theta1Dot*this->rTildeS1B_B*this->sHat11_B + (this->theta1Dot+this->theta2Dot)*this->IPntS2_S2(1,1)*this->omegaTildeBNLoc_B*this->sHat22_B + this->mass2*this->omegaTildeBNLoc_B*this->rTildeS2B_B*this->rPrimeS2B_B
                    + this->mass2*this->rTildeS2B_B*(this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_B + this->d2*(this->theta1Dot+this->theta2Dot)*(this->theta1Dot+this->theta2Dot)*this->sHat21_B) + (this->IPntS1_S1(1,1)*this->sHat12_B + this->mass1*this->d1*this->rTildeS1B_B*this->sHat13_B + this->IPntS2_S2(1,1)*this->sHat22_B
                    + this->mass2*this->l1*this->rTildeS2B_B*this->sHat13_B + this->mass2*this->d2*this->rTildeS2B_B*this->sHat23_B)*this->matrixEDHRB.row(0)*this->vectorVDHRB + (this->IPntS2_S2(1,1)*this->sHat22_B + this->mass2*this->d2*this->rTildeS2B_B*this->sHat23_B)*this->matrixEDHRB.row(1)*this->vectorVDHRB);

    return;
}

void DualHingedRigidBodyStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    // - Define necessarry variables
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcmBN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcmNB;                        /* direction cosine matrix from B to N */
    Eigen::MatrixXd theta1DDot(1,1);              /* thetaDDot variable to send to state manager */
    Eigen::MatrixXd theta2DDot(1,1);              /* thetaDDot variable to send to state manager */
    Eigen::Vector3d rDDotBNLoc_N;                 /* second time derivative of rBN in N frame */
    Eigen::Vector3d rDDotBNLoc_B;                 /* second time derivative of rBN in B frame */
    Eigen::Vector3d omegaDotBNLoc_B;              /* time derivative of omegaBN in B frame */

    // Grab necessarry values from manager (these have been previously computed in hubEffector)
    rDDotBNLoc_N = this->v_BN_NState->getStateDeriv();
    sigmaBNLocal = (Eigen::Vector3d )this->sigma_BNState->getState();
    omegaDotBNLoc_B = this->omega_BN_BState->getStateDeriv();
    dcmNB = sigmaBNLocal.toRotationMatrix();
    dcmBN = dcmNB.transpose();
    rDDotBNLoc_B = dcmBN*rDDotBNLoc_N;

    // - Compute Derivatives
    // - First is trivial
    this->theta1State->setDerivative(theta1DotState->getState());
    // - Second, a little more involved - see Allard, Diaz, Schaub flex/slosh paper
    theta1DDot(0,0) = this->matrixEDHRB.row(0).dot(this->matrixFDHRB*rDDotBNLoc_B) + this->matrixEDHRB.row(0)*this->matrixGDHRB*omegaDotBNLoc_B + this->matrixEDHRB.row(0)*this->vectorVDHRB;
    this->theta1DotState->setDerivative(theta1DDot);
    this->theta2State->setDerivative(theta2DotState->getState());
    theta2DDot(0,0) = this->matrixEDHRB.row(1)*(this->matrixFDHRB*rDDotBNLoc_B) + this->matrixEDHRB.row(1).dot(this->matrixGDHRB*omegaDotBNLoc_B) + this->matrixEDHRB.row(1)*this->vectorVDHRB;
    this->theta2DotState->setDerivative(theta2DDot);

    return;
}
/*! This method is for calculating the contributions of the DHRB state effector to the energy and momentum of the s/c */
void DualHingedRigidBodyStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // - Get the current omega state
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = this->omega_BN_BState->getState();
    
    // - Find rotational angular momentum contribution from hub
    Eigen::Vector3d omega_S1B_B;
    Eigen::Vector3d omega_S2B_B;
    Eigen::Vector3d omega_S1N_B;
    Eigen::Vector3d omega_S2N_B;
    Eigen::Matrix3d IPntS1_B;
    Eigen::Matrix3d IPntS2_B;
    Eigen::Vector3d rDot_S1B_B;
    Eigen::Vector3d rDot_S2B_B;
    omega_S1B_B = this->theta1Dot*this->sHat12_B;
    omega_S2B_B = (this->theta1Dot + this->theta2Dot)*this->sHat22_B;
    omega_S1N_B = omega_S1B_B + omegaLocal_BN_B;
    omega_S2N_B = omega_S2B_B + omegaLocal_BN_B;
    IPntS1_B = this->dcm_S1B.transpose()*this->IPntS1_S1*this->dcm_S1B;
    IPntS2_B = this->dcm_S2B.transpose()*this->IPntS2_S2*this->dcm_S2B;
    rDot_S1B_B = this->rPrimeS1B_B + omegaLocal_BN_B.cross(this->r_S1B_B);
    rDot_S2B_B = this->rPrimeS2B_B + omegaLocal_BN_B.cross(this->r_S2B_B);
    rotAngMomPntCContr_B = IPntS1_B*omega_S1N_B + this->mass1*this->r_S1B_B.cross(rDot_S1B_B)
                            + IPntS2_B*omega_S2N_B + this->mass2*this->r_S2B_B.cross(rDot_S2B_B);
    
    // - Find rotational energy contribution from the hub
    double rotEnergyContrS1;
    double rotEnergyContrS2;
    rotEnergyContrS1 =  0.5*omega_S1N_B.dot(IPntS1_B*omega_S1N_B)
                        + 0.5*this->mass1*rDot_S1B_B.dot(rDot_S1B_B)
                        + 0.5*this->k1*this->theta1*this->theta1;
    rotEnergyContrS2 =  0.5*omega_S2N_B.dot(IPntS2_B*omega_S2N_B)
                        + 0.5*this->mass2*rDot_S2B_B.dot(rDot_S2B_B)
                        + 0.5*this->k2*this->theta2*this->theta2;
    rotEnergyContr = rotEnergyContrS1 + rotEnergyContrS2;
    
    return;
}

/*! This method takes the computed theta states and outputs them to the m
 messaging system.
 @return void
 @param CurrentClock The current simulation time (used for time stamping)
 */
void DualHingedRigidBodyStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    SystemMessaging *messageSys = SystemMessaging::GetInstance();

    HingedRigidBodySimMsg panelOutputStates;  //!< instance of messaging system message struct
    // panel 1 states
    if (this->dualHingedRigidBodyOutMsgId[0] >= 0) {
        memset(&panelOutputStates, 0x0, sizeof(HingedRigidBodySimMsg));
        panelOutputStates.theta = this->theta1;
        panelOutputStates.thetaDot = this->theta1Dot;
        messageSys->WriteMessage(this->dualHingedRigidBodyOutMsgId[0], CurrentClock,
                             sizeof(HingedRigidBodySimMsg), reinterpret_cast<uint8_t*> (&panelOutputStates),
                                 this->moduleID);
    }
    // panel 2 states
    if (this->dualHingedRigidBodyOutMsgId[1] >= 0) {
        memset(&panelOutputStates, 0x0, sizeof(HingedRigidBodySimMsg));
        panelOutputStates.theta = this->theta2;
        panelOutputStates.thetaDot = this->theta2Dot;
        messageSys->WriteMessage(this->dualHingedRigidBodyOutMsgId[1], CurrentClock,
                             sizeof(HingedRigidBodySimMsg), reinterpret_cast<uint8_t*> (&panelOutputStates),
                                 this->moduleID);
    }


    // write out the panel state config log message
    SCPlusStatesSimMsg configLogMsg;
    // Note, logging the hinge frame S is the body frame B of that object
    for (int i=0; i<2; i++) {
        if (this->dualHingedRigidBodyConfigLogOutMsgId[i] >= 0) {
            memset(&configLogMsg, 0x0, sizeof(SCPlusStatesSimMsg));
            eigenVector3d2CArray(this->r_SN_N[i], configLogMsg.r_BN_N);
            eigenVector3d2CArray(this->v_SN_N[i], configLogMsg.v_BN_N);
            eigenVector3d2CArray(this->sigma_SN[i], configLogMsg.sigma_BN);
            eigenVector3d2CArray(this->omega_SN_S[i], configLogMsg.omega_BN_B);
            messageSys->WriteMessage(this->dualHingedRigidBodyConfigLogOutMsgId[i], CurrentClock,
                                 sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*> (&configLogMsg),
                                     this->moduleID);
        }
    }
}


/*! This method is used so that the simulation will ask DHRB to update messages.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void DualHingedRigidBodyStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Zero the command buffer and read the incoming command array
    if (this->motorTorqueInMsgId >= 0) {
        SingleMessageHeader LocalHeader;
        ArrayMotorTorqueIntMsg IncomingCmdBuffer;
        memset(&IncomingCmdBuffer, 0x0, sizeof(ArrayMotorTorqueIntMsg));
        SystemMessaging::GetInstance()->ReadMessage(this->motorTorqueInMsgId, &LocalHeader,
                                                    sizeof(ArrayMotorTorqueIntMsg),
                                                    reinterpret_cast<uint8_t*> (&IncomingCmdBuffer), moduleID);
        this->u1 = IncomingCmdBuffer.motorTorque[0];
        this->u2 = IncomingCmdBuffer.motorTorque[1];
    }

    /* compute panel inertial states */
    this->computePanelInertialStates();

    this->writeOutputStateMessages(CurrentSimNanos);

    return;
}

/*! This method computes the panel states relative to the inertial frame
 @return void
 */
void DualHingedRigidBodyStateEffector::computePanelInertialStates()
{
    // inertial attitudes
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->sigma_BNState->getState();
    Eigen::Matrix3d dcm_NB = sigmaBN.toRotationMatrix();
    this->sigma_SN[0] = eigenMRPd2Vector3d(eigenC2MRP(this->dcm_S1B*dcm_NB.transpose()));
    this->sigma_SN[1] = eigenMRPd2Vector3d(eigenC2MRP(this->dcm_S2B*dcm_NB.transpose()));

    // inertial angular velocities
    Eigen::Vector3d omega_BN_B;
    omega_BN_B = (Eigen::Vector3d)this->omega_BN_BState->getState();
    this->omega_SN_S[0] = this->dcm_S1B * ( omega_BN_B + this->theta1Dot*this->sHat12_B);
    this->omega_SN_S[1] = this->dcm_S1B * ( omega_BN_B + this->theta2Dot*this->sHat22_B);

    // inertial position vectors
    Eigen::Vector3d r_BN_N;
    r_BN_N = (Eigen::Vector3d)this->r_BN_NState->getState();
    this->r_SN_N[0] = (dcm_NB * this->r_S1B_B) + r_BN_N;
    this->r_SN_N[1] = (dcm_NB * this->r_S2B_B) + r_BN_N;

    // inertial velocity vectors
    Eigen::Vector3d omega_S1N_B = this->theta1Dot * this->sHat12_B + omega_BN_B;
    this->v_SN_N[0] = (Eigen::Vector3d)this->v_BN_NState->getState()
                    + omega_S1N_B.cross( -this->d1 * this->sHat11_B)
                    + omega_BN_B.cross(this->r_H1B_B);
    Eigen::Vector3d omega_S2N_B = this->theta2Dot * this->sHat22_B + omega_S1N_B;
    this->v_SN_N[1] = (Eigen::Vector3d)this->v_BN_NState->getState()
                    + omega_S2N_B.cross( -this->d2 * this->sHat21_B)
                    + omega_S1N_B.cross( -this->l1 * this->sHat11_B)
                    + omega_BN_B.cross(this->r_H1B_B);

    return;
}
