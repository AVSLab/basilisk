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
#include "architecture/utilities/avsEigenSupport.h"
#include <string>

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
    this->nameOfTheta1State = "DualHingedRigidBodyStateEffectorTheta1" + std::to_string(this->effectorID);
    this->nameOfTheta1DotState = "DualHingedRigidBodyStateEffectorTheta1Dot" + std::to_string(this->effectorID);
    this->nameOfTheta2State = "DualHingedRigidBodyStateEffectorTheta2" + std::to_string(this->effectorID);
    this->nameOfTheta2DotState = "DualHingedRigidBodyStateEffectorTheta2Dot" + std::to_string(this->effectorID);
    this->effectorID++;

    Message<HingedRigidBodyMsgPayload> *panelMsg;
    Message<SCStatesMsgPayload> *scMsg;
    for (int c = 0; c < 2; c++) {
        panelMsg = new Message<HingedRigidBodyMsgPayload>;
        this->dualHingedRigidBodyOutMsgs.push_back(panelMsg);
        scMsg = new Message<SCStatesMsgPayload>;
        this->dualHingedRigidBodyConfigLogOutMsgs.push_back(scMsg);
    }

    this->ModelTag = "";

    return;
}

uint64_t DualHingedRigidBodyStateEffector::effectorID = 1;

DualHingedRigidBodyStateEffector::~DualHingedRigidBodyStateEffector()
{
    for (int c=0; c<2; c++) {
        free(this->dualHingedRigidBodyOutMsgs.at(c));
        free(this->dualHingedRigidBodyConfigLogOutMsgs.at(c));
    }

    this->effectorID = 1;    /* reset the panel ID*/
    return;
}


/*! This method is used to reset the module.

 */
void DualHingedRigidBodyStateEffector::Reset(uint64_t CurrentSimNanos)
{

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

    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialVelocity);
    this->v_BN_NState = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + this->stateNameOfVelocity);

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
    // - Convert intial variables to mother craft frame relative information
    this->r_H1P_P = this->r_BP_P + this->dcm_BP.transpose()*this->r_H1B_B;
    this->dcm_H1P = this->dcm_H1B*this->dcm_BP;

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
    this->dcm_S1P = dcmS1H1*this->dcm_H1P;
    Eigen::Matrix3d dcmH2S1;
    dcmH2S1 = eigenM2(this->thetaH2S1);
    Eigen::Matrix3d dcmH2P;
    dcmH2P = dcmH2S1*this->dcm_S1P;
    Eigen::Matrix3d dcmS2H2;
    dcmS2H2 = eigenM2(this->theta2);
    this->dcm_S2P = dcmS2H2 * dcmH2P;
    this->sHat11_P = this->dcm_S1P.row(0);
    this->sHat12_P = this->dcm_S1P.row(1);
    this->sHat13_P = this->dcm_S1P.row(2);
    this->sHat21_P = this->dcm_S2P.row(0);
    this->sHat22_P = this->dcm_S2P.row(1);
    this->sHat23_P = this->dcm_S2P.row(2);
    this->r_S1P_P = this->r_H1P_P - this->d1*this->sHat11_P;
    this->r_S2P_P = this->r_H1P_P - this->l1*this->sHat11_P - this->d2*this->sHat21_P;
    this->effProps.rEff_CB_B = 1.0/this->effProps.mEff*(this->mass1*this->r_S1P_P + this->mass2*this->r_S2P_P);

    // - Find the inertia of the hinged rigid body about point B
    // - Define rTildeSB_B
    this->rTildeS1P_P = eigenTilde(this->r_S1P_P);
    this->rTildeS2P_P = eigenTilde(this->r_S2P_P);
    this->effProps.IEffPntB_B = this->dcm_S1P.transpose()*this->IPntS1_S1*this->dcm_S1P + this->mass1*this->rTildeS1P_P*this->rTildeS1P_P.transpose() + this->dcm_S2P.transpose()*this->IPntS2_S2*this->dcm_S2P + this->mass2*this->rTildeS2P_P*this->rTildeS2P_P.transpose();

    // First, find the rPrimeSB_B
    this->rPrimeS1P_P = this->d1*this->theta1Dot*this->sHat13_P;
    this->rPrimeS2P_P = this->l1*this->theta1Dot*this->sHat13_P + this->d2*(this->theta1Dot + this->theta2Dot)*this->sHat23_P;
    this->effProps.rEffPrime_CB_B = 1.0/this->effProps.mEff*(this->mass1*this->rPrimeS1P_P + this->mass2*this->rPrimeS2P_P);

    // - Next find the body time derivative of the inertia about point B
    // - Define tilde matrix of rPrimeSB_B
    this->rPrimeTildeS1P_P = eigenTilde(this->rPrimeS1P_P);
    this->rPrimeTildeS2P_P = eigenTilde(this->rPrimeS2P_P);
    // - Find body time derivative of IPntS_B
    this->IS1PrimePntS1_P = this->theta1Dot*(this->IPntS1_S1(2,2) - this->IPntS1_S1(0,0))*(this->sHat11_P*this->sHat13_P.transpose() + this->sHat13_P*this->sHat11_P.transpose());
    this->IS2PrimePntS2_P = (this->theta1Dot+this->theta2Dot)*(this->IPntS2_S2(2,2) - this->IPntS2_S2(0,0))*(this->sHat21_P*this->sHat23_P.transpose() + this->sHat23_P*this->sHat21_P.transpose());
    // - Find body time derivative of IPntB_B
    this->effProps.IEffPrimePntB_B = this->IS1PrimePntS1_P - this->mass1*(this->rPrimeTildeS1P_P*this->rTildeS1P_P + this->rTildeS1P_P*this->rPrimeTildeS1P_P) + this->IS2PrimePntS2_P - this->mass2*(this->rPrimeTildeS2P_P*this->rTildeS2P_P + this->rTildeS2P_P*this->rPrimeTildeS2P_P);

    return;
}

void DualHingedRigidBodyStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    Eigen::MRPd sigmaPNLocal;
    Eigen::Matrix3d dcmPN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcmNP;                        /* direction cosine matrix from B to N */
    Eigen::Vector3d gravityTorquePntH1_P;          /* torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gravityTorquePntH2_P;          /* torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_P;                          /* gravitational acceleration in B frame */
    gLocal_N = *this->g_N;

    // - Find dcmBN
    this->sigma_BN = sigma_BN;
    sigmaPNLocal = this->sigma_BN;
    dcmNP = sigmaPNLocal.toRotationMatrix();
    dcmPN = dcmNP.transpose();
    // - Map gravity to body frame
    g_P = dcmPN*gLocal_N;

    // - Define gravity terms
    Eigen::Vector3d gravTorquePan1PntH1 = -this->d1*this->sHat11_P.cross(this->mass1*g_P);
    Eigen::Vector3d gravForcePan2 = this->mass2*g_P;
    Eigen::Vector3d gravTorquePan2PntH2 = -this->d2*this->sHat21_P.cross(this->mass2*g_P);

    // - Define omegaBN_S
    this->omega_BN_B = omega_BN_B;
    this->omega_PNLoc_P = this->omega_BN_B;
    this->omega_PN_S1 = this->dcm_S1P*this->omega_PNLoc_P;
    this->omega_PN_S2 = this->dcm_S2P*this->omega_PNLoc_P;
    // - Define omegaTildeBNLoc_B
    this->omegaTildePNLoc_P = eigenTilde(this->omega_PNLoc_P);
    // - Define matrices needed for back substitution
    //gravityTorquePntH1_B = -this->d1*this->sHat11_B.cross(this->mass1*g_B); //Need to review these equations and implement them - SJKC
    //gravityTorquePntH2_B = -this->d2*this->sHat21_B.cross(this->mass2*g_B); //Need to review these equations and implement them - SJKC
    this->matrixADHRB(0,0) = this->IPntS1_S1(1,1) + this->mass1*this->d1*this->d1 + this->mass2*this->l1*this->l1 + this->mass2*this->l1*this->d2*this->sHat13_P.transpose()*(this->sHat23_P);
    this->matrixADHRB(0,1) = this->mass2*this->l1*this->d2*this->sHat13_P.transpose()*(this->sHat23_P);
    this->matrixADHRB(1,0) = IPntS2_S2(1,1) + this->mass2*this->d2*this->d2 + this->mass2*this->l1*this->d2*this->sHat23_P.transpose()*this->sHat13_P;
    this->matrixADHRB(1,1) = this->IPntS2_S2(1,1) + this->mass2*this->d2*this->d2;
    this->matrixEDHRB = this->matrixADHRB.inverse();
    this->matrixFDHRB.row(0) = -(this->mass2*this->l1 + this->mass1*this->d1)*this->sHat13_P.transpose();
    this->matrixFDHRB.row(1) = -this->mass2*this->d2*this->sHat23_P.transpose();

    this->matrixGDHRB.row(0) = -(this->IPntS1_S1(1,1)*this->sHat12_P.transpose() - this->mass1*this->d1*this->sHat13_P.transpose()*this->rTildeS1P_P - this->mass2*this->l1*this->sHat13_P.transpose()*this->rTildeS2P_P);
    this->matrixGDHRB.row(1) = -(this->IPntS2_S2(1,1)*this->sHat22_P.transpose() - this->mass2*this->d2*this->sHat23_P.transpose()*this->rTildeS2P_P);

    this->vectorVDHRB(0) =  -(this->IPntS1_S1(0,0) - this->IPntS1_S1(2,2))*this->omega_PN_S1(2)*this->omega_PN_S1(0)
                            + this->u1 - this->k1*this->theta1 - this->c1*this->theta1Dot + this->k2*this->theta2 + this->c2*this->theta2Dot + this->sHat12_P.dot(gravTorquePan1PntH1) + this->l1*this->sHat13_P.dot(gravForcePan2) -
                            this->mass1*this->d1*this->sHat13_P.transpose()*(2*this->omegaTildePNLoc_P*this->rPrimeS1P_P + this->omegaTildePNLoc_P*this->omegaTildePNLoc_P*this->r_S1P_P)
                            - this->mass2*this->l1*this->sHat13_P.transpose()*(2*this->omegaTildePNLoc_P*this->rPrimeS2P_P + this->omegaTildePNLoc_P*this->omegaTildePNLoc_P*this->r_S2P_P + this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_P + this->d2*(this->theta1Dot + this->theta2Dot)*(this->theta1Dot + this->theta2Dot)*this->sHat21_P); //still missing torque and force terms - SJKC

    this->vectorVDHRB(1) =  -(this->IPntS2_S2(0,0) - this->IPntS2_S2(2,2))*this->omega_PN_S2(2)*this->omega_PN_S2(0)
                            + this->u2 - this->k2*this->theta2 - this->c2*this->theta2Dot + this->sHat22_P.dot(gravTorquePan2PntH2) - this->mass2*this->d2*this->sHat23_P.transpose()*(2*this->omegaTildePNLoc_P*this->rPrimeS2P_P + this->omegaTildePNLoc_P*this->omegaTildePNLoc_P*this->r_S2P_P + this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_P); // still missing torque term. - SJKC

    // - Start defining them good old contributions - start with translation
    // - For documentation on contributions see Allard, Diaz, Schaub flex/slosh paper
    backSubContr.matrixA = (this->mass1*this->d1*this->sHat13_P + this->mass2*this->l1*this->sHat13_P + this->mass2*this->d2*this->sHat23_P)*matrixEDHRB.row(0)*this->matrixFDHRB + this->mass2*this->d2*this->sHat23_P*this->matrixEDHRB.row(1)*this->matrixFDHRB;
    backSubContr.matrixB = (this->mass1*this->d1*this->sHat13_P + this->mass2*this->l1*this->sHat13_P + this->mass2*this->d2*this->sHat23_P)*this->matrixEDHRB.row(0)*(matrixGDHRB) + this->mass2*this->d2*this->sHat23_P*this->matrixEDHRB.row(1)*(matrixGDHRB);
    backSubContr.vecTrans = -(this->mass1*this->d1*this->theta1Dot*this->theta1Dot*this->sHat11_P + this->mass2*(this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_P + this->d2*(this->theta1Dot+this->theta2Dot)*(this->theta1Dot+this->theta2Dot)*this->sHat21_P)
                    + (this->mass1*this->d1*this->sHat13_P + this->mass2*this->l1*this->sHat13_P + this->mass2*this->d2*this->sHat23_P)*this->matrixEDHRB.row(0)*this->vectorVDHRB + this->mass2*this->d2*this->sHat23_P*this->matrixEDHRB.row(1)*this->vectorVDHRB);

    // - Define rotational matrice contributions (Eq 96 in paper)

    backSubContr.matrixC = (this->IPntS1_S1(1,1)*this->sHat12_P + this->mass1*this->d1*this->rTildeS1P_P*this->sHat13_P + this->IPntS2_S2(1,1)*this->sHat22_P + this->mass2*this->l1*this->rTildeS2P_P*this->sHat13_P + this->mass2*this->d2*this->rTildeS2P_P*this->sHat23_P)*this->matrixEDHRB.row(0)*this->matrixFDHRB
                    + (this->IPntS2_S2(1,1)*this->sHat22_P + this->mass2*this->d2*this->rTildeS2P_P*this->sHat23_P)*this->matrixEDHRB.row(1)*this->matrixFDHRB;

    backSubContr.matrixD = (this->IPntS1_S1(1,1)*this->sHat12_P + this->mass1*this->d1*this->rTildeS1P_P*this->sHat13_P + this->IPntS2_S2(1,1)*this->sHat22_P + this->mass2*this->l1*this->rTildeS2P_P*this->sHat13_P + this->mass2*this->d2*this->rTildeS2P_P*this->sHat23_P)*this->matrixEDHRB.row(0)*this->matrixGDHRB
                    +(this->IPntS2_S2(1,1)*this->sHat22_P + this->mass2*this->d2*this->rTildeS2P_P*this->sHat23_P)*this->matrixEDHRB.row(1)*this->matrixGDHRB;

    backSubContr.vecRot = -(this->theta1Dot*this->IPntS1_S1(1,1)*this->omegaTildePNLoc_P*this->sHat12_P
                    + this->mass1*this->omegaTildePNLoc_P*this->rTildeS1P_P*this->rPrimeS1P_P + this->mass1*this->d1*this->theta1Dot*this->theta1Dot*this->rTildeS1P_P*this->sHat11_P + (this->theta1Dot+this->theta2Dot)*this->IPntS2_S2(1,1)*this->omegaTildePNLoc_P*this->sHat22_P + this->mass2*this->omegaTildePNLoc_P*this->rTildeS2P_P*this->rPrimeS2P_P
                    + this->mass2*this->rTildeS2P_P*(this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_P + this->d2*(this->theta1Dot+this->theta2Dot)*(this->theta1Dot+this->theta2Dot)*this->sHat21_P) + (this->IPntS1_S1(1,1)*this->sHat12_P + this->mass1*this->d1*this->rTildeS1P_P*this->sHat13_P + this->IPntS2_S2(1,1)*this->sHat22_P
                    + this->mass2*this->l1*this->rTildeS2P_P*this->sHat13_P + this->mass2*this->d2*this->rTildeS2P_P*this->sHat23_P)*this->matrixEDHRB.row(0)*this->vectorVDHRB + (this->IPntS2_S2(1,1)*this->sHat22_P + this->mass2*this->d2*this->rTildeS2P_P*this->sHat23_P)*this->matrixEDHRB.row(1)*this->vectorVDHRB);

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
    this->sigma_BN = sigma_BN;
    sigmaBNLocal = this->sigma_BN;
    omegaDotBNLoc_B = omegaDot_BN_B;
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
void DualHingedRigidBodyStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_P, double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // - Get the current omega state
    Eigen::Vector3d omegaLocal_PN_P;
    this->omega_BN_B = omega_BN_B;
    omegaLocal_PN_P = this->omega_BN_B;

    // - Find rotational angular momentum contribution from hub
    Eigen::Vector3d omega_S1P_P;
    Eigen::Vector3d omega_S2P_P;
    Eigen::Vector3d omega_S1N_P;
    Eigen::Vector3d omega_S2N_P;
    Eigen::Matrix3d IPntS1_P;
    Eigen::Matrix3d IPntS2_P;
    Eigen::Vector3d rDot_S1P_P;
    Eigen::Vector3d rDot_S2P_P;
    omega_S1P_P = this->theta1Dot*this->sHat12_P;
    omega_S2P_P = (this->theta1Dot + this->theta2Dot)*this->sHat22_P;
    omega_S1N_P = omega_S1P_P + omegaLocal_PN_P;
    omega_S2N_P = omega_S2P_P + omegaLocal_PN_P;
    IPntS1_P = this->dcm_S1P.transpose()*this->IPntS1_S1*this->dcm_S1P;
    IPntS2_P = this->dcm_S2P.transpose()*this->IPntS2_S2*this->dcm_S2P;
    rDot_S1P_P = this->rPrimeS1P_P + omegaLocal_PN_P.cross(this->r_S1P_P);
    rDot_S2P_P = this->rPrimeS2P_P + omegaLocal_PN_P.cross(this->r_S2P_P);
    rotAngMomPntCContr_P = IPntS1_P*omega_S1N_P + this->mass1*this->r_S1P_P.cross(rDot_S1P_P)
                            + IPntS2_P*omega_S2N_P + this->mass2*this->r_S2P_P.cross(rDot_S2P_P);

    // - Find rotational energy contribution from the hub
    double rotEnergyContrS1;
    double rotEnergyContrS2;
    rotEnergyContrS1 =  0.5*omega_S1N_P.dot(IPntS1_P*omega_S1N_P)
                        + 0.5*this->mass1*rDot_S1P_P.dot(rDot_S1P_P)
                        + 0.5*this->k1*this->theta1*this->theta1;
    rotEnergyContrS2 =  0.5*omega_S2N_P.dot(IPntS2_P*omega_S2N_P)
                        + 0.5*this->mass2*rDot_S2P_P.dot(rDot_S2P_P)
                        + 0.5*this->k2*this->theta2*this->theta2;
    rotEnergyContr = rotEnergyContrS1 + rotEnergyContrS2;

    return;
}

/*! This method takes the computed theta states and outputs them to the m
 messaging system.

 @param CurrentClock The current simulation time (used for time stamping)
 */
void DualHingedRigidBodyStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{

    HingedRigidBodyMsgPayload panelOutputStates;  //!< instance of messaging system message struct

    // panel 1 states
    panelOutputStates.theta = this->theta1;
    panelOutputStates.thetaDot = this->theta1Dot;
    this->dualHingedRigidBodyOutMsgs[0]->write(&panelOutputStates, this->moduleID, CurrentClock);
    // panel 2 states
    panelOutputStates.theta = this->theta2;
    panelOutputStates.thetaDot = this->theta2Dot;
    this->dualHingedRigidBodyOutMsgs[1]->write(&panelOutputStates, this->moduleID, CurrentClock);


    // write out the panel state config log message
    SCStatesMsgPayload configLogMsg;
    // Note, logging the hinge frame S is the body frame B of that object
    for (int i=0; i<2; i++) {
        configLogMsg = this->dualHingedRigidBodyConfigLogOutMsgs[i]->zeroMsgPayload;
        eigenVector3d2CArray(this->r_SN_N[i], configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_SN_N[i], configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_SN[i], configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_SN_S[i], configLogMsg.omega_BN_B);
        this->dualHingedRigidBodyConfigLogOutMsgs[i]->write(&configLogMsg, this->moduleID, CurrentClock);
    }
}


/*! This method is used so that the simulation will ask DHRB to update messages.

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void DualHingedRigidBodyStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Zero the command buffer and read the incoming command array
    if (this->motorTorqueInMsg.isLinked()) {
        ArrayMotorTorqueMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorTorqueInMsg();
        this->u1 = incomingCmdBuffer.motorTorque[0];
        this->u2 = incomingCmdBuffer.motorTorque[1];
    }

    /* compute panel inertial states */
    this->computePanelInertialStates();

    this->writeOutputStateMessages(CurrentSimNanos);

    return;
}

/*! This method computes the panel states relative to the inertial frame

 */
void DualHingedRigidBodyStateEffector::computePanelInertialStates()
{
    // inertial attitudes
    Eigen::MRPd sigmaPN;
    sigmaPN = this->sigma_BN;
    Eigen::Matrix3d dcm_NP = sigmaPN.toRotationMatrix();
    this->sigma_SN[0] = eigenMRPd2Vector3d(eigenC2MRP(this->dcm_S1P*dcm_NP.transpose()));
    this->sigma_SN[1] = eigenMRPd2Vector3d(eigenC2MRP(this->dcm_S2P*dcm_NP.transpose()));

    // inertial angular velocities
    Eigen::Vector3d omega_PN_P;
    omega_PN_P = this->omega_BN_B;
    this->omega_SN_S[0] = this->dcm_S1P * ( omega_PN_P + this->theta1Dot*this->sHat12_P);
    this->omega_SN_S[1] = this->dcm_S1P * ( omega_PN_P + this->theta2Dot*this->sHat22_P);

    // inertial position vectors
    Eigen::Vector3d r_PN_N;
    r_PN_N = (Eigen::Vector3d)(*this->inertialPositionProperty);
    this->r_SN_N[0] = (dcm_NP * this->r_S1P_P) + r_PN_N;
    this->r_SN_N[1] = (dcm_NP * this->r_S2P_P) + r_PN_N;

    // inertial velocity vectors
    Eigen::Vector3d omega_S1N_P = this->theta1Dot * this->sHat12_P + omega_PN_P;
    this->v_SN_N[0] = (Eigen::Vector3d)(*this->inertialVelocityProperty)
                    + omega_S1N_P.cross( -this->d1 * this->sHat11_P)
                    + omega_PN_P.cross(this->r_H1P_P);
    Eigen::Vector3d omega_S2N_P = this->theta2Dot * this->sHat22_P + omega_S1N_P;
    this->v_SN_N[1] = (Eigen::Vector3d)(*this->inertialVelocityProperty)
                    + omega_S2N_P.cross( -this->d2 * this->sHat21_P)
                    + omega_S1N_P.cross( -this->l1 * this->sHat11_P)
                    + omega_PN_P.cross(this->r_H1P_P);

    return;
}
