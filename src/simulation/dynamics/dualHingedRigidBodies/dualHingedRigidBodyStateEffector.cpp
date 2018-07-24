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
    this->mass2 = 0.0;
    this->d2 = 1.0;
    this->k2 = 1.0;
    this->c2 = 0.0;
    this->theta1Init = 0.0;
    this->theta1DotInit = 0.0;
    this->theta2Init = 0.0;
    this->theta2DotInit = 0.0;
    this->IPntS1_S1.setIdentity();
    this->IPntS2_S2.setIdentity();
    this->rH1B_B.setZero();
    this->dcmH1B.setIdentity();
    this->thetaH2S1 = 0.0;
    this->nameOfTheta1State = "hingedRigidBodyTheta1";
    this->nameOfTheta1DotState = "hingedRigidBodyTheta1Dot";
    this->nameOfTheta2State = "hingedRigidBodyTheta2";
    this->nameOfTheta2DotState = "hingedRigidBodyTheta2Dot";

    return;
}


DualHingedRigidBodyStateEffector::~DualHingedRigidBodyStateEffector()
{
    return;
}

void DualHingedRigidBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    // - Get access to the hubs sigma, omegaBN_B and velocity needed for dynamic coupling
    this->hubVelocity = statesIn.getStateObject("hubVelocity");
    this->hubSigma = statesIn.getStateObject("hubSigma");
    this->hubOmega = statesIn.getStateObject("hubOmega");
    this->g_N = statesIn.getPropertyReference("g_N");

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
    this->dcmS1B = dcmS1H1*this->dcmH1B;
    Eigen::Matrix3d dcmH2S1;
    dcmH2S1 = eigenM2(this->thetaH2S1);
    Eigen::Matrix3d dcmH2B;
    dcmH2B = dcmH2S1*this->dcmS1B;
    Eigen::Matrix3d dcmS2H2;
    dcmS2H2 = eigenM2(this->theta2);
    this->dcmS2B = dcmS2H2 * dcmH2B;
    this->sHat11_B = this->dcmS1B.row(0);
    this->sHat12_B = this->dcmS1B.row(1);
    this->sHat13_B = this->dcmS1B.row(2);
    this->sHat21_B = this->dcmS2B.row(0);
    this->sHat22_B = this->dcmS2B.row(1);
    this->sHat23_B = this->dcmS2B.row(2);
    this->rS1B_B = this->rH1B_B - this->d1*this->sHat11_B;
    this->rS2B_B = this->rH1B_B - this->l1*this->sHat11_B - this->d2*this->sHat21_B;
    this->effProps.rEff_CB_B = 1.0/this->effProps.mEff*(this->mass1*this->rS1B_B + this->mass2*this->rS2B_B);

    // - Find the inertia of the hinged rigid body about point B
    // - Define rTildeSB_B
    this->rTildeS1B_B = eigenTilde(this->rS1B_B);
    this->rTildeS2B_B = eigenTilde(this->rS2B_B);
    this->effProps.IEffPntB_B = this->dcmS1B.transpose()*this->IPntS1_S1*this->dcmS1B + this->mass1*this->rTildeS1B_B*this->rTildeS1B_B.transpose() + this->dcmS2B.transpose()*this->IPntS2_S2*this->dcmS2B + this->mass2*this->rTildeS2B_B*this->rTildeS2B_B.transpose();

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
    sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
    dcmNB = sigmaBNLocal.toRotationMatrix();
    dcmBN = dcmNB.transpose();
    // - Map gravity to body frame
    g_B = dcmBN*gLocal_N;

    // - Define gravity terms
    Eigen::Vector3d gravTorquePan1PntH1 = -this->d1*this->sHat11_B.cross(this->mass1*g_B);
    Eigen::Vector3d gravForcePan2 = this->mass2*g_B;
    Eigen::Vector3d gravTorquePan2PntH2 = -this->d2*this->sHat21_B.cross(this->mass2*g_B);

    // - Define omegaBN_S
    this->omegaBNLoc_B = this->hubOmega->getState();
    this->omegaBN_S1 = this->dcmS1B*this->omegaBNLoc_B;
    this->omegaBN_S2 = this->dcmS2B*this->omegaBNLoc_B;
    // - Define omegaTildeBNLoc_B
    this->omegaTildeBNLoc_B = eigenTilde(this->omegaBNLoc_B);
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

    this->vectorVDHRB(0) =  -(this->IPntS1_S1(0,0) - this->IPntS1_S1(2,2))*this->omegaBN_S1(2)*this->omegaBN_S1(0) - this->k1*this->theta1 - this->c1*this->theta1Dot + this->k2*this->theta2 + this->c2*this->theta2Dot + this->sHat12_B.dot(gravTorquePan1PntH1) + this->l1*this->sHat13_B.dot(gravForcePan2)
                            - this->mass1*this->d1*this->sHat13_B.transpose()*(2*this->omegaTildeBNLoc_B*this->rPrimeS1B_B + this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->rS1B_B)
                            - this->mass2*this->l1*this->sHat13_B.transpose()*(2*this->omegaTildeBNLoc_B*this->rPrimeS2B_B + this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->rS2B_B + this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_B + this->d2*(this->theta1Dot + this->theta2Dot)*(this->theta1Dot + this->theta2Dot)*this->sHat21_B); //still missing torque and force terms - SJKC

    this->vectorVDHRB(1) =  -(this->IPntS2_S2(0,0) - this->IPntS2_S2(2,2))*this->omegaBN_S2(2)*this->omegaBN_S2(0)
                            - this->k2*this->theta2 - this->c2*this->theta2Dot + this->sHat22_B.dot(gravTorquePan2PntH2) - this->mass2*this->d2*this->sHat23_B.transpose()*(2*this->omegaTildeBNLoc_B*this->rPrimeS2B_B + this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->rS2B_B + this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_B); // still missing torque term. - SJKC

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
    rDDotBNLoc_N = this->hubVelocity->getStateDeriv();
    sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
    omegaDotBNLoc_B = this->hubOmega->getStateDeriv();
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
    omegaLocal_BN_B = hubOmega->getState();
    
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
    IPntS1_B = this->dcmS1B.transpose()*this->IPntS1_S1*this->dcmS1B;
    IPntS2_B = this->dcmS2B.transpose()*this->IPntS2_S2*this->dcmS2B;
    rDot_S1B_B = this->rPrimeS1B_B + omegaLocal_BN_B.cross(this->rS1B_B);
    rDot_S2B_B = this->rPrimeS2B_B + omegaLocal_BN_B.cross(this->rS2B_B);
    rotAngMomPntCContr_B = IPntS1_B*omega_S1N_B + this->mass1*this->rS1B_B.cross(rDot_S1B_B)
                            + IPntS2_B*omega_S2N_B + this->mass2*this->rS2B_B.cross(rDot_S2B_B);
    
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
