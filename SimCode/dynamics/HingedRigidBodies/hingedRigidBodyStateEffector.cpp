/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "hingedRigidBodyStateEffector.h"

HingedRigidBodyStateEffector::HingedRigidBodyStateEffector()
{
    //! - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    //! - Initialize the variables to working values
    this->mass = 0.0;
    this->d = 1.0;
    this->k = 1.0;
    this->c = 0.0;
    this->IPntS_S.Identity();
    this->rHB_B.setZero();
    this->dcm_HB.Identity();
    this->nameOfThetaState = "hingedRigidBodyTheta";
    this->nameOfThetaDotState = "hingedRigidBodyThetaDot";

    return;
}


HingedRigidBodyStateEffector::~HingedRigidBodyStateEffector()
{
    return;
}

void HingedRigidBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    //! - Get access to the hubs sigma, omegaBN_B and velocity needed for dynamic coupling
    this->hubVelocity = statesIn.getStateObject("hubVelocity");
    this->hubSigma = statesIn.getStateObject("hubSigma");
    this->hubOmega = statesIn.getStateObject("hubOmega");
    this->g_N = statesIn.getPropertyReference("g_N");

    return;
}

void HingedRigidBodyStateEffector::registerStates(DynParamManager& states)
{
    //! - Register the states associated with hinged rigid bodies - theta and thetaDot
    this->thetaState = states.registerState(1, 1, this->nameOfThetaState);
    this->thetaDotState = states.registerState(1, 1, this->nameOfThetaDotState);

    return;
}

void HingedRigidBodyStateEffector::updateEffectorMassProps(double integTime)
{
    //! - Give the mass of the hinged rigid body to the effProps mass
    this->effProps.mEff = this->mass;

    //! - find hinged rigid bodies' position with respect to point B
    //! - First need to grab current states
    this->theta = this->thetaState->getState()(0, 0);
    this->thetaDot = this->thetaDotState->getState()(0, 0);
    //! - Next find the sHat unit vectors
    this->dcm_SH << cos(this->theta), 0, -sin(this->theta), 0, 1, 0, sin(this->theta), 0, cos(this->theta);
    this->dcm_SB = this->dcm_SH*this->dcm_HB;
    this->sHat1_B = this->dcm_SB.row(0);
    this->sHat2_B = this->dcm_SB.row(1);
    this->sHat3_B = this->dcm_SB.row(2);
    this->rSB_B = this->rHB_B - this->d*this->sHat1_B;
    this->effProps.rEff_CB_B = this->rSB_B;

    //! - Find the inertia of the hinged rigid body about point B
    //! - Define rTildeSB_B
    this->rTildeSB_B << 0 , -this->rSB_B(2), this->rSB_B(1), this->rSB_B(2), 0, -this->rSB_B(0), -this->rSB_B(1), this->rSB_B(0), 0;
    this->effProps.IEffPntB_B = this->dcm_SB.transpose()*this->IPntS_S*this->dcm_SB + this->mass*this->rTildeSB_B*this->rTildeSB_B.transpose();

    //! First, find the rPrimeSB_B
    this->rPrimeSB_B = this->d*this->thetaDot*this->sHat3_B;
    this->effProps.rEffPrime_CB_B = this->rPrimeSB_B;

    //! - Next find the body time derivative of the inertia about point B
    //! - Define tilde matrix of rPrimeSB_B
    this->rPrimeTildeSB_B << 0 , -this->rPrimeSB_B(2), this->rPrimeSB_B(1), this->rPrimeSB_B(2), 0, -this->rPrimeSB_B(0), -this->rPrimeSB_B(1), this->rPrimeSB_B(0), 0;
    //! - Find body time derivative of IPntS_B
    this->ISPrimePntS_B = this->thetaDot*(this->IPntS_S(2,2) - this->IPntS_S(0,0))*(this->sHat1_B*this->sHat3_B.transpose() + this->sHat3_B*this->sHat1_B.transpose());
    //! - Find body time derivative of IPntB_B
    this->effProps.IEffPrimePntB_B = this->ISPrimePntS_B - this->mass*(this->rPrimeTildeSB_B*this->rTildeSB_B + this->rTildeSB_B*this->rPrimeTildeSB_B);

    return;
}

void HingedRigidBodyStateEffector::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr)
{
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
    Eigen::Vector3d gravityTorquePntH_B;          /* torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_B;                          /* gravitational acceleration in B frame */
    gLocal_N = *this->g_N;

    //! - Find dcm_BN
    sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
    dcm_NB = sigmaBNLocal.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();
    //! - Map gravity to body frame
    g_B = dcm_BN*gLocal_N;

    //! - Define omegaBN_S
    this->omegaBNLoc_B = this->hubOmega->getState();
    this->omegaBN_S = this->dcm_SB*this->omegaBNLoc_B;
    //! - Define omegaTildeBNLoc_B
    this->omegaTildeBNLoc_B << 0 , -this->omegaBNLoc_B(2), this->omegaBNLoc_B(1), this->omegaBNLoc_B(2), 0, -this->omegaBNLoc_B(0), -this->omegaBNLoc_B(1), this->omegaBNLoc_B(0), 0;
    //! - Define a_theta
    gravityTorquePntH_B = -this->d*this->sHat1_B.cross(this->mass*g_B);
    this->a_theta = -this->k*this->theta - this->c*this->thetaDot + this->sHat2_B.dot(gravityTorquePntH_B) + (this->IPntS_S(2,2) - this->IPntS_S(0,0) + this->mass*this->d*this->d)*this->omegaBN_S(2)*this->omegaBN_S(0) - this->mass*this->d*this->sHat3_B.transpose()*this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->rHB_B;

    //! - Start defining them good old contributions - start with translation
    //! - For documentation on contributions see Allard, Diaz, Schaub flex/slosh paper
    matrixAcontr = -(this->mass*this->mass*this->d*this->d*this->sHat3_B*this->sHat3_B.transpose()/(this->IPntS_S(1,1) + this->mass*this->d*this->d));
    //! - need to define rTildeHB_B
    this->rTildeHB_B << 0 , -this->rHB_B(2), this->rHB_B(1), this->rHB_B(2), 0, -this->rHB_B(0), -this->rHB_B(1), this->rHB_B(0), 0;
    matrixBcontr = -(this->mass*this->d*sHat3_B/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*((this->IPntS_S(1,1)+this->mass*this->d*this->d)*this->sHat2_B.transpose() - this->mass*this->d*this->sHat3_B.transpose()*this->rTildeHB_B));
    vecTranscontr = -(this->mass*this->d*this->a_theta*this->sHat3_B/(this->IPntS_S(1,1) + this->mass*this->d*this->d) + this->mass*this->d*this->thetaDot*this->thetaDot*this->sHat1_B);

    //! - Define rotational matrice contributions
    matrixCcontr = -(this->IPntS_S(1,1)*this->sHat2_B + this->mass*this->d*this->rTildeSB_B*this->sHat3_B)/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*this->mass*this->d*this->sHat3_B.transpose();
    matrixDcontr = -(this->IPntS_S(1,1)*this->sHat2_B + this->mass*this->d*this->rTildeSB_B*this->sHat3_B)/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*((this->IPntS_S(1,1)+this->mass*this->d*this->d)*this->sHat2_B.transpose() - this->mass*this->d*this->sHat3_B.transpose()*this->rTildeHB_B);
    vecRotcontr = -(this->thetaDot*this->omegaTildeBNLoc_B*(this->IPntS_S(1,1)*this->sHat2_B+this->mass*this->d*this->rTildeSB_B*this->sHat3_B) + this->mass*this->d*this->thetaDot*this->thetaDot*this->rTildeSB_B*this->sHat1_B + this->a_theta*(this->IPntS_S(1,1)*this->sHat2_B + this->mass*this->d*this->rTildeSB_B*this->sHat3_B)/(this->IPntS_S(1,1) + this->mass*this->d*this->d));

    return;
}

void HingedRigidBodyStateEffector::computeDerivatives(double integTime)
{
    //! - Define necessarry variables
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
    Eigen::MatrixXd thetaDDot(1,1);               /* thetaDDot variable to send to state manager */
    Eigen::Vector3d rDDotBNLoc_N;                 /* second time derivative of rBN in N frame */
    Eigen::Vector3d rDDotBNLoc_B;                 /* second time derivative of rBN in B frame */
    Eigen::Vector3d omegaDotBNLoc_B;              /* time derivative of omegaBN in B frame */

    //! Grab necessarry values from manager (these have been previously computed in hubEffector)
    rDDotBNLoc_N = this->hubVelocity->getStateDeriv();
    sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
    omegaDotBNLoc_B = this->hubOmega->getStateDeriv();
    dcm_NB = sigmaBNLocal.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();
    rDDotBNLoc_B = dcm_BN*rDDotBNLoc_N;

    //! - Compute Derivatives
    //! - First is trivial
    this->thetaState->setDerivative(thetaDotState->getState());
    //! - Second, a little more involved - see Allard, Diaz, Schaub flex/slosh paper
    thetaDDot(0,0) = 1.0/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*(-this->mass*this->d*this->sHat3_B.dot(rDDotBNLoc_B) - (this->IPntS_S(1,1) + this->mass*this->d*this->d)*this->sHat2_B.transpose()*omegaDotBNLoc_B + this->mass*this->d*this->sHat3_B.transpose()*this->rTildeHB_B*omegaDotBNLoc_B + this->a_theta);
    this->thetaDotState->setDerivative(thetaDDot);

    return;
}

void HingedRigidBodyStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr)
{
    // Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = hubOmega->getState();
    Eigen::Vector3d omegaSB_B;
    Eigen::Vector3d omegaSN_B;
    Eigen::Matrix3d IPntS_B;
    Eigen::Vector3d rDotSB_B;

    // Call mass props to get current information on states
    this->updateEffectorMassProps(integTime);

    // Find rotational angular momentum contribution from hub
    omegaSB_B = this->thetaDot*this->sHat2_B;
    omegaSN_B = omegaSB_B + omegaLocal_BN_B;
    IPntS_B = this->dcm_SB.transpose()*this->IPntS_S*this->dcm_SB;
    rDotSB_B = this->rPrimeSB_B + omegaLocal_BN_B.cross(this->rSB_B);
    rotAngMomPntCContr_B = IPntS_B*omegaSN_B + this->mass*this->rSB_B.cross(rDotSB_B);

    // Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*omegaSN_B.dot(IPntS_B*omegaSN_B) + 1.0/2.0*this->mass*rDotSB_B.dot(rDotSB_B) + 1.0/2.0*this->k*this->theta*this->theta;
    return;
}
