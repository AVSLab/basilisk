/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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
    effProps.IEffPntB_B.fill(0.0);
    effProps.rCB_B.fill(0.0);
    effProps.mEff = 0.0;
    effProps.IEffPrimePntB_B.fill(0.0);
    return;
}


HingedRigidBodyStateEffector::~HingedRigidBodyStateEffector()
{
    return;
}

void HingedRigidBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->hubSigma = statesIn.getStateObject("hubPosition");
    this->hubOmega = statesIn.getStateObject("hubSigma");
}

void HingedRigidBodyStateEffector::registerStates(DynParamManager& states)
{
    this->thetaState = states.registerState(1, 1, "hingedRigidBodyTheta");
    this->thetaDotState = states.registerState(1, 1, "hingedRigidBodyThetaDot");
}

void HingedRigidBodyStateEffector::computeDerivatives(double integTime)
{
}

void HingedRigidBodyStateEffector::updateEffectorMassProps(double integTime)
{
    //! - Give the mass of the hinged rigid body to the effProps mass
    effProps.mEff = this->mass;
    //! - find hinged rigid bodies position with respect to point B
    Eigen::MatrixXd interMediateMatrix;
    interMediateMatrix = this->thetaState->getState();
    this->theta = interMediateMatrix(0,0);
    interMediateMatrix = this->thetaDotState->getState();
    this->thetaDot = interMediateMatrix(0,0);
    //! - Find the sHat unit vectors
    this->SH << cos(theta), 0, -sin(theta), 0, 1, 0, sin(theta), 0, cos(theta);
    this->SB = this->SH*this->HB;
    this->sHat1_B = this->SB.row(0);
    this->sHat2_B = this->SB.row(1);
    this->sHat3_B = this->SB.row(2);
    this->rSB_B = this->rHB_B -this->d*this->sHat1_B;
    effProps.rCB_B = this->rSB_B;

    //! - Find the inertia of the hinged rigid body about point B
    //! - Define rTildeSB_B
    this->rTildeSB_B << 0 , -rSB_B(2), rSB_B(1), rSB_B(2), 0, -rSB_B(0), -rSB_B(1), rSB_B(0), 0;
    effProps.IEffPntB_B = this->SB.transpose()*this->IPntS_S*this->SB + this->mass*this->rTildeSB_B*this->rTildeSB_B.transpose();
    return;
}

void HingedRigidBodyStateEffector::updateEffectorMassPropRates(double integTime)
{
    //! - Find the body time derivative of the inertia about point B
    //! First, find the rPrimeSB_B
    this->rPrimeSB_B = this->d*this->thetaDot*this->sHat3_B;
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
    //! - Need to define omegaBN_S
    this->omegaBN_S = this->SB*omegaBNLoc_B;
    //! - Need to define omegaTildeBNLoc_B
    this->omegaTildeBNLoc_B << 0 , -this->omegaBNLoc_B(2), this->omegaBNLoc_B(1), this->omegaBNLoc_B(2), 0, -this->omegaBNLoc_B(0), -this->omegaBNLoc_B(1), this->omegaBNLoc_B(0), 0;
    //! - Need to define a_theta (need to add in g_N)
    this->a_theta = -this->k*this->theta - this->c*this->thetaDot + (this->IPntS_S(2,2) - this->IPntS_S(0,0) + this->mass*this->d*this->d)*this->omegaBN_S(2,2)*this->omegaBN_S(0,0) - this->mass*this->d*this->sHat3_B.transpose()*this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->rHB_B;

    //! - Start defining them good old contributions - start with translation
    matrixAcontr = this->mass*this->mass*this->d*this->d*sHat3_B*sHat3_B.transpose()/(this->IPntS_S(1,1) + this->mass*this->d*this->d);
    //! - need to define rTildeHB_B
    this->rTildeHB_B << 0 , -this->rHB_B(2), this->rHB_B(1), this->rHB_B(2), 0, -this->rHB_B(0), -this->rHB_B(1), this->rHB_B(0), 0;
    matrixBcontr = -this->mass*this->d*sHat3_B/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*((this->IPntS_S(1,1)+this->mass*this->d*this->d)*sHat2_B.transpose() - this->mass*this->d*sHat3_B.transpose()*this->rTildeHB_B);
    vecTranscontr = -this->mass*this->d*this->a_theta*this->sHat3_B/(this->IPntS_S(1,1) + this->mass*this->d*this->d) + this->mass*this->d*this->thetaDot*this->thetaDot*this->sHat1_B;

    //! - Define rotational matrice contributions
    matrixCcontr = -(this->IPntS_S(1,1)*this->sHat2_B + this->mass*this->d*this->rTildeSB_B*this->sHat3_B)/(this->IPntS_S(1,1) - this->mass*this->d*this->d)*this->mass*this->d*sHat3_B.transpose();
    matrixDcontr = -(this->IPntS_S(1,1)*this->sHat2_B + this->mass*this->d*this->rTildeSB_B*this->sHat3_B)/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*((this->IPntS_S(1,1)+this->mass*this->d*this->d)*this->sHat2_B.transpose() - this->mass*this->d*this->sHat3_B.transpose()*this->rTildeHB_B);
    vecRotcontr = -(this->thetaDot*this->omegaTildeBNLoc_B*(this->IPntS_S(1,1)*this->sHat2_B+this->mass*this->d*this->rTildeSB_B*this->sHat3_B) + this->mass*this->d*this->thetaDot*this->thetaDot*this->rTildeSB_B*this->sHat1_B + this->a_theta*(this->IPntS_S(1,1)*this->sHat2_B + this->mass*this->d*this->rTildeSB_B*this->sHat3_B)/(this->IPntS_S(1,1)+this->mass*this->d*this->d));

    return;
}
