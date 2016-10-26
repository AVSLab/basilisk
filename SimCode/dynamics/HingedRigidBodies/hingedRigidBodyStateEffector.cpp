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
    effProps.IEffPntBPrime_B.fill(0.0);
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
    return;
}

void HingedRigidBodyStateEffector::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr, Eigen::Vector3d & vecRotcontr)
{
    return;
}
