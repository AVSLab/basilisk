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

using namespace Eigen;
using namespace std;

HingedRigidBodyStateEffector::HingedRigidBodyStateEffector()
{
    effProps.mEff = 0.0;
    effProps.rCB_B.fill(0.0);
    effProps.IEffPntB_B.fill(0.0);
    effProps.rPrimeCB_B.fill(0.0);
    effProps.IEffPrimePntB_B.fill(0.0);

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
}

void HingedRigidBodyStateEffector::registerStates(DynParamManager& states)
{
    //! - Register the states associated with hinged rigid bodies - theta and thetaDot
    this->thetaState = states.registerState(1, 1, this->nameOfThetaState);
    this->thetaDotState = states.registerState(1, 1, this->nameOfThetaDotState);
}

void HingedRigidBodyStateEffector::updateEffectorMassProps(double integTime)
{
    //! - Give the mass of the hinged rigid body to the effProps mass
    this->effProps.mEff = this->mass;

    //! - find hinged rigid bodies' position with respect to point B
    //! - First need to grab current states
    Eigen::MatrixXd interMediateMatrix(0,0);
    interMediateMatrix = this->thetaState->getState();
    this->theta = interMediateMatrix(0,0);
    interMediateMatrix = this->thetaDotState->getState();
    this->thetaDot = interMediateMatrix(0,0);
    //! - Next find the sHat unit vectors
    this->dcmSH << cos(this->theta), 0, -sin(this->theta), 0, 1, 0, sin(this->theta), 0, cos(this->theta);
    this->dcmSB = this->dcmSH*this->dcmHB;
    this->sHat1_B = this->dcmSB.row(0);
    this->sHat2_B = this->dcmSB.row(1);
    this->sHat3_B = this->dcmSB.row(2);
    this->rSB_B = this->rHB_B - this->d*this->sHat1_B;
    this->effProps.rCB_B = this->rSB_B;

    //! - Find the inertia of the hinged rigid body about point B
    //! - Define rTildeSB_B
    this->rTildeSB_B << 0 , -this->rSB_B(2), this->rSB_B(1), this->rSB_B(2), 0, -this->rSB_B(0), -this->rSB_B(1), this->rSB_B(0), 0;
    this->effProps.IEffPntB_B = this->dcmSB.transpose()*this->IPntS_S*this->dcmSB + this->mass*this->rTildeSB_B*this->rTildeSB_B.transpose();
    return;
}

void HingedRigidBodyStateEffector::updateEffectorMassPropRates(double integTime)
{
    //! First, find the rPrimeSB_B
    this->rPrimeSB_B = this->d*this->thetaDot*this->sHat3_B;
    this->effProps.rPrimeCB_B = this->rPrimeSB_B;

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
    //! - Define omegaBN_S
    this->omegaBNLoc_B = this->hubOmega->getState();
    this->omegaBN_S = this->dcmSB*this->omegaBNLoc_B;
    //! - Define omegaTildeBNLoc_B
    this->omegaTildeBNLoc_B << 0 , -this->omegaBNLoc_B(2), this->omegaBNLoc_B(1), this->omegaBNLoc_B(2), 0, -this->omegaBNLoc_B(0), -this->omegaBNLoc_B(1), this->omegaBNLoc_B(0), 0;
    //! - Define a_theta (need to add in g_N)
    this->a_theta = -this->k*this->theta - this->c*this->thetaDot + (this->IPntS_S(2,2) - this->IPntS_S(0,0) + this->mass*this->d*this->d)*this->omegaBN_S(2)*this->omegaBN_S(0) - this->mass*this->d*this->sHat3_B.transpose()*this->omegaTildeBNLoc_B*this->omegaTildeBNLoc_B*this->rHB_B;

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
    vecRotcontr = -(this->thetaDot*this->omegaTildeBNLoc_B*(this->IPntS_S(1,1)*this->sHat2_B+this->mass*this->d*this->rTildeSB_B*this->sHat3_B) + this->mass*this->d*this->thetaDot*this->thetaDot*this->rTildeSB_B*this->sHat1_B + this->a_theta*(this->IPntS_S(1,1)*this->sHat2_B + this->mass*this->d*this->rTildeSB_B*this->sHat3_B)/(this->IPntS_S(1,1)+this->mass*this->d*this->d));

    return;
}

void HingedRigidBodyStateEffector::computeDerivatives(double integTime)
{
    //! - Define necessarry variables
    MRPd sigmaBNLocal;
    Eigen::Matrix3d dcmBN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcmNB;                        /* direction cosine matrix from B to N */
    Eigen::MatrixXd thetaDDot(1,1);               /* thetaDDot variable to send to state manager */
    Eigen::Vector3d rDDotBNLoc_N;                 /* second time derivative of rBN in N frame */
    Eigen::Vector3d rDDotBNLoc_B;                 /* second time derivative of rBN in B frame */
    Eigen::Vector3d omegaDotBNLoc_B;              /* time derivative of omegaBN in B frame */

    //! Grab necessarry values from manager (these have been previously computed in hubEffector)
    rDDotBNLoc_N = this->hubVelocity->getStateDeriv();
    sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
    omegaDotBNLoc_B = this->hubOmega->getStateDeriv();
    dcmNB = sigmaBNLocal.toRotationMatrix();
    dcmBN = dcmNB.transpose();
    rDDotBNLoc_B = dcmBN*rDDotBNLoc_N;

    //! - Compute Derivatives
    //! - First is trivial
    thetaState->setDerivative(thetaDotState->getState());
    //! - Second, a little more involved - see Allard, Diaz, Schaub flex/slosh paper
    thetaDDot(0,0) = 1.0/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*(-this->mass*this->d*this->sHat3_B.dot(rDDotBNLoc_B) - (this->IPntS_S(1,1) + this->mass*this->d*this->d)*sHat2_B.transpose()*omegaDotBNLoc_B + this->mass*this->d*this->sHat3_B.transpose()*this->rTildeHB_B*omegaDotBNLoc_B + this->a_theta);
    thetaDotState->setDerivative(thetaDDot);
    
}
