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


#include "fuelSloshParticle.h"

#define tilde(vector) 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0

FuelSloshParticle::FuelSloshParticle()
{
	//! - zero the contributions for mass props and mass rates
	this->effProps.IEffPntB_B.fill(0.0);
	this->effProps.rCB_B.fill(0.0);
	this->effProps.rPrimeCB_B.fill(0.0);
	this->effProps.mEff = 0.0;
	this->effProps.IEffPrimePntB_B.fill(0.0);

	//! - Initialize the variables to working values
	this->massFSP = 0.0;
	this->r_PB_B.setZero();
	this->pHat_B.setIdentity();
	this->k = 1.0;
	this->c = 0.0;
	this->nameOfRhoState = "fuelSloshParticleRho";
	this->nameOfRhoDotState = "fuelSloshParticleRhoDot";

	return;
}


FuelSloshParticle::~FuelSloshParticle()
{
	return;
}

void FuelSloshParticle::linkInStates(DynParamManager& statesIn)
{
	this->omegaState = statesIn.getStateObject("hubOmega");
	this->sigmaState = statesIn.getStateObject("hubSigma");
	this->velocityState = statesIn.getStateObject("hubVelocity");
    this->g_N = statesIn.getPropertyReference("g_N");

    return;
}

void FuelSloshParticle::registerStates(DynParamManager& states)
{
	this->rhoState = states.registerState(1, 1, nameOfRhoState);
	this->rhoDotState = states.registerState(1, 1, nameOfRhoDotState);

    return;
}

void FuelSloshParticle::updateEffectorMassProps(double integTime) {
	//Cached values used in this function
	this->rho = this->rhoState->getState()(0,0);
	this->r_PcB_B = this->rho * this->pHat_B + this->r_PB_B;

	//Update the effectors mass
	this->effProps.mEff = this->massFSP;
	//Update the position of CoM
	this->effProps.rCB_B = this->r_PcB_B;
	//Update the inertia about B
	this->rTilde_PcB_B << tilde(this->r_PcB_B);
	this->effProps.IEffPntB_B = this->massFSP * this->rTilde_PcB_B * this->rTilde_PcB_B.transpose();

	//Cached values used in this function
	this->rhoDot = this->rhoDotState->getState()(0, 0);
	this->rPrime_PcB_B = this->rhoDot * this->pHat_B;

	//Update derivative of CoM
	this->effProps.rPrimeCB_B = this->rPrime_PcB_B;
	//Update the body time derivative of inertia about B
	this->rPrimeTilde_PcB_B << tilde(this->rPrime_PcB_B);
	this->effProps.IEffPrimePntB_B = -this->massFSP*(this->rPrimeTilde_PcB_B*this->rTilde_PcB_B + this->rTilde_PcB_B*this->rPrimeTilde_PcB_B);

    return;
}

void FuelSloshParticle::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
	Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
	Eigen::Vector3d & vecRotcontr) {
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcmBN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcmNB;                        /* direction cosine matrix from B to N */
	Eigen::Vector3d omega_BN_B_local = this->omegaState->getState();
	Eigen::Matrix3d omegaTilde_BN_B_local;
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_B;                          /* gravitational acceleration in B frame */
    gLocal_N = *this->g_N;

    //! - Find dcmBN
    sigmaBNLocal = (Eigen::Vector3d ) this->sigmaState->getState();
    dcmNB = sigmaBNLocal.toRotationMatrix();
    dcmBN = dcmNB.transpose();
    //! - Map gravity to body frame
    g_B = dcmBN*gLocal_N;
	omegaTilde_BN_B_local << tilde(omega_BN_B_local);

	//Cached value, used in computeDerivatives as well
	a_rho = this->pHat_B.dot(this->massFSP * g_B) - this->k*this->rho - this->c*this->rhoDot
		- 2 * this->massFSP*this->pHat_B.dot(omegaTilde_BN_B_local * this->rPrime_PcB_B)
		- this->massFSP*this->pHat_B.dot(omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->r_PcB_B);

	//Compute matrix/vector contributions
	matrixAcontr = -this->massFSP*this->pHat_B*this->pHat_B.transpose();
	matrixBcontr = this->massFSP*this->pHat_B*this->pHat_B.transpose()*this->rTilde_PcB_B;
	matrixCcontr = -this->massFSP*this->rTilde_PcB_B*this->pHat_B*pHat_B.transpose();
	matrixDcontr = this->massFSP*this->rTilde_PcB_B*this->pHat_B*this->pHat_B.transpose()*this->rTilde_PcB_B;

	vecTranscontr = -this->pHat_B*this->a_rho;
	vecRotcontr = -this->massFSP*omegaTilde_BN_B_local * this->rTilde_PcB_B *this->rPrime_PcB_B -
		this->a_rho*this->rTilde_PcB_B * this->pHat_B;

    return;
}

void FuelSloshParticle::computeDerivatives(double integTime)
{
	Eigen::Vector3d omegaDot_BN_B_local = this->omegaState->getStateDeriv();
	Eigen::Vector3d rDDot_BN_N_local = this->velocityState->getStateDeriv();
	
	//Convert rDDot_BN_N to rDDot_BN_B
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcmBN;                        /* direction cosine matrix from N to B */
	Eigen::Matrix3d dcmNB;                        /* direction cosine matrix from B to N */
	sigmaBNLocal = (Eigen::Vector3d) this->sigmaState->getState();
	dcmNB = sigmaBNLocal.toRotationMatrix();
	dcmBN = dcmNB.transpose();
	Eigen::Vector3d rDDot_BN_B_local = dcmBN*rDDot_BN_N_local;
	
	//Set the derivative of rho to rhoDot
	this->rhoState->setDerivative(this->rhoDotState->getState());

	//Compute rhoDDot
	Eigen::MatrixXd conv(1,1);
	conv(0, 0) = 1 / this->massFSP*(-this->massFSP*this->pHat_B.dot(rDDot_BN_B_local) +
		this->massFSP*this->pHat_B.dot(this->rTilde_PcB_B*omegaDot_BN_B_local) +
		this->a_rho);
	this->rhoDotState->setDerivative(conv);

    return;
}

