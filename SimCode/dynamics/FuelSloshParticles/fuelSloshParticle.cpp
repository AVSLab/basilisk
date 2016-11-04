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

#include "FuelSloshParticle.h"

#define tilde(vector) 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0



FuelSloshParticle::FuelSloshParticle()
{
	//! - zero the contributions for mass props and mass rates
	effProps.IEffPntB_B.fill(0.0);
	effProps.rCB_B.fill(0.0);
	effProps.rPrimeCB_B.fill(0.0);
	effProps.mEff = 0.0;
	effProps.IEffPrimePntB_B.fill(0.0);
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
}

void FuelSloshParticle::registerStates(DynParamManager& states)
{
	this->rhoState = states.registerState(1, 1, "fspRho");
	this->rhoDotState = states.registerState(1, 1, "fspRhoDot");
	//Initialize states?
}

void FuelSloshParticle::updateEffectorMassProps(double integTime) {
	//Cached values used in this function
	rho = rhoState->getState()[0];
	r_PcB_B = rho * pHat_B + r_PB_B;

	//Update the effectors mass
	effProps.mEff = massFSP;
	//Update the position of CoM
	effProps.rCB_B = r_PcB_B;
	//Update the inertia about B
	rTilde_PcB_B << tilde(r_PcB_B);
	effProps.IEffPntB_B = massFSP * rTilde_PcB_B * rTilde_PcB_B.transpose();
}
void FuelSloshParticle::updateEffectorMassPropRates(double integTime) {
	//Cached values used in this function
	rhoDot = rhoDotState->getState()[0];
	rPrime_PcB_B = rhoDot * pHat_B;

	//Update derivative of CoM
	effProps.rPrimeCB_B = rPrime_PcB_B;
	//Update the body time derivative of inertia about B
	rPrimeTilde_PcB_B << tilde(rPrime_PcB_B);
	effProps.IEffPrimePntB_B = -massFSP*(rPrimeTilde_PcB_B*rTilde_PcB_B + rTilde_PcB_B*rPrimeTilde_PcB_B);
}

void FuelSloshParticle::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
	Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
	Eigen::Vector3d & vecRotcontr) {
	Eigen::Matrix3d omega_BN_B_local = omegaState->getState();

	a_rho = /*pHat_B.dot(F_G_local)*/ - k*rho - c*rhoDot
		- 2 * massFSP*pHat_B.dot(omega_BN_B_local.cross(rPrime_PcB_B))
		- massFSP*pHat_B.dot(omega_BN_B_local.cross(omega_BN_B_local.cross(r_PcB_B)));

	matrixAcontr = -massFSP*pHat_B*pHat_B.transpose();
	matrixBcontr = massFSP*pHat_B*pHat_B.cross(r_PcB_B).transpose();
	matrixCcontr = -massFSP*r_PcB_B.cross(pHat_B)*pHat_B;
	matrixDcontr = massFSP*r_PcB_B.cross(pHat_B) * pHat_B.cross(r_PcB_B).transpose();

	vecTranscontr = -pHat_B*a_rho;
	vecRotcontr = -massFSP*omega_BN_B_local.cross(r_PcB_B.cross(rPrime_PcB_B)) +
		a_rho*r_PcB_B.cross(pHat_B);
}

void FuelSloshParticle::computeDerivatives(double integTime)
{
	Eigen::Vector3d omegaDot_BN_B_local = omegaState->getStateDeriv();
	Eigen::Vector3d rDDot_BN_N_local = velocityState->getStateDeriv();
	
	//Convert rDDot_BN_N to rDDot_BN_B
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcmBN;                        /* direction cosine matrix from N to B */
	Eigen::Matrix3d dcmNB;                        /* direction cosine matrix from B to N */
	sigmaBNLocal = (Eigen::Vector3d)sigmaState->getState();
	dcmNB = sigmaBNLocal.toRotationMatrix();
	dcmBN = dcmNB.transpose();
	Eigen::Vector3d rDDot_BN_B_local = dcmBN*rDDot_BN_N_local;
	
	//Set the derivative of rho to rhoDot
	rhoState->setDerivative(rhoDotState->getState());

	//Compute rhoDDot
	Eigen::MatrixXd conv(1,1);
	conv[0] = 1 / massFSP*(-massFSP*pHat_B.dot(rDDot_BN_B_local) +
		massFSP*pHat_B.dot(r_PcB_B.cross(omegaDot_BN_B_local)) + 
		a_rho);
	rhoDotState->setDerivative(conv);
}

