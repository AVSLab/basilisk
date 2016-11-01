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



FuelSloshParticle::FuelSloshParticle()
{
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
	this->rho = states.registerState(1, 1, "fspRho");
	this->rhoDot = states.registerState(1, 1, "fspRhoDot");
	//Initialize states?
}

void FuelSloshParticle::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
	Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
	Eigen::Vector3d & vecRotcontr) {
	double rho_local = rho->getState()[0];
	double rhoDot_local = rhoDot->getState()[0];
	Eigen::Matrix3d omega_BN_B_local = omegaState->getState();

	rPrime_PcB_B = rhoDot_local * pHat_B;
	r_PcB_B = rho_local * pHat_B + r_PB_B;

	a_rho = /*pHat_B.dot(F_G_local)*/ - k*rho_local - c*rhoDot_local
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
	Eigen::Vector3d rDDot_BN_B_local = velocityState->getStateDeriv();
	
	rho->setDerivative(rhoDot->getState());
	Eigen::MatrixXd conv(1,1);
	conv[0] = 1 / massFSP*(-massFSP*pHat_B.dot(rDDot_BN_B_local) +
		massFSP*pHat_B.dot(r_PcB_B.cross(omegaDot_BN_B_local)) + 
		a_rho);
	rhoDot->setDerivative(conv);
}

void FuelSloshParticle::updateEffectorMassProps(double integTime) {
	effProps.mEff = massFSP;
	effProps.re_B = r_PB_B;

}
void FuelSloshParticle::updateEffectorMassPropRates(double integTime) {

}
