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

#include "fuelTank.h"

FuelTank::FuelTank() 
	:fuelSloshParticles()
{
	//! - zero the contributions for mass props and mass rates
	this->effProps.IEffPntB_B.fill(0.0);
	this->effProps.rCB_B.fill(0.0);
	this->effProps.rPrimeCB_B.fill(0.0);
	this->effProps.mEff = 0.0;
	this->effProps.IEffPrimePntB_B.fill(0.0);

	//! - Initialize the variables to working values
	this->r_TB_B.setZero();
	this->nameOfMassState = "fuelTankMass";

    return;
}

FuelTank::~FuelTank() {
    return;
}

void FuelTank::pushFuelSloshParticle(FuelSloshParticle particle) {
	this->fuelSloshParticles.push_back(particle);

    return;
}

void FuelTank::linkInStates(DynParamManager& statesIn)
{
	std::vector<FuelSloshParticle>::iterator intFSP;
	for (intFSP = this->fuelSloshParticles.begin(); intFSP < this->fuelSloshParticles.end(); intFSP++)
		intFSP->linkInStates(statesIn);
}

void FuelTank::registerStates(DynParamManager& statesIn)
{
	std::vector<FuelSloshParticle>::iterator intFSP;
	for (intFSP = fuelSloshParticles.begin(); intFSP < fuelSloshParticles.end(); intFSP++)
		intFSP->registerStates(statesIn);
	this->massState = statesIn.registerState(1, 1, this->nameOfMassState);
}

void FuelTank::updateEffectorMassProps(double integTime) {
	std::vector<FuelSloshParticle>::iterator intFSP;
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B = this->effProps.IEffPrimePntB_B = Eigen::Matrix3d::Zero();
	this->effProps.rCB_B = effProps.rPrimeCB_B = Eigen::Vector3d::Zero();
	//Incorperate the effects of all of the particles
	for (intFSP = this->fuelSloshParticles.begin(); intFSP < this->fuelSloshParticles.end(); intFSP++) {
		intFSP->updateEffectorMassProps(integTime);
		this->effProps.mEff += intFSP->effProps.mEff;
		this->effProps.IEffPntB_B += intFSP->effProps.IEffPntB_B;
		this->effProps.IEffPrimePntB_B += intFSP->effProps.IEffPrimePntB_B;
		this->effProps.rCB_B += intFSP->effProps.mEff * intFSP->effProps.rCB_B;
		this->effProps.rPrimeCB_B += intFSP->effProps.mEff * intFSP->effProps.rPrimeCB_B;
	}

	//Contributions of the mass of the tank
	double massLocal = this->massState->getState()(0, 0);
	this->effProps.mEff += massLocal;
	this->effProps.IEffPntB_B += (2.0 / 5.0 * massLocal * radiusTank * radiusTank) * Eigen::Matrix3d::Identity() + massLocal * (r_TB_B.dot(r_TB_B)*Eigen::Matrix3d::Identity() - r_TB_B * r_TB_B.transpose());
	this->effProps.rCB_B += massLocal * r_TB_B;

    //! - Scale the center of mass location by 1/m_tot
	this->effProps.rCB_B /= effProps.mEff;
	this->effProps.rPrimeCB_B /= effProps.mEff;

    return;
}

void FuelTank::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
	Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
	Eigen::Vector3d & vecRotcontr) {
	std::vector<FuelSloshParticle>::iterator intFSP;
	matrixAcontr = matrixBcontr = matrixCcontr = matrixDcontr = Eigen::Matrix3d::Zero();
	vecTranscontr = vecRotcontr = Eigen::Vector3d::Zero();
	for (intFSP = fuelSloshParticles.begin(); intFSP < fuelSloshParticles.end(); intFSP++) {
		Eigen::Matrix3d Acontr, Bcontr, Ccontr, Dcontr;
		Eigen::Vector3d Transcontr, Rotcontr;
		intFSP->updateContributions(integTime, Acontr, Bcontr, Ccontr, Dcontr, Transcontr, Rotcontr);
		matrixAcontr += Acontr;
		matrixBcontr += Bcontr;
		matrixCcontr += Ccontr;
		matrixDcontr += Dcontr;
		vecTranscontr += Transcontr;
		vecRotcontr += Rotcontr;
	}
}

void FuelTank::computeDerivatives(double integTime)
{
	std::vector<FuelSloshParticle>::iterator intFSP;
	for (intFSP = fuelSloshParticles.begin(); intFSP < fuelSloshParticles.end(); intFSP++)
		intFSP->computeDerivatives(integTime);
    
	std::vector<DynamicEffector*>::iterator dynIt;
	//! - Mass depletion
	double fuelConsumption = 0.0;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->computeStateContribution(integTime);
        fuelConsumption += (*dynIt)->stateDerivContribution(0);
    }
	Eigen::MatrixXd conv(1, 1);
	conv(0, 0) = -fuelConsumption;
	this->massState->setDerivative(conv);
}
