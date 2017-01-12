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

#include "fuelTank.h"

FuelTank::FuelTank() 
	:fuelSloshParticles()
{
	//! - zero the contributions for mass props and mass rates
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.fill(0.0);
	this->effProps.rEff_CB_B.fill(0.0);
	this->effProps.rEffPrime_CB_B.fill(0.0);
	this->effProps.IEffPrimePntB_B.fill(0.0);
    this->propMassInit = 0.0;

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
	this->omegaState = statesIn.getStateObject("hubOmega");
}

void FuelTank::registerStates(DynParamManager& statesIn)
{
	std::vector<FuelSloshParticle>::iterator intFSP;
    Eigen::MatrixXd massMatrix(1,1);
	for (intFSP = fuelSloshParticles.begin(); intFSP < fuelSloshParticles.end(); intFSP++)
		intFSP->registerStates(statesIn);
	this->massState = statesIn.registerState(1, 1, this->nameOfMassState);
    massMatrix(0,0) = propMassInit;
    this->massState->setState(massMatrix);
}

void FuelTank::updateEffectorMassProps(double integTime) {
	std::vector<FuelSloshParticle>::iterator intFSP;
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B = this->effProps.IEffPrimePntB_B = Eigen::Matrix3d::Zero();
	this->effProps.rEff_CB_B = effProps.rEffPrime_CB_B = Eigen::Vector3d::Zero();
	//Incorperate the effects of all of the particles
	for (intFSP = this->fuelSloshParticles.begin(); intFSP < this->fuelSloshParticles.end(); intFSP++) {
		intFSP->updateEffectorMassProps(integTime);
		this->effProps.mEff += intFSP->effProps.mEff;
		this->effProps.IEffPntB_B += intFSP->effProps.IEffPntB_B;
		this->effProps.IEffPrimePntB_B += intFSP->effProps.IEffPrimePntB_B;
		this->effProps.rEff_CB_B += intFSP->effProps.mEff * intFSP->effProps.rEff_CB_B;
		this->effProps.rEffPrime_CB_B += intFSP->effProps.mEff * intFSP->effProps.rEffPrime_CB_B;
	}

	//Contributions of the mass of the tank
	double massLocal = this->massState->getState()(0, 0);
	this->effProps.mEff += massLocal;
    this->ITankPntT_B = (2.0 / 5.0 * massLocal * radiusTank * radiusTank) * Eigen::Matrix3d::Identity();
	this->effProps.IEffPntB_B += this->ITankPntT_B + massLocal * (r_TB_B.dot(r_TB_B)*Eigen::Matrix3d::Identity() - r_TB_B * r_TB_B.transpose());
	this->effProps.rEff_CB_B += massLocal * r_TB_B;

    //! - Scale the center of mass location by 1/m_tot
	this->effProps.rEff_CB_B /= effProps.mEff;
	this->effProps.rEffPrime_CB_B /= effProps.mEff;

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

    return;
}

void FuelTank::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr)
{
    // - call updateMassProps to get current mass props info
    this->updateEffectorMassProps(integTime);
    
    std::vector<FuelSloshParticle>::iterator intFSP;
    for (intFSP = fuelSloshParticles.begin(); intFSP < fuelSloshParticles.end(); intFSP++)
    {
        Eigen::Vector3d rotAngMomPntCContrFSP_B;
        double rotEnergyContrFSP = 0.0;
        rotAngMomPntCContrFSP_B.setZero();

        intFSP->updateEnergyMomContributions(integTime, rotAngMomPntCContrFSP_B, rotEnergyContrFSP);
        rotAngMomPntCContr_B += rotAngMomPntCContrFSP_B;
        rotEnergyContr += rotEnergyContrFSP;
    }

    // Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();
    Eigen::Vector3d rDotTB_B;

    // Call mass props to get current information on states
    this->updateEffectorMassProps(integTime);

    // Find rotational angular momentum contribution from hub
    double massLocal = this->massState->getState()(0, 0);
    rDotTB_B = omegaLocal_BN_B.cross(this->r_TB_B);
    rotAngMomPntCContr_B += this->ITankPntT_B*omegaLocal_BN_B + massLocal*this->r_TB_B.cross(rDotTB_B);

    // Find rotational energy contribution from the hub
    rotEnergyContr += 1.0/2.0*omegaLocal_BN_B.dot(this->ITankPntT_B*omegaLocal_BN_B) + 1.0/2.0*massLocal*rDotTB_B.dot(rDotTB_B);
    
    return;
}
