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
	effProps.IEffPntB_B.fill(0.0);
	effProps.rCB_B.fill(0.0);
	effProps.rPrimeCB_B.fill(0.0);
	effProps.mEff = 0.0;
	effProps.IEffPrimePntB_B.fill(0.0);

	//! - Initialize the variables to working values
	r_TB_B.setZero();
	nameOfMassState = "fuelTankMass";
}

FuelTank::~FuelTank() {

}

void FuelTank::pushFuelSloshParticle(FuelSloshParticle particle) {
	fuelSloshParticles.push_back(particle);
}

void FuelTank::linkInStates(DynParamManager& statesIn)
{
	std::vector<FuelSloshParticle>::iterator i;
	for (i = fuelSloshParticles.begin(); i < fuelSloshParticles.end(); i++)
		i->linkInStates(statesIn);
}

void FuelTank::registerStates(DynParamManager& statesIn)
{
	std::vector<FuelSloshParticle>::iterator i;
	for (i = fuelSloshParticles.begin(); i < fuelSloshParticles.end(); i++)
		i->registerStates(statesIn);
	this->massState = statesIn.registerState(1, 1, nameOfMassState);
}

void FuelTank::updateEffectorMassProps(double integTime) {
	std::vector<FuelSloshParticle>::iterator i;
	effProps.mEff = 0;
	effProps.IEffPntB_B = effProps.IEffPrimePntB_B = Eigen::Matrix3d::Zero();
	effProps.rCB_B = effProps.rPrimeCB_B = Eigen::Vector3d::Zero();
	//Incorperate the effects of all of the particles
	for (i = fuelSloshParticles.begin(); i < fuelSloshParticles.end(); i++) {
		i->updateEffectorMassProps(integTime);
		effProps.mEff += i->effProps.mEff;
		effProps.IEffPntB_B += i->effProps.IEffPntB_B;
		effProps.IEffPrimePntB_B += i->effProps.IEffPrimePntB_B;
		effProps.rCB_B += i->effProps.mEff * i->effProps.rCB_B;
		effProps.rPrimeCB_B += i->effProps.mEff * i->effProps.rPrimeCB_B;
	}

	//Contributions of the mass of the tank
	double massLocal = massState->getState()(0, 0);
	double massDotLocal = massState->getState()(0, 0);
	effProps.mEff += massLocal;
	effProps.IEffPntB_B += (2 / 5 * massLocal * radiusTank * radiusTank) * Eigen::Matrix3d::Identity() + massLocal * (r_TB_B.dot(r_TB_B)*Eigen::Matrix3d::Identity() - r_TB_B * r_TB_B.transpose());
	effProps.IEffPrimePntB_B += (2 / 5 * massDotLocal * radiusTank * radiusTank) * Eigen::Matrix3d::Identity() + massDotLocal * (r_TB_B.dot(r_TB_B)*Eigen::Matrix3d::Identity() - r_TB_B * r_TB_B.transpose());
	effProps.rCB_B += massLocal * r_TB_B;
	effProps.rPrimeCB_B += massDotLocal * r_TB_B;

	effProps.rCB_B /= effProps.mEff;
	effProps.rPrimeCB_B /= effProps.mEff;


}

void FuelTank::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
	Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
	Eigen::Vector3d & vecRotcontr) {
	std::vector<FuelSloshParticle>::iterator i;
	matrixAcontr = matrixBcontr = matrixCcontr = matrixDcontr = Eigen::Matrix3d::Zero();
	vecTranscontr = vecRotcontr = Eigen::Vector3d::Zero();
	for (i = fuelSloshParticles.begin(); i < fuelSloshParticles.end(); i++) {
		Eigen::Matrix3d Acontr, Bcontr, Ccontr, Dcontr;
		Eigen::Vector3d Transcontr, Rotcontr;
		i->updateContributions(integTime, Acontr, Bcontr, Ccontr, Dcontr, Transcontr, Rotcontr);
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
	std::vector<FuelSloshParticle>::iterator i;
	for (i = fuelSloshParticles.begin(); i < fuelSloshParticles.end(); i++)
		i->computeDerivatives(integTime);
	
	//! - Mass depletion
	double fuelConsumption = 0.0;
	//TODO: add in the contributions from the thrusters
	Eigen::MatrixXd conv(1, 1);
	conv(0, 0) = -fuelConsumption;
	this->massState->setDerivative(conv);
}
