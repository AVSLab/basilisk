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
}

void FuelTank::updateEffectorMassProps(double integTime) {
	std::vector<FuelSloshParticle>::iterator i;
	effProps.mEff = 0;
	effProps.IEffPntB_B = effProps.IEffPrimePntB_B = Eigen::Matrix3d::Zero();
	effProps.rCB_B = effProps.rPrimeCB_B = Eigen::Vector3d::Zero();
	for (i = fuelSloshParticles.begin(); i < fuelSloshParticles.end(); i++) {
		i->updateEffectorMassProps(integTime);
		effProps.mEff += i->effProps.mEff;
		effProps.IEffPntB_B += i->effProps.IEffPntB_B;
		effProps.IEffPrimePntB_B += i->effProps.IEffPrimePntB_B;
		effProps.rCB_B += i->effProps.mEff * i->effProps.rCB_B;
		effProps.rPrimeCB_B += i->effProps.mEff * i->effProps.rPrimeCB_B;
	}
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
}