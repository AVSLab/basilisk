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

#ifndef FUEL_SLOSH_PARTICLE_H
#define FUEL_SLOSH_PARTICLE_H

#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/sys_model.h"
#include "../SimCode/utilities/avsEigenMRP.h"

/*! @brief Class that implements an effector representing a sloshing particle
*/
class FuelSloshParticle :
	public StateEffector, public SysModel
{
public:
	double massFSP;            //!< kg, mass of fuel slosh particle
	Eigen::Vector3d r_PB_B;    //!< m, position vector from B point to slosh equilibrium, P, in body frame
	Eigen::Vector3d pHat_B;    //!< slosh direction unit vector, in body frame
	double k;                  //!< N/m, linear spring constant for fuel slosh
	double c;                  //!< N-s/m, linear damping term for fuel slosh
	std::string nameOfRhoState;   //!< [-] Identifier for the rho state data container
	std::string nameOfRhoDotState; //!< [-] Identifier for the rhoDot state data container

private:
	//Eigen::MatrixXd *F_G;

	//Cached values, used in multiple functions
	Eigen::Vector3d rPrime_PcB_B;
	Eigen::Matrix3d rPrimeTilde_PcB_B;
	Eigen::Vector3d r_PcB_B;
	Eigen::Matrix3d rTilde_PcB_B;

	double a_rho;
	double rho;
	double rhoDot;

	StateData *rhoState;			   //!< m, fuel slosh displacement from equilibrium
	StateData *rhoDotState;		   //!< m/s, time derivative of rho;
	StateData *omegaState;
	StateData *sigmaState;
	StateData *velocityState;

public:
	FuelSloshParticle();
	~FuelSloshParticle();
	void registerStates(DynParamManager& states);
	void linkInStates(DynParamManager& states);
	void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
		Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
		Eigen::Vector3d & vecRotcontr);
	void computeDerivatives(double integTime);
	void updateEffectorMassProps(double integTime);
};

#endif

