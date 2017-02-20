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

#ifndef FUEL_TANK_H
#define FUEL_TANK_H

#include <vector>
#include "../_GeneralModuleFiles/stateEffector.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "../SimCode/utilities/avsEigenMRP.h"
#include "fuelSloshParticle.h"

/*! @brief This class is an instantiation of the stateEffector abstract class and implements an effector representing a 
 fuel tank. This fuel tank has one state associated with it and is the mass of the fuel inside the tank */
class FuelTank :
	public StateEffector, public SysModel
{
public:
	double radiusTank;                                 //!< [m] radius of the spherical tank
    double propMassInit;                               //!< [kg] Initial propellant mass in tank
	std::string nameOfMassState;                       //!< -- name of mass state
	Eigen::Vector3d r_TB_B;                            //!< [m] position vector from B to tank point in B frame comp.
    Eigen::Matrix3d ITankPntT_B;                       //!< [kg m^2] Inertia of tank about pnt T in B frame comp.
    std::vector<FuelSloshParticle> fuelSloshParticles; //!< -- vector of fuel slosh particles
    std::vector<DynamicEffector*> dynEffectors;        //!< -- Vector of dynamic effectors for thrusters

private:
	StateData *massState;                              //!< -- state data for mass state
	StateData *omegaState;                             //!< -- state data for omega_BN of the hub
	double fuelConsumption;							   //!< [kg/s] rate of fuel being consumed 

public:
	FuelTank();                                        //!< -- Contructor
	~FuelTank();                                       //!< -- Destructor
	void pushFuelSloshParticle(FuelSloshParticle particle);  //!< -- Method to attach fuel slosh particle
	void registerStates(DynParamManager& states);  //!< -- Method to register mass state with state manager
	void linkInStates(DynParamManager& states);  //!< -- Method to give the tank access to other states
	void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
		Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
		Eigen::Vector3d & vecRotcontr);  //!< -- Method to add contributions for back-sub from the tank
	void computeDerivatives(double integTime);  //!< -- Method to compute derivatives for the tank
	void updateEffectorMassProps(double integTime);  //!< -- Method to add contribtution mass props from the tank
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr);  //!< -- Method to calculate energy and mom. for tank
    void addThrusterSet(DynamicEffector *NewdynEff) {dynEffectors.push_back(NewdynEff);}  //!< -- Method to add thruster
};

#endif /* FUEL_TANK_H */
