/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "../simulation/utilities/avsEigenMRP.h"
#include "../simulation/utilities/avsEigenSupport.h"
#include "simMessages/fuelTankSimMsg.h"
#include "../_GeneralModuleFiles/fuelSlosh.h"

//Fuel tank models
/*! @brief This class is an abstract class that has the minimum interfaces for a tank model */
struct FuelTankModel {
	double propMassInit;                               //!< [kg] Initial propellant mass in tank
	Eigen::Vector3d r_TcT_TInit;                       //!< [m] Initial position vector from B to tank point in B frame comp.
	Eigen::Matrix3d ITankPntT_T;					   //!< [kg m^2] Inertia of tank about pnt T in B frame comp.
	Eigen::Matrix3d IPrimeTankPntT_T;				   //!< [kg m^2/s] Derivative of inertia of tank about pnt T in B frame comp.
	Eigen::Vector3d r_TcT_T;                           //!< [m] position vector from B to tank point in B frame comp.
	Eigen::Vector3d rPrime_TcT_T;                      //!< [m/s] Derivative of position vector from B to tank point in B frame comp.
	Eigen::Vector3d rPPrime_TcT_T;                     //!< [m/s^2] Second derivative of position vector from B to tank point in B frame comp.
	virtual void computeTankProps(double mFuel) = 0;
	virtual void computeTankPropDerivs(double mFuel, double mDotFuel) = 0;
	FuelTankModel() {
		propMassInit = 0.0;
		r_TcT_TInit.setZero();
	}
    virtual ~FuelTankModel() {
    }
};

struct FuelTankModelConstantVolume_t :
	public FuelTankModel
{
	double radiusTankInit;                             //!< [m] Initial radius of the spherical tank

	void computeTankProps(double mFuel) {
		r_TcT_T = r_TcT_TInit;
		ITankPntT_T = 2.0 / 5.0 * mFuel*radiusTankInit*radiusTankInit*Eigen::Matrix3d::Identity();
	}

	void computeTankPropDerivs(double mFuel, double mDotFuel) {
		IPrimeTankPntT_T = 2.0 / 5.0 * mDotFuel*radiusTankInit*radiusTankInit*Eigen::Matrix3d::Identity();
		rPrime_TcT_T.setZero();
		rPPrime_TcT_T.setZero();
	}
};

struct FuelTankModelConstantDensity_t :
	public FuelTankModel
{
	double radiusTankInit;                             //!< [m] Initial radius of the spherical tank
	double radiusTank;								   //!< [m] Current radius of the spherical tank

	void computeTankProps(double mFuel) {
		radiusTank = std::pow(mFuel / propMassInit, 1.0 / 3.0)*radiusTankInit;
		r_TcT_T = r_TcT_TInit;
		ITankPntT_T = 2.0 / 5.0 * mFuel*radiusTank*radiusTank*Eigen::Matrix3d::Identity();
	}

	void computeTankPropDerivs(double mFuel, double mDotFuel) {
		IPrimeTankPntT_T = 2.0 / 3.0 * mDotFuel*radiusTank*radiusTank*Eigen::Matrix3d::Identity();
		rPrime_TcT_T.setZero();
		rPPrime_TcT_T.setZero();
	}
};

struct FuelTankModelEmptying_t :
	public FuelTankModel
{
	double radiusTankInit;                             //!< [m] Initial radius of the spherical tank
	double rhoFuel;                                    //!< [kg/m^3] density of the fuel
	double thetaStar;								   //!< [rad] angle from vertical to top of fuel
	double thetaDotStar;                               //!< [rad/s] derivative of angle from vertical to top of fuel
	double thetaDDotStar;							   //!< [rad/s^2] second derivative of angle from vertical to top of fuel
	Eigen::Vector3d k3;								   //!< -- Direction of fuel depletion 

	void computeTankProps(double mFuel) {
		rhoFuel = propMassInit / (4.0 / 3.0*M_PI*radiusTankInit*radiusTankInit*radiusTankInit);
		double rtank = radiusTankInit;
		double volume;
		double deltaRadiusK3;
		k3 << 0, 0, 1; //k3 is zhat

		if (mFuel != propMassInit) {
			double rhoFuel = this->rhoFuel;
			std::function<double(double)> f = [rhoFuel, rtank, mFuel](double thetaStar)-> double {
				return 2.0 / 3.0*M_PI*rhoFuel*rtank*rtank*rtank*(1 + 3.0 / 2.0*cos(thetaStar) - 1.0 / 2.0*pow(cos(thetaStar), 3))-mFuel;
			};
			std::function<double(double)> fPrime = [rhoFuel, rtank](double thetaStar)-> double {
				return  2.0 / 3.0*M_PI*rhoFuel*rtank*rtank*rtank*(-3.0 / 2.0*sin(thetaStar) + 3.0 / 2.0*pow(cos(thetaStar), 2)*sin(thetaStar));
			};

			thetaStar = newtonRaphsonSolve(M_PI/2.0, 1E-20, f, fPrime);
		}
		else {
			thetaStar = 0.0;
		}
		volume = 2.0 / 3.0*M_PI*std::pow(radiusTankInit,3)*(1 + 3.0 / 2.0*std::cos(thetaStar) - 1.0 / 2.0*std::pow(std::cos(thetaStar), 3));
		if (volume != 0) {
			deltaRadiusK3 = M_PI*std::pow(radiusTankInit, 4) / (4.0*volume)*(2.0*std::pow(std::cos(thetaStar), 2) - std::pow(std::cos(thetaStar), 4) - 1);
		}
		else {
			deltaRadiusK3 = -radiusTankInit;
		}

		r_TcT_T = r_TcT_TInit + deltaRadiusK3*k3;
		ITankPntT_T.setZero();
		IPrimeTankPntT_T.setZero();
		ITankPntT_T(2, 2) = 2.0 / 5.0 *M_PI*rhoFuel*std::pow(radiusTankInit, 5) *
			(2.0 / 3.0 + 1.0 / 4.0*std::cos(thetaStar)*std::pow(std::sin(thetaStar), 4) - 1 / 12.0*(std::cos(3 * thetaStar) - 9 * std::cos(thetaStar)));
		ITankPntT_T(0, 0) = ITankPntT_T(1, 1) = 2.0 / 5.0 *M_PI*rhoFuel*std::pow(radiusTankInit, 5) *
			(2.0 / 3.0 - 1.0 / 4.0*std::pow(std::cos(thetaStar), 5) + 1 / 24.0*(std::cos(3 * thetaStar) - 9 * std::cos(thetaStar)) + 
				5.0 / 4.0*cos(thetaStar) + 1 / 8.0*std::cos(thetaStar)*std::pow(std::sin(thetaStar), 4));

		
	}
	void computeTankPropDerivs(double mFuel, double mDotFuel) {
		if (mFuel != propMassInit) {
			thetaDotStar = -mDotFuel / (M_PI*rhoFuel*std::pow(radiusTankInit, 3)*std::sin(thetaStar));
			thetaDDotStar = -3 * thetaDotStar*thetaDotStar*std::cos(thetaStar) / std::sin(thetaStar); //This assumes that mddot = 0
		}
		else {
			thetaDotStar = 0.0;
			thetaDDotStar = 0.0;
		}
		IPrimeTankPntT_T(2, 2) = 2.0 / 5.0 *M_PI*rhoFuel*std::pow(radiusTankInit, 5) * thetaDotStar *
			(std::pow(std::cos(thetaStar), 2)*std::pow(std::sin(thetaStar), 3) - 1.0 / 4.0*std::pow(std::sin(thetaStar), 5) +
				1 / 4.0*std::sin(3 * thetaStar) - 3.0 / 4.0*std::sin(thetaStar));
		IPrimeTankPntT_T(0, 0) = IPrimeTankPntT_T(1, 1) = 2.0 / 5.0 *M_PI*rhoFuel*std::pow(radiusTankInit, 5) * thetaDotStar *
			(5.0 / 4.0*std::sin(thetaStar)*std::cos(thetaStar) - 5.0 / 4.0*std::sin(thetaStar) - 1 / 8.0*std::sin(3 * thetaStar) +
				3.0 / 8.0*sin(thetaStar) + 1 / 2.0*std::pow(std::cos(thetaStar), 2)*std::pow(std::sin(thetaStar), 3) - 1 / 8.0*std::pow(std::sin(thetaStar), 5));
		if (mFuel != 0) {
			rPrime_TcT_T = -M_PI*std::pow(radiusTankInit, 4)*rhoFuel / (4 * mFuel*mFuel)*(4 * mFuel*thetaDotStar*std::pow(std::sin(thetaStar), 3)*std::cos(thetaStar) +
				mDotFuel*(2 * std::pow(std::cos(thetaStar), 2) - std::pow(std::cos(thetaStar), 4) - 1))*k3;

			rPPrime_TcT_T = -M_PI*std::pow(radiusTankInit, 4)*rhoFuel / (2 * mFuel*mFuel*mFuel)*(4 * mFuel*std::pow(std::sin(thetaStar), 3)*std::cos(thetaStar)*
				(thetaDDotStar*mFuel - 2 * thetaDotStar*mDotFuel) - 4 * mFuel*mFuel*thetaDotStar*thetaDotStar*std::pow(std::sin(thetaStar), 2)*
				(3 * std::pow(std::cos(thetaStar), 2) - std::pow(std::sin(thetaStar), 2)) + (2 * std::pow(std::cos(thetaStar), 2) - std::pow(std::cos(thetaStar), 4) - 1)*
				(-2 * mDotFuel*mDotFuel))*k3;

		}
		else {
			rPrime_TcT_T.setZero();
			rPPrime_TcT_T.setZero();
		}
	}
};

struct FuelTankModelUniformBurn_t :
	public FuelTankModel
{
	double radiusTankInit;                             //!< [m] Initial radius of the cylindrical tank
	double lengthTank;								   //!< [m] Length of the tank
	
	void computeTankProps(double mFuel) {
		r_TcT_T = r_TcT_TInit;
		ITankPntT_T.setZero();
		ITankPntT_T(0, 0) = ITankPntT_T(1, 1) = mFuel * (radiusTankInit*radiusTankInit / 4.0 + lengthTank*lengthTank / 12.0);
		ITankPntT_T(2, 2) = mFuel*radiusTankInit*radiusTankInit/2;
	}
	void computeTankPropDerivs(double mFuel, double mDotFuel) {
		IPrimeTankPntT_T.setZero();
		IPrimeTankPntT_T(0, 0) = IPrimeTankPntT_T(1, 1) = mDotFuel * (radiusTankInit*radiusTankInit / 4.0 + lengthTank*lengthTank / 12.0);
		IPrimeTankPntT_T(2, 2) = mDotFuel*radiusTankInit*radiusTankInit / 2;
		rPrime_TcT_T.setZero();
		rPPrime_TcT_T.setZero();
	}
};

struct FuelTankModelCentrifugalBurn_t :
	public FuelTankModel
{
	double radiusTankInit;                             //!< [m] Initial radius of the cylindrical tank
	double lengthTank;								   //!< [m] Length of the tank
	double radiusInner;								   //!< [m] Inner radius of the cylindrical tank

	void computeTankProps(double mFuel) {
		double rhoFuel = propMassInit / (M_PI*radiusTankInit*radiusTankInit*lengthTank);
		radiusInner = std::sqrt(std::max(radiusTankInit*radiusTankInit - mFuel / (M_PI*lengthTank*rhoFuel),0.0));
		r_TcT_T = r_TcT_TInit;
		ITankPntT_T.setZero();
		ITankPntT_T(0, 0) = ITankPntT_T(1, 1) = mFuel * ((radiusTankInit*radiusTankInit+radiusInner*radiusInner) / 4.0 + lengthTank*lengthTank / 12.0);
		ITankPntT_T(2, 2) = mFuel*(radiusTankInit*radiusTankInit +radiusInner*radiusInner)/ 2;
	}
	void computeTankPropDerivs(double mFuel, double mDotFuel) {
		IPrimeTankPntT_T.setZero();
		IPrimeTankPntT_T(0, 0) = IPrimeTankPntT_T(1, 1) = mDotFuel * (radiusInner*radiusInner / 2.0 + lengthTank*lengthTank / 12.0);
		IPrimeTankPntT_T(2, 2) = mDotFuel*radiusInner*radiusInner;
		rPrime_TcT_T.setZero();
		rPPrime_TcT_T.setZero();
	}
};

enum FuelTankModelTypes {
	TANK_MODEL_FIRST_MODEL,
	TANK_MODEL_CONSTANT_VOLUME = TANK_MODEL_FIRST_MODEL,
	TANK_MODEL_CONSTANT_DENSITY,
	TANK_MODEL_EMPTYING,
	TANK_MODEL_UNIFORM_BURN,
	TANK_MODEL_CENTRIFUGAL_BURN,
	TANK_MODEL_LAST_MODEL,
};

extern FuelTankModelConstantVolume_t FuelTankModelConstantVolume;
extern FuelTankModelConstantDensity_t FuelTankModelConstantDensity;
extern FuelTankModelEmptying_t FuelTankModelEmptying;
extern FuelTankModelUniformBurn_t FuelTankModelUniformBurn;
extern FuelTankModelCentrifugalBurn_t FuelTankModelCentrifugalBurn;

extern FuelTankModel* FuelTankModels[TANK_MODEL_LAST_MODEL - TANK_MODEL_FIRST_MODEL];

/*! @brief This class is an instantiation of the stateEffector abstract class and implements an effector representing a 
 fuel tank. This fuel tank has one state associated with it and is the mass of the fuel inside the tank

 The module
 [PDF Description](Basilisk-FUELTANK-20171203.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.

 */
class FuelTank :
	public StateEffector, public SysModel
{
public:
	std::string nameOfMassState;                       //!< -- name of mass state
    std::vector<FuelSlosh*> fuelSloshParticles;         //!< -- vector of fuel slosh particles
    std::vector<DynamicEffector*> dynEffectors;        //!< -- Vector of dynamic effectors for thrusters
	Eigen::Matrix3d dcm_TB;							   //!< -- DCM from body frame to tank frame
	Eigen::Vector3d r_TB_B;							   //!< [m] position of tank in B frame
	bool updateOnly;								   //!< -- Sets whether to use update only mass depletion
    std::string FuelTankOutMsgName;                    //!< -- fuel tank output message name
    FuelTankSimMsg FuelTankMassPropMsg;                //!< instance of messaging system message struct

private:
	StateData *omegaState;                             //!< -- state data for omega_BN of the hub
	StateData *massState;                              //!< -- state data for mass state
	double fuelConsumption;							   //!< [kg/s] rate of fuel being consumed 
	double tankFuelConsumption;						   //!< [kg/s] rate of fuel being consumed from tank
	FuelTankModel* fuelTankModel;					   //!< -- style of tank to simulate
	Eigen::Matrix3d ITankPntT_B;					   
	Eigen::Vector3d r_TcB_B;
    int64_t FuelTankOutMsgId;                          //!< -- state output message ID

public:
	FuelTank();                                        //!< -- Contructor
	~FuelTank();                                       //!< -- Destructor
    void SelfInit();
    void CrossInit();
    void WriteOutputMessages(uint64_t CurrentClock);
    void UpdateState(uint64_t CurrentSimNanos);
	void setTankModel(FuelTankModelTypes model);
	void pushFuelSloshParticle(FuelSlosh *particle);  //!< -- Method to attach fuel slosh particle
	void registerStates(DynParamManager& states);  //!< -- Method to register mass state with state manager
	void linkInStates(DynParamManager& states);  //!< -- Method to give the tank access to other states
	void updateEffectorMassProps(double integTime);  //!< -- Method to add contribtution mass props from the tank
    void addThrusterSet(DynamicEffector *NewdynEff) {dynEffectors.push_back(NewdynEff);}  //!< -- Method to add thruster
    virtual void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    virtual void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    virtual void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives
};

#endif /* FUEL_TANK_H */
