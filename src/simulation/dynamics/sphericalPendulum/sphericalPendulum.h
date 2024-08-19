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


#ifndef SPHERICAL_PENDULUM_H
#define SPHERICAL_PENDULUM_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "simulation/dynamics/_GeneralModuleFiles/fuelSlosh.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief spherical pendulum state effector model */
class SphericalPendulum :
	public StateEffector, public SysModel, public FuelSlosh
{
public:
	double pendulumRadius;             //!< [m] distance between the center of the tank and the spherical pendulum mass
    Eigen::Matrix3d D;                    //!< [N*s/m] linear damping matrix for spherical pendulum
    double phiDotInit;             //!< [rad/s] Initial value for spherical pendulum pendulum offset derivative
    double thetaDotInit;             //!< [rad/s] Initial value for spherical pendulum pendulum offset derivative
    double massInit;               //!< [m] Initial value for spherical pendulum pendulum mass
    std::string nameOfPhiState;    //!< [-] Identifier for the phi state data container
    std::string nameOfThetaState;    //!< [-] Identifier for the theta state data container
    std::string nameOfPhiDotState; //!< [-] Identifier for the phiDot state data container
    std::string nameOfThetaDotState; //!< [-] Identifier for the thetaDot state data container
	std::string nameOfMassState;   //!< [-] Identifier for the mass state data container
	Eigen::Vector3d d;        //!< [m] position vector from B point to tank center , T, in body frame
	StateData *massState;		   //!< -- state data for the pendulums mass
    Eigen::Vector3d pHat_01;      //!<-- first vector of the P0 frame in B frame components
    Eigen::Vector3d pHat_02;       //!<-- second vector of the P0 frame in  B frame components
    Eigen::Vector3d pHat_03;        //!<-- third vector of the P0 frame in B frame components
		BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    double phiInit;                //!< [rad] Initial value for spherical pendulum pendulum offset
    double thetaInit;                //!< [rad] Initial value for spherical pendulum pendulum offset
    double phi;					//!< [rad] spherical pendulum displacement in P0 frame
    double theta;                    //!< [rad] spherical pendulum displacement in P0 frame
    double phiDot;					 //!< [rad/s] time derivative of displacement in P0 frame
    double thetaDot;                //!< [rad/s] time derivative of displacement in P0 frame
	double massFSP;                //!< [kg] mass of spherical pendulum pendulum
    Eigen::Vector3d r_PcB_B;       //!< [m] position vector form B to center of mass location of pendulum
    Eigen::Matrix3d rTilde_PcB_B;  //!< [m] tilde matrix of r_Pc_B
	Eigen::Vector3d rPrime_PcB_B;  //!< [m/s] Body time derivative of r_Pc_B
	Eigen::Matrix3d rPrimeTilde_PcB_B;  //!< [m/s] Tilde matrix of rPrime_PcB_B

    Eigen::Vector3d aPhi;          //!< -- Term needed for back-sub method
    Eigen::Vector3d bPhi;          //!< -- Term needed for back-sub method
    Eigen::Vector3d aTheta;          //!< -- Term needed for back-sub method
    Eigen::Vector3d bTheta;          //!< -- Term needed for back-sub method
    double cPhi;                   //!< -- Term needed for back-sub method
    double cTheta;                   //!< -- Term needed for back-sub method

    Eigen::MatrixXd *g_N;      //!< [m/s^2] Gravitational acceleration in N frame components
    Eigen::Vector3d l_B;         //!< [m] vector from the center of the tank to the spherical pendulum pendulum in B frame
	Eigen::Vector3d lPrime_B;    //!< [m/s] derivative of l respect to B frame
    Eigen::Vector3d lPrime_P0;  //!< [m/s] derivative of l in P0 frame
	StateData *phiState;	   //!< -- state data for spherical pendulum displacement
	StateData *thetaState;	   //!< -- state data for spherical pendulum displacement
	StateData *phiDotState;     //!< -- state data for time derivative of phi;
	StateData *thetaDotState;		   //!< -- state data for time derivative of theta;
    Eigen::Matrix3d dcm_B_P0;      // Rotation matrix from P0 to B frame
    static uint64_t effectorID;        //!< [] ID number of this panel



public:
	SphericalPendulum();           //!< -- Contructor
	~SphericalPendulum();          //!< -- Destructor
	void registerStates(DynParamManager& states);  //!< -- Method for FSP to register its states
	void linkInStates(DynParamManager& states);  //!< -- Method for FSP to get access of other states
	void updateEffectorMassProps(double integTime);  //!< -- Method for FSP to add its contributions to mass props
    void modifyStates(double integTime); //!<-- Method to force states modification during integration
    void retrieveMassValue(double integTime);
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives
};


#endif /* SPHERICAL_PENDULUM_H */
