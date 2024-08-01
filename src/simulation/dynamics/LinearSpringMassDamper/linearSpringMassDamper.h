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


#ifndef LINEAR_SPRING_MASS_DAMPER_H
#define LINEAR_SPRING_MASS_DAMPER_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "simulation/dynamics/_GeneralModuleFiles/fuelSlosh.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief linear spring mass damper state effector class */
class LinearSpringMassDamper :
	public StateEffector, public SysModel, public FuelSlosh
{
public:
    double k;                      //!< [N/m] linear spring constant for spring mass damper
    double c;                      //!< [N-s/m] linear damping term for spring mass damper
    double rhoInit;                //!< [m] Initial value for spring mass damper particle offset
    double rhoDotInit;             //!< [m/s] Initial value for spring mass damper particle offset derivative
    double massInit;               //!< [m] Initial value for spring mass damper particle mass
    std::string nameOfRhoState;    //!< [-] Identifier for the rho state data container
    std::string nameOfRhoDotState; //!< [-] Identifier for the rhoDot state data container
	std::string nameOfMassState;      //!< [-] Identifier for the mass state data container
	Eigen::Vector3d r_PB_B;        //!< [m] position vector from B point to particle equilibrium, P, in body frame
	Eigen::Vector3d pHat_B;        //!< [-] particle direction unit vector, in body frame
	StateData *massState;		   //!< -- state data for the particles mass
	BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    double cRho;                   //!< -- Term needed for back-sub method
    double rho;                    //!< [m] spring mass damper displacement from equilibrium
    double rhoDot;                 //!< [m/s] time derivative of displacement from equilibrium
	double massSMD;                //!< [kg] mass of spring mass damper particle
    Eigen::Vector3d r_PcB_B;       //!< [m] position vector form B to center of mass location of particle
    Eigen::Matrix3d rTilde_PcB_B;  //!< [m] tilde matrix of r_Pc_B
	Eigen::Vector3d rPrime_PcB_B;  //!< [m/s] Body time derivative of r_Pc_B
	Eigen::Matrix3d rPrimeTilde_PcB_B;  //!< [m/s] Tilde matrix of rPrime_PcB_B
    Eigen::Vector3d aRho;          //!< -- Term needed for back-sub method
    Eigen::Vector3d bRho;          //!< -- Term needed for back-sub method
    Eigen::MatrixXd *g_N;          //!< [m/s^2] Gravitational acceleration in N frame components
	StateData *rhoState;		   //!< -- state data for spring mass damper displacement from equilibrium
    Eigen::MatrixXd *c_B;            //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;       //!< [m/s] Body time derivative of vector c_B in B frame components
	StateData *rhoDotState;		   //!< -- state data for time derivative of rho;
    static uint64_t effectorID;    //!< [] ID number of this panel

public:
	LinearSpringMassDamper();           //!< -- Contructor
	~LinearSpringMassDamper();          //!< -- Destructor
	void registerStates(DynParamManager& states);  //!< -- Method for SMD to register its states
	void linkInStates(DynParamManager& states);  //!< -- Method for SMD to get access of other states
    void retrieveMassValue(double integTime);
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B);  //!< -- Force and torque on s/c due to linear spring mass damper
    void updateEffectorMassProps(double integTime);  //!< -- Method for stateEffector to give mass contributions
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives
};


#endif /* LINEAR_SPRING_MASS_DAMPER_H */
