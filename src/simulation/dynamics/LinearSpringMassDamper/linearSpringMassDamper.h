/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "../_GeneralModuleFiles/stateEffector.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "../simulation/utilities/avsEigenMRP.h"
#include "../_GeneralModuleFiles/fuelSlosh.h"

/*! @brief This class in an instantiation of the state effector class and implements an effector representing a oscillating particle

 The module
 [PDF Description](Basilisk-LINEARSPRINGMASSDAMPER-20180102.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 
 */
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
	StateData *rhoDotState;		   //!< -- state data for time derivative of rho;
	StateData *omegaState;         //!< -- state data for the hubs omega_BN_B
	StateData *sigmaState;         //!< -- state data for the hubs sigma_BN
	StateData *velocityState;      //!< -- state data for the hubs rDot_BN_N

public:
	LinearSpringMassDamper();           //!< -- Contructor
	~LinearSpringMassDamper();          //!< -- Destructor
	void registerStates(DynParamManager& states);  //!< -- Method for SMD to register its states
	void linkInStates(DynParamManager& states);  //!< -- Method for SMD to get access of other states
	void updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
		Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
		Eigen::Vector3d & vecRotcontr);  //!< -- Method for SMD to add contributions to the back-sub method
	void computeDerivatives(double integTime);  //!< -- Method for SMD to compute its derivatives
	void updateEffectorMassProps(double integTime);  //!< -- Method for SMD to add its contributions to mass props
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr);  //!< -- Method for SMD to add contr. to energy and mom.
    void retrieveMassValue(double integTime);
};

#endif /* LINEAR_SPRING_MASS_DAMPER_H */

