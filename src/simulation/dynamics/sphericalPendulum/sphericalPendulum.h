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

#include <optional>
#include <cstdint>
#include <string>
#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#ifndef SWIG
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#endif
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "simulation/dynamics/_GeneralModuleFiles/fuelSlosh.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief spherical pendulum state effector model */
class SphericalPendulum final :
	public StateEffector, public SysModel, public FuelSlosh
{
public:
	double pendulumRadius;             //!< [m] distance between the center of the tank and the spherical pendulum mass
    Eigen::Matrix3d D;                    //!< [N*s/m] symmetric positive-semidefinite damping matrix in P0 components
    double phiDotInit;             //!< [rad/s] Initial value for spherical pendulum pendulum offset derivative
    double thetaDotInit;             //!< [rad/s] Initial value for spherical pendulum pendulum offset derivative
    double massInit;               //!< [kg] Initial value for spherical pendulum mass

	Eigen::Vector3d d;        //!< [m] position vector from B point to tank center , T, in body frame
	StateData *massState = nullptr;		   //!< state data for the pendulums mass
    Eigen::Vector3d pHat_01;      //!< first vector of the P0 frame in B frame components
    Eigen::Vector3d pHat_02;       //!< second vector of the P0 frame in  B frame components
    Eigen::Vector3d pHat_03;        //!< third vector of the P0 frame in B frame components
		BSKLogger bskLogger;                      //!< BSK Logging

    /** @brief Set the explicit phi state name; Python retains nameOfPhiState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfPhiState(const std::string& value);
    /** @brief Get the current phi state name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfPhiState() const { return this->nameOfPhiState; }
    /** @brief Set the explicit theta state name; Python retains nameOfThetaState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfThetaState(const std::string& value);
    /** @brief Get the current theta state name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfThetaState() const { return this->nameOfThetaState; }
    /** @brief Set the explicit phiDot state name; Python retains nameOfPhiDotState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfPhiDotState(const std::string& value);
    /** @brief Get the current phiDot state name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfPhiDotState() const { return this->nameOfPhiDotState; }
    /** @brief Set the explicit thetaDot state name; Python retains nameOfThetaDotState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfThetaDotState(const std::string& value);
    /** @brief Get the current thetaDot state name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfThetaDotState() const { return this->nameOfThetaDotState; }
    /** @brief Set the explicit mass state name; Python retains nameOfMassState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfMassState(const std::string& value);
    /** @brief Get the current mass state name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfMassState() const { return this->nameOfMassState; }

private:
    std::string nameOfPhiState; //!< Current phi state name.
    std::optional<std::string> customPhiState; //!< Explicit override, independent of automatic names.
    std::string nameOfThetaState; //!< Current theta state name.
    std::optional<std::string> customThetaState; //!< Explicit override, independent of automatic names.
    std::string nameOfPhiDotState; //!< Current phiDot state name.
    std::optional<std::string> customPhiDotState; //!< Explicit override, independent of automatic names.
    std::string nameOfThetaDotState; //!< Current thetaDot state name.
    std::optional<std::string> customThetaDotState; //!< Explicit override, independent of automatic names.
    std::string nameOfMassState; //!< Current mass state name.
    std::optional<std::string> customMassState; //!< Explicit override, independent of automatic names.
    bool effectorNamesResolved = false; //!< Prevent changes to registered manager-local names.
    /** @brief Record an explicit name assignment, enforcing resolved-name immutability.
     * @param currentName Currently visible name to update.
     * @param customName Metadata identifying an explicit override.
     * @param value Exact custom name.
     */
    void setCustomName(std::string& currentName, std::optional<std::string>& customName, const std::string& value);
    /** @brief Apply the prepared names before registering effector states.
     * @param manager Dynamics manager holding this effector's declaration.
     */
    void applyResolvedNames(DynParamManager& manager);

    void validateConfiguration(); //!< Validate initial mass and damping without changing the current states
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

    Eigen::Vector3d aPhi;          //!< Term needed for back-sub method
    Eigen::Vector3d bPhi;          //!< Term needed for back-sub method
    Eigen::Vector3d aTheta;          //!< Term needed for back-sub method
    Eigen::Vector3d bTheta;          //!< Term needed for back-sub method
    double cPhi;                   //!< Term needed for back-sub method
    double cTheta;                   //!< Term needed for back-sub method

    Eigen::MatrixXd *g_N;      //!< [m/s^2] Gravitational acceleration in N frame components
    Eigen::Vector3d l_B;         //!< [m] vector from the center of the tank to the spherical pendulum pendulum in B frame
	Eigen::Vector3d lPrime_B;    //!< [m/s] derivative of l respect to B frame
    Eigen::Vector3d lPrime_P0;  //!< [m/s] derivative of l in P0 frame
	StateData *phiState;	   //!< state data for spherical pendulum displacement
	StateData *thetaState;	   //!< state data for spherical pendulum displacement
	StateData *phiDotState;     //!< state data for time derivative of phi;
	StateData *thetaDotState;		   //!< state data for time derivative of theta;
    Eigen::Matrix3d dcm_B_P0;      // Rotation matrix from P0 to B frame
    static uint64_t effectorID;        //!< [] ID number of this panel



public:
	SphericalPendulum();           //!< Constructor
	~SphericalPendulum();          //!< Destructor
    void Reset(uint64_t CurrentSimNanos) override;
	void registerStates(DynParamManager& states) override;  //!< Method for FSP to register its states
	void linkInStates(DynParamManager& states) override;  //!< Method for FSP to get access of other states
	void updateEffectorMassProps(double integTime) override;  //!< Method for FSP to add its contributions to mass props
    void modifyStates(double integTime) override; //!< Method to force states modification during integration
    void retrieveMassValue(double integTime) override;
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N) override;  //!< Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B) override;  //!< Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN) override;  //!< Method for each stateEffector to calculate derivatives
#ifndef SWIG
protected:
    /** @brief Declare the state names allocated together for this effector.
     * @return Group with a shared automatic index and independently tracked custom names.
     */
    EffectorNameGroup describeEffectorNames() const override;
#endif
};


#endif /* SPHERICAL_PENDULUM_H */
