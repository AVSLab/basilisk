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

/*! @brief linear spring mass damper state effector class */
class LinearSpringMassDamper final :
	public StateEffector, public SysModel, public FuelSlosh
{
public:
    double k;                      //!< [N/m] linear spring constant for spring mass damper
    double c;                      //!< [N-s/m] linear damping term for spring mass damper
    double rhoInit;                //!< [m] Initial value for spring mass damper particle offset
    double rhoDotInit;             //!< [m/s] Initial value for spring mass damper particle offset derivative
    double massInit;               //!< [kg] Initial value for spring mass damper particle mass

	Eigen::Vector3d r_PB_B;        //!< [m] position vector from B point to particle equilibrium, P, in body frame
	Eigen::Vector3d pHat_B;        //!< [-] particle direction unit vector, in body frame
	StateData *massState = nullptr;		   //!< state data for the particles mass
	BSKLogger bskLogger;                      //!< BSK Logging

    /** @brief Set the explicit rho state name; Python retains nameOfRhoState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfRhoState(const std::string& value);
    /** @brief Get the current rho state name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfRhoState() const { return this->nameOfRhoState; }
    /** @brief Set the explicit rhoDot state name; Python retains nameOfRhoDotState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfRhoDotState(const std::string& value);
    /** @brief Get the current rhoDot state name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfRhoDotState() const { return this->nameOfRhoDotState; }
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
    std::string nameOfRhoState; //!< Current rho state name.
    std::optional<std::string> customRhoState; //!< Explicit override, independent of automatic names.
    std::string nameOfRhoDotState; //!< Current rhoDot state name.
    std::optional<std::string> customRhoDotState; //!< Explicit override, independent of automatic names.
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

    void validateConfiguration(); //!< Validate the initial particle mass without changing its current state
    double cRho;                   //!< Term needed for back-sub method
    double rho;                    //!< [m] spring mass damper displacement from equilibrium
    double rhoDot;                 //!< [m/s] time derivative of displacement from equilibrium
	double massSMD;                //!< [kg] mass of spring mass damper particle
    Eigen::Vector3d r_PcB_B;       //!< [m] position vector form B to center of mass location of particle
    Eigen::Matrix3d rTilde_PcB_B;  //!< [m] tilde matrix of r_Pc_B
	Eigen::Vector3d rPrime_PcB_B;  //!< [m/s] Body time derivative of r_Pc_B
	Eigen::Matrix3d rPrimeTilde_PcB_B;  //!< [m/s] Tilde matrix of rPrime_PcB_B
    Eigen::Vector3d aRho;          //!< Term needed for back-sub method
    Eigen::Vector3d bRho;          //!< Term needed for back-sub method
    Eigen::MatrixXd *g_N;          //!< [m/s^2] Gravitational acceleration in N frame components
	StateData *rhoState;		   //!< state data for spring mass damper displacement from equilibrium
    Eigen::MatrixXd *c_B;            //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd *cPrime_B;       //!< [m/s] Body time derivative of vector c_B in B frame components
	StateData *rhoDotState;		   //!< state data for time derivative of rho;
    static uint64_t effectorID;    //!< [] ID number of this panel

public:
	LinearSpringMassDamper();           //!< Constructor
	~LinearSpringMassDamper();          //!< Destructor
    void Reset(uint64_t CurrentSimNanos) override;
	void registerStates(DynParamManager& states) override;  //!< Method for SMD to register its states
	void linkInStates(DynParamManager& states) override;  //!< Method for SMD to get access of other states
    void retrieveMassValue(double integTime) override;
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B) override;  //!< Force and torque on s/c due to linear spring mass damper
    void updateEffectorMassProps(double integTime) override;  //!< Method for stateEffector to give mass contributions
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


#endif /* LINEAR_SPRING_MASS_DAMPER_H */
