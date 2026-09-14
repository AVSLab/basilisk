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


#ifndef VSCMGSTATEEFFECTOR_H
#define VSCMGSTATEEFFECTOR_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "../_GeneralModuleFiles/dynParamManager.h"
#include "../_GeneralModuleFiles/dynamicEffector.h"
#include "../_GeneralModuleFiles/dynamicObject.h"
#include "../_GeneralModuleFiles/stateData.h"
#include "../_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#include <Eigen/Dense>

#include "architecture/msgPayloadDefC/VSCMGSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/VSCMGCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/VSCMGArrayTorqueMsgPayload.h"
#include "architecture/msgPayloadDefCpp/VSCMGConfigMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"



/*! @brief VSCMG state effector class */
class VSCMGStateEffector final:  public SysModel, public StateEffector {
public:
    VSCMGStateEffector();
	~VSCMGStateEffector();
    void registerStates(DynParamManager& states) override;
    void linkInStates(DynParamManager& states) override;
    void updateEffectorMassProps(double integTime) override;
    void Reset(uint64_t CurrentSimNanos) override;
    void AddVSCMG(VSCMGConfigMsgPayload *NewVSCMG);
    void UpdateState(uint64_t CurrentSimNanos) override;
    void WriteOutputMessages(uint64_t CurrentClock);
	void ReadInputs();
	void ConfigureVSCMGRequests(double CurrentTime);
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::MRPd sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override; //!< [-] Back-sub contributions
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override; //!< [-] Energy and momentum calculations
    void computeDerivatives(
      double integTime,
      Eigen::Vector3d rDDot_BN_N,
      Eigen::Vector3d omegaDot_BN_B,
      Eigen::MRPd sigma_BN) override; //!< [-] Method for each stateEffector to calculate derivatives

public:
	std::vector<VSCMGConfigMsgPayload> VSCMGData; //!< [-] VSCMG data structure
    Eigen::MatrixXd *g_N; 						//!< [m/s^2] Gravitational acceleration in N frame components

    ReadFunctor<VSCMGArrayTorqueMsgPayload> cmdsInMsg;  //!< [-] motor torque command input message
	Message<VSCMGSpeedMsgPayload> speedOutMsg; 	        //!< [-] VSCMG speed output message
    std::vector<Message<VSCMGConfigMsgPayload>*> vscmgOutMsgs;   //!< [-] vector of VSCMG output messages

    std::vector<VSCMGCmdMsgPayload> newVSCMGCmds; 	//!< [-] Incoming torque commands
	VSCMGSpeedMsgPayload outputStates{}; 			//!< [-] Output data from the VSCMGs

    int numVSCMG = 0;                           //!< [-] number of VSCMGs
	int numVSCMGJitter = 0;                     //!< [-] number of VSCMGs with jitter
    BSKLogger bskLogger;                      //!< [-] BSK Logging

    /** @brief Read the current state name; automatic names become final during initialization.
     * @return Current name of the VSCMGOmegas state.
     */
    const std::string& getNameOfVSCMGOmegasState() const { return this->nameOfVSCMGOmegasState; }
    /** @brief Override the state name while preserving legacy assignment behavior.
     * @param value Exact custom name; manager-local names cannot change after registration.
     */
    void setNameOfVSCMGOmegasState(const std::string& value);
    /** @brief Read the current state name; automatic names become final during initialization.
     * @return Current name of the VSCMGThetas state.
     */
    const std::string& getNameOfVSCMGThetasState() const { return this->nameOfVSCMGThetasState; }
    /** @brief Override the state name while preserving legacy assignment behavior.
     * @param value Exact custom name; manager-local names cannot change after registration.
     */
    void setNameOfVSCMGThetasState(const std::string& value);
    /** @brief Read the current state name; automatic names become final during initialization.
     * @return Current name of the VSCMGGammas state.
     */
    const std::string& getNameOfVSCMGGammasState() const { return this->nameOfVSCMGGammasState; }
    /** @brief Override the state name while preserving legacy assignment behavior.
     * @param value Exact custom name; manager-local names cannot change after registration.
     */
    void setNameOfVSCMGGammasState(const std::string& value);
    /** @brief Read the current state name; automatic names become final during initialization.
     * @return Current name of the VSCMGGammaDots state.
     */
    const std::string& getNameOfVSCMGGammaDotsState() const { return this->nameOfVSCMGGammaDotsState; }
    /** @brief Override the state name while preserving legacy assignment behavior.
     * @param value Exact custom name; manager-local names cannot change after registration.
     */
    void setNameOfVSCMGGammaDotsState(const std::string& value);

#ifndef SWIG
    /** @brief Declare the complete group of state names for this device array.
     * @return Automatic patterns and explicit overrides.
     */
    EffectorNameGroup describeEffectorNames() const override;
#endif

private:
#ifndef SWIG
    /** @brief Apply the manager's resolved names before state registration.
     * @param manager Manager that owns this effector's name request.
     */
    void applyResolvedNames(DynParamManager& manager);
    /** @brief Track explicit assignments independently of constructor defaults.
     * @param currentName Current public name.
     * @param customName Explicit override, if one was supplied.
     * @param value Requested name.
     */
    void setCustomName(std::string& currentName, std::optional<std::string>& customName, const std::string& value);
    bool effectorNamesResolved = false;                         //!< Names have been applied from a manager.
    std::string nameOfVSCMGOmegasState;                         //!< Current state name.
    std::optional<std::string> customNameOfVSCMGOmegasState;    //!< Explicit name override.
    std::string nameOfVSCMGThetasState;                         //!< Current state name.
    std::optional<std::string> customNameOfVSCMGThetasState;    //!< Explicit name override.
    std::string nameOfVSCMGGammasState;                         //!< Current state name.
    std::optional<std::string> customNameOfVSCMGGammasState;    //!< Explicit name override.
    std::string nameOfVSCMGGammaDotsState;                      //!< Current state name.
    std::optional<std::string> customNameOfVSCMGGammaDotsState; //!< Explicit name override.
#endif
    /** @brief Validate and derive configuration without requiring linked states or clearing commands. */
    void initializeConfiguration();
    VSCMGArrayTorqueMsgPayload incomingCmdBuffer{}; //!< [-] One-time allocation for savings
	uint64_t prevCommandTime;                  	//!< [-] Time for previous valid thruster firing

	StateData *hubOmega;                        //!< [rad/s] angular velocity of the B frame wrt the N frame in B frame components
	StateData *OmegasState;                     //!< [rad/s] RW spin state
	StateData *thetasState;                     //!< [rad] RW angle
	StateData *gammasState;                     //!< [rad] CMG gimbal angle
	StateData *gammaDotsState;                  //!< [rad/s] CMG gimbal angle rate

};


#endif /* STATE_EFFECTOR_H */
