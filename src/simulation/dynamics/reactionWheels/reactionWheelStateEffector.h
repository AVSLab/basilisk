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


#ifndef REACTIONWHEELSTATEEFFECTOR_H
#define REACTIONWHEELSTATEEFFECTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/macroDefinitions.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include <Eigen/Dense>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/RWCmdMsgPayload.h"
#include "simulation/dynamics/_GeneralModuleFiles/RWConfigPayload.h"
#include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"

#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"



/*! @brief reaction wheel state effector class */
class ReactionWheelStateEffector final:  public SysModel, public StateEffector {
public:
    ReactionWheelStateEffector();
	~ReactionWheelStateEffector();
    void registerStates(DynParamManager& states) override;
    void linkInStates(DynParamManager& states) override;
    void writeOutputStateMessages(uint64_t integTimeNanos) override;
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::MRPd sigma_BN) override;
    void updateEffectorMassProps(double integTime) override; //!< Method for stateEffector to give mass contributions
    void updateContributions(double integTime,
                             BackSubMatrices& backSubContr,
                             Eigen::MRPd sigma_BN,
                             Eigen::Vector3d omega_BN_B,
                             Eigen::Vector3d g_N) override; //!< Back-sub contributions
    void updateEnergyMomContributions(double integTime,
                                      Eigen::Vector3d& rotAngMomPntCContr_B,
                                      double& rotEnergyContr,
                                      Eigen::Vector3d omega_BN_B) override; //!< Energy and momentum calculations
    void Reset(uint64_t CurrentSimNanos) override;
    void addReactionWheel(std::shared_ptr<RWConfigPayload> NewRW);
    void UpdateState(uint64_t CurrentSimNanos) override;
    void WriteOutputMessages(uint64_t CurrentClock);
	void ReadInputs();
	void ConfigureRWRequests(double CurrentTime);

public:
	std::vector<std::shared_ptr<RWConfigPayload>> ReactionWheelData;          //!< RW information

	ReadFunctor<ArrayMotorTorqueMsgPayload> rwMotorCmdInMsg;    //!< RW motor torque array cmd input message
	Message<RWSpeedMsgPayload> rwSpeedOutMsg;                   //!< RW speed array output message
    std::vector<Message<RWConfigLogMsgPayload>*> rwOutMsgs;      //!< vector of RW log output messages

    std::vector<RWCmdMsgPayload> NewRWCmds;                     //!< Incoming attitude commands
    RWSpeedMsgPayload rwSpeedMsgBuffer = {};                    //!< (-) Output data from the reaction wheels

    size_t numRW;                                               //!< number of reaction wheels
	size_t numRWJitter;                                         //!< number of RW with jitter
    BSKLogger bskLogger;                                        //!< BSK Logging

    /** @brief Read the current state name; automatic names become final during initialization.
     * @return Current name of the reactionWheelOmegas state.
     */
    const std::string& getNameOfReactionWheelOmegasState() const { return this->nameOfReactionWheelOmegasState; }
    /** @brief Override the state name while preserving legacy assignment behavior.
     * @param value Exact custom name; manager-local names cannot change after registration.
     */
    void setNameOfReactionWheelOmegasState(const std::string& value);
    /** @brief Read the current state name; automatic names become final during initialization.
     * @return Current name of the reactionWheelThetas state.
     */
    const std::string& getNameOfReactionWheelThetasState() const { return this->nameOfReactionWheelThetasState; }
    /** @brief Override the state name while preserving legacy assignment behavior.
     * @param value Exact custom name; manager-local names cannot change after registration.
     */
    void setNameOfReactionWheelThetasState(const std::string& value);

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
    bool effectorNamesResolved = false;                              //!< Names have been applied from a manager.
    std::string nameOfReactionWheelOmegasState;                      //!< Current state name.
    std::optional<std::string> customNameOfReactionWheelOmegasState; //!< Explicit name override.
    std::string nameOfReactionWheelThetasState;                      //!< Current state name.
    std::optional<std::string> customNameOfReactionWheelThetasState; //!< Explicit name override.
#endif
    void validateDimensions();  //!< Validate the wheel count against command and speed message capacities
    void validateRegisteredLayout();  //!< Reject changes to the registered speed and angle state layout
    std::optional<std::vector<bool>> registeredWheelLayout;  //!< Per-wheel jitter-state allocation after registration
    void initializeWheelConfiguration(RWConfigPayload& rw);

    ArrayMotorTorqueMsgPayload incomingCmdBuffer = {};          //!< One-time allocation for savings
	uint64_t prevCommandTime;                                   //!< Time for previous valid thruster firing

	StateData *OmegasState;                                     //!< class variable
	StateData *thetasState;                                     //!< class variable
    Eigen::MatrixXd *g_N;           //!< [m/s^2] Gravitational acceleration in N frame components

    double maxWheelAcceleration = 1.0e6;    //!< [rad/s^2] Maximum allowed wheel acceleration to prevent numerical instability
    double largeTorqueThreshold = 10.0;     //!< [Nm] Threshold for warning about large torque with unlimited torque setting

public:
    /*! @brief Get the maximum wheel acceleration threshold
     * @return Maximum wheel acceleration in rad/s^2
     */
    double getMaxWheelAcceleration() const { return maxWheelAcceleration; }

    /*! @brief Set the maximum wheel acceleration threshold
     * @param val New maximum wheel acceleration value in rad/s^2
     */
    void setMaxWheelAcceleration(double val) { maxWheelAcceleration = val; }

    /*! @brief Get the large torque threshold for unlimited torque warning
     * @return Large torque threshold in Nm
     */
    double getLargeTorqueThreshold() const { return largeTorqueThreshold; }

    /*! @brief Set the large torque threshold for unlimited torque warning
     * @param val New large torque threshold value in Nm
     */
    void setLargeTorqueThreshold(double val) { largeTorqueThreshold = val; }

};


#endif /* STATE_EFFECTOR_H */
