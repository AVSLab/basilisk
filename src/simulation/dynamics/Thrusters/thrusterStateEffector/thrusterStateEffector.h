/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#ifndef THRUSTER_STATE_EFFECTOR_H
#define THRUSTER_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#ifndef SWIG
#include "simulation/dynamics/_GeneralModuleFiles/effectorName.h"
#endif
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/THRSimConfig.h"
#include "simulation/dynamics/_GeneralModuleFiles/THROperation.h"
#include "simulation/dynamics/_GeneralModuleFiles/BodyToHubInfo.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include <vector>
#include <cstdint>
#include <memory>
#include <cstddef>
#include <optional>
#include <string>
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "architecture/utilities/avsEigenMRP.h"




/*! @brief thruster dynamic effector class */
class ThrusterStateEffector final: public StateEffector, public SysModel {
public:
    ThrusterStateEffector();
    ~ThrusterStateEffector();
    void Reset(uint64_t CurrentSimNanos) override;
    bool ReadInputs();
    void writeOutputStateMessages(uint64_t CurrentClock) override;
    void registerStates(DynParamManager& states) override;  //!< Method for the effector to register its states
    void linkInStates(DynParamManager& states) override;  //!< Method for the effector to get access of other states
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN) override;  //!< Method for each stateEffector to calculate derivatives
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B) override;
    void updateContributions(double integTime, BackSubMatrices& backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N) override;  //!< Method to pass the forces and torques onto the hub
    void updateEffectorMassProps(double integTime) override;
    void UpdateState(uint64_t CurrentSimNanos) override;


    void addThruster(std::shared_ptr<THRSimConfig> newThruster); //!< Add a new thruster to the thruster set
    void addThruster(std::shared_ptr<THRSimConfig> newThruster, Message<SCStatesMsgPayload>* bodyStateMsg); //!< (overloaded) Add a new thruster to the thruster set connect to a body different than the hub
    void ConfigureThrustRequests();
    void UpdateThrusterProperties();

public:
    // Input and output messages
    ReadFunctor<THRArrayOnTimeCmdMsgPayload> cmdsInMsg;  //!< input message with thruster commands
    std::vector<Message<THROutputMsgPayload>*> thrusterOutMsgs;  //!< output message vector for thruster data
    std::vector<std::shared_ptr<THRSimConfig>> thrusterData; //!< Thruster information
    std::vector<double> NewThrustCmds;             //!< Incoming thrust commands

    // State information
    std::vector<double> kappaInit;                //!< [] Vector of initial thruster states

    // State structures
	StateData *hubSigma;        //!< pointer to hub attitude states
    StateData *hubOmega;        //!< pointer to hub angular velocity states
    StateData* kappaState;      //!< state manager of theta for hinged rigid body
    Eigen::MatrixXd* inertialPositionProperty;  //!< [m] r_N inertial position relative to system spice zeroBase/refBase
    Eigen::Vector3d r_PcP_P;                    //!< [m] position vector of parent body CoM w.r.t. parent body frame origin (only used if thrusters attached to non-hub body)

    BSKLogger bskLogger;        //!< BSK Logging

    // Mass flow rate
    double mDotTotal = 0.0;           //!< [kg/s] Current mass flow rate of thrusters

    /** @brief Set the explicit kappa state name; Python retains nameOfKappaState.
     * @param value Exact custom name, including names that match an automatic name.
     * @note Manager-local names cannot change after registration.
     */
    void setNameOfKappaState(const std::string& value);
    /** @brief Get the current kappa state name.
     * @return Legacy name before preparation, or the resolved name after registration.
     */
    const std::string& getNameOfKappaState() const { return this->nameOfKappaState; }

private:
    std::string nameOfKappaState; //!< Current kappa state name.
    std::optional<std::string> customKappaState; //!< Explicit override, independent of automatic names.
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

    void validateConfiguration();  //!< Validate dimensions before accessing thruster data or commands
    void validateRegisteredCount();  //!< Reject changes to the number of registered thrust-factor states
    std::optional<std::size_t> registeredThrusterCount;  //!< Thruster count after successful state registration
    std::vector<THROutputMsgPayload> thrusterOutBuffer;//!< Message buffer for thruster data

    THRArrayOnTimeCmdMsgPayload incomingCmdBuffer;     //!< One-time allocation for savings

    std::vector<ReadFunctor<SCStatesMsgPayload>> attachedBodyInMsgs;       //!< vector of body states message where the thrusters attach to
    SCStatesMsgPayload attachedBodyBuffer;
    std::vector<BodyToHubInfo> bodyToHubInfo;

    double prevCommandTime;                       //!< [s] Time for previous valid thruster firing
    static uint64_t effectorID;    //!< [] ID number of this panel
#ifndef SWIG
protected:
    /** @brief Declare the state names allocated together for this effector.
     * @return Group with a shared automatic index and independently tracked custom names.
     */
    EffectorNameGroup describeEffectorNames() const override;
#endif
};


#endif /* THRUSTER_STATE_EFFECTOR_H */
