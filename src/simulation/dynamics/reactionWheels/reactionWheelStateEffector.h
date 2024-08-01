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

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include <Eigen/Dense>
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/RWCmdMsgPayload.h"
#include "architecture/msgPayloadDefCpp/RWConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"

#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"



/*! @brief reaction wheel state effector class */
class ReactionWheelStateEffector:  public SysModel, public StateEffector {
public:
    ReactionWheelStateEffector();
	~ReactionWheelStateEffector();
	void registerStates(DynParamManager& states);
	void linkInStates(DynParamManager& states);
    void writeOutputStateMessages(uint64_t integTimeNanos);
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);
    void updateEffectorMassProps(double integTime);  //!< -- Method for stateEffector to give mass contributions
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    void Reset(uint64_t CurrentSimNanos);
    void addReactionWheel(RWConfigMsgPayload *NewRW);
	void UpdateState(uint64_t CurrentSimNanos);
	void WriteOutputMessages(uint64_t CurrentClock);
	void ReadInputs();
	void ConfigureRWRequests(double CurrentTime);

public:
	std::vector<RWConfigMsgPayload *> ReactionWheelData;          //!< -- RW information

	ReadFunctor<ArrayMotorTorqueMsgPayload> rwMotorCmdInMsg;    //!< -- RW motor torque array cmd input message
	Message<RWSpeedMsgPayload> rwSpeedOutMsg;                   //!< -- RW speed array output message
    std::vector<Message<RWConfigLogMsgPayload>*> rwOutMsgs;      //!< -- vector of RW log output messages

    std::vector<RWCmdMsgPayload> NewRWCmds;                     //!< -- Incoming attitude commands
    RWSpeedMsgPayload rwSpeedMsgBuffer = {};                    //!< (-) Output data from the reaction wheels
    std::string nameOfReactionWheelOmegasState;                 //!< class variable
    std::string nameOfReactionWheelThetasState;                 //!< class variable
	size_t numRW;                                               //!< number of reaction wheels
	size_t numRWJitter;                                         //!< number of RW with jitter
    BSKLogger bskLogger;                                        //!< -- BSK Logging

private:
    ArrayMotorTorqueMsgPayload incomingCmdBuffer = {};          //!< -- One-time allocation for savings
	uint64_t prevCommandTime;                                   //!< -- Time for previous valid thruster firing

	StateData *OmegasState;                                     //!< class variable
	StateData *thetasState;                                     //!< class variable
    Eigen::MatrixXd *g_N;           //!< [m/s^2] Gravitational acceleration in N frame components

};


#endif /* STATE_EFFECTOR_H */
