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

#include "../../_GeneralModuleFiles/stateEffector.h"
#include "../../_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/THRTimePairMsgPayload.h"
#include "architecture/msgPayloadDefCpp/THROperationMsgPayload.h"
#include "architecture/msgPayloadDefCpp/THRSimConfigMsgPayload.h"
#include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include <vector>



/*! @brief thruster dynamic effector class */
class ThrusterStateEffector: public SysModel, public StateEffector {
public:
    ThrusterStateEffector();
    ~ThrusterStateEffector();
    bool ReadInputs();
    void writeOutputStateMessages(uint64_t CurrentClock);
    void registerStates(DynParamManager& states);  //!< -- Method for the effector to register its states
    void linkInStates(DynParamManager& states);  //!< -- Method for the effector to get access of other states
    void calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B);  //!< -- Force and torque on s/c
    void updateEffectorMassProps(double integTime);  //!< -- Method for stateEffector to give mass contributions
    void updateContributions(double integTime, BackSubMatrices& backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Back-sub contributions
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d& rotAngMomPntCContr_B, double& rotEnergyContr, Eigen::Vector3d omega_BN_B);  //!< -- Energy and momentum calculations
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for each stateEffector to calculate derivatives
    void UpdateState(uint64_t CurrentSimNanos);


    void addThruster(THRSimConfigMsgPayload *newThruster); //!< -- Add a new thruster to the thruster set 

public:
    ReadFunctor<THRArrayOnTimeCmdMsgPayload> cmdsInMsg;  //!< -- input message with thruster commands
    std::vector<Message<THROutputMsgPayload>*> thrusterOutMsgs;  //!< -- output message vector for thruster data

    int stepsInRamp;                               //!< class variable
    std::vector<THRSimConfigMsgPayload> thrusterData; //!< -- Thruster information
    std::vector<double> NewThrustCmds;             //!< -- Incoming thrust commands
    double kappaInit;                //!< [rad] Initial hinged rigid body angle
    double kappaDotInit;             //!< [rad/s] Initial hinged rigid body angle rate
    std::string nameOfKappaState;    //!< -- Identifier for the theta state data container
    std::string nameOfKappaDotState; //!< -- Identifier for the thetaDot state data container
    double mDotTotal;                              //!< kg/s Current mass flow rate of thrusters
    double prevFireTime;                           //!< s  Previous thruster firing time
	StateData *hubSigma;                           //!< class variable
    StateData *hubOmega;                           //!< class varaible
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    std::vector<THROutputMsgPayload> thrusterOutBuffer;//!< -- Message buffer for thruster data
    THRArrayOnTimeCmdMsgPayload incomingCmdBuffer;     //!< -- One-time allocation for savings
    uint64_t prevCommandTime;                       //!< -- Time for previous valid thruster firing
    static uint64_t effectorID;    //!< [] ID number of this panel

};


#endif /* THRUSTER_STATE_EFFECTOR_H */
