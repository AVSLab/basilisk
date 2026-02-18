/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef THRJOINTCOMPENSATION_H
#define THRJOINTCOMPENSATION_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefCpp/THRArmConfigMsgPayload.h"
#include "architecture/msgPayloadDefCpp/MJSysMassMatrixMsgPayload.h"
#include "architecture/msgPayloadDefCpp/MJJointReactionsMsgPayload.h"
#include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This module determines the motor torques needed to compensate for the disturbance torques caused by
the current system configuration and the thrusters when they are firing
 */
class ThrJointCompensation: public SysModel {
public:
    ThrJointCompensation() = default; //!< This is the constructor for the module class.
    ~ThrJointCompensation() = default; //!< This is the destructor for the module class.

    /*!
    * This method is used to reset the module and checks that required input messages are connected.
    */
    void Reset(uint64_t CurrentSimNanos);

    /*!
    * This is the main method that gets called every time the module is updated.
    * It computes the motor torques for the full array of hinged joints.
    * */
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<THRArmConfigMsgPayload> armConfigInMsg;  //!< static spacecraft configuration input msg
    ReadFunctor<MJSysMassMatrixMsgPayload> massMatrixInMsg;  //!< system mass matrix input msg
    ReadFunctor<MJJointReactionsMsgPayload> reactionForcesInMsg;  //!< joint reaction forces and torques input msg
    std::vector<ReadFunctor<ScalarJointStateMsgPayload>> jointStatesInMsgs;  //!< vector of joint state input msgs
    std::vector<ReadFunctor<SingleActuatorMsgPayload>> thrForcesInMsgs;  //!< vector of thruster force input msgs

    std::vector<Message<SingleActuatorMsgPayload>*> motorTorquesOutMsgs;  //!< vector of joint motor torque output msgs

    BSKLogger bskLogger;              //!< BSK Logging

    /** setter for `uMax` property */
    void setUMax(std::vector<double>);
    /** getter for `uMax` property */
    std::vector<double> getUMax() const {return this->uMax;}
    /** method for adding a new hinged joint to the system */
    void addHingedJoint();
    /** method for adding a new thruster to the system */
    void addThruster();

private:
    int numHingedJoints = 0;  //!< number of hinged joints in the system
    int numThrusters = 0;  //!< number of thrusters in the system
    std::vector<double> uMax = {};  //!< [Nm] (optional) maximum joint motor torque
    struct TreeInfo {
        int freeJointIdx = -1;
        std::vector<int> hingeJointIdxs;
        std::vector<int> hingeGlobalIdxs;
    };                                  //!< struct to hold info about each kinematic tree
    bool treeInfoInitialized = false;   //!< flag indicating if tree info has been initialized
    int numKinematicTrees = 0; //!< number of kinematic trees in the system
    std::vector<int> hingeToTree;   //!< maps global hinge index to kinematic tree id
    std::unordered_map<int, TreeInfo> treeMap; //!< map from kinematic tree index to tree info
    struct JointPoseCache {
        Eigen::Matrix3d dcm_CB;
        Eigen::Vector3d r_CB_B;
    };                                  //!< struct to hold the pose information for each joint
    std::vector<JointPoseCache> jointPoseFlat;  //!< vector of flattened joint pose info
    struct ArmKinematicsConfig {
        std::vector<int> thrArmIdx;
        std::vector<int> thrArmJointIdx;
        std::vector<int> armTreeIdx;
        std::vector<int> armJointStart;
        std::vector<int> armJointCount;
        std::vector<int> armHingeGlobalIdx;
        std::vector<double> r_CP_P;
        std::vector<double> r_TP_P;
        std::vector<double> shat_P;
        std::vector<double> fhat_P;
        std::vector<double> dcm_C0P;
    };
    ArmKinematicsConfig kinCfg;      //!< struct holding the information from the static armConfigInMsg
};


#endif
