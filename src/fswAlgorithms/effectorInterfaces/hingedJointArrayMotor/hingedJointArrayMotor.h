/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef HINGEDJOINTARRAYMOTOR_H
#define HINGEDJOINTARRAYMOTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefCpp/MJSysMassMatrixMsgPayload.h"
#include "architecture/msgPayloadDefCpp/MJJointReactionsMsgPayload.h"
#include "architecture/msgPayloadDefCpp/JointArrayStateMsgPayload.h"
#include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This module determines the motor torques for an array of hinged joints based on commanded angles and the current system status.
 */
class HingedJointArrayMotor: public SysModel {
public:
    HingedJointArrayMotor() = default; //!< This is the constructor for the module class.
    ~HingedJointArrayMotor() = default; //!< This is the destructor for the module class.

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
    ReadFunctor<MJSysMassMatrixMsgPayload> massMatrixInMsg;  //!< system mass matrix input msg
    ReadFunctor<MJJointReactionsMsgPayload> reactionForcesInMsg;  //!< joint reaction forces and torques input msg
    ReadFunctor<JointArrayStateMsgPayload> desJointStatesInMsg;  //!< desired joint states input msg
    std::vector<ReadFunctor<ScalarJointStateMsgPayload>> jointStatesInMsgs;  //!< vector of joint state input msgs
    std::vector<ReadFunctor<ScalarJointStateMsgPayload>> jointStateDotsInMsgs;  //!< vector of joint state derivative input msgs

    std::vector<Message<SingleActuatorMsgPayload>*> motorTorquesOutMsgs;  //!< vector of joint motor torque output msgs

    BSKLogger bskLogger;              //!< BSK Logging
    /** setter for `Ktheta` property */
    void setKtheta(std::vector<double>);
    /** getter for `Ktheta` property */
    Eigen::MatrixXd getKtheta() const {return this->Ktheta;}
    /** setter for `Ptheta` property */
    void setPtheta(std::vector<double>);
    /** getter for `Ptheta` property */
    Eigen::MatrixXd getPtheta() const {return this->Ptheta;}
    /** setter for `uMax` property */
    void setUMax(std::vector<double>);
    /** getter for `uMax` property */
    std::vector<double> getUMax() const {return this->uMax;}
    /** method for adding a new hinged joint to the system */
    void addHingedJoint();

private:
    int numHingedJoints = 0;  //!< number of hinged joints in the system
    Eigen::MatrixXd Ktheta = Eigen::MatrixXd{};  //!< [1/s^2] proportional gain for hinged joints
    Eigen::MatrixXd Ptheta = Eigen::MatrixXd{};  //!< [1/s] proportional gain for hinged joints
    std::vector<double> uMax = {};  //!< [Nm] (optional) maximum joint motor torque
    struct TreeInfo {
        int freeJointIdx = -1;
        std::vector<int> hingeJointIdxs;
        std::vector<int> hingeGlobalIdxs;
    };                                  //< struct to hold info about each kinematic tree
    bool treeInfoInitialized = false;   //!< flag indicating if tree info has been initialized
    int numKinematicTrees = 0; //!< number of kinematic trees in the system
    std::unordered_map<int, TreeInfo> treeMap; //!< map from kinematic tree index to tree info
};


#endif
