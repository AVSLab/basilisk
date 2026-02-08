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


#ifndef JOINTMOTIONCOMPENSATOR_H
#define JOINTMOTIONCOMPENSATOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefCpp/MJSysMassMatrixMsgPayload.h"
#include "architecture/msgPayloadDefCpp/MJJointReactionsMsgPayload.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This module determines the hub torques required to negate the reaction torques induced by moving joints on a spacecraft.
 */
class JointMotionCompensator: public SysModel {
public:
    JointMotionCompensator() = default; //!< This is the constructor for the module class.
    ~JointMotionCompensator() = default; //!< This is the destructor for the module class.

    /*!
    * This method is used to reset the module and checks that required input messages are connected.
    */
    void Reset(uint64_t CurrentSimNanos);

    /*!
    * This is the main method that gets called every time the module is updated.
    * It computes the hub torques needed to negate the reaction torques induced by moving joints.
    * */
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<MJSysMassMatrixMsgPayload> massMatrixInMsg;  //!< system mass matrix input msg
    ReadFunctor<MJJointReactionsMsgPayload> reactionForcesInMsg;  //!< joint reaction forces and torques input msg
    std::vector<ReadFunctor<SingleActuatorMsgPayload>> jointTorqueInMsgs;  //!< vector of joint motor torque input msgs

    std::vector<Message<SingleActuatorMsgPayload>*> hubTorqueOutMsgs;  //!< vector of hub torque output msgs

    BSKLogger bskLogger;              //!< BSK Logging

    /** setter for `uMax` property */
    void setUMax(std::vector<double>);
    /** getter for `uMax` property */
    std::vector<double> getUMax() const {return this->uMax;}
    /** method for adding a new spacecraft to the system */
    void addSpacecraft();
    /** method for adding a new hinged joint to the system */
    void addHingedJoint();

private:
    std::vector<double> uMax = {};  //!< [Nm] (optional) maximum hub motor torque
    int numSpacecraft = 0;       //!< [-] number of spacecraft in the system
    int numHingedJoints = 0;     //!< [-] number of hinged joints in the system
    bool treeInfoInitialized = false;  //!< [-] flag indicating if the treeInfo struct has been initialized
    struct TreeInfo{
        int freeJointIdx = -1;      //!< [-] free joint index
        std::vector<int> hingeJointIdxs;   //!< [-] per-tree hinged joint indices
        std::vector<int> hingeGlobalIdxs;    //!< [-] global indices of hinged joints
    }; //< struct to hold info about each kinematic tree
    std::unordered_map<int, TreeInfo> treeMap;  //!< map holding tree info for each kinematic tree

};


#endif
