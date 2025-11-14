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


#ifndef MJJOINTREACTIONFORCES_H
#define MJJOINTREACTIONFORCES_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefCpp/MJJointReactionsMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJScene.h"

/*! @brief This C++ module extracts the reaction forces and torques acting on the joints from a MuJoCo scene
 */
class MJJointReactionForces: public SysModel {
public:
    MJJointReactionForces() = default;  //!< This is the constructor for the module class.
    ~MJJointReactionForces() = default; //!< This is the destructor for the module class.

    /**
     * This method is used to reset the module and checks that scene is setup.
     */
    void Reset(uint64_t CurrentSimNanos);

    /**
     * This is the main method that gets called every time the module is updated.
     * It is used to read the reaction forces and torques acting on the joints from the MuJoCo scene
     * and write them to an output message.
     */
    void UpdateState(uint64_t CurrentSimNanos);

public:
    MJScene* scene{nullptr};  //!< pointer to the MuJoCo scene

    /**
     * joint reaction forces and torques output msg
     */
    Message<MJJointReactionsMsgPayload> reactionForcesOutMsg;

    BSKLogger bskLogger;              //!< BSK Logging
private:

    std::size_t nDOF = 0;                     //!< number of total DOF
    std::vector<int> jointTreeIdx;            //!< list of the kinematic tree index for each joint
    std::vector<int> jointParentBodyIdx;      //!< list of the parent body index for each joint
    std::vector<int> jointTypes;              //!< list of joint types in the system
    std::vector<int> jointDOFStart;           //!< list of the starting DOF index for each joint

};


#endif
