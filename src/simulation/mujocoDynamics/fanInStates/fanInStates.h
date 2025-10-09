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


#ifndef FANINSTATES_H
#define FANINSTATES_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "cMsgCInterface/JointArrayStateMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJScene.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This single use module fans in all the joint states into a single output message
 */
class FanInStates: public SysModel {
public:
    FanInStates();
    ~FanInStates() = default;

    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:

    MJScene* scene{nullptr};  //!< pointer to the MuJoCo scene

    Message<JointArrayStateMsgPayload> jointStatesOutMsg;  //!< joint states C++ output msg
    JointArrayStateMsg_C               jointStatesOutMsgC = {};  //!< joint states C output msg

    BSKLogger bskLogger;              //!< BSK Logging


};


#endif
