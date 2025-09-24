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


#ifndef HUBPRESCRIBEDTORQUE_H
#define HUBPRESCRIBEDTORQUE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/MJSysMassMatrixMsgPayload.h"
#include "architecture/msgPayloadDefC/MJNonActuatorForcesMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "cMsgCInterface/CmdTorqueBodyMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This module determines the torque on the hub required to negate hub angular acceleration
resulting from joint motor torques.
 */
class HubPrescribedTorque: public SysModel {
public:
    HubPrescribedTorque();
    ~HubPrescribedTorque() = default;

    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<MJSysMassMatrixMsgPayload> massMatrixInMsg;  //!< system mass matrix input msg
    ReadFunctor<MJNonActuatorForcesMsgPayload> nonActForceInMsg;  //!< non-actuator forces input msg
    ReadFunctor<ArrayMotorTorqueMsgPayload> jointTorqueInMsg;  //!< joint motor torques input msg

    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;  //!< prescribed hub torque C++ output msg in body frame components
    CmdTorqueBodyMsg_C               cmdTorqueOutMsgC = {};  //!< prescribed hub torque C output msg in body frame components

    BSKLogger bskLogger;              //!< BSK Logging


};


#endif
