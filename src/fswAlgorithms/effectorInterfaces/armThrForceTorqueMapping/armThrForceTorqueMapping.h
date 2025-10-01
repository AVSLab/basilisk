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


#ifndef ARMTHRFORCETORQUEMAPPING_H
#define ARMTHRFORCETORQUEMAPPING_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/MJSCConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/JointArrayStateMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayCmdForceMsgPayload.h"
#include "cMsgCInterface/CmdTorqueBodyMsg_C.h"
#include "cMsgCInterface/CmdForceBodyMsg_C.h"
#include "cMsgCInterface/ArrayMotorTorqueMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This C++ module maps the thruster forces to body force and torques and joint torquesfor a spacecraft with arm mounted thrusters.
 */
class ArmThrForceTorqueMapping: public SysModel {
public:
    ArmThrForceTorqueMapping();
    ~ArmThrForceTorqueMapping() = default;

    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<MJSCConfigMsgPayload> configInMsg;  //!< spacecraft configuration input msg
    ReadFunctor<JointArrayStateMsgPayload> jointStatesInMsg;  //!< joint states input msg
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;  //!< thruster forces input msg

    Message<CmdForceBodyMsgPayload> bodyForceOutMsg;  //!< body force C++ output msg
    CmdForceBodyMsg_C               bodyForceOutMsgC = {};  //!< body force C output msg
    Message<CmdTorqueBodyMsgPayload> bodyTorqueOutMsg;  //!< body torque C++ output msg
    CmdTorqueBodyMsg_C               bodyTorqueOutMsgC = {};  //!< body torque C output msg
    Message<ArrayMotorTorqueMsgPayload> jointTorqueOutMsg;  //!< joint torque C++ output msg
    ArrayMotorTorqueMsg_C               jointTorqueOutMsgC = {};  //!< joint torque C output msg

    BSKLogger bskLogger;              //!< BSK Logging

private:

    Eigen::Matrix3d dcm_H1B;      //< [-] DCM from hinge1 frame to body frame
    Eigen::Matrix3d dcm_H2B;      //< [-] DCM from hinge2 frame to body frame
    Eigen::Matrix<double,3,2> thr_F_T;   //< [-] thruster force directions in thruster frame
    Eigen::Matrix<double,3,10> r_S2S1_S1;    //< [m] position vector from site 2 to site 1 in site 1 frame
};


#endif
