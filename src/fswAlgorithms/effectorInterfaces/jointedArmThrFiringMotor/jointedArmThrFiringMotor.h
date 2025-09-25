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


#ifndef JOINTEDARMTHRFIRINGMOTOR_H
#define JOINTEDARMTHRFIRINGMOTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/MJSysMassMatrixMsgPayload.h"
#include "architecture/msgPayloadDefC/MJNonActuatorForcesMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "cMsgCInterface/ArrayMotorTorqueMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This C++ module determines the needed joint torques to prevent the arms from accelerating when the thrusters fire.
 */
class JointedArmThrFiringMotor: public SysModel {
public:
    JointedArmThrFiringMotor();
    ~JointedArmThrFiringMotor() = default;

    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<MJSysMassMatrixMsgPayload> massMatrixInMsg;  //!< system mass matrix input msg
    ReadFunctor<MJNonActuatorForcesMsgPayload> nonActForceInMsg;  //!< non-actuator forces input msg
    ReadFunctor<CmdForceBodyMsgPayload> bodyForceInMsg;  //!< body force input msg
    ReadFunctor<CmdTorqueBodyMsgPayload> bodyTorqueInMsg;  //!< body torque input msg
    ReadFunctor<ArrayMotorTorqueMsgPayload> jointTorqueInMsg;  //!< joint torque input msg

    Message<ArrayMotorTorqueMsgPayload> jointTorqueOutMsg;  //!< joint motor torque C++ output msg
    ArrayMotorTorqueMsg_C               jointTorqueOutMsgC = {};  //!< joint motor torque C output msg

    BSKLogger bskLogger;              //!< BSK Logging

    /** setter for `u_max` property */
    void setU_max(std::vector<double>);
    /** getter for `u_max` property */
    std::vector<double> getU_max() const {return this->u_max;}

private:
    std::vector<double> u_max;  //!< [Nm] (optional) max torque output

};


#endif
