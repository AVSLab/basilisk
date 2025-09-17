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


#ifndef JOINTEDARMPDMOTOR_H
#define JOINTEDARMPDMOTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/MJSysMassMatrixMsgPayload.h"
#include "architecture/msgPayloadDefC/JointArrayStateMsgPayload.h"
#include "architecture/msgPayloadDefC/MJNonActuatorForcesMsgPayload.h"
#include "cMsgCInterface/ArrayMotorTorqueMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This module determines the motor torques for a jointed arm using a PD control law.
 */
class JointedArmPDMotor: public SysModel {
public:
    JointedArmPDMotor();
    ~JointedArmPDMotor() = default;

    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<MJSysMassMatrixMsgPayload> massMatrixInMsg;  //!< system mass matrix input msg
    ReadFunctor<JointArrayStateMsgPayload> jointStateInMsg;  //!< current joint states input msg
    ReadFunctor<JointArrayStateMsgPayload> desJointStateInMsg;  //!< desired joint states input msg
    ReadFunctor<MJNonActuatorForcesMsgPayload> nonActForceInMsg;  //!< non-actuator forces input msg

    Message<ArrayMotorTorqueMsgPayload> jointTorqueOutMsg;  //!< joint motor torque C++ output msg
    ArrayMotorTorqueMsg_C               jointTorqueOutMsgC = {};  //!< joint motor torque C output msg

    BSKLogger bskLogger;              //!< BSK Logging

    /** setter for `Ktheta` property */
    void setKtheta(std::vector<double>);
    /** getter for `Ktheta` property */
    Eigen::MatrixXd getKtheta() const {return this->Ktheta;}
    /** setter for `Ptheta` property */
    void setPtheta(std::vector<double>);
    /** getter for `Ptheta` property */
    Eigen::MatrixXd getPtheta() const {return this->Ptheta;}
    /** setter for `u_max` property */
    void setU_max(std::vector<double>);
    /** getter for `u_max` property */
    std::vector<double> getU_max() const {return this->u_max;}

private:
    Eigen::MatrixXd Ktheta;  //!< [1/s^2] proportional gain matrix
    Eigen::MatrixXd Ptheta;  //!< [1/s] derivative gain matrix
    std::vector<double> u_max;  //!< [Nm] (optional) max torque output

};


#endif
