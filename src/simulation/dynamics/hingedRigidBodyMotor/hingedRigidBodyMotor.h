/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef HINGEDRIGIDBODYMOTOR_H
#define HINGEDRIGIDBODYMOTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

/*! @brief Calculates a motor torque to drive a hinged panel to a reference angle state. A sensed and reference hinged rigid body angle
           drives a simple PD control law.
 */
class HingedRigidBodyMotor: public SysModel {
public:
    HingedRigidBodyMotor();
    ~HingedRigidBodyMotor();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    
    double K;  //!< gain on theta
    double P; //!< gain on theta dot

    ReadFunctor<HingedRigidBodyMsgPayload> hingedBodyStateSensedInMsg;  //!< sensed rigid body state (theta, theta dot)
    ReadFunctor<HingedRigidBodyMsgPayload> hingedBodyStateReferenceInMsg;  //!< reference hinged rigid body state (theta, theta dot)

    Message<ArrayMotorTorqueMsgPayload> motorTorqueOutMsg;  //!< motor torque on hinged rigid body

    BSKLogger bskLogger;              //!< -- BSK Logging

};


#endif
