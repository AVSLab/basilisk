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


#ifndef VSCMG_GIMBAL_RATE_SERVO_H
#define VSCMG_GIMBAL_RATE_SERVO_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/VSCMGArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/VSCMGRefStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/VSCMGSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/VSCMGArrayTorqueMsgPayload.h"
#include "cMsgCInterface/VSCMGArrayTorqueMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include <Eigen/Dense>
#include "architecture/utilities/avsEigenSupport.h"

/*! @brief Mapping desired gimbal rates and RW wheel accelerations to motor torques.
 */
class VscmgGimbalRateServo: public SysModel {
public:
    VscmgGimbalRateServo();
    ~VscmgGimbalRateServo() = default;
    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<VSCMGArrayConfigMsgPayload> vsmcgParamsInMsg;  //!< [-] VSCMG array configuration input message
    ReadFunctor<VSCMGRefStatesMsgPayload> vscmgRefStatesInMsg;  //!< [-] reference VSCMG states input message
    ReadFunctor<NavAttMsgPayload> attInMsg;  //!< [-] attitude navigation input message
    ReadFunctor<VSCMGSpeedMsgPayload> speedsInMsg;  //!< [-] VSCMG speeds input message
    Message<VSCMGArrayTorqueMsgPayload> cmdsOutMsg;  //!< [-] VSCMG motor torque C++ output message
    VSCMGArrayTorqueMsg_C               cmdsOutMsgC = {};    //!< [-] VSCMG motor torque C output message
    BSKLogger bskLogger;              //!< [-] BSK Logging

    void setK_gammaDot(double);  //!< [-] setter for `K-gammaDot` property
    double getK_gammaDot() const {return this->K_gammaDot;}   //!< [-] getter for `K-gammaDot` property

private:
    double K_gammaDot;  //!< [1/s] proportional gain applied to gimbal rate errors
    VSCMGArrayTorqueMsgPayload outputTorques;   //!< [-] output torques for the VSCMGs
    VSCMGArrayConfigMsgPayload vscmgConfigParams;  //!< [-] struct to store message containing VSCMG config parameters in body B frame

};


#endif
