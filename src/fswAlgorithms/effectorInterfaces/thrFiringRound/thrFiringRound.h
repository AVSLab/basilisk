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


#ifndef THRFIRINGROUND_H
#define THRFIRINGROUND_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "cMsgCInterface/THRArrayCmdForceMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <Eigen/Dense>

/*! @brief This module takes in thruster on times and uses them to determine the thrust from the thruster at the current time step.
 */
class ThrFiringRound: public SysModel {
public:
    ThrFiringRound();
    ~ThrFiringRound() = default;

    void SelfInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<THRArrayOnTimeCmdMsgPayload> onTimeInMsg;  //!< total thruster on time input message

    Message<THRArrayCmdForceMsgPayload> thrForceOutMsg;  //!< thruster force C++ output msg
    THRArrayCmdForceMsg_C               thrForceOutMsgC = {};  //!< thruster force C output msg

    BSKLogger bskLogger;              //!< BSK Logging

    /** setter for `thrForce` property */
    void setTHRForce(std::vector<double>);
    /** getter for `thrForce` property */
    std::vector<double> getTHRForce() const {return this->thrForce;}
    /** setter for `timeStep` property */
    void setTimeStep(double timeStep);
    /** getter for `timeStep` property */
    double getTimeStep() const { return this->timeStep;}

private:
    double timeStep;                //!< [s] simulation time step
    std::vector<double> thrForce;  //!< [N] thruster force array

    uint32_t stepsToFire[MAX_EFF_CNT] = {0};  //!< counter for number of steps to fire each thruster
    uint32_t stepsFired[MAX_EFF_CNT] = {0};   //!< counter for number of steps taken

    uint64_t lastWritten = std::numeric_limits<uint64_t>::max();               //!< previous time the message was written was called

};


#endif
