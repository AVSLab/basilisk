/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef ENCODER_H
#define ENCODER_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/simDefinitions.h"
#include "architecture/messaging/messaging.h"

/*! @brief wheel speed encoder module class */
class Encoder: public SysModel {
public:
    Encoder();
    ~Encoder();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentClock);
    void encode(uint64_t CurrentSimNanos);

public:
    Message<RWSpeedMsgPayload> rwSpeedOutMsg;       //!< [rad/s] reaction wheel speed output message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedInMsg;    //!< [rad/s] reaction wheel speed input message
    int rwSignalState[MAX_EFF_CNT];                 //!< vector of reaction wheel signal states
    int clicksPerRotation;                          //!< number of clicks per full rotation
    int numRW;                                      //!< number of reaction wheels
    BSKLogger bskLogger;                            //!< -- BSK Logging

private:
    RWSpeedMsgPayload rwSpeedBuffer;        //!< reaction wheel speed buffer for internal calculations
    RWSpeedMsgPayload rwSpeedConverted;     //!< reaction wheel speed buffer for converted values
    double remainingClicks[MAX_EFF_CNT];    //!< remaining clicks from the previous iteration

    uint64_t prevTime;                      //!< -- Previous simulation time observed

};


#endif
