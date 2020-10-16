/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _HILL_TO_ATT_H
#define _HILL_TO_ATT_H

#include <stdint.h>
#include <Eigen/Dense>
#include <string.h>
#include "architecture/messaging/system_messaging.h"
#include "../simulation/_GeneralModuleFiles/sys_model.h"
#include "../simulation/utilities/avsEigenMRP.h"
#include "../simulation/utilities/bskLogging.h"
#include "fswMessages/hillRelStateFswMsg.h"
#include "fswMessages/attRefFswMsg.h"
#include "../simulation/simFswInterfaceMessages/navAttIntMsg.h"


/*! @brief visual planet tracking with Hough circles */
class HillToAttRef: public SysModel {
public:
    HillToAttRef();
    ~HillToAttRef();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    
public:
    std::string hillStateInMsgName;                //!< Message name for the hill-frame relative position message.
    std::string attStateInMsgName;                //!< Message name for the target spacecraft's attitude nav message.
    std::string attRefOutMsgName;  //!< Attitude reference message generated after the control law is applied.
    std::vector<std::vector<std::vector<double>>> gainMatrixVec; //!< Arbitrary dimension gain matrix, stored as a vector (varible length) of double,6 arrays
    BSKLogger bskLogger;                //!< -- BSK Logging

private:
    uint64_t OutputBufferCount;          //!< [-] Count on the number of output message buffers

    std::vector<std::vector<std::vector<double>>>::iterator matrixIndex;

    int32_t hillStateInMsgId;        //!< ID for the outgoing message
    int32_t attStateInMsgId;
    int32_t attRefOutMsgId;                //!< ID for the outgoing message

    HillRelStateFswMsg hillStateInMsg;
    NavAttIntMsg attStateInMsg;
    AttRefFswMsg attRefOutMsg;

};

#endif

