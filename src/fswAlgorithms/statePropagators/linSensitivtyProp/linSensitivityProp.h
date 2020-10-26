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

#ifndef _LIN_SENS_PROP_H
#define _LIN_SENS_PROP_H

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
class LinSensProp: public SysModel {
public:
    LinSensProp();
    ~LinSensProp();

    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);

    void WriteMessages(uint64_t CurrentSimNanos);
    void ReadMessages(uint64_t CurrentSimNanos);
    
public:
    std::string depAttInMsgName;                //!< Message name for the hill-frame relative position message.
    std::string chiefAttInMsgName; //!< Message name for the chief
    std::string hillStateInMsgName;                //!< Message name for the target spacecraft's attitude nav message.
    std::string sensOutMsgName;  //!< Attitude reference message generated after the control law is applied.
    Eigen::MatrixXd A; //!< State dynamics matrix
    Eigen::MatrixXd D; //!< Control Effects Matrix
    Eigen::MatrixXd C; //!< State sensitivity matrix

    BSKLogger bskLogger;                //!< -- BSK Logging

private:
    uint64_t OutputBufferCount;          //!< [-] Count on the number of output message buffers

    int32_t depAttInMsgName;        //!< ID for the outgoing message
    int32_t chiefAttInMsgName;        //!< ID for the outgoing message
    int32_t hillStateInMsgId;
    int32_t sensOutMsgId;                //!< ID for the outgoing message

    Eigen::Vector6d sensitivityState;
    HillRelStateFswMsg hillStateInMsg;
    NavAttIntMsg depAttInMsg;
    NavAttIntMsg chiefAttInMsg;
    HillRelStateFswMsg sensOutMsg;

};

#endif

