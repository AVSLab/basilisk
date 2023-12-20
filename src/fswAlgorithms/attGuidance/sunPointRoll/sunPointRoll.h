/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _SUN_POINT_ROLL_
#define _SUN_POINT_ROLL_

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "cMsgCInterface/AttRefMsg_C.h"


/*! @brief A class to perform EMA SEP pointing */
class SunPointRoll: public SysModel {
public:
    SunPointRoll();
    ~SunPointRoll();
    void SelfInit();                                               //!< Self initialization for C-wrapped messages
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    double            hRefHat_B[3];                                //!< Sun-pointing direction in B-frame coordinates
    double            a1Hat_B[3];                                  //!< solar array drive axis in B-frame coordinates
    double            omegaRoll = 0;                               //!< roll rate about Sun direction [rad/s]

    ReadFunctor<NavAttMsgPayload>          attNavInMsg;            //!< input msg measured attitude
    Message<AttRefMsgPayload>              attRefOutMsg;           //!< Attitude reference output message
    AttRefMsg_C                            attRefOutMsgC = {};     //!< C-wrapped attitude reference output message

private:
    uint64_t          resetTime;                                   //!< callTime one update step prior
    double            sigma_RN_1[3];                               //!< reference attitude one update step prior
    double            sigma_RN_2[3];                               //!< reference attitude two update steps prior
    BSKLogger                              bskLogger;              //!< BSK Logging
};

#endif
