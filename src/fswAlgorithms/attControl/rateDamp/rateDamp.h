/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _RATE_DAMP_
#define _RATE_DAMP_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceBodyMsgPayload.h"
#include "cMsgCInterface/CmdTorqueBodyMsg_C.h"


/*! @brief A class to compute rate damping control */
class RateDamp: public SysModel {
public:
    RateDamp();
    ~RateDamp();
    void SelfInit();                                              //!< Self initialization for C-wrapped messages
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    ReadFunctor<NavAttMsgPayload>          attNavInMsg;           //!< input msg measured attitude
    Message<CmdTorqueBodyMsgPayload>       cmdTorqueOutMsg;       //!< commanded torque output message
    CmdTorqueBodyMsg_C                     cmdTorqueOutMsgC = {}; //!< C-wrapped commanded torque output message

    void setRateGain(double const p);
    double getRateGain() const;

private:
    double P;       //!< [N*m*s] Rate feedback gain

};


#endif
