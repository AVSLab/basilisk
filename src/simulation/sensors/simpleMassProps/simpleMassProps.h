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

#ifndef SIMPLE_MASS_PROPS_H
#define SIMPLE_MASS_PROPS_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

/*! @brief FSW mass properties converter module class */
class SimpleMassProps: public SysModel {
public:
    SimpleMassProps();
    ~SimpleMassProps();

    void Reset(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t CurrentClock);
    void computeMassProperties();
    void UpdateState(uint64_t CurrentSimNanos);


public:

    ReadFunctor<SCMassPropsMsgPayload> scMassPropsInMsg;        //!< sc mass properties input msg
    Message<VehicleConfigMsgPayload> vehicleConfigOutMsg;       //!< vehicle configuration output msg
    

    BSKLogger bskLogger;              //!< -- BSK Logging

private:

    SCMassPropsMsgPayload scMassPropsMsgBuffer;         //! buffer for the mass properties message
    VehicleConfigMsgPayload vehicleConfigMsgBuffer;     //! buffer for the vehicle configuration message


};


#endif
