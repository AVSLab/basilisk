/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _MRP_PD_
#define _MRP_PD_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include <stdint.h>

/*! @brief MRP PD control class. */
class MrpProportionalDerivative: public SysModel {
public:

    MrpProportionalDerivative() = default;                            //!< Constructor
    ~MrpProportionalDerivative() = default;                           //!< Destructor

    void Reset(uint64_t CurrentSimNanos) override;                    //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;              //!< Update member function

    /* Declare public module variables */
    double K;                                                         //!< [rad/s] Proportional gain applied to MRP errors
    double P;                                                         //!< [N*m*s] Rate error feedback gain applied
    double knownTorquePntB_B[3];                                      //!< [N*m] Known external torque expressed in body frame components
    
    /* Declare private module variables */
    double ISCPntB_B[9];                                              //!< [kg*m^2] Spacecraft inertia about point B expressed in body frame components

    ReadFunctor<AttGuidMsgPayload> guidInMsg;                         //!< Attitude guidance input message
    ReadFunctor<VehicleConfigMsgPayload> vehConfigInMsg;              //!< Vehicle configuration input message
    Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;                 //!< Commanded torque output message
    
    BSKLogger *bskLogger;                                             //!< BSK Logging
};

#endif
