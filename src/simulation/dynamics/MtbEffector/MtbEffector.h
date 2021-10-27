/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef MTBEFFECTOR_H
#define MTBEFFECTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/msgPayloadDefC/MTBCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/MagneticFieldMsgPayload.h"
#include "architecture/msgPayloadDefC/MTBArrayConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/MTBMsgPayload.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
    
/*! @brief This module converts magnetic torque bar dipoles to body torques.
 */
class MtbEffector: public SysModel, public DynamicEffector {
    
public:
    MtbEffector();
    ~MtbEffector();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime, double timeStep);
    void WriteOutputMessages(uint64_t CurrentClock);
    
public:
    Message<MTBMsgPayload> mtbOutMsg;                       //!< output message containing net torque produced by the torque bars in Body components
    StateData *hubSigma;                                    //!< Hub/Inertial attitude represented by MRP
    ReadFunctor<MTBCmdMsgPayload> mtbCmdInMsg;              //!< input msg for commanded mtb dipole array in the magnetic torque bar frame T
    ReadFunctor<MagneticFieldMsgPayload> magInMsg;          //!< input msg for magnetic field data in inertial frame N
    ReadFunctor<MTBArrayConfigMsgPayload> mtbParamsInMsg;   //!< input msg for layout of magnetic torque bars
    BSKLogger bskLogger;                                    //!< -- BSK Logging
    
private:
    MTBCmdMsgPayload mtbCmdInMsgBuffer;         //!< msg buffer or commanded mtb dipole array in the magnetic torque bar frame T
    MagneticFieldMsgPayload magInMsgBuffer;     //!< msg buffer for magnetic field data in inertial frame N
    MTBArrayConfigMsgPayload mtbConfigParams;   //!< msg for layout of magnetic torque bars
};


#endif
