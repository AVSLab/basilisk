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


#ifndef DENTONFLUXMODEL_H
#define DENTONFLUXMODEL_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
//#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
//#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/FluxMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

/*! @brief This is an auto-created sample C++ module.  The description is included with the module class definition
 */
class DentonFluxModel: public SysModel {
public:
    DentonFluxModel(int kp, double energy);
    ~DentonFluxModel();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    
    // Other Methods
    double calcLocalTime(double v1[3], double v2[3]);
    long double bilinear(int, int, long double, long double, double, long double, long double, long double, long double, long double);
    
public:
    
    int choose_kp;
    double choose_energy;
    double choose_local_time;
    
    ReadFunctor<SCStatesMsgPayload> satStateInMsg;  //!< input spacecraft states
    //ReadFunctor<EphemerisMsgPayload> earthStateInMsg;  //!< input Earth states
    //ReadFunctor<EphemerisMsgPayload> sunStateInMsg;  //!< input sun states

    //ReadFunctor<SpicePlanetStateMsgPayload> earthStateInputMsg;  //!< input planet states
    //ReadFunctor<SpicePlanetStateMsgPayload> sunStateInputMsg;  //!< input sun states

    Message<FluxMsgPayload> fluxOutMsg;  //!< output ion and electron fluxes

    BSKLogger bskLogger;              //!< -- BSK Logging

};


#endif
