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
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/PlasmaFluxMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

/*! @brief This module provides the 10-year averaged GEO elecon and ion flux as discussed in the paper by Denton.
 */
class DentonFluxModel: public SysModel {
public:
    
    // Constructor And Destructor
    DentonFluxModel();
    ~DentonFluxModel();

    // Methods
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    
    /* public variables */
    int numEnergies = -1;          //!< number of energy bins used in the output message
    int kpIndex = -1;              //!< Kp index
    
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;  //!<  spacecraft state input message
    ReadFunctor<SpicePlanetStateMsgPayload> earthStateInMsg;  //!< Earth planet state input message
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg;  //!< sun state input message

    Message<PlasmaFluxMsgPayload> fluxOutMsg;  //!< output message with ion and electron fluxes

    BSKLogger bskLogger;              //!< -- BSK Logging

private:

    double chooseEnergy;
    double localTime;           //!< [??] local time in the GEO belt

    void calcLocalTime(double v1[3], double v2[3]);     //!< calculate the local time
    double bilinear(int, int, double, double, double, double, double, double, double);

};


#endif
