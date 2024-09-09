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

#ifndef ORB_ELEM_CONVERT_H
#define ORB_ELEM_CONVERT_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/ClassicElementsMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/bskLogging.h"



/*! @brief orbit element converter module class */
class OrbElemConvert: public SysModel {
public:
    OrbElemConvert();
    ~OrbElemConvert();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    void Elements2Cartesian();
    void Cartesian2Elements();
    void ReadInputs();

public:
    double r_N[3];                    //!< m  Current position vector (inertial)
    double v_N[3];                    //!< m/s Current velocity vector (inertial)
    double mu;                        //!< -- Current grav param (inertial)
    ClassicElements CurrentElem;                      //!< -- Current orbital elements
    SCStatesMsgPayload statesIn;                            //!< -- spacecraft state message
    SpicePlanetStateMsgPayload planetIn;                        //!< -- planet state message
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;           //!< -- sc state input message
    ReadFunctor<SpicePlanetStateMsgPayload> spiceStateInMsg;    //!< -- spice state input message
    ReadFunctor<ClassicElementsMsgPayload> elemInMsg;           //!< -- orbit element input message
    Message<SCStatesMsgPayload> scStateOutMsg;              //!< -- sc state output message
    Message<SpicePlanetStateMsgPayload> spiceStateOutMsg;       //!< -- spice state input message
    Message<ClassicElementsMsgPayload> elemOutMsg;              //!< -- orbit element output message

private:
    bool inputsGood;                  //!< -- flag indicating that inputs are good
    BSKLogger bskLogger;              //!< -- BSK Logging

};


#endif
