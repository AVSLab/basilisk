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

#ifndef BORE_ANG_CALC_H
#define BORE_ANG_CALC_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/BoreAngleMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"


/*! @brief A class to perform a range of boresight related calculations.
 */
class BoreAngCalc: public SysModel {
public:
    BoreAngCalc();
    ~BoreAngCalc();
    
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void computeAxisPoint();
    void computeOutputData();
    void WriteOutputMessages(uint64_t CurrentClock);
    void ReadInputs();
    
public:
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;       //!< (-) spacecraft state input message
    ReadFunctor<SpicePlanetStateMsgPayload> celBodyInMsg;   //!< (-) celestial body state msg at which we pointing at
    Message<BoreAngleMsgPayload> angOutMsg;                 //!< (-) bore sight output message

    double boreVec_B[3];              //!< (-) boresight vector in structure
    double boreVecPoint[3];           //!< (-) pointing vector in the target relative point frame
    BoreAngleMsgPayload boresightAng; //!< (-) Boresight angles relative to target
    bool inputsGood;                  //!< (-) Flag indicating that inputs were read correctly
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    SpicePlanetStateMsgPayload localPlanet;//!< (-) planet that we are pointing at
    SCStatesMsgPayload localState;   //!< (-) observed state of the spacecraft
};


#endif
