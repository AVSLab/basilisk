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
#include <Eigen/Dense>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/BoreAngleMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"


/*! @brief A class to perform a range of boresight related calculations.
 */
class BoreAngCalc: public SysModel {
public:
    BoreAngCalc();
    ~BoreAngCalc();
    
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void computeCelestialAxisPoint();
    void computeCelestialOutputData();
    void computeInertialOutputData();
    void WriteOutputMessages(uint64_t CurrentClock);
    void ReadInputs();
    
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;           //!< (-) spacecraft state input message
    ReadFunctor<SpicePlanetStateMsgPayload> celBodyInMsg;   //!< (-) celestial body state msg at which we pointing at
    Message<BoreAngleMsgPayload> angOutMsg;                 //!< (-) bore sight output message

    Eigen::Vector3d boreVec_B;              //!< (-) boresight vector in structure
    Eigen::Vector3d boreVec_Po;             //!< (-) pointing vector in the target relative point frame
    Eigen::Vector3d inertialHeadingVec_N;   //!< (-) inertial boresight vector

private:
    SpicePlanetStateMsgPayload localPlanet; //!< (-) planet that we are pointing at
    SCStatesMsgPayload localState;          //!< (-) observed state of the spacecraft

    BoreAngleMsgPayload boresightAng = {};  //!< (-) Boresight angles relative to target
    bool inputsGood = false;                //!< (-) Flag indicating that inputs were read correctly
    bool useCelestialHeading = false;       //!< (-) Flag indicating that the module should use the celestial body heading
    bool useInertialHeading = false;        //!< (-) Flag indicating that the module should use the inertial heading
    BSKLogger bskLogger;                    //!< -- BSK Logging
};


#endif
