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


#include "fswAlgorithms/smallBodyNavigation//smallBodyNavEKF/smallBodyNavEKF.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include <iostream>
#include <cstring>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
SmallBodyNavEKF::SmallBodyNavEKF()
{
    this->numStates = 18;
}

/*! Module Destructor */
SmallBodyNavEKF::~SmallBodyNavEKF()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.
    @return void
*/
void SmallBodyNavEKF::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->navTransInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.navTransInMsg was not linked.");
    }
    if (!this->navAttInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.navAttInMsg was not linked.");
    }
    if (!this->asteroidEphemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.asteroidEphemerisInMsg was not linked.");
    }
    if (!this->sunEphemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.sunEphemerisInMsg was not linked.");
    }
    if (!this->rwSpeedInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.rwSpeedInMsg was not linked.");
    }

}

void SmallBodyNavEKF::readMessages(){
    NavTransMsgPayload navTransInMsgBuffer;  //!< local copy of message buffer
    NavAttMsgPayload navAttInMsgBuffer;  //!< local copy of message buffer
    EphemerisMsgPayload asteroidEphemerisInMsgBuffer;  //!< local copy of message buffer
    EphemerisMsgPayload sunEphemerisInMsgBuffer;  //!< local copy of message buffer
    RWSpeedMsgPayload rwSpeedInMsgBuffer;  //!< local copy of message buffer
    NavTransMsgPayload navTransOutMsgBuffer;  //!< local copy of message buffer
    NavAttMsgPayload navAttOutMsgBuffer;  //!< local copy of message buffer
    SmallBodyNavMsgPayload smallBodyNavOutMsgBuffer;  //!< local copy of message buffer
    EphemerisMsgPayload asteroidEphemerisOutMsgBuffer;  //!< local copy of message buffer

    // read in the input messages
    navTransInMsgBuffer = this->navTransInMsg();
    navAttInMsgBuffer = this->navAttInMsg();
    asteroidEphemerisInMsgBuffer = this->asteroidEphemerisInMsg();
    sunEphemerisInMsgBuffer = this->sunEphemerisInMsg();
    rwSpeedInMsgBuffer = this->rwSpeedInMsg();
}

void SmallBodyEKF::predict(){
    


}

void SmallBodyEKF::computeAMatrix(){

}

void SmallBodyEkf::measurementUpdate(){

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
*/
void SmallBodyNavEKF::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();
    this->predict();
    this->measurementUpdate();
    this->writeMessages();
    this->prevTime = currentSimNanos;
}


void SmallBodyNavEKF::writeMessages(){
    // always zero the output message buffers before assigning values
    navTransOutMsgBuffer = this->navTransOutMsg.zeroMsgPayload;
    navAttOutMsgBuffer = this->navAttOutMsg.zeroMsgPayload;
    smallBodyNavOutMsgBuffer = this->smallBodyNavOutMsg.zeroMsgPayload;
    asteroidEphemerisOutMsgBuffer = this->asteroidEphemerisOutMsg.zeroMsgPayload;

    // write to the output messages
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->navAttOutMsg.write(&navAttOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->smallBodyNavOutMsg.write(&smallBodyNavOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->asteroidEphemerisOutMsg.write(&asteroidEphemerisOutMsgBuffer, this->moduleID, CurrentSimNanos);
}

