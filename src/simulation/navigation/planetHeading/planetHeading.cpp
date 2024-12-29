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

#include "planetHeading.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! Customer constructor just sets the spacecraftSTateInMsg by default*/
PlanetHeading::PlanetHeading()
{

}


/*! This method reads messages, calculates the planet heading, and writes out the heading message

 */
void PlanetHeading::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();

    /*! - evaluate planet position relative to the s/c body in body frame components */
    auto r_PB_N = this->r_PN_N - this->r_BN_N;
    /*! - normalize and convert to body frame */
    this->rHat_PB_B = (this->sigma_BN.toRotationMatrix().transpose() * r_PB_N).normalized();

    this->writeMessages(CurrentSimNanos);
}

/*! Read input messages and save data to member variables

 */
void PlanetHeading::readMessages() {
    SpicePlanetStateMsgPayload planetPositionMsgData;
    /*! - read in planet state message (required) */
    planetPositionMsgData = this->planetPositionInMsg();
    this->r_PN_N = Eigen::Vector3d(planetPositionMsgData.PositionVector);

    SCStatesMsgPayload scStatesMsgData;
    /*! - read in spacecraft state message (required) */
    scStatesMsgData = this->spacecraftStateInMsg();
    this->r_BN_N = Eigen::Vector3d(scStatesMsgData.r_BN_N);
    this->sigma_BN = Eigen::MRPd(scStatesMsgData.sigma_BN);
}

/*! This method is used to write out the planet heading message

 */
void PlanetHeading::writeMessages(uint64_t CurrentSimNanos) {
    BodyHeadingMsgPayload planetHeadingOutMsgData;
    planetHeadingOutMsgData = this->planetHeadingOutMsg.zeroMsgPayload;
    eigenVector3d2CArray(this->rHat_PB_B, planetHeadingOutMsgData.rHat_XB_B);

    /*! - write the output message */
    this->planetHeadingOutMsg.write(&planetHeadingOutMsgData, this->moduleID, CurrentSimNanos);
}

/*! This method is used to reset the module. Currently no tasks are required.

 */
void PlanetHeading::Reset(uint64_t CurrentSimNanos)
{

    // check if input message has not been included
    if (!this->planetPositionInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "planetHeading.planetPositionInMsg was not linked.");
    }
    if (!this->spacecraftStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "planetHeading.spacecraftStateInMsg was not linked.");
    }

    return;
}
