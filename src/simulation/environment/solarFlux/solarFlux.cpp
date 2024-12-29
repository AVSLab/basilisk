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

#include "solarFlux.h"
#include "architecture/utilities/astroConstants.h"


/*! This method is used to reset the module. Currently no tasks are required.

 */
void SolarFlux::Reset(uint64_t CurrentSimNanos)
{
    // check if input message has not been included
    if (!this->sunPositionInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "solarFlux.sunPositionInMsg was not linked.");
    }
    if (!this->spacecraftStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "solarFlux.spacecraftStateInMsg was not linked.");
    }

    return;
}

/*! Read Messages and scale the solar flux then write it out

 */
void SolarFlux::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();

    /*! - evaluate spacecraft position relative to the sun in N frame components */
    auto r_SSc_N = this->r_SN_N - this->r_ScN_N;

    /*! - compute the scalar distance to the sun.  The following math requires this to be in km. */
    double dist_SSc_N = r_SSc_N.norm() / 1000;  // to km

    /*! - compute the local solar flux value */
    this->fluxAtSpacecraft = SOLAR_FLUX_EARTH * pow(AU, 2) / pow(dist_SSc_N, 2) * this->eclipseFactor;

    this->writeMessages(CurrentSimNanos);
}

/*! This method is used to  read messages and save values to member attributes

 */
void SolarFlux::readMessages()
{
    /*! - read in planet state message (required) */
    SpicePlanetStateMsgPayload sunPositionMsgData;
    sunPositionMsgData = this->sunPositionInMsg();
    this->r_SN_N = Eigen::Vector3d(sunPositionMsgData.PositionVector);

    /*! - read in spacecraft state message (required) */
    SCStatesMsgPayload scStatesMsgData;
    scStatesMsgData = this->spacecraftStateInMsg();
    this->r_ScN_N = Eigen::Vector3d(scStatesMsgData.r_BN_N);

    /*! - read in eclipse message (optional) */
    if (this->eclipseInMsg.isLinked()) {
        EclipseMsgPayload eclipseInMsgData;
        eclipseInMsgData = this->eclipseInMsg();
        this->eclipseFactor = eclipseInMsgData.shadowFactor;
    }

}

/*! This method is used to write the output flux message

 */
void SolarFlux::writeMessages(uint64_t CurrentSimNanos) {
    SolarFluxMsgPayload fluxMsgOutData = {this->fluxAtSpacecraft};
    this->solarFluxOutMsg.write(&fluxMsgOutData, this->moduleID, CurrentSimNanos);
}
