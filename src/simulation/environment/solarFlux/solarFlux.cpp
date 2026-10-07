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
#include "architecture/utilities/stateExtrapolation.h"

/*! This method is used to reset the module. Currently no tasks are required.

 */
void SolarFlux::Reset(uint64_t CurrentSimNanos)
{
    this->previousUpdateNanos = CurrentSimNanos;
    this->scStateExtrapolation.reset();

    // check if input message has not been included
    if (!this->sunPositionInMsg.isLinked()) {
        bskLogger.bskError("solarFlux.sunPositionInMsg was not linked.");
    }
    if (!this->spacecraftStateInMsg.isLinked()) {
        bskLogger.bskError("solarFlux.spacecraftStateInMsg was not linked.");
    }

    return;
}

/*! Read Messages and scale the solar flux then write it out

 */
void SolarFlux::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages(CurrentSimNanos);

    /*! - evaluate spacecraft position relative to the sun in N frame components */
    auto r_SSc_N = this->r_SN_N - this->r_ScN_N;

    /*! - compute the scalar distance to the sun.  The following math requires this to be in km. */
    double dist_SSc_N = r_SSc_N.norm() / 1000;  // to km

    /*! - compute the local solar flux value */
    this->fluxAtSpacecraft = SOLAR_FLUX_EARTH * pow(AU, 2) / pow(dist_SSc_N, 2) * this->eclipseFactor;

    this->writeMessages(CurrentSimNanos);
    this->previousUpdateNanos = CurrentSimNanos;
}

/*! Enables or disables the extrapolation of the spacecraft state to the middle of the interval the next spacecraft
 update integrates, see extrapolateScStateToStepMidpoint(). It is disabled by default, in which case the spacecraft
 state message is used as written. The extrapolation assumes that the module and the spacecraft run at the same task
 rate with a constant spacecraft step; a warning is logged once if a spacecraft message was not written at the
 previous module update, which is typically a task rate mismatch. A mismatch is not always detectable, but a message
 that is not the output of the previous module update is never extrapolated.
 @param enable [-] true to extrapolate the spacecraft state to the middle of the step
 */
void SolarFlux::setExtrapolateScStateToStepMidpoint(bool enable)
{
    this->scStateExtrapolation.setEnabled(enable);
}

/*! Returns whether the spacecraft state extrapolation is enabled.
 @return [-] true if the spacecraft state is extrapolated
 */
bool SolarFlux::getExtrapolateScStateToStepMidpoint() const
{
    return this->scStateExtrapolation.isEnabled();
}

/*! This method is used to  read messages and save values to member attributes. If enabled with
 setExtrapolateScStateToStepMidpoint(), the spacecraft position is extrapolated to the middle of the interval the
 next spacecraft update integrates, see extrapolateScStateToStepMidpoint().
 @param CurrentSimNanos [ns] current simulation time

 */
void SolarFlux::readMessages(uint64_t CurrentSimNanos)
{
    /*! - read in spacecraft state message (required) */
    SCStatesMsgPayload scStatesMsgData;
    this->scStateExtrapolation.prepare(
      CurrentSimNanos, this->previousUpdateNanos, { this->spacecraftStateInMsg.timeWritten() }, this->bskLogger);
    scStatesMsgData = this->scStateExtrapolation.apply(this->spacecraftStateInMsg(),
                                                       CurrentSimNanos,
                                                       this->spacecraftStateInMsg.timeWritten(),
                                                       this->previousUpdateNanos);
    this->r_ScN_N = Eigen::Vector3d(scStatesMsgData.r_BN_N);

    /*! - read in planet state message (required), evaluated at the same epoch as the spacecraft state */
    SpicePlanetStateMsgPayload sunPositionMsgData;
    sunPositionMsgData = this->scStateExtrapolation.applyPlanet(this->sunPositionInMsg(),
                                                                CurrentSimNanos,
                                                                this->sunPositionInMsg.timeWritten(),
                                                                this->previousUpdateNanos);
    this->r_SN_N = Eigen::Vector3d(sunPositionMsgData.PositionVector);

    /*! - read in eclipse message (optional) */
    if (this->eclipseInMsg.isLinked()) {
        EclipseMsgPayload eclipseInMsgData;
        eclipseInMsgData = this->eclipseInMsg();
        this->eclipseFactor = eclipseInMsgData.illuminationFactor;
    }

}

/*! This method is used to write the output flux message

 */
void SolarFlux::writeMessages(uint64_t CurrentSimNanos) {
    SolarFluxMsgPayload fluxMsgOutData = {this->fluxAtSpacecraft};
    this->solarFluxOutMsg.write(&fluxMsgOutData, this->moduleID, CurrentSimNanos);
}
