/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

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

#ifndef EARTH_RADIATION_MODEL_H
#define EARTH_RADIATION_MODEL_H

#include "architecture/msgPayloadDefC/EarthRadiationMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "simulation/environment/_GeneralModuleFiles/planetRadiationBase.h"

/*! @brief Earth Radiation Pressure environment module.
 *
 * Computes albedo and IR radiation fluxes at the
 * spacecraft position due to Earth emission. Inherits PlanetRadiationBase
 * for grid management and message plumbing.
 *
 * The module is a sub-class of `PlanetRadiationBase`. See that class for the
 * nominal messages used and setup instructions.
 */
class EarthRadiationModel : public PlanetRadiationBase {
public:
    EarthRadiationModel()  = default;
    ~EarthRadiationModel() = default;

    Message<EarthRadiationMsgPayload> earthRadiationOutMsg; //!< total flux + direction vectors

protected:
    /*! Clears per-timestep accumulators before the planet loop.
     *  @param scMsg   Spacecraft state message (unused).
     *  @param sunMsg  Sun state message (unused).
     *  @param nanos   Current simulation time (ns).
     */
    void onUpdateBegin(const SCStatesMsgPayload&         scMsg,
                       const SpicePlanetStateMsgPayload& sunMsg,
                       uint64_t                          nanos) override;

    /*! Calls entry.grid.computePatches() and accumulates flux-weighted directions.
     *  @param scMsg      Spacecraft state message.
     *  @param sunMsg     Sun state message.
     *  @param planetMsg  Planet state message.
     *  @param entry      Planet entry with initialised grid and authalic radius.
     *  @param S_sun      [W/m^2] distance-corrected solar flux at planet.
     *  @param nanos      Current simulation time (ns).
     */
    void evaluatePlanet(const SCStatesMsgPayload&         scMsg,
                        const SpicePlanetStateMsgPayload& sunMsg,
                        SpicePlanetStateMsgPayload        planetMsg,
                        PlanetEntry&                      entry,
                        const double                      S_sun,
                        uint64_t                          nanos) override;

    /*! EarthRadiationModel uses planetInMsg directly and leaves planets empty
     *  until Reset(); opt in to the single-planet backward-compat fallback.
     */
    bool allowSinglePlanetFallback() const override { return true; }

    /*! Normalises direction vectors and writes earthRadiationOutMsg.
     *  @param nanos  Current simulation time (ns).
     */
    void onUpdateEnd(uint64_t nanos) override;

private:
    double albedoFlux     = 0.0;              //!< [W/m^2] accumulated albedo flux
    double irFlux         = 0.0;              //!< [W/m^2] accumulated IR flux
    double albedoVec_N[3] = {0.0, 0.0, 0.0};  //!< [-] flux-weighted albedo direction, inertial N
    double irVec_N[3]     = {0.0, 0.0, 0.0};  //!< [-] flux-weighted IR direction, inertial N
    uint64_t currentNanos = 0;                //!< [ns] current simulation time
};

#endif /* EARTH_RADIATION_MODEL_H */
