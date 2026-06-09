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

#include "earthRadiationModel.h"

#include <cmath>

void EarthRadiationModel::onUpdateBegin(const SCStatesMsgPayload&         /*scMsg*/,
                                        const SpicePlanetStateMsgPayload& /*sunMsg*/,
                                        uint64_t                          nanos)
{
    this->albedoFlux     = 0.0;   // [W/m^2]
    this->irFlux         = 0.0;   // [W/m^2]
    this->currentNanos   = nanos; // [ns]
    std::fill(std::begin(this->albedoVec_N), std::end(this->albedoVec_N), 0.0);
    std::fill(std::begin(this->irVec_N), std::end(this->irVec_N), 0.0);
}

void EarthRadiationModel::evaluatePlanet(const SCStatesMsgPayload&         scMsg,
                                         const SpicePlanetStateMsgPayload& sunMsg,
                                         SpicePlanetStateMsgPayload        planetMsg,
                                         PlanetEntry&                      entry,
                                         const double                      S_sun,
                                         uint64_t                          /*nanos*/)
{
    std::vector<PatchResult> patches = entry.grid.computePatches(
        scMsg.r_BN_N,
        sunMsg.PositionVector,
        planetMsg.PositionVector,
        planetMsg.J20002Pfix,
        entry.R_planet,
        S_sun);

    for (const PatchResult& p : patches) {
        // Apply eclipse check: if patch is eclipsed, skip albedo contribution
        const double illum = this->getEclipseCase()
                ? this->computeIlluminationAtdA(this->currentPlanetRadius,
                                                Eigen::Vector3d(p.r_dAP_N),
                                                this->currentR_SP_N)
                : 1.0;  // [-]
        this->albedoFlux += p.albedoFlux * illum;  // [W/m^2]
        for (int i = 0; i < 3; ++i) {
            this->albedoVec_N[i] += p.albedoFlux * illum * p.rHat_IdA_N[i];  // [W/m^2]
        }
        // IR flux is always added (emitted by patch regardless of eclipse)
        this->irFlux += p.irFlux;  // [W/m^2]
        for (int i = 0; i < 3; ++i) {
            this->irVec_N[i] += p.irFlux * p.rHat_IdA_N[i];  // [W/m^2]
        }
    }
}

void EarthRadiationModel::onUpdateEnd(uint64_t nanos)
{
    // Normalise direction vectors; zero if no flux
    double albedoDir_N[3] = {0.0, 0.0, 0.0};  // [-]
    if (this->albedoFlux > 0.0) {
        const double mag = std::hypot(this->albedoVec_N[0], this->albedoVec_N[1], this->albedoVec_N[2]);  // [W/m^2]
        if (mag > 0.0) {
            albedoDir_N[0] = this->albedoVec_N[0] / mag;  // [-]
            albedoDir_N[1] = this->albedoVec_N[1] / mag;  // [-]
            albedoDir_N[2] = this->albedoVec_N[2] / mag;  // [-]
        }
    }

    double irDir_N[3] = {0.0, 0.0, 0.0};  // [-]
    if (this->irFlux > 0.0) {
        const double mag = std::hypot(this->irVec_N[0], this->irVec_N[1], this->irVec_N[2]);  // [W/m^2]
        if (mag > 0.0) {
            irDir_N[0] = this->irVec_N[0] / mag;  // [-]
            irDir_N[1] = this->irVec_N[1] / mag;  // [-]
            irDir_N[2] = this->irVec_N[2] / mag;  // [-]
        }
    }

    EarthRadiationMsgPayload outMsg;
    std::memset(&outMsg, 0x0, sizeof(outMsg));
    outMsg.albedoFlux = this->albedoFlux;  // [W/m^2]
    outMsg.irFlux     = this->irFlux;  // [W/m^2]
    for (int i = 0; i < 3; ++i) {
        outMsg.albedoDir_N[i] = albedoDir_N[i];  // [-]
        outMsg.irDir_N[i]     = irDir_N[i];  // [-]
    }
    outMsg.timeTag = nanos;  // [ns]
    this->earthRadiationOutMsg.write(&outMsg, this->moduleID, nanos);
}
