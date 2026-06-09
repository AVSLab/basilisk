/*
 ISC License

 Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "albedo.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/macroDefinitions.h"

#include <cmath>
#include <utility>
#include <vector>

/*! Albedo module constructor */
Albedo::Albedo()
{
    this->albOutMsgs.clear();
    this->r_IB_Bs.clear();
    this->nHat_Bs.clear();
    this->fovs.clear();

    // Default resolution: 180 lat x 360 lon  (1-deg bands)
    this->defaultNumLat = 180;
    this->defaultNumLon = 360;

    this->nHat_B_default        = {1.0, 0.0, 0.0};
    this->fov_default           = 90.0 * D2R;
    this->illuminationFactorAtdA = 1.0;
    this->altitudeRateLimit     = -1.0;
}

/*! Albedo module destructor */
Albedo::~Albedo()
{
    for (Message<AlbedoMsgPayload>* msg : this->albOutMsgs) { delete msg; }
}

/*! Config constructor */
Config::Config() {
    this->fov = -1.0;
    this->nHat_B.fill(0.0);
    this->r_IB_B.fill(0.0);
}

/*! Config destructor */
Config::~Config() {}

/*! Adds the instrument configuration and automatically creates an output message name (overloaded function) */
void Albedo::addInstrumentConfig(instConfig_t configMsg)
{
    Message<AlbedoMsgPayload>* msg = new Message<AlbedoMsgPayload>;
    this->albOutMsgs.push_back(msg);

    // Do a sanity check and push fov back to the vector (if not defined, use the default value.)
    if (configMsg.fov < 0.0) {
        this->fovs.push_back(this->fov_default);
        this->bskLogger.bskLog(BSK_WARNING, "Albedo Module (addInstrumentConfig): For the instrument (%lu)'s half field of view angle (fov), the default value is used.", this->albOutMsgs.size()-1);
    } else {
        this->fovs.push_back(configMsg.fov);
    }

    // Push r_IB_B back to the vector
    this->r_IB_Bs.push_back(configMsg.r_IB_B);  // [m]

    // Do a sanity check and push nHat_B back to the vector (if not defined, use the default value.)
    if (!configMsg.nHat_B.isZero()) {
        this->nHat_Bs.push_back(configMsg.nHat_B / configMsg.nHat_B.norm());
    } else {
        this->nHat_Bs.push_back(this->nHat_B_default);  // [-]
        this->bskLogger.bskLog(BSK_WARNING, "Albedo Module (addInstrumentConfig): For the instrument (%lu)'s unit normal vector (nHat_B), the default vector is used.", this->albOutMsgs.size()-1);
    }
}

/*! Adds the instrument configuration and automatically creates an output message name (overloaded function) */
void Albedo::addInstrumentConfig(double fov, Eigen::Vector3d nHat_B, Eigen::Vector3d r_IB_B)
{
    Message<AlbedoMsgPayload>* msg = new Message<AlbedoMsgPayload>;
    this->albOutMsgs.push_back(msg);

    // Do a sanity check and push fov back to the vector (if not defined, use the default value.)
    if (fov < 0.0) {
        this->fovs.push_back(this->fov_default);
        this->bskLogger.bskLog(BSK_WARNING, "Albedo Module (addInstrumentConfig): Instrument (%lu)'s half field of view angle (fov) cannot be negative, the default value is used instead.", this->albOutMsgs.size()-1);
    } else {
        this->fovs.push_back(fov);
    }

    // Push r_IB_B back to the vector
    this->r_IB_Bs.push_back(r_IB_B);  // [m]

    // Do a sanity check and push nHat_B back to the vector (if not defined, use the default value.)
    if (!nHat_B.isZero()) {
        this->nHat_Bs.push_back(nHat_B / nHat_B.norm());  // [-]
    } else {
        this->nHat_Bs.push_back(this->nHat_B_default);
        this->bskLogger.bskLog(BSK_WARNING, "Albedo Module (addInstrumentConfig): Instrument (%lu)'s unit normal vector (nHat_B) cannot be composed of all zeros, the default vector is used instead.", this->albOutMsgs.size()-1);
    }
}

/*! Sentinel values in the PlanetGrid cfg signal resolvePlanetEntry() to look up
 *  the value from the planet name:
 *    REQ_m = -1   → always filled from planet name
 *    RP_m  = -1   → always filled from planet name
 *    albedoAvg = -1   → fill from planet name (ALBEDO_AVG_IMPLICIT case)
 *    nLat = 0, nLon = 0  → use base-class defaultNumLat/defaultNumLon
 */
/*! This method subscribes to the planet msg and sets the albedo average model (overloaded function) */
void Albedo::addPlanetandAlbedoAverageModel(Message<SpicePlanetStateMsgPayload>* msg)
{
    PlanetGrid cfg;
    cfg.useAlbedoData = false;
    cfg.albedoAvg     = -1.0;   // resolve from planet name in resolvePlanetEntry
    cfg.nLat          = 0;      // use base-class defaultNumLat
    cfg.nLon          = 0;      // use base-class defaultNumLon
    cfg.REQ_m         = -1.0;
    cfg.RP_m          = -1.0;
    this->addPlanetEntry(msg->addSubscriber(), cfg);
}

/*! This method subscribes to the planet msg and sets the albedo average model (overloaded function) */
void Albedo::addPlanetandAlbedoAverageModel(Message<SpicePlanetStateMsgPayload>* msg,
                                             double ALB_avg, int numLat, int numLon)
{
    PlanetGrid cfg;
    cfg.useAlbedoData = false;
    cfg.albedoAvg     = ALB_avg;
    cfg.nLat          = numLat;
    cfg.nLon          = numLon;
    cfg.REQ_m         = -1.0;
    cfg.RP_m          = -1.0;
    this->addPlanetEntry(msg->addSubscriber(), cfg);
}

/*! This method subscribes to the planet msg and sets the albedo data model */
void Albedo::addPlanetandAlbedoDataModel(Message<SpicePlanetStateMsgPayload>* msg,
                                          std::string dataPath, std::string fileName)
{
    PlanetGrid cfg;
    cfg.useAlbedoData  = true;
    cfg.albedoDataPath = dataPath;
    cfg.albedoDataFile = fileName;
    cfg.albedoAvg      = -1.0;   // not used for DATA model, but keep consistent sentinel
    cfg.nLat           = 0;      // use base-class defaultNumLat
    cfg.nLon           = 0;      // use base-class defaultNumLon
    cfg.REQ_m          = -1.0;
    cfg.RP_m           = -1.0;
    this->addPlanetEntry(msg->addSubscriber(), cfg);
}

/*! Fills in planet-dependent grid parameters (REQ_m, RP_m, albedoAvg, nLat, nLon) from the planet name */
void Albedo::resolvePlanetEntry(const SpicePlanetStateMsgPayload& planetMsg,
                                PlanetGrid& grid, int /*idx*/)
{
    const std::string name(planetMsg.PlanetName);

    auto [REQ, RP] = this->lookupPlanetRadius(name);
    grid.REQ_m = REQ;  // [m]
    grid.RP_m  = RP;   // [m]

    if (grid.albedoAvg < 0.0) {
        grid.albedoAvg = this->getAlbedoAverage(name);  // [-]
    }
    if (grid.nLat <= 0) { grid.nLat = this->defaultNumLat; }
    if (grid.nLon <= 0) { grid.nLon = this->defaultNumLon; }
}

/*! This method resets the module; validates that instruments and planets have been configured */
void Albedo::customReset(uint64_t /*nanos*/)
{
    if (this->planets.empty()) {
        this->bskLogger.bskError(
            "Albedo (Reset): no planets registered. "
            "Call addPlanetandAlbedoAverageModel() or addPlanetandAlbedoDataModel().");
    }
    if (this->albOutMsgs.empty()) {
        this->bskLogger.bskError(
            "Albedo (Reset): no instruments configured. "
            "Call addInstrumentConfig().");
    }
    // Size per-instrument accumulators; values are cleared each timestep in onUpdateBegin.
    const size_t nInst = this->albOutMsgs.size();
    this->albedoAtInstrumentMax.assign(nInst, 0.0);
    this->albedoAtInstrument.assign(nInst, 0.0);
    this->SfluxAtInstrument.assign(nInst, 0.0);
    this->AfluxAtInstrumentMax.assign(nInst, 0.0);
    this->AfluxAtInstrument.assign(nInst, 0.0);
}

/*! This method reads the messages and caches spacecraft/sun state; clears per-instrument accumulators */
void Albedo::onUpdateBegin(const SCStatesMsgPayload&         scMsg,
                           const SpicePlanetStateMsgPayload& sunMsg,
                           uint64_t                          nanos)
{
    this->r_BN_N       = Eigen::Vector3d(scMsg.r_BN_N);  // [m]
    const Eigen::MRPd sigma_BN(Eigen::Vector3d(scMsg.sigma_BN));
    this->dcm_BN       = sigma_BN.toRotationMatrix().transpose(); // dcm_BN: rows are B axes expressed in N
    this->r_SN_N       = Eigen::Vector3d(sunMsg.PositionVector);  // [m]
    this->currentNanos = nanos;  // [ns]

    const size_t nInst = this->albOutMsgs.size();
    this->albedoAtInstrumentMax.assign(nInst, 0.0);
    this->albedoAtInstrument.assign(nInst, 0.0);
    this->SfluxAtInstrument.assign(nInst, 0.0);
    this->AfluxAtInstrumentMax.assign(nInst, 0.0);
    this->AfluxAtInstrument.assign(nInst, 0.0);
}

/*! This method evaluates the albedo model for one planet: applies FOV and eclipse filtering per instrument */
void Albedo::evaluatePlanet(const SCStatesMsgPayload&         /*scMsg*/,
                             const SpicePlanetStateMsgPayload& sunMsg,
                             SpicePlanetStateMsgPayload        planetMsg,
                             PlanetEntry&                      entry,
                             const double                      S_sun,
                             uint64_t                          /*nanos*/)
{
    const Eigen::Vector3d r_PN_N(planetMsg.PositionVector);  // [m]
    const Eigen::Vector3d r_SP_N = this->r_SN_N - r_PN_N;  // [m]

    for (int instIdx = 0; instIdx < static_cast<int>(this->albOutMsgs.size()); instIdx++) {
        double alb_I = 0.0, alb_Imax = 0.0;
        const Eigen::Vector3d r_IB_N = this->dcm_BN.transpose() * this->r_IB_Bs[instIdx];  // [m]
        const Eigen::Vector3d r_IP_N = r_IB_N + this->r_BN_N - r_PN_N;  // [m]

        // Altitude-rate-limit guard
        if (this->altitudeRateLimit >= 0.0) {
            const double altiI = r_IP_N.norm() - entry.R_planet;  // [m]
            if (altiI / entry.R_planet > this->altitudeRateLimit) {
                this->bskLogger.bskLog(BSK_WARNING, "Albedo Module (evaluatePlanet): The rate (altitude to planet's radii) limit is exceeded for the planet (%s) and albedo set to zero.", planetMsg.PlanetName);
                continue;
            }
        }

        // Distance-corrected solar flux at the instrument location
        const Eigen::Vector3d r_SI_N = r_SP_N - r_IP_N;  // [m]
        const double rSI = r_SI_N.norm();  // [m]
        this->SfluxAtInstrument[instIdx] = SOLAR_FLUX_EARTH * std::pow(AU2M, 2) / pow(rSI, 2);  // [W/m^2]

        // Compute per-patch fluxes using INSTRUMENT position (inertial; kernel subtracts r_planet)
        const Eigen::Vector3d r_IN_N = r_IB_N + this->r_BN_N;  // [m]
        const double r_IN_arr[3] = {r_IN_N[0], r_IN_N[1], r_IN_N[2]};  // [m]
        const std::vector<PatchResult> patches = entry.grid.computePatches(
            r_IN_arr,
            sunMsg.PositionVector,
            planetMsg.PositionVector,
            planetMsg.J20002Pfix,
            entry.R_planet,
            S_sun);

        const Eigen::Vector3d nHat_N = dcm_BN.transpose() * this->nHat_Bs[instIdx];  // [-]

        for (const PatchResult& p : patches) {
            if (p.albedoFlux <= 0.0) { continue; }  // skip unilluminated patches

            const double tempmax = p.albedoFlux / S_sun;  // [-]

            // Optional eclipse shadowing: smooth penumbra factor via base class helper
            const double illum = this->getEclipseCase()
                ? this->computeIlluminationAtdA(this->currentPlanetRadius,
                                                Eigen::Vector3d(p.r_dAP_N),
                                                this->currentR_SP_N)
                : this->illuminationFactorAtdA;  // [-]

            alb_Imax += tempmax * illum;  // [-]

            // FOV check: f3 = nHat_N · (−dir_N) - instrument looking toward patch
            const Eigen::Vector3d dir_N(p.rHat_IdA_N[0], p.rHat_IdA_N[1], p.rHat_IdA_N[2]);  // [-]
            const double f3 = nHat_N.dot(-dir_N);  // [-]
            if (f3 >= std::cos(fovs[instIdx])) {
                const double tempfov = tempmax * f3;
                alb_I += tempfov * illum;  // [-]
            }
        }

        this->albedoAtInstrument[instIdx] += alb_I;  // [-]
        this->albedoAtInstrumentMax[instIdx] += alb_Imax;  // [-]
        this->AfluxAtInstrumentMax[instIdx] += this->SfluxAtInstrument[instIdx] * alb_Imax;  // [W/m^2]
        this->AfluxAtInstrument[instIdx] += this->SfluxAtInstrument[instIdx] * alb_I;  // [W/m^2]
    }
}

/*! This method writes the output albedo messages from accumulated per-instrument data */
void Albedo::onUpdateEnd(uint64_t nanos)
{
    AlbedoMsgPayload localMsg;
    std::memset(&localMsg, 0x0, sizeof(localMsg));

    for (size_t idx = 0; idx < this->albOutMsgs.size(); idx++) {
        localMsg.albedoAtInstrumentMax = this->albedoAtInstrumentMax[idx];  // [-]
        localMsg.albedoAtInstrument    = this->albedoAtInstrument[idx];  // [-]
        localMsg.AfluxAtInstrumentMax  = this->AfluxAtInstrumentMax[idx];  // [W/m^2]
        localMsg.AfluxAtInstrument     = this->AfluxAtInstrument[idx];  // [W/m^2]
        this->albOutMsgs[idx]->write(&localMsg, this->moduleID, nanos);
    }
}

/*! Planet's equatorial radii and polar radii (if exists) in meters
 *  @return pair {REQ_m, RP_m}; RP_m < 0 for spherical bodies */
std::pair<double, double> Albedo::lookupPlanetRadius(const std::string& name)
{
    if      (name == "mercury")                           { return {REQ_MERCURY * 1000.0, -1.0}; }
    else if (name == "venus")                             { return {REQ_VENUS   * 1000.0, -1.0}; }
    else if (name == "earth")                             { return {REQ_EARTH   * 1000.0, RP_EARTH  * 1000.0}; }
    else if (name == "moon")                              { return {REQ_MOON    * 1000.0, -1.0}; }
    else if (name == "mars" || name == "mars barycenter") { return {REQ_MARS    * 1000.0, RP_MARS   * 1000.0}; }
    else if (name == "jupiter")                           { return {REQ_JUPITER * 1000.0, -1.0}; }
    else if (name == "saturn")                            { return {REQ_SATURN  * 1000.0, -1.0}; }
    else if (name == "uranus")                            { return {REQ_URANUS  * 1000.0, -1.0}; }
    else if (name == "neptune")                           { return {REQ_NEPTUNE * 1000.0, -1.0}; }
    else {
        this->bskLogger.bskError("Albedo Module (lookupPlanetRadius): The planet's radius cannot be obtained. The planet (%s) is not found.", name.c_str());
    }
}

/*! This method gets the average albedo value of the planet
 *  @return double */
double Albedo::getAlbedoAverage(std::string planetSpiceName)
{
    if      (planetSpiceName == "mercury")                          { return 0.119; }
    else if (planetSpiceName == "venus")                            { return 0.75;  }
    else if (planetSpiceName == "earth")                            { return 0.29;  }
    else if (planetSpiceName == "moon")                             { return 0.123; }
    else if (planetSpiceName == "mars" || planetSpiceName == "mars barycenter") { return 0.16; }
    else if (planetSpiceName == "jupiter")                          { return 0.34;  }
    else if (planetSpiceName == "saturn")                           { return 0.34;  }
    else if (planetSpiceName == "uranus")                           { return 0.29;  }
    else if (planetSpiceName == "neptune")                          { return 0.31;  }
    else {
        this->bskLogger.bskError("Albedo Module (getAlbedoAverage): The average albedo value is not defined for the specified planet (%s).", planetSpiceName.c_str());
    }
}
