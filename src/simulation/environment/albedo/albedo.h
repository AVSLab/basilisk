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

#ifndef ALBEDO_H
#define ALBEDO_H


#include <Eigen/Dense>
#include <string>

#include "architecture/msgPayloadDefC/AlbedoMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/astroConstants.h"
#include "simulation/environment/_GeneralModuleFiles/planetRadiationBase.h"

/*! albedo instrument configuration class */
typedef class Config {
public:
    Config();
    ~Config();
    double fov;              //!< [rad] instrument's field of view half angle
    Eigen::Vector3d nHat_B;  //!< [-] unit normal of the instrument (spacecraft body)
    Eigen::Vector3d r_IB_B;  //!< [m] instrument's misalignment wrt spacecraft's body frame
} instConfig_t;

/*! @brief albedo class
 *
 * Note: `spacecraftStateInMsg` and `sunPositionInMsg` are provided by `PlanetRadiationBase`.
 * The inherited `planetInMsg` is unused in this class; planets must be registered via
 * `addPlanetandAlbedoAverageModel()` or `addPlanetandAlbedoDataModel()`.
 */
class Albedo : public PlanetRadiationBase {
public:
    Albedo();
    ~Albedo();

    void addInstrumentConfig(instConfig_t configMsg);  //!< @brief adds instrument configuration (overloaded function)
    void addInstrumentConfig(double fov, Eigen::Vector3d nHat_B, Eigen::Vector3d r_IB_B);  //!< @brief adds instrument configuration (overloaded function)
    void addPlanetandAlbedoAverageModel(Message<SpicePlanetStateMsgPayload> *msg); //!< @brief This method adds planet msg and albedo average model name (overloaded function)
    void addPlanetandAlbedoAverageModel(Message<SpicePlanetStateMsgPayload> *msg, double ALB_avg, int numLat, int numLon);  //!< @brief This method adds planet name and albedo average model name (overloaded function)
    void addPlanetandAlbedoDataModel(Message<SpicePlanetStateMsgPayload> *msg, std::string dataPath, std::string fileName); //!< @brief This method adds planet name and albedo data model
    double getAlbedoAverage(std::string planetSpiceName);     //!< @brief gets the average albedo value of the specified planet

    std::vector<Message<AlbedoMsgPayload>*> albOutMsgs; //!< output messages (one per instrument)

    Eigen::Vector3d nHat_B_default = {1.0, 0.0, 0.0}; //!< [-] default instrument unit normal (body frame)
    double fov_default = 90.0 * D2R;                  //!< [rad] default instrument FOV half angle
    double illuminationFactorAtdA = 1.0;              //!< [-] constant illumination factor (m_eclipseCase=false)
    double altitudeRateLimit = -1.0;                  //!< [-] max altitude/radius ratio; <0 = disabled

protected:
    // ------------------------------------------------------------------ //
    // PlanetRadiationBase hooks                                            //
    // ------------------------------------------------------------------ //

    void resolvePlanetEntry(const SpicePlanetStateMsgPayload& planetMsg,
                            PlanetGrid& grid, int idx) override;  //!< fills in planet-dependent grid parameters
    void customReset(uint64_t nanos) override;                    //!< validates instrument and planet configuration
    void onUpdateBegin(const SCStatesMsgPayload&         scMsg,
                       const SpicePlanetStateMsgPayload& sunMsg,
                       uint64_t                          nanos) override;  //!< caches spacecraft/sun state; clears accumulators
    void evaluatePlanet(const SCStatesMsgPayload&         scMsg,
                        const SpicePlanetStateMsgPayload& sunMsg,
                        SpicePlanetStateMsgPayload        planetMsg,
                        PlanetEntry&                      entry,
                        const double                      S_sun,
                        uint64_t                          nanos) override;  //!< evaluates the albedo model for one planet
    void onUpdateEnd(uint64_t nanos) override;                    //!< writes the output albedo messages

private:
    std::pair<double, double> lookupPlanetRadius(const std::string& name);  //!< gets the planet's equatorial and polar radii in meters

    std::vector<Eigen::Vector3d> r_IB_Bs;       //!< [m] instrument misalignment, body frame
    std::vector<Eigen::Vector3d> nHat_Bs;       //!< [-] instrument unit normal, body frame
    std::vector<double> fovs;                   //!< [rad] instrument FOV half angles
    Eigen::Vector3d r_BN_N;                     //!< [m] spacecraft position, N frame
    Eigen::Matrix3d dcm_BN;                     //!< [-] rotation matrix body to inertial
    Eigen::Vector3d r_SN_N;                     //!< [m] sun position, N frame
    uint64_t currentNanos = 0;                  //!< [ns] current simulation time
    std::vector<double> albedoAtInstrumentMax;  //!< [-] albedo ratio sum over all visible patches
    std::vector<double> albedoAtInstrument;     //!< [-] FOV-filtered albedo ratio sum
    std::vector<double> SfluxAtInstrument;      //!< [W/m^2] solar flux at instrument (last-planet value)
    std::vector<double> AfluxAtInstrumentMax;   //!< [-] Albedo flux ratio at instrument
    std::vector<double> AfluxAtInstrument;      //!< [W/m^2] Albedo flux at instrument
};

#endif /* ALBEDO_H */
