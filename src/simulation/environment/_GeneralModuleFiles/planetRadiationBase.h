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

#ifndef PLANET_RADIATION_BASE_H
#define PLANET_RADIATION_BASE_H

#include <Eigen/Dense>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Per-patch flux and geometry returned by PlanetGrid::computePatches(). */
struct PatchResult {
    double albedoFlux;     //!< [W/m^2] albedo flux from this patch; 0 if not illuminated
    double irFlux;         //!< [W/m^2] IR flux from this patch; > 0 for all visible patches
    double rHat_IdA_N[3];  //!< [-] unit vector patch centroid -> satellite, inertial N
    double solidAngle;     //!< [sr] solid angle of patch as seen from satellite
    double r_dAP_N[3];     //!< [m] patch centroid position relative to planet centre, N frame
};

/*! @brief Initialised patch grid for one planet.
 *
 * Configure all public members, then call initialize().
 * After that, call computePatches() every timestep.
 */
struct PlanetGrid {
    int    nLat         = 180;    //!< [-] latitude zones
    int    nLon         = 360;    //!< [-] longitude zones per band
    double irFluxMean   = 237.0;  //!< [W/m^2] mean
    double albedoAvg    = 0.30;   //!< [-] Bond albedo (albedoDataPath="")
    bool useAlbedoData  = false;  //!< true = load CSV albedo data
    std::string albedoDataPath = "";  //!< path to supportData/AlbedoData/
    std::string albedoDataFile = "";  //!< CSV filename
    double REQ_m = -1.0;  //!< [m] equatorial radius; -1 = must be set before initialize()
    double RP_m  = -1.0;  //!< [m] polar radius; -1 = spherical (REQ used)

    /*! Builds the patch grid and loads optional CSV albedo data.
     *  Call once after setting all configuration members.
     */
    void initialize(BSKLogger bskLogger);

    /*! Computes per-patch radiation fluxes for one timestep.
     *
     *  Only patches with cos_sat > 0 (visible from r_sat_N) are returned.
     *  Eclipse shadowing is not applied here; use isPatchEclipsed() post-kernel.
     *
     *  @param r_sat_N    [m] satellite (or instrument) position, inertial N
     *  @param r_sun_N    [m] Sun position, inertial N
     *  @param r_planet_N [m] planet centre position, inertial N
     *  @param J20002Pfix [-] SpicePlanetStateMsgPayload rotation matrix (row-major)
     *  @param R_planet   [m] authalic planet radius
     *  @param S_sun      [W/m^2] solar flux at planet, distance-corrected
     */
    std::vector<PatchResult> computePatches(
        const double r_sat_N[3],
        const double r_sun_N[3],
        const double r_planet_N[3],
        double       J20002Pfix[3][3],
        double       R_planet,
        const double S_sun) const;

private:
    std::vector<std::vector<double>> loadAlbedoGridFromCsv(BSKLogger bskLogger) const;

    std::vector<std::array<double, 3>> patchDirs_P;  //!< [-] patch centroid directions in planet frame
    std::vector<double> normAreas;   //!< [sr] normalised patch areas
    std::vector<double> patchAlbedo; //!< [-] per-patch albedo values
    double t_aut[3]    = {0.0, 0.0, 0.0}; //!< authalic latitude correction coefficients
    bool   hasAuthalic  = false; //!< true when oblate spheroid correction is active
    double latDiff      = 0.0;   //!< [rad] latitude spacing between grid points
    double lonDiff      = 0.0;   //!< [rad] longitude spacing between grid points
};

/*! @brief One registered planet: message functor, initialised grid, authalic radius. */
struct PlanetEntry {
    ReadFunctor<SpicePlanetStateMsgPayload> planetMsg;      //!< per-planet input message
    PlanetGrid                              grid;           //!< initialised patch grid
    double                                  R_planet = 0.0; //!< [m] authalic radius, set in Reset()
};

/*! @brief Abstract base class for Basilisk planetary-radiation environment modules.
 *
 * PlanetRadiationBase reads spacecraft and sun state messages, iterates over
 * registered planets, and dispatches per-planet evaluation to the derived class
 * via evaluatePlanet().  Concrete subclasses implement evaluatePlanet() and
 * optionally override the other virtual hooks.
 */
class PlanetRadiationBase : public SysModel {
public:
    PlanetRadiationBase() = default;
    ~PlanetRadiationBase() override = default;

    /*! Validates messages, builds planet entries, and initialises patch grids.
     *  @param CurrentSimNanos  Current simulation time (ns).
     */
    void Reset(uint64_t CurrentSimNanos) override;

    /*! Reads inputs, dispatches per-planet evaluation, and writes outputs.
     *  @param CurrentSimNanos  Current simulation time (ns).
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /*! Registers a planet. Call from derived addPlanet*() methods.
     *
     *  gridConfig may have sentinel values (e.g. REQ_m=-1, albedoAvg=-1, nLat=0)
     *  that resolvePlanetEntry() fills in during Reset().
     *
     *  @param msg        Planet state message functor.
     *  @param gridConfig Partially-configured patch grid.
     */
    void addPlanetEntry(ReadFunctor<SpicePlanetStateMsgPayload> msg,
                        const PlanetGrid&                       gridConfig);

    const bool getEclipseCase();  //!< @brief getter for the eclipseCase flag
    void setEclipseCase(bool value);  //!< @brief setter for the eclipseCase flag

    int    defaultNumLat = 180;   //!< [-] default number of latitude grid points
    int    defaultNumLon = 360;   //!< [-] default number of longitude grid points
    double irFluxMean    = 237.0; //!< [W/m^2] mean outgoing IR radiation
    double albedoAvg     = 0.30;  //!< [-] Bond albedo, used when albedoDataPath is left empty

    std::string albedoDataPath = "";  //!< path to supportData/AlbedoData/
    std::string albedoDataFile = "";  //!< CSV filename

    double REQ_m = -1.0;  //!< [m] equatorial radius; -1 (default) = use WGS-84 Earth value
    double RP_m  = -1.0;  //!< [m] polar radius; -1 (default) = use WGS-84 Earth value

    ReadFunctor<SCStatesMsgPayload>         spacecraftStateInMsg; //!< spacecraft position input message
    ReadFunctor<SpicePlanetStateMsgPayload> planetInMsg;          //!< single-planet convenience (ERM path)
    ReadFunctor<SpicePlanetStateMsgPayload> sunPositionInMsg;     //!< Sun position input message

    BSKLogger bskLogger; //!< BSK logging

    bool eclipseCase = false;  //!< true = compute eclipse shadowing at each patch. Deprecated, will be removed after May 1st 2027.

    std::vector<PlanetEntry> planets; //!< registered planets, built in Reset() or addPlanetEntry()

protected:
    /*! Called once per planet per timestep.  PURE VIRTUAL.
     *
     *  @param scMsg      spacecraft state
     *  @param sunMsg     sun state
     *  @param planetMsg  planet state (non-const copy, safe to pass J20002Pfix onward)
     *  @param entry      planet entry with initialised grid and authalic radius
     *  @param S_sun      [W/m^2] distance-corrected solar flux at planet
     *  @param nanos      simulation time
     */
    virtual void evaluatePlanet(const SCStatesMsgPayload&         scMsg,
                                const SpicePlanetStateMsgPayload& sunMsg,
                                SpicePlanetStateMsgPayload        planetMsg,
                                PlanetEntry&                      entry,
                                const double                      S_sun,
                                uint64_t                          nanos) = 0;

    /*! Called in Reset() for each entry after reading the planet message.
     *
     *  Override to fill in grid members that depend on the planet name,
     *  e.g. REQ_m, RP_m, albedoAvg, nLat, nLon.
     *  Default: no-op (ERM relies on scalar config set before Reset()).
     *
     *  @param planetMsg  Planet state message.
     *  @param grid       PlanetGrid to fill in.
     *  @param idx        Index of this planet entry.
     */
    virtual void resolvePlanetEntry(const SpicePlanetStateMsgPayload& planetMsg,
                                    PlanetGrid& grid,
                                    int         idx) {}

    /*! Called at the start of UpdateState(), before the planet loop.
     *  Override to clear per-timestep accumulators or cache the spacecraft attitude.
     *  Default: no-op.
     *
     *  @param scMsg   Spacecraft state message.
     *  @param sunMsg  Sun state message.
     *  @param nanos   Current simulation time (ns).
     */
    virtual void onUpdateBegin(const SCStatesMsgPayload&         scMsg,
                               const SpicePlanetStateMsgPayload& sunMsg,
                               uint64_t                          nanos) {}

    /*! Called after the planet loop in UpdateState().
     *  Override to write output messages.
     *  Default: no-op.
     *
     *  @param nanos  Current simulation time (ns).
     */
    virtual void onUpdateEnd(uint64_t nanos) {}

    /*! Called at the end of Reset().
     *  Override for additional validation (e.g. instrument configuration).
     *  Default: no-op.
     *
     *  @param nanos  Current simulation time (ns).
     */
    virtual void customReset(uint64_t nanos) {}

    /*! Return true to enable the single-planet backward-compat fallback in Reset():
     *  when planets is empty and planetInMsg is linked, a default PlanetGrid
     *  is built from the scalar config fields.
     *  Default: false.
     */
    virtual bool allowSinglePlanetFallback() const { return false; }

    /*! Determines if a patch is in eclipse (umbra/penumbra).
     *  Returns true if the patch should contribute zero albedo flux.
     *
     *  @param patch  Patch result to evaluate.
     *  @param scMsg  Spacecraft state message.
     *  @param sunMsg Sun state message.
     */
    virtual bool isPatchEclipsed(const PatchResult&                patch,
                                 const SCStatesMsgPayload&         scMsg,
                                 const SpicePlanetStateMsgPayload& sunMsg);

    /*! Computes the umbra/penumbra illumination factor [0, 1] for a surface
     *  element at r_dAP_N (relative to planet centre).
     *  Returns 0 in full umbra, 1 in full sunlight, smooth intermediate in penumbra.
     *
     *  @param Rplanet   [m] planet radius
     *  @param r_dAP_N   [m] patch position relative to planet centre
     *  @param r_SP_N    [m] Sun position relative to planet centre
     */
    static double computeIlluminationAtdA(double          Rplanet,
                                          Eigen::Vector3d r_dAP_N,
                                          Eigen::Vector3d r_SP_N);

    Eigen::Vector3d currentR_SP_N = Eigen::Vector3d::Zero(); //!< [m] Sun position relative to current planet centre
    double currentPlanetRadius = 0.0;                        //!< [m] current planet authalic radius
    bool useAlbedoData = false;                              //!< true = load CSV albedo data
    bool m_eclipseCase = false;                              //!< true = compute eclipse shadowing at each patch
};

#endif /* PLANET_RADIATION_BASE_H */
