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

#include "simulation/environment/_GeneralModuleFiles/planetRadiationBase.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>
namespace fs = std::filesystem;


void PlanetGrid::initialize(BSKLogger bskLogger)
{
    // --- Validate grid dimensions ---
    // Check that latitude and longitude grid counts are positive
    if (this->nLat <= 0) {
        bskLogger.bskError("PlanetGrid::initialize(): nLat must be positive, got %d.", this->nLat);
    }
    if (this->nLon <= 0) {
        bskLogger.bskError("PlanetGrid::initialize(): nLon must be positive, got %d.", this->nLon);
    }

    // --- Authalic latitude correction coefficients ---
    // For oblate spheroids, compute the authalic latitude correction used to preserve
    // surface area when mapping the ellipsoid onto an equivalent sphere.
    this->hasAuthalic = false;
    std::fill(std::begin(this->t_aut), std::end(this->t_aut), 0.0);
    if (this->REQ_m > 0.0 && this->RP_m > 0.0 && this->RP_m != this->REQ_m) {
        if (this->RP_m >= this->REQ_m) {
            bskLogger.bskError(
                "PlanetGrid::initialize(): RP_m must be less than REQ_m for oblate spheroid, "
                "got RP_m=%f, REQ_m=%f.", this->RP_m, this->REQ_m);
        }
        // Compute eccentricity and its powers for authalic correction
        const double e  = std::sqrt(1.0 - pow(this->RP_m, 2) / pow(this->REQ_m, 2));
        const double e2 = pow(e, 2), e4 = pow(e2, 2), e6 = e4 * e2;
        // Authalic latitude correction polynomial coefficients
        this->t_aut[0] = e2 / 3.0 + 31.0 * e4 / 180.0 + 59.0 * e6 / 560.0;
        this->t_aut[1] = 17.0 * e4 / 360.0 + 61.0 * e6 / 1260.0;
        this->t_aut[2] = 383.0 * e6 / 45360.0;
        this->hasAuthalic = true;
    } else if (this->REQ_m > 0.0 && this->RP_m <= 0.0) {
        // REQ_m is set but RP_m is not positive - this is an error unless RP_m == -1 (sentinel)
        if (this->RP_m != -1.0) {
            bskLogger.bskError(
                "PlanetGrid::initialize(): RP_m must be positive or -1 (sentinel) when REQ_m is set, "
                "got RP_m=%f, REQ_m=%f.", this->RP_m, this->REQ_m);
        }
    } else if (this->RP_m > 0.0 && this->REQ_m <= 0.0) {
        // RP_m is set but REQ_m is not positive
        bskLogger.bskError(
            "PlanetGrid::initialize(): REQ_m must be positive when RP_m is set, "
            "got RP_m=%f, REQ_m=%f.", this->RP_m, this->REQ_m);
    }

    // --- Validate albedo data mode ---
    // Ensure albedo data configuration is consistent
    if (this->useAlbedoData && this->albedoDataPath.empty()) {
        bskLogger.bskError(
            "PlanetGrid::initialize(): useAlbedoData is true but albedoDataPath is empty. "
            "Provide a valid albedoDataPath and albedoDataFile, or set useAlbedoData to false.");
    }

    // --- Load CSV or fill with constant albedo ---
    // If albedoDataPath is provided, load albedo values from CSV file
    std::vector<std::vector<double>> albGrid;
    if (!this->albedoDataPath.empty()) {
        this->useAlbedoData = true;
        albGrid = this->loadAlbedoGridFromCsv(bskLogger);
        if (albGrid.empty()) {
            bskLogger.bskError("PlanetGrid::initialize(): failed to load albedo grid.");
        }
        // Use the dimensions of the loaded albedo map as the grid dimensions
        this->nLat = static_cast<int>(albGrid.size());
        this->nLon = static_cast<int>(albGrid[0].size());
        // Re-validate dimensions after loading from CSV
        if (this->nLat <= 0) {
            bskLogger.bskError("PlanetGrid::initialize(): nLat from albedo file must be positive, got %d.", this->nLat);
        }
        if (this->nLon <= 0) {
            bskLogger.bskError("PlanetGrid::initialize(): nLon from albedo file must be positive, got %d.", this->nLon);
        }
    }

    // --- Build lat/lon grid ---
    // Compute angular spacing between grid points in radians
    this->latDiff = (180.0 / this->nLat) * M_PI / 180.0;  // [rad]
    this->lonDiff = (360.0 / this->nLon) * M_PI / 180.0;  // [rad]
    const int halfLat = this->nLat / 2;
    const int halfLon = this->nLon / 2;

    // Create grid point coordinates: centered at origin, spanning -180 to 180 deg lon, -90 to 90 deg lat
    std::vector<double> gdlat(this->nLat), gdlon(this->nLon);// Normalised area = solid angle of patch on unit sphere: dOmega = dlon * (sin(lat1) - sin(lat2))
    for (int i = 0; i < this->nLat; ++i) { gdlat[i] = (i - halfLat + 0.5) * this->latDiff; }
    for (int j = 0; j < this->nLon; ++j) { gdlon[j] = (j - halfLon + 0.5) * this->lonDiff; }

    // --- Precompute patch directions (P frame) and normalised areas ---
    // For each grid cell, compute the unit direction vector from planet center to patch center
    // and the normalised area (solid angle) of the patch on the unit sphere.
    const int nPatches = this->nLat * this->nLon;
    this->patchDirs_P.resize(nPatches);
    this->normAreas.resize(nPatches);
    this->patchAlbedo.resize(nPatches);

    for (int ilat = 0; ilat < this->nLat; ++ilat) {
        const double lat  = gdlat[ilat];  // [rad]
        // Latitude bounds of this grid cell
        double lat1 = lat + 0.5 * this->latDiff;  // [rad]
        double lat2 = lat - 0.5 * this->latDiff;  // [rad]

        // Apply authalic correction to latitude bounds for oblate spheroids
        if (this->hasAuthalic) {
            lat1 -= this->t_aut[0]*std::sin(2.0*lat1) - t_aut[1]*std::sin(4.0*lat1)
                  + this->t_aut[2]*std::sin(6.0*lat1);  // [rad]
            lat2 -= this->t_aut[0]*std::sin(2.0*lat2) - t_aut[1]*std::sin(4.0*lat2)
                  + this->t_aut[2]*std::sin(6.0*lat2);  // [rad]
        }
        // Normalised area = solid angle of patch on unit sphere: dOmega = dlon * (sin(lat1) - sin(lat2))
        const double normArea = lonDiff * std::fabs(std::sin(lat1) - std::sin(lat2));  // [sr]
        const double cosLat = std::cos(lat), sinLat = std::sin(lat);  // [-]

        for (int ilon = 0; ilon < this->nLon; ++ilon) {
            const int k = ilat * this->nLon + ilon;
            const double lon = gdlon[ilon];  // [rad]
            // Unit vector from planet center to patch center in planet frame P
            this->patchDirs_P[k] = {cosLat*std::cos(lon), cosLat*std::sin(lon), sinLat};  // [-]
            this->normAreas[k]   = normArea;  // [sr]
            // Assign albedo value: from CSV data if loaded, otherwise use constant average
            this->patchAlbedo[k] = this->useAlbedoData ? albGrid[ilat][ilon] : this->albedoAvg;  // [-]
        }
    }
}

std::vector<std::vector<double>> PlanetGrid::loadAlbedoGridFromCsv(BSKLogger bskLogger) const
{
    // Load albedo grid from CSV file at albedoDataPath/albedoDataFile
    std::error_code ec;

    // Resolve the base directory path (canonical = absolute, resolved symlinks)
    fs::path base = fs::canonical(this->albedoDataPath, ec);
    if (ec) {
        bskLogger.bskError(
            "PlanetGrid::loadAlbedoGridFromCsv(): invalid albedoDataPath '%s' (%s)",
            this->albedoDataPath.c_str(), ec.message().c_str());
    }

    ec.clear();
    // Resolve the target file path while tolerating non-existent path elements.
    fs::path filePath = fs::weakly_canonical(base / this->albedoDataFile, ec);
    if (ec) {
        bskLogger.bskError(
            "PlanetGrid::loadAlbedoGridFromCsv(): invalid albedo file path '%s' (%s)",
            (base / this->albedoDataFile).string().c_str(), ec.message().c_str());
    }

    // Ensure the resolved file path remains within the configured base directory.
    auto [baseEnd, _] = std::mismatch(base.begin(), base.end(), filePath.begin());
    if (baseEnd != base.end()) {
        bskLogger.bskError("PlanetGrid::loadAlbedoGridFromCsv(): path not acceptable.");
    }

    // Open and read the CSV file
    std::ifstream input(filePath);
    if (!input) {
        bskLogger.bskError(
            "PlanetGrid::loadAlbedoGridFromCsv(): cannot open albedo file: %s", filePath.string().c_str());
    }

    std::vector<std::vector<double>> albGrid;
    std::string line = "", field = "";
    int expectedColumns = -1;  // Track expected column count from first non-empty line
    int lineNumber = 0;        // Track line number for error reporting

    // Parse each line of the CSV file
    while (std::getline(input, line)) {
        std::vector<double> row;
        std::stringstream ss(line);

        try {
            // Parse comma-separated values from the current row
            while (std::getline(ss, field, ',')) {
                // Trim leading and trailing whitespace
                field.erase(0, field.find_first_not_of(" \t\n\r"));
                auto last = field.find_last_not_of(" \t\n\r");

                if (last != std::string::npos) {
                    field.erase(last + 1);
                    if (field.empty()) {
                        bskLogger.bskError("PlanetGrid::loadAlbedoGridFromCsv(): empty field encountered.");
                    }
                } else {
                    bskLogger.bskError("PlanetGrid::loadAlbedoGridFromCsv(): empty field encountered.");
                }

                // Convert field to double using classic (C) locale for consistent number parsing
                double val = 0.0;
                std::istringstream iss(field);
                iss.imbue(std::locale::classic());
                if (!(iss >> val)) {
                    bskLogger.bskError("PlanetGrid::loadAlbedoGridFromCsv(): invalid double: %s", field.c_str());
                }

                row.push_back(val);
            }
        } catch (const std::exception& e) {
            bskLogger.bskError(
                "PlanetGrid::loadAlbedoGridFromCsv(): invalid data format in file %s at line %d - %s",
                filePath.string().c_str(), lineNumber, e.what());
        }

        // Track and validate column counts
        if (!row.empty()) {
            if (expectedColumns == -1) {
                expectedColumns = static_cast<int>(row.size());
            } else if (static_cast<int>(row.size()) != expectedColumns) {
                bskLogger.bskError(
                    "PlanetGrid::loadAlbedoGridFromCsv(): inconsistent number of columns in file %s",
                    filePath.string().c_str());
            }
            albGrid.push_back(row);
        }
        ++lineNumber;
    }

    // Validate that we loaded data
    if (albGrid.empty() || albGrid[0].empty()) {
        bskLogger.bskError(
            "PlanetGrid::loadAlbedoGridFromCsv(): empty albedo file: %s", filePath.string().c_str());
    }

    return albGrid;
}

std::vector<PatchResult> PlanetGrid::computePatches(
    const double r_sat_N[3],
    const double r_sun_N[3],
    const double r_planet_N[3],
    double       J20002Pfix[3][3],
    double       R_planet,
    const double S_sun) const
{
    // Convert the SPICE J2000->P rotation matrix into an Eigen matrix.
    // Due to row-major/column-major interpretation, this yields J20002Pfixᵀ,
    // i.e. the rotation from planet-fixed frame P to inertial frame N.
    const Eigen::Matrix3d dcm_NP = cArray2EigenMatrixXd(*J20002Pfix, 3, 3);  // [-]

    // Relative position vectors from planet center to satellite and Sun (in inertial N frame)
    const Eigen::Vector3d rs(r_sat_N[0]-r_planet_N[0], r_sat_N[1]-r_planet_N[1], r_sat_N[2]-r_planet_N[2]);  // [m]
    const Eigen::Vector3d rS(r_sun_N[0]-r_planet_N[0], r_sun_N[1]-r_planet_N[1], r_sun_N[2]-r_planet_N[2]);  // [m]

    const double R2 = pow(R_planet, 2);  // [m^2]

    std::vector<PatchResult> results;
    // Reserve approximate storage; only visible patches contribute.
    results.reserve(static_cast<size_t>(this->nLat * this->nLon) / 2);

    // Iterate over all patches and compute radiation contributions
    for (int k = 0; k < this->nLat * this->nLon; ++k) {
        // Patch direction in planet frame P
        const Eigen::Vector3d dir_P(this->patchDirs_P[k][0], this->patchDirs_P[k][1], this->patchDirs_P[k][2]);  // [-]
        // Rotate patch direction to inertial frame N and scale by planet radius
        const Eigen::Vector3d r_dAP_N = R_planet * (dcm_NP * dir_P);  // [m]
        const Eigen::Vector3d rHat_dAP_N = r_dAP_N / r_dAP_N.norm();  // [-]

        // Vector from patch to satellite
        const Eigen::Vector3d r_IdA_N = rs - r_dAP_N;  // [m]
        const double d = r_IdA_N.norm();  // [m]
        const Eigen::Vector3d rHat_IdA_N = r_IdA_N / d;  // [-]

        // Cosine of angle between patch normal and satellite direction
        // f2 <= 0 means patch is not visible from satellite (back-facing)
        const double f2 = rHat_dAP_N.dot(rHat_IdA_N);  // [-]
        if (f2 <= 0.0) { continue; }

        // Actual patch area on the planet surface (normalised area * planet radius squared)
        const double dArea    = this->normAreas[k] * R2;  // [m^2]
        // IR contribution assuming Lambertian emission.
        // M_1_PI accounts for Lambert's cosine law.
        const double dF_ir = this->irFluxMean * M_1_PI * f2 * dArea / pow(d, 2);  // [W/m^2]

        double dF_alb = 0.0;
        // Vector from patch to Sun
        const Eigen::Vector3d r_SdA_N = rS - r_dAP_N;  // [m]
        const double r_SdA_norm = r_SdA_N.norm();  // [m]
        // Cosine of angle between patch normal and Sun direction
        // f1 > 0 means patch is sunlit
        const double f1 = rHat_dAP_N.dot(r_SdA_N / r_SdA_norm);  // [-]
        if (f1 > 0.0) {
            // Reflected solar (albedo) contribution assuming Lambertian reflection.
            dF_alb = (S_sun * M_1_PI) * this->patchAlbedo[k] * f1 * f2 * dArea / pow(d, 2);  // [W/m^2]
        }

        PatchResult pr;
        pr.albedoFlux    = dF_alb;
        pr.irFlux        = dF_ir;
        pr.rHat_IdA_N[0] = rHat_IdA_N[0]; pr.rHat_IdA_N[1] = rHat_IdA_N[1]; pr.rHat_IdA_N[2] = rHat_IdA_N[2];
        pr.solidAngle    = f2 * dArea / pow(d, 2);
        pr.r_dAP_N[0]    = r_dAP_N[0]; pr.r_dAP_N[1] = r_dAP_N[1]; pr.r_dAP_N[2] = r_dAP_N[2];
        results.push_back(pr);
    }
    return results;
}

void PlanetRadiationBase::addPlanetEntry(ReadFunctor<SpicePlanetStateMsgPayload> msg,
                                          const PlanetGrid& gridConfig)
{
    PlanetEntry e;
    e.planetMsg = msg;
    e.grid      = gridConfig;
    this->planets.push_back(e);
}

void PlanetRadiationBase::Reset(uint64_t CurrentSimNanos)
{
    if (!this->spacecraftStateInMsg.isLinked()) {
        this->bskLogger.bskError(
            "PlanetRadiationBase (Reset): spacecraftStateInMsg is not linked.");
    }
    if (!this->sunPositionInMsg.isLinked()) {
        this->bskLogger.bskError(
            "PlanetRadiationBase (Reset): sunPositionInMsg is not linked.");
    }

    // Backward-compatibility path: construct a single planet entry from the
    // legacy scalar configuration when no planets have been registered.
    if (this->planets.empty() && this->allowSinglePlanetFallback()) {
        if (!this->planetInMsg.isLinked()) {
            this->bskLogger.bskError(
                "PlanetRadiationBase (Reset): planetInMsg is not linked and no planets "
                "were registered via addPlanetEntry().");
        }
        PlanetGrid cfg;
        cfg.nLat          = this->defaultNumLat;
        cfg.nLon          = this->defaultNumLon;
        cfg.irFluxMean    = this->irFluxMean;
        cfg.albedoAvg     = this->albedoAvg;
        cfg.useAlbedoData = this->useAlbedoData;
        cfg.albedoDataPath= this->albedoDataPath;
        cfg.albedoDataFile= this->albedoDataFile;
        cfg.REQ_m         = (this->REQ_m > 0.0) ? this->REQ_m : REQ_EARTH * 1000.0;  // [m]
        cfg.RP_m          = (this->RP_m  > 0.0) ? this->RP_m  : RP_EARTH  * 1000.0;  // [m]
        addPlanetEntry(this->planetInMsg, cfg);
    }

    for (int i = 0; i < static_cast<int>(this->planets.size()); ++i) {
        PlanetEntry& e = this->planets[i];

        SpicePlanetStateMsgPayload pm = e.planetMsg();

        // Hook: Albedo overrides to look up REQ_m, RP_m, albedoAvg from planet name
        this->resolvePlanetEntry(pm, e.grid, i);

        // Authalic planet radius
        const double REQ = e.grid.REQ_m;
        const double RP  = e.grid.RP_m;
        if (REQ <= 0.0) {
            this->bskLogger.bskError(
                "PlanetRadiationBase (Reset): REQ_m must be positive, got %f for planet %d.",
                REQ, i);
        }
        if (RP < 0.0 && RP != -1.0) {
            this->bskLogger.bskError(
                "PlanetRadiationBase (Reset): RP_m must be non-negative or -1 (sentinel), got %f for planet %d.",
                RP, i);
        }
        if (RP > 0.0 && RP != REQ) {
            if (RP >= REQ) {
                this->bskLogger.bskError(
                    "PlanetRadiationBase (Reset): RP_m must be less than REQ_m for oblate spheroid, "
                    "got RP_m=%f, REQ_m=%f for planet %d.", RP, REQ, i);
            }
            const double ec = std::sqrt(1.0 - pow(RP, 2) / pow(REQ, 2));
            e.R_planet = std::sqrt(pow(REQ, 2) * 0.5
                                   * (1.0 + (1.0 - pow(ec, 2)) / (2.0 * ec)
                                            * std::log((1.0 + ec) / (1.0 - ec))));
        } else {
            e.R_planet = REQ;  // spherical body: authalic radius == equatorial radius
        }

        try {
            e.grid.initialize(this->bskLogger);
        } catch (const std::exception& ex) {
            this->bskLogger.bskError(
                "PlanetRadiationBase (Reset): grid.initialize(this->bskLogger) failed for planet %d: %s",
                i, ex.what());
        }
    }

    customReset(CurrentSimNanos);
}

void PlanetRadiationBase::UpdateState(uint64_t CurrentSimNanos)
{
    SCStatesMsgPayload         scMsg  = this->spacecraftStateInMsg();
    SpicePlanetStateMsgPayload sunMsg = this->sunPositionInMsg();

    this->onUpdateBegin(scMsg, sunMsg, CurrentSimNanos);

    for (PlanetEntry& e : this->planets) {
        SpicePlanetStateMsgPayload planetMsg = e.planetMsg();
        if (m33Determinant(planetMsg.J20002Pfix) == 0.0) {
            m33SetIdentity(planetMsg.J20002Pfix);
        }

        const double r_SE[3] = {
            sunMsg.PositionVector[0] - planetMsg.PositionVector[0],
            sunMsg.PositionVector[1] - planetMsg.PositionVector[1],
            sunMsg.PositionVector[2] - planetMsg.PositionVector[2]
        };  // [m]
        const double r_SE_norm = std::hypot(r_SE[0], r_SE[1], r_SE[2]);  // [m]
        if (r_SE_norm <= 0) {
            this->bskLogger.bskError(
                "Distance Sun - planet (%s) <= 0 m.", planetMsg.PlanetName);
            }
        const double S_sun = SOLAR_FLUX_EARTH * pow(AU2M, 2) / pow(r_SE_norm, 2);  // [W/m^2]

        // Store eclipse geometry for use by isPatchEclipsed() during planet evaluation.
        this->currentR_SP_N       = Eigen::Vector3d(r_SE[0], r_SE[1], r_SE[2]);  // [m]
        this->currentPlanetRadius = e.R_planet;  // [m]

        this->evaluatePlanet(scMsg, sunMsg, planetMsg, e, S_sun, CurrentSimNanos);
    }

    this->onUpdateEnd(CurrentSimNanos);
}

/*! Compute the fraction of the solar disk visible from a surface patch.
    Returns:
     1.0 -> fully illuminated
     0.0 -> full umbra
     0.0 < value < 1.0 -> penumbra

   The calculation follows the apparent angular radii of the Sun and planet
   as seen from the patch and computes the overlap area of the two disks.*/
double PlanetRadiationBase::computeIlluminationAtdA(double          Rplanet,  // [m]
                                                    Eigen::Vector3d r_dAP_N,  // [m]
                                                    Eigen::Vector3d r_SP_N)   // [m]
{
    using std::asin; using std::acos; using std::sqrt; using std::fabs;

    const double REQ_SUN_M = REQ_SUN * 1000.0;  // [m]

    const Eigen::Vector3d r_SdA_N = r_SP_N - r_dAP_N;  // [m]
    const double s   = r_dAP_N.norm();  // [m]
    const double f_1 = safeAsin((REQ_SUN_M + Rplanet) / r_SP_N.norm());  // [rad]
    const double f_2 = safeAsin((REQ_SUN_M - Rplanet) / r_SP_N.norm());  // [rad]
    const double s_0 = (-r_dAP_N.dot(r_SP_N)) / r_SP_N.norm();  // [m]
    const double c_1 = s_0 + Rplanet / sin(f_1);  // [m]
    const double c_2 = s_0 - Rplanet / sin(f_2);  // [m]
    const double l   = sqrt(std::max(0.0, pow(s, 2) - pow(s_0, 2)));  // [m]
    const double l_1 = c_1 * tan(f_1);  // [m]
    const double l_2 = c_2 * tan(f_2);  // [m]

    const double a = safeAsin(REQ_SUN_M / r_SdA_N.norm());  // [rad]
    const double b = safeAsin(Rplanet   / r_dAP_N.norm());  // [rad]
    const double c = safeAcos((-r_dAP_N.dot(r_SdA_N)) / (r_dAP_N.norm() * r_SdA_N.norm()));  // [rad]

    double illuminationFactor = 1.0;  // [-]
    if (fabs(l) < fabs(l_2) || fabs(l) < fabs(l_1)) {
        if (c < b - a) {
            illuminationFactor = 0.0;  // [-]
        } else if (c < a - b) {
            const double areaSun  = M_PI * pow(a, 2);  // [sr]
            const double areaBody = M_PI * pow(b, 2);  // [sr]
            illuminationFactor = 1.0 - (areaSun - areaBody) / areaSun;  // [-]
        } else if (c < a + b) {
            const double x    = (pow(c, 2) + pow(a, 2) - pow(b, 2)) / (2.0 * c);  // [rad]
            const double y    = sqrt(std::max(0.0, pow(a, 2) - pow(x, 2)));  // [rad]
            const double area = pow(a, 2) * safeAcos(x / a) + pow(b, 2) * safeAcos((c - x) / b) - c * y;  // [sr]
            illuminationFactor = 1.0 - area / (M_PI * pow(a, 2));  // [-]
        }
    }
    return std::max(0.0, std::min(1.0, illuminationFactor));
}

bool PlanetRadiationBase::isPatchEclipsed(const PatchResult&                patch,
                                          const SCStatesMsgPayload&         /*scMsg*/,
                                          const SpicePlanetStateMsgPayload& /*sunMsg*/)
{
    if (!this->getEclipseCase()) { return false; }
    return this->computeIlluminationAtdA(this->currentPlanetRadius,
                              Eigen::Vector3d(patch.r_dAP_N[0],
                                             patch.r_dAP_N[1],
                                             patch.r_dAP_N[2]),
                              this->currentR_SP_N) < 0.001;
}

const bool PlanetRadiationBase::getEclipseCase() {
    static bool warned = false;
    if (this->m_eclipseCase != this->eclipseCase) {
        if (!warned) {
            this->bskLogger.bskLog(BSK_WARNING,
                "The flag eclipseCase will become a protected member after May 1st 2027. "
                "Use setEclipseCase()/getEclipseCase() instead.");
            warned = true;
            }
        this->m_eclipseCase = this->eclipseCase;
    }
    return this->m_eclipseCase;
}

void PlanetRadiationBase::setEclipseCase(bool value) {
    static bool warned = false;
    if (this->m_eclipseCase != this->eclipseCase) {
        if (!warned) {
            this->bskLogger.bskLog(BSK_WARNING,
                "The flag eclipseCase will become a protected member after May 1st 2027. "
                "Use setEclipseCase()/getEclipseCase() instead.");
            warned = true;
        }
    }
    this->eclipseCase = value;
    this->m_eclipseCase = value;
}
