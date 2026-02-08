/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway

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

#ifndef HASLAM_BACKGROUND_RADIATION_H
#define HASLAM_BACKGROUND_RADIATION_H

#include <vector>
#include <cmath>
#include "fitsio.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief J2000/ICRS to Galactic IAU (1976) rotation matrix
 *
 * Direction cosine matrix for transforming vectors from J2000/ICRS inertial
 * frame to Galactic coordinates. Based on https://doi.org/10.1051/0004-6361/201014961
 */
inline const Eigen::Matrix3d dcm_GN = (Eigen::Matrix3d() <<
    -0.054875539390, -0.873437104725, -0.483834991775,
    +0.494109453633, -0.444829594298, +0.746982248696,
    -0.867666135681, -0.198076389622, +0.455983794523).finished();

/*! @brief Singleton class for Haslam 408 MHz all-sky background radiation map
 *
 * Reads the Haslam 408 MHz all-sky map from a HEALPix FITS file and provides
 * brightness temperature lookups for given celestial coordinates.
 */
class HaslamMap {
public:
    /*! @brief Get singleton instance of HaslamMap
     *  @return Reference to the singleton instance
     */
    static HaslamMap& getInstance();

    /*! @brief Initialize the map by loading FITS data
     *  @return True if initialization successful, false otherwise
     */
    bool initialize();

    /*! @brief Beam-averaged brightness temperature at 408 MHz
     *  @param n_A_N      Antenna boresight direction in inertial {N} frame (normalized)
     *  @param beamRadius Circular beam half-angle [rad]
     *  @return           Average brightness temperature over beam area [K]
     */
    double getBrightnessTemperature(Eigen::Vector3d n_A_N, double beamRadius) const;

    /*! @brief Scale brightness temperature from 408 MHz to target frequency
     *  @param T_408         Temperature at 408 MHz [K]
     *  @param targetFreq_Hz Target frequency [Hz]
     *  @return              Scaled temperature [K]
     */
    double scaleToFrequency(double T_408, double targetFreq_Hz) const;

    /*! @brief Check if map data has been loaded
     *  @return True if initialized, false otherwise
     */
    bool isInitialized() const { return this->initialized; }

    /*! @brief Get HEALPix NSIDE parameter
     *  @return NSIDE value
     */
    long getNside() const { return this->nside; }

    /*! @brief Get total number of HEALPix pixels
     *  @return Number of pixels
     */
    long getNpix() const { return this->npix; }

    void configureBrightnessFile(const std::string& file);

    HaslamMap(const HaslamMap&)            = delete;  //!< Deleted copy constructor
    HaslamMap& operator=(const HaslamMap&) = delete;  //!< Deleted copy assignment

private:
    HaslamMap();            //!< Private constructor for singleton pattern
    ~HaslamMap() = default; //!< Default destructor

    /*! @brief Load Haslam map data from FITS file
     *  @return True if loading successful, false otherwise
     */
    bool loadHaslamMap();

    /*! @brief Convert spherical angles to HEALPix ring-scheme pixel index
     *  @param theta Colatitude angle [rad]
     *  @param phi   Longitude angle [rad]
     *  @return      Pixel index
     */
    long ang2pix_ring(double theta, double phi) const;

    /*! @brief Convert HEALPix pixel index to spherical angles
     *  @param ipix  Pixel index
     *  @param theta Output colatitude angle [rad]
     *  @param phi   Output longitude angle [rad]
     */
    void pix2ang_ring(long ipix, double& theta, double& phi) const;

    /*! @brief Query all pixels within a disc on the sphere
     *  @param theta0 Disc center colatitude [rad]
     *  @param phi0   Disc center longitude [rad]
     *  @param radius Disc radius [rad]
     *  @return       Vector of pixel indices within the disc
     */
    std::vector<long> query_disc(double theta0, double phi0, double radius) const;

    /*! @brief Get brightness temperature for a single pixel
     *  @param ipix Pixel index
     *  @return     Brightness temperature [K]
     */
    double getPixelTemperature(long ipix) const;

    BSKLogger          bskLogger;               //!< BSK Logging
    std::vector<float> brightnessTemperatures;  //!< [K] Brightness temperature data
    long               nside;                   //!< HEALPix NSIDE parameter
    long               npix;                    //!< Total number of pixels
    bool               initialized;             //!< Map loaded flag
    double             beamSize_arcmin;         //!< [arcmin] Original map beam size
    std::string        filepath;

    // Constants
    static constexpr double      spectralIndex       = -2.7;   //!< Galactic synchrotron spectral index
    static constexpr double      f_408               = 408e6;  //!< [Hz] Reference frequency
    static constexpr double      angularRes          = 56.0;   //!< [arcmin] Map angular resolution
    static constexpr int         HEALPIX_BASE_PIXELS = 12;     //!< Number of base-resolution HEALPix pixels
};

#endif // HASLAM_BACKGROUND_RADIATION_H
