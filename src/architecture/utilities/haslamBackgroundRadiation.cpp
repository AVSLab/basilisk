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

#include "architecture/utilities/haslamBackgroundRadiation.h"
#include <algorithm>
#include <cstring>

HaslamMap::HaslamMap()
    : nside(0), npix(0), initialized(false), beamSize_arcmin(this->angularRes) {
}

HaslamMap& HaslamMap::getInstance() {
    static HaslamMap instance;
    return instance;
}

bool HaslamMap::initialize() {
    if (this->initialized) {
        this->bskLogger.bskLog(BSK_WARNING, "HaslamMap: Sky brightness map already loaded.");
        return true;
    }
    if (this->loadHaslamMap()) {
        this->initialized = true;
        return true;
    }
    return false;
}

bool HaslamMap::loadHaslamMap() {
    fitsfile* fptr = nullptr;
    int status = 0;

    // 1) Open FITS file FIRST
    if (fits_open_file(&fptr, this->filepath.c_str(), READONLY, &status)) {
        char err_text[256];
        fits_get_errstatus(status, err_text);
        this->bskLogger.bskLog(BSK_ERROR,
            ("HaslamMap: Error opening FITS file: " + std::string(err_text)).c_str());
        return false;
    }

    auto closeFile = [&]() {
        int tmp = 0;
        if (fptr) fits_close_file(fptr, &tmp);
        fptr = nullptr;
    };

    // 2) Now it is safe to read keys (but in the right HDU!)
    // COORDSYS might be in primary HDU or the map HDU depending on file.
    // Read it after moving to the intended HDU, OR try both.

    // Move to binary table extension (HDU 2)
    int hdutype = 0;
    if (fits_movabs_hdu(fptr, 2, &hdutype, &status) || hdutype != BINARY_TBL) {
        this->bskLogger.bskLog(BSK_ERROR, "HaslamMap: HDU 2 is not a binary table");
        closeFile();
        return false;
    }

    // Read COORDSYS (after moving to HDU 2)
    status = 0;
    char coordsys[20] = {0};
    if (fits_read_key(fptr, TSTRING, "COORDSYS", coordsys, nullptr, &status)) {
        this->bskLogger.bskLog(BSK_ERROR, "HaslamMap: Error reading COORDSYS");
        closeFile();
        return false;
    }
    if (strcmp(coordsys, "GALACTIC") != 0) {
        this->bskLogger.bskLog(BSK_ERROR, "HaslamMap: Expected GALACTIC coordinate system");
        closeFile();
        return false;
    }
    // Read HEALPix parameters
    long nside_header;
    char ordering[20], pixtype[20];

    status = 0;
    if (fits_read_key(fptr, TLONG, "NSIDE", &nside_header, nullptr, &status)) {
        this->bskLogger.bskLog(BSK_ERROR, "HaslamMap: Error reading NSIDE");
        closeFile();
        return false;
    }

    this->nside = nside_header;
    this->npix  = this->HEALPIX_BASE_PIXELS * this->nside * this->nside;

    // Verify PIXTYPE and ORDERING
    status = 0;
    fits_read_key(fptr, TSTRING, "PIXTYPE", pixtype, nullptr, &status);
    if (status || strcmp(pixtype, "HEALPIX") != 0) {
        this->bskLogger.bskLog(BSK_ERROR, "HaslamMap: FITS file is not HEALPix format");
        closeFile();
        return false;
    }


    status = 0;
    fits_read_key(fptr, TSTRING, "ORDERING", ordering, nullptr, &status);
    if (status || strcmp(ordering, "RING") != 0) {
        this->bskLogger.bskLog(BSK_ERROR, "HaslamMap: HEALPix map must be RING ordered");
        closeFile();
        return false;
    }


    status = 0;
    fits_read_key(fptr, TDOUBLE, "BEAMSIZE", &this->beamSize_arcmin, nullptr, &status);
    status = 0;  // BEAMSIZE is optional

    // Verify pixel count
    long nrows, values_per_row = 1024;
    fits_get_num_rows(fptr, &nrows, &status);
    if (status || nrows * values_per_row != this->npix) {
        this->bskLogger.bskLog(BSK_ERROR, "HaslamMap: Pixel count mismatch");
        closeFile();
        return false;
    }

    // Read temperature data
    this->brightnessTemperatures.resize(this->npix);
    float nullval = 0.0;
    int anynull;

    if (fits_read_col(fptr, TFLOAT, 1, 1, 1, this->npix, &nullval,
                      this->brightnessTemperatures.data(), &anynull, &status)) {
        char err_text[256];
        fits_get_errstatus(status, err_text);
        this->bskLogger.bskLog(BSK_ERROR, ("HaslamMap: Error reading data: " + std::string(err_text)).c_str());
        this->brightnessTemperatures.clear();
        closeFile();
        return false;
    }

    this->bskLogger.bskLog(BSK_INFORMATION,
        ("HaslamMap: Loaded NSIDE=" + std::to_string(this->nside) + ", NPIX=" + std::to_string(this->npix)).c_str());
    closeFile();
    return true;
}

/*! Method to configure the background temperature data file path
    @param file background temperature data file full path
*/
void HaslamMap::configureBrightnessFile(const std::string& file) {
    this->filepath = file;
}

long HaslamMap::ang2pix_ring(double theta, double phi) const {
    // Normalize angles
    while (phi < 0) phi += 2.0 * M_PI;
    while (phi >= 2.0 * M_PI) phi -= 2.0 * M_PI;
    theta = std::clamp(theta, 0.0, M_PI);

    double z  = std::cos(theta);
    double za = std::abs(z);
    double tt = phi / (0.5 * M_PI);

    long nl2  = 2 * this->nside;
    long nl4  = 4 * this->nside;
    long ncap = nl2 * (this->nside - 1);
    long ipix;

    if (za <= 2.0/3.0) {
        // Equatorial region
        long jp     = long(this->nside * (0.5 + tt - z * 0.75));
        long jm     = long(this->nside * (0.5 + tt + z * 0.75));
        long ir     = this->nside + 1 + jp - jm;
        long kshift = (ir % 2 == 0) ? 1 : 0;
        long ip     = ((jp + jm - this->nside + kshift + 1) / 2) + 1;
        if (ip > nl4) ip -= nl4;
        ipix = ncap + nl4 * (ir - this->nside) + ip - 1;
    } else {
        // Polar caps
        double tp  = tt - std::floor(tt);
        double tmp = std::sqrt(3.0 * (1.0 - za));
        long jp    = std::min(this->nside - 1, long(this->nside * tp * tmp));
        long jm    = std::min(this->nside - 1, long(this->nside * (1.0 - tp) * tmp));
        long ir    = jp + jm + 1;
        long ip    = long(tt * ir) + 1;
        if (ip > 4 * ir) ip -= 4 * ir;

        if (z > 0) {
            ipix = 2 * ir * (ir - 1) + ip - 1;
        } else {
            ipix = this->npix - 2 * ir * (ir + 1) + ip - 1;
        }
    }
    return ipix;
}

void HaslamMap::pix2ang_ring(long ipix, double& theta, double& phi) const {
    long nl2  = 2 * this->nside;
    long nl4  = 4 * this->nside;
    long ncap = nl2 * (this->nside - 1);

    if (ipix < ncap) {
        // North polar cap
        long iring = long(0.5 * (1 + std::sqrt(1 + 2 * ipix)));
        long iphi  = ipix + 1 - 2 * iring * (iring - 1);
        theta      = std::acos(1.0 - iring * iring / (3.0 * this->nside * this->nside));
        phi        = (iphi - 0.5) * M_PI / (2.0 * iring);
    } else if (ipix < this->npix - ncap) {
        // Equatorial region
        long ip     = ipix - ncap;
        long iring  = ip / nl4 + this->nside;
        long iphi   = ip % nl4 + 1;
        double fodd = ((iring + this->nside) % 2) ? 1.0 : 0.5;
        theta       = std::acos((nl2 - iring) / (1.5 * this->nside));
        phi         = (iphi - fodd) * M_PI / nl2;
    } else {
        // South polar cap
        long ip    = this->npix - ipix;
        long iring = long(0.5 * (1 + std::sqrt(2 * ip - 1)));
        long iphi  = 4 * iring + 1 - (ip - 2 * iring * (iring - 1));
        theta      = std::acos(-1.0 + iring * iring / (3.0 * this->nside * this->nside));
        phi        = (iphi - 0.5) * M_PI / (2.0 * iring);
    }
}

std::vector<long> HaslamMap::query_disc(double theta0, double phi0, double radius) const {
    std::vector<long> pixels;

    // Center direction vector
    double          st0    = std::sin(theta0), ct0 = std::cos(theta0);
    double          sp0    = std::sin(phi0),   cp0 = std::cos(phi0);
    Eigen::Vector3d v0(st0 * cp0, st0 * sp0, ct0);
    double          cosrad = std::cos(radius);

    // Check all pixels
    for (long ipix = 0; ipix < this->npix; ++ipix) {
        double theta, phi;
        this->pix2ang_ring(ipix, theta, phi);
        double          st = std::sin(theta), ct = std::cos(theta);
        double          sp = std::sin(phi),   cp = std::cos(phi);
        Eigen::Vector3d v(st * cp, st * sp, ct);

        if (v.dot(v0) >= cosrad) {
            pixels.push_back(ipix);
        }
    }
    return pixels;
}

double HaslamMap::getPixelTemperature(long ipix) const {
    if (ipix < 0 || ipix >= this->npix) return 0.0;
    float val = this->brightnessTemperatures[ipix];
    if (val < -1.0e30) return 0.0;  // BAD_DATA sentinel
    return val;
}

double HaslamMap::scaleToFrequency(double T_408, double targetFreq_Hz) const {
    // T_target = T_408 * (targetFreq / 408 MHz)^spectralIndex
    return T_408 * std::pow(targetFreq_Hz / this->f_408, this->spectralIndex);
}

double HaslamMap::getBrightnessTemperature(Eigen::Vector3d n_A_N, double beamRadius) const {
    if (!this->initialized) {
        return 0.0;
    }
    // Transform boresight to Galactic coordinates
    Eigen::Vector3d boresight_G = dcm_GN * n_A_N.normalized();

    // Convert to spherical coordinates
    double theta0 = std::acos(boresight_G.z());
    double phi0   = std::atan2(boresight_G.y(), boresight_G.x());
    if (phi0 < 0) {
        phi0 += 2.0 * M_PI;
    }

    // Query pixels within beam
    std::vector<long> pixels = this->query_disc(theta0, phi0, beamRadius);

    // Fallback to center pixel if no pixels found
    if (pixels.empty()) {
        return this->getPixelTemperature(this->ang2pix_ring(theta0, phi0));
    }

    // Compute uniform average
    double sum = 0.0;
    for (long ipix : pixels) {
        sum += this->getPixelTemperature(ipix);
    }
    return sum / static_cast<double>(pixels.size());
}
