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
#ifndef WMM_MAGNETIC_FIELD_H
#define WMM_MAGNETIC_FIELD_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/environment/_GeneralModuleFiles/magneticFieldBase.h"
#include "architecture/utilities/astroConstants.h"
#include "GeomagnetismHeader.h"
#include "architecture/utilities/bskLogging.h"
#include <ctime>

/*! @brief magnetic field WMM class */
class MagneticFieldWMM:  public MagneticFieldBase {
public:
    MagneticFieldWMM();
    ~MagneticFieldWMM();

private:
    void evaluateMagneticFieldModel(MagneticFieldMsgPayload *msg, double currentTime);
    void initializeWmm();
    void cleanupEarthMagFieldModel();
    void computeWmmField(double decimalYear, MAGtype_CoordSpherical coord, double B_M[3]);
    void customReset(uint64_t CurrentClock);
    void customSetEpochFromVariable();
    void decimalYear2Gregorian(double fractionalYear, struct tm *gregorian);
    double gregorian2DecimalYear(double currentTime);

public:
    std::string dataPath;                   //!< -- String with the path to the WMM coefficient file
    double      epochDateFractionalYear;    //!< Specified epoch date as a fractional year
    BSKLogger bskLogger;                    //!< -- BSK Logging

private:
    MAGtype_MagneticModel *magneticModel;
    MAGtype_MagneticModel *timedMagneticModel;
    MAGtype_Ellipsoid      ellip;
    MAGtype_Geoid          geoid;
    MAGtype_Date           userDate;

    MAGtype_LegendreFunction* LegendreFunction;
    MAGtype_SphericalHarmonicVariables* SphVariables;

};


#endif /* WMM_MAGNETIC_FIELD_H */
