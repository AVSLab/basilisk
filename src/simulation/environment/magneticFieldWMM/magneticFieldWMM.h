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
#include "../../_GeneralModuleFiles/sys_model.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/magneticFieldSimMsg.h"
#include "simMessages/epochSimMsg.h"
#include "../_GeneralModuleFiles/magneticFieldBase.h"
#include "GeomagnetismHeader.h"

/*! \addtogroup SimModelGroup
 * @{
 */

/*! @brief Evaluate a magnetic field model about Earth for a range of spacecraft locations using the World Magnetic Model (WMM).
 
    For more information on this module see this [PDF Documentation](Basilisk-magFieldWMM-20190618.pdf).
 */
class MagneticFieldWMM:  public MagneticFieldBase {
public:
    MagneticFieldWMM();
    ~MagneticFieldWMM();

private:
    void evaluateMagneticFieldModel(MagneticFieldSimMsg *msg, double currentTime);
    void initializeWmm(const char *dataPath);
    void cleanupEarthMagFieldModel();
    void computeWmmField(double decimalYear, double phi, double lambda, double h, double B_M[3]);
    void customReset(uint64_t CurrentClock);
    void customCrossInit();

public:
    std::string epochInMsgName;             //!< -- Message name of the epoch message
    std::string dataPath;                   //!< -- String with the path to the WMM coefficient file
    double      epochDateFractionalYear;    //!< Specified epoch date as a fractional year


private:
    MAGtype_MagneticModel * magneticModels[1];
    MAGtype_MagneticModel *timedMagneticModel;
    MAGtype_Ellipsoid      ellip;
    MAGtype_Geoid          geoid;
    MAGtype_Date           userDate;
};

/*! @} */

#endif /* WMM_MAGNETIC_FIELD_H */
