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


#ifndef CENTERED_DIPOLE_MAGNETIC_FIELD_H
#define CENTERED_DIPOLE_MAGNETIC_FIELD_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "simulation/environment/_GeneralModuleFiles/magneticFieldBase.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief magnetic field centered dipole class */
class MagneticFieldCenteredDipole:  public MagneticFieldBase {
public:
    MagneticFieldCenteredDipole();
    ~MagneticFieldCenteredDipole();

private:
    void evaluateMagneticFieldModel(MagneticFieldMsgPayload *msg, double currentTime);


public:
    // More info on these IGRF parameters can be found on this [link](https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html)
    double g10;                 //!< [T] IGRF coefficient g_1^0
    double g11;                 //!< [T] IGRF coefficient g_1^1
    double h11;                 //!< [T] IGRF coefficient h_1^1

    BSKLogger bskLogger;                      //!< -- BSK Logging

};


#endif /* CENTERED_DIPOLE_MAGNETIC_FIELD_H */
