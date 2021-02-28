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


#ifndef EXPONENTIAL_ATMOSPHERE_H
#define EXPONENTIAL_ATMOSPHERE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "simulation/environment/_GeneralModuleFiles/atmosphereBase.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief exponential atmosphere model */
class ExponentialAtmosphere:  public AtmosphereBase {
public:
    ExponentialAtmosphere();
    ~ExponentialAtmosphere();

private:
    void evaluateAtmosphereModel(AtmoPropsMsgPayload *msg, double currentTime);


public:
    double baseDensity;             //!< [kg/m^3] Density at h=0
    double scaleHeight;             //!< [m] Exponential characteristic height
    double localTemp = 293.0;       //!< [K] Local atmospheric temperature; set to be constant.
    BSKLogger bskLogger;                      //!< -- BSK Logging
};


#endif /* EXPONENTIAL_ATMOSPHERE_H */
