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

#pragma once

#include "_GeneralModuleFiles/sys_model.h"
#include "utilities/bskLogging.h"
#include <Eigen/Dense>

/*! @brief solar flux class */
class SolarFlux: public SysModel {
public:
    SolarFlux(){};
    ~SolarFlux(){};
    
    void SelfInit() override;
    void CrossInit() override;
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;
    void writeMessages(uint64_t CurrentSimNanos);
    void readMessages();

public:
    std::string sunPositionInMsgName = "sun_planet_data";               //!< msg name
    std::string spacecraftStateInMsgName = "inertial_state_output";     //!< msg name
    std::string solarFluxOutMsgName = "solar_flux";                     //!< msg name
    std::string eclipseInMsgName = "";                                  //!< msg name
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    double fluxAtSpacecraft;  //!< [W/m2]
    double eclipseFactor = 1.0;  //!< [] 1.0 is full sun, 0.0 is full eclipse
    Eigen::Vector3d r_SN_N;  //!< [m] sun position
    Eigen::Vector3d r_ScN_N;  //!< [m] s/c position
    int64_t sunPositionInMsgId = -1;                                    //!< msg ID
    int64_t spacecraftStateInMsgId = -1;                                //!< msg ID
    int64_t solarFluxOutMsgId = -1;                                     //!< msg ID
    int64_t eclipseInMsgId = -1;                                        //!< msg ID

};
