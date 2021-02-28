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

#ifndef BASILISK_SIMPLESOLARPANEL_H
#define BASILISK_SIMPLESOLARPANEL_H

#include <Eigen/Dense>
#include <vector>
#include "simulation/power/_GeneralModuleFiles/powerNodeBase.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"

#include "architecture/utilities/bskLogging.h"



/*! @brief simple solar panel class */
class SimpleSolarPanel: public PowerNodeBase {

public:
    SimpleSolarPanel();
    ~SimpleSolarPanel();
    bool customReadMessages();
    void customReset(uint64_t CurrentClock);
    void setPanelParameters(Eigen::Vector3d nHat_B, double panelArea, double panelEfficiency);

private:
    void evaluatePowerModel(PowerNodeUsageMsgPayload *powerUsageMsg);
    void computeSunData();
public:
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;   //!< [-] sun data input message
    ReadFunctor<SCStatesMsgPayload> stateInMsg;     //!< [-] spacecraft state input message
    ReadFunctor<EclipseMsgPayload> sunEclipseInMsg;     //!< [-] Messun eclipse state input message
    double panelArea;                           //!< [m^2] Panel area in meters squared.
    double panelEfficiency;                     //!< [W/W] Panel efficiency in converting solar energy to electrical energy.
    Eigen::Vector3d nHat_B;                     //!< [-] Panel normal unit vector relative to the spacecraft body frame.
    BSKLogger bskLogger;                          //!< -- BSK Logging

private:
    double projectedArea;                        //!< [m^2] Area of the panel projected along the sun vector.
    double sunDistanceFactor;                   //!< [-] Scale factor on the base solar power computed using the true s/c-sun distance.
    SpicePlanetStateMsgPayload sunData;            //!< [-] Unused for now, but including it for future
    SCStatesMsgPayload stateCurrent;           //!< [-] Current SSBI-relative state
    double shadowFactor;                        //!< [-] solar eclipse shadow factor from 0 (fully obscured) to 1 (fully visible)


};


#endif //BASILISK_SIMPLESOLARPANEL_H
