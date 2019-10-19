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
#include "power/_GeneralModuleFiles/powerNodeBase.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/eclipseSimMsg.h"

/*! \addtogroup SimModelGroup
 * @{
 */

/*! @brief Simple body-fixed solar panel model that considers shadowing from eclipse, body attitude, and panel parameters.

 ## Detailed Description
This module provides first-order modeling of power generation from an attitude and orbitally coupled solar panel. Specifically, it:

1. Evaluates the impact of shadowing using an assigned EclipseSimMsg;
2. Computes power generation using a cosine law based on the panel area, efficiency, and attitude
3. Allows for the panel body-fixed orientation nHat_B, the panel area, and the panel efficiency to be set via setPanelParameters.
4. Writes out a PowerNodeUsageSimMsg describing its power generation.

Power generation is computed according to \cite SMAD :
\f[
    W_{out} = W_{base} * C_{eclipse} * C_{panel} * (\hat{n}\cdot \hat{s}) A_{panel} 
\f]
where \f$W_{base} \f$ is the base power (in \f$\mbox{W}/\mbox{m}^2\f$) at the spacecraft location from the sun, \f$C_{eclipse}\f$ is the eclipse/penumbra mitigator on the sun's power (1 corresponds to no shadow, 0 corresponds to eclipse), \f$C_{panel}\f$ represents the 
panel's efficiency at converting solar energy into electrical energy, \f$(\hat{n}\cdot \hat{s})\f$ represents the alignment between the panel's normal vector and the spaceraft-sun unit vector, and \f$A_{panel}\f$ represents the panel area in meters squared.

For more information on how to set up and use this module, see the simple power system example: @ref scenarioSimplePowerDemo

 ## Message Connection Descriptions

 The following table lists additional module input messages beyond those specified in PowerNodeBase.

 Msg Variable Name | Msg Type | Description
 ------------------|----------|-------------
sunInMsgName | SpicePlanetStateSimMsg | Required input message. Describes sun position.
stateInMsgName |  SCPlusStatesSimMsg |  Required input message. Describes spacecraft position, attitude.
 sunEclipseInMsgName | EclipseSimMsg | Required input message. Describes shadow factor due to planetary bodies.

 */


class SimpleSolarPanel: public PowerNodeBase {

public:
    SimpleSolarPanel();
    ~SimpleSolarPanel();
    void customCrossInit();
    bool customReadMessages();
    void customReset(uint64_t CurrentClock);
    void setPanelParameters(Eigen::Vector3d nHat_B, double panelArea, double panelEfficiency);

private:
    void evaluatePowerModel(PowerNodeUsageSimMsg *powerUsageMsg);
    void computeSunData();
public:
    std::string sunInMsgName;                    //!< [-] Message name for sun data
    std::string stateInMsgName;                  //!< [-] Message name for spacecraft state 
    std::string cssDataOutMsgName;                  //!< [-] Message name for CSS output data 
    std::string sunEclipseInMsgName;            //!< [-] Message name for sun eclipse state message
    double panelArea;                           //!< [m^2] Panel area in meters squared.
    double panelEfficiency;                     //!< [W/W] Panel efficiency in converting solar energy to electrical energy.
    Eigen::Vector3d nHat_B;                     //!< [-] Panel normal unit vector relative to the spacecraft body frame.
    Eigen::Vector3d sHat_B;                     //!< [-] Sun direction unit vector relative to the spacecraft body frame.

private:
    double projectedArea;                        //!< [m^2] Area of the panel projected along the sun vector.
    double sunDistanceFactor;                   //!< [-] Scale factor on the base solar power computed using the true s/c-sun distance.
    int64_t sunInMsgID;                         //!< [-] Connect to input time message
    int64_t stateInMsgID;                       //!< [-] Connect to input time message
    int64_t sunEclipseInMsgID;                  //!< [-] Connect to input sun eclipse message
    SpicePlanetStateSimMsg sunData;            //!< [-] Unused for now, but including it for future
    SCPlusStatesSimMsg stateCurrent;           //!< [-] Current SSBI-relative state
    double shadowFactor;                        //!< [-] solar eclipse shadow factor from 0 (fully obscured) to 1 (fully visible)


};


#endif //BASILISK_SIMPLESOLARPANEL_H
