

#ifndef BASILISK_SIMPLESOLARPANEL_H
#define BASILISK_SIMPLESOLARPANEL_H

#include <Eigen/Dense>
#include <vector>
#include "../_GeneralModuleFiles/simPowerNodeBase.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/eclipseSimMsg.h"


/*! \addtogroup SimModelGroup
 * @{
 */



/*! @brief Simple body-fixed solar panel model that considers shadowing from eclipse, body attitude, and panel parameters.

This module provides first-order modeling of power generation from an attitude and orbitally coupled solar panel. Specifically, it:

1. Evaluates the impact of shadowing using an assigned EclipseSimMsg;
2. Computes power generation using a cosine law based on the panel area, efficiency, and attitude
3. Allows for the panel body-fixed attitude nHat_B, the panel area, and the panel efficiency to be set via setPanelParameters.
4. Writes out a PowerNodeUsageSimMsg describing its power generation.

Power generation is computed according to \cite SMAD :
\f[
    W_{out} = W_{base} * C_{eclipse} * C_{panel} * (\hat{n}\cdot \hat{s}) A_{panel} 
\f]
where \f$W_{base} \f$ is the base power (in \f$\mbox{W}/\mbox{m}^2\f$) at the spacecraft location from the sun, \f$C_{eclipse}\f$ is the eclipse/penumbra mitigator on the sun's power (1 corresponds to no shadow, 0 corresponds to eclipse), \f$C_{panel}\f$ represents the 
panel's efficiency at converting solar energy into electrical energy, \f$(\hat{n}\cdot \hat{s})\f$ represents the alignment between the panel's normal vector and the spaceraft-sun unit vector, and \f$A_{panel}\f$ represents the panel area in meters squared.

\bibliography
*/


class SimpleSolarPanel: public PowerNodeBase {

public:
    SimpleSolarPanel();
    ~SimpleSolarPanel();
    void customCrossInit();
    bool customReadMessages();
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
    EclipseSimMsg sunVisibilityFactor;          //!< [-] scaling parameter from 0 (fully obscured) to 1 (fully visible)


};


#endif //BASILISK_SIMPLESOLARPANEL_H
