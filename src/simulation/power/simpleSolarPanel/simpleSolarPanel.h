//
// Created by andrew on 7/12/19.
//

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



//! @brief Simple body-fixed solar panel model that considers shadowing from eclipse, body attitude, and panel parameters.

/*! This module is intended to serve as a basic power node with a constant power load or draw. Specifically, it:

1. Writes out a PowerNodeUsageSimMsg describing its power generation;
2. Evaluates the impact of shadowing using an assigned EclipseSimMsg;
3. Computes power generation using a cosine law based on the panel area, efficiency, and attitude
4. Allows for the panel body-fixed attitude nHat_B, the panel area, and the panel efficiency to be set via setPanelParameters.

Core functionality is wrapped in the evaluatePowerModel protected virtual void method, which uses a simple cosine law to compute the projected panel area and multiplies that by the panel efficiency and sunVisibilityFactor to arrive at wattage.
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
    std::string stateInMsgName;                  //!< [-] Message name for spacecraft state */
    std::string cssDataOutMsgName;                  //!< [-] Message name for CSS output data */
    std::string sunEclipseInMsgName;            //!< [-] Message name for sun eclipse state message
    double panelArea;
    double panelEfficiency;
    Eigen::Vector3d nHat_B;
    Eigen::Vector3d sHat_B;

private:
    double projectedArea;
    double sunDistanceFactor;
    int64_t sunInMsgID;                         //!< [-] Connect to input time message
    int64_t stateInMsgID;                       //!< [-] Connect to input time message
    int64_t sunEclipseInMsgID;                  //!< [-] Connect to input sun eclipse message
    SpicePlanetStateSimMsg sunData;            //!< [-] Unused for now, but including it for future
    SCPlusStatesSimMsg stateCurrent;           //!< [-] Current SSBI-relative state
    EclipseSimMsg sunVisibilityFactor;          //!< [-] scaling parameter from 0 (fully obscured) to 1 (fully visible)


};


#endif //BASILISK_SIMPLESOLARPANEL_H
