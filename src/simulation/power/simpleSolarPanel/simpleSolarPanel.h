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
