//
// Created by andrew on 7/12/19.
//
#include <math.h>
#include <iostream>
#include <cstring>
#include <algorithm>
#include "simpleSolarPanel.h"
#include "../../simMessages/powerNodeUsageSimMsg.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "utilities/avsEigenSupport.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/avsEigenMRP.h"
#include "utilities/bsk_Print.h"

SimpleSolarPanel::SimpleSolarPanel(){

    this->nodePowerOut = 0.0;
    this->sunInMsgID = -1;
    this->stateInMsgID = -1;
    this->sunEclipseInMsgID = -1;

    this->sunInMsgName = "";
    this->stateInMsgName = "";
    this->sunEclipseInMsgName = "";

    return;

}

SimpleSolarPanel::~SimpleSolarPanel(){

    return;
}

void SimpleSolarPanel::customCrossInit(){
//! - If we have a valid sun msg ID, subscribe to it
    if(this->sunInMsgName.length() > 0)
    {
        this->sunInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->sunInMsgName,
                                                           sizeof(SpicePlanetStateSimMsg),
                                                           moduleID);
    }
    //! - If we have a state in msg name, subscribe to it
    if(this->stateInMsgName.length() > 0)
    {
        this->stateInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->stateInMsgName,
                                                                              sizeof(SCPlusStatesSimMsg),
                                                                              moduleID);
    }
    if(this->sunEclipseInMsgName.length() > 0) {
        this->sunEclipseInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->sunEclipseInMsgName,
                                                                              sizeof(EclipseSimMsg),
                                                                              moduleID);
    }

}

bool SimpleSolarPanel::customReadMessages()
{
    SingleMessageHeader localHeader;

    //! - Zero ephemeris information
    memset(&this->sunData, 0x0, sizeof(SpicePlanetStateSimMsg));
    memset(&this->stateCurrent, 0x0, sizeof(SCPlusStatesSimMsg));
    //! - If we have a valid sun ID, read Sun ephemeris message
    if(this->sunInMsgID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->sunInMsgID, &localHeader,
                                                    sizeof(SpicePlanetStateSimMsg),
                                                    reinterpret_cast<uint8_t*> (&this->sunData),
                                                    this->moduleID);
    }
    //! - If we have a valid state ID, read vehicle state ephemeris message
    if(this->stateInMsgID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(this->stateInMsgID, &localHeader,
                                                    sizeof(SCPlusStatesSimMsg),
                                                    reinterpret_cast<uint8_t*> (&this->stateCurrent),
                                                    this->moduleID);
    }
    if(this->sunEclipseInMsgID >= 0) {
        SystemMessaging::GetInstance()->ReadMessage(this->sunEclipseInMsgID, &localHeader,
                                                    sizeof(EclipseSimMsg),
                                                    reinterpret_cast<uint8_t*> (&this->sunVisibilityFactor),
                                                    this->moduleID);
    }
    return(true);
}

/*! This method allows for solar panel parameters to be set.
 @param nHat_B: The normal vector of the solar panel expressed in the spacecraft body frame.
 @param panelArea: the spacecraft panel area in meters squared.
 @param panelEfficiency: The efficiency of the solar panel in nondimensional units; typically ranges from 0.1 to 0.5.
 @return void
 */
void SimpleSolarPanel::setPanelParameters(Eigen::Vector3d nHat_B, double panelArea, double panelEfficiency)
{
    this->nHat_B = nHat_B;
    this->panelArea = panelArea;
    this->panelEfficiency = panelEfficiency;
    return;
}

/*! This method computes the spacecraft-sun vector, the solar panel's projected area, and the sunDistanceFactor based on the magnitude of the spacecraft sun vector.
 @return void
 */
void SimpleSolarPanel::computeSunData()
{
    Eigen::Vector3d Sc2Sun_Inrtl;
    Eigen::Vector3d sHat_N;
    Eigen::Matrix3d dcm_BN;

    Eigen::Vector3d r_BN_N_eigen;
    Eigen::Vector3d sunPos;
    Eigen::MRPd sigma_BN_eigen;

    //! - Get the position from spacecraft to Sun

    //! - Read Message data to eigen
    r_BN_N_eigen = cArray2EigenVector3d(this->stateCurrent.r_BN_N);
    sunPos = cArray2EigenVector3d(this->sunData.PositionVector);
    sigma_BN_eigen = cArray2EigenVector3d(this->stateCurrent.sigma_BN);


    //! - Find sun heading unit vector
    Sc2Sun_Inrtl = sunPos -  r_BN_N_eigen;
    sHat_N = Sc2Sun_Inrtl / Sc2Sun_Inrtl.norm();

    //! - Get the inertial to body frame transformation information and convert sHat to body frame
    dcm_BN = sigma_BN_eigen.toRotationMatrix().transpose();
    this->sHat_B = dcm_BN * sHat_N;

    //! - Compute the panel projected area
    this->projectedArea = this->panelArea * -(this->sHat_B.dot(nHat_B));
    if(this->projectedArea<0){
        this->projectedArea = 0;
    }

    //! - compute sun distance factor
    double r_Sun_Sc = Sc2Sun_Inrtl.norm();
    this->sunDistanceFactor = pow(AU*1000., 2.)/pow(r_Sun_Sc, 2.);
}

/*! This method computes the power generated by the solar panel and stores it in a PowerNodeUsageSimMsg. This is evaluated in two steps:
1. Compute the amount of power available at the spacecraft position using the SOLAR_FLUX_EARTH macro, distance factor, and eclipse shadow factor;
2. Compute how much of that power is captured and converted to power using the projectedArea and panelEfficiency attributes.
 @return void
 */
void SimpleSolarPanel::evaluatePowerModel(PowerNodeUsageSimMsg *powerUsageSimMsg) {

    this->computeSunData();
    // SunPowerFactor = baseSunPower * distanceFactor * illuminationFactor 

    double sunPowerFactor = SOLAR_FLUX_EARTH * this->sunDistanceFactor * this->sunVisibilityFactor.shadowFactor;
    powerUsageSimMsg->netPower_W = sunPowerFactor * this->projectedArea * this->panelEfficiency;

    return;
}