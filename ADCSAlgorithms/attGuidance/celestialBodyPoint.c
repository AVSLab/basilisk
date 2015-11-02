
#include "attGuidance/celestialBodyPoint.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "SimCode/environment/spice/spice_planet_state.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "attDetermination/CSSEst/navStateOut.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the nominal delta-V maneuver guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the celestial body guidance
 */
void SelfInit_celestialBodyPoint(celestialBodyPointConfig *ConfigData)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(
        ConfigData->outputDataName, sizeof(attCmdOut), "attCmdOut");
    return;
    
}

/*! This method performs the second stage of initialization for the celestial body
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the attitude maneuver guidance
 */
void CrossInit_celestialBodyPoint(celestialBodyPointConfig *ConfigData)
{
    ConfigData->inputCelID = FindMessageID(ConfigData->inputCelMessName);
    ConfigData->inputNavID = FindMessageID(ConfigData->inputNavDataName);
    ConfigData->inputSecID = -1;
    if(strlen(ConfigData->inputSecMessName) > 0)
    {
        ConfigData->inputSecID = FindMessageID(ConfigData->inputSecMessName);
    }
    return;
    
}

/*! This method takes the spacecraft and points a specified axis at a named 
    celestial body specified in the configuration data.  It generates the 
    commanded attitude and assumes that the control errors are computed 
    downstream.
 @return void
 @param ConfigData The configuration data associated with the celestial body guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_celestialBodyPoint(celestialBodyPointConfig *ConfigData,
    uint64_t callTime)
{
    uint64_t writeTime;
    uint32_t writeSize;
    NavStateOut navData;
    SpicePlanetState primPlanet;
    SpicePlanetState secPlanet;
    double secPointVector[3];
    double primPointVector[3];
	double relVelVector[3];
	double relPosVector[3];
    double T_Inrtl2Point[3][3];
    double T_Inrtl2Bdy[3][3];
    
    ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize,
                sizeof(IMUOutputData), &navData);
    ReadMessage(ConfigData->inputCelID, &writeTime, &writeSize,
                sizeof(SpicePlanetState), &primPlanet);
    if(ConfigData->inputSecID >= 0)
    {
        ReadMessage(ConfigData->inputSecID, &writeTime, &writeSize,
            sizeof(SpicePlanetState), &secPlanet);
        v3Subtract(secPlanet.PositionVector, navData.vehPosition,
                   secPointVector);
        v3Normalize(secPointVector, secPointVector);
    }
    else
    {
		v3Subtract(navData.vehPosition, primPlanet.PositionVector, relPosVector);
		v3Subtract(navData.vehVelocity, primPlanet.VelocityVector, relVelVector);
        v3Cross(relPosVector, relVelVector, secPointVector);
        v3Normalize(secPointVector, secPointVector);
    }
    v3Subtract(primPlanet.PositionVector, navData.vehPosition, primPointVector);
    v3Normalize(primPointVector, primPointVector);
    v3Copy(primPointVector, &(T_Inrtl2Point[0][0]));
    v3Cross(primPointVector, secPointVector, &(T_Inrtl2Point[2][0]));
    v3Normalize(&(T_Inrtl2Point[2][0]), &(T_Inrtl2Point[2][0]));
    v3Cross(&(T_Inrtl2Point[2][0]), &(T_Inrtl2Point[0][0]),
            &(T_Inrtl2Point[1][0]));
    m33MultM33(ConfigData->TPoint2Bdy, T_Inrtl2Point, T_Inrtl2Bdy);
    C2MRP(&(T_Inrtl2Bdy[0][0]), ConfigData->attCmd.sigma_BR);
    v3SetZero(ConfigData->attCmd.omega_BR);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attCmdOut),
        &(ConfigData->attCmd));
    
    return;
}

