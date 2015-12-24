
#include "attGuidance/celestialBodyPoint/celestialBodyPoint.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "SimCode/environment/spice/spice_planet_state.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "attDetermination/CSSEst/navStateOut.h"
#include "vehicleConfigData/ADCSAlgorithmMacros.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the nominal delta-V maneuver guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the celestial body guidance
 */
void SelfInit_celestialBodyPoint(celestialBodyPointConfig *ConfigData,
    uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(
        ConfigData->outputDataName, sizeof(attCmdOut), "attCmdOut", moduleID);
    return;
    
}

/*! This method performs the second stage of initialization for the celestial body
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the attitude maneuver guidance
 */
void CrossInit_celestialBodyPoint(celestialBodyPointConfig *ConfigData,
    uint64_t moduleID)
{
    ConfigData->inputCelID = subscribeToMessage(ConfigData->inputCelMessName,
        sizeof(SpicePlanetState), moduleID);
    ConfigData->inputNavID = subscribeToMessage(ConfigData->inputNavDataName,
        sizeof(NavStateOut), moduleID);
    ConfigData->inputSecID = -1;
    if(strlen(ConfigData->inputSecMessName) > 0)
    {
        ConfigData->inputSecID = subscribeToMessage(ConfigData->inputSecMessName,
            sizeof(SpicePlanetState), moduleID);
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
    uint64_t callTime, uint64_t moduleID)
{
    uint64_t writeTime;
    uint32_t writeSize;
    NavStateOut navData;
    SpicePlanetState primPlanet;
    SpicePlanetState secPlanet;
	double platAngDiff;
    double secPointVector[3];
    double secVelVector[3];
    double primPointVector[3];
    double primVelVector[3];
	double primHatPointVector[3];
	double relVelVector[3];
	double relPosVector[3];
    double T_Inrtl2Point[3][3];
    double T_Inrtl2Bdy[3][3];
    double omegVecComp1[3], omegVecComp2[3];
    double totalSecDeriv[3];
    double b3CrossProd[3];
    double cDotMatrix[3][3];
    double omegVectrix[3][3];
    double vecMag, dotProd;
    double omegPointN_Point[3];
	uint32_t noValidConstraint;
    
    ReadMessage(ConfigData->inputNavID, &writeTime, &writeSize,
                sizeof(NavStateOut), &navData);
    ReadMessage(ConfigData->inputCelID, &writeTime, &writeSize,
                sizeof(SpicePlanetState), &primPlanet);
	v3Subtract(primPlanet.PositionVector, navData.r_BN_N, primPointVector);
	v3Subtract(primPlanet.VelocityVector, navData.v_BN_N, primVelVector);
	v3Normalize(primPointVector, primHatPointVector);
	noValidConstraint = 0;
    if(ConfigData->inputSecID >= 0)
    {
        ReadMessage(ConfigData->inputSecID, &writeTime, &writeSize,
            sizeof(SpicePlanetState), &secPlanet);
        v3Subtract(secPlanet.PositionVector, navData.r_BN_N,
                   secPointVector);
        v3Subtract(secPlanet.VelocityVector, navData.v_BN_N, secVelVector);
        vecMag = v3Norm(secPointVector);
        dotProd = v3Dot(secPointVector, secVelVector);
        v3Scale(1.0/vecMag, secVelVector, omegVecComp1);
        v3Scale(-dotProd/(vecMag*vecMag*vecMag), secPointVector, omegVecComp2);
        v3Add(omegVecComp1, omegVecComp2, totalSecDeriv);
        v3Normalize(secPointVector, secPointVector);
		platAngDiff = acos(v3Dot(secPointVector, primHatPointVector));
		if (fabs(platAngDiff) < ConfigData->singularityThresh && ConfigData->prevAvail == 1)
		{
			v3Copy(ConfigData->prevConstraintAxis, secPointVector);
			v3SetZero(secVelVector);
			v3SetZero(totalSecDeriv);
		}
		else if (fabs(platAngDiff) < ConfigData->singularityThresh && ConfigData->prevAvail != 1)
		{
			noValidConstraint = 1;
		}
    }
    
	if(ConfigData->inputSecID < 0 || noValidConstraint == 1)
    {
		v3Subtract(navData.r_BN_N, primPlanet.PositionVector, relPosVector);
		v3Subtract(navData.v_BN_N, primPlanet.VelocityVector, relVelVector);
        v3Cross(relPosVector, relVelVector, secPointVector);
        v3SetZero(totalSecDeriv);
        v3Normalize(secPointVector, secPointVector);
    }
    vecMag = v3Norm(primPointVector);
    dotProd = v3Dot(primPointVector, primVelVector);
    v3Scale(1.0/vecMag, primVelVector, omegVecComp1);
    v3Scale(-dotProd/(vecMag*vecMag*vecMag), primPointVector, omegVecComp2);
    v3Add(omegVecComp1, omegVecComp2, &(cDotMatrix[0][0]));
    v3Normalize(primPointVector, primPointVector);
    v3Copy(primPointVector, &(T_Inrtl2Point[0][0]));
    v3Cross(primPointVector, secPointVector, b3CrossProd);
    vecMag = v3Norm(b3CrossProd);
    v3Cross(&(cDotMatrix[0][0]), secPointVector, omegVecComp1);
    v3Cross(primPointVector, totalSecDeriv, omegVecComp2);
    v3Add(omegVecComp1, omegVecComp2, omegVecComp1);
    v3Scale(1.0/vecMag, omegVecComp1, omegVecComp1);
    dotProd = v3Dot(omegVecComp1, b3CrossProd);
    vecMag = -1.0/vecMag*1.0/vecMag*dotProd;
    v3Scale(vecMag, b3CrossProd, omegVecComp2);
    v3Add(omegVecComp1, omegVecComp2, &(cDotMatrix[2][0]));
    
    v3Normalize(b3CrossProd, &(T_Inrtl2Point[2][0]));
    v3Cross(&(T_Inrtl2Point[2][0]), &(T_Inrtl2Point[0][0]),
            &(T_Inrtl2Point[1][0]));
	v3Copy(&(T_Inrtl2Point[2][0]), ConfigData->prevConstraintAxis);
	ConfigData->prevAvail = 1;
    v3Cross(&(cDotMatrix[2][0]), primPointVector, omegVecComp1);
    v3Cross(&(T_Inrtl2Point[2][0]), &(cDotMatrix[0][0]), omegVecComp2);
    
    v3Add(omegVecComp1, omegVecComp2, &(cDotMatrix[1][0]));
    m33MultM33t(RECAST3X3 cDotMatrix, T_Inrtl2Point, omegVectrix);
    
    omegPointN_Point[0] = -(-omegVectrix[1][2] + omegVectrix[2][1])/2.0;
    omegPointN_Point[1] = -(omegVectrix[0][2] - omegVectrix[2][0])/2.0;
    omegPointN_Point[2] = -(-omegVectrix[0][1] + omegVectrix[1][0])/2.0;
    m33MultM33(RECAST3X3 ConfigData->TPoint2Bdy, T_Inrtl2Point, T_Inrtl2Bdy);
    C2MRP(RECAST3X3 &(T_Inrtl2Bdy[0][0]), ConfigData->attCmd.sigma_BR);
    m33MultV3(RECAST3X3 ConfigData->TPoint2Bdy, omegPointN_Point,
        ConfigData->attCmd.omega_BR);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attCmdOut),
        &(ConfigData->attCmd), moduleID);
    
    return;
}

