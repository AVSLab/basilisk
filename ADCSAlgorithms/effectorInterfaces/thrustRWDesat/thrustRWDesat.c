
#include "effectorInterfaces/thrustRWDesat/thrustRWDesat.h"
#include "effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include "attControl/_GeneralModuleFiles/vehControlOut.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the thruster-based RW desat module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the thruster desat
 @param moduleID The module ID associated with ConfigData
 */
void SelfInit_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Loop over number of thruster blocks and create output messages */
  
    ConfigData->outputThrID = CreateNewMessage(
        ConfigData->outputThrName, sizeof(vehEffectorOut),
        "vehEffectorOut", moduleID);
  
    
}

/*! This method performs the second stage of initialization for the thruster RW desat
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the RW desat logic
 @param moduleID The module ID associated with ConfigData
 */
void CrossInit_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputSpeedID = subscribeToMessage(ConfigData->inputSpeedName,
        sizeof(RWSpeedData), moduleID);
    
}

/*! This method takes in the current oberved reaction wheel angular velocities.
 @return void
 @param ConfigData The configuration data associated with the RW desat logic
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{

    uint64_t ClockTime;
    uint32_t ReadSize;
    uint32_t i;
	int32_t selectedThruster;
    RWSpeedData rwSpeeds;
	double observedSpeedVec[3];   /* The total summed speed of RWAs*/
	double singleSpeedVec[3];     /* The speed vector for a single wheel*/
	double bestMatch;
	double currentMatch;
	vehEffectorOut outputData;
    
	if ((callTime - ConfigData->previousFiring)*1.0E-9 < 
		ConfigData->thrFiringPeriod)
	{
		return;
	}

    /*! Begin method steps*/
    /*! - Read the input requested torque from the feedback controller*/
    ReadMessage(ConfigData->inputSpeedID, &ClockTime, &ReadSize,
                sizeof(RWSpeedData), (void*) &(rwSpeeds));
    
	v3SetZero(observedSpeedVec);
	for (i = 0; i < ConfigData->numRWAs; i++)
	{
		v3Scale(rwSpeeds.wheelSpeeds[i], &(ConfigData->rwAlignMap[i * 3]), 
			singleSpeedVec);
		v3Subtract(observedSpeedVec, singleSpeedVec, observedSpeedVec);
	}

	selectedThruster = -1;
	bestMatch = 0.0;
	for (i = 0; i < ConfigData->numThrusters; i++)
	{
		currentMatch = v3Dot(observedSpeedVec, 
			&(ConfigData->thrTorqueMap[i * 3]));
		currentMatch -= v3Dot(ConfigData->accumulatedImp,
			&(ConfigData->thrAlignMap[i * 3]));
		if (currentMatch > bestMatch)
		{
			selectedThruster = i;
			bestMatch = currentMatch;
		}
	}

	memset(&outputData, 0x0, sizeof(vehEffectorOut));
	if (bestMatch > 0.0)
	{
		outputData.effectorRequest[selectedThruster] = v3Dot(observedSpeedVec,
			&(ConfigData->thrTorqueMap[selectedThruster * 3]));
		outputData.effectorRequest[selectedThruster] =
			outputData.effectorRequest[selectedThruster] > ConfigData->maxFiring ?
			ConfigData->maxFiring : outputData.effectorRequest[selectedThruster];
		ConfigData->previousFiring = callTime;
		ConfigData->totalAccumFiring += outputData.effectorRequest[selectedThruster];
	}
 
	WriteMessage(ConfigData->outputThrID, callTime, sizeof(vehEffectorOut),
		&(outputData), moduleID);

    return;
}

/*! This method resets the ConfigData for the thruster-based RW desat module.
@return void
@param ConfigData The configuration data associated with the thruster desat
@param moduleID The module ID associated with ConfigData
*/
void Reset_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t moduleID)
{
	ConfigData->previousFiring = 0;
	v3SetZero(ConfigData->accumulatedImp);
	ConfigData->totalAccumFiring = 0.0;
}

