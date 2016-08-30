/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
/*
    Thrust Firing Schmitt
 
 */

/* modify the path to reflect the new module names */
#include "effectorInterfaces/thrFiringSchmitt/thrFiringSchmitt.h"

/* update this include to reflect the required module input messages */
#include "effectorInterfaces/errorConversion/vehEffectorOut.h"
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include <stdio.h>




/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */


/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->onTimeOutMsgID = CreateNewMessage(ConfigData->onTimeOutMsgName,
                                               sizeof(vehEffectorOut),
                                               "vehEffectorOut",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t moduleID)
{
	/*! - Get the input message ID's */
	ConfigData->thrForceInMsgID = subscribeToMessage(ConfigData->thrForceInMsgName,
														 sizeof(vehEffectorOut),
														 moduleID);
	ConfigData->thrConfInMsgID = subscribeToMessage(ConfigData->thrConfInMsgName,
												sizeof(ThrusterCluster),
												moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
	ThrusterCluster     localThrusterData;     /*!< local copy of the thruster data message */
	uint64_t            clockTime;
	uint32_t            readSize;
	int 				i;

	ConfigData->prevCallTime = 0;

	/* read in the support messages */
	ReadMessage(ConfigData->thrConfInMsgID, &clockTime, &readSize,
				sizeof(ThrusterCluster), &localThrusterData, moduleID);

	ConfigData->numThrusters = localThrusterData.numThrusters;

	for(i=0; i<ConfigData->numThrusters; i++) {
		ConfigData->maxThrust[i] = localThrusterData.thrusters[i].maxThrust;
		ConfigData->thrOnTimeOut.effectorRequest[i] = 0.0;
	}
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{

	uint64_t            clockTime;
	uint32_t            readSize;
	int i;
	double 				level;
	double				controlPeriod;
	double				onTime[MAX_EFF_CNT];

	if(ConfigData->prevCallTime == 0) {
		ConfigData->prevCallTime = callTime;

		for(i = 0; i < ConfigData->numThrusters; i++) {
			ConfigData->thrOnTimeOut.effectorRequest[i] = (double)(ConfigData->baseThrustState) * 2.0;
			ConfigData->thrOnTimeOut.effectorRequest[i] = 8.0;
		}

		WriteMessage(ConfigData->onTimeOutMsgID, callTime, sizeof(vehEffectorOut),   /* update module name */
					 (void*) &(ConfigData->thrOnTimeOut), moduleID);
		return;
	}

	controlPeriod = ((double)(callTime - ConfigData->prevCallTime)) * NANO2SEC;
	ConfigData->prevCallTime = callTime;

	/*! Begin method steps */
	/*! - Read the input messages */
	ReadMessage(ConfigData->thrForceInMsgID, &clockTime, &readSize,
				sizeof(vehEffectorOut), (void*) &(ConfigData->thrForceIn), moduleID);

	/*! Loop through thrusters */
	for(i = 0; i < ConfigData->numThrusters; i++) {

		/*! Correct for off-pulsing if necessary */
		if (ConfigData->baseThrustState == 1) {
			ConfigData->thrForceIn.effectorRequest[i] += ConfigData->maxThrust[i];
		}

		/*! Do not allow thrust requests less than zero */
		if (ConfigData->thrForceIn.effectorRequest[i] < 0.0) {
			ConfigData->thrForceIn.effectorRequest[i] = 0.0;
		}
		/*! Compute T_on from thrust request, max thrust, and control period */
		onTime[i] = ConfigData->thrForceIn.effectorRequest[i]/ConfigData->maxThrust[i]*controlPeriod;

		if (onTime[i] < ConfigData->thrMinFireTime) {
			/*! Request is less than minimum pulse time */
			level = onTime[i]/ConfigData->thrMinFireTime;
			if (level >= ConfigData->level_on) {
				ConfigData->thrustState[i] = BOOL_TRUE;
				onTime[i] = ConfigData->thrMinFireTime;
			} else if (level <= ConfigData->level_off) {
				ConfigData->thrustState[i] = BOOL_FALSE;
				onTime[i] = 0.0;
			} else if (ConfigData->thrustState[i] == BOOL_TRUE) {
				onTime[i] = ConfigData->thrMinFireTime;
			} else {
				onTime[i] = 0.0;
			}
		} else if (onTime[i] >= controlPeriod) {
			/*! Request is greater than control period */
			onTime[i] = 1.1*controlPeriod; // oversaturate to avoid numerical error
		}

		/*! Set the output data */
		ConfigData->thrOnTimeOut.effectorRequest[i] = onTime[i];
		ConfigData->thrOnTimeOut.effectorRequest[i] = 9.0;
		
	}

	WriteMessage(ConfigData->onTimeOutMsgID, callTime, sizeof(vehEffectorOut),   /* update module name */
				 (void*) &(ConfigData->thrOnTimeOut), moduleID);

	printf("completed Update_thrFiringSchmitt");

	return;

}
