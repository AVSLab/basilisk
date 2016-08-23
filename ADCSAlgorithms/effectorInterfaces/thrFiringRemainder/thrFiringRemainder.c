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
    Thrust Firing Remainder
 
 */

/* modify the path to reflect the new module names */
#include "effectorInterfaces/thrFiringRemainder/thrFiringRemainder.h"

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
void SelfInit_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(vehEffectorOut),
                                               "vehEffectorOut",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t moduleID)
{
	/*! - Get the input message ID's */
	ConfigData->inputMsgID = subscribeToMessage(ConfigData->inputDataName,
														 sizeof(vehEffectorOut),
														 moduleID);
	ConfigData->inputThrusterConfID = subscribeToMessage(ConfigData->inputThrusterConfName,
												sizeof(ThrusterCluster),
												moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
	ThrusterCluster     localThrusterData;     /*!< local copy of the thruster data message */
	uint64_t            clockTime;
	uint32_t            readSize;
	int 				i;

	ConfigData->prevCallTime = ~0;

	/* read in the support messages */
	ReadMessage(ConfigData->inputThrusterConfID, &clockTime, &readSize,
				sizeof(ThrusterCluster), &localThrusterData, moduleID);

	for(i=0; i<ConfigData->numThrusters; i++)
	{
		ConfigData->maxThrust[i] = localThrusterData.thrusters[i].maxThrust;
		ConfigData->pulseTimeMin[i] = localThrusterData.thrusters[i].pulseTimeMin;
	}
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
	uint64_t            clockTime;
	uint32_t            readSize;
	int i;

	if(ConfigData->prevCallTime == ~0) {
		ConfigData->prevCallTime = callTime;

		for(i = 0; i < ConfigData->numThrusters; i++) {
			ConfigData->thrFiringRemainderOut.effectorRequest[i] = (double)(ConfigData->baseThrustState[i]) * 2.0;
		}

		WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehEffectorOut),   /* update module name */
					 (void*) &(ConfigData->thrFiringRemainderOut), moduleID);
		return;
	}

	ConfigData->controlPeriod = ((double)(callTime - ConfigData->prevCallTime)) * NANO2SEC;
	ConfigData->prevCallTime = callTime;

	/*! Begin method steps */
	/*! - Read the input messages */
	ReadMessage(ConfigData->inputMsgID, &clockTime, &readSize,
				sizeof(vehEffectorOut), (void*) &(ConfigData->thrFiringRemainderIn), moduleID);

	/*! Loop through thrusters */
	for(i = 0; i < ConfigData->numThrusters; i++) {

		/*! Correct for off-pulsing if necessary */
		ConfigData->thrFiringRemainderIn.effectorRequest[i] += ConfigData->baseThrustState[i] == BOOL_TRUE ? ConfigData->maxThrust[i] : 0.0;

		/*! Do not allow thrust requests less than zero */
		ConfigData->thrFiringRemainderIn.effectorRequest[i] = ConfigData->thrFiringRemainderIn.effectorRequest[i] < 0.0 ? 0.0 : ConfigData->thrFiringRemainderIn.effectorRequest[i];

		/*! Compute T_on from thrust request, max thrust, and control period */
		ConfigData->onTime[i] = ConfigData->thrFiringRemainderIn.effectorRequest[i]/ConfigData->maxThrust[i]*ConfigData->controlPeriod;
		ConfigData->onTime[i] += ConfigData->pulseRemainder[i]*ConfigData->pulseTimeMin[i];
		ConfigData->pulseRemainder[i] = 0.0;

		/*! Pulse remainder logic */
		if(ConfigData->onTime[i] < ConfigData->pulseTimeMin[i]) {
			/*! Request is less than minimum pulse time */
			ConfigData->pulseRemainder[i] = ConfigData->onTime[i]/ConfigData->pulseTimeMin[i];
			ConfigData->onTime[i] = 0.0;
		} else if (ConfigData->onTime[i] >= ConfigData->controlPeriod) {
			/*! Request is greater than control period */
			ConfigData->pulseRemainder[i] = 0.0;
			ConfigData->onTime[i] = 1.1*ConfigData->controlPeriod; // oversaturate to avoid numerical error
		}

		/*! Set the output data */
		ConfigData->thrFiringRemainderOut.effectorRequest[i] = ConfigData->onTime[i];
		
	}

	WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehEffectorOut),   /* update module name */
				 (void*) &(ConfigData->thrFiringRemainderOut), moduleID);

	return;

}
