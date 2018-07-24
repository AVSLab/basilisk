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
/*
    Thrust Firing Remainder
 
 */

/* modify the path to reflect the new module names */
#include "effectorInterfaces/thrFiringRemainder/thrFiringRemainder.h"

/* update this include to reflect the required module input messages */
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
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
    ConfigData->onTimeOutMsgID = CreateNewMessage(ConfigData->onTimeOutMsgName,
                                               sizeof(THRArrayOnTimeCmdIntMsg),
                                               "THRArrayOnTimeCmdIntMsg",          /* add the output structure name */
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
	ConfigData->thrForceInMsgID = subscribeToMessage(ConfigData->thrForceInMsgName,
														 sizeof(THRArrayCmdForceFswMsg),
														 moduleID);
	ConfigData->thrConfInMsgID = subscribeToMessage(ConfigData->thrConfInMsgName,
												sizeof(THRArrayConfigFswMsg),
												moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrFiringRemainder(thrFiringRemainderConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
	THRArrayConfigFswMsg   localThrusterData;     /*!< local copy of the thruster data message */
	uint64_t            clockTime;
	uint32_t            readSize;
	int 				i;

	ConfigData->prevCallTime = 0;

	/* read in the support messages */
	ReadMessage(ConfigData->thrConfInMsgID, &clockTime, &readSize,
				sizeof(THRArrayConfigFswMsg), &localThrusterData, moduleID);

	ConfigData->numThrusters = localThrusterData.numThrusters;

	for(i=0; i<ConfigData->numThrusters; i++) {
		ConfigData->maxThrust[i] = localThrusterData.thrusters[i].maxThrust;
		ConfigData->pulseRemainder[i] = 0.0;
		ConfigData->thrOnTimeOut.OnTimeRequest[i] = 0.0;
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
	int 				i;
	double				controlPeriod;			/*!< [s] control period */
	double				onTime[MAX_EFF_CNT];	/*!< [s] array of commanded on time for thrusters */

	if(ConfigData->prevCallTime == 0) {
		ConfigData->prevCallTime = callTime;

		for(i = 0; i < ConfigData->numThrusters; i++) {
			ConfigData->thrOnTimeOut.OnTimeRequest[i] = (double)(ConfigData->baseThrustState) * 2.0;
		}

		WriteMessage(ConfigData->onTimeOutMsgID, callTime, sizeof(THRArrayOnTimeCmdIntMsg),   /* update module name */
					 (void*) &(ConfigData->thrOnTimeOut), moduleID);
		return;
	}

	controlPeriod = ((double)(callTime - ConfigData->prevCallTime)) * NANO2SEC;
	ConfigData->prevCallTime = callTime;

	/*! Begin method steps */
	/*! - Read the input messages */
	ReadMessage(ConfigData->thrForceInMsgID, &clockTime, &readSize,
				sizeof(THRArrayCmdForceFswMsg), (void*) &(ConfigData->thrForceIn), moduleID);

	/*! Loop through thrusters */
	for(i = 0; i < ConfigData->numThrusters; i++) {

		/*! Correct for off-pulsing if necessary */
		if (ConfigData->baseThrustState == 1) {
			ConfigData->thrForceIn.thrForce[i] += ConfigData->maxThrust[i];
		}

		/*! Do not allow thrust requests less than zero */
		if (ConfigData->thrForceIn.thrForce[i] < 0.0) {
			ConfigData->thrForceIn.thrForce[i] = 0.0;
		}

		/*! Compute T_on from thrust request, max thrust, and control period */
		onTime[i] = ConfigData->thrForceIn.thrForce[i]/ConfigData->maxThrust[i]*controlPeriod;
		/*! Add in remainder */
		onTime[i] += ConfigData->pulseRemainder[i]*ConfigData->thrMinFireTime;
		/*! Set pulse remainder to zero. Remainder now stored in onTime */
		ConfigData->pulseRemainder[i] = 0.0;

		/*! Pulse remainder logic */
		if(onTime[i] < ConfigData->thrMinFireTime) {
			/*! Request is less than minimum pulse time */
			ConfigData->pulseRemainder[i] = onTime[i]/ConfigData->thrMinFireTime;
			onTime[i] = 0.0;
		} else if (onTime[i] >= controlPeriod) {
			/*! Request is greater than control period */
			onTime[i] = 1.1*controlPeriod; // oversaturate to avoid numerical error
		}

		/*! Set the output data */
		ConfigData->thrOnTimeOut.OnTimeRequest[i] = onTime[i];
		
	}

	WriteMessage(ConfigData->onTimeOutMsgID, callTime, sizeof(THRArrayOnTimeCmdIntMsg),   /* update module name */
				 (void*) &(ConfigData->thrOnTimeOut), moduleID);

	return;

}
