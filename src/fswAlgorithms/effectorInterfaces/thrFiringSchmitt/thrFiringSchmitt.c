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
    Thrust Firing Schmitt
 
 */

#include "effectorInterfaces/thrFiringSchmitt/thrFiringSchmitt.h"
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <stdio.h>
#include <string.h>






/*! This method initializes the ConfigData for this module.  It creates a single output message of type
 [THRArrayOnTimeCmdIntMsg](\ref THRArrayOnTimeCmdIntMsg).
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t moduleID)
{
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->onTimeOutMsgId = CreateNewMessage(ConfigData->onTimeOutMsgName,
                                               sizeof(THRArrayOnTimeCmdIntMsg),
                                               "THRArrayOnTimeCmdIntMsg",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It links to 2 required input messages of type [THRArrayCmdForceFswMsg](\ref THRArrayCmdForceFswMsg)
 and [THRArrayConfigFswMsg](\ref THRArrayConfigFswMsg).
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t moduleID)
{
	/*! - Get the input message ID's */
	ConfigData->thrForceInMsgId = subscribeToMessage(ConfigData->thrForceInMsgName,
														 sizeof(THRArrayCmdForceFswMsg),
														 moduleID);
	ConfigData->thrConfInMsgId = subscribeToMessage(ConfigData->thrConfInMsgName,
												sizeof(THRArrayConfigFswMsg),
												moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
	THRArrayConfigFswMsg   localThrusterData;     /* local copy of the thruster data message */
	uint64_t            timeOfMsgWritten;
	uint32_t            sizeOfMsgWritten;
	int 				i;

	ConfigData->prevCallTime = 0;

	/*! - Zero and read in the support messages */
    memset(&localThrusterData, 0x0, sizeof(THRArrayConfigFswMsg));
	ReadMessage(ConfigData->thrConfInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
				sizeof(THRArrayConfigFswMsg), &localThrusterData, moduleID);

    /*! - store the number of installed thrusters */
	ConfigData->numThrusters = localThrusterData.numThrusters;

    /*! - loop over all thrusters and for each copy over maximum thrust, set last state to off */
	for(i=0; i<ConfigData->numThrusters; i++) {
		ConfigData->maxThrust[i] = localThrusterData.thrusters[i].maxThrust;
		ConfigData->lastThrustState[i] = BOOL_FALSE;
	}
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrFiringSchmitt(thrFiringSchmittConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{

	uint64_t            timeOfMsgWritten;
	uint32_t            sizeOfMsgWritten;
	int 				i;
	double 				level;					/* [-] duty cycle fraction */
	double				controlPeriod;			/* [s] control period */
	double				onTime[MAX_EFF_CNT];	/* [s] array of commanded on time for thrusters */
    THRArrayCmdForceFswMsg thrForceIn;          /* -- copy of the thruster force input message */
    THRArrayOnTimeCmdIntMsg thrOnTimeOut;       /* -- copy of the thruster on-time output message */

    /*! - zero the output message */
    memset(&thrOnTimeOut, 0x0, sizeof(THRArrayOnTimeCmdIntMsg));

    /*! - the first time update() is called there is no information on the time step.  Here
     return either all thrusters off or on depending on the baseThrustState state */
	if(ConfigData->prevCallTime == 0) {
		ConfigData->prevCallTime = callTime;

		for(i = 0; i < ConfigData->numThrusters; i++) {
			thrOnTimeOut.OnTimeRequest[i] = (double)(ConfigData->baseThrustState) * 2.0;
		}

		WriteMessage(ConfigData->onTimeOutMsgId, callTime, sizeof(THRArrayOnTimeCmdIntMsg),
					 (void*) &(thrOnTimeOut), moduleID);
		return;
	}

    /*! - compute control time period Delta_t */
	controlPeriod = ((double)(callTime - ConfigData->prevCallTime)) * NANO2SEC;
	ConfigData->prevCallTime = callTime;

    /*! - Zero and read the input thruster force message */
    memset(&thrForceIn, 0x0, sizeof(THRArrayCmdForceFswMsg));
	ReadMessage(ConfigData->thrForceInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
				sizeof(THRArrayCmdForceFswMsg), (void*) &thrForceIn, moduleID);

    /*! - Loop through thrusters */
	for(i = 0; i < ConfigData->numThrusters; i++) {

        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
		if (ConfigData->baseThrustState == 1) {
			thrForceIn.thrForce[i] += ConfigData->maxThrust[i];
		}

        /*! - Do not allow thrust requests less than zero */
		if (thrForceIn.thrForce[i] < 0.0) {
			thrForceIn.thrForce[i] = 0.0;
		}
        /*! - Compute T_on from thrust request, max thrust, and control period */
		onTime[i] = thrForceIn.thrForce[i]/ConfigData->maxThrust[i]*controlPeriod;

        /*! - Apply Schmitt trigger logic */
		if (onTime[i] < ConfigData->thrMinFireTime) {
			/*! - Request is less than minimum fire time */
			level = onTime[i]/ConfigData->thrMinFireTime;
			if (level >= ConfigData->level_on) {
				ConfigData->lastThrustState[i] = BOOL_TRUE;
				onTime[i] = ConfigData->thrMinFireTime;
			} else if (level <= ConfigData->level_off) {
				ConfigData->lastThrustState[i] = BOOL_FALSE;
				onTime[i] = 0.0;
			} else if (ConfigData->lastThrustState[i] == BOOL_TRUE) {
				onTime[i] = ConfigData->thrMinFireTime;
			} else {
				onTime[i] = 0.0;
			}
		} else if (onTime[i] >= controlPeriod) {
            /*! - Request is greater than control period then oversaturate onTime */
			ConfigData->lastThrustState[i] = BOOL_TRUE;
			onTime[i] = 1.1*controlPeriod; // oversaturate to avoid numerical error
		} else {
			/*! - Request is greater than minimum fire time and less than control period */
			ConfigData->lastThrustState[i] = BOOL_TRUE;
		}

		/*! Set the output data */
		thrOnTimeOut.OnTimeRequest[i] = onTime[i];
	}

	WriteMessage(ConfigData->onTimeOutMsgId, callTime, sizeof(THRArrayOnTimeCmdIntMsg),
				 (void*) &thrOnTimeOut, moduleID);

	return;

}
