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

#include "effectorInterfaces/thrFiringRemainder/thrFiringRemainder.h"
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <stdio.h>
#include <string.h>



/*! This method initializes the configData for this module.  It creates a single output message of type
 [THRArrayOnTimeCmdIntMsg](\ref THRArrayOnTimeCmdIntMsg).
 @return void
 @param configData The configuration data associated with this module
 */
void SelfInit_thrFiringRemainder(thrFiringRemainderConfig *configData, int64_t moduleID)
{
    configData->bskLogger = _BSKLogger();
    /*! - Create output message for module */
    configData->onTimeOutMsgId = CreateNewMessage(configData->onTimeOutMsgName,
                                               sizeof(THRArrayOnTimeCmdIntMsg),
                                               "THRArrayOnTimeCmdIntMsg",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It links to 2 required input messages of type [THRArrayCmdForceFswMsg](\ref THRArrayCmdForceFswMsg)
 and [THRArrayConfigFswMsg](\ref THRArrayConfigFswMsg).
 @return void
 @param configData The configuration data associated with this module
 */
void CrossInit_thrFiringRemainder(thrFiringRemainderConfig *configData, int64_t moduleID)
{
	/*! - Get the input message ID's */
	configData->thrForceInMsgId = subscribeToMessage(configData->thrForceInMsgName,
														 sizeof(THRArrayCmdForceFswMsg),
														 moduleID);
	configData->thrConfInMsgId = subscribeToMessage(configData->thrConfInMsgName,
												sizeof(THRArrayConfigFswMsg),
												moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 */
void Reset_thrFiringRemainder(thrFiringRemainderConfig *configData, uint64_t callTime, int64_t moduleID)
{
	THRArrayConfigFswMsg   localThrusterData;     /* local copy of the thruster data message */
	uint64_t            timeOfMsgWritten;
	uint32_t            sizeOfMsgWritten;
	int 				i;

	configData->prevCallTime = 0;

	/*! - zero and read in the support messages */
    memset(&localThrusterData, 0x0, sizeof(THRArrayConfigFswMsg));
	ReadMessage(configData->thrConfInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
				sizeof(THRArrayConfigFswMsg), &localThrusterData, moduleID);

    /*! - store the number of installed thrusters */
	configData->numThrusters = localThrusterData.numThrusters;

    /*! - loop over all thrusters and for each copy over maximum thrust, zero the impulse remainder */
	for(i=0; i<configData->numThrusters; i++) {
		configData->maxThrust[i] = localThrusterData.thrusters[i].maxThrust;
		configData->pulseRemainder[i] = 0.0;
	}

}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrFiringRemainder(thrFiringRemainderConfig *configData, uint64_t callTime, int64_t moduleID)
{
	uint64_t            timeOfMsgWritten;
	uint32_t            sizeOfMsgWritten;
	int 				i;
	double				controlPeriod;			/* [s] control period */
	double				onTime[MAX_EFF_CNT];	/* [s] array of commanded on time for thrusters */
    THRArrayCmdForceFswMsg thrForceIn;          /* [-] copy of the thruster force input message */
    THRArrayOnTimeCmdIntMsg thrOnTimeOut;       /* [-] copy of the thruster on-time output message */

    /*! - zero the output message */
    memset(&thrOnTimeOut, 0x0, sizeof(THRArrayOnTimeCmdIntMsg));

    /*! - the first time update() is called there is no information on the time step.  Here
     return either all thrusters off or on depending on the baseThrustState state */
	if(configData->prevCallTime == 0) {
		configData->prevCallTime = callTime;

		for(i = 0; i < configData->numThrusters; i++) {
            /*! - If on-pulsing is used, then the OnTimeRequest is set to zero.
             If off-pulsing is used, then the OnTimeRequest is set to 2 seconds */
			thrOnTimeOut.OnTimeRequest[i] = (double)(configData->baseThrustState) * 2.0;
		}

		WriteMessage(configData->onTimeOutMsgId, callTime, sizeof(THRArrayOnTimeCmdIntMsg),   /* update module name */
					 (void*) &thrOnTimeOut, moduleID);
		return;
	}

    /*! - compute control time period Delta_t */
	controlPeriod = ((double)(callTime - configData->prevCallTime)) * NANO2SEC;
	configData->prevCallTime = callTime;

	/*! - Read the input thruster force message */
    memset(&thrForceIn, 0x0, sizeof(THRArrayCmdForceFswMsg));
	ReadMessage(configData->thrForceInMsgId, &timeOfMsgWritten, &sizeOfMsgWritten,
				sizeof(THRArrayCmdForceFswMsg), (void*) &thrForceIn, moduleID);

	/*! - Loop through thrusters */
	for(i = 0; i < configData->numThrusters; i++) {

		/*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
		if (configData->baseThrustState == 1) {
			thrForceIn.thrForce[i] += configData->maxThrust[i];
		}

		/*! - Do not allow thrust requests less than zero */
		if (thrForceIn.thrForce[i] < 0.0) {
			thrForceIn.thrForce[i] = 0.0;
		}

		/*! - Compute T_on from thrust request, max thrust, and control period */
		onTime[i] = thrForceIn.thrForce[i]/configData->maxThrust[i]*controlPeriod;
		/*! - Add in remainder from the last control step */
		onTime[i] += configData->pulseRemainder[i]*configData->thrMinFireTime;
		/*! - Set pulse remainder to zero. Remainder now stored in onTime */
		configData->pulseRemainder[i] = 0.0;

		/* Pulse remainder logic */
		if(onTime[i] < configData->thrMinFireTime) {
			/*! - If request is less than minimum pulse time zero onTime an store remainder */
			configData->pulseRemainder[i] = onTime[i]/configData->thrMinFireTime;
			onTime[i] = 0.0;
		} else if (onTime[i] >= controlPeriod) {
			/*! - If request is greater than control period then oversaturate onTime */
			onTime[i] = 1.1*controlPeriod;
		}

		/*! - Set the output data for each thruster */
		thrOnTimeOut.OnTimeRequest[i] = onTime[i];
		
	}

    /*! - write the moduel output message */
	WriteMessage(configData->onTimeOutMsgId, callTime, sizeof(THRArrayOnTimeCmdIntMsg),
				 (void*) &thrOnTimeOut, moduleID);

	return;

}
