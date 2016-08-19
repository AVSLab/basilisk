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
	printf("completed SelfInit_thrFiringRemainder");
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
	printf("completed CrossInit_thrFiringRemainder");
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
	}
	printf("completed Reset_thrFiringRemainder");
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
	double ratioOfPulses;
    double onTimeRequest;
	int numPulses;
	double onTime; // onTime


	if(ConfigData->prevCallTime == ~0) {
		ConfigData->prevCallTime = callTime;
		return;
	}

	ConfigData->controlPeriod = ((double)(callTime - ConfigData->prevCallTime)) * 1e-9;
	ConfigData->prevCallTime = callTime;

	/*! Begin method steps */
	/*! - Read the input messages */
	ReadMessage(ConfigData->inputMsgID, &clockTime, &readSize,
				sizeof(vehEffectorOut), (void*) &(ConfigData->thrFiringRemainderIn), moduleID);

	/*! Loop through thrusters */
	for(i = 0; i < ConfigData->numThrusters; i++) {

		onTimeRequest = ConfigData->thrFiringRemainderIn.effectorRequest[i];

		/*! Pulse remainder logic */
		ratioOfPulses = onTimeRequest / ConfigData->pulseTimeResolution[i];
		numPulses = (int)(ratioOfPulses + ConfigData->pulseRemainder[i]);
		ConfigData->pulseRemainder[i] = ratioOfPulses + ConfigData->pulseRemainder[i] - (double)(numPulses);
		onTime = numPulses * ConfigData->pulseTimeResolution[i];

		if(onTime < ConfigData->pulseTimeMin[i]) {
			/*! Request is less than minimum pulse time */
			ConfigData->pulseRemainder[i] += onTime/ConfigData->pulseTimeResolution[i];
			onTime = 0.0;
		} else if (onTime >= ConfigData->controlPeriod) {
			/*! Request is greater than control period */
			onTime = 1.01*ConfigData->controlPeriod; // oversaturate to avoid numerical error
			ConfigData->pulseRemainder[i] = 0.0;
		}

		/*! Set the output data */
		ConfigData->thrFiringRemainderOut.effectorRequest[i] = onTime;

	}

	WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehEffectorOut),   /* update module name */
				 (void*) &(ConfigData->thrFiringRemainderOut), moduleID);

	return;

}
