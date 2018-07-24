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

#include "effectorInterfaces/thrustRWDesat/thrustRWDesat.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
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
        ConfigData->outputThrName, sizeof(THRArrayOnTimeCmdIntMsg),
        "THRArrayOnTimeCmdIntMsg", moduleID);
  
    
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
    RWConstellationFswMsg localRWData;
    THRArrayConfigFswMsg localThrustData;
    VehicleConfigFswMsg localConfigData;
    int i;
    uint64_t ClockTime;
    uint32_t ReadSize;
    double momentArm[3];
    double thrustDat_B[3];
    
    /*! - Get the control data message ID*/
    ConfigData->inputSpeedID = subscribeToMessage(ConfigData->inputSpeedName,
        sizeof(RWSpeedIntMsg), moduleID);
    ConfigData->inputRWConfID = subscribeToMessage(ConfigData->inputRWConfigData,
                                                   sizeof(RWConstellationFswMsg), moduleID);
    ConfigData->inputMassPropID = subscribeToMessage(
        ConfigData->inputMassPropsName, sizeof(VehicleConfigFswMsg), moduleID);
    ConfigData->inputThrConID = subscribeToMessage(ConfigData->inputThrConfigName,
                                                   sizeof(THRArrayConfigFswMsg), moduleID);
    /*! - Read input messages */
    ReadMessage(ConfigData->inputRWConfID, &ClockTime, &ReadSize,
                sizeof(RWConstellationFswMsg), &localRWData, moduleID);
    ReadMessage(ConfigData->inputMassPropID, &ClockTime, &ReadSize,
                sizeof(VehicleConfigFswMsg), &localConfigData, moduleID);
    ReadMessage(ConfigData->inputThrConID, &ClockTime, &ReadSize,
                sizeof(THRArrayConfigFswMsg), &localThrustData, moduleID);
    
    /*! - Transform from structure S to body B frame */
    ConfigData->numRWAs = localRWData.numRW;
    for(i=0; i<ConfigData->numRWAs; i=i+1)
    {
        v3Copy(localRWData.reactionWheels[i].gsHat_B, &ConfigData->rwAlignMap[i*3]);
    }
    
    ConfigData->numThrusters = localThrustData.numThrusters;
    for(i=0; i<ConfigData->numThrusters; i=i+1)
    {
        v3Copy(localThrustData.thrusters[i].tHatThrust_B, &ConfigData->thrAlignMap[i*3]);
        v3Copy(localThrustData.thrusters[i].rThrust_B, thrustDat_B);
        v3Subtract(thrustDat_B, localConfigData.CoM_B, momentArm);
        v3Copy(localThrustData.thrusters[i].tHatThrust_B, thrustDat_B);
        v3Cross(momentArm, thrustDat_B, &(ConfigData->thrTorqueMap[i*3]));
    }
    
    
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
	int32_t selectedThruster;     /* Thruster index to fire */
    RWSpeedIntMsg rwSpeeds;      /* Local reaction wheel speeds */
	double observedSpeedVec[3];   /* The total summed speed of RWAs*/
	double singleSpeedVec[3];     /* The speed vector for a single wheel*/
	double bestMatch;             /* The current best thruster/wheel matching*/
	double currentMatch;          /* Assessment of the current match */
    double fireValue;             /* Amount of time to fire the jet for */
	THRArrayOnTimeCmdIntMsg outputData;    /* Local output firings */
  
    /*! Begin method steps*/
    /*! - If we haven't met the cooldown threshold, do nothing */
	if ((callTime - ConfigData->previousFiring)*1.0E-9 < 
		ConfigData->thrFiringPeriod)
	{
		return;
	}

    /*! - Read the input rwheel speeds from the reaction wheels*/
    ReadMessage(ConfigData->inputSpeedID, &ClockTime, &ReadSize,
                sizeof(RWSpeedIntMsg), (void*) &(rwSpeeds), moduleID);
    
    /*! - Accumulate the total momentum vector we want to apply (subtract speed vectors)*/
	v3SetZero(observedSpeedVec);
	for (i = 0; i < ConfigData->numRWAs; i++)
	{
		v3Scale(rwSpeeds.wheelSpeeds[i], &(ConfigData->rwAlignMap[i * 3]), 
			singleSpeedVec);
		v3Subtract(observedSpeedVec, singleSpeedVec, observedSpeedVec);
	}

	/*! - If we are within the specified threshold for the momentum, stop desaturation.*/
	if (v3Norm(observedSpeedVec) < ConfigData->DMThresh)
	{
		return;
	}

    /*! - Iterate through the list of thrusters and find the "best" match for the 
          observed momentum vector that does not continue to perturb the velocity 
          in the same direction as previous aggregate firings.  Only do this once we have 
		  removed the specified momentum accuracy from the current direction.*/
	selectedThruster = -1;
	bestMatch = 0.0;
	if (v3Dot(ConfigData->currDMDir, observedSpeedVec) <= ConfigData->DMThresh)
	{
		for (i = 0; i < ConfigData->numThrusters; i++)
		{

			fireValue = v3Dot(observedSpeedVec,
				&(ConfigData->thrTorqueMap[i * 3]));
			currentMatch = v3Dot(ConfigData->accumulatedImp,
				&(ConfigData->thrAlignMap[i * 3]));
			if (fireValue - currentMatch > bestMatch && fireValue > 0.0)
			{
				selectedThruster = i;
				bestMatch = fireValue - currentMatch;
			}
		}
		if (selectedThruster >= 0)
		{
			v3Normalize(&ConfigData->thrTorqueMap[selectedThruster * 3],
				ConfigData->currDMDir);
		}
	}
    
    /*! - Zero out the thruster commands prior to setting the selected thruster.
          Only apply thruster firing if the best match is non-zero.  Find the thruster 
		  that best matches the current specified direction.
    */
	memset(&outputData, 0x0, sizeof(THRArrayOnTimeCmdIntMsg));
	selectedThruster = -1;
	bestMatch = 0.0;
	for (i = 0; i < ConfigData->numThrusters; i++)
	{

		fireValue = v3Dot(ConfigData->currDMDir,
			&(ConfigData->thrTorqueMap[i * 3]));
		currentMatch = v3Dot(ConfigData->accumulatedImp,
			&(ConfigData->thrAlignMap[i * 3]));
		if (fireValue - currentMatch > bestMatch && fireValue > 0.0)
		{
			selectedThruster = i;
			bestMatch = fireValue - currentMatch;
		}
	}
    /*! - If we have a valid match: 
          - Set firing based on the best counter to the observed momentum.
          - Saturate based on the maximum allowable firing
          - Accumulate impulse and the total firing
          - Set the previous call time value for cooldown check */
	if (bestMatch > 0.0)
	{
		outputData.OnTimeRequest[selectedThruster] = v3Dot(ConfigData->currDMDir,
			&(ConfigData->thrTorqueMap[selectedThruster * 3]));
		outputData.OnTimeRequest[selectedThruster] =
			outputData.OnTimeRequest[selectedThruster] > ConfigData->maxFiring ?
			ConfigData->maxFiring : outputData.OnTimeRequest[selectedThruster];
		ConfigData->previousFiring = callTime;
		ConfigData->totalAccumFiring += outputData.OnTimeRequest[selectedThruster];
        v3Scale(outputData.OnTimeRequest[selectedThruster],
                &(ConfigData->thrAlignMap[selectedThruster * 3]), singleSpeedVec);
        v3Add(ConfigData->accumulatedImp, singleSpeedVec,
            ConfigData->accumulatedImp);
	}
   
    /*! - Write the output message to the thruster system */
	WriteMessage(ConfigData->outputThrID, callTime, sizeof(THRArrayOnTimeCmdIntMsg),
		&(outputData), moduleID);

    return;
}

/*! This method resets the ConfigData for the thruster-based RW desat module.
@return void
@param ConfigData The configuration data associated with the thruster desat
@param moduleID The module ID associated with ConfigData
*/
void Reset_thrustRWDesat(thrustRWDesatConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
	ConfigData->previousFiring = 0;
	v3SetZero(ConfigData->accumulatedImp);
	ConfigData->totalAccumFiring = 0.0;
}

