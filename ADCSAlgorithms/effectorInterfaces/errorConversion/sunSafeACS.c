
#include "effectorInterfaces/errorConversion/sunSafeACS.h"
#include "attControl/vehControlOut.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the sun safe ACS control.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the sun safe control
 */
void SelfInit_sunSafeACS(sunSafeACSConfig *ConfigData)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(vehEffectorOut));
    
}

/*! This method performs the second stage of initialization for the sun safe ACS
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the sun safe ACS control
 */
void CrossInit_sunSafeACS(sunSafeACSConfig *ConfigData)
{
    /*! - Get the control data message ID*/
    ConfigData->inputMsgID = FindMessageID(ConfigData->inputControlName);
    
}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param ConfigData The configuration data associated with the sun safe ACS control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t callTime)
{
    
    double unSortOnTime[MAX_NUM_EFFECTORS];
    effPairs unSortPairs[MAX_NUM_EFFECTORS];
    effPairs sortPairs[MAX_NUM_EFFECTORS];
    uint64_t ClockTime;
    uint32_t ReadSize;
    uint32_t i;
    vehControlOut cntrRequest;
    
    /*! Begin method steps*/
    /*! - Read the input parsed CSS sensor data message*/
    ReadMessage(ConfigData->inputMsgID, &ClockTime, &ReadSize,
                sizeof(vehControlOut), (void*) &(cntrRequest));
    
    v3Scale(-1.0, cntrRequest.accelRequestBody, cntrRequest.accelRequestBody);
    mMultV(ConfigData->thrOnMap, ConfigData->numThrusters, 3,
           cntrRequest.accelRequestBody, unSortOnTime);
    
    for(i=0; i<ConfigData->numThrusters; i++)
    {
        if(unSortOnTime[i] < ConfigData->minThrustRequest)
        {
            unSortOnTime[i] = 0.0;
        }
    }
    
    for(i=0; i<ConfigData->numThrusters; i++)
    {
        unSortPairs[i].onTime = unSortOnTime[i];
        unSortPairs[i].thrustIndex = i;
    }
    effectorVSort(unSortPairs, sortPairs, ConfigData->numThrusters);
    memset(ConfigData->cmdRequests.effectorRequest, 0x0,
           MAX_NUM_EFFECTORS*sizeof(double));
    for(i=0; i<ConfigData->maxNumCmds; i=i+1)
    {
        ConfigData->cmdRequests.effectorRequest[sortPairs[i].thrustIndex] =
        sortPairs[i].onTime;
    }
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehEffectorOut),
                 (void*) &(ConfigData->cmdRequests));
    return;
}

void effectorVSort(effPairs *Input, effPairs *Output, size_t dim)
{
    int i, j;
    int Swapped;
    Swapped = 1;
    memcpy(Output, Input, dim*sizeof(effPairs));
    for(i=0; i<dim && Swapped > 0; i++)
    {
        Swapped = 0;
        for(j=0; j<dim-1; j++)
        {
            if(Output[j].onTime<Output[j+1].onTime)
            {
                double tempOn = Output[j+1].onTime;
                uint32_t tempIndex = Output[j+1].thrustIndex;
                Output[j+1].onTime = Output[j].onTime;
                Output[j+1].thrustIndex = Output[j].thrustIndex;
                Output[j].onTime = tempOn;
                Output[j].thrustIndex = tempIndex;
                Swapped = 1;
            }
        }
    }
}
