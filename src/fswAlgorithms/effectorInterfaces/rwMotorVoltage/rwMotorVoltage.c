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
    FSW MODULE: RW motor voltage command
 
 */

#include "effectorInterfaces/rwMotorVoltage/rwMotorVoltage.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "simulation/utilities/linearAlgebra.h"
#include <string.h>


/*! This method initializes the ConfigData for this module.
 It creates the output message.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_rwMotorVoltage(rwMotorVoltageConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->voltageOutMsgID = CreateNewMessage(ConfigData->voltageOutMsgName,
                                               sizeof(RWArrayVoltageIntMsg),
                                               "RWArrayVoltageIntMsg",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_rwMotorVoltage(rwMotorVoltageConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->torqueInMsgID = subscribeToMessage(ConfigData->torqueInMsgName,
                                                sizeof(RWArrayTorqueIntMsg),
                                                moduleID);
    ConfigData->rwParamsInMsgID = -1;
    ConfigData->inputRWSpeedsInMsgID = -1;
    ConfigData->rwAvailInMsgID = -1;

    ConfigData->rwParamsInMsgID = subscribeToMessage(ConfigData->rwParamsInMsgName,
                                                     sizeof(RWArrayConfigFswMsg), moduleID);

    if (strlen(ConfigData->inputRWSpeedsInMsgName) > 0) {
        ConfigData->inputRWSpeedsInMsgID = subscribeToMessage(ConfigData->inputRWSpeedsInMsgName,
                                                         sizeof(RWSpeedIntMsg), moduleID);
    }
    if(strlen(ConfigData->rwAvailInMsgName) > 0) {
        ConfigData->rwAvailInMsgID = subscribeToMessage(ConfigData->rwAvailInMsgName,
                                                        sizeof(RWAvailabilityFswMsg), moduleID);
    }
}

/*! This method performs a reset of the module as far as closed loop control is concerned.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime Sim time in nanos
 */
void Reset_rwMotorVoltage(rwMotorVoltageConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read static RW config data message and store it in module variables*/
    uint64_t clockTime;
    uint32_t readSize;
    ReadMessage(ConfigData->rwParamsInMsgID, &clockTime, &readSize,
                sizeof(RWArrayConfigFswMsg), &(ConfigData->rwConfigParams), moduleID);

    /* reset variables */
    memset(ConfigData->rwSpeedOld, 0, sizeof(double)*MAX_EFF_CNT);
    ConfigData->resetFlag = BOOL_TRUE;

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    ConfigData->priorTime = 0;
}

/*! Update performs the torque to voltage conversion. If a wheel speed message was provided, it also does closed loop control of the voltage sent. It then writes the voltage message.
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_rwMotorVoltage(rwMotorVoltageConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /* - Read the input messages */
    uint64_t            clockTime;
    uint32_t            readSize;
    double              torqueCmd[MAX_EFF_CNT];     /*!< [Nm]   copy of RW motor torque input vector */
    uint32_t i;
    ReadMessage(ConfigData->torqueInMsgID, &clockTime, &readSize,
                sizeof(RWArrayTorqueIntMsg), (void*) torqueCmd, moduleID);
    RWSpeedIntMsg       rwSpeed;                    /*!< [r/s] Reaction wheel speed estimates */
    if (ConfigData->inputRWSpeedsInMsgID >= 0) {
        ReadMessage(ConfigData->inputRWSpeedsInMsgID, &clockTime, &readSize,
                    sizeof(RWSpeedIntMsg), (void*) &(rwSpeed), moduleID);
    }
    RWAvailabilityFswMsg  rwAvailability;             /*!< []    Reaction wheel availability */
    memset(rwAvailability.wheelAvailability, 0x0, MAX_EFF_CNT * sizeof(int)); // wheelAvailability set to 0 (AVAILABLE) by default
    if (ConfigData->rwAvailInMsgID >= 0){
        ReadMessage(ConfigData->rwAvailInMsgID, &clockTime, &readSize,
                    sizeof(RWAvailabilityFswMsg), &rwAvailability, moduleID);
    }

    /* zero the output voltage vector */
    double              voltage[MAX_EFF_CNT];       /*!< [V]   RW voltage output commands */
    memset(voltage, 0, sizeof(double)*MAX_EFF_CNT);

    /* compute the often used double array size of RW double values */
    uint32_t rwArrayMemorySize = ConfigData->rwConfigParams.numRW*sizeof(double);

    /* if the torque closed-loop is on, evaluate the feedback term */
    if (ConfigData->inputRWSpeedsInMsgID >= 0) {
        /* make sure the clock didn't just initialize, or the module was recently reset */
        if (ConfigData->priorTime != 0) {
            double dt = (callTime - ConfigData->priorTime) * NANO2SEC; /*!< [s]   control update period */
            double              OmegaDot[MAX_EFF_CNT];     /*!< [r/s^2] RW angular acceleration */
            for (i=0; i<ConfigData->rwConfigParams.numRW; i++) {
                if (rwAvailability.wheelAvailability[i] == AVAILABLE && ConfigData->resetFlag == BOOL_FALSE) {
                    OmegaDot[i] = (rwSpeed.wheelSpeeds[i] - ConfigData->rwSpeedOld[i])/dt;
                    torqueCmd[i] -= ConfigData->K * (ConfigData->rwConfigParams.JsList[i] * OmegaDot[i] - torqueCmd[i]);
                }
                ConfigData->rwSpeedOld[i] = rwSpeed.wheelSpeeds[i];
            }
            ConfigData->resetFlag = BOOL_FALSE;
        }
        ConfigData->priorTime = callTime;
    }

    /* evaluate the feedforward mapping of torque into voltage */
    for (i=0; i<ConfigData->rwConfigParams.numRW; i++) {
        if (rwAvailability.wheelAvailability[i] == AVAILABLE) {
            voltage[i] = (ConfigData->VMax - ConfigData->VMin)/ConfigData->rwConfigParams.uMax[i]
                        * torqueCmd[i];
            if (voltage[i]>0.0) voltage[i] += ConfigData->VMin;
            if (voltage[i]<0.0) voltage[i] -= ConfigData->VMin;
        }
    }

    /* check for voltage saturation */
    for (i=0; i<ConfigData->rwConfigParams.numRW; i++) {
        if (voltage[i] > ConfigData->VMax) {
            voltage[i] = ConfigData->VMax;
        }
        if (voltage[i] < -ConfigData->VMax) {
            voltage[i] = -ConfigData->VMax;
        }
    }

    /*
     store the output message 
     */
    memcpy(ConfigData->voltageOut.voltage,
           voltage,
           rwArrayMemorySize);

    WriteMessage(ConfigData->voltageOutMsgID, callTime, sizeof(RWArrayVoltageIntMsg),
                 (void*) &(ConfigData->voltageOut), moduleID);

    return;
}
