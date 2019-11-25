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


/*! This method initializes the configData for this module.
 It creates the output message.
 @return void
 @param configData The configuration data associated with this module
 */
void SelfInit_rwMotorVoltage(rwMotorVoltageConfig *configData, int64_t moduleID)
{
    configData->bskPrint = _BSKPrint();
    /*! - Create output message for module */
    configData->voltageOutMsgID = CreateNewMessage(configData->voltageOutMsgName,
                                               sizeof(RWArrayVoltageIntMsg),
                                               "RWArrayVoltageIntMsg",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param configData The configuration data associated with this module
 */
void CrossInit_rwMotorVoltage(rwMotorVoltageConfig *configData, int64_t moduleID)
{
    /*! - Get the control data message ID*/
    configData->torqueInMsgID = subscribeToMessage(configData->torqueInMsgName,
                                                sizeof(RWArrayTorqueIntMsg),
                                                moduleID);
    configData->rwParamsInMsgID = -1;
    configData->inputRWSpeedsInMsgID = -1;
    configData->rwAvailInMsgID = -1;

    configData->rwParamsInMsgID = subscribeToMessage(configData->rwParamsInMsgName,
                                                     sizeof(RWArrayConfigFswMsg), moduleID);

    if (strlen(configData->inputRWSpeedsInMsgName) > 0) {
        configData->inputRWSpeedsInMsgID = subscribeToMessage(configData->inputRWSpeedsInMsgName,
                                                         sizeof(RWSpeedIntMsg), moduleID);
    }
    if(strlen(configData->rwAvailInMsgName) > 0) {
        configData->rwAvailInMsgID = subscribeToMessage(configData->rwAvailInMsgName,
                                                        sizeof(RWAvailabilityFswMsg), moduleID);
    }
}

/*! This method performs a reset of the module as far as closed loop control is concerned.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime Sim time in nanos
 */
void Reset_rwMotorVoltage(rwMotorVoltageConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Read static RW config data message and store it in module variables*/
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    ReadMessage(configData->rwParamsInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(RWArrayConfigFswMsg), &(configData->rwConfigParams), moduleID);

    /* reset variables */
    memset(configData->rwSpeedOld, 0, sizeof(double)*MAX_EFF_CNT);
    configData->resetFlag = BOOL_TRUE;

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    configData->priorTime = 0;
}

/*! Update performs the torque to voltage conversion. If a wheel speed message was provided, it also does closed loop control of the voltage sent. It then writes the voltage message.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_rwMotorVoltage(rwMotorVoltageConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /* - Read the input messages */
    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;
    double              torqueCmd[MAX_EFF_CNT];     /*!< [Nm]   copy of RW motor torque input vector */
    uint32_t i;
    ReadMessage(configData->torqueInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(RWArrayTorqueIntMsg), (void*) torqueCmd, moduleID);
    RWSpeedIntMsg       rwSpeed;                    /*!< [r/s] Reaction wheel speed estimates */
    if (configData->inputRWSpeedsInMsgID >= 0) {
        ReadMessage(configData->inputRWSpeedsInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(RWSpeedIntMsg), (void*) &(rwSpeed), moduleID);
    }
    RWAvailabilityFswMsg  rwAvailability;             /*!< []    Reaction wheel availability */
    memset(rwAvailability.wheelAvailability, 0x0, MAX_EFF_CNT * sizeof(int)); // wheelAvailability set to 0 (AVAILABLE) by default
    if (configData->rwAvailInMsgID >= 0){
        ReadMessage(configData->rwAvailInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(RWAvailabilityFswMsg), &rwAvailability, moduleID);
    }

    /* zero the output voltage vector */
    double              voltage[MAX_EFF_CNT];       /*!< [V]   RW voltage output commands */
    memset(voltage, 0, sizeof(double)*MAX_EFF_CNT);

    /* compute the often used double array size of RW double values */
    uint32_t rwArrayMemorySize = configData->rwConfigParams.numRW*sizeof(double);

    /* if the torque closed-loop is on, evaluate the feedback term */
    if (configData->inputRWSpeedsInMsgID >= 0) {
        /* make sure the clock didn't just initialize, or the module was recently reset */
        if (configData->priorTime != 0) {
            double dt = (callTime - configData->priorTime) * NANO2SEC; /*!< [s]   control update period */
            double              OmegaDot[MAX_EFF_CNT];     /*!< [r/s^2] RW angular acceleration */
            for (i=0; i<configData->rwConfigParams.numRW; i++) {
                if (rwAvailability.wheelAvailability[i] == AVAILABLE && configData->resetFlag == BOOL_FALSE) {
                    OmegaDot[i] = (rwSpeed.wheelSpeeds[i] - configData->rwSpeedOld[i])/dt;
                    torqueCmd[i] -= configData->K * (configData->rwConfigParams.JsList[i] * OmegaDot[i] - torqueCmd[i]);
                }
                configData->rwSpeedOld[i] = rwSpeed.wheelSpeeds[i];
            }
            configData->resetFlag = BOOL_FALSE;
        }
        configData->priorTime = callTime;
    }

    /* evaluate the feedforward mapping of torque into voltage */
    for (i=0; i<configData->rwConfigParams.numRW; i++) {
        if (rwAvailability.wheelAvailability[i] == AVAILABLE) {
            voltage[i] = (configData->VMax - configData->VMin)/configData->rwConfigParams.uMax[i]
                        * torqueCmd[i];
            if (voltage[i]>0.0) voltage[i] += configData->VMin;
            if (voltage[i]<0.0) voltage[i] -= configData->VMin;
        }
    }

    /* check for voltage saturation */
    for (i=0; i<configData->rwConfigParams.numRW; i++) {
        if (voltage[i] > configData->VMax) {
            voltage[i] = configData->VMax;
        }
        if (voltage[i] < -configData->VMax) {
            voltage[i] = -configData->VMax;
        }
    }

    /*
     store the output message 
     */
    memcpy(configData->voltageOut.voltage,
           voltage,
           rwArrayMemorySize);

    WriteMessage(configData->voltageOutMsgID, callTime, sizeof(RWArrayVoltageIntMsg),
                 (void*) &(configData->voltageOut), moduleID);

    return;
}
