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
    FSW MODULE Template
 
 */

/* modify the path to reflect the new module names */
#include "effectorInterfaces/thrMomentumDumping/thrMomentumDumping.h"
#include <string.h>
#include <stdio.h>

/* update this include to reflect the required module input messages */



/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "simulation/utilities/bsk_Print.h"


/*! This method initializes the ConfigData for this module.
 It creates the output message.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->thrusterOnTimeOutMsgID = CreateNewMessage(ConfigData->thrusterOnTimeOutMsgName,
                                               sizeof(THRArrayOnTimeCmdIntMsg),
                                               "THRArrayOnTimeCmdIntMsg",          /* add the output structure name */
                                               moduleID);

}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->thrusterImpulseInMsgID = subscribeToMessage(ConfigData->thrusterImpulseInMsgName,
                                                sizeof(THRArrayCmdForceFswMsg),
                                                moduleID);
    ConfigData->thrusterConfInMsgID = subscribeToMessage(ConfigData->thrusterConfInMsgName,
                                                         sizeof(THRArrayConfigFswMsg),
                                                         moduleID);

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    THRArrayConfigFswMsg   localThrusterData;     /*!< local copy of the thruster data message */
    uint64_t            clockTime;
    uint32_t            readSize;
    int                 i;

    ConfigData->priorTime = 0;              /* reset the prior time flag state.  If set
                                             to zero, the control time step is not evaluated on the
                                             first function call */


    /* read in number of thrusters installed */
    ReadMessage(ConfigData->thrusterConfInMsgID, &clockTime, &readSize,
                sizeof(THRArrayConfigFswMsg), &localThrusterData, moduleID);
    ConfigData->numThrusters = localThrusterData.numThrusters;
    for (i=0;i<ConfigData->numThrusters;i++) {
        ConfigData->thrMaxForce[i] = localThrusterData.thrusters[i].maxThrust;
    }

    ConfigData->thrDumpingCounter = 0;

    /* zero out some vectors */
    memset(ConfigData->thrOnTimeRemaining, 0x0, MAX_EFF_CNT*sizeof(double));
    memset(ConfigData->Delta_p, 0x0, MAX_EFF_CNT*sizeof(double));
    memset(&(ConfigData->thrOnTimeOut), 0x0, sizeof(THRArrayOnTimeCmdIntMsg));

    if (ConfigData->maxCounterValue < 1) {
        BSK_PRINT(MSG_WARNING,"The maxCounterValue flag must be set to a positive value.\n");
    }

}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t            clockTime;
    uint32_t            readSize;
    double              dt;                             /*!< [s]    control update period */
    double              Delta_P_input[MAX_EFF_CNT];     /*!< [Ns]   input vector of requested net thruster impulses */
    double              tOnOut[MAX_EFF_CNT];            /*!< [s]    vector of requested thruster on times per dumping cycle */
    int                 i;

    /* zero the output on time vector */
    memset(tOnOut, 0x0, MAX_EFF_CNT*sizeof(double));

    if (ConfigData->priorTime != 0) {       /* don't compute dt if this is the first call after a reset */

        /* compute control update time */
        dt = (callTime - ConfigData->priorTime)*NANO2SEC;
        if (dt > 10.0) dt = 10.0;           /* cap the maximum control time step possible */
        if (dt < 0.0) dt = 0.0;             /* ensure no negative numbers are used */

        /*! - Read the input messages */
        ReadMessage(ConfigData->thrusterImpulseInMsgID, &clockTime, &readSize,
                    sizeof(THRArrayCmdForceFswMsg), (void*) Delta_P_input, moduleID);

        if (memcmp(Delta_P_input, ConfigData->Delta_p, ConfigData->numThrusters*sizeof(double)) == 0) {
            /* idential net thruster impulse request case, continue with existing RW momentum dumping */

            if (ConfigData->thrDumpingCounter <= 0) {
                /* time to fire thrusters again */
                memmove(tOnOut, ConfigData->thrOnTimeRemaining, ConfigData->numThrusters*sizeof(double));
                for (i=0;i<ConfigData->numThrusters;i++) {
                    if (ConfigData->thrOnTimeRemaining[i] >0.0)
                        ConfigData->thrOnTimeRemaining[i] -= dt;
                }
                ConfigData->thrDumpingCounter = ConfigData->maxCounterValue;
            } else {
                /* no thrusters are firing, giving RWs time to settle attitude */
                ConfigData->thrDumpingCounter -= 1;
                memset(tOnOut, 0x0, MAX_EFF_CNT*sizeof(double));
            }


        } else {
            /* new net thruster impulse request case */
            memmove(ConfigData->Delta_p, Delta_P_input, ConfigData->numThrusters*sizeof(double));
            for (i=0;i<ConfigData->numThrusters;i++) {
                ConfigData->thrOnTimeRemaining[i] = Delta_P_input[i]/ConfigData->thrMaxForce[i];
            }
            memmove(tOnOut, ConfigData->thrOnTimeRemaining, ConfigData->numThrusters*sizeof(double));
            ConfigData->thrDumpingCounter = ConfigData->maxCounterValue;
            for (i=0;i<ConfigData->numThrusters;i++) {
                ConfigData->thrOnTimeRemaining[i] -= dt;
            }
        }



        /* check for negative or saturated firing times */
        for (i=0;i<ConfigData->numThrusters;i++) {
            if (tOnOut[i] < ConfigData->thrMinFireTime) tOnOut[i] = 0.0;
            if (ConfigData->thrOnTimeRemaining[i] < 0.0) ConfigData->thrOnTimeRemaining[i] = 0.0;
            if (tOnOut[i] > dt)  tOnOut[i] = dt;
        }


    } else {
        /* first time this module is updated.  Need a 2nd run before the control period is evaluated */
        /* set the thruster firing times to zero */
        memset(tOnOut, 0x0, MAX_EFF_CNT*sizeof(double));
    }

    ConfigData->priorTime = callTime;

    /*
     store the output message
     */
    memmove(ConfigData->thrOnTimeOut.OnTimeRequest, tOnOut, sizeof(THRArrayOnTimeCmdIntMsg));

    WriteMessage(ConfigData->thrusterOnTimeOutMsgID, callTime, sizeof(THRArrayOnTimeCmdIntMsg), 
                 (void*) &(ConfigData->thrOnTimeOut), moduleID);

    return;
}
