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

#include "effectorInterfaces/thrMomentumDumping/thrMomentumDumping.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "simulation/utilities/bsk_Print.h"
#include <string.h>
#include <stdio.h>


/*! This method initializes the ConfigData for this module.  It creates a single output message of type
 [THRArrayOnTimeCmdIntMsg](\ref THRArrayOnTimeCmdIntMsg).
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t moduleID)
{
    /*! - Create output message for module */
    ConfigData->thrusterOnTimeOutMsgID = CreateNewMessage(ConfigData->thrusterOnTimeOutMsgName,
                                               sizeof(THRArrayOnTimeCmdIntMsg),
                                               "THRArrayOnTimeCmdIntMsg",
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It links to 3 required input messages of type [THRArrayCmdForceFswMsg](\ref THRArrayCmdForceFswMsg)
 and [THRArrayConfigFswMsg](\ref THRArrayConfigFswMsg).
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the message ID for the requested thruster impulse message */
    ConfigData->thrusterImpulseInMsgID = subscribeToMessage(ConfigData->thrusterImpulseInMsgName,
                                                sizeof(THRArrayCmdForceFswMsg),
                                                moduleID);

    /*! - Get the message ID for the thruster configuration message */
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
    THRArrayConfigFswMsg   localThrusterData;     /* local copy of the thruster data message */
    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;
    int                 i;

    /*! - reset the prior time flag state.  If set to zero, the control time step is not evaluated on the
     first function call */
    ConfigData->priorTime = 0;


    /*! - read in number of thrusters installed and maximum thrust values */
    ReadMessage(ConfigData->thrusterConfInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(THRArrayConfigFswMsg), &localThrusterData, moduleID);
    ConfigData->numThrusters = localThrusterData.numThrusters;
    for (i=0;i<ConfigData->numThrusters;i++) {
        ConfigData->thrMaxForce[i] = localThrusterData.thrusters[i].maxThrust;
    }

    /*! - reset dumping counter */
    ConfigData->thrDumpingCounter = 0;

    /*! - zero out some vectors */
    memset(ConfigData->thrOnTimeRemaining, 0x0, MAX_EFF_CNT*sizeof(double));
    memset(ConfigData->Delta_p, 0x0, MAX_EFF_CNT*sizeof(double));
    memset(&(ConfigData->thrOnTimeOut), 0x0, sizeof(THRArrayOnTimeCmdIntMsg));

    /*! - perform sanity check that the module maxCounterValue value is set to a positive value */
    if (ConfigData->maxCounterValue < 1) {
        BSK_PRINT(MSG_WARNING,"The maxCounterValue flag must be set to a positive value.\n");
    }

}

/*! This method reads in the requested thruster impulse message.  If it is a new message then a fresh
 thruster firing cycle is setup to achieve the desired RW momentum dumping.  The the same message is read
 in, then the thrust continue to periodically fire to achieve the net thruster impuleses requested.
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t            timeOfMsgWritten;
    uint32_t            sizeOfMsgWritten;
    double              dt;                             /* [s]    control update period */
    double              Delta_P_input[MAX_EFF_CNT];     /* [Ns]   input vector of requested net thruster impulses */
    double              tOnOut[MAX_EFF_CNT];            /* [s]    vector of requested thruster on times per dumping cycle */
    int                 i;

    /*! - zero the output array of on-time values */
    memset(tOnOut, 0x0, MAX_EFF_CNT*sizeof(double));

    /*! - check if this is the first call after reset.  If yes, write zero output message and exit */
    if (ConfigData->priorTime != 0) {       /* don't compute dt if this is the first call after a reset */

        /* - compute control update time */
        dt = (callTime - ConfigData->priorTime)*NANO2SEC;
        if (dt > 10.0) dt = 10.0;           /* cap the maximum control time step possible */
        if (dt < 0.0) dt = 0.0;             /* ensure no negative numbers are used */

        /*! - Read the input messages */
        ReadMessage(ConfigData->thrusterImpulseInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(THRArrayCmdForceFswMsg), (void*) Delta_P_input, moduleID);

        /*! - check if the thruster impulse input message is identical to current values (continue
         with current momentum dumping), or if the message is new (setup new dumping strategy)  */
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



        /*! - check for negative or saturated firing times */
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

    /*! - write out the output message */
    memmove(ConfigData->thrOnTimeOut.OnTimeRequest, tOnOut, sizeof(THRArrayOnTimeCmdIntMsg));

    WriteMessage(ConfigData->thrusterOnTimeOutMsgID, callTime, sizeof(THRArrayOnTimeCmdIntMsg), 
                 (void*) &(ConfigData->thrOnTimeOut), moduleID);

    return;
}
