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
#include "simulation/utilities/linearAlgebra.h"
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
    memset(&localThrusterData, 0x0, sizeof(THRArrayConfigFswMsg));
    ReadMessage(ConfigData->thrusterConfInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(THRArrayConfigFswMsg), (void *) &localThrusterData, moduleID);
    ConfigData->numThrusters = localThrusterData.numThrusters;
    for (i=0;i<ConfigData->numThrusters;i++) {
        ConfigData->thrMaxForce[i] = localThrusterData.thrusters[i].maxThrust;
    }

    /*! - reset dumping counter */
    ConfigData->thrDumpingCounter = 0;

    /*! - zero out some vectors */
    mSetZero(ConfigData->thrOnTimeRemaining, 1, MAX_EFF_CNT);
    mSetZero(ConfigData->Delta_p, 1, MAX_EFF_CNT);

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
    double              *Delta_P_input;                 /* []     pointer to vector of requested net thruster impulses */
    double              *tOnOut;                        /*        pointer to vector of requested thruster on times per dumping cycle */
    THRArrayOnTimeCmdIntMsg thrOnTimeOut;               /* []     output message container */
    THRArrayCmdForceFswMsg  thrusterImpulseInMsg;       /* []     thruster inpulse input message */
    int                 i;

    /*! - zero the output array of on-time values */
    tOnOut = thrOnTimeOut.OnTimeRequest;
    memset(&thrOnTimeOut, 0x0, sizeof(THRArrayOnTimeCmdIntMsg));

    /*! - check if this is the first call after reset.  If yes, write zero output message and exit */
    if (ConfigData->priorTime != 0) {       /* don't compute dt if this is the first call after a reset */

        /* - compute control update time */
        dt = (callTime - ConfigData->priorTime)*NANO2SEC;
        if (dt > 10.0) dt = 10.0;           /* cap the maximum control time step possible */
        if (dt < 0.0) dt = 0.0;             /* ensure no negative numbers are used */

        /*! - Read the requester thruster impulse input message */
        memset(&thrusterImpulseInMsg, 0x0, sizeof(THRArrayCmdForceFswMsg));
        ReadMessage(ConfigData->thrusterImpulseInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(THRArrayCmdForceFswMsg), (void*) &thrusterImpulseInMsg, moduleID);
        Delta_P_input = thrusterImpulseInMsg.thrForce;

        /*! - check if the thruster impulse input message is identical to current values (continue
         with current momentum dumping), or if the message is new (setup new dumping strategy)  */
        if (memcmp(Delta_P_input, ConfigData->Delta_p, ConfigData->numThrusters*sizeof(double)) == 0) {
            /* idential net thruster impulse request case, continue with existing RW momentum dumping */

            if (ConfigData->thrDumpingCounter <= 0) {
                /* time to fire thrusters again */
                mCopy(ConfigData->thrOnTimeRemaining, 1, ConfigData->numThrusters, tOnOut);
                /* subtract next control period from remaining impulse time */
                for (i=0;i<ConfigData->numThrusters;i++) {
                    if (ConfigData->thrOnTimeRemaining[i] >0.0)
                        ConfigData->thrOnTimeRemaining[i] -= dt;
                }
                /* reset the dumping counter */
                ConfigData->thrDumpingCounter = ConfigData->maxCounterValue;
            } else {
                /* no thrusters are firing, giving RWs time to settle attitude */
                ConfigData->thrDumpingCounter -= 1;
            }


        } else {
            /* new net thruster impulse request case */
            mCopy(Delta_P_input, 1, ConfigData->numThrusters, ConfigData->Delta_p); /* store current Delta_p */
            for (i=0;i<ConfigData->numThrusters;i++) {
                /* compute net time required to implement requested thruster impulse */
                ConfigData->thrOnTimeRemaining[i] = ConfigData->Delta_p[i]/ConfigData->thrMaxForce[i];
            }
            /* set thruster on time to requested impulse time */
            mCopy(ConfigData->thrOnTimeRemaining, 1, ConfigData->numThrusters, tOnOut);
            /* reset the dumping counter */
            ConfigData->thrDumpingCounter = ConfigData->maxCounterValue;
            /* subtract next control period from remaining impulse time */
            for (i=0;i<ConfigData->numThrusters;i++) {
                ConfigData->thrOnTimeRemaining[i] -= dt;
            }
        }

        /*! - check for negative, saturated firing times or negative remaining times */
        for (i=0;i<ConfigData->numThrusters;i++) {
            /* if thruster on time is less than the minimum firing time, set thrust time command to zero */
            if (tOnOut[i] < ConfigData->thrMinFireTime) tOnOut[i] = 0.0;
            /* if the thruster time remainder is negative, zero out the remainder */
            if (ConfigData->thrOnTimeRemaining[i] < 0.0) ConfigData->thrOnTimeRemaining[i] = 0.0;
            /* if the thruster on time is larger than the control period, set it equal to control period */
            if (tOnOut[i] > dt)  tOnOut[i] = dt;
        }
    }

    ConfigData->priorTime = callTime;

    /*! - write out the output message */
    WriteMessage(ConfigData->thrusterOnTimeOutMsgID, callTime, sizeof(THRArrayOnTimeCmdIntMsg), 
                 (void*) &thrOnTimeOut, moduleID);

    return;
}
