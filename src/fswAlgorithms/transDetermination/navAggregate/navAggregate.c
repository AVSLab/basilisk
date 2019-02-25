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

#include "navAggregate.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/bsk_Print.h"
#include "utilities/linearAlgebra.h"
#include <string.h>

/*! This method initializes the ConfigData for the nav aggregation algorithm.  
    It initializes the output messages in the messaging system.
 @return void
 @param ConfigData The configuration data associated with the Nav aggregation interface
 */
void SelfInit_aggregateNav(NavAggregateData *ConfigData, uint64_t moduleID)
{
    /*! - create the attitude navigation output message */
    ConfigData->navAttOutMsgID = CreateNewMessage(ConfigData->outputAttName,
        sizeof(NavAttIntMsg), "NavAttIntMsg", moduleID);

    /*! - create the translation navigation output message */
    ConfigData->navTransOutMsgID = CreateNewMessage(ConfigData->outputTransName,
        sizeof(NavTransIntMsg), "NavTransIntMsg", moduleID);

    return;
}

/*! This method performs the second stage of initialization for the nav aggregration 
    interface.  For each configured input message, it subscribes to the associated target message
    and saves the ID.
 @return void
 @param ConfigData The configuration data associated with the aggregate nav interface
 */
void CrossInit_aggregateNav(NavAggregateData *ConfigData, uint64_t moduleID)
{
    uint32_t i;
    /*! - loop over the number of attitude input messages */
    for(i=0; i<ConfigData->attMsgCount; i=i+1)
    {
        if (strcmp(ConfigData->attMsgs[i].inputNavName,"")==0) {
            BSK_PRINT(MSG_ERROR, "An attitude input message name was not specified.  Be sure that attMsgCount is set properly.\n");
        } else {
            /*!   - subscribe to attitude navigation message */
            ConfigData->attMsgs[i].inputNavID = subscribeToMessage(
                ConfigData->attMsgs[i].inputNavName, sizeof(NavAttIntMsg), moduleID);
        }
    }
    /*! - loop over the number of translational input messages */
    for(i=0; i<ConfigData->transMsgCount; i=i+1)
    {
        if (strcmp(ConfigData->transMsgs[i].inputNavName,"")==0) {
            BSK_PRINT(MSG_ERROR, "A translation input message name was not specified.  Be sure that transMsgCount is set properly.\n");
        } else {
            ConfigData->transMsgs[i].inputNavID = subscribeToMessage(
                ConfigData->transMsgs[i].inputNavName, sizeof(NavTransIntMsg), moduleID);
        }
    }

    return;
}

/*! This resets the module to original states.
 @return void
 @param ConfigData The configuration data associated with this module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the ConfigData
 */
void Reset_aggregateNav(NavAggregateData *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - ensure incoming message counters are not larger than MAX_AGG_NAV_MSG */
    if (ConfigData->attMsgCount > MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The attitude message count %d is larger than allowed (%d)\n",
                  ConfigData->attMsgCount, MAX_AGG_NAV_MSG);
    if (ConfigData->transMsgCount > MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The translation message count %d is larger than allowed (%d)\n",
                  ConfigData->transMsgCount, MAX_AGG_NAV_MSG);

    /*! - ensure the attitude message index locations are less than MAX_AGG_NAV_MSG */
    if (ConfigData->attTimeIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The attTimeIdx variable %d is too large. Must be less than %d.\n",
                  ConfigData->attTimeIdx, MAX_AGG_NAV_MSG);
    if (ConfigData->attIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The attIdx variable %d is too large. Must be less than %d.\n",
                  ConfigData->attIdx, MAX_AGG_NAV_MSG);
    if (ConfigData->rateIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The rateIdx variable %d is too large. Must be less than %d.\n",
                  ConfigData->rateIdx, MAX_AGG_NAV_MSG);
    if (ConfigData->sunIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The sunIdx variable %d is too large. Must be less than %d.\n",
                  ConfigData->sunIdx, MAX_AGG_NAV_MSG);

    /*! - ensure the translational message index locations are less than MAX_AGG_NAV_MSG */
    if (ConfigData->transTimeIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The transTimeIdx variable %d is too large. Must be less than %d.\n",
                  ConfigData->transTimeIdx, MAX_AGG_NAV_MSG);
    if (ConfigData->posIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The posIdx variable %d is too large. Must be less than %d.\n",
                  ConfigData->posIdx, MAX_AGG_NAV_MSG);
    if (ConfigData->velIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The velIdx variable %d is too large. Must be less than %d.\n",
                  ConfigData->velIdx, MAX_AGG_NAV_MSG);
    if (ConfigData->dvIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The dvIdx variable %d is too large. Must be less than %d.\n",
                  ConfigData->dvIdx, MAX_AGG_NAV_MSG);

}


/*! This method takes the navigation message snippets created by the various 
    navigation components in the FSW and aggregates them into a single complete 
    navigation message.
 @return void
 @param ConfigData The configuration data associated with the aggregate nav module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_aggregateNav(NavAggregateData *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    uint32_t i;

    /*! - zero the output message buffers */
    memset(&(ConfigData->navAttOutMsgBuffer), 0x0, sizeof(NavAttIntMsg));
    memset(&(ConfigData->navTransOutMsgBuffer), 0x0, sizeof(NavTransIntMsg));

    /*! - check that attitude navigation messages are present */
    if (ConfigData->attMsgCount) {
        /*! - Iterate through all of the attitude input messages, clear local Msg buffer and archive the new nav data */
        for(i=0; i<ConfigData->attMsgCount; i=i+1)
        {
            memset(&(ConfigData->attMsgs[i].msgStorage), 0x0, sizeof(NavAttIntMsg));
            ReadMessage(ConfigData->attMsgs[i].inputNavID, &timeOfMsgWritten, &sizeOfMsgWritten,
                        sizeof(NavAttIntMsg), &(ConfigData->attMsgs[i].msgStorage), moduleID);
        }

        /*! - Copy out each part of the attitude source message into the target output message*/
        ConfigData->navAttOutMsgBuffer.timeTag = ConfigData->attMsgs[ConfigData->attTimeIdx].msgStorage.timeTag;
        v3Copy(ConfigData->attMsgs[ConfigData->attIdx].msgStorage.sigma_BN, ConfigData->navAttOutMsgBuffer.sigma_BN);
        v3Copy(ConfigData->attMsgs[ConfigData->rateIdx].msgStorage.omega_BN_B, ConfigData->navAttOutMsgBuffer.omega_BN_B);
        v3Copy(ConfigData->attMsgs[ConfigData->sunIdx].msgStorage.vehSunPntBdy, ConfigData->navAttOutMsgBuffer.vehSunPntBdy);

    }

    /*! - check that translation navigation messages are present */
    if (ConfigData->transMsgCount) {
        /*! - Iterate through all of the translation input messages, clear local Msg buffer and archive the new nav data */
        for(i=0; i<ConfigData->transMsgCount; i=i+1)
        {
            memset(&(ConfigData->transMsgs[i].msgStorage), 0x0, sizeof(NavAttIntMsg));
            ReadMessage(ConfigData->transMsgs[i].inputNavID, &timeOfMsgWritten, &sizeOfMsgWritten,
                        sizeof(NavTransIntMsg), &(ConfigData->transMsgs[i].msgStorage), moduleID);
        }

        /*! - Copy out each part of the translation source message into the target output message*/
        ConfigData->navTransOutMsgBuffer.timeTag = ConfigData->transMsgs[ConfigData->transTimeIdx].msgStorage.timeTag;
        v3Copy(ConfigData->transMsgs[ConfigData->posIdx].msgStorage.r_BN_N, ConfigData->navTransOutMsgBuffer.r_BN_N);
        v3Copy(ConfigData->transMsgs[ConfigData->velIdx].msgStorage.v_BN_N, ConfigData->navTransOutMsgBuffer.v_BN_N);
        v3Copy(ConfigData->transMsgs[ConfigData->dvIdx].msgStorage.vehAccumDV, ConfigData->navTransOutMsgBuffer.vehAccumDV);
    }

    /*! - Write the total message out for everyone else to pick up */
    WriteMessage(ConfigData->navAttOutMsgID, callTime, sizeof(NavAttIntMsg),
                 &(ConfigData->navAttOutMsgBuffer), moduleID);
    WriteMessage(ConfigData->navTransOutMsgID, callTime, sizeof(NavTransIntMsg),
                 &(ConfigData->navTransOutMsgBuffer), moduleID);

    return;
}
