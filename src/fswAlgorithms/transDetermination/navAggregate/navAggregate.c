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

/*! This method initializes the configData for the nav aggregation algorithm.
    It initializes the output messages in the messaging system.
 @return void
 @param configData The configuration data associated with the Nav aggregation interface
 */
void SelfInit_aggregateNav(NavAggregateData *configData, uint64_t moduleID)
{
    /*! - create the attitude navigation output message */
    configData->navAttOutMsgID = CreateNewMessage(configData->outputAttName,
        sizeof(NavAttIntMsg), "NavAttIntMsg", moduleID);

    /*! - create the translation navigation output message */
    configData->navTransOutMsgID = CreateNewMessage(configData->outputTransName,
        sizeof(NavTransIntMsg), "NavTransIntMsg", moduleID);

    return;
}

/*! This method performs the second stage of initialization for the nav aggregration 
    interface.  For each configured input message, it subscribes to the associated target message
    and saves the ID.
 @return void
 @param configData The configuration data associated with the aggregate nav interface
 */
void CrossInit_aggregateNav(NavAggregateData *configData, uint64_t moduleID)
{
    uint32_t i;
    /*! - loop over the number of attitude input messages */
    for(i=0; i<configData->attMsgCount; i=i+1)
    {
        if (strcmp(configData->attMsgs[i].inputNavName,"")==0) {
            BSK_PRINT(MSG_ERROR, "An attitude input message name was not specified.  Be sure that attMsgCount is set properly.\n");
        } else {
            /*!   - subscribe to attitude navigation message */
            configData->attMsgs[i].inputNavID = subscribeToMessage(
                configData->attMsgs[i].inputNavName, sizeof(NavAttIntMsg), moduleID);
        }
    }
    /*! - loop over the number of translational input messages */
    for(i=0; i<configData->transMsgCount; i=i+1)
    {
        if (strcmp(configData->transMsgs[i].inputNavName,"")==0) {
            BSK_PRINT(MSG_ERROR, "A translation input message name was not specified.  Be sure that transMsgCount is set properly.\n");
        } else {
            configData->transMsgs[i].inputNavID = subscribeToMessage(
                configData->transMsgs[i].inputNavName, sizeof(NavTransIntMsg), moduleID);
        }
    }

    return;
}

/*! This resets the module to original states.
 @return void
 @param configData The configuration data associated with this module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Reset_aggregateNav(NavAggregateData *configData, uint64_t callTime, uint64_t moduleID)
{
    /*! - ensure incoming message counters are not larger than MAX_AGG_NAV_MSG */
    if (configData->attMsgCount > MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The attitude message count %d is larger than allowed (%d)\n",
                  configData->attMsgCount, MAX_AGG_NAV_MSG);
    if (configData->transMsgCount > MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The translation message count %d is larger than allowed (%d)\n",
                  configData->transMsgCount, MAX_AGG_NAV_MSG);

    /*! - ensure the attitude message index locations are less than MAX_AGG_NAV_MSG */
    if (configData->attTimeIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The attTimeIdx variable %d is too large. Must be less than %d.\n",
                  configData->attTimeIdx, MAX_AGG_NAV_MSG);
    if (configData->attIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The attIdx variable %d is too large. Must be less than %d.\n",
                  configData->attIdx, MAX_AGG_NAV_MSG);
    if (configData->rateIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The rateIdx variable %d is too large. Must be less than %d.\n",
                  configData->rateIdx, MAX_AGG_NAV_MSG);
    if (configData->sunIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The sunIdx variable %d is too large. Must be less than %d.\n",
                  configData->sunIdx, MAX_AGG_NAV_MSG);

    /*! - ensure the translational message index locations are less than MAX_AGG_NAV_MSG */
    if (configData->transTimeIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The transTimeIdx variable %d is too large. Must be less than %d.\n",
                  configData->transTimeIdx, MAX_AGG_NAV_MSG);
    if (configData->posIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The posIdx variable %d is too large. Must be less than %d.\n",
                  configData->posIdx, MAX_AGG_NAV_MSG);
    if (configData->velIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The velIdx variable %d is too large. Must be less than %d.\n",
                  configData->velIdx, MAX_AGG_NAV_MSG);
    if (configData->dvIdx >= MAX_AGG_NAV_MSG)
        BSK_PRINT(MSG_ERROR, "The dvIdx variable %d is too large. Must be less than %d.\n",
                  configData->dvIdx, MAX_AGG_NAV_MSG);

}


/*! This method takes the navigation message snippets created by the various 
    navigation components in the FSW and aggregates them into a single complete 
    navigation message.
 @return void
 @param configData The configuration data associated with the aggregate nav module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_aggregateNav(NavAggregateData *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    uint32_t i;
    NavAttIntMsg navAttOutMsgBuffer;     /* [-] The local storage of the outgoing attitude navibation message data*/
    NavTransIntMsg navTransOutMsgBuffer; /* [-] The local storage of the outgoing message data*/

    /*! - zero the output message buffers */
    memset(&(navAttOutMsgBuffer), 0x0, sizeof(NavAttIntMsg));
    memset(&(navTransOutMsgBuffer), 0x0, sizeof(NavTransIntMsg));

    /*! - check that attitude navigation messages are present */
    if (configData->attMsgCount) {
        /*! - Iterate through all of the attitude input messages, clear local Msg buffer and archive the new nav data */
        for(i=0; i<configData->attMsgCount; i=i+1)
        {
            memset(&(configData->attMsgs[i].msgStorage), 0x0, sizeof(NavAttIntMsg));
            ReadMessage(configData->attMsgs[i].inputNavID, &timeOfMsgWritten, &sizeOfMsgWritten,
                        sizeof(NavAttIntMsg), &(configData->attMsgs[i].msgStorage), moduleID);
        }

        /*! - Copy out each part of the attitude source message into the target output message*/
        navAttOutMsgBuffer.timeTag = configData->attMsgs[configData->attTimeIdx].msgStorage.timeTag;
        v3Copy(configData->attMsgs[configData->attIdx].msgStorage.sigma_BN, navAttOutMsgBuffer.sigma_BN);
        v3Copy(configData->attMsgs[configData->rateIdx].msgStorage.omega_BN_B, navAttOutMsgBuffer.omega_BN_B);
        v3Copy(configData->attMsgs[configData->sunIdx].msgStorage.vehSunPntBdy, navAttOutMsgBuffer.vehSunPntBdy);

    }

    /*! - check that translation navigation messages are present */
    if (configData->transMsgCount) {
        /*! - Iterate through all of the translation input messages, clear local Msg buffer and archive the new nav data */
        for(i=0; i<configData->transMsgCount; i=i+1)
        {
            memset(&(configData->transMsgs[i].msgStorage), 0x0, sizeof(NavAttIntMsg));
            ReadMessage(configData->transMsgs[i].inputNavID, &timeOfMsgWritten, &sizeOfMsgWritten,
                        sizeof(NavTransIntMsg), &(configData->transMsgs[i].msgStorage), moduleID);
        }

        /*! - Copy out each part of the translation source message into the target output message*/
        navTransOutMsgBuffer.timeTag = configData->transMsgs[configData->transTimeIdx].msgStorage.timeTag;
        v3Copy(configData->transMsgs[configData->posIdx].msgStorage.r_BN_N, navTransOutMsgBuffer.r_BN_N);
        v3Copy(configData->transMsgs[configData->velIdx].msgStorage.v_BN_N, navTransOutMsgBuffer.v_BN_N);
        v3Copy(configData->transMsgs[configData->dvIdx].msgStorage.vehAccumDV, navTransOutMsgBuffer.vehAccumDV);
    }

    /*! - Write the total message out for everyone else to pick up */
    WriteMessage(configData->navAttOutMsgID, callTime, sizeof(NavAttIntMsg),
                 &(navAttOutMsgBuffer), moduleID);
    WriteMessage(configData->navTransOutMsgID, callTime, sizeof(NavTransIntMsg),
                 &(navTransOutMsgBuffer), moduleID);

    return;
}
