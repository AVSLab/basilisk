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

#ifndef _EPHEM_DIFFERENCE_H_
#define _EPHEM_DIFFERENCE_H_

#define MAX_NUM_CHANGE_BODIES 10

#include "messaging/static_messaging.h"
#include "./simFswInterfaceMessages/ephemerisIntMsg.h"

/*! \defgroup ephemDifference
 * @brief This module reads in the position and velocity of multiple orbital bodies
 *  and outputs position and velocity of each body relative to a single other
 *  orbital body position and velocity.
 *
 * More information can be found in the [PDF Description](Basilisk-ephemDifference-2019-03-27.pdf).
 *  @{
 */

/*! @brief Container with paired input/output message names and IDs */
typedef struct{
    char ephInMsgName[MAX_STAT_MSG_LENGTH];  //!< [-] Input name for the ephemeris message
    int32_t ephInMsgId;                      //!< [-] Input message ID for ephemeris
    char ephOutMsgName[MAX_STAT_MSG_LENGTH]; //!< [-] The name converted output message
    int32_t ephOutMsgId;                     //!< [-] Ephemeris output message ID
}EphemChangeConfig;

/*! @brief Container holding ephemDifference module variables */
typedef struct {
    char ephBaseInMsgName[MAX_STAT_MSG_LENGTH]; //!< Name of the base ephemeris input message name
    EphemChangeConfig changeBodies[MAX_NUM_CHANGE_BODIES]; //!< [-] The list of bodies to change out
    uint32_t ephBdyCount; //!< [-] The number of ephemeris bodies we are changing
    int32_t ephBaseInMsgId; //!< [-] The ID associated with the incoming clock correlation
}EphemDifferenceData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_ephemDifference(EphemDifferenceData *configData, uint64_t moduleID);
    void CrossInit_ephemDifference(EphemDifferenceData *configData, uint64_t moduleID);
    void Update_ephemDifference(EphemDifferenceData *configData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_ephemDifference(EphemDifferenceData *configData, uint64_t callTime,
                              uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
