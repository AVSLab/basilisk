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

typedef struct{
    char ephInMsgName[MAX_STAT_MSG_LENGTH];  /*!< [-] Input name for the ephemeris message*/
    int32_t ephInMsgID;                      /*!< [-] Input message ID for ephemeris*/
    char ephOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the clock correlation message*/
    int32_t ephOutMsgID;                     /*!< [-] Ephemeris output message*/
    EphemerisIntMsg ephStore;               /*!< [-] Storage buffer for output information*/
}EphemChangeConfig;

/*! @brief Top level structure for the converter that takes an 
    ephemeris output message and converts it over to a translational 
	state estimate message.
*/
typedef struct {
    char ephBaseInMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the clock correlation message*/
    double baseScale;   /*!< [-] The scale factor used for the secondary parameter (zero passivates)*/
    EphemChangeConfig changeBodies[MAX_NUM_CHANGE_BODIES]; /*!< [-] The list of bodies to change out*/
    uint32_t ephBdyCount;    /*!< [-] The number of ephemeris bodies we are changing*/

    int32_t ephBaseInMsgID;  /*!< [-] The ID associated with the incoming clock correlation*/
    EphemerisIntMsg baseEphem;               /*!< [-] Storage buffer for output information*/
    
}EphemDifferenceData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_ephemDifference(EphemDifferenceData *ConfigData, uint64_t moduleID);
    void CrossInit_ephemDifference(EphemDifferenceData *ConfigData, uint64_t moduleID);
    void Update_ephemDifference(EphemDifferenceData *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_ephemDifference(EphemDifferenceData *ConfigData, uint64_t callTime,
                              uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
