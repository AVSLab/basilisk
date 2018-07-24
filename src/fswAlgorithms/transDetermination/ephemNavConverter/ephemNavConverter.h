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

#ifndef _EPHEM_NAV_CONVERTER_H_
#define _EPHEM_NAV_CONVERTER_H_

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/ephemerisIntMsg.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"

/*! @brief Top level structure for the converter that takes an 
    ephemeris output message and converts it over to a translational 
	state estimate message.
*/
typedef struct {
    char stateOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the output navigation message for pos/vel*/
    char ephInMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the clock correlation message*/

    int32_t stateOutMsgID;    /*!< [-] The ID associated with the outgoing message*/
    int32_t ephInMsgID;  /*!< [-] The ID associated with the incoming clock correlation*/
    
    NavTransIntMsg outputState; /*!< [-] The local storage of the outgoing message data*/
}EphemNavConverterData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_ephemNavConverter(EphemNavConverterData *ConfigData, uint64_t moduleID);
    void CrossInit_ephemNavConverter(EphemNavConverterData *ConfigData, uint64_t moduleID);
    void Update_ephemNavConverter(EphemNavConverterData *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_ephemNavConverter(EphemNavConverterData *ConfigData, uint64_t callTime,
                              uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
