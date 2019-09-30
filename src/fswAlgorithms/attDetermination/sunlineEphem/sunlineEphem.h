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

#ifndef _SUNLINE_EPHEM_FSW_MSG_H_
#define _SUNLINE_EPHEM_FSW_MSG_H_

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "simFswInterfaceMessages/ephemerisIntMsg.h"
#include <stdint.h>


/*! \defgroup sunlineEphem
 * @brief This module computes an ephemeris-based sunline heading.
 *
 * More information can be found in the [PDF Description](Basilisk-SunlineEphem-20181204.pdf).
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* declare module IO interfaces */
    char navStateOutMsgName[MAX_STAT_MSG_LENGTH];   //!< The name of the output message
    char sunPositionInMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the sun ephemeris input message
    char scPositionInMsgName[MAX_STAT_MSG_LENGTH];  //!< The name of the spacecraft ephemeris input message
    char scAttitudeInMsgName[MAX_STAT_MSG_LENGTH];  //!< The name of the spacecraft attitude input message
    
    int32_t navStateOutMsgId;   //!<  [-]  ID for the outgoing body estimate message
    int32_t sunPositionInMsgId; //!<  [-]  ID for the incoming CSS sensor message
    int32_t scPositionInMsgId;  //!<  [-]  ID for the incoming spacecraft position message
    int32_t scAttitudeInMsgId;  //!<  [-]  ID for the incoming spacecraft attitude message

}sunlineEphemConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunlineEphem(sunlineEphemConfig *configData, int64_t moduleID);
    void CrossInit_sunlineEphem(sunlineEphemConfig *configData, int64_t moduleID);
    void Update_sunlineEphem(sunlineEphemConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_sunlineEphem(sunlineEphemConfig *configData, uint64_t callTime, int64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
