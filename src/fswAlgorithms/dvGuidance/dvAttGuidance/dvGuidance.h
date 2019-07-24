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

#ifndef _DV_GUIDANCE_POINT_H_
#define _DV_GUIDANCE_POINT_H_

#include "messaging/static_messaging.h"
#include "fswMessages/attRefFswMsg.h"
#include "fswMessages/dvBurnCmdFswMsg.h"
#include <stdint.h>

/*! \defgroup  dvAttGuidance
    @brief This module creates a time varying attitude reference frame message that allows the orbit correction burn direction to rotate at a constant rate.

 A message is read in containing the base \f$\Delta\mathbf{v}\f$ direction, the burn duration, as well as a nominal rotation axis.  A base burn frame is created relative to which a constant rotation about the 3rd frame axis is performed.  The output message contains the full reference frame states including the constant angular velocity vector and a zero angular acceleration vector. More information can be found in the [PDF Description](Basilisk-dvGuidance-2019-03-28.pdf).
 * @{
 */


/*! @brief Top level structure for the nominal delta-V guidance*/
typedef struct {
    char outputDataName[MAX_STAT_MSG_LENGTH]; //!< The name of the output message
    char inputBurnDataName[MAX_STAT_MSG_LENGTH];//<! Input message that configures the vehicle burn
    int32_t outputMsgID;     //!< (-) ID for the outgoing body estimate message
    int32_t inputBurnCmdID;  //!< [-] ID for the incoming burn command data
}dvGuidanceConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_dvGuidance(dvGuidanceConfig *configData, uint64_t moduleID);
    void CrossInit_dvGuidance(dvGuidanceConfig *configData, uint64_t moduleID);
    void Update_dvGuidance(dvGuidanceConfig *configData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_dvGuidance(dvGuidanceConfig *configData, uint64_t callTime,
                           uint64_t moduleID);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
