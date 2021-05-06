/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef LOCATIONPOINTING_H
#define LOCATIONPOINTING_H

#include <stdint.h>
#include "cMsgCInterface/SCStatesMsg_C.h"
#include "cMsgCInterface/GroundStateMsg_C.h"
#include "cMsgCInterface/AttGuidMsg_C.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief This module is used to generate the attitude reference message in order to have a spacecraft point at a location on the ground
 */
typedef struct {

    /* variables */
    double counter;         /*!< counter for numerical integration*/
    double sigma_RB_new;    /*< Newest sigma_BR value, stored for finite diff*/
    double sigma_RB_old;    /*< Older sigma_BR value, stored for finite diff*/
    double omega_RN_B_new;
    double omega_RN_B_old;
    double time_new;
    double time_old;
    double pHat;

    /* declare module IO interfaces */
    SCStatesMsg_C SCInMsg;  //!< describe space craft states
    GroundStateMsg_C LocationInMsg;  //!< ground state info
    AttGuidMsg_C AttGuidOutMsg;  //!< output msg description
    

    BSKLogger *bskLogger;  //!< BSK Logging
}locationPointingConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_locationPointing(locationPointingConfig *configData, int64_t moduleID);
    void Update_locationPointing(locationPointingConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_locationPointing(locationPointingConfig *configData, uint64_t callTime, int64_t moduleID);

#ifdef __cplusplus
}
#endif

#endif
