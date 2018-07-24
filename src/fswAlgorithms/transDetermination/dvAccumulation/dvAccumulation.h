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

#ifndef _DV_ACCUMULATION_H_
#define _DV_ACCUMULATION_H_

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "fswMessages/AccDataFswMsg.h"
#include "fswMessages/AccPktDataFswMsg.h"


/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
typedef struct {
    char outputNavName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char accPktInMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the input accelerometer message*/
    uint32_t msgCount;      /*!< [-] The total number of messages read from inputs */
    uint64_t previousTime;  /*!< [ns] The clock time associated with the previous run of algorithm*/
    int32_t outputNavMsgID;    /*!< [-] The ID associated with the outgoing message*/
    int32_t accPktInMsgID;     /*!< [-] The ID associated with the incoming accelerometer buffer*/
    
    NavTransIntMsg outputData; /*!< [-] The local storage of the outgoing message data*/
}DVAccumulationData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_dvAccumulation(DVAccumulationData *ConfigData, uint64_t moduleID);
    void CrossInit_dvAccumulation(DVAccumulationData *ConfigData, uint64_t moduleID);
    void Update_dvAccumulation(DVAccumulationData *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    void dvAccumulation_swap(AccPktDataFswMsg *p, AccPktDataFswMsg *q);
    int dvAccumulation_partition(AccPktDataFswMsg *A, int start, int end);
    void dvAccumulation_QuickSort(AccPktDataFswMsg *A, int start, int end);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
