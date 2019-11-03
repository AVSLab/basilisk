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

#ifndef _FAULT_DETECTION_H_
#define _FAULT_DETECTION_H_

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/cameraConfigMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "fswMessages/opNavFswMsg.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/bsk_Print.h"


/*! \defgroup faultDetection
 *  @brief Module that takes two image processing messages, camera information, and attitude knowledge. It outputs a fault detection and a best measurement given the covariances.

 *  @{
 */
/*! @brief The configuration structure for the opnav fault detection module.*/
typedef struct {
    char opNavOutMsgName[MAX_STAT_MSG_LENGTH]; //!< [-] The name of the output navigation message for relative position
    char attInMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the attitude message
    char navMeasPrimaryMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the first meas message
    char navMeasSecondaryMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the second meas message
    char cameraConfigMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the camera config message
    int32_t planetTarget; //!< The planet targeted (None = 0, Earth = 1, Mars = 2, Jupiter = 3 are allowed)
    double faultMode; //!< What fault mode to go in: 0 is dissimilar (use the primary measurement and compare with secondary), 1 merges the measurements if they are both valid and similar. 
    double sigmaFault; //!< What is the sigma multiplication factor when comparing measurements
    
    int32_t stateOutMsgID;    //!< [-] The ID associated with the outgoing message
    int32_t navMeas1MsgID;    //!< [-] The ID associated with the first incoming measurements
    int32_t navMeas2MsgID;    //!< [-] The ID associated with the second incoming measurements
    int32_t attInMsgID;    //!< [-] The ID associated with the incoming attitude message
    int32_t cameraMsgID;    //!< [-] The ID associated with the second incoming measurements
}FaultDetectionData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_faultDetection(FaultDetectionData *configData, int64_t moduleID);
    void CrossInit_faultDetection(FaultDetectionData *configData, int64_t moduleID);
    void Update_faultDetection(FaultDetectionData *configData, uint64_t callTime,
        int64_t moduleID);
    void Reset_faultDetection(FaultDetectionData *configData, uint64_t callTime, int64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
