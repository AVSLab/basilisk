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

#ifndef _HORIZON_OPNAV_H_
#define _HORIZON_OPNAV_H_

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/limbOpNavMsg.h"
#include "simFswInterfaceMessages/cameraConfigMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "fswMessages/opNavFswMsg.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "utilities/rigidBodyKinematics.h"

/*! \defgroup horizonOpNav
 *  @brief Converter that takes a limb message and camera information and outputs a relative position to the object. This algorithm was developed by J. Christian.
 
 The module
 [PDF Description](Basilisk-horizonOpNav-20190918.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.

 *  @{
 */
/*! @brief The configuration structure for the horizon OpNav module.*/
typedef struct {
    char opNavOutMsgName[MAX_STAT_MSG_LENGTH]; //!< [-] The name of the output navigation message for relative position
    char cameraConfigMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the camera config message
    char attInMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the attitude message
    char limbInMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the limb message
    int32_t planetTarget; //!< The planet targeted (None = 0, Earth = 1, Mars = 2, Jupiter = 3 are allowed)

    int32_t stateOutMsgID;    //!< [-] The ID associated with the outgoing message
    int32_t attInMsgID;    //!< [-] The ID associated with the outgoing message
    int32_t limbInMsgID;    //!< [-] The ID associated with the incoming circle message
    int32_t cameraConfigMsgID;  //!< [-] The ID associated with the incoming camera config message
}HorizonOpNavData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_horizonOpNav(HorizonOpNavData *configData, uint64_t moduleID);
    void CrossInit_horizonOpNav(HorizonOpNavData *configData, uint64_t moduleID);
    void Update_horizonOpNav(HorizonOpNavData *configData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_horizonOpNav(HorizonOpNavData *configData, uint64_t callTime, uint64_t moduleID);
    void QRDecomp(double *inMat, int32_t nRow, int32_t nCol, double *Q , double *R);
    void BackSub(double *R, double *inVec, int32_t nRow, double *n);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
