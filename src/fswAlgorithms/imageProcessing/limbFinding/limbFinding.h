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

#ifndef _IMAGE_LIMB_FIND_H_
#define _IMAGE_LIMB_FIND_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "messaging2/messaging2.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"

#include "msgPayloadDefC/CameraImageMsgPayload.h"
#include "msgPayloadDefC/LimbOpNavMsgPayload.h"

#include "_GeneralModuleFiles/sys_model.h"
#include "utilities/avsEigenMRP.h"
#include "utilities/bskLogging.h"



/*! @brief visual limb finding module */
class LimbFinding: public SysModel {
public:
    LimbFinding();
    ~LimbFinding();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    
public:
    std::string filename;                //!< Filename for module to read an image directly
    Message<LimbOpNavMsgPayload> opnavLimbOutMsg;  //!< The name of the Limb output message
    ReadFunctor<CameraImageMsgPayload> imageInMsg;          //!< The name of the ImageFswMsg output message
    std::string saveDir;                //!< Directory to save images to

    uint64_t sensorTimeTag;              //!< [ns] Current time tag for sensor out
    /* OpenCV specific arguments needed for Limb finding*/
    int32_t blurrSize;                   //!< [px] Size of the blurring box in pixels
    int32_t cannyThreshHigh;                 //!< [px] Canny edge detection Threshold
    int32_t cannyThreshLow;                  //!< [-] Second Threshold for Canny detection
    int32_t saveImages;                  //!< [-] 1 to save images to file for debugging
    int32_t limbNumThresh;                  //!< [-] Threshold for when a limb is detected
    
    BSKLogger bskLogger;                //!< -- BSK Logging

};


#endif

