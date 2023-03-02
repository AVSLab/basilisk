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

#ifndef _IMAGE_PROC_COB_H_
#define _IMAGE_PROC_COB_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "architecture/messaging/messaging.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/dnn.hpp"

#include "architecture/msgPayloadDefC/CameraImageMsgPayload.h"
#include "architecture/msgPayloadDefCpp/OpNavCOBMsgPayload.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"


/*! @brief visual object tracking using center of brightness detection */
class CenterOfBrightness: public SysModel {
public:
    CenterOfBrightness();
    ~CenterOfBrightness();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);
    
public:
    std::string filename;                //!< Filename for module to read an image directly
    Message<OpNavCOBMsgPayload> opnavCOBOutMsg;  //!< The name of the OpNav center of brightness output message
    ReadFunctor<CameraImageMsgPayload> imageInMsg;          //!< The name of the camera output message
    std::string saveDir;                //!< The name of the directory to save images
    uint64_t sensorTimeTag;              //!< [ns] Current time tag for sensor out
    /* OpenCV specific arguments needed for finding all non zero pixels*/
    int32_t blurSize;                   //!< [px] Size of the bluring box in pixels
    int32_t threshold;                 //!< [px] Theshold value on whether or not to include the solution
    int32_t saveImages;                  //!< [-] 1 to save images to file for debugging
    BSKLogger bskLogger;                //!< -- BSK Logging
};


#endif

