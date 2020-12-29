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

#ifndef _IMAGE_PROC_CNN_H_
#define _IMAGE_PROC_CNN_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"

#include "architecture/msgPayloadDefC/CameraImageMsgPayload.h"
#include "architecture/msgPayloadDefC/CirclesOpNavMsgPayload.h"
#include "architecture/messaging2/messaging2.h"


#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

/*! @brief The CNN based center radius visual tracking module. */
class CenterRadiusCNN: public SysModel {
public:
    CenterRadiusCNN();
    ~CenterRadiusCNN();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    
public:
    std::string filename;                //!< Filename for module to read an image directly
    Message<CirclesOpNavMsgPayload> opnavCirclesOutMsg;  //!< The name of the CirclesOpnavMsg output message
    
    ReadFunctor<CameraImageMsgPayload> imageInMsg;          //!< The name of the ImageFswMsg output message
    
    std::string pathToNetwork;                  //!< Path to the trained CNN
    uint64_t sensorTimeTag;              //!< [ns] Current time tag for sensor out
    /* OpenCV specific arguments needed for HoughCircle finding*/
    int32_t saveImages;                  //!< [-] 1 to save images to file for debugging
    double pixelNoise[3];                 //!< [-] Pixel Noise for the estimate

private:
    cv::dnn::Net positionNet2;           //!< Network for evaluation of centers
};


#endif

