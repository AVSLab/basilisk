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

#ifndef _IMAGE_PROC_HOUGH_H_
#define _IMAGE_PROC_HOUGH_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "architecture/messaging/system_messaging.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "../simulation/simFswInterfaceMessages/cameraImageMsg.h"
#include "../simulation/simFswInterfaceMessages/circlesOpNavMsg.h"
#include "../simulation/_GeneralModuleFiles/sys_model.h"
#include "../simulation/utilities/avsEigenMRP.h"
#include "../../fswUtilities/CImg.h"



/*! \addtogroup SimModelGroup
 * @{
 */

//!@brief Sensor model to simulate a Star Tracker.
/*!
 The module
 [PDF Description](Basilisk-star_tracker-20161101.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 The corruption types are outlined in this
 [PDF document](BasiliskCorruptions.pdf).
 */
class HoughCircles: public SysModel {
public:
    HoughCircles();
    ~HoughCircles();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    
public:
    
    std::string opnavCirclesOutMsgName;  //! The name of the CirclesOpnavMsg output message*/
    std::string imageInMsgName;          //! The name of the ImageFswMsg output message*/
    uint64_t sensorTimeTag;              //! [ns] Current time tag for sensor out
    /*! OpenCV specific arguments need for HoughCircle finding*/
    int32_t blurrSize;
    int32_t cannyThresh1;
    int32_t cannyThresh2;
    int32_t houghMinDist;
    int32_t houghMinRadius;
    int32_t houghMaxRadius;
    
private:
    uint64_t OutputBufferCount;       //!< [-] Count on the number of output message buffers
    int32_t opnavCirclesOutMsgID;                      //! ID for the outgoing message */
    int32_t imageInMsgID;                           //! ID for the outgoing message */
};

/* @} */

#endif

