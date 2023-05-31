/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef KEYPOINTSMSG_H
#define KEYPOINTSMSG_H

#include "fswAlgorithms/imageProcessing/opticalFlow/opticalFlowDefinitions.h"

//!@brief Optical Navigation measurement message containing the matched key points between two images
/*! This message is output by the optical flow module and contains the key points shared between the two image
 * that were processed, as well as the attitudes, validity, time tags, the camera ID, and the number of points detected.
 */
typedef struct
//@cond DOXYGEN_IGNORE
PairedKeyPointsMsgPayload
//@endcond
{
    bool valid; //!< --  Quality of measurement
    int64_t cameraID; //!< -- [-]   ID of the camera that took the images
    uint64_t timeTag_firstImage; //!< --[ns]   Current vehicle time-tag associated with first image
    uint64_t timeTag_secondImage; //!< --[ns]   Current vehicle time-tag associated with second image
    double keyPoints_firstImage[2*MAX_KEY_POINTS]; //!< -- [-]   Paired features in first image
    double keyPoints_secondImage[2*MAX_KEY_POINTS]; //!< -- [-]   Paired features in second image
    double sigma_BN_firstImage[3]; //!< -- [-]   Spacecraft body frame attitude associated with first image
    double sigma_BN_secondImage[3]; //!< -- [-]  Spacecraft body frame attitude associated with second image
    double sigma_TN_firstImage[3]; //!< -- [-]   Target frame wrt inertial at time of first image
    double sigma_TN_secondImage[3]; //!< -- [-]  Target frame wrt inertial at time of second image
    int keyPointsFound; //!< -- [-] Number of paired features
}PairedKeyPointsMsgPayload;

#endif /* KEYPOINTSMSG_H */
