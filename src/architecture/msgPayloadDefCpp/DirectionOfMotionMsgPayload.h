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

#ifndef DIRECTION_MOTION_MSG_H
#define DIRECTION_MOTION_MSG_H

//!@brief Optical Navigation measurement message containing the matched key points between two images
/*! This message is output by the optical flow module and contains the key points shared between the two image
 * that were processed, as well as the attitudes, validity, time tags, the camera ID, and the number of points detected.
 */
typedef struct
//@cond DOXYGEN_IGNORE
DirectionOfMotionMsgPayload
//@endcond
{
    bool valid; //!< --  Quality of measurement
    uint64_t cameraID; //!< --  Camera who's motion is in question
    uint64_t timeOfDirectionEstimate; //!< --  time of the direction of motion computation
    double v_C_hat[3]; //!< -- [-]   Camera direction of motion
    double covar_C[3*3]; //!< -- [-]   Unvertainty in direction of motion
}DirectionOfMotionMsgPayload;

#endif /* DIRECTION_MOTION_MSG_H */
