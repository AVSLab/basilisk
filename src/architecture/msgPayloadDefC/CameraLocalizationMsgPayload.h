/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef CAMERA_LOCALIZATION_MSG_H
#define CAMERA_LOCALIZATION_MSG_H

/*! @brief This structure includes the estimated camera location obtained from triangulation given a point cloud*/
typedef struct {
    bool valid; //!< [-] Quality of measurement
    int64_t cameraID; //!< [-] ID of the camera
    uint64_t timeTag; //!< [ns] vehicle time-tag when images where taken
    double sigma_BN[3]; //!< [-] MRP orientation of spacecraft when images where taken
    double cameraPos_N[3]; //!< [m] Camera location in inertial frame */
    double covariance_N[3*3]; //!< [m^2] covariance of estimated camera location in inertial frame
}CameraLocalizationMsgPayload;

#endif
