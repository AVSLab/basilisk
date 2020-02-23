/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef CAMERA_MSG_H
#define CAMERA_MSG_H

#define MAX_STRING_LENGTH 256
/*! @brief Structure used to define the camera parameters*/

#include "simFswInterfaceMessages/macroDefinitions.h"

typedef struct {
    int64_t cameraID;          //!< [-]   ID of the camera that took the snapshot*/
    int isOn; //!<  The camera is taking images at rendering rate if 1, 0 if not*/
    char parentName[MAX_STRING_LENGTH];  // Name of the parent body to which the camera should be attached
    double fieldOfView;        //!< [rad]   Camera Field of View */
    int resolution[2];         //!< [-] Camera resolution, width/height in pixels (pixelWidth/pixelHeight in Unity) in pixels*/
    uint64_t renderRate;       //!< [ns] Frame time interval at which to capture images in units of nanosecond */
    double focalLength;        //!< [m] Camera Focal Length in meters*/
    double sensorSize[2];      //!< [m] Size of the camera sensor */
    double cameraPos_B[3];     //!< [m] Camera position in body frame */
    double sigma_CB[3];        //!< [-] MRP defining the orientation of the camera frame relative to the body frame */
    char skyBox[MAX_STRING_LENGTH]; //!< string containing the star field preference
}CameraConfigMsg;

/*! @} */

#endif
