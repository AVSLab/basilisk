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

/*! @brief Structure used to define the output definition for attitude guidance*/
typedef struct {
    int64_t cameraID;          //!< [-]   ID of the camera that took the snapshot*/
    double fieldOfView;        //!< [deg]   Camera Field of View in degrees*/
    int resolution[2];      //!< [-] Camera resolution, width/height in pixels (pixelWidth/pixelHeight in Unity) in pixels*/
    uint64_t renderRate;       //!< [ns] Rate at which to capture images in nano sec */
    double focalLength;        //!< [m]   Camera Focal Length in meters*/
    double sensorSize[2];      //!< [mm]   Size of the camera sensor-paired with resolution gives you pixel size in mm*/
    double cameraPos_B[3];     //!< [m] Camera position in body frame */
    double sigma_BC[3];     //!< [m] Rotation to the camera frame MRP*/
}CameraConfigMsg;

