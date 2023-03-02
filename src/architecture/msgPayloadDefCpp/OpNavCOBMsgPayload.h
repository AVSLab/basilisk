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

#ifndef COBOPNAVMSG_H
#define COBOPNAVMSG_H

//!@brief Center of brightness optical navigation measurement message
/*! This message is output by the center of brightness module and contains the center of brightness of the image
 * that was input, as well as the validity of the image processing process, the camera ID, and the number of pixels
 * found during the computation.
 */
typedef struct
//@cond DOXYGEN_IGNORE
OpNavCOBMsgPayload
//@endcond
{
    uint64_t timeTag; //!< --[ns]   Current vehicle time-tag associated with measurements
    bool valid; //!< --  Quality of measurement
    int64_t cameraID; //!< -- [-]   ID of the camera that took the image
    double centerOfBrightness[2]; //!< -- [-]   Center x, y of bright pixels
    int32_t pixelsFound; //!< -- [-] Number of bright pixels found in the image
}OpNavCOBMsgPayload;

#endif /* COBOPNAVMSG_H */
