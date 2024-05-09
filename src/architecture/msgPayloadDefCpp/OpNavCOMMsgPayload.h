/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef COMOPNAVMSG_H
#define COMOPNAVMSG_H

//!@brief Center of mass optical navigation measurement message
/*! This message is output by the center of brightness converter module and contains the center of mass (COM) after
 * offsetting the center of brightness (COB) using the phase angle correction.
 */
typedef struct
//@cond DOXYGEN_IGNORE
OpNavCOMMsgPayload
//@endcond
{
    uint64_t timeTag; //!< --[ns] Current vehicle time-tag associated with measurements
    bool valid; //!< -- Quality of measurement
    int64_t cameraID; //!< -- [-] ID of the camera that took the image
    double centerOfMass[2]; //!< -- [-] Center x, y of bright pixels
    double offsetFactor; //!< -- [-] COM/COB offset factor as a fraction of object radius
    int32_t objectPixelRadius; //!< -- [-] radius of object in pixels
    double phaseAngle; //!< -- [rad] angle between Sun-Object-Camera
    double sunDirection; //!< -- [rad] Sun direction in the image
}OpNavCOMMsgPayload;

#endif /* COMOPNAVMSG_H */
