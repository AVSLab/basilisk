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

#ifndef POINTCLOUDMSG_H
#define POINTCLOUDMSG_H

#include <array>
#include "fswAlgorithms/pointCloudProcessing/SICP/sicpDefinitions.h"

//!@brief N-D point cloud
/*! This message contains a point cloud and corresponding time tag
 */
typedef struct
//@cond DOXYGEN_IGNORE
PointCloudMsgPayload
//@endcond
{
    uint64_t timeTag; //!< --[ns]   Current vehicle time-tag associated with cloud
    bool valid; //!< --  Quality of measurement
    int numberOfPoints; //!< -- [-] Number of points detected
    double points[MAX_POINTS*POINT_DIM]; //!< -- [-]  Point cloud array
}PointCloudMsgPayload;

#endif /* POINTCLOUDMSG_H */
