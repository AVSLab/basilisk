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

#ifndef SICP_H
#define SICP_H

#include <array>
#include "fswAlgorithms/pointCloudProcessing/SICP/sicpDefinitions.h"

//!@brief Message containing the output of the Scaling Iterative Closest Point algorithm
/*! This message is output by the SICP module and contains the scaling matrix, rotation matrix, and translation vector
 * output after two point clouds are registered.
 */
typedef struct
//@cond DOXYGEN_IGNORE
SICPMsgPayload
//@endcond
{
    bool valid; //!< --[-] Message was purposefully populated
    uint64_t numberOfIteration; //!< --[-] Number of iterations used for SICP convergence
    uint64_t timeTag; //!< --[-] Time at which these iterations were computed
    double scaleFactor[MAX_ITERATIONS]; //!< -- [-] Array of scale factors
    double rotationMatrix[POINT_DIM*POINT_DIM*MAX_ITERATIONS]; //!< -- [-]  Array of rotation matrices
    double translation[POINT_DIM*MAX_ITERATIONS]; //!< -- [-] Array of translation vectors
}SICPMsgPayload;

#endif /* SICP_H */
