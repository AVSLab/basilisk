/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef DRAG_GEOMETRY_MSG_PAYLOAD_H
#define DRAG_GEOMETRY_MSG_PAYLOAD_H


//! @brief Container for basic drag parameters.
typedef struct {
    double projectedArea; //!< [m^2] Area of spacecraft projected in velocity direction
    double dragCoeff;     //!< [-]   Nondimensional drag coefficient
    double r_CP_S[3];     //!< [m]   Position of center of pressure relative to the center of frame S, given in frame S [m]
}DragGeometryMsgPayload;

#endif
