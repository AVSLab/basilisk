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

#ifndef ID_EPHEMERIS_OUTPUT_MESSAGE_H
#define ID_EPHEMERIS_OUTPUT_MESSAGE_H

#include "simFswInterfaceMessages/ephemerisIntMsg.h"

/*! @brief Message to store the converted Spice ephemeris data */
typedef struct{
    int64_t inputID;                        //!< [-] Message ID associated with ephemeris output
    uint64_t clockTime;                     //!< [-] Clock time associated with msg write
    SpicePlanetStateSimMsg messageData;     //!< [-] Data container for message data
    EphemerisIntMsg outputData;             //!< [-] Data container for output ephemeris estimate
} IDEphemerisSimMsg;




#endif
