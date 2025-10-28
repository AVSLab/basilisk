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

#ifndef antennaStateMsg_H
#define antennaStateMsg_H

#include <stdint.h>

/*! Antenna state enumeration - defined here for message independence */
//typedef enum {
//    ANTENNA_OFF  = 0,
//    ANTENNA_RX   = 1,
//    ANTENNA_TX   = 2,
//    ANTENNA_RXTX = 3
//} AntennaStateEnum;

/*! antenna state message definition */
typedef struct {
    uint32_t antennaState;  //!< Current antenna state TODO shouldnt that be an enum?
}AntennaStateMsgPayload;
#endif /* antennaStateMsg_H */
