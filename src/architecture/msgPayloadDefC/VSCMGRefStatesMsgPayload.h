/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#ifndef VSCMG_REF_STATES_MESSAGE_STRUCT_H
#define VSCMG_REF_STATES_MESSAGE_STRUCT_H

#include "architecture/utilities/macroDefinitions.h"

/*! @brief Structure used to define the desired VSCMG RW accelerations and gimbal rates */
typedef struct {
    double wheelAccels[MAX_EFF_CNT]; //!< [rad/s^2] The desired wheel accelerations for the VSCMGs
    double gimbalRates[MAX_EFF_CNT]; //!< [rad/s] The desired gimbal rates for the VSCMGs
} VSCMGRefStatesMsgPayload;

#endif
