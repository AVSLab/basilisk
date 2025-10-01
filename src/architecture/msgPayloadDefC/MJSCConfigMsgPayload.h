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

#ifndef MJ_SC_CONFIG_MESSAGE_H
#define MJ_SC_CONFIG_MESSAGE_H

#include "architecture/utilities/macroDefinitions.h"

/*! @brief This structure is used in the messaging system to pass the spacecraft configuration information. */
typedef struct {
    double dcm_HB[3][6];          //!< [-] set of DCM's from body to hinge frames
    double thr_F_T[3][2];        //!< [-] set of thruster force directions in thruster frames
    double r_S2S1_S1[3][10];       //!< [-] set of position vectors from S1 to S2 in S1 frame
}MJSCConfigMsgPayload;


#endif
