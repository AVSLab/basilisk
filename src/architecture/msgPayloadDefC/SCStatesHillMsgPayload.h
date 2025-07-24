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

#ifndef SC_STATE_HILL_MESSAGE_H
#define SC_STATE_HILL_MESSAGE_H


/*! @brief This structure is used in the messaging system to communicate what the
 state of the vehicle is currently in a hill frame.*/
typedef struct {
    double r_BH_H[3];                 //!< m  Current position vector (hill frame)
    double v_BH_H[3];                 //!< m/s Current velocity vector (hill frame)
    double sigma_BH[3];               //!< -- Current MRPs (hill frame)
    double omega_BH_B[3];             //!< r/s Current angular velocity (body frame)S
    double sigma_HN[3];               //!< -- Current MRP of the target hill frame compared to inertial (inertial)
    double n;          //!< r/s Angular rate of the Hillframe compared to inertial
}SCStatesHillMsgPayload;



#endif
