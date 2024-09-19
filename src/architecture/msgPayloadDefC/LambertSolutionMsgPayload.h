/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef LAMBERT_SOLUTION_MESSAGE_H
#define LAMBERT_SOLUTION_MESSAGE_H


/*! @brief Structure used to define the output of the Lambert problem solution */
typedef struct {
    double v1_N[3];            //!< [m/s] velocity solution at t1
    double v2_N[3];            //!< [m/s] velocity solution at t2
    int valid;                 //!< [-] valid solution if 1, not if 0
    double v1Sol2_N[3];        //!< [m/s] second velocity solution at t1 (for multi-revolution solutions)
    double v2Sol2_N[3];        //!< [m/s] second velocity solution at t2 (for multi-revolution solutions)
    int validSol2;             //!< [-] valid second solution if 1, not if 0
}LambertSolutionMsgPayload;

#endif
