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

#ifndef LAMBERT_PROBLEM_MESSAGE_H
#define LAMBERT_PROBLEM_MESSAGE_H

typedef enum {
    IZZO,
    GOODING
} SolverMethod;

/*! @brief Structure used to define the input for Lambert problem */
typedef struct {
    SolverMethod solverMethod;          //!< [-] lambert solver algorithm (GOODING or IZZO)
    double r1vec[3];                    //!< [m] position vector at t0
    double r2vec[3];                    //!< [m] position vector at t1
    double transferTime;                //!< [s] time of flight between r1vec and r2vec (t1-t0)
    double mu;                          //!< [m^3 s^-2] gravitational parameter of body
    int numRevolutions;                 //!< [-] number of revolutions to be completed (completed orbits)
}LambertProblemMsgPayload;

#endif
