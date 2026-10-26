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

/*! @brief Supported Lambert solver algorithms. */
typedef enum
{
    IZZO = 0,   //!< Izzo algorithm; the default for zero-initialized messages.
    GOODING = 1 //!< Gooding algorithm.
} SolverMethod;

/*! @brief Transfer direction relative to the positive z-axis of the input frame. */
typedef enum
{
    PROGRADE = 0,  //!< Nonnegative z-component of orbital angular momentum; the default.
    RETROGRADE = 1 //!< Nonpositive z-component of orbital angular momentum.
} LambertTransferDirection;

/*! @brief Structure used to define the input for Lambert problem */
typedef struct
{
    int solverMethod;      //!< [-] IZZO (zero/default) or GOODING; raw integer validated by the solver
    double r1_N[3];        //!< [m] position vector at t0
    double r2_N[3];        //!< [m] position vector at t1
    double transferTime;   //!< [s] time of flight between r1_N and r2_N (t1-t0)
    double mu;             //!< [m^3 s^-2] gravitational parameter of body
    int numRevolutions;    //!< [-] number of revolutions to be completed (completed orbits)
    int transferDirection; //!< [-] PROGRADE (zero/default) or RETROGRADE; raw integer validated by the solver
} LambertProblemMsgPayload;

#endif
