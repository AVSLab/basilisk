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

#ifndef LAMBERT_PERFORMANCE_MESSAGE_H
#define LAMBERT_PERFORMANCE_MESSAGE_H


/*! @brief Structure used to define extra output of the Lambert problem solution */
typedef struct {
    double x;            //!< [-] solution for free variable (iteration variable)
    int numIter;         //!< [-] number of root-finder iterations to find x
    double errX;         //!< [-] difference in x between last and second-to-last iteration
    double xSol2;        //!< [-] second solution for free variable (iteration variable)
    int numIterSol2;     //!< [-] number of root-finder iterations to find x_sol2
    double errXSol2;     //!< [-] difference in x_sol2 between last and second-to-last iteration
}LambertPerformanceMsgPayload;

#endif
