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

#ifndef OPNAV_MESSAGE_H
#define OPNAV_MESSAGE_H

#define ODUKF_N_STATES 6
#define ODUKF_N_STATES_HALF 3
#define ODUKF_N_MEAS 3

/*! @brief structure for filter-states output for the unscented kalman filter
 implementation of the sunline state estimator*/
typedef struct {
    double timeTag;                             /*!< [s] Current time of validity for output */
    double covar_N[3*3];    /*!< [-] Current covariance of the filter */
    double covar_B[3*3];    /*!< [-] Current covariance of the filter */
    double r_N[3];                 /*!< [km] Current estimated state of the filter */
    double r_B[3];                 /*!< [km] Current estimated state of the filter */
}OpnavFswMsg;




#endif
